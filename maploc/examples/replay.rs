//! Run the mapper over a recorded session and draw what it built.
//!
//! The sessions are `microduck_vslam`'s recorder output: `state.jsonl` holds `robot.state`
//! notifications and `tof.jsonl` holds `tof.frame` ones, each `{"arrival_ns", "params"}` with the
//! robot's own `t_ns` inside — the same two streams the service subscribes to, so a replay makes
//! exactly the decisions the robot would have.
//!
//! ```text
//! cargo run -p maploc --release --example replay -- <session-dir> [--out map.ppm] [--latency-ms N] [--axial] [--quiet]
//!     [--export run.json] [--res 0.025]
//! ```
//!
//! `--export` writes what the mapper decided, for scoring against ground truth elsewhere: every
//! stop with its odometry pose, the pose it was placed at live and its pose in the final optimized
//! map; the tracked pose over time (≤ 10 Hz) and why it was lost when it was; every event; and the
//! grid at `--res`. Poses are `[x, y, yaw]` — odometry ones in the odometry frame, the rest in the
//! map's — and the grid's `rows[0]` is its lowest y, one character a cell: `#` occupied, `.` seen
//! clear, `?` unknown.

use std::io::{BufRead, BufReader, Write};
use std::path::PathBuf;

use duck_ipc_proto::{RobotState, TofFrame};
use maploc::grid::Cell;
use maploc::input::{DepthFrame, StateSample};
use maploc::mapper::{Event, LostCause, Mapper, MapperConfig};
use maploc::se2::Pose2;
use serde::{Deserialize, Serialize};

#[derive(Deserialize)]
struct Line<T> {
    params: T,
}

enum Ev {
    State(Box<StateSample>),
    Depth(DepthFrame),
}

#[derive(Serialize)]
struct Export {
    version: u32,
    keyframes: Vec<KeyframeOut>,
    poses: Vec<PoseOut>,
    events: Vec<EventOut>,
    grid: Option<GridOut>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    grids: Vec<TimedGrid>,
}

#[derive(Serialize)]
struct KeyframeOut {
    id: u32,
    t_ns: u64,
    odom: [f64; 3],
    live: Option<[f64; 3]>,
    #[serde(rename = "final")]
    final_: Option<[f64; 3]>,
    in_map: bool,
}

#[derive(Serialize)]
struct PoseOut {
    t_ns: u64,
    pose: Option<[f64; 3]>,
    lost: Option<&'static str>,
}

#[derive(Serialize)]
struct EventOut {
    t_ns: u64,
    kind: &'static str,
    text: String,
}

#[derive(Serialize)]
struct GridOut {
    res: f64,
    origin: [f64; 2],
    w: usize,
    h: usize,
    rows: Vec<String>,
}

/// The map as it stood after an event that changed it, for drawing it being built.
#[derive(Serialize)]
struct TimedGrid {
    t_ns: u64,
    #[serde(flatten)]
    grid: GridOut,
}

fn grid_out(g: &maploc::grid::OccupancyGrid) -> GridOut {
    GridOut {
        res: g.res,
        origin: g.origin,
        w: g.w,
        h: g.h,
        rows: g
            .cells
            .chunks(g.w)
            .map(|row| {
                row.iter()
                    .map(|c| match c {
                        Cell::Occupied => '#',
                        Cell::Free => '.',
                        Cell::Unknown => '?',
                    })
                    .collect()
            })
            .collect(),
    }
}

fn xyt(p: Pose2) -> [f64; 3] {
    [p.x, p.y, p.yaw]
}

/// The words `mapd` puts in `map.status`.
fn lost_name(c: LostCause) -> &'static str {
    match c {
        LostCause::Boot => "boot",
        LostCause::Carried => "carried",
        LostCause::Bumped => "bumped",
        LostCause::Sat => "sat",
        LostCause::Fell => "fell",
        LostCause::Contradiction => "contradiction",
    }
}

fn kind(e: &Event) -> &'static str {
    match e {
        Event::Keyframe { .. } => "keyframe",
        Event::Discarded { .. } => "discarded",
        Event::Covered { .. } => "covered",
        Event::MapRestarted { .. } => "map_restarted",
        Event::LoopClosed { .. } => "loop_closed",
        Event::Lost { .. } => "lost",
        Event::Bumped { .. } => "bumped",
        Event::Relocalized { .. } => "relocalized",
        Event::EdgeRejected { .. } => "edge_rejected",
        Event::Searched { .. } => "searched",
    }
}

/// Decimation of the exported pose track: 10 Hz.
const POSE_EVERY_NS: u64 = 100_000_000;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let dir = PathBuf::from(args.next().ok_or(
        "usage: replay <session-dir> [--out map.ppm] [--latency-ms N] [--axial] [--quiet] [--export run.json] [--res 0.025]",
    )?);
    let mut out = PathBuf::from("map.ppm");
    let mut export_to: Option<PathBuf> = None;
    let mut grids = false;
    let mut res = 0.025;
    let mut cfg = MapperConfig::default();
    let mut quiet = false;
    while let Some(a) = args.next() {
        match a.as_str() {
            "--out" => out = args.next().ok_or("--out needs a path")?.into(),
            "--latency-ms" => {
                cfg.tof_latency_ms = args.next().ok_or("--latency-ms needs a number")?.parse()?
            }
            "--axial" => cfg.project.range_is_axial = true,
            "--tof-pitch-deg" => {
                cfg.project.pitch_offset_rad = args
                    .next()
                    .ok_or("--tof-pitch-deg needs a number")?
                    .parse::<f64>()?
                    .to_radians()
            }
            "--max-range" => {
                cfg.project.max_range_m =
                    args.next().ok_or("--max-range needs a number")?.parse()?
            }
            "--quiet" => quiet = true,
            // Any MapperConfig field by its path, e.g. `--set min_hit_frac=0.6` or
            // `--set project.range_is_axial=true`: the bench's knob for trying a setting
            // without a rebuild. The value is read as JSON, or as a string if it is not.
            "--set" => {
                let kv = args.next().ok_or("--set needs key=value")?;
                let (k, v) = kv.split_once('=').ok_or("--set needs key=value")?;
                let mut root = serde_json::to_value(&cfg)?;
                let mut at = &mut root;
                for part in k.split('.') {
                    at = at.get_mut(part).ok_or_else(|| format!("no setting {k}"))?;
                }
                *at = serde_json::from_str(v).unwrap_or(serde_json::Value::String(v.to_owned()));
                cfg = serde_json::from_value(root)?;
            }
            "--export" => export_to = Some(args.next().ok_or("--export needs a path")?.into()),
            "--res" => res = args.next().ok_or("--res needs a number")?.parse()?,
            // Also export the grid after every event that changed it: stops, relocalizations,
            // loop closures — the map being built, for a video.
            "--grids" => grids = true,
            other => return Err(format!("unknown argument {other}").into()),
        }
    }

    let mut events: Vec<(u64, Ev)> = Vec::new();
    for line in BufReader::new(std::fs::File::open(dir.join("state.jsonl"))?).lines() {
        let l: Line<RobotState> = serde_json::from_str(&line?)?;
        if let Some(s) = StateSample::from_proto(&l.params) {
            events.push((s.t_ns, Ev::State(Box::new(s))));
        }
    }
    let n_state = events.len();
    for line in BufReader::new(std::fs::File::open(dir.join("tof.jsonl"))?).lines() {
        let l: Line<TofFrame> = serde_json::from_str(&line?)?;
        if let Some(f) = DepthFrame::from_proto(&l.params) {
            events.push((f.t_ns, Ev::Depth(f)));
        }
    }
    // Both streams interleaved on the robot's clock, the order the service would see them in
    // were transport free.
    events.sort_by_key(|(t, _)| *t);
    println!(
        "{} state samples, {} depth frames",
        n_state,
        events.len() - n_state
    );

    let started = std::time::Instant::now();
    let mut m = Mapper::new(cfg);
    let (
        mut kfs,
        mut matched,
        mut full,
        mut loops,
        mut rejected,
        mut lost,
        mut reloc,
        mut discarded,
        mut covered,
    ) = (0, 0, 0, 0, 0, 0, 0, 0, 0);
    let mut export = Export {
        version: 1,
        keyframes: Vec::new(),
        poses: Vec::new(),
        events: Vec::new(),
        grid: None,
        grids: Vec::new(),
    };
    let mut t_state = 0u64;
    let mut next_pose_ns = 0u64;
    for (_, e) in events {
        match e {
            Ev::State(s) => {
                t_state = s.t_ns;
                m.on_state(*s);
                if t_state >= next_pose_ns {
                    next_pose_ns = t_state + POSE_EVERY_NS;
                    export.poses.push(PoseOut {
                        t_ns: t_state,
                        pose: m.pose().map(xyt),
                        lost: m.lost().map(lost_name),
                    });
                }
            }
            Ev::Depth(f) => m.on_depth(f),
        }
        let evs = m.take_events();
        let changed = evs.iter().any(|e| {
            matches!(
                e,
                Event::Keyframe { .. } | Event::Relocalized { .. } | Event::LoopClosed { .. }
            )
        });
        if grids
            && changed
            && let Some(g) = m.grid(res)
        {
            export.grids.push(TimedGrid {
                t_ns: t_state,
                grid: grid_out(&g),
            });
        }
        for ev in evs {
            export.events.push(EventOut {
                t_ns: t_state,
                kind: kind(&ev),
                text: format!("{ev:?}"),
            });
            if let Event::Keyframe { id, .. } = ev {
                export.keyframes.push(KeyframeOut {
                    id,
                    t_ns: t_state,
                    odom: xyt(m.keyframes()[id as usize].odom),
                    live: m.pose().map(xyt),
                    final_: None,
                    in_map: false,
                });
            }
            match &ev {
                Event::Keyframe { matched: mt, .. } => {
                    kfs += 1;
                    if let Some(ms) = mt.as_ref().filter(|m| m.accepted) {
                        matched += 1;
                        if ms.rank == 3 {
                            full += 1;
                        }
                    }
                }
                Event::LoopClosed { .. } => loops += 1,
                Event::EdgeRejected { .. } => rejected += 1,
                Event::Lost { .. } => lost += 1,
                Event::Relocalized { .. } => reloc += 1,
                Event::Discarded { .. } => discarded += 1,
                Event::Covered { .. } => covered += 1,
                Event::MapRestarted { .. } => {}
                Event::Searched { .. } | Event::Bumped { .. } => {}
            }
            if !quiet {
                println!("{ev:?}");
            }
        }
    }
    let elapsed = started.elapsed();
    println!(
        "keyframes {kfs} (matched {matched}, fully pinned {full}), covered stops {covered}, discarded stops {discarded}, loops {loops}, rejected edges {rejected}, lost {lost}, relocalized {reloc}"
    );
    println!("mapper time {:.2} s", elapsed.as_secs_f64());
    if let Some(p) = m.pose() {
        println!(
            "final pose {:.3} {:.3} {:.1}°",
            p.x,
            p.y,
            p.yaw.to_degrees()
        );
    }

    let finals: std::collections::HashMap<u32, Pose2> =
        m.map_keyframes().map(|(p, k)| (k.id, p)).collect();
    for k in &mut export.keyframes {
        k.final_ = finals.get(&k.id).copied().map(xyt);
        k.in_map = k.final_.is_some();
    }
    let grid = m.grid(res);
    if let Some(path) = &export_to {
        export.grid = grid.as_ref().map(grid_out);
        std::fs::write(path, serde_json::to_vec(&export)?)?;
        println!(
            "exported {} stops, {} poses, {} events to {}",
            export.keyframes.len(),
            export.poses.len(),
            export.events.len(),
            path.display()
        );
    }

    let Some(g) = grid else {
        println!("no map");
        return Ok(());
    };
    println!(
        "grid {}x{} at {} m: occupied {} free {} unknown {}",
        g.w,
        g.h,
        g.res,
        g.count(Cell::Occupied),
        g.count(Cell::Free),
        g.count(Cell::Unknown)
    );
    // A colour picture: the grid, the keyframes in red, odometry-only stops in orange.
    let mut px: Vec<[u8; 3]> = g
        .cells
        .iter()
        .map(|c| match c {
            Cell::Unknown => [150, 150, 150],
            Cell::Free => [255, 255, 255],
            Cell::Occupied => [0, 0, 0],
        })
        .collect();
    let mut dot = |x: f64, y: f64, rgb: [u8; 3]| {
        let i = ((x - g.origin[0]) / g.res) as isize;
        let j = ((y - g.origin[1]) / g.res) as isize;
        for dj in -1..=1 {
            for di in -1..=1 {
                let (a, b) = (i + di, j + dj);
                if a >= 0 && b >= 0 && (a as usize) < g.w && (b as usize) < g.h {
                    px[b as usize * g.w + a as usize] = rgb;
                }
            }
        }
    };
    for (pose, _) in m.map_keyframes() {
        dot(pose.x, pose.y, [220, 30, 30]);
    }
    let mut f = std::fs::File::create(&out)?;
    write!(f, "P6\n{} {}\n255\n", g.w, g.h)?;
    for j in (0..g.h).rev() {
        for i in 0..g.w {
            f.write_all(&px[j * g.w + i])?;
        }
    }
    println!("wrote {}", out.display());
    Ok(())
}
