//! Run the mapper over a recorded session and draw what it built.
//!
//! The sessions are `microduck_vslam`'s recorder output: `state.jsonl` holds `robot.state`
//! notifications and `tof.jsonl` holds `tof.frame` ones, each `{"arrival_ns", "params"}` with the
//! robot's own `t_ns` inside — the same two streams the service subscribes to, so a replay makes
//! exactly the decisions the robot would have.
//!
//! ```text
//! cargo run -p maploc --release --example replay -- <session-dir> [--out map.ppm] [--latency-ms N] [--axial] [--quiet]
//! ```

use std::io::{BufRead, BufReader, Write};
use std::path::PathBuf;

use duck_ipc_proto::{RobotState, TofFrame};
use maploc::grid::Cell;
use maploc::input::{DepthFrame, StateSample};
use maploc::mapper::{Event, Mapper, MapperConfig};
use serde::Deserialize;

#[derive(Deserialize)]
struct Line<T> {
    params: T,
}

enum Ev {
    State(Box<StateSample>),
    Depth(DepthFrame),
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let dir = PathBuf::from(args.next().ok_or(
        "usage: replay <session-dir> [--out map.ppm] [--latency-ms N] [--axial] [--quiet]",
    )?);
    let mut out = PathBuf::from("map.ppm");
    let mut cfg = MapperConfig::default();
    let mut quiet = false;
    while let Some(a) = args.next() {
        match a.as_str() {
            "--out" => out = args.next().ok_or("--out needs a path")?.into(),
            "--latency-ms" => {
                cfg.tof_latency_ms = args.next().ok_or("--latency-ms needs a number")?.parse()?
            }
            "--axial" => cfg.project.range_is_axial = true,
            "--quiet" => quiet = true,
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
    ) = (0, 0, 0, 0, 0, 0, 0, 0);
    for (_, e) in events {
        match e {
            Ev::State(s) => m.on_state(*s),
            Ev::Depth(f) => m.on_depth(f),
        }
        for ev in m.take_events() {
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
                Event::Searched { .. } => {}
            }
            if !quiet {
                println!("{ev:?}");
            }
        }
    }
    let elapsed = started.elapsed();
    println!(
        "keyframes {kfs} (matched {matched}, fully pinned {full}), discarded stops {discarded}, loops {loops}, rejected edges {rejected}, lost {lost}, relocalized {reloc}"
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

    let Some(g) = m.grid(0.025) else {
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
