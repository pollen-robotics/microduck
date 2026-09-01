//! Ground-truth SLAM bench: replay a robotd-recorded `.mdlg` session
//! (format v2 — see `maploc::record`) against a measured room, through the
//! SAME `Mapper` the robot ran, and score what happened.
//!
//!     cargo run -p maploc --release --example evaluate -- \
//!         <session.mdlg> <truth.toml> [out_dir]
//!
//! The experiment protocol the metrics assume:
//!
//!   1. boot the robot at the truth file's `start` pose (this anchors the
//!      map frame — place it carefully, every error below inherits any
//!      placement error);
//!   2. walk around, mapping;
//!   3. drive back to the start pose, stand still ~10 s;
//!   4. SIT the robot (the sit is the kidnap marker — odometry cannot see
//!      a carry, but it cannot miss a sit), carry it to the `kidnap`
//!      pose, place it, STAND it;
//!   5. move around again.
//!
//! The truth file (units: centimetres and degrees, the units a tape
//! measure speaks):
//!
//! ```toml
//! walls = [ [x1, y1, x2, y2], ... ]   # wall segments, room frame
//! start  = [x, y, yaw_deg]            # boot pose, room frame
//! kidnap = [x, y, yaw_deg]            # post-carry pose, room frame
//! ```
//!
//! What comes out: return-to-start tracking error vs raw odometry error,
//! kidnap detection and relocalization latency + pose error, map-vs-room
//! wall statistics, and two PGMs (the map, and the map with the truth
//! walls burned in) for eyeballing.

use std::path::PathBuf;

use kinematics::tof::{Posture, Reprojector};
use maploc::mapper::{Mapper, MapperConfig, MapperSample, Note};
use maploc::pipeline::{Slam, SlamConfig};
use maploc::pose_graph::between;
use maploc::replay::{Record, SessionReplayer};
use maploc::submap::{Pose2, Scan};

const N_ZONES: usize = kinematics::tof::ROWS * kinematics::tof::COLS;

#[derive(serde::Deserialize)]
struct TruthFile {
    /// Wall segments [x1, y1, x2, y2], centimetres, room frame.
    walls: Vec<[f32; 4]>,
    /// [x_cm, y_cm, yaw_deg] — where the robot booted.
    start: [f32; 3],
    /// [x_cm, y_cm, yaw_deg] — where the carry put it down.
    kidnap: [f32; 3],
}

struct Truth {
    /// Metres, room frame.
    walls: Vec<[f32; 4]>,
    start: Pose2,
    kidnap: Pose2,
}

impl Truth {
    fn load(path: &std::path::Path) -> Truth {
        let text = std::fs::read_to_string(path).expect("read truth file");
        let f: TruthFile = toml::from_str(&text).expect("parse truth file");
        let pose = |p: [f32; 3]| (p[0] / 100.0, p[1] / 100.0, p[2].to_radians());
        Truth {
            walls: f
                .walls
                .iter()
                .map(|w| [w[0] / 100.0, w[1] / 100.0, w[2] / 100.0, w[3] / 100.0])
                .collect(),
            start: pose(f.start),
            kidnap: pose(f.kidnap),
        }
    }

    /// A room-frame pose in the MAP frame (map origin = the boot pose).
    fn in_map(&self, room_pose: Pose2) -> Pose2 {
        between(self.start, room_pose)
    }

    /// Mean distance of a composite's endpoints (at `pose`, map frame) to
    /// the nearest truth wall, clamped at 0.5 m — the same flavour of
    /// number as the watchdog's, but judged by the tape measure instead of
    /// the built map. Separates "the scan/pose is wrong" from "the map
    /// painted this region wrong".
    fn scan_agreement(&self, pose: Pose2, scan: &Scan) -> f32 {
        let (sy, cy) = pose.2.sin_cos();
        let (mut sum, mut n) = (0.0_f32, 0u32);
        for (bx, by) in scan.endpoints_body() {
            let ex = pose.0 + cy * bx - sy * by;
            let ey = pose.1 + sy * bx + cy * by;
            sum += self.wall_distance((ex, ey)).min(0.5);
            n += 1;
        }
        if n > 0 { sum / n as f32 } else { 0.0 }
    }

    /// Distance from a map-frame point to the nearest truth wall.
    fn wall_distance(&self, map_xy: (f32, f32)) -> f32 {
        // Map point → room frame.
        let (sy, cy) = self.start.2.sin_cos();
        let rx = self.start.0 + cy * map_xy.0 - sy * map_xy.1;
        let ry = self.start.1 + sy * map_xy.0 + cy * map_xy.1;
        let mut best = f32::INFINITY;
        for w in &self.walls {
            let (ax, ay, bx, by) = (w[0], w[1], w[2], w[3]);
            let (dx, dy) = (bx - ax, by - ay);
            let len_sq = dx * dx + dy * dy;
            let t = if len_sq > 1e-9 {
                (((rx - ax) * dx + (ry - ay) * dy) / len_sq).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let (px, py) = (ax + t * dx, ay + t * dy);
            best = best.min((rx - px).hypot(ry - py));
        }
        best
    }
}

fn main() {
    let mut args = std::env::args().skip(1);
    let session: PathBuf = args
        .next()
        .expect("usage: evaluate <session.mdlg> <truth.toml> [out_dir]")
        .into();
    let truth_path: PathBuf = args
        .next()
        .expect("usage: evaluate <session.mdlg> <truth.toml> [out_dir]")
        .into();
    let out_dir: PathBuf = args.next().map(Into::into).unwrap_or_else(|| {
        session
            .parent()
            .unwrap_or(&PathBuf::from("."))
            .to_path_buf()
    });
    std::fs::create_dir_all(&out_dir).expect("create out_dir");

    let truth = Truth::load(&truth_path);
    let kidnap_map = truth.in_map(truth.kidnap);

    let rp = Reprojector::alpha();
    let mut mapper = Mapper::new(MapperConfig::default(), Slam::new(SlamConfig::default()));
    let mut notes: Vec<Note> = Vec::new();

    // Protocol timeline, reconstructed from the log.
    let mut sit_span: Option<(f32, f32)> = None; // the carry
    let mut sitting_since: Option<f32> = None;
    let mut tracked_at_sit: Option<Pose2> = None;
    let mut odom_at_sit: Option<Pose2> = None;
    let mut first_odom: Option<Pose2> = None;
    let mut latest_odom: Option<maploc::replay::OdomRecord> = None;

    // A clean snapshot of the map at the sit (pre-kidnap), for diagnosing
    // what the kidnapped windows look like against it.
    let mut sit_grid: Option<maploc::OccupancyGrid> = None;

    // Post-kidnap observations.
    let mut lost_at: Option<f32> = None;
    let mut relocalized: Option<(f32, Pose2, f32)> = None; // (t, pose, residual)
    let mut reloc_rejections = 0u32;
    let mut windows_inked_while_wrong = 0u32;

    let (mut n_odom, mut n_frames, mut n_windows_pre) = (0u64, 0u64, 0u32);
    let mut t_end = 0.0f32;

    for record in SessionReplayer::open(&session).expect("open session") {
        let record = record.expect("read record");
        let t = record.ts_us() as f32 / 1e6;
        t_end = t;
        match record {
            Record::Twin(_) => {
                eprintln!("this is a v1 prototype capture — use the `replay` example");
                std::process::exit(2);
            }
            Record::Odom(o) => {
                n_odom += 1;
                let odom = (o.odom_x, o.odom_y, o.odom_yaw);
                if first_odom.is_none() {
                    first_odom = Some(odom);
                }
                // The sit marker: the first sit longer than 2 s is the carry.
                if o.sitting {
                    if sitting_since.is_none() {
                        sitting_since = Some(t);
                        if sit_span.is_none() {
                            tracked_at_sit = Some(mapper.slam().tracked());
                            odom_at_sit = Some(odom);
                            sit_grid = mapper.slam().render();
                        }
                    }
                } else if let Some(since) = sitting_since.take()
                    && sit_span.is_none()
                    && t - since > 2.0
                {
                    sit_span = Some((since, t));
                    n_windows_pre = mapper.windows();
                }
                latest_odom = Some(o);
                mapper.observe(
                    t,
                    MapperSample {
                        odom,
                        moving: o.moving,
                        sitting: o.sitting,
                        fallen: o.fallen,
                    },
                    &mut notes,
                );
            }
            Record::Tof(frame) => {
                n_frames += 1;
                let Some(o) = latest_odom.as_ref() else {
                    continue;
                };
                let posture = Posture {
                    gravity: [
                        f64::from(o.gravity[0]),
                        f64::from(o.gravity[1]),
                        f64::from(o.gravity[2]),
                    ],
                    trunk_height_m: (o.trunk_z > 0.02).then_some(f64::from(o.trunk_z)),
                };
                let head = o.head.map(f64::from);
                let mut ranges = [None; N_ZONES];
                for (slot, (row, srow)) in ranges
                    .chunks_mut(8)
                    .zip(frame.ranges_m.iter().zip(frame.status.iter()))
                {
                    for ((s, &r), &st) in slot.iter_mut().zip(row.iter()).zip(srow.iter()) {
                        if (st == 5 || st == 9) && r.is_finite() && r > 0.0 {
                            *s = Some(f64::from(r));
                        }
                    }
                }
                let flat = rp.flatten(&ranges, head, &posture);
                if flat.angles_body.is_empty() {
                    continue;
                }
                let scan = Scan::from_polar(&flat.angles_body, &flat.ranges, flat.sensor_xy, 1e-3);
                mapper.frame(t, scan);
            }
        }
        for note in notes.drain(..) {
            let after_kidnap = sit_span.is_some();
            match note {
                Note::WindowIntegrated {
                    beams,
                    mean_residual_m,
                    n_observed,
                    n_beams,
                    ..
                } => {
                    if let (true, Some(g), Some((p, sc))) =
                        (after_kidnap, sit_grid.as_ref(), mapper.last_window())
                    {
                        // Breakdown vs the CLEAN pre-kidnap map.
                        let cfg = *g.cfg();
                        let (w, h) = (g.width(), g.height());
                        let (sy, cy) = p.2.sin_cos();
                        let (mut on_wall, mut in_free, mut unknown, mut out) =
                            (0u32, 0u32, 0u32, 0u32);
                        for (bx, by) in sc.endpoints_body() {
                            let ex = p.0 + cy * bx - sy * by;
                            let ey = p.1 + sy * bx + cy * by;
                            let j = ((ex - cfg.x_range.0) / cfg.cell).floor() as i32;
                            let i = ((ey - cfg.y_range.0) / cfg.cell).floor() as i32;
                            if i < 0 || j < 0 || i as usize >= h || j as usize >= w {
                                out += 1;
                                continue;
                            }
                            let lo = g.log_at(i as usize, j as usize);
                            if lo > 50 {
                                on_wall += 1;
                            } else if lo < -50 {
                                in_free += 1;
                            } else {
                                unknown += 1;
                            }
                        }
                        println!(
                            "[{t:7.1}s]   vs pre-kidnap map: wall {on_wall}, FREE {in_free}, unknown {unknown}, off-map {out}"
                        );
                    }
                    let tr = mapper.slam().tracked();
                    let vs_truth = mapper
                        .last_window()
                        .map(|(p, s)| truth.scan_agreement(*p, s))
                        .unwrap_or(f32::NAN);
                    println!(
                        "[{t:7.1}s] window {beams:5} beams  vs-map {mean_residual_m:.3} ({n_observed:5}/{n_beams:5})  vs-TRUTH {vs_truth:.3}  tracked ({:6.2}, {:6.2}, {:6.1}°)",
                        tr.0,
                        tr.1,
                        tr.2.to_degrees()
                    );
                    if after_kidnap && lost_at.is_none() && relocalized.is_none() {
                        windows_inked_while_wrong += 1;
                    }
                }
                Note::LostTracking {
                    mean_residual_m,
                    n_observed,
                } => {
                    let vs_truth = mapper
                        .last_window()
                        .map(|(p, s)| truth.scan_agreement(*p, s))
                        .unwrap_or(f32::NAN);
                    println!(
                        "[{t:7.1}s] LOST: vs-map {mean_residual_m:.3} m over {n_observed} beams, vs-TRUTH {vs_truth:.3}"
                    );
                    if after_kidnap && lost_at.is_none() {
                        lost_at = Some(t);
                    }
                }
                Note::Relocalized {
                    pose,
                    mean_residual_m,
                } => {
                    println!(
                        "[{t:7.1}s] RELOCALIZED to ({:.2}, {:.2}, {:.1}°), residual {:.3}",
                        pose.0,
                        pose.1,
                        pose.2.to_degrees(),
                        mean_residual_m
                    );
                    if after_kidnap && relocalized.is_none() {
                        relocalized = Some((t, pose, mean_residual_m));
                    }
                }
                Note::RelocalizeRejected { .. } => {
                    if after_kidnap {
                        reloc_rejections += 1;
                    }
                }
                Note::SuspectAfterSit => {
                    println!("[{t:7.1}s] SAT — pose suspect");
                    if lost_at.is_none() {
                        lost_at = Some(t);
                    }
                }
                Note::SuspectAfterFall => {
                    println!("[{t:7.1}s] FELL — pose suspect");
                }
                Note::ResumedUnverified { pose } => {
                    println!(
                        "[{t:7.1}s] resumed UNVERIFIED at ({:.2}, {:.2}, {:.1}°) — nothing could judge the pose",
                        pose.0,
                        pose.1,
                        pose.2.to_degrees()
                    );
                }
                Note::WindowQuarantined {
                    mean_residual_m,
                    n_observed,
                } => {
                    println!(
                        "[{t:7.1}s] quarantined: vs-map {mean_residual_m:.3} m over {n_observed} beams"
                    );
                }
                Note::RelocalizeCandidate {
                    pose,
                    mean_residual_m,
                } => {
                    println!(
                        "[{t:7.1}s] candidate ({:.2}, {:.2}, {:.1}°) residual {mean_residual_m:.3} — awaiting confirmation",
                        pose.0,
                        pose.1,
                        pose.2.to_degrees()
                    );
                }
                Note::LoopClosed {
                    n_loops, dx, dy, ..
                } => {
                    println!("[{t:7.1}s] loop closed (total {n_loops}), moved ({dx:.3}, {dy:.3})");
                }
                Note::WindowDiscarded { .. } => {}
            }
        }
    }

    println!();
    println!(
        "session: {:.0} s, {} odom ticks, {} depth frames, {} windows, {} submaps, {} loops",
        t_end,
        n_odom,
        n_frames,
        mapper.windows(),
        mapper.slam().n_submaps(),
        mapper.slam().n_loops()
    );

    // ── Return-to-start ─────────────────────────────────────────────────
    // The robot boots at `start` (map origin) and stands there again just
    // before the sit: truth for both the tracked pose and raw odometry is
    // exactly (0, 0, 0).
    println!();
    match (tracked_at_sit, odom_at_sit, first_odom) {
        (Some(tracked), Some(odom), Some(first)) => {
            let raw = between(first, odom);
            println!(
                "return-to-start   tracked: {:.3} m / {:.1}°    raw odometry: {:.3} m / {:.1}°",
                tracked.0.hypot(tracked.1),
                tracked.2.to_degrees().abs(),
                raw.0.hypot(raw.1),
                raw.2.to_degrees().abs()
            );
        }
        _ => println!("return-to-start: no sit found — protocol not detected in this log"),
    }

    // ── The kidnap ──────────────────────────────────────────────────────
    match sit_span {
        Some((s0, s1)) => {
            println!(
                "carry (sit): {s0:.1} s → {s1:.1} s;  truth after: map ({:.2}, {:.2}, {:.1}°)",
                kidnap_map.0,
                kidnap_map.1,
                kidnap_map.2.to_degrees()
            );
            match lost_at {
                Some(t) if t <= s1 => {
                    println!("  pose suspect from the sit itself (armed at {t:.1} s)")
                }
                Some(t) => println!("  tracking lost {:.1} s after the stand", t - s1),
                None => println!("  tracking was NEVER declared lost after the kidnap"),
            }
            match relocalized {
                Some((t, pose, resid)) => {
                    let err = (pose.0 - kidnap_map.0).hypot(pose.1 - kidnap_map.1);
                    let dyaw = (pose.2 - kidnap_map.2).rem_euclid(std::f32::consts::TAU);
                    let dyaw = dyaw.min(std::f32::consts::TAU - dyaw);
                    println!(
                        "  relocalized {:.1} s after the stand: error {:.3} m / {:.1}° (residual {:.3}, {} rejected attempts)",
                        t - s1,
                        err,
                        dyaw.to_degrees(),
                        resid,
                        reloc_rejections
                    );
                    println!(
                        "  (pose error is vs the measured kidnap pose — only exact if the robot had not yet walked)"
                    );
                }
                None => println!(
                    "  NEVER relocalized ({reloc_rejections} rejected attempts, {} windows inked at a wrong pose)",
                    windows_inked_while_wrong
                ),
            }
            if windows_inked_while_wrong > 0 {
                println!(
                    "  {} windows inked before the loss was noticed",
                    windows_inked_while_wrong
                );
            }
            let _ = n_windows_pre;
        }
        None => println!("kidnap: no ≥2 s sit in the log — protocol not detected"),
    }

    // ── Map vs the measured room ────────────────────────────────────────
    if let Some(grid) = mapper.slam().render() {
        let cfg = *grid.cfg();
        let mut dists: Vec<f32> = Vec::new();
        for i in 0..grid.height() {
            for j in 0..grid.width() {
                if grid.log_at(i, j) > 150 {
                    let x = cfg.x_range.0 + (j as f32 + 0.5) * cfg.cell;
                    let y = cfg.y_range.0 + (i as f32 + 0.5) * cfg.cell;
                    dists.push(truth.wall_distance((x, y)));
                }
            }
        }
        dists.sort_by(f32::total_cmp);
        if !dists.is_empty() {
            let mean = dists.iter().sum::<f32>() / dists.len() as f32;
            let q = |p: f32| dists[((dists.len() - 1) as f32 * p) as usize];
            println!();
            println!(
                "map walls vs room ({} cells): mean {:.3} m   p50 {:.3}   p90 {:.3}   max {:.3}",
                dists.len(),
                mean,
                q(0.5),
                q(0.9),
                dists[dists.len() - 1]
            );
        }

        // PGMs: the map, and the map with the truth walls burned in.
        let stem = session
            .file_stem()
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_else(|| "session".into());
        save_pgm(&grid, &out_dir.join(format!("{stem}.pgm")), None);
        save_pgm(
            &grid,
            &out_dir.join(format!("{stem}_truth.pgm")),
            Some(&truth),
        );
        println!(
            "maps: {}/{{{stem}.pgm, {stem}_truth.pgm}}",
            out_dir.display()
        );
    } else {
        println!("no map was built");
    }
}

/// Dark = map wall, light = free, grey = unknown; with `truth`, the
/// measured walls burn in at mid-dark so alignment is visible at a glance.
fn save_pgm(grid: &maploc::OccupancyGrid, path: &std::path::Path, truth: Option<&Truth>) {
    let (w, h) = (grid.width(), grid.height());
    let cfg = *grid.cfg();
    let mut out = format!("P5 {w} {h} 255\n").into_bytes();
    for i in (0..h).rev() {
        for j in 0..w {
            let lo = grid.log_at(i, j);
            let mut px = if lo > 150 {
                0u8
            } else if lo < -50 {
                230
            } else {
                160
            };
            if let Some(t) = truth {
                let x = cfg.x_range.0 + (j as f32 + 0.5) * cfg.cell;
                let y = cfg.y_range.0 + (i as f32 + 0.5) * cfg.cell;
                if t.wall_distance((x, y)) < cfg.cell {
                    px = 70;
                }
            }
            out.push(px);
        }
    }
    std::fs::write(path, out).expect("write pgm");
}
