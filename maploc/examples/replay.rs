//! Offline SLAM bench: run a recorded `.mdlg` session through the full
//! pipeline and interrogate the result — the tool the prototype never had
//! when its relocalization "never worked".
//!
//!     cargo run -p maploc --release --example replay -- <session.mdlg> [out_dir]
//!
//! What it does, in order:
//!   1. Replays the session: odometry from the twin records, depth frames
//!      reprojected through `kinematics::tof` with the *recorded* IMU
//!      attitude and trunk height (the lean-aware filter the prototype
//!      lacked), integrated into submaps with loop closure — the same
//!      pipeline robotd hosts.
//!   2. Dumps the global map as a PGM next to the session, for eyeballs.
//!   3. Re-runs every N-th scan of the session's second half as a
//!      relocalize probe against the finished map: brute-force search and
//!      the residual at the "true" (tracked) pose, so map/scan agreement
//!      is a number, not a feeling.
//!
//! Ground truth here is the tracked pose itself — circular, but exactly
//! the right measure for the question "can a scan taken from a known pose
//! find that pose again in the map it helped build".

use std::collections::VecDeque;
use std::path::PathBuf;

use kinematics::tof::{Posture, Reprojector};
use maploc::accumulator::{AccumulatorConfig, WindowAccumulator};
use maploc::loop_closer::LoopCloserConfig;
use maploc::mcl::{Localizer, MclConfig};
use maploc::pipeline::{Slam, SlamConfig};
use maploc::pose_graph::{between, compose};
use maploc::relocalize::{RelocalizeConfig, relocalize_against_grid};
use maploc::replay::{Record, SessionReplayer, TwinRecord};
use maploc::submap::{Pose2, Scan};

const N_ZONES: usize = kinematics::tof::ROWS * kinematics::tof::COLS;

fn main() {
    let mut args = std::env::args().skip(1);
    let session: PathBuf = args
        .next()
        .expect("usage: replay <session.mdlg> [out_dir]")
        .into();
    let out_dir: PathBuf = args.next().map(Into::into).unwrap_or_else(|| {
        session
            .parent()
            .unwrap_or(&PathBuf::from("."))
            .to_path_buf()
    });

    let rp = Reprojector::alpha();
    let mut slam = Slam::new(SlamConfig {
        loops: LoopCloserConfig {
            verbose: std::env::var("MAPLOC_VERBOSE").is_ok(),
            ..SlamConfig::default().loops
        },
        ..SlamConfig::default()
    });
    let mut latest_twin: Option<TwinRecord> = None;
    // Stillness from odometry: displacement across the last ~0.5 s window.
    let mut odom_window: VecDeque<(u64, f32, f32, f32)> = VecDeque::new();

    let continuous = std::env::var("MAPLOC_CONTINUOUS").is_ok();
    let raw_filter = std::env::var("MAPLOC_RAW").is_ok(); // bypass the accumulator
    let mut acc = WindowAccumulator::new(AccumulatorConfig::default());
    // Integrated composites, each anchored to the submap it was inked
    // into: (submap index, pose in that submap's frame, scan). Truth for
    // the probes is recomputed from the FINAL anchors — loop closures move
    // submaps, and a probe scored against the corrected map must be
    // scored at the corrected pose.
    let mut kept: Vec<(usize, Pose2, Scan)> = Vec::new();
    let mut raw_kept: Vec<(Pose2, Scan)> = Vec::new(); // every still frame, unfiltered
    let mut kept_head_yaw: Vec<f32> = Vec::new(); // parallel to raw_kept
    let (mut n_frames, mut n_used, mut n_beams_total) = (0u32, 0u32, 0u64);
    let mut n_still_frames = 0u32;

    for record in SessionReplayer::open(&session).expect("open session") {
        let record = record.expect("read record");
        match record {
            Record::Twin(t) => {
                let odom = (t.odom_x, t.odom_y, t.odom_yaw);
                if odom.0.is_finite() && odom.1.is_finite() && odom.2.is_finite() {
                    slam.observe_odom(odom);
                    odom_window.push_back((t.ts_us, odom.0, odom.1, odom.2));
                    while odom_window
                        .front()
                        .is_some_and(|f| t.ts_us.saturating_sub(f.0) > 500_000)
                    {
                        odom_window.pop_front();
                    }
                }
                latest_twin = Some(t);
                let before = slam.n_loops();
                slam.tick(t.ts_us as f32 / 1e6);
                if slam.n_loops() > before {
                    println!("loops: {} (total)", slam.n_loops());
                }
            }
            Record::Odom(_) => {
                // Version-2 logs (robotd's own recorder) are the `evaluate`
                // example's domain — this bench replays prototype captures.
            }
            Record::Tof(frame) => {
                n_frames += 1;
                let Some(twin) = latest_twin.as_ref() else {
                    continue;
                };
                let still = {
                    let (first, last) = match (odom_window.front(), odom_window.back()) {
                        (Some(f), Some(l)) => (f, l),
                        _ => continue,
                    };
                    let d = ((last.1 - first.1).powi(2) + (last.2 - first.2).powi(2)).sqrt();
                    let dyaw = wrap_pi(last.3 - first.3).abs();
                    d < 0.01 && dyaw < 0.05
                };
                if still {
                    n_still_frames += 1;
                }
                if !continuous && !still {
                    // A still window just ended: vote, filter, integrate.
                    if let Some((pose, composite)) = acc.finish() {
                        let idx = slam.n_submaps().saturating_sub(1);
                        let local = between(slam.anchor(idx).unwrap_or((0.0, 0.0, 0.0)), pose);
                        slam.integrate(pose, &composite);
                        n_used += 1;
                        kept.push((idx, local, composite));
                    }
                    continue;
                }

                // Reproject through the head FK with the recorded posture —
                // the lean-aware path the prototype never had.
                let q = kinematics::Quat::new(
                    f64::from(twin.quat_wxyz[0]),
                    f64::from(twin.quat_wxyz[1]),
                    f64::from(twin.quat_wxyz[2]),
                    f64::from(twin.quat_wxyz[3]),
                )
                .normalized();
                let gravity = q.conjugate().rotate([0.0, 0.0, -1.0]);
                let head = [
                    f64::from(twin.joints[5]),
                    f64::from(twin.joints[6]),
                    f64::from(twin.joints[7]),
                    f64::from(twin.joints[8]),
                ];
                let posture = Posture {
                    gravity,
                    trunk_height_m: (twin.odom_z > 0.02).then_some(f64::from(twin.odom_z)),
                };
                let mut ranges = [None; N_ZONES];
                for (slot, row) in ranges.chunks_mut(8).zip(frame.ranges_m.iter()) {
                    for (s, &r) in slot.iter_mut().zip(row.iter()) {
                        if r.is_finite() && r > 0.0 {
                            *s = Some(f64::from(r));
                        }
                    }
                }
                let flat = rp.flatten(&ranges, head, &posture);
                if flat.angles_body.is_empty() {
                    continue;
                }
                n_beams_total += flat.angles_body.len() as u64;
                let scan = Scan::from_polar(&flat.angles_body, &flat.ranges, flat.sensor_xy, 1e-3);
                raw_kept.push((slam.tracked(), scan.clone()));
                kept_head_yaw.push(twin.joints[7]);
                if continuous || raw_filter {
                    let tracked = slam.tracked();
                    let idx = slam.n_submaps().saturating_sub(1);
                    let local = maploc::pose_graph::between(
                        slam.anchor(idx).unwrap_or((0.0, 0.0, 0.0)),
                        tracked,
                    );
                    slam.integrate(tracked, &scan);
                    n_used += 1;
                    kept.push((idx, local, scan));
                } else {
                    acc.push(slam.tracked(), scan);
                }
            }
        }
    }

    if let Some((pose, composite)) = acc.finish() {
        let idx = slam.n_submaps().saturating_sub(1);
        let local = between(slam.anchor(idx).unwrap_or((0.0, 0.0, 0.0)), pose);
        slam.integrate(pose, &composite);
        n_used += 1;
        kept.push((idx, local, composite));
    }
    // Truth poses through the FINAL anchors.
    let kept: Vec<(Pose2, Scan)> = kept
        .into_iter()
        .map(|(idx, local, scan)| {
            (
                compose(slam.anchor(idx).unwrap_or((0.0, 0.0, 0.0)), local),
                scan,
            )
        })
        .collect();
    println!(
        "\n{}: {n_frames} depth frames, {n_still_frames} still, {n_used} integrated \
         (mean {:.1} beams/scan), {} submaps, {} loops",
        session.display(),
        n_beams_total as f64 / f64::from(n_used.max(1)),
        slam.n_submaps(),
        slam.n_loops(),
    );

    // Composite scans: consecutive frames captured at (nearly) the same
    // body pose merge into one wide signature — per-beam origins make the
    // merge exact even when the head panned between frames. This is what a
    // relocalizing robot can manufacture on demand by sweeping its head.
    let mut composites: Vec<(Pose2, Scan)> = Vec::new();
    for (pose, scan) in &kept {
        match composites.last_mut() {
            Some((cp, cs))
                if (cp.0 - pose.0).hypot(cp.1 - pose.1) < 0.03
                    && wrap_pi(cp.2 - pose.2).abs() < 0.05 =>
            {
                cs.merge(scan);
            }
            _ => composites.push((*pose, scan.clone())),
        }
    }
    println!(
        "composites: {} (mean {:.0} beams)",
        composites.len(),
        composites.iter().map(|(_, s)| s.n_valid()).sum::<usize>() as f64
            / composites.len().max(1) as f64
    );

    // ── Within-window self-consistency ──────────────────────────────────
    // Frames captured at one still pose disagree with each other exactly as
    // much as the projection pipeline is wrong (sensor noise aside): build
    // a small grid from a window's even frames, score its odd frames at the
    // same pose. High numbers here mean the fuzz is born before mapping —
    // FK/calibration/wobble — not accumulated by it.
    {
        let mut groups: Vec<Vec<usize>> = Vec::new();
        for (i, (pose, _)) in raw_kept.iter().enumerate() {
            match groups.last_mut() {
                Some(g)
                    if {
                        let p0 = raw_kept[g[0]].0;
                        (p0.0 - pose.0).hypot(p0.1 - pose.1) < 0.03
                            && wrap_pi(p0.2 - pose.2).abs() < 0.05
                    } =>
                {
                    g.push(i)
                }
                _ => groups.push(vec![i]),
            }
        }
        let mut resids: Vec<f32> = Vec::new();
        for g in groups.iter().filter(|g| g.len() >= 20) {
            let pose = raw_kept[g[0]].0;
            let mut local = maploc::OccupancyGrid::new(maploc::GridConfig {
                x_range: (pose.0 - 3.0, pose.0 + 3.0),
                y_range: (pose.1 - 3.0, pose.1 + 3.0),
                cell: 0.05,
            });
            let mut probe = Scan::default();
            for (k, &idx) in g.iter().enumerate() {
                if k % 2 == 0 {
                    // Even frames ink the reference grid at the window pose.
                    let (p, scan) = &raw_kept[idx];
                    let (sy, cy) = (p.2.sin(), p.2.cos());
                    for &((obx, oby), (ebx, eby)) in &scan.beams {
                        let ox = p.0 + cy * obx - sy * oby;
                        let oy = p.1 + sy * obx + cy * oby;
                        let hx = p.0 + cy * ebx - sy * eby;
                        let hy = p.1 + sy * ebx + cy * eby;
                        local.integrate_ray(ox, oy, hx, hy, true);
                    }
                } else {
                    probe.merge(&raw_kept[idx].1);
                }
            }
            if probe.n_valid() < 50 {
                continue;
            }
            // Score every odd frame at ITS OWN tracked pose, so within-
            // window pose drift doesn't masquerade as projection error.
            let (mut sum, mut n) = (0.0f32, 0usize);
            for (k, &idx) in g.iter().enumerate() {
                if k % 2 == 0 {
                    continue;
                }
                let (p, scan) = &raw_kept[idx];
                if scan.n_valid() == 0 {
                    continue;
                }
                sum += residual_at(&mut local, scan, *p) * scan.n_valid() as f32;
                n += scan.n_valid();
            }
            if n > 0 {
                let resid = sum / n as f32;
                let (mut lo, mut hi) = (f32::INFINITY, f32::NEG_INFINITY);
                for &idx in g {
                    lo = lo.min(kept_head_yaw[idx]);
                    hi = hi.max(kept_head_yaw[idx]);
                }
                println!(
                    "  window {:>4} frames  head-yaw span {:>5.1}°  resid {:.3}",
                    g.len(),
                    (hi - lo).to_degrees(),
                    resid
                );
                resids.push(resid);
            }
        }
        resids.sort_by(f32::total_cmp);
        if !resids.is_empty() {
            println!(
                "window self-consistency over {} windows: median {:.3} m, p90 {:.3} m",
                resids.len(),
                resids[resids.len() / 2],
                resids[(resids.len() * 9 / 10).min(resids.len() - 1)],
            );
        }
    }

    let Some(mut global) = slam.render() else {
        println!("no submaps — nothing to render");
        return;
    };
    let pgm = out_dir.join(format!(
        "{}.pgm",
        session.file_stem().unwrap_or_default().to_string_lossy()
    ));
    save_pgm(&global, &pgm);
    println!(
        "map: {} ({}x{})",
        pgm.display(),
        global.width(),
        global.height()
    );
    // Trajectory next to it, for overlay tooling: x, y, yaw per kept scan.
    let traj = out_dir.join(format!(
        "{}.traj.csv",
        session.file_stem().unwrap_or_default().to_string_lossy()
    ));
    let mut csv = String::from("x,y,yaw\n");
    let gcfg = *global.cfg();
    csv.push_str(&format!(
        "# x_min={} y_min={} cell={}\n",
        gcfg.x_range.0, gcfg.y_range.0, gcfg.cell
    ));
    for (p, _) in &raw_kept {
        csv.push_str(&format!("{},{},{}\n", p.0, p.1, p.2));
    }
    std::fs::write(&traj, csv).expect("write traj");

    // ── Relocalize probes: one composite each ───────────────────────────
    let probes: Vec<&(Pose2, Scan)> = composites
        .iter()
        .skip(composites.len() / 2)
        .step_by((composites.len() / 20).max(1))
        .collect();
    println!("\nrelocalize probes ({}):", probes.len());
    let mut hits = 0usize;
    for (truth, scan) in &probes {
        // What the truth pose itself scores — the floor any search result
        // must beat, and the直est measure of map/scan self-consistency.
        let truth_resid = residual_at(&mut global, scan, *truth);
        let bf = relocalize_against_grid(&mut global, scan, &RelocalizeConfig::default());
        match bf {
            Some(bf) => {
                let dx = bf.pose.0 - truth.0;
                let dy = bf.pose.1 - truth.1;
                let derr = (dx * dx + dy * dy).sqrt();
                let yerr = wrap_pi(bf.pose.2 - truth.2).abs();
                let ok = derr < 0.30 && yerr < 0.35;
                if ok {
                    hits += 1;
                }
                println!(
                    "  {}  truth=({:+.2},{:+.2},{:+.0}°) scores {:.3}  \
                     found=({:+.2},{:+.2},{:+.0}°) err={:.2} m/{:.0}°  \
                     resid={:.3}  beams={}/{}  accepted={}",
                    if ok { "OK " } else { "MISS" },
                    truth.0,
                    truth.1,
                    truth.2.to_degrees(),
                    truth_resid,
                    bf.pose.0,
                    bf.pose.1,
                    bf.pose.2.to_degrees(),
                    derr,
                    yerr.to_degrees(),
                    bf.mean_residual_m,
                    bf.n_beams_used,
                    scan.n_valid(),
                    bf.accepted,
                );
            }
            None => println!(
                "  NONE  truth=({:+.2},{:+.2})  scan had {} beams (< min gate?)",
                truth.0,
                truth.1,
                scan.n_valid()
            ),
        }
    }
    println!("brute-force: {hits}/{} within 0.30 m / 20°", probes.len());

    // ── MCL from uniform, fed the probe sequence in order ────────────────
    let mut mcl = Localizer::new(MclConfig::default(), 0xC0FFEE);
    mcl.seed_uniform(&global);
    let mut prev: Option<Pose2> = None;
    let mut locked_at = None;
    for (i, (truth, scan)) in composites.iter().skip(composites.len() / 2).enumerate() {
        if let Some(p) = prev {
            let (dxw, dyw) = (truth.0 - p.0, truth.1 - p.1);
            let (cp, sp) = (p.2.cos(), p.2.sin());
            mcl.predict(
                cp * dxw + sp * dyw,
                -sp * dxw + cp * dyw,
                wrap_pi(truth.2 - p.2),
            );
        }
        prev = Some(*truth);
        mcl.update(&mut global, scan);
        if mcl.is_locked() && locked_at.is_none() {
            let best = mcl.dominant_cluster_mean();
            let derr = ((best.0 - truth.0).powi(2) + (best.1 - truth.1).powi(2)).sqrt();
            locked_at = Some((i, derr, wrap_pi(best.2 - truth.2).abs()));
        }
    }
    match locked_at {
        Some((i, derr, yerr)) => println!(
            "MCL locked at frame {i}: err {derr:.2} m / {:.0}°",
            yerr.to_degrees()
        ),
        None => {
            let best = mcl.dominant_cluster_mean();
            let frac = mcl.dominant_cluster_frac();
            println!(
                "MCL never locked ({} frames): cluster=({:+.2},{:+.2},{:+.0}°) \
                 frac={:.0}% resid={:.3} streak={}",
                composites.len() - composites.len() / 2,
                best.0,
                best.1,
                best.2.to_degrees(),
                frac * 100.0,
                mcl.last_residual_m(),
                mcl.locked_streak(),
            );
        }
    }
}

/// The global grid as a PGM: dark = occupied, light = free, grey = unknown.
fn save_pgm(grid: &maploc::OccupancyGrid, path: &std::path::Path) {
    let (w, h) = (grid.width(), grid.height());
    let mut out = format!("P5 {w} {h} 255\n").into_bytes();
    for i in (0..h).rev() {
        for j in 0..w {
            let lo = grid.log_at(i, j);
            let px = if lo > 50 {
                0u8
            } else if lo < -50 {
                230
            } else {
                128
            };
            out.push(px);
        }
    }
    std::fs::write(path, out).expect("write pgm");
}

/// Mean clamped distance-to-wall of the scan's beams at `pose` — the same
/// score `relocalize` uses, evaluated at one pose.
fn residual_at(grid: &mut maploc::OccupancyGrid, scan: &Scan, pose: Pose2) -> f32 {
    let cfg = RelocalizeConfig::default();
    let field = grid.distance_field(cfg.wall_threshold_fp).to_vec();
    let g = *grid.cfg();
    let (w, h) = (grid.width(), grid.height());
    let (sy, cy) = pose.2.sin_cos();
    let mut sum = 0.0f32;
    let mut n = 0usize;
    for (_, (bx, by)) in &scan.beams {
        let ex = cy * bx - sy * by;
        let ey = sy * bx + cy * by;
        let hx = pose.0 + ex;
        let hy = pose.1 + ey;
        let j = ((hx - g.x_range.0) / g.cell).floor() as i32;
        let i = ((hy - g.y_range.0) / g.cell).floor() as i32;
        let mj = ((pose.0 + ex * 0.5 - g.x_range.0) / g.cell).floor() as i32;
        let mi = ((pose.1 + ey * 0.5 - g.y_range.0) / g.cell).floor() as i32;
        let through_wall = mi >= 0
            && mj >= 0
            && (mi as usize) < h
            && (mj as usize) < w
            && grid.log_at(mi as usize, mj as usize) > cfg.wall_threshold_fp;
        let d = if through_wall || i < 0 || j < 0 || (i as usize) >= h || (j as usize) >= w {
            cfg.clamp_m
        } else {
            field[(i as usize) * w + (j as usize)].min(cfg.clamp_m)
        };
        sum += d;
        n += 1;
    }
    if n == 0 { f32::NAN } else { sum / n as f32 }
}

fn wrap_pi(a: f32) -> f32 {
    use std::f32::consts::PI;
    let two_pi = 2.0 * PI;
    let mut y = (a + PI).rem_euclid(two_pi) - PI;
    if y == PI {
        y = -PI;
    }
    y
}
