//! Brute-force relocalize against a saved global map.
//!
//! Used for kidnapped-robot recovery: given a single ToF scan and a
//! prior occupancy grid (e.g. the global render of a loaded session),
//! search over `(x, y, yaw)` candidates and return the pose with the
//! smallest mean per-beam residual against the grid's distance field.
//!
//! Cost is O(N_free × N_yaw × N_beams). On a 4×4 m room at 5 cm cells,
//! that's ~6400 free cells × 36 yaw bins × 64 beams = ~15 M lookups —
//! a few hundred ms of one-shot search at the start of a session, then
//! we go back to the regular live SLAM pipeline.
//!
//! Two-stage search: a coarse pass over the full grid at `cfg.coarse_xy_stride`
//! cells / `cfg.coarse_yaw_bins` bins, followed by a refinement pass at
//! single-cell / fine-bin resolution around the best coarse candidate.

use crate::grid::OccupancyGrid;
use crate::pose_graph::wrap_pi;
use crate::submap::Scan;

#[derive(Debug, Clone)]
pub struct RelocalizeConfig {
    /// Coarse stride over (x, y) free cells.
    pub coarse_xy_stride: usize,
    /// Coarse number of yaw bins covering 0..2π.
    pub coarse_yaw_bins: usize,
    /// Half-width of the local refinement window (cells).
    pub refine_xy_radius: usize,
    /// Refinement yaw bins covering ±`refine_yaw_half_rad`.
    pub refine_yaw_bins: usize,
    pub refine_yaw_half_rad: f32,
    /// Acceptance threshold on the mean per-beam residual (metres).
    pub max_mean_residual_m: f32,
    /// Minimum number of valid beams a candidate must explain.
    pub min_beams_used: u32,
    /// Per-beam residual is clamped to this so a single very-far beam
    /// can't dominate the mean (matches the Hector saturation trick).
    pub clamp_m: f32,
    /// Fixed-point log-odds threshold above which a cell counts as a
    /// wall for the distance-field score. 0 = use every barely-positive
    /// cell (legacy); ~200 = require 3+ net hits to confirm a wall, so
    /// transient ToF flickers in the saved map don't pull the search
    /// off-target.
    pub wall_threshold_fp: i16,
    /// Threshold for the SEE-THROUGH penalty (a ray crossing a wall).
    /// Deliberately higher than `wall_threshold_fp`: a single window's
    /// floor phantoms land as one-window ink, and treating them as
    /// opaque mass-clamps every ray from the next stop nearby — the
    /// measured false-LOST of field test four. Crossing only counts
    /// against a wall more than one window confirmed.
    pub see_through_fp: i16,
}

impl Default for RelocalizeConfig {
    fn default() -> Self {
        // Tight defaults — we'd rather reject and search more frames
        // than declare a wrong pose locked. The two-stage search
        // doesn't cost a lot at 5 cm resolution, and a real ToF scan
        // captured from the *correct* pose against a clean global
        // render gives mean per-beam residuals well under 5 cm.
        Self {
            coarse_xy_stride: 2,
            coarse_yaw_bins: 36,
            refine_xy_radius: 4,
            refine_yaw_bins: 11,
            refine_yaw_half_rad: 10.0_f32.to_radians(),
            max_mean_residual_m: 0.05,
            min_beams_used: 24,
            clamp_m: 0.15,
            wall_threshold_fp: 200,
            see_through_fp: 300,
        }
    }
}

#[derive(Debug, Clone)]
pub struct RelocalizeResult {
    pub pose: (f32, f32, f32),
    pub mean_residual_m: f32,
    pub n_beams_used: u32,
    /// True iff `mean_residual_m <= cfg.max_mean_residual_m` and
    /// `n_beams_used >= cfg.min_beams_used`.
    pub accepted: bool,
}

/// Score a single (cx, cy) candidate against a precomputed distance
/// field, with the beam endpoints already rotated by the candidate yaw.
/// Precomputing the offsets per yaw removes all trig from the inner loop —
/// the offsets are constant across every (cx, cy) at a given yaw, which is
/// ~1600 cells in the coarse pass. Returns `(sum_clamped_residual, n_in_map)`.
///
/// **Every beam is counted.** A beam whose endpoint falls outside the map
/// contributes the full clamp penalty rather than being skipped: skipping
/// let a wrong pose that throws most of its beams off the map get scored
/// on whatever cherry-picked subset happened to land on a wall — measured
/// on real sessions, such poses scored mean residuals near zero from a
/// tenth of the scan and beat the true pose everywhere. The returned
/// count is the number of beams that actually landed in the map, so the
/// caller can still gate on coverage.
///
/// Index conversion is floor-based (truncation aliased a one-cell band
/// outside the min borders onto row/col 0).
/// A wall the ray would have had to see *through* is as damning as an
/// endpoint in free space. Without this check, endpoints-only scoring has
/// a degenerate optimum: any pose that throws its whole scan into a dense
/// blob of wall cells scores near zero — measured on real sessions, such
/// poses beat the true one by an order of magnitude. One sample at the
/// ray's midpoint breaks the degeneracy for a fraction of a ray-cast's
/// cost.
#[inline]
#[allow(clippy::too_many_arguments)] // a hot kernel fed unpacked grid metadata on purpose
fn score_offsets(
    cx: f32,
    cy: f32,
    offsets: &[(f32, f32)],
    field: &[f32],
    log: &[i16],
    see_through_fp: i16,
    w: usize,
    h: usize,
    x_min: f32,
    y_min: f32,
    cell: f32,
    clamp_m: f32,
) -> (f32, u32) {
    let mut sum = 0.0_f32;
    let mut n = 0u32;
    for &(ox, oy) in offsets {
        let hx = cx + ox;
        let hy = cy + oy;
        let j = ((hx - x_min) / cell).floor() as i32;
        let i = ((hy - y_min) / cell).floor() as i32;
        if i < 0 || j < 0 || (i as usize) >= h || (j as usize) >= w {
            sum += clamp_m;
            continue;
        }
        // Midpoint of the (approximate) ray from the pose to the endpoint:
        // seeing through a confident wall costs the full clamp — but only
        // when the crossed cell is well away from the endpoint's. A beam
        // that ENDS on a wall at close range or grazing incidence has its
        // midpoint inside that same wall's cells, and clamping it punishes
        // the beam for hitting the very wall it measured (the measured
        // false-LOST of field test four: a robot 10 cm from a divider had
        // 4 200 of 10 800 beams "seeing through" the divider they hit).
        let mj = ((cx + ox * 0.5 - x_min) / cell).floor() as i32;
        let mi = ((cy + oy * 0.5 - y_min) / cell).floor() as i32;
        if mi >= 0
            && mj >= 0
            && (mi as usize) < h
            && (mj as usize) < w
            && ((mi - i).abs() > 1 || (mj - j).abs() > 1)
            && log[(mi as usize) * w + (mj as usize)] > see_through_fp
        {
            sum += clamp_m;
            n += 1;
            continue;
        }
        let d = field[(i as usize) * w + (j as usize)];
        sum += d.min(clamp_m);
        n += 1;
    }
    (sum, n)
}

/// Rotate the body-frame endpoints by `yaw` — the candidate is a BODY
/// pose, and each offset already carries the sensor origin.
fn beam_offsets(endpoints: &[(f32, f32)], yaw: f32) -> Vec<(f32, f32)> {
    let (sy, cy) = yaw.sin_cos();
    endpoints
        .iter()
        .map(|&(bx, by)| (cy * bx - sy * by, sy * bx + cy * by))
        .collect()
}

/// How well a scan agrees with the map at one given pose — the tracking
/// watchdog's number.
#[derive(Debug, Clone, Copy)]
pub struct PoseAgreement {
    /// Mean clamped per-beam residual over the beams the map can judge.
    pub mean_residual_m: f32,
    /// Beams the map could judge: endpoint in a cell it has an opinion on
    /// (|log-odds| ≥ `observed_fp`), or a ray that would have had to see
    /// through a confident wall.
    pub n_observed: u32,
    /// All valid beams in the scan.
    pub n_beams: u32,
}

/// Score a scan against the map at a *known* pose. Beams landing in
/// unexplored cells are excluded from the mean and from `n_observed` —
/// walking into a new room must read as "cannot judge", never as "wrong".
/// A kidnapped robot, by contrast, throws beams into territory the map
/// knows well and disagrees with everywhere: high `n_observed`, high mean.
///
/// Endpoint distances only — no see-through term. The see-through clamp
/// exists to break a degeneracy of the SEARCH (a wrong pose burying its
/// scan in a wall blob); at a trusted pose there is nothing to exploit,
/// and the clamp's failure mode — punishing beams that hit a wall the
/// robot stands right next to — false-fired the watchdog in the field.
pub fn score_pose(
    grid: &mut OccupancyGrid,
    scan: &Scan,
    pose: (f32, f32, f32),
    clamp_m: f32,
    wall_threshold_fp: i16,
    observed_fp: i16,
) -> PoseAgreement {
    let field = grid.distance_field_shared(wall_threshold_fp);
    let cfg_g = *grid.cfg();
    let (w, h) = (grid.width(), grid.height());
    let (cell, x_min, y_min) = (cfg_g.cell, cfg_g.x_range.0, cfg_g.y_range.0);
    let log = grid.log_raw();

    let (sy, cy) = pose.2.sin_cos();
    let (mut sum, mut n_observed, mut n_beams) = (0.0_f32, 0u32, 0u32);
    for (bx, by) in scan.endpoints_body() {
        n_beams += 1;
        let ex = pose.0 + cy * bx - sy * by;
        let ey = pose.1 + sy * bx + cy * by;
        let j = ((ex - x_min) / cell).floor() as i32;
        let i = ((ey - y_min) / cell).floor() as i32;
        if i < 0 || j < 0 || (i as usize) >= h || (j as usize) >= w {
            continue; // off the map: nothing to compare against
        }
        let lo = log[(i as usize) * w + (j as usize)];
        if lo.abs() < observed_fp {
            continue; // unexplored cell: cannot judge this beam
        }
        sum += field[(i as usize) * w + (j as usize)].min(clamp_m);
        n_observed += 1;
    }
    PoseAgreement {
        mean_residual_m: if n_observed > 0 {
            sum / n_observed as f32
        } else {
            0.0
        },
        n_observed,
        n_beams,
    }
}

/// Search the grid for the best matching pose. Returns the global
/// minimum-residual candidate (ignoring acceptance — caller checks
/// `accepted` to decide whether to use the pose).
pub fn relocalize_against_grid(
    grid: &mut OccupancyGrid,
    scan: &Scan,
    cfg: &RelocalizeConfig,
) -> Option<RelocalizeResult> {
    // Cheap Arc clone of the cached field (no 100–200 KB copy) so we can
    // drop the mutable borrow before the search loop accesses immutable
    // grid methods.
    let field = grid.distance_field_shared(cfg.wall_threshold_fp);
    let log: Vec<i16> = grid.log_raw().to_vec();
    let cfg_g = *grid.cfg();
    let w = grid.width();
    let h = grid.height();
    let cell = cfg_g.cell;
    let x_min = cfg_g.x_range.0;
    let y_min = cfg_g.y_range.0;

    let valid: Vec<(f32, f32)> = scan.endpoints_body().collect();
    if (valid.len() as u32) < cfg.min_beams_used {
        return None;
    }

    // Free-cell candidate centres, computed once (shared by every yaw).
    let stride = cfg.coarse_xy_stride.max(1);
    let mut centres: Vec<(f32, f32)> = Vec::new();
    for ci in (0..h).step_by(stride) {
        for cj in (0..w).step_by(stride) {
            if !grid.is_known_free(ci, cj) {
                continue;
            }
            centres.push((
                x_min + (cj as f32 + 0.5) * cell,
                y_min + (ci as f32 + 0.5) * cell,
            ));
        }
    }
    if centres.is_empty() {
        return None;
    }

    let two_pi = 2.0 * std::f32::consts::PI;
    let coarse_yaw_bins = cfg.coarse_yaw_bins.max(1);
    let dyaw = two_pi / coarse_yaw_bins as f32;

    // Phase 1 — coarse pass, parallel over yaw bins (each bin is
    // independent: one offset table, one sweep over the centres). Each
    // bin keeps its own small top list; merged below. Keeping several
    // candidates (not just the winner) lets refinement examine
    // runners-up — in near-symmetric rooms the true pose routinely
    // coarse-scores second or third. Scoped threads, one chunk of yaw
    // bins per core, instead of a rayon dependency: the parallelism is
    // embarrassing and the pool would sit idle the rest of the session.
    const PER_YAW_TOP: usize = 4;
    let score_yaw_bin = |yi: usize| -> Vec<RelocalizeResult> {
        let yaw = -std::f32::consts::PI + yi as f32 * dyaw;
        let offsets = beam_offsets(&valid, yaw);
        let mut top: Vec<RelocalizeResult> = Vec::with_capacity(PER_YAW_TOP + 1);
        for &(cx, cy) in &centres {
            let (sum, n) = score_offsets(
                cx,
                cy,
                &offsets,
                &field,
                &log,
                cfg.see_through_fp,
                w,
                h,
                x_min,
                y_min,
                cell,
                cfg.clamp_m,
            );
            if n < cfg.min_beams_used {
                continue;
            }
            // Mean over EVERY beam — off-map beams carry the clamp.
            let mean = sum / (offsets.len() as f32);
            if top.len() == PER_YAW_TOP && mean >= top.last().unwrap().mean_residual_m {
                continue;
            }
            let cand = RelocalizeResult {
                pose: (cx, cy, yaw),
                mean_residual_m: mean,
                n_beams_used: n,
                accepted: false,
            };
            let at = top.partition_point(|c| c.mean_residual_m <= mean);
            top.insert(at, cand);
            top.truncate(PER_YAW_TOP);
        }
        top
    };
    let n_threads = std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(1)
        .min(coarse_yaw_bins);
    let mut candidates: Vec<RelocalizeResult> = std::thread::scope(|scope| {
        let score_yaw_bin = &score_yaw_bin;
        let handles: Vec<_> = (0..n_threads)
            .map(|t| {
                scope.spawn(move || {
                    let mut out = Vec::new();
                    let mut yi = t;
                    while yi < coarse_yaw_bins {
                        out.extend(score_yaw_bin(yi));
                        yi += n_threads;
                    }
                    out
                })
            })
            .collect();
        handles
            .into_iter()
            .flat_map(|handle| handle.join().expect("scoring cannot panic"))
            .collect()
    });
    if candidates.is_empty() {
        return None;
    }
    candidates.sort_by(|a, b| a.mean_residual_m.total_cmp(&b.mean_residual_m));

    // Deduplicate to the top-K spatially distinct seeds: a good pose is
    // surrounded by near-identical coarse candidates that would waste
    // the refinement budget re-examining the same basin.
    const TOP_K: usize = 5;
    const MIN_SEP_M: f32 = 0.40;
    const MIN_SEP_YAW: f32 = 0.60; // ~35°
    let mut seeds: Vec<RelocalizeResult> = Vec::with_capacity(TOP_K);
    for c in candidates {
        let dup = seeds.iter().any(|s| {
            let dx = s.pose.0 - c.pose.0;
            let dy = s.pose.1 - c.pose.1;
            let dyw = wrap_pi(s.pose.2 - c.pose.2).abs();
            dx * dx + dy * dy < MIN_SEP_M * MIN_SEP_M && dyw < MIN_SEP_YAW
        });
        if !dup {
            seeds.push(c);
        }
        if seeds.len() >= TOP_K {
            break;
        }
    }

    // Phase 2 — local refinement around every seed; global best wins.
    let mut best: Option<RelocalizeResult> = None;
    let r_xy = cfg.refine_xy_radius as i32;
    let yaw_bins = cfg.refine_yaw_bins.max(1);
    let yaw_step = if yaw_bins == 1 {
        0.0
    } else {
        2.0 * cfg.refine_yaw_half_rad / (yaw_bins - 1) as f32
    };
    for seed in &seeds {
        let (bx, by, byaw) = seed.pose;
        let bj = ((bx - x_min) / cell).floor() as i32;
        let bi = ((by - y_min) / cell).floor() as i32;
        for yi in 0..yaw_bins {
            let yaw = byaw - cfg.refine_yaw_half_rad + yi as f32 * yaw_step;
            let offsets = beam_offsets(&valid, yaw);
            for ddi in -r_xy..=r_xy {
                for ddj in -r_xy..=r_xy {
                    let ci = bi + ddi;
                    let cj = bj + ddj;
                    if ci < 0 || cj < 0 || (ci as usize) >= h || (cj as usize) >= w {
                        continue;
                    }
                    if !grid.is_known_free(ci as usize, cj as usize) {
                        continue;
                    }
                    let cx = x_min + (cj as f32 + 0.5) * cell;
                    let cy = y_min + (ci as f32 + 0.5) * cell;
                    let (sum, n) = score_offsets(
                        cx,
                        cy,
                        &offsets,
                        &field,
                        &log,
                        cfg.see_through_fp,
                        w,
                        h,
                        x_min,
                        y_min,
                        cell,
                        cfg.clamp_m,
                    );
                    if n < cfg.min_beams_used {
                        continue;
                    }
                    let mean = sum / (offsets.len() as f32);
                    if best.as_ref().is_none_or(|b| mean < b.mean_residual_m) {
                        best = Some(RelocalizeResult {
                            pose: (cx, cy, wrap_pi(yaw)),
                            mean_residual_m: mean,
                            n_beams_used: n,
                            accepted: false,
                        });
                    }
                }
            }
        }
    }

    let mut best = best.or_else(|| seeds.into_iter().next())?;
    best.pose.2 = wrap_pi(best.pose.2);
    best.accepted =
        best.mean_residual_m <= cfg.max_mean_residual_m && best.n_beams_used >= cfg.min_beams_used;
    Some(best)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::{GridConfig, OccupancyGrid};

    fn make_test_room() -> OccupancyGrid {
        // 4×4 m square room, walls at ±2 m, room is "L"-shaped to break
        // 4-fold symmetry: a chunk is missing in the +x/+y quadrant so
        // the relocalize candidate can be unambiguous.
        let mut g = OccupancyGrid::new(GridConfig {
            x_range: (-2.5, 2.5),
            y_range: (-2.5, 2.5),
            cell: 0.05,
        });
        // Outer walls — cast each ray a few times so the wall cells
        // accumulate enough log-odds to clear a reasonable `wall_threshold_fp`.
        let n = 200;
        for _ in 0..4 {
            for i in 0..n {
                let t = -2.0 + 4.0 * (i as f32 / (n - 1) as f32);
                g.integrate_ray(0.0, 0.0, t, 2.0, true);
                g.integrate_ray(0.0, 0.0, t, -2.0, true);
                g.integrate_ray(0.0, 0.0, 2.0, t, true);
                g.integrate_ray(0.0, 0.0, -2.0, t, true);
            }
        }
        // Asymmetric divider.
        for _ in 0..4 {
            for i in 0..n / 2 {
                let t = 0.0 + 2.0 * (i as f32 / ((n / 2) as f32));
                g.integrate_ray(0.0, 0.0, t, 0.5, true);
            }
        }
        g
    }

    #[test]
    fn relocalize_finds_known_pose() {
        let mut grid = make_test_room();
        // Simulated scan: at ground truth (0.5, -0.5, 0.0) cast 36 beams
        // and use the grid raycasts as ranges.
        let truth = (0.5_f32, -0.5_f32, 0.0_f32);
        let n_beams = 36;
        let mut angles = Vec::new();
        let mut ranges = Vec::new();
        for k in 0..n_beams {
            // ±90° fan in body frame.
            let a = -std::f32::consts::FRAC_PI_2
                + (k as f32 / (n_beams - 1) as f32) * std::f32::consts::PI;
            let r = grid.cast_ray(truth.0, truth.1, truth.2 + a, 4.0);
            if r > 0.0 {
                angles.push(a);
                ranges.push(r);
            }
        }
        let scan = Scan::from_polar(&angles, &ranges, (0.0, 0.0), 1e-6);
        let cfg = RelocalizeConfig::default();
        let res = relocalize_against_grid(&mut grid, &scan, &cfg)
            .expect("relocalize returns a candidate");
        assert!(
            res.accepted,
            "expected accepted, got mean_residual={:.3} m, n={}",
            res.mean_residual_m, res.n_beams_used
        );
        // Should land within a couple of cells of ground truth and within
        // a few degrees of the right yaw.
        let dx = res.pose.0 - truth.0;
        let dy = res.pose.1 - truth.1;
        let dist = (dx * dx + dy * dy).sqrt();
        assert!(
            dist < 0.20,
            "position {:.2} m off truth ({:?} vs {:?})",
            dist,
            res.pose,
            truth
        );
        let dyaw = (res.pose.2 - truth.2).abs();
        assert!(
            dyaw < 0.20 || (2.0 * std::f32::consts::PI - dyaw) < 0.20,
            "yaw {:.1}° off truth",
            dyaw.to_degrees()
        );
    }
}
