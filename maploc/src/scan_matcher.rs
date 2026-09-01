//! Hector-style scan-to-map ICP.
//!
//! Gauss-Newton minimization of the sum of squared distances from each
//! beam endpoint to the nearest mapped obstacle. The grid's distance
//! field gives O(1) per-beam residual + bilinear gradient, so each
//! iteration is cheap and ~10 iterations converge.
//!
//! Why this exists alongside MCL: MCL is a particle filter — great for
//! relocalising in a *known* map (uniform cloud → scan likelihood pulls
//! it home), bad for tracking + mapping a *partial* map (high residuals
//! in unmapped regions trigger kidnap injection / global relocalize and
//! the filter snaps to wrong-but-also-consistent clusters). Scan
//! matching is the textbook tracker: bounded local search, no particle
//! cloud, no chance of teleporting. Use this for SLAM mode (build +
//! drift-correct), use MCL for relocalize-from-uniform mode.
//!
//! Cost: ~1–3 ms per scan on a Pi 4 with 64 valid beams; should run
//! comfortably on a Pi Zero 2 W.

use crate::grid::OccupancyGrid;
use crate::pose_graph::wrap_pi;
use crate::submap::Scan;

/// Hyperparameters for [`match_scan`].
#[derive(Debug, Clone, Copy)]
pub struct ScanMatchConfig {
    pub max_iters: u32,
    /// Convergence: stop when |Δ| drops below all three thresholds.
    pub eps_translation_m: f32,
    pub eps_rotation_rad: f32,
    /// Levenberg damping added to the Hessian diagonal each step.
    /// 0 = pure Gauss-Newton.
    pub lambda: f32,
    /// Per-beam residual saturation. Beams whose endpoint is more than
    /// `sigma_m` from any wall contribute a capped distance instead of
    /// blowing up the cost — keeps unmapped regions from dominating.
    pub sigma_m: f32,
    /// Occupancy threshold (fixed-point log-odds) used by
    /// `OccupancyGrid::distance_field`. Kept equal to the MCL /
    /// relocalize default (200 ≈ 3+ net hits) so consumers sharing a
    /// grid also share its single-slot distance-field cache instead of
    /// forcing a full recompute on every alternation.
    pub occ_threshold_fp: i16,
    /// Optional Gaussian regularizer pulling the optimized pose toward
    /// `prior_pose` (typically the odometry-predicted pose). 0 = off.
    /// Stops the matcher from wandering when there's not enough scan
    /// signal to constrain all 3 DoF (e.g. looking at a long corridor).
    pub prior_sigma_xy: f32,
    pub prior_sigma_yaw: f32,
}

impl Default for ScanMatchConfig {
    fn default() -> Self {
        Self {
            max_iters: 12,
            eps_translation_m: 1e-3,
            eps_rotation_rad: 1e-3,
            lambda: 1e-3,
            sigma_m: 0.30,
            occ_threshold_fp: 200,
            prior_sigma_xy: 0.50,
            prior_sigma_yaw: 0.50,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ScanMatchResult {
    pub pose: (f32, f32, f32),
    /// Mean per-beam residual in metres (RMS distance-to-nearest-wall),
    /// evaluated at the *final* pose over beams within `sigma_m` of a
    /// wall. Beam-only: the pose prior regularizes the optimization but
    /// is excluded here, so gates like the loop closer's
    /// `max_residual_m` measure geometry, not how far the matcher moved
    /// from its seed. `INFINITY` when no beam landed near a wall.
    pub residual_m: f32,
    pub iterations: u32,
    pub converged: bool,
    /// Number of beams that contributed at the final pose (within
    /// `sigma_m` of a wall; NaN/zero ranges skipped).
    pub n_beams_used: u32,
    /// Number of geometrically valid beams in the scan (finite, > 0),
    /// regardless of whether they landed near a mapped wall.
    pub n_beams_valid: u32,
    /// Of the valid beams, how many landed in a cell the target grid has
    /// actually OBSERVED (known free or wall) at the final pose. THIS is
    /// the denominator for a coverage gate: a 360° scan matched against a
    /// small submap parks most of its beams outside what that submap ever
    /// saw, and counting those as "disagreeing" rejects every honest match.
    pub n_beams_observed: u32,
}

/// Scan-to-map ICP. Iteratively shifts `initial_pose` (a BODY pose) so
/// beam endpoints land on mapped obstacles, minimizing sum of squared
/// distances from each endpoint to its nearest wall (looked up via the
/// grid's distance field with bilinear interpolation). Endpoints are
/// `pose ∘ (sensor_in_body + r·dir)` — the same convention the map was
/// inked with.
///
/// `prior_pose` (if `Some`) is a Gaussian anchor pulling the result
/// toward that pose with the configured sigmas. Pass the odometry-
/// predicted pose to keep the matcher honest when scan signal is weak.
pub fn match_scan(
    grid: &mut OccupancyGrid,
    scan: &Scan,
    initial_pose: (f32, f32, f32),
    prior_pose: Option<(f32, f32, f32)>,
    cfg: &ScanMatchConfig,
) -> ScanMatchResult {
    // Body-frame endpoints, once: everything below is rotation + add.
    let endpoints: Vec<(f32, f32)> = scan.endpoints_body().collect();
    let n_beams_valid = endpoints.len() as u32;

    // Snapshot grid metadata before we (mutably) touch the field.
    let cell = grid.cell();
    let cell_inv = 1.0 / cell;
    let cfg_g = *grid.cfg();
    let h = grid.height();
    let w = grid.width();
    // The Arc clone releases the grid borrow, so the observedness check in
    // the final scoring can read log-odds while the field stays alive.
    let field = grid.distance_field_shared(cfg.occ_threshold_fp);

    // Bilinear-interpolated distance + gradient at world point.
    // Returns (d, ∂d/∂x, ∂d/∂y). Out-of-bounds: saturate to sigma, no gradient.
    let sample = |fx: f32, fy: f32| -> (f32, f32, f32) {
        let cx = (fx - cfg_g.x_range.0) * cell_inv - 0.5;
        let cy = (fy - cfg_g.y_range.0) * cell_inv - 0.5;
        let i0 = cy.floor() as i32;
        let j0 = cx.floor() as i32;
        if i0 < 0 || j0 < 0 || (i0 + 1) as usize >= h || (j0 + 1) as usize >= w {
            // Out of grid bounds: return distance > sigma_m so the caller
            // skips this beam outright.
            return (f32::INFINITY, 0.0, 0.0);
        }
        let i0u = i0 as usize;
        let j0u = j0 as usize;
        let fx_frac = cx - j0 as f32;
        let fy_frac = cy - i0 as f32;
        let d00 = field[i0u * w + j0u];
        let d01 = field[i0u * w + j0u + 1];
        let d10 = field[(i0u + 1) * w + j0u];
        let d11 = field[(i0u + 1) * w + j0u + 1];
        let d = (1.0 - fx_frac) * (1.0 - fy_frac) * d00
            + fx_frac * (1.0 - fy_frac) * d01
            + (1.0 - fx_frac) * fy_frac * d10
            + fx_frac * fy_frac * d11;
        let dd_dx = ((d01 - d00) * (1.0 - fy_frac) + (d11 - d10) * fy_frac) * cell_inv;
        let dd_dy = ((d10 - d00) * (1.0 - fx_frac) + (d11 - d01) * fx_frac) * cell_inv;
        (d, dd_dx, dd_dy)
    };

    let (mut x, mut y, mut yaw) = initial_pose;
    let mut iterations = 0u32;
    let mut converged = false;

    for iter in 0..cfg.max_iters {
        iterations = iter + 1;
        // 3x3 Hessian + 3x1 gradient (J^T·J  and  J^T·r).
        let mut hm = [[0.0_f32; 3]; 3];
        let mut gv = [0.0_f32; 3];
        let mut n_used = 0u32;

        let (sin_t, cos_t) = yaw.sin_cos();
        for &(bx, by) in &endpoints {
            let ex = x + cos_t * bx - sin_t * by;
            let ey = y + sin_t * bx + cos_t * by;
            let (d, dd_dx, dd_dy) = sample(ex, ey);
            // Skip beams whose endpoint is too far from any wall to be
            // informative. Previously we saturated the residual with
            // `d.min(sigma_m)` while still using the full Jacobian —
            // that's the bug from v1: far-from-wall beams kept tugging
            // the pose toward an irrelevant nearest wall. Outright
            // skipping is the proper Huber-rejection equivalent for our
            // distance-field cost.
            if !d.is_finite() || d > cfg.sigma_m {
                continue;
            }
            n_used += 1;
            // Jacobian of d w.r.t. (x, y, yaw). The endpoint is
            // p + R(yaw)·e_b, so ∂e/∂yaw = R'(yaw)·e_b.
            let dex_dyaw = -sin_t * bx - cos_t * by;
            let dey_dyaw = cos_t * bx - sin_t * by;
            let j0 = dd_dx;
            let j1 = dd_dy;
            let j2 = dd_dx * dex_dyaw + dd_dy * dey_dyaw;
            let js = [j0, j1, j2];
            for a in 0..3 {
                for b in 0..3 {
                    hm[a][b] += js[a] * js[b];
                }
                gv[a] += js[a] * d;
            }
        }
        if n_used == 0 {
            break;
        }

        // Gaussian pose-prior regularizer: adds (1/σ²) * Δpose² to the
        // *optimization* cost (diagonal + gradient terms). Deliberately
        // NOT counted in the reported residual — the prior terms are
        // dimensionless (σ-normalized) and grow with how far the matcher
        // moved from its seed, so mixing them into a "metres" residual
        // made loop-closure gates reject exactly the closures that
        // correct meaningful drift.
        if let Some((px, py, pyaw)) = prior_pose {
            if cfg.prior_sigma_xy > 0.0 {
                let inv2 = 1.0 / (cfg.prior_sigma_xy * cfg.prior_sigma_xy);
                let dx = x - px;
                let dy = y - py;
                hm[0][0] += inv2;
                hm[1][1] += inv2;
                gv[0] += inv2 * dx;
                gv[1] += inv2 * dy;
            }
            if cfg.prior_sigma_yaw > 0.0 {
                let inv2 = 1.0 / (cfg.prior_sigma_yaw * cfg.prior_sigma_yaw);
                let dyaw = wrap_pi(yaw - pyaw);
                hm[2][2] += inv2;
                gv[2] += inv2 * dyaw;
            }
        }

        // Levenberg damping.
        hm[0][0] += cfg.lambda;
        hm[1][1] += cfg.lambda;
        hm[2][2] += cfg.lambda;

        let delta = match solve_3x3(&hm, &gv) {
            Some(d) => [-d[0], -d[1], -d[2]],
            None => break, // singular Hessian, give up
        };
        x += delta[0];
        y += delta[1];
        yaw = wrap_pi(yaw + delta[2]);

        if delta[0].abs() < cfg.eps_translation_m
            && delta[1].abs() < cfg.eps_translation_m
            && delta[2].abs() < cfg.eps_rotation_rad
        {
            converged = true;
            break;
        }
    }

    // Score the FINAL pose (the one we return), not the pose one
    // Gauss-Newton step behind it — the old code reported the residual
    // computed before the last delta was applied.
    let mut residual_sum_sq = 0.0_f32;
    let mut n_used = 0u32;
    let mut n_observed = 0u32;
    let (sin_t, cos_t) = yaw.sin_cos();
    for &(bx, by) in &endpoints {
        let ex = x + cos_t * bx - sin_t * by;
        let ey = y + sin_t * bx + cos_t * by;
        let (d, _, _) = sample(ex, ey);
        // `sample` answers +∞ exactly when the endpoint left the grid.
        if !d.is_finite() {
            continue;
        }
        // Observed = the submap has an opinion about this cell at all
        // (|log| ≥ 50 is half a single hit/miss). A separate METRIC, not a
        // filter on the residual: the Gauss-Newton loop above optimizes
        // every finite beam within sigma, and a score computed over a
        // stricter subset than was optimized deflated n_beams_used under
        // the loop closer's gates — good closures bounced for it.
        if let Some((i, j)) = grid.world_to_idx(ex, ey)
            && grid.log_at(i, j).unsigned_abs() >= 50
        {
            n_observed += 1;
        }
        if d > cfg.sigma_m {
            continue;
        }
        residual_sum_sq += d * d;
        n_used += 1;
    }
    let residual_m = if n_used > 0 {
        (residual_sum_sq / n_used as f32).sqrt()
    } else {
        f32::INFINITY
    };

    ScanMatchResult {
        pose: (x, y, yaw),
        residual_m,
        iterations,
        converged,
        n_beams_used: n_used,
        n_beams_valid,
        n_beams_observed: n_observed,
    }
}

fn solve_3x3(a: &[[f32; 3]; 3], b: &[f32; 3]) -> Option<[f32; 3]> {
    let det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    if det.abs() < 1e-9 {
        return None;
    }
    let inv_det = 1.0 / det;
    let inv = [
        [
            (a[1][1] * a[2][2] - a[1][2] * a[2][1]) * inv_det,
            (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * inv_det,
            (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * inv_det,
        ],
        [
            (a[1][2] * a[2][0] - a[1][0] * a[2][2]) * inv_det,
            (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * inv_det,
            (a[0][2] * a[1][0] - a[0][0] * a[1][2]) * inv_det,
        ],
        [
            (a[1][0] * a[2][1] - a[1][1] * a[2][0]) * inv_det,
            (a[0][1] * a[2][0] - a[0][0] * a[2][1]) * inv_det,
            (a[0][0] * a[1][1] - a[0][1] * a[1][0]) * inv_det,
        ],
    ];
    Some([
        inv[0][0] * b[0] + inv[0][1] * b[1] + inv[0][2] * b[2],
        inv[1][0] * b[0] + inv[1][1] * b[1] + inv[1][2] * b[2],
        inv[2][0] * b[0] + inv[2][1] * b[1] + inv[2][2] * b[2],
    ])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::{GridConfig, OccupancyGrid};

    fn make_room() -> OccupancyGrid {
        let mut g = OccupancyGrid::new(GridConfig {
            x_range: (-2.5, 2.5),
            y_range: (-2.5, 2.5),
            cell: 0.05,
        });
        let n = 200;
        for _ in 0..6 {
            for i in 0..n {
                let t = -2.0 + 4.0 * (i as f32 / (n - 1) as f32);
                g.integrate_ray(0.0, 0.0, t, 2.0, true);
                g.integrate_ray(0.0, 0.0, t, -2.0, true);
                g.integrate_ray(0.0, 0.0, 2.0, t, true);
                g.integrate_ray(0.0, 0.0, -2.0, t, true);
            }
            // Asymmetric divider so yaw is well constrained.
            for i in 0..n / 2 {
                let t = 2.0 * (i as f32 / ((n / 2) as f32));
                g.integrate_ray(0.0, 0.0, t, 0.5, true);
            }
        }
        g
    }

    fn scan_at(grid: &mut OccupancyGrid, pose: (f32, f32, f32)) -> Scan {
        let mut a = Vec::new();
        let mut r = Vec::new();
        for k in 0..64 {
            let aa = -std::f32::consts::PI + (k as f32) * (2.0 * std::f32::consts::PI / 64.0);
            let rr = grid.cast_ray(pose.0, pose.1, pose.2 + aa, 4.0);
            a.push(aa);
            r.push(rr);
        }
        Scan::from_polar(&a, &r, (0.0, 0.0), 1e-6)
    }

    /// F1 regression: the reported residual must measure beam geometry
    /// at the final pose ONLY. The old code summed the σ-normalized
    /// pose-prior penalty into it, so a match that had to move away
    /// from its (drifted) prior reported a large "residual" even when
    /// the beams fit perfectly — and loop-closure gates rejected it.
    #[test]
    fn residual_excludes_prior_penalty() {
        let mut grid = make_room();
        let truth = (0.4_f32, -0.3_f32, 0.2_f32);
        let scan = scan_at(&mut grid, truth);
        // Seed AND prior 0.2 m / 0.1 rad away from truth — the matcher
        // must walk back to truth against the prior.
        let drifted = (truth.0 + 0.15, truth.1 - 0.12, truth.2 + 0.10);
        let cfg = ScanMatchConfig::default();
        let res = match_scan(&mut grid, &scan, drifted, Some(drifted), &cfg);
        let dx = res.pose.0 - truth.0;
        let dy = res.pose.1 - truth.1;
        assert!(
            (dx * dx + dy * dy).sqrt() < 0.10,
            "matcher failed to recover truth: {:?} vs {:?}",
            res.pose,
            truth
        );
        // The old contaminated residual here was ~sqrt(prior/n) ≈ 0.1+;
        // the beam-only residual at a converged pose is a few cm.
        assert!(
            res.residual_m < 0.06,
            "residual contaminated? got {:.3} m",
            res.residual_m
        );
        assert!(res.n_beams_valid >= res.n_beams_used);
        assert!(res.n_beams_used > 16);
    }
}
