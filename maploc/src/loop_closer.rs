//! LoopCloser — detect submap-to-submap loop closures via scan matching.
//!
//! Triggered after a submap closes. For each older submap whose anchor
//! is within `radius_m` of the new submap's anchor (and not the
//! immediate predecessor — that's already captured by the odom edge),
//! match several of the new submap's stored raw scans against the older
//! submap's grid (coarse correlative search, then Gauss-Newton refine).
//!
//! A closure is emitted only when it survives ALL of:
//!   * per-scan gates — final-pose residual ≤ `max_residual_m`,
//!     ≥ `min_beams_used` beams, and ≥ `min_coverage` of the scan's
//!     valid beams landing near mapped walls (an aliased pose can score
//!     a great residual on a small agreeing subset while most beams
//!     disagree);
//!   * cross-scan consistency — every verified scan must imply the same
//!     corrected anchor within `verify_max_spread_m` / `_rad`. One
//!     45°-FOV scan is far too weak a witness on its own: a single
//!     aliased match warps the whole graph;
//!   * a correction floor — edges that agree with the current graph
//!     estimate within `min_correction_m` / `_rad` are dropped. They
//!     cannot fix anything; they only inject scan-match noise into the
//!     optimizer, which shows up as wavy walls when submaps freeze
//!     every few seconds in a small room.

use crate::pose_graph::{between, compose, inverse, wrap_pi};
use crate::scan_matcher::{ScanMatchConfig, match_scan};
use crate::submap::{Pose2, Submap};

#[derive(Debug, Clone, Copy)]
pub struct LoopCloserConfig {
    /// Spatial proximity gate (metres): only consider older submaps
    /// within this radius of the new anchor.
    pub radius_m: f32,
    /// Don't try matching against this many submaps directly preceding
    /// the new one (their odom edges already constrain things and
    /// short-range loops are noisy).
    pub min_index_gap: usize,
    /// Per-beam RMS residual upper bound (metres) for accepting a
    /// match as a real loop closure.
    pub max_residual_m: f32,
    /// At least this many beams must have contributed to the match
    /// for it to be trustworthy.
    pub min_beams_used: u32,
    /// Scan matcher hyperparameters used for the actual match.
    pub sm: ScanMatchConfig,
    /// Coarse correlative pre-search window around the odometry seed.
    /// The Gauss-Newton matcher's basin of attraction is < `sm.sigma_m`
    /// (~0.3 m) — beams farther than that from any wall are skipped, so
    /// GN alone physically cannot recover drift beyond it. The coarse
    /// grid search finds the basin first; GN then refines inside it.
    pub coarse_radius_m: f32,
    pub coarse_step_m: f32,
    pub coarse_yaw_half_rad: f32,
    pub coarse_yaw_step_rad: f32,
    /// Per-axis sigmas for the loop edge's information matrix. These are
    /// the *floor*; the caller should widen them with the match residual
    /// (a 9 cm match is not known to 5 cm).
    pub edge_sigma_xy: f32,
    pub edge_sigma_yaw: f32,
    /// Number of stored raw scans matched independently per candidate
    /// (capped by how many the submap retained). All of them must pass
    /// the per-scan gates AND agree on the implied anchor.
    pub verify_scans: usize,
    /// A witness must carry at least this many beams to testify. The
    /// accumulator's composites run to thousands; a 12-beam scrap that
    /// aliases somewhere plausible would otherwise veto the consensus the
    /// strong witnesses agree on.
    pub min_witness_beams: usize,
    /// Max deviation of any verified scan's implied anchor from the
    /// consensus (translation / yaw).
    pub verify_max_spread_m: f32,
    pub verify_max_spread_rad: f32,
    /// Minimum `n_beams_used / n_beams_valid` at the final matched pose.
    pub min_coverage: f32,
    /// Witness scans are decimated to at most this many beams before
    /// matching — cost is O(search cells × beams) per witness per freeze.
    pub max_probe_beams: usize,
    /// Drop closures whose correction relative to the current graph
    /// estimate is below this (translation AND yaw) — nothing to fix.
    pub min_correction_m: f32,
    pub min_correction_rad: f32,
    /// And drop closures whose correction is IMPLAUSIBLY LARGE for the
    /// number of submaps between the two: odometry drifts a few percent of
    /// distance travelled, so a "correction" far beyond what could have
    /// accumulated is an aliased match — accepting one folds distinct rooms
    /// onto each other (observed on recorded sessions). Allowed correction
    /// = `base + per_submap × index gap`, capped at `cap`.
    pub max_correction_base_m: f32,
    pub max_correction_per_submap_m: f32,
    pub max_correction_cap_m: f32,
    pub max_correction_base_rad: f32,
    pub max_correction_per_submap_rad: f32,
    pub max_correction_cap_rad: f32,
    /// If true, print one line per rejection (residual / beams) so we
    /// can debug "why did no loop close".
    pub verbose: bool,
}

impl Default for LoopCloserConfig {
    fn default() -> Self {
        // The prior is re-anchored at the coarse-search winner (a
        // geometry-derived pose), so it stabilizes weakly-constrained
        // DoF without dragging the match back toward odometry drift.
        // Wide sigmas keep that stabilizing role gentle.
        let sm = ScanMatchConfig {
            prior_sigma_xy: 0.50,
            prior_sigma_yaw: 0.35,
            ..ScanMatchConfig::default()
        };
        Self {
            radius_m: 1.5,
            min_index_gap: 2,
            max_residual_m: 0.10, // 10 cm — tighter would reject good closes
            min_beams_used: 16,
            sm,
            coarse_radius_m: 0.50,
            coarse_step_m: 0.10,
            coarse_yaw_half_rad: 20.0_f32.to_radians(),
            coarse_yaw_step_rad: 5.0_f32.to_radians(),
            edge_sigma_xy: 0.05,
            edge_sigma_yaw: 0.03,
            verify_scans: 3,
            min_witness_beams: 150,
            verify_max_spread_m: 0.06,
            verify_max_spread_rad: 0.05, // ~3°
            min_coverage: 0.40,
            max_probe_beams: 512,
            min_correction_m: 0.04,
            min_correction_rad: 0.03,
            max_correction_base_m: 0.06,
            max_correction_per_submap_m: 0.08,
            max_correction_cap_m: 0.60,
            max_correction_base_rad: 0.05,
            max_correction_per_submap_rad: 0.03,
            max_correction_cap_rad: 0.45,
            verbose: false,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct LoopClosure {
    pub from_idx: usize,
    pub to_idx: usize,
    /// Relative pose from `from_idx`'s anchor to `to_idx`'s anchor as
    /// inferred by the scan match (i.e. the loop edge's measurement).
    pub measurement: Pose2,
    pub residual_m: f32,
    pub n_beams_used: u32,
}

/// Try to detect loop closures for a freshly-closed submap.
/// `submaps[new_idx]` is the new submap. We scan its first stored raw
/// scan against each candidate older submap's grid.
pub fn detect_loops(
    submaps: &mut [Submap],
    new_idx: usize,
    cfg: &LoopCloserConfig,
) -> Vec<LoopClosure> {
    let n = submaps.len();
    if new_idx == 0 || new_idx >= n {
        return Vec::new();
    }

    // Snapshot what we need from the new submap so the borrow is
    // released before we mutably touch older submaps' grids.
    let new_anchor = submaps[new_idx].anchor_pose();
    let scans = submaps[new_idx].raw_scans().to_vec();
    if scans.is_empty() {
        return Vec::new();
    }

    let mut out = Vec::new();
    for older_idx in 0..n {
        if older_idx == new_idx {
            continue;
        }
        if older_idx + cfg.min_index_gap >= new_idx {
            continue;
        }

        // Spatial gate.
        let older_anchor = submaps[older_idx].anchor_pose();
        let dx = new_anchor.0 - older_anchor.0;
        let dy = new_anchor.1 - older_anchor.1;
        if (dx * dx + dy * dy).sqrt() > cfg.radius_m {
            continue;
        }

        // Pick the strongest witnesses: the largest scans first, and only
        // ones carrying enough beams to mean something. `pose_in_submap`
        // differs per scan, so a stable TRUE match makes them all imply the
        // same corrected anchor; an aliased match doesn't. When only one
        // strong witness exists, it testifies alone — under a stricter
        // residual gate below, because nobody cross-examines it.
        let mut by_size: Vec<&crate::submap::RawScan> = scans.iter().collect();
        by_size.sort_by_key(|s| std::cmp::Reverse(s.scan.n_valid()));
        let mut picked: Vec<&crate::submap::RawScan> = Vec::new();
        for s in by_size {
            if s.scan.n_valid() < cfg.min_witness_beams {
                continue;
            }
            // Two witnesses must be two OBSERVATIONS: sessions written
            // before scans were stored once per window carry byte-identical
            // duplicates, and a duplicate in the second slot cross-examines
            // a scan against itself.
            if picked
                .iter()
                .any(|p| p.pose_in_submap == s.pose_in_submap && p.scan.beams == s.scan.beams)
            {
                continue;
            }
            picked.push(s);
            if picked.len() >= cfg.verify_scans.max(1) {
                break;
            }
        }
        if picked.len() < 2 {
            // A closure warps every anchor in the graph; one witness —
            // however large — is not enough to swear that in. Measured on
            // recorded sessions: lone-witness closures were the wrong ones.
            continue;
        }

        // Match each picked scan: coarse-to-fine — grid-search a window
        // around the drifted seed to land inside the GN basin, then
        // refine from (and prior to) the coarse winner rather than the
        // drifted odometry estimate.
        let mut anchors: Vec<Pose2> = Vec::with_capacity(picked.len());
        let mut worst_residual = 0.0_f32;
        let mut min_beams = u32::MAX;
        let mut all_pass = true;
        let max_residual = cfg.max_residual_m;
        for scan in &picked {
            let pose_world = compose(new_anchor, scan.pose_in_submap);
            let pose_in_older = between(older_anchor, pose_world);
            // A window composite can carry thousands of beams; both the
            // coarse grid search and the refinement position just as well
            // on a few hundred, at a tenth of the lookups per freeze.
            let probe = scan.scan.decimated(cfg.max_probe_beams);
            let seed = coarse_search(submaps[older_idx].grid_mut(), &probe, pose_in_older, cfg);
            let result = match_scan(
                submaps[older_idx].grid_mut(),
                &probe,
                seed,
                Some(seed),
                &cfg.sm,
            );
            // Coverage judges by beams landing where the TARGET has an
            // opinion — n_beams_observed, not n_beams_used: the used count
            // includes near-wall beams in never-swept cells.
            let coverage = if result.n_beams_valid > 0 {
                result.n_beams_observed as f32 / result.n_beams_valid as f32
            } else {
                0.0
            };
            if cfg.verbose {
                eprintln!(
                    "[loop-try] {} → {}  resid={:.3}  beams={}/{}  iters={}",
                    older_idx,
                    new_idx,
                    result.residual_m,
                    result.n_beams_used,
                    result.n_beams_valid,
                    result.iterations
                );
            }
            if !result.residual_m.is_finite()
                || result.residual_m > max_residual
                || result.n_beams_used < cfg.min_beams_used
                || coverage < cfg.min_coverage
            {
                all_pass = false;
                break;
            }
            worst_residual = worst_residual.max(result.residual_m);
            min_beams = min_beams.min(result.n_beams_used);
            // Corrected anchor for the new submap implied by this scan.
            let corrected_world = compose(older_anchor, result.pose);
            anchors.push(compose(corrected_world, inverse(scan.pose_in_submap)));
        }
        if !all_pass {
            continue;
        }

        // Cross-scan consistency: every implied anchor must sit near the
        // consensus (mean position, circular-mean yaw).
        let n_f = anchors.len() as f32;
        let mx = anchors.iter().map(|a| a.0).sum::<f32>() / n_f;
        let my = anchors.iter().map(|a| a.1).sum::<f32>() / n_f;
        let myaw = {
            let s: f32 = anchors.iter().map(|a| a.2.sin()).sum();
            let c: f32 = anchors.iter().map(|a| a.2.cos()).sum();
            s.atan2(c)
        };
        let consistent = anchors.iter().all(|a| {
            let d = ((a.0 - mx).powi(2) + (a.1 - my).powi(2)).sqrt();
            d <= cfg.verify_max_spread_m && wrap_pi(a.2 - myaw).abs() <= cfg.verify_max_spread_rad
        });
        if !consistent {
            if cfg.verbose {
                eprintln!(
                    "[loop-try] {} → {}  REJECTED: scans disagree \
                           on the corrected anchor ({anchors:?})",
                    older_idx, new_idx
                );
            }
            continue;
        }

        let corrected_new_anchor = (mx, my, myaw);
        let measurement = between(older_anchor, corrected_new_anchor);

        // Correction floor: if the closure just re-states the current
        // estimate, it can't fix drift — it only injects scan noise
        // into the optimizer at every submap freeze.
        let z_pred = between(older_anchor, new_anchor);
        let corr_xy =
            ((measurement.0 - z_pred.0).powi(2) + (measurement.1 - z_pred.1).powi(2)).sqrt();
        let corr_yaw = wrap_pi(measurement.2 - z_pred.2).abs();
        if corr_xy < cfg.min_correction_m && corr_yaw < cfg.min_correction_rad {
            if cfg.verbose {
                eprintln!(
                    "[loop-try] {} → {}  skipped: correction \
                           ({corr_xy:.3} m, {:.1}°) below floor",
                    older_idx,
                    new_idx,
                    corr_yaw.to_degrees()
                );
            }
            continue;
        }
        let gap = (new_idx - older_idx) as f32;
        let allow_xy = (cfg.max_correction_base_m + cfg.max_correction_per_submap_m * gap)
            .min(cfg.max_correction_cap_m);
        let allow_yaw = (cfg.max_correction_base_rad + cfg.max_correction_per_submap_rad * gap)
            .min(cfg.max_correction_cap_rad);
        if corr_xy > allow_xy || corr_yaw > allow_yaw {
            if cfg.verbose {
                eprintln!(
                    "[loop-try] {} → {}  REJECTED: correction ({corr_xy:.3} m, {:.1}°) \
                     implausible for a {gap:.0}-submap gap",
                    older_idx,
                    new_idx,
                    corr_yaw.to_degrees()
                );
            }
            continue;
        }

        out.push(LoopClosure {
            from_idx: older_idx,
            to_idx: new_idx,
            measurement,
            residual_m: worst_residual,
            n_beams_used: min_beams,
        });
    }
    out
}

/// Correlative pre-search: score a small (dx, dy, dyaw) window around
/// `seed` against `grid`'s distance field and return the best pose.
/// Score = mean per-beam distance-to-wall, clamped at `sm.sigma_m` so
/// out-of-map beams don't dominate. Per-yaw beam-endpoint offsets are
/// precomputed, so the inner loop is add + lookup only (~30k lookups on
/// the default window — trivial at submap-close cadence).
fn coarse_search(
    grid: &mut crate::grid::OccupancyGrid,
    scan: &crate::submap::Scan,
    seed: Pose2,
    cfg: &LoopCloserConfig,
) -> Pose2 {
    let field = grid.distance_field_shared(cfg.sm.occ_threshold_fp);
    let g = *grid.cfg();
    let (w, h) = (grid.width(), grid.height());
    let clamp = cfg.sm.sigma_m;

    let step = cfg.coarse_step_m.max(g.cell);
    let n_xy = (cfg.coarse_radius_m / step).round() as i32;
    let yaw_step = cfg.coarse_yaw_step_rad.max(1e-3);
    let n_yaw = (cfg.coarse_yaw_half_rad / yaw_step).round() as i32;

    let valid: Vec<(f32, f32)> = scan.endpoints_body().collect();
    if valid.is_empty() {
        return seed;
    }

    let mut best = seed;
    let mut best_score = f32::INFINITY;
    for yi in -n_yaw..=n_yaw {
        let yaw = seed.2 + yi as f32 * yaw_step;
        // Beam-endpoint offsets are constant across (dx, dy) at fixed yaw.
        let (sy, cyw) = yaw.sin_cos();
        let offsets: Vec<(f32, f32)> = valid
            .iter()
            .map(|&(bx, by)| (cyw * bx - sy * by, sy * bx + cyw * by))
            .collect();
        for dyi in -n_xy..=n_xy {
            for dxi in -n_xy..=n_xy {
                let cx = seed.0 + dxi as f32 * step;
                let cy = seed.1 + dyi as f32 * step;
                let mut sum = 0.0_f32;
                for &(ox, oy) in &offsets {
                    let (i, j) = grid.world_to_cell(cx + ox, cy + oy);
                    let d = if i < 0 || j < 0 || (i as usize) >= h || (j as usize) >= w {
                        clamp
                    } else {
                        field[(i as usize) * w + (j as usize)].min(clamp)
                    };
                    sum += d;
                }
                let score = sum / valid.len() as f32;
                if score < best_score {
                    best_score = score;
                    best = (cx, cy, yaw);
                }
            }
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::GridConfig;
    use crate::submap::Scan;

    fn sc(angles: &[f32], ranges: &[f32]) -> Scan {
        Scan::from_polar(angles, ranges, (0.0, 0.0), 1e-6)
    }

    #[test]
    fn empty_or_single_submap_returns_no_loops() {
        let mut submaps = Vec::<Submap>::new();
        assert!(detect_loops(&mut submaps, 0, &LoopCloserConfig::default()).is_empty());
        let cfg = GridConfig::default();
        let mut s = vec![Submap::new_at((0.0, 0.0, 0.0), cfg)];
        assert!(detect_loops(&mut s, 0, &LoopCloserConfig::default()).is_empty());
    }

    /// End-to-end drift-recovery: a new submap whose anchor drifted
    /// 0.4 m from truth must still close against the older submap of
    /// the same room, and the emitted measurement must recover the true
    /// relative pose. Exercises three fixes at once:
    ///   * coarse-to-fine search (0.4 m > the GN basin of ~sigma_m);
    ///   * prior re-anchored at the coarse winner (not the drifted seed);
    ///   * beam-only residual (the old prior-contaminated residual
    ///     inflated past the 0.10 m gate on exactly this kind of match).
    #[test]
    fn loop_closure_recovers_drift_beyond_gn_basin() {
        let grid_cfg = GridConfig {
            x_range: (-2.5, 2.5),
            y_range: (-2.5, 2.5),
            cell: 0.05,
        };
        // Older submap A at world origin: paint an asymmetric room
        // (perimeter walls + divider) by casting from several origins.
        let mut a = Submap::new_at((0.0, 0.0, 0.0), grid_cfg);
        let n = 160;
        let mut walls: Vec<(f32, f32)> = Vec::new();
        for i in 0..n {
            let t = -2.0 + 4.0 * (i as f32 / (n - 1) as f32);
            walls.push((t, 2.0));
            walls.push((t, -2.0));
            walls.push((2.0, t));
            walls.push((-2.0, t));
        }
        for i in 0..n / 2 {
            let t = 2.0 * (i as f32 / ((n / 2) as f32));
            walls.push((t, 0.5));
        }
        for &(ox, oy) in &[(0.0_f32, 0.0_f32), (0.5, -0.5), (-0.5, 0.8), (0.3, 0.2)] {
            for _ in 0..4 {
                for &(wx, wy) in &walls {
                    let dx = wx - ox;
                    let dy = wy - oy;
                    a.integrate_scan(
                        (ox, oy, 0.0),
                        &sc(&[dy.atan2(dx)], &[(dx * dx + dy * dy).sqrt()]),
                    );
                }
            }
        }

        // Ground truth: the duck is at world (0.3, 0.2, 0). Odometry
        // drifted by a constant (+0.4, 0) offset, so the new submap B is
        // anchored at (0.7, 0.2, 0). Store THREE raw scans from slightly
        // different poses — the verification layer matches each
        // independently and requires them to agree on the corrected
        // anchor. Each scan is what the duck REALLY saw (ray-cast
        // against A's grid at the true pose) but is filed under the
        // drifted pose, exactly like real odometry drift.
        let truth = (0.3_f32, 0.2_f32, 0.0_f32);
        let drift = (0.4_f32, 0.0_f32);
        let drifted_anchor = (truth.0 + drift.0, truth.1 + drift.1, truth.2);
        let mut b = Submap::new_at(drifted_anchor, grid_cfg);
        for &(dx, dy, dyaw) in &[
            (0.0_f32, 0.0_f32, 0.0_f32),
            (0.06, 0.04, 0.05),
            (-0.05, 0.06, -0.04),
        ] {
            let true_pose = (truth.0 + dx, truth.1 + dy, truth.2 + dyaw);
            let filed_pose = (true_pose.0 + drift.0, true_pose.1 + drift.1, true_pose.2);
            let mut angles = Vec::new();
            let mut ranges = Vec::new();
            for k in 0..64 {
                let aa = -std::f32::consts::PI + (k as f32) * (2.0 * std::f32::consts::PI / 64.0);
                let rr = a
                    .grid()
                    .cast_ray(true_pose.0, true_pose.1, true_pose.2 + aa, 4.0);
                angles.push(aa);
                ranges.push(rr);
            }
            b.integrate_scan(filed_pose, &sc(&angles, &ranges));
        }

        // Two far-away fillers to satisfy min_index_gap without adding
        // candidates (outside radius_m).
        let far1 = Submap::new_at((50.0, 0.0, 0.0), grid_cfg);
        let far2 = Submap::new_at((60.0, 0.0, 0.0), grid_cfg);

        let mut submaps = vec![a, far1, far2, b];
        // The test scans are single 64-beam raycasts; the witness gate is
        // tuned for the accumulator's composites, so admit them here. And
        // this scenario's whole point is a 0.4 m drift over a 3-submap gap —
        // beyond the default plausibility budget on purpose.
        let cfg = LoopCloserConfig {
            min_witness_beams: 32,
            max_correction_per_submap_m: 0.15,
            ..LoopCloserConfig::default()
        };
        let loops = detect_loops(&mut submaps, 3, &cfg);
        assert_eq!(
            loops.len(),
            1,
            "expected exactly one closure, got {loops:?}"
        );
        let lc = &loops[0];
        assert_eq!((lc.from_idx, lc.to_idx), (0, 3));
        // Measurement = corrected B anchor in A's frame ≈ truth.
        let (mx, my, myaw) = lc.measurement;
        let err = ((mx - truth.0).powi(2) + (my - truth.1).powi(2)).sqrt();
        assert!(
            err < 0.10,
            "closure measurement ({mx:.2},{my:.2}) is {err:.2} m off truth"
        );
        assert!(
            myaw.abs() < 0.12,
            "closure yaw {:.1}° off truth",
            myaw.to_degrees()
        );
        assert!(
            lc.residual_m < 0.06,
            "beam-only residual should be small, got {:.3}",
            lc.residual_m
        );
    }

    /// Correction-floor regression: a candidate whose match AGREES with
    /// the current graph estimate must be skipped, not emitted. Emitting
    /// it re-states what odometry already says, at scan-match noise
    /// level — with submaps freezing every few seconds in one room,
    /// those noise edges made the optimizer jiggle every anchor at each
    /// freeze and the rendered walls came out wavy.
    #[test]
    fn agreeing_closure_is_skipped() {
        let grid_cfg = GridConfig {
            x_range: (-2.5, 2.5),
            y_range: (-2.5, 2.5),
            cell: 0.05,
        };
        let mut a = Submap::new_at((0.0, 0.0, 0.0), grid_cfg);
        let n = 160;
        let mut walls: Vec<(f32, f32)> = Vec::new();
        for i in 0..n {
            let t = -2.0 + 4.0 * (i as f32 / (n - 1) as f32);
            walls.push((t, 2.0));
            walls.push((t, -2.0));
            walls.push((2.0, t));
            walls.push((-2.0, t));
        }
        for i in 0..n / 2 {
            let t = 2.0 * (i as f32 / ((n / 2) as f32));
            walls.push((t, 0.5));
        }
        for &(ox, oy) in &[(0.0_f32, 0.0_f32), (0.5, -0.5), (-0.5, 0.8), (0.3, 0.2)] {
            for _ in 0..4 {
                for &(wx, wy) in &walls {
                    let dx = wx - ox;
                    let dy = wy - oy;
                    a.integrate_scan(
                        (ox, oy, 0.0),
                        &sc(&[dy.atan2(dx)], &[(dx * dx + dy * dy).sqrt()]),
                    );
                }
            }
        }

        // NO drift this time: B's anchor is exactly where the duck is.
        let pose = (0.3_f32, 0.2_f32, 0.0_f32);
        let mut b = Submap::new_at(pose, grid_cfg);
        let mut angles = Vec::new();
        let mut ranges = Vec::new();
        for k in 0..64 {
            let aa = -std::f32::consts::PI + (k as f32) * (2.0 * std::f32::consts::PI / 64.0);
            let rr = a.grid().cast_ray(pose.0, pose.1, pose.2 + aa, 4.0);
            angles.push(aa);
            ranges.push(rr);
        }
        b.integrate_scan(pose, &sc(&angles, &ranges));

        let far1 = Submap::new_at((50.0, 0.0, 0.0), grid_cfg);
        let far2 = Submap::new_at((60.0, 0.0, 0.0), grid_cfg);
        let mut submaps = vec![a, far1, far2, b];
        // The test scans are single 64-beam raycasts; the witness gate is
        // tuned for the accumulator's composites, so admit them here.
        let cfg = LoopCloserConfig {
            min_witness_beams: 32,
            ..LoopCloserConfig::default()
        };
        let loops = detect_loops(&mut submaps, 3, &cfg);
        assert!(
            loops.is_empty(),
            "agreeing closure must be skipped by the correction floor, \
                 got {loops:?}"
        );
    }
}
