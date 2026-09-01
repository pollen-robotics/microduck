//! Particle-filter (MCL) relocalize against a saved global grid.
//!
//! Consumes one ToF scan per call, narrows a cloud of (x, y, yaw)
//! hypotheses over a few seconds. The runtime wires it under
//! `pending_relocalize`: while the cloud is spread, no submap ingestion;
//! once it collapses (and stays collapsed for a few frames), the runtime
//! snaps `tracked` to `best()` and resumes regular SLAM.
//!
//! Sensor model: per-beam residual against the grid's distance field,
//! Gaussian likelihood with `beam_sigma_m`, residuals clamped at
//! `beam_clamp_m` so a single bad beam can't flatten the weight.
//!
//! Motion model: odometry-driven body-frame translation + yaw delta with
//! Gaussian noise scaled by travelled distance + |yaw delta|. Same
//! convention as v1's MCL.
//!
//! Resampling: low-variance (systematic) sampler triggered when the
//! effective sample size falls below `resample_ess_frac * N`.

use crate::grid::OccupancyGrid;
use crate::pose_graph::wrap_pi;
use crate::rng::Rng;
use crate::submap::{Pose2, Scan};

#[derive(Debug, Clone)]
pub struct MclConfig {
    pub n_particles: usize,
    /// Motion-model noise: σ_xy = `sigma_xy_per_m * |Δxy|
    /// + sigma_xy_per_rad * |Δyaw|`. Same for yaw.
    pub sigma_xy_per_m: f32,
    pub sigma_xy_per_rad: f32,
    pub sigma_yaw_per_m: f32,
    pub sigma_yaw_per_rad: f32,
    /// Gaussian std on per-beam distance-field residual.
    pub beam_sigma_m: f32,
    /// Clamp per-beam residual at this (saturation guard).
    pub beam_clamp_m: f32,
    /// Skip the update when fewer than this many beams are valid.
    pub min_beams_used: u32,
    /// Resample when effective sample size < `frac * N`.
    pub resample_ess_frac: f32,
    /// Lock criteria — cloud spread + best-particle residual.
    pub locked_xy_std_m: f32,
    pub locked_yaw_std_rad: f32,
    pub locked_max_residual_m: f32,
    /// Require N consecutive frames meeting the lock criteria.
    pub locked_min_frames: u32,
    /// Tiny "exploration" noise injected on every predict — helps the
    /// cloud not collapse onto a single point too fast on quiet odom.
    pub jitter_xy_m: f32,
    pub jitter_yaw_rad: f32,
    /// Fraction of particles replaced with fresh uniform samples after
    /// each resample. Probes for missed posterior peaks; without this,
    /// a wrong-but-plausible cluster can capture the cloud and never
    /// release it.
    pub random_inject_frac: f32,
    /// Fixed-point log-odds threshold above which a cell counts as a
    /// wall for the distance-field likelihood. `OCC_THRESHOLD` (= 0)
    /// reproduces the previous behaviour; bumping it up filters
    /// transient noise — useful when the saved map is fuzzy. 200 ≈ "a
    /// cell needed 3+ net hits to be a wall".
    pub wall_threshold_fp: i16,
    /// See-through penalties only count walls past this — higher than
    /// `wall_threshold_fp` so one window's phantoms do not read as opaque
    /// (mirrors `RelocalizeConfig::see_through_fp`).
    pub see_through_fp: i16,
    /// Tempering factor for the observation likelihood. log-likelihoods
    /// are multiplied by this before normalising. 1.0 = raw (very peaky
    /// with ~64 beams; the cloud collapses to a single cluster after
    /// one frame, which is usually wrong with a 45° FOV); 0.0 = ignore
    /// the scan entirely. Lower values keep competing modes alive long
    /// enough that subsequent motion can disambiguate. 0.3 is a
    /// reasonable default for ~64-beam VL53L5CX scans.
    pub likelihood_temper: f32,
    /// Skip resampling for the first N updates. Lets weight evidence
    /// accumulate multiplicatively across frames so the cloud doesn't
    /// collapse on frame 1 to whichever wrong cluster fit best. After
    /// the grace period, normal ESS-based resampling resumes.
    pub min_updates_before_resample: u32,
    /// Required fraction of particles within `locked_xy_std_m` of the
    /// weighted-mean pose for the cloud to count as "locked". Robust to
    /// minor multi-modal stragglers: once the dominant cluster has this
    /// fraction, the few outliers don't block lock. Set to 1.0 to fall
    /// back to "all particles tight" behaviour.
    pub locked_dominant_frac: f32,
}

impl Default for MclConfig {
    fn default() -> Self {
        Self {
            n_particles: 800,
            sigma_xy_per_m: 0.10,
            sigma_xy_per_rad: 0.05,
            sigma_yaw_per_m: 0.05,
            sigma_yaw_per_rad: 0.10,
            beam_sigma_m: 0.20,
            beam_clamp_m: 0.50,
            random_inject_frac: 0.10,
            min_beams_used: 16,
            resample_ess_frac: 0.5,
            locked_xy_std_m: 0.15,
            locked_yaw_std_rad: 8.0_f32.to_radians(),
            // Tight residual gate. With a clean v2-threshold map, the
            // correct pose returns residuals < 3 cm; anything above
            // 5 cm is a wrong-cluster fit using map noise as a
            // crutch. Better to never lock than to lock wrong.
            locked_max_residual_m: 0.05,
            // 25 frames at 15 Hz ≈ 1.7 s of consistent narrow cloud.
            // Faster than this and a one-frame fluke can declare lock.
            locked_min_frames: 25,
            jitter_xy_m: 0.005,
            jitter_yaw_rad: 0.005,
            wall_threshold_fp: 200,
            see_through_fp: 300,
            // Soften the posterior. With 64 beams and σ=0.20 m the
            // raw posterior spans ~200 nats; temper=0.30 compresses
            // that to ~60 per frame, but multiplied across 5 grace
            // frames (`min_updates_before_resample`) the effective
            // discrimination is plenty to commit cleanly while still
            // letting competing modes survive past frame 1.
            likelihood_temper: 0.30,
            // Don't resample on frames 1..5 — let weights multiply
            // across updates so the cloud commits only after multiple
            // frames of consistent evidence.
            min_updates_before_resample: 5,
            // 80% of particles in the dominant cluster is enough; the
            // remaining 20% can stragglers from secondary modes.
            locked_dominant_frac: 0.80,
        }
    }
}

pub struct Localizer {
    cfg: MclConfig,
    particles: Vec<Pose2>,
    weights: Vec<f32>,
    rng: Rng,
    /// Last computed best-particle residual (NaN until the first update).
    last_residual_m: f32,
    /// Frames in a row that satisfied the lock criteria.
    ///
    /// NOTE: MCL itself has no motion gate — a stationary duck whose
    /// cloud collapses onto an aliased pose CAN reach `is_locked()`.
    /// The runtime enforces the motion requirement externally (net
    /// world-frame displacement since search start) before accepting a
    /// lock; any other consumer must do the same.
    locked_streak: u32,
    /// Number of `update` calls so far. Used by the
    /// `min_updates_before_resample` grace period.
    update_count: u32,
    /// Dominant-cluster anchor computed by the last `update` (P: avoids
    /// re-binning all particles for each of the `dominant_*` getters).
    /// Invalidated by `predict` and the seeding methods.
    anchor_cache: Option<Pose2>,
}

impl Localizer {
    pub fn new(cfg: MclConfig, seed: u64) -> Self {
        let n = cfg.n_particles.max(1);
        Self {
            cfg,
            particles: vec![(0.0, 0.0, 0.0); n],
            weights: vec![1.0; n],
            rng: Rng::from_seed(seed),
            last_residual_m: f32::NAN,
            locked_streak: 0,
            update_count: 0,
            anchor_cache: None,
        }
    }

    pub fn n_particles(&self) -> usize {
        self.particles.len()
    }
    pub fn last_residual_m(&self) -> f32 {
        self.last_residual_m
    }
    pub fn locked_streak(&self) -> u32 {
        self.locked_streak
    }
    /// Anchor for the dominant-cluster fraction checks. Three steps:
    ///
    ///   1. densest (x, y) bin — coarse, world-axis-aligned;
    ///   2. one mean-shift step: mean of the particles within the lock
    ///      radius of that bin's centre. Without this, a tight cluster
    ///      sitting near a bin *corner* has its anchor ~0.16 m away
    ///      (bin half-diagonal > lock radius 0.15 m) and the ≥80%
    ///      fraction check can never pass — lock stalls forever at
    ///      certain world positions;
    ///   3. yaw = circular mean (sin/cos sums) of those same particles —
    ///      no yaw binning at all, so a cluster straddling ±π doesn't
    ///      split into two half-full bins.
    ///
    /// Cached per `update` (see `anchor_cache`) since the getters below
    /// are typically all polled every frame.
    fn cluster_anchor(&self) -> Pose2 {
        if let Some(a) = self.anchor_cache {
            return a;
        }
        self.compute_cluster_anchor()
    }

    fn compute_cluster_anchor(&self) -> Pose2 {
        // Bin slightly larger than the lock radius so the cluster's bulk
        // lands in one bin even when particles spill near a boundary.
        let bin = self.cfg.locked_xy_std_m * 1.5;
        let (bx, by) = self.densest_xy_bin_center(bin);
        // Mean-shift within the lock radius (position only), then take
        // the circular yaw mean of the same members.
        let r2 = self.cfg.locked_xy_std_m * self.cfg.locked_xy_std_m;
        let (mut x, mut y, mut sy, mut cy) = (0.0_f32, 0.0_f32, 0.0_f32, 0.0_f32);
        let mut n = 0usize;
        for p in &self.particles {
            let dx = p.0 - bx;
            let dy = p.1 - by;
            if dx * dx + dy * dy >= r2 {
                continue;
            }
            x += p.0;
            y += p.1;
            sy += p.2.sin();
            cy += p.2.cos();
            n += 1;
        }
        if n == 0 {
            return (bx, by, 0.0);
        }
        let nf = n as f32;
        (x / nf, y / nf, sy.atan2(cy))
    }

    /// Centre of the densest (x, y) bin. Yaw is deliberately not binned
    /// — see `cluster_anchor`.
    fn densest_xy_bin_center(&self, bin_m: f32) -> (f32, f32) {
        use std::collections::HashMap;
        if self.particles.is_empty() {
            return (0.0, 0.0);
        }
        let inv = 1.0 / bin_m.max(1e-6);
        let mut counts: HashMap<(i32, i32), u32> = HashMap::new();
        for p in &self.particles {
            let i = (p.0 * inv).floor() as i32;
            let j = (p.1 * inv).floor() as i32;
            *counts.entry((i, j)).or_insert(0) += 1;
        }
        // Ties broken by bin coordinates, not HashMap order: RandomState
        // reseeds per process, and the crate's whole reason for carrying
        // its own RNG is that a recorded run replays bit-for-bit.
        let ((bi, bj), _) = counts
            .iter()
            .max_by_key(|(bin, count)| (**count, std::cmp::Reverse(**bin)))
            .unwrap();
        ((*bi as f32 + 0.5) * bin_m, (*bj as f32 + 0.5) * bin_m)
    }

    /// Fraction of particles within `locked_xy_std_m` of the densest
    /// bin's centre — robust to multimodal clouds and to the "best"
    /// particle hopping between similar-weight neighbours.
    pub fn dominant_cluster_frac(&self) -> f32 {
        let a = self.cluster_anchor();
        self.fraction_within(a.0, a.1, self.cfg.locked_xy_std_m)
    }

    /// Heading-axis analogue.
    pub fn dominant_yaw_frac(&self) -> f32 {
        let a = self.cluster_anchor();
        self.fraction_within_yaw(a.2, self.cfg.locked_yaw_std_rad)
    }

    /// Pose to commit when MCL locks: mean of the dominant cluster
    /// (defined as particles within both radii of the densest bin).
    pub fn dominant_cluster_mean(&self) -> Pose2 {
        let a = self.cluster_anchor();
        self.cluster_mean(
            a.0,
            a.1,
            a.2,
            self.cfg.locked_xy_std_m,
            self.cfg.locked_yaw_std_rad,
        )
    }

    /// Spread particles uniformly over `grid`'s free cells with random
    /// yaw. Falls back to (0, 0, *) if the grid has no free cells (e.g.
    /// blank). Sets all weights equal.
    pub fn seed_uniform(&mut self, grid: &OccupancyGrid) {
        let cfg = grid.cfg();
        let cell = cfg.cell;
        let w = grid.width();
        let h = grid.height();
        let mut free: Vec<(f32, f32)> = Vec::with_capacity(w * h / 4);
        for i in 0..h {
            for j in 0..w {
                if grid.is_known_free(i, j) {
                    let cx = cfg.x_range.0 + (j as f32 + 0.5) * cell;
                    let cy = cfg.y_range.0 + (i as f32 + 0.5) * cell;
                    free.push((cx, cy));
                }
            }
        }
        let two_pi = 2.0 * std::f32::consts::PI;
        for p in self.particles.iter_mut() {
            if free.is_empty() {
                let yaw = self.rng.f32() * two_pi - std::f32::consts::PI;
                *p = (0.0, 0.0, yaw);
            } else {
                let (cx, cy) = free[self.rng.index(free.len())];
                let jx = (self.rng.f32() - 0.5) * cell;
                let jy = (self.rng.f32() - 0.5) * cell;
                let yaw = self.rng.f32() * two_pi - std::f32::consts::PI;
                *p = (cx + jx, cy + jy, yaw);
            }
        }
        self.reset_weights();
        self.locked_streak = 0;
        self.last_residual_m = f32::NAN;
        self.update_count = 0;
        self.anchor_cache = None;
    }

    /// Seed particles from a small list of candidate poses (e.g. the
    /// top-K from brute-force search). Particles are split evenly across
    /// seeds with Gaussian noise (σ_xy=`spread_xy_m`, σ_yaw=`spread_yaw_rad`).
    pub fn seed_around(&mut self, seeds: &[Pose2], spread_xy_m: f32, spread_yaw_rad: f32) {
        if seeds.is_empty() {
            return;
        }
        for (i, p) in self.particles.iter_mut().enumerate() {
            let s = seeds[i % seeds.len()];
            *p = (
                s.0 + self.rng.normal(spread_xy_m),
                s.1 + self.rng.normal(spread_xy_m),
                wrap_pi(s.2 + self.rng.normal(spread_yaw_rad)),
            );
        }
        self.reset_weights();
        self.locked_streak = 0;
        self.last_residual_m = f32::NAN;
        self.update_count = 0;
        self.anchor_cache = None;
    }

    /// Mixed seed: a fraction of particles around `seeds` (Gaussian
    /// noise) and the rest uniformly over the grid's free cells. Use
    /// this to combine a brute-force candidate with exploration — if
    /// the brute-force pose is wrong, the uniform cloud still has a
    /// chance to win after a few frames of motion.
    pub fn seed_mixed(
        &mut self,
        seeds: &[Pose2],
        seeds_frac: f32,
        grid: &OccupancyGrid,
        spread_xy_m: f32,
        spread_yaw_rad: f32,
    ) {
        // Start uniform, then overwrite the front `frac * N` particles
        // with seeded ones.
        self.seed_uniform(grid);
        if seeds.is_empty() {
            return;
        }
        let frac = seeds_frac.clamp(0.0, 1.0);
        let n_seed = ((self.particles.len() as f32) * frac) as usize;
        if n_seed == 0 {
            return;
        }
        for i in 0..n_seed {
            let s = seeds[i % seeds.len()];
            self.particles[i] = (
                s.0 + self.rng.normal(spread_xy_m),
                s.1 + self.rng.normal(spread_xy_m),
                wrap_pi(s.2 + self.rng.normal(spread_yaw_rad)),
            );
        }
        // reset_weights / streak / residual already zeroed by seed_uniform.
    }

    /// Apply a body-frame motion delta with noise.
    pub fn predict(&mut self, dx_b: f32, dy_b: f32, dyaw: f32) {
        let trans = (dx_b * dx_b + dy_b * dy_b).sqrt();
        let rot = dyaw.abs();
        self.anchor_cache = None;
        let sigma_xy = self.cfg.sigma_xy_per_m * trans
            + self.cfg.sigma_xy_per_rad * rot
            + self.cfg.jitter_xy_m;
        let sigma_yaw = self.cfg.sigma_yaw_per_m * trans
            + self.cfg.sigma_yaw_per_rad * rot
            + self.cfg.jitter_yaw_rad;
        let sigma_xy = sigma_xy.max(1e-6);
        let sigma_yaw = sigma_yaw.max(1e-6);
        for p in self.particles.iter_mut() {
            // Compose body-frame delta in particle frame.
            let cy = p.2.cos();
            let sy = p.2.sin();
            let dx_w = cy * dx_b - sy * dy_b + self.rng.normal(sigma_xy);
            let dy_w = sy * dx_b + cy * dy_b + self.rng.normal(sigma_xy);
            p.0 += dx_w;
            p.1 += dy_w;
            p.2 = wrap_pi(p.2 + dyaw + self.rng.normal(sigma_yaw));
        }
    }

    /// Reweight particles against a single scan and resample if needed.
    /// Particles are BODY poses; the endpoints include the scan's own
    /// sensor origin, so the filter scores the same geometry the map was
    /// inked with.
    pub fn update(&mut self, grid: &mut OccupancyGrid, scan: &Scan) {
        // Precompute body-frame beam endpoints once per scan. Per
        // particle the endpoint is then a 2D rotation by the particle
        // yaw — 4 mul + 4 add per beam instead of 2 libm trig calls. At
        // 800 particles × 64 beams × 15 Hz the old code was ~1.5 M
        // sin/cos per second, the dominant MCL cost on the A55.
        let beams: Vec<(f32, f32)> = scan.endpoints_body().collect();
        if (beams.len() as u32) < self.cfg.min_beams_used {
            return;
        }

        let field = grid.distance_field_shared(self.cfg.wall_threshold_fp);
        let cfg_g = *grid.cfg();
        let w = grid.width();
        let h = grid.height();
        let cell = cfg_g.cell;
        let x_min = cfg_g.x_range.0;
        let y_min = cfg_g.y_range.0;
        let two_sigma2 = 2.0 * self.cfg.beam_sigma_m * self.cfg.beam_sigma_m;
        let clamp = self.cfg.beam_clamp_m;

        // Clamped distance-to-wall at a scan endpoint for pose (x, y)
        // with precomputed (cos yaw, sin yaw). An endpoint outside the map
        // scores the full clamp — skipping it let poses that throw beams
        // off the map compete on a cherry-picked subset (see
        // `relocalize::score_offsets`, same fix). Floor-based conversion:
        // truncation (`as i32`) aliased a one-cell band outside the
        // min-x/min-y borders onto row/col 0.
        let log: Vec<i16> = grid.log_raw().to_vec();
        let see_through_fp = self.cfg.see_through_fp;
        let endpoint_d = |px: f32, py: f32, cy: f32, sy: f32, bx: f32, by: f32| -> f32 {
            let ex = cy * bx - sy * by;
            let ey = sy * bx + cy * by;
            let hx = px + ex;
            let hy = py + ey;
            let j = ((hx - x_min) / cell).floor() as i32;
            let i = ((hy - y_min) / cell).floor() as i32;
            if i < 0 || j < 0 || (i as usize) >= h || (j as usize) >= w {
                return clamp;
            }
            // Seeing through a confident wall at the ray's midpoint is as
            // damning as missing the endpoint — see relocalize::score_offsets,
            // INCLUDING its graze exemption: a beam that ends on a wall at
            // close range or grazing incidence has its midpoint inside that
            // same wall's cells, and clamping it punishes the beam for
            // hitting the very wall it measured (the measured false-LOST
            // of field test four).
            let mj = ((px + ex * 0.5 - x_min) / cell).floor() as i32;
            let mi = ((py + ey * 0.5 - y_min) / cell).floor() as i32;
            if mi >= 0
                && mj >= 0
                && (mi as usize) < h
                && (mj as usize) < w
                && ((mi - i).abs() > 1 || (mj - j).abs() > 1)
                && log[(mi as usize) * w + (mj as usize)] > see_through_fp
            {
                return clamp;
            }
            field[(i as usize) * w + (j as usize)].min(clamp)
        };

        let mut log_w: Vec<f32> = Vec::with_capacity(self.particles.len());
        for p in &self.particles {
            let (sy, cy) = p.2.sin_cos();
            let mut sum = 0.0_f32;
            let mut n = 0u32;
            for &(bx, by) in &beams {
                let d = endpoint_d(p.0, p.1, cy, sy, bx, by);
                sum += -(d * d) / two_sigma2;
                n += 1;
            }
            // Sum (not mean) so weight contrast is high enough that
            // ESS-based resampling fires when the cloud needs to
            // collapse. Softness comes from `beam_sigma_m`.
            if n == 0 {
                log_w.push(-1e6);
            } else {
                log_w.push(sum);
            }
        }
        // Temper the log-likelihoods before normalizing (raise the
        // posterior to a power < 1). Keeps competing modes alive past
        // the first update — with 64 beams and σ=0.20 m the raw
        // posterior is peaky enough that one frame collapses the cloud
        // to whichever cluster happened to fit best, regardless of
        // whether it's right.
        let temper = self.cfg.likelihood_temper.max(1e-6);
        // Stabilize: subtract the max before exponentiating.
        let max_lw = log_w.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
        // Multiplicative weight update — keep prior evidence around so
        // the cloud commits only after multiple frames agree. After
        // resampling fires, weights are reset uniform anyway, so this
        // collapses to the usual single-frame likelihood update from
        // that point on.
        let mut sum_w = 0.0_f32;
        for (w_out, &lw) in self.weights.iter_mut().zip(log_w.iter()) {
            let factor = ((lw - max_lw) * temper).exp();
            *w_out *= factor;
            sum_w += *w_out;
        }
        if sum_w > 0.0 {
            for w_out in self.weights.iter_mut() {
                *w_out /= sum_w;
            }
        } else {
            self.reset_weights();
        }
        self.update_count += 1;

        // Best particle residual for the lock check.
        if let Some((best_idx, _)) = self
            .weights
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
        {
            let p = self.particles[best_idx];
            let (sy, cy) = p.2.sin_cos();
            let mut sum = 0.0_f32;
            for &(bx, by) in &beams {
                sum += endpoint_d(p.0, p.1, cy, sy, bx, by);
            }
            self.last_residual_m = sum / (beams.len() as f32);
        }

        // Resample if ESS too low — but only after the grace period.
        // During grace, weights keep accumulating evidence across
        // frames; resampling early throws that away.
        let ess: f32 = 1.0 / self.weights.iter().map(|w| w * w).sum::<f32>().max(1e-12);
        if self.update_count >= self.cfg.min_updates_before_resample
            && ess < self.cfg.resample_ess_frac * (self.particles.len() as f32)
        {
            self.systematic_resample();
            // Replace `random_inject_frac` of the (now duplicated)
            // resampled particles with fresh uniform samples drawn from
            // the grid's free cells. Keeps exploration alive without
            // melting the rest of the posterior.
            let n_inject = ((self.particles.len() as f32)
                * self.cfg.random_inject_frac.clamp(0.0, 1.0)) as usize;
            if n_inject > 0 {
                self.inject_uniform(grid, n_inject);
            }
        }

        // Update lock streak based on dominant-cluster cohesion in
        // BOTH (x, y) AND yaw, anchored on the mean-shifted densest-bin
        // centre. This anchor is stable across frames — best() can hop
        // between similar-weight particles inside a cluster and cause
        // spurious streak resets, the cluster mean does not. Cache it
        // for the `dominant_*` getters polled after this update.
        let anchor = self.compute_cluster_anchor();
        self.anchor_cache = Some(anchor);
        let cluster_frac = self.fraction_within(anchor.0, anchor.1, self.cfg.locked_xy_std_m);
        let yaw_frac = self.fraction_within_yaw(anchor.2, self.cfg.locked_yaw_std_rad);
        let res_ok = self.last_residual_m.is_finite()
            && self.last_residual_m <= self.cfg.locked_max_residual_m;
        let spread_ok = cluster_frac >= self.cfg.locked_dominant_frac
            && yaw_frac >= self.cfg.locked_dominant_frac;
        // Streak counts consecutive frames of cluster + residual
        // cohesion. The motion requirement is enforced separately by
        // the runtime (total net world-frame travel since search
        // start), so an MCL per-step motion gate would just double-
        // count and starve when odom goes quiet after a walk.
        if res_ok && spread_ok {
            self.locked_streak += 1;
        } else {
            self.locked_streak = 0;
        }
    }

    /// Highest-weight particle.
    pub fn best(&self) -> Pose2 {
        self.weights
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
            .map(|(i, _)| self.particles[i])
            .unwrap_or((0.0, 0.0, 0.0))
    }

    /// Weighted-mean particle (alternative to `best()` when the cloud
    /// is multi-modal — collapses to the centre of mass).
    pub fn weighted_mean(&self) -> Pose2 {
        let mut x = 0.0_f32;
        let mut y = 0.0_f32;
        let mut sx = 0.0_f32;
        let mut cy = 0.0_f32;
        let mut sw = 0.0_f32;
        for (p, &w) in self.particles.iter().zip(self.weights.iter()) {
            x += w * p.0;
            y += w * p.1;
            sx += w * p.2.sin();
            cy += w * p.2.cos();
            sw += w;
        }
        if sw <= 0.0 {
            return (0.0, 0.0, 0.0);
        }
        (x / sw, y / sw, sx.atan2(cy))
    }

    /// 1D std-dev across the cloud's xy positions, equally weighted.
    pub fn position_std(&self) -> f32 {
        let n = self.particles.len() as f32;
        if n < 2.0 {
            return 0.0;
        }
        let (mut mx, mut my) = (0.0_f32, 0.0_f32);
        for p in &self.particles {
            mx += p.0;
            my += p.1;
        }
        mx /= n;
        my /= n;
        let mut var = 0.0_f32;
        for p in &self.particles {
            let dx = p.0 - mx;
            let dy = p.1 - my;
            var += dx * dx + dy * dy;
        }
        (var / n).sqrt()
    }

    /// Fraction of particles within `radius_m` of `(cx, cy)`. Used to
    /// measure dominant-cluster cohesion when anchored on a known good
    /// center (best particle).
    pub fn fraction_within(&self, cx: f32, cy: f32, radius_m: f32) -> f32 {
        if self.particles.is_empty() {
            return 0.0;
        }
        let r2 = radius_m * radius_m;
        let mut n = 0usize;
        for p in &self.particles {
            let dx = p.0 - cx;
            let dy = p.1 - cy;
            if dx * dx + dy * dy < r2 {
                n += 1;
            }
        }
        n as f32 / self.particles.len() as f32
    }

    /// Fraction of particles within `yaw_radius_rad` of `yaw_c` on the
    /// circle. Used alongside `fraction_within` to detect when the
    /// dominant cluster is *also* tight in yaw, not just position.
    pub fn fraction_within_yaw(&self, yaw_c: f32, yaw_radius_rad: f32) -> f32 {
        if self.particles.is_empty() {
            return 0.0;
        }
        let mut n = 0usize;
        for p in &self.particles {
            if wrap_pi(p.2 - yaw_c).abs() < yaw_radius_rad {
                n += 1;
            }
        }
        n as f32 / self.particles.len() as f32
    }

    /// Find the centre of the densest cluster by coarse binning.
    /// Particles are dropped into (x, y, yaw) bins of size
    /// `cell_m` × `cell_m` × `yaw_bin_rad`; the bin with the most
    /// occupants is the dominant cluster. Returns its centre — which
    /// is *stable across frames* even when individual particle weights
    /// hop within a cluster, unlike `best()`.
    ///
    /// Trade-off: O(N) per call (sparse-hash binning); negligible at
    /// our particle counts.
    pub fn densest_bin_center(&self, cell_m: f32, yaw_bin_rad: f32) -> Pose2 {
        use std::collections::HashMap;
        if self.particles.is_empty() {
            return (0.0, 0.0, 0.0);
        }
        let inv_cell = 1.0 / cell_m.max(1e-6);
        let inv_yaw = 1.0 / yaw_bin_rad.max(1e-6);
        let mut counts: HashMap<(i32, i32, i32), u32> = HashMap::new();
        for p in &self.particles {
            let i = (p.0 * inv_cell).floor() as i32;
            let j = (p.1 * inv_cell).floor() as i32;
            let k = ((p.2 + std::f32::consts::PI) * inv_yaw).floor() as i32;
            *counts.entry((i, j, k)).or_insert(0) += 1;
        }
        // Deterministic tie-break — same reasoning as `densest_xy_bin_center`.
        let ((bi, bj, bk), _) = counts
            .iter()
            .max_by_key(|(bin, count)| (**count, std::cmp::Reverse(**bin)))
            .unwrap();
        let bx = (*bi as f32 + 0.5) * cell_m;
        let by = (*bj as f32 + 0.5) * cell_m;
        let byaw = (*bk as f32 + 0.5) * yaw_bin_rad - std::f32::consts::PI;
        (bx, by, byaw)
    }

    /// Mean pose of particles within `radius_m` of `(bx, by)` and
    /// within `yaw_radius_rad` of `byaw`. Use this to snap to the
    /// dominant cluster after lock — it ignores secondary-mode
    /// stragglers that would pull `weighted_mean()` off-truth.
    pub fn cluster_mean(
        &self,
        bx: f32,
        by: f32,
        byaw: f32,
        radius_m: f32,
        yaw_radius_rad: f32,
    ) -> Pose2 {
        let r2 = radius_m * radius_m;
        let mut x = 0.0_f32;
        let mut y = 0.0_f32;
        let mut sx = 0.0_f32;
        let mut cyy = 0.0_f32;
        let mut n = 0usize;
        for p in &self.particles {
            let dx = p.0 - bx;
            let dy = p.1 - by;
            if dx * dx + dy * dy >= r2 {
                continue;
            }
            if wrap_pi(p.2 - byaw).abs() >= yaw_radius_rad {
                continue;
            }
            x += p.0;
            y += p.1;
            sx += p.2.sin();
            cyy += p.2.cos();
            n += 1;
        }
        if n == 0 {
            return (bx, by, byaw);
        }
        let nf = n as f32;
        (x / nf, y / nf, sx.atan2(cyy))
    }

    pub fn yaw_std(&self) -> f32 {
        let n = self.particles.len() as f32;
        if n < 2.0 {
            return 0.0;
        }
        let mut sx = 0.0_f32;
        let mut cy = 0.0_f32;
        for p in &self.particles {
            sx += p.2.sin();
            cy += p.2.cos();
        }
        // Circular variance → effective std (Mardia).
        let r = ((sx * sx + cy * cy).sqrt()) / n;
        if r >= 1.0 {
            0.0
        } else {
            (-2.0 * r.ln()).sqrt()
        }
    }

    /// True iff the lock criteria have held for `locked_min_frames`
    /// consecutive frames.
    pub fn is_locked(&self) -> bool {
        self.locked_streak >= self.cfg.locked_min_frames
    }

    fn reset_weights(&mut self) {
        let n = self.particles.len() as f32;
        for w in self.weights.iter_mut() {
            *w = 1.0 / n;
        }
    }

    /// Replace `count` randomly-chosen particles with fresh uniform
    /// samples over the grid's free cells. Random victims, not the front
    /// of the array: after systematic resampling the array preserves
    /// ancestor order, so always overwriting indices `0..count` would
    /// repeatedly kill copies of the same low-index ancestors and bias
    /// the surviving posterior. Weights are left untouched (they get
    /// reset on the next resample anyway).
    fn inject_uniform(&mut self, grid: &OccupancyGrid, count: usize) {
        let cfg = grid.cfg();
        let cell = cfg.cell;
        let w = grid.width();
        let h = grid.height();
        let n = self.particles.len();
        if n == 0 {
            return;
        }
        // Sample free cells one at a time; if we hit too many occupied
        // cells in a row just give up and leave the particles alone.
        let mut placed = 0usize;
        let mut tries = 0usize;
        let two_pi = 2.0 * std::f32::consts::PI;
        while placed < count && tries < count * 20 {
            tries += 1;
            let i = self.rng.index(h);
            let j = self.rng.index(w);
            if !grid.is_known_free(i, j) {
                continue;
            }
            let cx = cfg.x_range.0 + (j as f32 + 0.5) * cell;
            let cy = cfg.y_range.0 + (i as f32 + 0.5) * cell;
            let yaw = self.rng.f32() * two_pi - std::f32::consts::PI;
            let victim = self.rng.index(n);
            self.particles[victim] = (cx, cy, yaw);
            placed += 1;
        }
    }

    fn systematic_resample(&mut self) {
        let n = self.particles.len();
        if n == 0 {
            return;
        }
        let mut cum: Vec<f32> = Vec::with_capacity(n);
        let mut acc = 0.0_f32;
        for &w in &self.weights {
            acc += w;
            cum.push(acc);
        }
        if acc <= 0.0 {
            self.reset_weights();
            return;
        }
        let step = acc / n as f32;
        let u0: f32 = self.rng.f32() * step;
        let mut new_particles: Vec<Pose2> = Vec::with_capacity(n);
        let mut k = 0;
        for i in 0..n {
            let u = u0 + i as f32 * step;
            while k < n - 1 && cum[k] < u {
                k += 1;
            }
            new_particles.push(self.particles[k]);
        }
        self.particles = new_particles;
        self.reset_weights();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::{GridConfig, OccupancyGrid};

    fn make_room() -> OccupancyGrid {
        // 4×4 m room with asymmetric divider. Cast rays from many
        // origins so free cells densely fill the inside, mimicking what
        // a real-robot map looks like after a walk-around.
        let mut g = OccupancyGrid::new(GridConfig {
            x_range: (-2.5, 2.5),
            y_range: (-2.5, 2.5),
            cell: 0.05,
        });
        let perim = 200;
        let mut walls: Vec<(f32, f32)> = Vec::new();
        for i in 0..perim {
            let t = -2.0 + 4.0 * (i as f32 / (perim - 1) as f32);
            walls.push((t, 2.0));
            walls.push((t, -2.0));
            walls.push((2.0, t));
            walls.push((-2.0, t));
        }
        for i in 0..perim / 2 {
            let t = 2.0 * (i as f32 / ((perim / 2) as f32));
            walls.push((t, 0.5)); // asymmetric divider
        }
        let origins: Vec<(f32, f32)> = {
            let mut v = Vec::new();
            let mut y = -1.5_f32;
            while y <= 1.5 {
                let mut x = -1.5_f32;
                while x <= 1.5 {
                    v.push((x, y));
                    x += 0.30;
                }
                y += 0.30;
            }
            v
        };
        for (ox, oy) in &origins {
            for (wx, wy) in &walls {
                g.integrate_ray(*ox, *oy, *wx, *wy, true);
            }
        }
        g
    }

    fn fake_scan(grid: &mut OccupancyGrid, pose: Pose2, n_beams: usize) -> Scan {
        let mut a = Vec::new();
        let mut r = Vec::new();
        let half = std::f32::consts::FRAC_PI_2; // ±90° fan
        for k in 0..n_beams {
            let aa = -half + (k as f32 / (n_beams - 1) as f32) * 2.0 * half;
            let rr = grid.cast_ray(pose.0, pose.1, pose.2 + aa, 4.0);
            if rr > 0.0 {
                a.push(aa);
                r.push(rr);
            }
        }
        Scan::from_polar(&a, &r, (0.0, 0.0), 1e-6)
    }

    /// M5 regression, part 1: a tightly converged cluster sitting on a
    /// bin CORNER must still measure ~100% dominant fraction. The old
    /// densest-bin-centre anchor sat up to the bin half-diagonal
    /// (≈ 0.159 m) from the cluster — beyond the 0.15 m lock radius —
    /// so the ≥80% check could never pass at certain world positions.
    #[test]
    fn cluster_on_bin_corner_still_measures_dominant() {
        let mut mcl = Localizer::new(MclConfig::default(), 7);
        // Bin size = locked_xy_std_m * 1.5 = 0.225; put the cluster
        // exactly on a bin corner (multiples of 0.225).
        let cx = 0.225_f32 * 4.0;
        let cy = 0.225_f32 * 7.0;
        let n = mcl.n_particles();
        for (k, p) in mcl.particles.iter_mut().enumerate() {
            // Tiny deterministic jitter, well inside the lock radius.
            let t = k as f32 / n as f32 * std::f32::consts::TAU;
            *p = (cx + 0.02 * t.cos(), cy + 0.02 * t.sin(), 0.3);
        }
        mcl.anchor_cache = None;
        assert!(
            mcl.dominant_cluster_frac() > 0.95,
            "corner cluster frac = {}",
            mcl.dominant_cluster_frac()
        );
        let a = mcl.dominant_cluster_mean();
        let d = ((a.0 - cx).powi(2) + (a.1 - cy).powi(2)).sqrt();
        assert!(d < 0.05, "anchor {:.3} m off the cluster centre", d);
    }

    /// M5 regression, part 2: a cluster whose yaw straddles ±π must not
    /// split across yaw bins — the circular-mean anchor handles wrap.
    #[test]
    fn yaw_cluster_straddling_pi_measures_dominant() {
        let mut mcl = Localizer::new(MclConfig::default(), 7);
        let n = mcl.n_particles();
        for (k, p) in mcl.particles.iter_mut().enumerate() {
            // Yaws alternate just under +π and just above −π (same
            // physical heading, ±3°).
            let dy = 0.05 * ((k % 5) as f32 - 2.0) / 2.0;
            let yaw = if k % 2 == 0 {
                std::f32::consts::PI - 0.05 + dy
            } else {
                -std::f32::consts::PI + 0.05 + dy
            };
            let t = k as f32 / n as f32 * std::f32::consts::TAU;
            *p = (0.02 * t.cos(), 0.02 * t.sin(), wrap_pi(yaw));
        }
        mcl.anchor_cache = None;
        assert!(
            mcl.dominant_yaw_frac() > 0.95,
            "±π-straddling yaw frac = {}",
            mcl.dominant_yaw_frac()
        );
    }

    #[test]
    fn seed_uniform_lands_inside_free_space() {
        let grid = make_room();
        let mut mcl = Localizer::new(
            MclConfig {
                n_particles: 200,
                ..MclConfig::default()
            },
            0,
        );
        mcl.seed_uniform(&grid);
        // All particles should be inside the grid bounds; most should
        // be in cells we marked as known-free (we accept ~95% — the cell
        // jitter inside `seed_uniform` can land just over the edge of
        // the closest occupied cell).
        let mut inside_free = 0usize;
        for p in &mcl.particles {
            if let Some((i, j)) = grid.world_to_idx(p.0, p.1)
                && grid.is_known_free(i, j)
            {
                inside_free += 1;
            }
        }
        assert!(
            inside_free as f32 / mcl.n_particles() as f32 > 0.95,
            "{} of {} particles fell outside known-free",
            mcl.n_particles() - inside_free,
            mcl.n_particles()
        );
    }

    #[test]
    fn predict_advances_particles_with_noise() {
        let mut mcl = Localizer::new(
            MclConfig {
                n_particles: 200,
                jitter_xy_m: 0.0,
                jitter_yaw_rad: 0.0,
                ..MclConfig::default()
            },
            0,
        );
        // Seed all particles at origin facing +x.
        for p in mcl.particles.iter_mut() {
            *p = (0.0, 0.0, 0.0);
        }
        mcl.predict(1.0, 0.0, 0.0);
        // Mean should be close to +1 m on x.
        let mut mx = 0.0;
        for p in &mcl.particles {
            mx += p.0;
        }
        mx /= mcl.n_particles() as f32;
        assert!((mx - 1.0).abs() < 0.10, "mean x after predict = {mx}");
        // Spread should be > 0 (noise injected).
        assert!(mcl.position_std() > 0.0);
    }

    #[test]
    fn update_concentrates_weight_around_truth_seed() {
        let mut grid = make_room();
        let truth = (0.5_f32, -0.5_f32, 0.0_f32);
        // Seed half the particles around truth, half on a decoy 1.5 m
        // away. After one update the truth half should dominate.
        let mut mcl = Localizer::new(
            MclConfig {
                n_particles: 400,
                ..MclConfig::default()
            },
            0,
        );
        let half = mcl.n_particles() / 2;
        for i in 0..mcl.n_particles() {
            let s = if i < half {
                truth
            } else {
                (-1.0, 0.5, std::f32::consts::PI)
            };
            mcl.particles[i] = (
                s.0 + mcl.rng.normal(0.05),
                s.1 + mcl.rng.normal(0.05),
                wrap_pi(s.2 + mcl.rng.normal(0.02)),
            );
        }
        mcl.reset_weights();
        let scan = fake_scan(&mut grid, truth, 64);
        // Run several updates so the cloud commits past the
        // `min_updates_before_resample` grace period.
        for _ in 0..8 {
            mcl.update(&mut grid, &scan);
        }

        // After update + (internal) resample, weights are uniform — the
        // signal of "truth dominated" lives in particle *positions*, not
        // weights. Count particles near the truth pose vs the decoy.
        let near = |p: (f32, f32, f32), c: (f32, f32, f32), r: f32| {
            let dx = p.0 - c.0;
            let dy = p.1 - c.1;
            (dx * dx + dy * dy).sqrt() < r
        };
        let decoy = (-1.0_f32, 0.5_f32, std::f32::consts::PI);
        let n_truth = mcl
            .particles
            .iter()
            .filter(|p| near(**p, truth, 0.30))
            .count();
        let n_decoy = mcl
            .particles
            .iter()
            .filter(|p| near(**p, decoy, 0.30))
            .count();
        assert!(
            n_truth > 3 * n_decoy.max(1),
            "truth cluster didn't dominate: near_truth={n_truth} \
                 near_decoy={n_decoy}"
        );
    }
}
