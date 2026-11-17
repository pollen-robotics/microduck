//! A map region, prepared for scoring a scan against it.
//!
//! Built on demand from keyframes at their *current* graph poses — never inked once and kept —
//! so a loop closure that moves twenty keyframes moves the walls they saw with them, and a
//! keyframe found to be wrong is simply left out.
//!
//! Each cell holds the log-likelihood of "a return landed here":
//!   - near a wall, a Gaussian in the distance to it (peak 0);
//!   - on floor the robot has *seen* clear, low — a return there contradicts the map;
//!   - unknown, in between — the sensor is a keyhole, and a scan that reaches past what was
//!     mapped is normal, not wrong.
//!
//! The first maploc's distance field held only the walls, so a scan that landed behind a wall or
//! straight across a mapped room scored as well as one that fit; the floor term is what tells a
//! pose that explains the scan from one that merely touches walls.

use crate::keyframe::Keyframe;
use crate::se2::Pose2;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct FieldConfig {
    /// Spread of the wall likelihood, metres: range noise plus a zone's footprint.
    pub sigma_m: f64,
    /// Likelihood of a return on floor seen clear.
    pub p_free: f64,
    /// Likelihood of a return where the map knows nothing.
    pub p_unknown: f64,
}

impl Default for FieldConfig {
    fn default() -> Self {
        Self {
            sigma_m: 0.04,
            p_free: 0.03,
            p_unknown: 0.2,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Field {
    pub res: f64,
    pub origin: [f64; 2],
    pub w: usize,
    pub h: usize,
    /// Per cell: log-likelihood of a return landing there.
    pub ll: Vec<f32>,
    /// Per cell: an obstacle cell — a floor-seen-clear point of a scan landing here went through
    /// a wall.
    pub occ: Vec<bool>,
    /// Per cell: anything was observed (obstacle or clear floor).
    pub known: Vec<bool>,
    pub ll_unknown: f32,
    pub n_occ: usize,
}

impl Field {
    /// The field over `keyframes` at the given map poses. `None` when they hold no obstacle at
    /// all — there is nothing to match against.
    pub fn build(
        cfg: &FieldConfig,
        keyframes: &[(Pose2, &Keyframe)],
        margin_m: f64,
    ) -> Option<Self> {
        let res = keyframes.first()?.1.res_m;
        let mut lo = [f64::INFINITY; 2];
        let mut hi = [f64::NEG_INFINITY; 2];
        let mut any_occ = false;
        for (pose, kf) in keyframes {
            for p in kf.occ_points().chain(kf.free_points()) {
                let q = pose.apply(p);
                lo = [lo[0].min(q[0]), lo[1].min(q[1])];
                hi = [hi[0].max(q[0]), hi[1].max(q[1])];
            }
            any_occ |= !kf.occ.is_empty();
        }
        if !any_occ {
            return None;
        }
        let origin = [lo[0] - margin_m, lo[1] - margin_m];
        let w = ((hi[0] - lo[0] + 2.0 * margin_m) / res).ceil() as usize + 1;
        let h = ((hi[1] - lo[1] + 2.0 * margin_m) / res).ceil() as usize + 1;
        let n = w * h;

        // Votes, once per keyframe per cell.
        let mut occ_votes = vec![0u16; n];
        let mut free_votes = vec![0u16; n];
        let mut stamp = vec![u32::MAX; n];
        let mut free_stamp = vec![u32::MAX; n];
        let idx = |q: [f64; 2]| -> Option<usize> {
            let i = ((q[0] - origin[0]) / res).floor();
            let j = ((q[1] - origin[1]) / res).floor();
            (i >= 0.0 && j >= 0.0 && (i as usize) < w && (j as usize) < h)
                .then(|| j as usize * w + i as usize)
        };
        for (k, (pose, kf)) in keyframes.iter().enumerate() {
            let k = k as u32;
            for p in kf.occ_points() {
                if let Some(c) = idx(pose.apply(p))
                    && stamp[c] != k
                {
                    stamp[c] = k;
                    occ_votes[c] += 1;
                }
            }
            for p in kf.free_points() {
                if let Some(c) = idx(pose.apply(p))
                    && free_stamp[c] != k
                {
                    free_stamp[c] = k;
                    free_votes[c] += 1;
                }
            }
        }

        let ll_free = cfg.p_free.ln() as f32;
        let ll_unknown = cfg.p_unknown.ln() as f32;
        let mut occ = vec![false; n];
        let mut known = vec![false; n];
        let mut ll = vec![ll_unknown; n];
        let mut n_occ = 0;
        for c in 0..n {
            if occ_votes[c] > 0 && occ_votes[c] >= free_votes[c] {
                occ[c] = true;
                known[c] = true;
                n_occ += 1;
            } else if free_votes[c] > 0 {
                known[c] = true;
                ll[c] = ll_free;
            }
        }
        if n_occ == 0 {
            return None;
        }
        // Splat the wall likelihood out to 3σ, keeping the best per cell.
        let r = (3.0 * cfg.sigma_m / res).ceil() as isize;
        let inv2s2 = 1.0 / (2.0 * cfg.sigma_m * cfg.sigma_m);
        let mut kernel = Vec::new();
        for dy in -r..=r {
            for dx in -r..=r {
                let d2 = ((dx * dx + dy * dy) as f64) * res * res;
                kernel.push((dx, dy, (-d2 * inv2s2) as f32));
            }
        }
        for c in 0..n {
            if !occ[c] {
                continue;
            }
            let (ci, cj) = ((c % w) as isize, (c / w) as isize);
            for &(dx, dy, v) in &kernel {
                let (i, j) = (ci + dx, cj + dy);
                if i < 0 || j < 0 || i >= w as isize || j >= h as isize {
                    continue;
                }
                let t = j as usize * w + i as usize;
                if v > ll[t] {
                    ll[t] = v;
                    known[t] = true;
                }
            }
        }
        Some(Self {
            res,
            origin,
            w,
            h,
            ll,
            occ,
            known,
            ll_unknown,
            n_occ,
        })
    }

    /// The cell `q` falls in, as signed indices (may be outside the field).
    pub fn cell(&self, q: [f64; 2]) -> (isize, isize) {
        (
            ((q[0] - self.origin[0]) / self.res).floor() as isize,
            ((q[1] - self.origin[1]) / self.res).floor() as isize,
        )
    }

    pub fn index(&self, i: isize, j: isize) -> Option<usize> {
        (i >= 0 && j >= 0 && (i as usize) < self.w && (j as usize) < self.h)
            .then(|| j as usize * self.w + i as usize)
    }

    /// The same field with each cell holding the best value of the `k × k` block whose low corner
    /// it is. Scoring a translation that is a multiple of `k` cells against it bounds, from above,
    /// the score of every translation in the block — what makes the coarse pass of a wide search
    /// unable to discard the right answer.
    pub fn pooled(&self, k: usize) -> Self {
        let mut out = self.clone();
        let mut row = vec![f32::NEG_INFINITY; self.w * self.h];
        for j in 0..self.h {
            for i in 0..self.w {
                let mut m = f32::NEG_INFINITY;
                for a in 0..k {
                    m = m.max(if i + a < self.w {
                        self.ll[j * self.w + i + a]
                    } else {
                        self.ll_unknown
                    });
                }
                row[j * self.w + i] = m;
            }
        }
        for j in 0..self.h {
            for i in 0..self.w {
                let mut m = f32::NEG_INFINITY;
                for b in 0..k {
                    m = m.max(if j + b < self.h {
                        row[(j + b) * self.w + i]
                    } else {
                        self.ll_unknown
                    });
                }
                out.ll[j * self.w + i] = m;
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn kf(occ: Vec<[i16; 2]>, free: Vec<[i16; 2]>) -> Keyframe {
        Keyframe {
            id: 0,
            odom: Pose2::IDENTITY,
            t_start_ns: 0,
            t_end_ns: 0,
            res_m: 0.025,
            occ,
            free,
            n_frames: 20,
            span_rad: 0.0,
        }
    }

    #[test]
    fn walls_score_best_seen_floor_worst_unknown_between() {
        let k = kf(vec![[40, 0]], (10..30).map(|x| [x, 0]).collect());
        let f = Field::build(&FieldConfig::default(), &[(Pose2::IDENTITY, &k)], 0.3).unwrap();
        let at = |x: f64| {
            let (i, j) = f.cell([x, 0.0125]);
            f.ll[f.index(i, j).unwrap()]
        };
        let wall = at(1.0125);
        let floor = at(0.5);
        let unknown = at(-0.02);
        assert!(
            wall > unknown && unknown > floor,
            "{wall} {unknown} {floor}"
        );
    }

    #[test]
    fn pooling_bounds_every_translation_in_its_block() {
        let k = kf(vec![[40, 0], [41, 3], [7, -9]], vec![[20, 0]]);
        let f = Field::build(&FieldConfig::default(), &[(Pose2::IDENTITY, &k)], 0.3).unwrap();
        let p = f.pooled(4);
        for j in 0..f.h {
            for i in 0..f.w {
                for b in 0..4 {
                    for a in 0..4 {
                        if let Some(t) = f.index((i + a) as isize, (j + b) as isize) {
                            assert!(p.ll[j * f.w + i] >= f.ll[t]);
                        }
                    }
                }
            }
        }
    }
}
