//! The pose graph: one node per keyframe, edges for what odometry and scan matching said about
//! the motion between them.
//!
//! **What the first maploc got backwards, and this one fixes in the weights.** Heading comes from
//! the IMU's integrated gyro and is good to a fraction of a degree per lap; position comes from
//! the legs and slips. The old graph gave every odometry edge a flat 0.10 m / 0.05 rad regardless
//! of how far the robot had walked, so the graph believed the chain's heading was worth about
//! 0.16 rad over a lap and handed nearly all of the yaw vote to whichever loop closure arrived —
//! 6.7° of heading error tracked where raw odometry had 0.1°. Here odometry uncertainty grows
//! with distance walked and angle turned, separately along and across the direction of travel,
//! and the heading term is the gyro's.
//!
//! **Solver.** Levenberg–Marquardt with step acceptance, on a skyline (envelope) Cholesky. The
//! graph is a chain with a few long edges, so the envelope of each row reaches back only to its
//! oldest neighbour: a closure from node 300 to node 5 makes row 300 dense from column 5, and no
//! other row grows. That is linear in the chain for the cost of one dense row per closure — the
//! old dense 3N×3N solve was cubic and ran on every freeze. Scan-match and loop edges go through a
//! Cauchy kernel, so one wrong closure bends instead of folding the map; edges whose residual
//! stays wild after optimization are reported for removal.

use serde::{Deserialize, Serialize};

use crate::se2::{Pose2, wrap};

pub type Mat3 = [[f64; 3]; 3];

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EdgeKind {
    Odometry,
    /// A keyframe matched against the map around it while tracking.
    Match,
    /// A keyframe matched against a part of the map the robot left long ago.
    Loop,
    /// A keyframe placed by relocalization after a carry, a fall or a power cycle.
    Relocalize,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Edge {
    pub from: usize,
    pub to: usize,
    /// The pose of `to` in `from`'s frame.
    pub z: Pose2,
    /// Information (inverse covariance) of the residual, `(x, y, yaw)` in `to`'s frame — see
    /// [`info_in_frame`] for turning a map-frame or `from`-frame covariance into it.
    pub info: Mat3,
    pub kind: EdgeKind,
}

impl Edge {
    fn robust(&self) -> bool {
        self.kind != EdgeKind::Odometry
    }

    /// The residual `z⁻¹ ∘ (from⁻¹ ∘ to)` and its Jacobians with respect to `from` and `to`.
    fn linearize(&self, a: Pose2, b: Pose2) -> ([f64; 3], Mat3, Mat3) {
        let (sa, ca) = a.yaw.sin_cos();
        let (sz, cz) = self.z.yaw.sin_cos();
        let dx = b.x - a.x;
        let dy = b.y - a.y;
        // Translation of `to` in `from`'s frame.
        let lx = ca * dx + sa * dy;
        let ly = -sa * dx + ca * dy;
        let ex = lx - self.z.x;
        let ey = ly - self.z.y;
        let e = [
            cz * ex + sz * ey,
            -sz * ex + cz * ey,
            wrap(b.yaw - a.yaw - self.z.yaw),
        ];
        // d(lx, ly)/d(a.x, a.y, a.yaw) and /d(b.x, b.y).
        let dl_da = [[-ca, -sa, ly], [sa, -ca, -lx]];
        let dl_db = [[ca, sa], [-sa, ca]];
        let rz = [[cz, sz], [-sz, cz]];
        let mut ja = [[0.0; 3]; 3];
        let mut jb = [[0.0; 3]; 3];
        for r in 0..2 {
            for c in 0..3 {
                ja[r][c] = rz[r][0] * dl_da[0][c] + rz[r][1] * dl_da[1][c];
            }
            for c in 0..2 {
                jb[r][c] = rz[r][0] * dl_db[0][c] + rz[r][1] * dl_db[1][c];
            }
        }
        ja[2][2] = -1.0;
        jb[2][2] = 1.0;
        (e, ja, jb)
    }
}

fn quad(e: &[f64; 3], m: &Mat3) -> f64 {
    let mut s = 0.0;
    for a in 0..3 {
        for b in 0..3 {
            s += e[a] * m[a][b] * e[b];
        }
    }
    s
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct GraphConfig {
    /// Cauchy kernel scale on non-odometry edges, in units of Mahalanobis distance.
    pub cauchy_c: f64,
    /// An edge whose Mahalanobis distance stays above this after optimization contradicts the
    /// rest of the graph.
    pub outlier_mahalanobis: f64,
    pub max_iterations: usize,
}

impl Default for GraphConfig {
    fn default() -> Self {
        Self {
            cauchy_c: 3.0,
            outlier_mahalanobis: 6.0,
            max_iterations: 30,
        }
    }
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Graph {
    pub nodes: Vec<Pose2>,
    /// Nodes held still: the gauge. Each disconnected piece of the graph needs one, or its
    /// position is undetermined and the system singular.
    pub fixed: Vec<bool>,
    pub edges: Vec<Edge>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Report {
    pub iterations: usize,
    pub initial_cost: f64,
    pub final_cost: f64,
    /// Non-odometry edges that disagree with the optimized graph, by index.
    pub outliers: Vec<usize>,
}

impl Graph {
    pub fn add_node(&mut self, pose: Pose2, fixed: bool) -> usize {
        self.nodes.push(pose);
        self.fixed.push(fixed);
        self.nodes.len() - 1
    }

    pub fn add_edge(&mut self, e: Edge) {
        debug_assert!(e.from < self.nodes.len() && e.to < self.nodes.len());
        self.edges.push(e);
    }

    fn cost(&self, nodes: &[Pose2], cfg: &GraphConfig) -> f64 {
        let c2 = cfg.cauchy_c * cfg.cauchy_c;
        self.edges
            .iter()
            .map(|e| {
                let (r, _, _) = e.linearize(nodes[e.from], nodes[e.to]);
                let s = quad(&r, &e.info);
                if e.robust() {
                    c2 * (1.0 + s / c2).ln()
                } else {
                    s
                }
            })
            .sum()
    }

    /// Optimize every free node. Returns `None` when there is nothing to move.
    pub fn optimize(&mut self, cfg: &GraphConfig) -> Option<Report> {
        let mut var = vec![usize::MAX; self.nodes.len()];
        let mut n_var = 0;
        for (i, &f) in self.fixed.iter().enumerate() {
            if !f {
                var[i] = n_var;
                n_var += 1;
            }
        }
        if n_var == 0 || self.edges.is_empty() {
            return None;
        }
        let n = 3 * n_var;
        // Envelope: each block row reaches back to its oldest connected free block.
        let mut first_block: Vec<usize> = (0..n_var).collect();
        for e in &self.edges {
            let (a, b) = (var[e.from], var[e.to]);
            if a != usize::MAX && b != usize::MAX {
                let (lo, hi) = if a < b { (a, b) } else { (b, a) };
                first_block[hi] = first_block[hi].min(lo);
            }
        }
        let first: Vec<usize> = (0..n).map(|i| 3 * first_block[i / 3]).collect();

        let c2 = cfg.cauchy_c * cfg.cauchy_c;
        let initial_cost = self.cost(&self.nodes, cfg);
        let mut cost = initial_cost;
        let mut lambda = 1e-4;
        let mut iterations = 0;
        for _ in 0..cfg.max_iterations {
            iterations += 1;
            let mut h = Skyline::new(&first);
            let mut g = vec![0.0; n];
            for e in &self.edges {
                let (r, ja, jb) = e.linearize(self.nodes[e.from], self.nodes[e.to]);
                let mut w = 1.0;
                if e.robust() {
                    w = 1.0 / (1.0 + quad(&r, &e.info) / c2);
                }
                let blocks = [(var[e.from], ja), (var[e.to], jb)];
                // Jᵀ Ω for each side.
                let mut jt_om = [[[0.0; 3]; 3]; 2];
                for (s, (_, j)) in blocks.iter().enumerate() {
                    for a in 0..3 {
                        for b in 0..3 {
                            jt_om[s][a][b] =
                                w * (0..3).map(|k| j[k][a] * e.info[k][b]).sum::<f64>();
                        }
                    }
                }
                for (s, &(vs, _)) in blocks.iter().enumerate() {
                    if vs == usize::MAX {
                        continue;
                    }
                    for a in 0..3 {
                        g[3 * vs + a] += (0..3).map(|k| jt_om[s][a][k] * r[k]).sum::<f64>();
                    }
                    // Lower triangle only: the other side's turn adds the mirror block.
                    for &(vt, jt) in &blocks {
                        if vt == usize::MAX || vt > vs {
                            continue;
                        }
                        for a in 0..3 {
                            for b in 0..3 {
                                let (row, col) = (3 * vs + a, 3 * vt + b);
                                if col <= row {
                                    h.add(
                                        row,
                                        col,
                                        (0..3).map(|k| jt_om[s][a][k] * jt[k][b]).sum(),
                                    );
                                }
                            }
                        }
                    }
                }
            }
            let diag: Vec<f64> = (0..n).map(|i| h.get(i, i)).collect();
            let mut accepted = false;
            for _ in 0..10 {
                let mut hl = h.clone();
                for i in 0..n {
                    hl.add(i, i, lambda * (diag[i] + 1e-9));
                }
                let Some(step) = hl.solve(&g.iter().map(|v| -v).collect::<Vec<_>>()) else {
                    lambda *= 10.0;
                    continue;
                };
                let mut trial = self.nodes.clone();
                for (i, &vi) in var.iter().enumerate() {
                    if vi == usize::MAX {
                        continue;
                    }
                    trial[i].x += step[3 * vi];
                    trial[i].y += step[3 * vi + 1];
                    trial[i].yaw = wrap(trial[i].yaw + step[3 * vi + 2]);
                }
                let c = self.cost(&trial, cfg);
                if c <= cost {
                    let gain = cost - c;
                    self.nodes = trial;
                    cost = c;
                    lambda = (lambda / 3.0).max(1e-9);
                    accepted = true;
                    let max_step = step.iter().fold(0.0f64, |m, v| m.max(v.abs()));
                    if max_step < 1e-6 || gain < 1e-9 * (1.0 + cost) {
                        return Some(self.report(iterations, initial_cost, cost, cfg));
                    }
                    break;
                }
                lambda *= 5.0;
            }
            if !accepted {
                break;
            }
        }
        Some(self.report(iterations, initial_cost, cost, cfg))
    }

    fn report(
        &self,
        iterations: usize,
        initial_cost: f64,
        final_cost: f64,
        cfg: &GraphConfig,
    ) -> Report {
        let lim = cfg.outlier_mahalanobis * cfg.outlier_mahalanobis;
        let outliers = self
            .edges
            .iter()
            .enumerate()
            .filter(|(_, e)| e.robust())
            .filter(|(_, e)| {
                let (r, _, _) = e.linearize(self.nodes[e.from], self.nodes[e.to]);
                quad(&r, &e.info) > lim
            })
            .map(|(i, _)| i)
            .collect();
        Report {
            iterations,
            initial_cost,
            final_cost,
            outliers,
        }
    }
}

/// Information of one odometry edge, from what odometry measured.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct OdometryNoise {
    /// Fixed floor, metres — a stand policy shuffling, the anchor switching feet.
    pub base_xy_m: f64,
    /// Error per metre walked along the direction of travel (foot slip, stride scale).
    pub along_per_m: f64,
    /// Error per metre walked across it.
    pub across_per_m: f64,
    /// Heading floor, radians.
    pub base_yaw_rad: f64,
    /// Heading error per radian turned: the gyro's scale error.
    pub yaw_per_rad: f64,
    /// Heading error per second: gyro bias the IMU fusion has not removed.
    pub yaw_per_s: f64,
}

impl Default for OdometryNoise {
    fn default() -> Self {
        // Placeholders until the tape-measured laps fit them: the twin drifts ~0.3 % per metre,
        // the real robot certainly more; the old field tests saw 0.2–0.6 m between stops, a
        // number that also carried the old matcher's own error.
        Self {
            base_xy_m: 0.01,
            along_per_m: 0.08,
            across_per_m: 0.08,
            base_yaw_rad: 0.3f64.to_radians(),
            yaw_per_rad: 0.02,
            yaw_per_s: (0.5f64 / 60.0).to_radians(),
        }
    }
}

impl OdometryNoise {
    /// Information for the motion `z` (in the start frame) taken over `dt_s` seconds, walking
    /// `path_m` metres and turning `turned_rad` in total.
    pub fn info(&self, z: Pose2, path_m: f64, turned_rad: f64, dt_s: f64) -> Mat3 {
        let s_along = self.base_xy_m + self.along_per_m * path_m;
        let s_across = self.base_xy_m + self.across_per_m * path_m;
        let s_yaw = self.base_yaw_rad + self.yaw_per_rad * turned_rad.abs() + self.yaw_per_s * dt_s;
        // The direction of travel, in the residual's frame (the end of the motion).
        let phi = if z.x.hypot(z.y) > 1e-6 {
            z.y.atan2(z.x) - z.yaw
        } else {
            0.0
        };
        let (s, c) = phi.sin_cos();
        let (ia, ic) = (1.0 / (s_along * s_along), 1.0 / (s_across * s_across));
        [
            [c * c * ia + s * s * ic, c * s * (ia - ic), 0.0],
            [c * s * (ia - ic), s * s * ia + c * c * ic, 0.0],
            [0.0, 0.0, 1.0 / (s_yaw * s_yaw)],
        ]
    }
}

/// Re-express an information matrix stated in a frame rotated by `yaw` (relative to the map) in
/// a residual frame rotated by `frame_yaw`: `Mᵀ Ω M` with `M` the planar rotation between them.
/// A scan match's covariance is in the map frame; an edge's residual is in its `to` node's
/// frame, so a match edge passes the matched pose's yaw here.
pub fn info_in_frame(info: &Mat3, frame_yaw: f64) -> Mat3 {
    let (s, c) = frame_yaw.sin_cos();
    let m = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]];
    let mut out = [[0.0; 3]; 3];
    for a in 0..3 {
        for b in 0..3 {
            out[a][b] = (0..3)
                .flat_map(|k| (0..3).map(move |l| (k, l)))
                .map(|(k, l)| m[k][a] * info[k][l] * m[l][b])
                .sum();
        }
    }
    out
}

/// A symmetric matrix stored as its lower envelope.
#[derive(Debug, Clone)]
struct Skyline {
    first: Vec<usize>,
    start: Vec<usize>,
    data: Vec<f64>,
}

impl Skyline {
    fn new(first: &[usize]) -> Self {
        let mut start = Vec::with_capacity(first.len());
        let mut len = 0;
        for (i, &f) in first.iter().enumerate() {
            start.push(len);
            len += i - f + 1;
        }
        Self {
            first: first.to_vec(),
            start,
            data: vec![0.0; len],
        }
    }

    fn at(&self, r: usize, c: usize) -> usize {
        debug_assert!(c <= r && c >= self.first[r]);
        self.start[r] + c - self.first[r]
    }

    fn add(&mut self, r: usize, c: usize, v: f64) {
        let k = self.at(r, c);
        self.data[k] += v;
    }

    fn get(&self, r: usize, c: usize) -> f64 {
        if c < self.first[r] {
            0.0
        } else {
            self.data[self.at(r, c)]
        }
    }

    /// Solve `self · x = b` by in-place Cholesky. `None` when the matrix is not positive definite.
    fn solve(mut self, b: &[f64]) -> Option<Vec<f64>> {
        let n = self.first.len();
        for i in 0..n {
            for j in self.first[i]..=i {
                let lo = self.first[i].max(self.first[j]);
                let mut s = self.get(i, j);
                for k in lo..j {
                    s -= self.get(i, k) * self.get(j, k);
                }
                let idx = self.at(i, j);
                if j < i {
                    self.data[idx] = s / self.get(j, j);
                } else {
                    if s <= 0.0 || !s.is_finite() {
                        return None;
                    }
                    self.data[idx] = s.sqrt();
                }
            }
        }
        let mut y = b.to_vec();
        for i in 0..n {
            let mut s = y[i];
            for k in self.first[i]..i {
                s -= self.get(i, k) * y[k];
            }
            y[i] = s / self.get(i, i);
        }
        for i in (0..n).rev() {
            y[i] /= self.get(i, i);
            let yi = y[i];
            for k in self.first[i]..i {
                y[k] -= self.get(i, k) * yi;
            }
        }
        Some(y)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn iso(sxy: f64, syaw: f64) -> Mat3 {
        [
            [1.0 / (sxy * sxy), 0.0, 0.0],
            [0.0, 1.0 / (sxy * sxy), 0.0],
            [0.0, 0.0, 1.0 / (syaw * syaw)],
        ]
    }

    #[test]
    fn skyline_solves_like_dense() {
        // A random SPD matrix with a chain-plus-one-long-edge envelope.
        let n = 9;
        let mut first: Vec<usize> = (0..n).map(|i: usize| i.saturating_sub(2)).collect();
        first[8] = 0;
        let mut a = vec![vec![0.0; n]; n];
        let mut seed = 7u64;
        let mut rnd = || {
            seed = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((seed >> 33) as f64) / f64::from(u32::MAX) - 0.5
        };
        for i in 0..n {
            for j in first[i]..i {
                let v = rnd();
                a[i][j] = v;
                a[j][i] = v;
            }
            a[i][i] = 4.0 + rnd();
        }
        let mut s = Skyline::new(&first);
        for i in 0..n {
            for j in first[i]..=i {
                s.add(i, j, a[i][j]);
            }
        }
        let b: Vec<f64> = (0..n).map(|i| i as f64 - 3.0).collect();
        let x = s.solve(&b).unwrap();
        for i in 0..n {
            let r: f64 = (0..n).map(|j| a[i][j] * x[j]).sum();
            assert!((r - b[i]).abs() < 1e-9, "row {i}: {r} vs {}", b[i]);
        }
    }

    #[test]
    fn a_loop_closure_spreads_the_error_along_the_chain() {
        // A square walked with odometry that under-reads every turn by 3°; the true square
        // closes, the odometry one does not. One closure edge from the last node back to the
        // first must pull the end home and bend every corner a little.
        let mut g = Graph::default();
        let side = 2.0;
        let turn = std::f64::consts::FRAC_PI_2 - 3f64.to_radians();
        let mut pose = Pose2::IDENTITY;
        g.add_node(pose, true);
        let noise = OdometryNoise::default();
        let mut prev = 0;
        for _ in 0..4 {
            let z = Pose2::new(side, 0.0, turn);
            pose = pose.compose(z);
            let id = g.add_node(pose, false);
            g.add_edge(Edge {
                from: prev,
                to: id,
                z,
                info: noise.info(z, side, turn, 30.0),
                kind: EdgeKind::Odometry,
            });
            prev = id;
        }
        let end_before = g.nodes[prev];
        // Truth: node 4 is node 0, facing the same way.
        g.add_edge(Edge {
            from: 0,
            to: prev,
            z: Pose2::IDENTITY,
            info: iso(0.02, 0.01),
            kind: EdgeKind::Loop,
        });
        let r = g.optimize(&GraphConfig::default()).unwrap();
        let end_after = g.nodes[prev];
        assert!(
            end_after.dist(Pose2::IDENTITY) < 0.1 * end_before.dist(Pose2::IDENTITY),
            "{end_before:?} → {end_after:?}"
        );
        assert!(r.final_cost < r.initial_cost);
        assert!(r.outliers.is_empty());
    }

    #[test]
    fn a_wrong_closure_bends_but_does_not_fold_and_is_named() {
        // A straight corridor walked by good odometry, and one closure claiming the far end is
        // back at the start. Without a robust kernel that edge would drag half the corridor;
        // with it the chain holds and the edge is reported as an outlier.
        let mut g = Graph::default();
        g.add_node(Pose2::IDENTITY, true);
        let noise = OdometryNoise::default();
        for i in 1..=6usize {
            let z = Pose2::new(0.5, 0.0, 0.0);
            g.add_node(Pose2::new(0.5 * i as f64, 0.0, 0.0), false);
            g.add_edge(Edge {
                from: i - 1,
                to: i,
                z,
                info: noise.info(z, 0.5, 0.0, 5.0),
                kind: EdgeKind::Odometry,
            });
        }
        g.add_edge(Edge {
            from: 0,
            to: 6,
            z: Pose2::IDENTITY,
            info: iso(0.03, 0.02),
            kind: EdgeKind::Loop,
        });
        let r = g.optimize(&GraphConfig::default()).unwrap();
        assert!(g.nodes[6].x > 2.5, "the corridor folded: {:?}", g.nodes[6]);
        assert_eq!(r.outliers, vec![6]);
    }

    #[test]
    fn heading_is_trusted_over_a_loose_closure() {
        // Odometry heading is the gyro's; a closure with a sloppy yaw must not rotate the chain.
        let mut g = Graph::default();
        g.add_node(Pose2::IDENTITY, true);
        let noise = OdometryNoise::default();
        let z = Pose2::new(1.0, 0.0, 0.0);
        g.add_node(Pose2::new(1.0, 0.0, 0.0), false);
        g.add_edge(Edge {
            from: 0,
            to: 1,
            z,
            info: noise.info(z, 1.0, 0.0, 10.0),
            kind: EdgeKind::Odometry,
        });
        // A match saying 1.05 m and 5° — believed in x, not in yaw.
        g.add_edge(Edge {
            from: 0,
            to: 1,
            z: Pose2::new(1.05, 0.0, 5f64.to_radians()),
            info: iso(0.02, 0.05),
            kind: EdgeKind::Match,
        });
        g.optimize(&GraphConfig::default()).unwrap();
        assert!((g.nodes[1].x - 1.05).abs() < 0.01, "{:?}", g.nodes[1]);
        assert!(
            g.nodes[1].yaw.abs() < 0.5f64.to_radians(),
            "{:?}",
            g.nodes[1]
        );
    }
}
