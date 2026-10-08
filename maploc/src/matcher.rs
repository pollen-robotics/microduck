//! Where a keyframe fits in a field, and how sure that is — per direction.
//!
//! Exhaustive correlative search (Olson, *Real-Time Correlative Scan Matching*, 2009): every pose
//! on a grid inside the window is scored, and the scores, read as a likelihood, give both the
//! answer and its covariance. The covariance is the point. A 45° keyhole on one wall pins the
//! distance to the wall and says nothing about the position along it; the first maploc's matcher
//! returned a pose anyway — the corner of its search window on a flat score — and the pose graph
//! took it at a fixed 5 cm in every direction. Here a flat direction shows up as a covariance as
//! wide as the window, and [`MatchResult::info`] gives it zero weight: the graph keeps odometry
//! there and takes the wall's distance from the scan.
//!
//! A search too wide to run exhaustively (relocalization, loop closure) runs coarse-to-fine:
//! a first pass against a max-pooled field, which bounds each block of translations from above,
//! so refining the best coarse candidates cannot miss a better one by much.

use serde::{Deserialize, Serialize};

use crate::field::Field;
use crate::keyframe::Keyframe;
use crate::se2::{Pose2, wrap};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct MatchConfig {
    /// Neighbouring returns on one wall are not independent measurements; the summed
    /// log-likelihood is divided by this before it is read as a probability, or every match
    /// claims millimetre certainty.
    pub temperature: f64,
    /// Obstacle cells scored per match, at most — the rest are skipped evenly.
    pub max_points: usize,
    /// Floor-seen-clear cells scored per match, at most.
    pub max_free_points: usize,
    /// Log-likelihood charged per clear-floor cell of the scan that lands on a mapped obstacle
    /// (the scan saw through a wall).
    pub through_ll: f64,
    /// A direction whose spread exceeds this fraction of the window is unconstrained. A flat
    /// score over the window spreads 1/√3 ≈ 0.58 of it.
    pub unconstrained_frac: f64,
    /// Floors on the reported spread, so a sharp peak is never read as better than the grid can
    /// resolve.
    pub min_sigma_xy: f64,
    pub min_sigma_yaw: f64,
}

impl Default for MatchConfig {
    fn default() -> Self {
        Self {
            temperature: 4.0,
            max_points: 400,
            max_free_points: 200,
            through_ll: (0.1f64).ln(),
            unconstrained_frac: 0.4,
            min_sigma_xy: 0.01,
            min_sigma_yaw: 0.2f64.to_radians(),
        }
    }
}

/// A box of poses around a guess.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Window {
    pub center: Pose2,
    pub half_xy: f64,
    pub half_yaw: f64,
    pub step_yaw: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct MatchResult {
    pub pose: Pose2,
    /// Spread of the pose, `(x, y, yaw)` in the map frame.
    pub cov: [[f64; 3]; 3],
    /// What the pose graph should believe: the inverse of `cov` with every unconstrained
    /// direction zeroed.
    pub info: [[f64; 3]; 3],
    /// Directions the scan pins, of three.
    pub rank: usize,
    /// Mean log-likelihood per scored obstacle cell at the best pose.
    pub mean_ll: f64,
    /// Fraction of the scan's obstacles within one σ of a mapped wall.
    pub hit_frac: f64,
    /// Fraction of the scan's obstacles landing anywhere the map has seen.
    pub known_frac: f64,
    /// Fraction landing on floor the map saw clear — contradictions.
    pub conflict_frac: f64,
    pub n_points: usize,
}

struct Points {
    occ: Vec<[f64; 2]>,
    free: Vec<[f64; 2]>,
}

fn decimate(it: impl ExactSizeIterator<Item = [f64; 2]>, max: usize) -> Vec<[f64; 2]> {
    let n = it.len();
    let stride = n.div_ceil(max.max(1)).max(1);
    it.step_by(stride).collect()
}

fn points(kf: &Keyframe, cfg: &MatchConfig) -> Points {
    Points {
        occ: decimate(kf.occ.iter().map(|&c| kf.cell_center(c)), cfg.max_points),
        free: decimate(
            kf.free.iter().map(|&c| kf.cell_center(c)),
            cfg.max_free_points,
        ),
    }
}

/// Scores of every pose in a window: `scores[yaw][dj][di]`, flattened.
struct Grid {
    n: isize,
    step: isize,
    yaws: Vec<f64>,
    scores: Vec<f64>,
}

impl Grid {
    fn side(&self) -> usize {
        (2 * self.n + 1) as usize
    }
}

/// Score every pose `center + (di·step·res, dj·step·res, yaw)` for |di|,|dj| ≤ n.
fn score_grid(
    field: &Field,
    pts: &Points,
    center: Pose2,
    n: isize,
    step: isize,
    yaws: Vec<f64>,
    cfg: &MatchConfig,
) -> Grid {
    let side = (2 * n + 1) as usize;
    let mut scores = vec![0.0; yaws.len() * side * side];
    let through = cfg.through_ll as f32;
    let mut cells = Vec::with_capacity(pts.occ.len());
    let mut free_cells = Vec::with_capacity(pts.free.len());
    for (k, &dyaw) in yaws.iter().enumerate() {
        let pose = Pose2::new(center.x, center.y, center.yaw + dyaw);
        cells.clear();
        cells.extend(pts.occ.iter().map(|&p| field.cell(pose.apply(p))));
        free_cells.clear();
        free_cells.extend(pts.free.iter().map(|&p| field.cell(pose.apply(p))));
        let base = k * side * side;
        for dj in -n..=n {
            for di in -n..=n {
                let (oi, oj) = (di * step, dj * step);
                let mut s = 0.0f32;
                for &(i, j) in &cells {
                    s += field
                        .index(i + oi, j + oj)
                        .map_or(field.ll_unknown, |c| field.ll[c]);
                }
                for &(i, j) in &free_cells {
                    if field.index(i + oi, j + oj).is_some_and(|c| field.occ[c]) {
                        s += through;
                    }
                }
                scores[base + ((dj + n) as usize) * side + (di + n) as usize] = f64::from(s);
            }
        }
    }
    Grid {
        n,
        step,
        yaws,
        scores,
    }
}

fn yaw_steps(half: f64, step: f64) -> Vec<f64> {
    let k = (half / step).round().max(0.0) as i64;
    (-k..=k).map(|i| i as f64 * step).collect()
}

/// The best pose in a window, searched exhaustively at the field's resolution.
pub fn match_local(
    field: &Field,
    kf: &Keyframe,
    win: Window,
    cfg: &MatchConfig,
) -> Option<MatchResult> {
    let pts = points(kf, cfg);
    if pts.occ.is_empty() {
        return None;
    }
    let n = (win.half_xy / field.res).round().max(1.0) as isize;
    let grid = score_grid(
        field,
        &pts,
        win.center,
        n,
        1,
        yaw_steps(win.half_yaw, win.step_yaw),
        cfg,
    );
    Some(summarize(field, &pts, win, &grid, cfg))
}

fn summarize(
    field: &Field,
    pts: &Points,
    win: Window,
    grid: &Grid,
    cfg: &MatchConfig,
) -> MatchResult {
    let side = grid.side();
    let (best_k, &best) = grid
        .scores
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.total_cmp(b.1).then(b.0.cmp(&a.0)))
        .expect("non-empty window");
    let unpack = |k: usize| {
        let yaw_i = k / (side * side);
        let r = k % (side * side);
        let dj = (r / side) as isize - grid.n;
        let di = (r % side) as isize - grid.n;
        let s = (grid.step as f64) * field.res;
        [di as f64 * s, dj as f64 * s, grid.yaws[yaw_i]]
    };
    // The distribution over the window, and its moments.
    let mut wsum = 0.0;
    let mut mean = [0.0; 3];
    let mut m2 = [[0.0; 3]; 3];
    for (k, &s) in grid.scores.iter().enumerate() {
        let w = ((s - best) / cfg.temperature).exp();
        if w < 1e-9 {
            continue;
        }
        let v = unpack(k);
        wsum += w;
        for a in 0..3 {
            mean[a] += w * v[a];
            for b in 0..3 {
                m2[a][b] += w * v[a] * v[b];
            }
        }
    }
    let mut cov = [[0.0; 3]; 3];
    for a in 0..3 {
        mean[a] /= wsum;
    }
    for a in 0..3 {
        for b in 0..3 {
            cov[a][b] = m2[a][b] / wsum - mean[a] * mean[b];
        }
    }
    let step_xy = grid.step as f64 * field.res;
    let step_yaw = if grid.yaws.len() > 1 {
        grid.yaws[1] - grid.yaws[0]
    } else {
        0.0
    };
    cov[0][0] += step_xy * step_xy / 12.0 + cfg.min_sigma_xy * cfg.min_sigma_xy;
    cov[1][1] += step_xy * step_xy / 12.0 + cfg.min_sigma_xy * cfg.min_sigma_xy;
    cov[2][2] += step_yaw * step_yaw / 12.0 + cfg.min_sigma_yaw * cfg.min_sigma_yaw;

    // Normalize by the window so "as wide as the window" is one number for every axis.
    let half = [
        win.half_xy.max(step_xy),
        win.half_xy.max(step_xy),
        win.half_yaw.max(cfg.min_sigma_yaw),
    ];
    let mut cn = [[0.0; 3]; 3];
    for a in 0..3 {
        for b in 0..3 {
            cn[a][b] = cov[a][b] / (half[a] * half[b]);
        }
    }
    let (vals, vecs) = eigen_sym3(cn);
    let limit = cfg.unconstrained_frac * cfg.unconstrained_frac;
    let mut info_n = [[0.0; 3]; 3];
    let mut rank = 0;
    for e in 0..3 {
        if vals[e] > limit || vals[e] <= 0.0 {
            continue;
        }
        rank += 1;
        for a in 0..3 {
            for b in 0..3 {
                info_n[a][b] += vecs[a][e] * vecs[b][e] / vals[e];
            }
        }
    }
    let mut info = [[0.0; 3]; 3];
    for a in 0..3 {
        for b in 0..3 {
            info[a][b] = info_n[a][b] / (half[a] * half[b]);
        }
    }

    let d = unpack(best_k);
    let pose = Pose2::new(
        win.center.x + d[0],
        win.center.y + d[1],
        wrap(win.center.yaw + d[2]),
    );
    let (mut hit, mut known, mut conflict) = (0usize, 0usize, 0usize);
    for &p in &pts.occ {
        let (i, j) = field.cell(pose.apply(p));
        if let Some(c) = field.index(i, j) {
            if field.ll[c] > -0.5 {
                hit += 1;
            }
            if field.known[c] {
                known += 1;
                if field.ll[c] < field.ll_unknown && !field.occ[c] {
                    conflict += 1;
                }
            }
        }
    }
    let n = pts.occ.len();
    MatchResult {
        pose,
        cov,
        info,
        rank,
        mean_ll: best / n as f64,
        hit_frac: hit as f64 / n as f64,
        known_frac: known as f64 / n as f64,
        conflict_frac: conflict as f64 / n as f64,
        n_points: n,
    }
}

/// A wide search's answer: the best pose refined, and how the runner-up compares.
#[derive(Debug, Clone, PartialEq)]
pub struct WideResult {
    pub best: MatchResult,
    /// How much more likely the best is than the best candidate elsewhere — more than
    /// [`WIDE_SEPARATION_M`] or [`WIDE_SEPARATION_RAD`] away — as a log ratio (temperature
    /// applied). Infinite when nothing else came close enough to be refined.
    pub margin: f64,
}

pub const WIDE_SEPARATION_M: f64 = 0.25;
pub const WIDE_SEPARATION_RAD: f64 = 0.17;

/// Coarse-to-fine search over a window too large to score exhaustively.
pub fn match_wide(
    field: &Field,
    kf: &Keyframe,
    win: Window,
    cfg: &MatchConfig,
) -> Option<WideResult> {
    const POOL: isize = 4;
    const REFINE: usize = 24;
    let pts = points(kf, cfg);
    if pts.occ.is_empty() {
        return None;
    }
    let pooled = field.pooled(POOL as usize);
    let n = (win.half_xy / (field.res * POOL as f64)).ceil().max(1.0) as isize;
    let coarse = score_grid(
        &pooled,
        &pts,
        win.center,
        n,
        POOL,
        yaw_steps(win.half_yaw, win.step_yaw),
        cfg,
    );
    let side = coarse.side();
    let mut order: Vec<usize> = (0..coarse.scores.len()).collect();
    order.sort_by(|&a, &b| {
        coarse.scores[b]
            .total_cmp(&coarse.scores[a])
            .then(a.cmp(&b))
    });

    // Refine the best coarse cells, skipping ones a refined candidate already covers.
    let mut refined: Vec<(f64, MatchResult)> = Vec::new();
    let mut best_fine = f64::NEG_INFINITY;
    for &k in order.iter() {
        if refined.len() >= REFINE || coarse.scores[k] < best_fine {
            // The pooled score bounds the block from above: once it falls below the best
            // refined score, no remaining block can win.
            break;
        }
        let yaw_i = k / (side * side);
        let r = k % (side * side);
        let dj = (r / side) as isize - n;
        let di = (r % side) as isize - n;
        let s = field.res * POOL as f64;
        // The pooled block covers [0, POOL) cells forward of its corner; centre the fine window
        // on the block.
        let c = Pose2::new(
            win.center.x + di as f64 * s + 0.5 * s - 0.5 * field.res,
            win.center.y + dj as f64 * s + 0.5 * s - 0.5 * field.res,
            win.center.yaw + coarse.yaws[yaw_i],
        );
        if refined
            .iter()
            .any(|(_, m)| m.pose.dist(c) < 0.5 * s && wrap(m.pose.yaw - c.yaw).abs() < win.step_yaw)
        {
            continue;
        }
        let fine_win = Window {
            center: c,
            half_xy: 0.5 * s + field.res,
            half_yaw: 0.5 * win.step_yaw,
            step_yaw: win.step_yaw / 4.0,
        };
        let grid = score_grid(
            field,
            &pts,
            c,
            (fine_win.half_xy / field.res).round() as isize,
            1,
            yaw_steps(fine_win.half_yaw, fine_win.step_yaw),
            cfg,
        );
        let m = summarize(field, &pts, fine_win, &grid, cfg);
        let total = m.mean_ll * m.n_points as f64;
        best_fine = best_fine.max(total);
        refined.push((total, m));
    }
    refined.sort_by(|a, b| b.0.total_cmp(&a.0));
    let (best_total, best) = refined.first()?.clone();
    let runner = refined.iter().skip(1).find(|(_, m)| {
        m.pose.dist(best.pose) > WIDE_SEPARATION_M
            || wrap(m.pose.yaw - best.pose.yaw).abs() > WIDE_SEPARATION_RAD
    });
    let margin = runner.map_or(f64::INFINITY, |(t, _)| (best_total - t) / cfg.temperature);
    // The covariance of the answer comes from a local window around it, not from the narrow
    // refinement box — a well-separated peak can still be flat along a wall.
    let local = match_local(
        field,
        kf,
        Window {
            center: best.pose,
            half_xy: 0.2,
            half_yaw: 4f64.to_radians(),
            step_yaw: 1f64.to_radians(),
        },
        cfg,
    )?;
    Some(WideResult {
        best: local,
        margin,
    })
}

/// Eigen-decomposition of a symmetric 3×3 matrix by cyclic Jacobi rotations. Returns the
/// eigenvalues and the eigenvectors as columns.
pub fn eigen_sym3(mut a: [[f64; 3]; 3]) -> ([f64; 3], [[f64; 3]; 3]) {
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..50 {
        let off = a[0][1].abs() + a[0][2].abs() + a[1][2].abs();
        if off < 1e-15 {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            if a[p][q].abs() < 1e-300 {
                continue;
            }
            let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
            let t = if theta == 0.0 { 1.0 } else { t };
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            for k in 0..3 {
                let (akp, akq) = (a[k][p], a[k][q]);
                a[k][p] = c * akp - s * akq;
                a[k][q] = s * akp + c * akq;
            }
            for k in 0..3 {
                let (apk, aqk) = (a[p][k], a[q][k]);
                a[p][k] = c * apk - s * aqk;
                a[q][k] = s * apk + c * aqk;
            }
            for k in 0..3 {
                let (vkp, vkq) = (v[k][p], v[k][q]);
                v[k][p] = c * vkp - s * vkq;
                v[k][q] = s * vkp + c * vkq;
            }
        }
    }
    ([a[0][0], a[1][1], a[2][2]], v)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::field::FieldConfig;

    fn kf(occ: Vec<[i16; 2]>) -> Keyframe {
        Keyframe {
            id: 0,
            odom: Pose2::IDENTITY,
            t_start_ns: 0,
            t_end_ns: 0,
            res_m: 0.025,
            occ,
            free: vec![],
            n_frames: 20,
            span_rad: 0.0,
        }
    }

    /// A wall along y at x = 1 m, 1.6 m long.
    fn wall() -> Vec<[i16; 2]> {
        (-32..32).map(|y| [40, y]).collect()
    }

    /// A corner: the wall plus a perpendicular one at y = 0.8 m.
    fn corner() -> Vec<[i16; 2]> {
        let mut v = wall();
        v.extend((0..40).map(|x| [x, 32]));
        v
    }

    #[test]
    fn eigen_reconstructs() {
        let m = [[4.0, 1.0, 0.5], [1.0, 3.0, 0.2], [0.5, 0.2, 1.0]];
        let (l, v) = eigen_sym3(m);
        for a in 0..3 {
            for b in 0..3 {
                let r: f64 = (0..3).map(|e| v[a][e] * l[e] * v[b][e]).sum();
                assert!((r - m[a][b]).abs() < 1e-9);
            }
        }
    }

    #[test]
    fn one_wall_pins_its_distance_and_not_its_length() {
        // The first maploc's edge-of-window bug: a single wall, a flat score along it, and a
        // pose returned from the window's corner at full confidence. Here the along-wall axis
        // must come back unconstrained, and the across-wall one recovered.
        let map = kf(wall());
        let field = Field::build(&FieldConfig::default(), &[(Pose2::IDENTITY, &map)], 0.5).unwrap();
        let scan = kf((-12..12).map(|y| [40, y]).collect());
        let truth = Pose2::new(0.0, 0.0, 0.0);
        let guess = Pose2::new(0.06, 0.1, 0.0);
        let m = match_local(
            &field,
            &scan,
            Window {
                center: guess,
                half_xy: 0.25,
                half_yaw: 3f64.to_radians(),
                step_yaw: 1f64.to_radians(),
            },
            &MatchConfig::default(),
        )
        .unwrap();
        assert!(
            (m.pose.x - truth.x).abs() < 0.02,
            "across-wall error {}",
            m.pose.x
        );
        assert!(m.rank < 3, "a single wall cannot pin three directions");
        // Information along the wall (y) must be negligible next to across it (x).
        assert!(m.info[1][1] < 0.01 * m.info[0][0], "{:?}", m.info);
    }

    #[test]
    fn a_corner_pins_everything() {
        let map = kf(corner());
        let field = Field::build(&FieldConfig::default(), &[(Pose2::IDENTITY, &map)], 0.5).unwrap();
        let scan = kf(corner());
        let guess = Pose2::new(0.08, -0.07, 2f64.to_radians());
        let m = match_local(
            &field,
            &scan,
            Window {
                center: guess,
                half_xy: 0.25,
                half_yaw: 4f64.to_radians(),
                step_yaw: 1f64.to_radians(),
            },
            &MatchConfig::default(),
        )
        .unwrap();
        assert!(
            m.pose.x.abs() < 0.015 && m.pose.y.abs() < 0.015 && m.pose.yaw.abs() < 0.02,
            "{:?}",
            m.pose
        );
        assert_eq!(m.rank, 3);
        assert!(m.hit_frac > 0.9);
    }

    #[test]
    fn wide_search_finds_a_far_pose_and_reports_symmetry_as_ambiguous() {
        // An L-shaped room corner, searched over ±1.5 m and all headings.
        let mut room = corner();
        room.extend((0..20).map(|y| [-30, -60 + y])); // an asymmetric stub elsewhere
        let map = kf(room);
        let field = Field::build(&FieldConfig::default(), &[(Pose2::IDENTITY, &map)], 0.5).unwrap();
        let scan = kf(corner());
        let w = match_wide(
            &field,
            &scan,
            Window {
                center: Pose2::new(0.9, -0.7, 0.4),
                half_xy: 1.5,
                half_yaw: std::f64::consts::PI,
                step_yaw: 2f64.to_radians(),
            },
            &MatchConfig::default(),
        )
        .unwrap();
        assert!(
            w.best.pose.dist(Pose2::IDENTITY) < 0.03 && w.best.pose.yaw.abs() < 0.03,
            "{:?}",
            w.best.pose
        );

        // One straight wall matches anywhere along itself: the wide search must say so.
        let map = kf(wall());
        let field = Field::build(&FieldConfig::default(), &[(Pose2::IDENTITY, &map)], 0.5).unwrap();
        let scan = kf((-6..6).map(|y| [40, y]).collect());
        let w = match_wide(
            &field,
            &scan,
            Window {
                center: Pose2::IDENTITY,
                half_xy: 1.0,
                half_yaw: 0.5,
                step_yaw: 2f64.to_radians(),
            },
            &MatchConfig::default(),
        )
        .unwrap();
        assert!(
            w.margin < 3.0 || w.best.rank < 3,
            "a wall segment was called unique: {w:?}"
        );
    }
}
