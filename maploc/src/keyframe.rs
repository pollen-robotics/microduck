//! A stop, turned into one rigid local scan.
//!
//! **Why a stop is the unit.** The robot's odometry is good while it stands and drifts while it
//! walks; the first maploc measured 0.2–0.6 m between stops. A submap that spans several stops is
//! therefore not rigid, and its loop-closure witnesses contradicted each other — which is why that
//! design had to shrink submaps until they were barely bigger than a stop anyway. Here a keyframe
//! *is* a stop: every frame in it was taken from one standing place, so it is rigid by
//! construction, and everything between two stops is one odometry edge with an honest
//! uncertainty.
//!
//! **Voting.** A stop with the head sweeping collects ~100 frames of 64 zones. One frame is noise
//! (mixed pixels at depth edges, a passer-by, multipath); a cell becomes occupied only when
//! several frames agree and the floor was not seen there more often. Each frame votes at most
//! once per cell, so a wall seen for 3 s does not outweigh one seen for 1 s by sheer frame count
//! — the over-counting that made the first maploc's log-odds a binary "last window wins".
//!
//! Cells are stored as integer indices in the keyframe's own frame: compact, exact to save, and
//! the matcher wants cells anyway.

use std::collections::HashMap;

use serde::{Deserialize, Serialize};

use crate::project::Projected;
use crate::se2::Pose2;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct KeyframeConfig {
    /// Cell size, metres. Half a ToF zone's footprint at 0.5 m.
    pub res_m: f64,
    /// Frames that must agree before a cell is an obstacle.
    pub min_hits: u16,
    /// A stop shorter than this is a pause in walking, not a look around.
    pub min_frames: u32,
    /// A stop longer than this is closed and a fresh one opened, so a robot parked for an hour
    /// does not hold one window open forever and a person who stands in front of it for a minute
    /// is not one keyframe's whole truth.
    pub max_frames: u32,
    /// The robot's own footprint radius: the floor it stands on is free whether or not the
    /// sensor ever saw it.
    pub footprint_m: f64,
}

impl Default for KeyframeConfig {
    fn default() -> Self {
        Self {
            res_m: 0.025,
            min_hits: 3,
            min_frames: 10,
            max_frames: 150,
            footprint_m: 0.08,
        }
    }
}

/// One stop's evidence, in the frame of the trunk where it stood.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Keyframe {
    pub id: u32,
    /// The trunk's planar pose in the odometry frame when the stop began.
    pub odom: Pose2,
    pub t_start_ns: u64,
    pub t_end_ns: u64,
    pub res_m: f64,
    /// Obstacle cells, as indices at `res_m` in this keyframe's frame.
    pub occ: Vec<[i16; 2]>,
    /// Floor seen clear.
    pub free: Vec<[i16; 2]>,
    pub n_frames: u32,
    /// How much of the circle around the robot the obstacles cover, radians, in 10° bins. A
    /// single wall seen through the 45° keyhole aliases onto every wall at the same range; the
    /// matcher's gates read this.
    pub span_rad: f64,
}

impl Keyframe {
    pub fn cell_center(&self, c: [i16; 2]) -> [f64; 2] {
        [
            (f64::from(c[0]) + 0.5) * self.res_m,
            (f64::from(c[1]) + 0.5) * self.res_m,
        ]
    }

    pub fn occ_points(&self) -> impl Iterator<Item = [f64; 2]> + '_ {
        self.occ.iter().map(|&c| self.cell_center(c))
    }

    pub fn free_points(&self) -> impl Iterator<Item = [f64; 2]> + '_ {
        self.free.iter().map(|&c| self.cell_center(c))
    }

    /// This keyframe widened with others, each given by its pose in this one's frame. A single
    /// stop sees through a 45° keyhole even with the head sweeping; a wide search on that alone
    /// aliases. The last two or three stops, chained by odometry over a metre or two, are close
    /// enough to rigid to be matched as one.
    pub fn composite(&self, others: &[(Pose2, &Keyframe)]) -> Keyframe {
        let mut occ: std::collections::BTreeSet<[i16; 2]> = self.occ.iter().copied().collect();
        let mut free: std::collections::BTreeSet<[i16; 2]> = self.free.iter().copied().collect();
        let cell = |p: [f64; 2]| {
            [
                (p[0] / self.res_m).floor().clamp(-32768.0, 32767.0) as i16,
                (p[1] / self.res_m).floor().clamp(-32768.0, 32767.0) as i16,
            ]
        };
        for (pose, kf) in others {
            occ.extend(kf.occ_points().map(|p| cell(pose.apply(p))));
            free.extend(kf.free_points().map(|p| cell(pose.apply(p))));
        }
        let free: Vec<_> = free.into_iter().filter(|c| !occ.contains(c)).collect();
        let occ: Vec<_> = occ.into_iter().collect();
        Keyframe {
            span_rad: span_of(&occ),
            occ,
            free,
            n_frames: self.n_frames + others.iter().map(|(_, k)| k.n_frames).sum::<u32>(),
            ..self.clone()
        }
    }
}

#[derive(Debug, Default, Clone, Copy)]
struct Cell {
    hits: u16,
    frees: u16,
    last_hit: u32,
    last_free: u32,
}

/// Collects one stop's frames.
pub struct KeyframeBuilder {
    cfg: KeyframeConfig,
    odom: Pose2,
    to_local: Pose2,
    t_start_ns: u64,
    t_end_ns: u64,
    n_frames: u32,
    cells: HashMap<(i32, i32), Cell>,
}

impl KeyframeBuilder {
    pub fn new(cfg: KeyframeConfig, odom: Pose2, t_ns: u64) -> Self {
        Self {
            cfg,
            odom,
            to_local: odom.inverse(),
            t_start_ns: t_ns,
            t_end_ns: t_ns,
            n_frames: 0,
            cells: HashMap::new(),
        }
    }

    pub fn n_frames(&self) -> u32 {
        self.n_frames
    }

    pub fn is_full(&self) -> bool {
        self.n_frames >= self.cfg.max_frames
    }

    pub fn odom(&self) -> Pose2 {
        self.odom
    }

    pub fn t_start_ns(&self) -> u64 {
        self.t_start_ns
    }

    fn cell_of(&self, odom_xy: [f64; 2]) -> (i32, i32) {
        let p = self.to_local.apply(odom_xy);
        (
            (p[0] / self.cfg.res_m).floor() as i32,
            (p[1] / self.cfg.res_m).floor() as i32,
        )
    }

    /// Fold one projected frame in. Its points are in the odometry frame; they are expressed in
    /// the keyframe's frame through the odometry pose *at the start of the stop*, so a stand
    /// policy shuffling a few millimetres is carried into the geometry rather than averaged away.
    pub fn add(&mut self, frame: &Projected) {
        self.n_frames += 1;
        self.t_end_ns = frame.t_ns;
        let stamp = self.n_frames;
        let res = self.cfg.res_m;
        let mut hit_cells = Vec::with_capacity(frame.hits.len());
        for &h in &frame.hits {
            let c = self.cell_of(h);
            hit_cells.push(c);
            let cell = self.cells.entry(c).or_default();
            if cell.last_hit != stamp {
                cell.last_hit = stamp;
                cell.hits = cell.hits.saturating_add(1);
            }
        }
        for run in &frame.free {
            let len = (run.to[0] - run.from[0]).hypot(run.to[1] - run.from[1]);
            let n = ((len / (0.5 * res)).ceil() as usize).max(1);
            let end = self.cell_of(run.to);
            for k in 0..=n {
                let u = k as f64 / n as f64;
                let p = [
                    run.from[0] + u * (run.to[0] - run.from[0]),
                    run.from[1] + u * (run.to[1] - run.from[1]),
                ];
                let c = self.cell_of(p);
                // Keep clear of the obstacle a run ends on: a beam grazing a wall at a shallow
                // angle crosses cells the wall itself occupies.
                if run.ends_at_hit && (c.0 - end.0).abs() <= 1 && (c.1 - end.1).abs() <= 1 {
                    continue;
                }
                let cell = self.cells.entry(c).or_default();
                if cell.last_free != stamp {
                    cell.last_free = stamp;
                    cell.frees = cell.frees.saturating_add(1);
                }
            }
        }
    }

    /// Close the stop. `None` when it was too short to be a look around.
    pub fn finish(self, id: u32) -> Option<Keyframe> {
        if self.n_frames < self.cfg.min_frames {
            return None;
        }
        let narrow = |v: i32| v.clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16;
        let mut occ = Vec::new();
        let mut free = Vec::new();
        for (&(x, y), c) in &self.cells {
            if c.hits >= self.cfg.min_hits && c.hits >= c.frees {
                occ.push([narrow(x), narrow(y)]);
            } else if c.frees > 0 && c.hits == 0 {
                free.push([narrow(x), narrow(y)]);
            }
        }
        // The floor under the robot.
        let r = (self.cfg.footprint_m / self.cfg.res_m).ceil() as i32;
        for x in -r..r {
            for y in -r..r {
                let c = [
                    (f64::from(x) + 0.5) * self.cfg.res_m,
                    (f64::from(y) + 0.5) * self.cfg.res_m,
                ];
                if c[0].hypot(c[1]) <= self.cfg.footprint_m
                    && self
                        .cells
                        .get(&(x, y))
                        .is_none_or(|c| c.hits < self.cfg.min_hits)
                    && !free.contains(&[narrow(x), narrow(y)])
                {
                    free.push([narrow(x), narrow(y)]);
                }
            }
        }
        // HashMap iteration order is per-process random; the matcher's tie-breaks and the saved
        // session must not depend on it.
        occ.sort_unstable();
        free.sort_unstable();
        let span_rad = span_of(&occ);
        Some(Keyframe {
            id,
            odom: self.odom,
            t_start_ns: self.t_start_ns,
            t_end_ns: self.t_end_ns,
            res_m: self.cfg.res_m,
            occ,
            free,
            n_frames: self.n_frames,
            span_rad,
        })
    }
}

/// How much of the circle around the keyframe's origin its obstacles cover, in 10° bins.
fn span_of(occ: &[[i16; 2]]) -> f64 {
    let mut bins = [false; 36];
    for &[x, y] in occ {
        let a = (f64::from(y) + 0.5).atan2(f64::from(x) + 0.5);
        let b = (((a + std::f64::consts::PI) / (2.0 * std::f64::consts::PI)) * 36.0) as usize;
        bins[b.min(35)] = true;
    }
    bins.iter().filter(|&&b| b).count() as f64 * 10f64.to_radians()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::project::FreeRun;

    fn frame(hits: Vec<[f64; 2]>, free: Vec<FreeRun>) -> Projected {
        Projected {
            t_ns: 0,
            hits,
            free,
            n_used: 0,
        }
    }

    #[test]
    fn a_passer_by_does_not_survive_the_vote_and_a_wall_does() {
        let mut b = KeyframeBuilder::new(KeyframeConfig::default(), Pose2::IDENTITY, 0);
        for k in 0..30 {
            let mut hits = vec![[1.0, 0.0]];
            // Someone walking across at a slow 0.5 m/s: 3 cm a frame at 15 Hz, so each cell
            // sees them once.
            hits.push([0.5, -0.45 + 0.03 * f64::from(k)]);
            b.add(&frame(hits, vec![]));
        }
        let kf = b.finish(0).unwrap();
        let pts: Vec<_> = kf.occ_points().collect();
        assert!(
            pts.iter()
                .any(|p| (p[0] - 1.0).abs() < 0.02 && p[1].abs() < 0.02)
        );
        assert!(pts.iter().all(|p| (p[0] - 0.5).abs() > 0.02), "{pts:?}");
    }

    #[test]
    fn a_frame_counts_once_per_cell() {
        // Many beams of one frame landing in the same cell are one observation: otherwise a wall
        // the sensor faces squarely (all 8 rows of a column in one cell) wins every vote against
        // the floor seen beside it.
        let mut b = KeyframeBuilder::new(KeyframeConfig::default(), Pose2::IDENTITY, 0);
        for _ in 0..2 {
            b.add(&frame(vec![[1.0, 0.0]; 8], vec![]));
        }
        for _ in 0..8 {
            b.add(&frame(vec![], vec![]));
        }
        let kf = b.finish(0).unwrap();
        assert!(kf.occ.is_empty(), "two frames are below min_hits = 3");
    }

    #[test]
    fn evidence_is_kept_in_the_frame_of_the_stop() {
        let odom = Pose2::new(2.0, 1.0, std::f64::consts::FRAC_PI_2);
        let mut b = KeyframeBuilder::new(KeyframeConfig::default(), odom, 0);
        for _ in 0..12 {
            // One metre straight ahead of a robot facing +y.
            b.add(&frame(vec![[2.0, 2.0]], vec![]));
        }
        let kf = b.finish(7).unwrap();
        let p = kf.occ_points().next().unwrap();
        assert!((p[0] - 1.0).abs() < 0.03 && p[1].abs() < 0.03, "{p:?}");
        assert_eq!(kf.id, 7);
    }

    #[test]
    fn floor_beside_a_wall_end_is_not_carved_into_the_wall() {
        let mut b = KeyframeBuilder::new(KeyframeConfig::default(), Pose2::IDENTITY, 0);
        for _ in 0..12 {
            b.add(&frame(
                vec![[1.0, 0.0]],
                vec![FreeRun {
                    from: [0.3, 0.0],
                    to: [1.0, 0.0],
                    ends_at_hit: true,
                }],
            ));
        }
        let kf = b.finish(0).unwrap();
        assert_eq!(kf.occ.len(), 1);
        // The wall's cell starts at 1.0 m; one cell of clearance before it stays unclaimed.
        assert!(
            kf.free_points().all(|p| p[0] < 0.97),
            "free reached the wall"
        );
    }
}
