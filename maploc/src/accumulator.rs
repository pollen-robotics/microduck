//! Still-window scan accumulator: many noisy frames in, one vetted scan out.
//!
//! Stop-and-scan gives the mapper 50–200 depth frames per stop, usually with
//! the head panning. The prototype inked every frame straight into the
//! submap, which let two kinds of junk through, measured on its own recorded
//! sessions (see `examples/replay.rs`):
//!
//!   - **transient returns** — a person walking beside the robot paints leg
//!     arcs that later free-space rays only partially erase;
//!   - **the far-range noise tail** — the sensor's error grows steeply
//!     toward its range limit, and a handful of 2–4 m outliers fuzz a wall
//!     more than a hundred good returns sharpen it.
//!
//! Together they set a ~9 cm map noise floor that no downstream gate
//! survived: loop closures (10 cm gate) always just missed, and the true
//! pose scored worse at relocalize than the acceptance threshold.
//!
//! The accumulator holds a window's frames and, when the window closes,
//! keeps only beams whose endpoint cell was hit in at least
//! `min_frames` *distinct frames*. A wall is confirmed by every frame that
//! looks at it; a walking leg is somewhere else each time. The survivors
//! merge into one wide composite scan expressed in the body frame of the
//! window's middle pose — per-beam origins make that exact even across the
//! pan.

use std::collections::HashMap;

use crate::submap::{Pose2, Scan};

#[derive(Debug, Clone, Copy)]
pub struct AccumulatorConfig {
    /// Endpoint-vote bin size. Matches the map cell: a vote means "this
    /// map cell would be inked".
    pub cell_m: f32,
    /// Distinct frames that must hit an endpoint cell before its beams
    /// count. 1 = pass-through.
    pub min_frames: u32,
    /// Beams longer than this are dropped entirely — the sensor's noise
    /// past here costs more than the coverage buys.
    pub max_range_m: f32,
    /// Windows with fewer frames than this skip the vote (nothing to
    /// vote with) and pass through unfiltered.
    pub min_window_frames: usize,
}

impl Default for AccumulatorConfig {
    fn default() -> Self {
        Self {
            cell_m: 0.05,
            min_frames: 3,
            max_range_m: 2.0,
            min_window_frames: 6,
        }
    }
}

pub struct WindowAccumulator {
    cfg: AccumulatorConfig,
    frames: Vec<(Pose2, Scan)>,
}

impl WindowAccumulator {
    pub fn new(cfg: AccumulatorConfig) -> Self {
        Self {
            cfg,
            frames: Vec::new(),
        }
    }

    pub fn push(&mut self, pose: Pose2, scan: Scan) {
        self.frames.push((pose, scan));
    }

    pub fn len(&self) -> usize {
        self.frames.len()
    }

    pub fn is_empty(&self) -> bool {
        self.frames.is_empty()
    }

    /// Close the window: vote, filter, merge. Returns the composite scan
    /// and the pose it is expressed at (the window's middle frame — the
    /// median of a still window, without assuming the window was still).
    pub fn finish(&mut self) -> Option<(Pose2, Scan)> {
        let frames = std::mem::take(&mut self.frames);
        if frames.is_empty() {
            return None;
        }
        let rep = frames[frames.len() / 2].0;

        // Vote: distinct frames per world endpoint cell.
        let inv = 1.0 / self.cfg.cell_m.max(1e-6);
        let mut votes: HashMap<(i32, i32), (u32, u32)> = HashMap::new(); // (count, last frame)
        let vote_enabled = frames.len() >= self.cfg.min_window_frames;
        let max_r_sq = self.cfg.max_range_m * self.cfg.max_range_m;
        if vote_enabled {
            for (f_idx, (pose, scan)) in frames.iter().enumerate() {
                let (sy, cy) = pose.2.sin_cos();
                for &((obx, oby), (ebx, eby)) in &scan.beams {
                    let dx = ebx - obx;
                    let dy = eby - oby;
                    if dx * dx + dy * dy > max_r_sq {
                        continue;
                    }
                    let wx = pose.0 + cy * ebx - sy * eby;
                    let wy = pose.1 + sy * ebx + cy * eby;
                    let key = ((wx * inv).floor() as i32, (wy * inv).floor() as i32);
                    let e = votes.entry(key).or_insert((0, u32::MAX));
                    if e.1 != f_idx as u32 {
                        e.0 += 1;
                        e.1 = f_idx as u32;
                    }
                }
            }
        }

        // Filter + merge into the representative body frame.
        let mut out = Scan::default();
        let (ry, rx, ryaw) = (rep.1, rep.0, rep.2);
        let (rs, rc) = (ryaw.sin(), ryaw.cos());
        for (pose, scan) in &frames {
            let (sy, cy) = pose.2.sin_cos();
            for &((obx, oby), (ebx, eby)) in &scan.beams {
                let dx = ebx - obx;
                let dy = eby - oby;
                if dx * dx + dy * dy > max_r_sq {
                    continue;
                }
                // World coordinates of this beam.
                let owx = pose.0 + cy * obx - sy * oby;
                let owy = pose.1 + sy * obx + cy * oby;
                let ewx = pose.0 + cy * ebx - sy * eby;
                let ewy = pose.1 + sy * ebx + cy * eby;
                if vote_enabled {
                    // Check the endpoint's 3×3 neighbourhood, not just its
                    // own cell: a wall sitting on a cell boundary splits its
                    // votes between the two cells, and a filter that punishes
                    // geometry for landing on a lattice line would thin the
                    // very walls it exists to sharpen.
                    let ki = (ewx * inv).floor() as i32;
                    let kj = (ewy * inv).floor() as i32;
                    let confirmed = (-1..=1).any(|di| {
                        (-1..=1).any(|dj| {
                            votes
                                .get(&(ki + di, kj + dj))
                                .is_some_and(|v| v.0 >= self.cfg.min_frames)
                        })
                    });
                    if !confirmed {
                        continue;
                    }
                }
                // World → representative body frame.
                let o = (
                    rc * (owx - rx) + rs * (owy - ry),
                    -rs * (owx - rx) + rc * (owy - ry),
                );
                let e = (
                    rc * (ewx - rx) + rs * (ewy - ry),
                    -rs * (ewx - rx) + rc * (ewy - ry),
                );
                out.beams.push((o, e));
            }
        }
        if out.beams.is_empty() {
            None
        } else {
            Some((rep, out))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn one_beam_scan(angle: f32, range: f32) -> Scan {
        Scan::from_polar(&[angle], &[range], (0.0, 0.0), 0.0)
    }

    /// A wall every frame sees survives; a "leg" that is somewhere else
    /// each frame does not.
    #[test]
    fn persistent_returns_survive_and_transients_die() {
        let mut acc = WindowAccumulator::new(AccumulatorConfig::default());
        for i in 0..10 {
            let mut scan = one_beam_scan(0.0, 1.0); // the wall, every frame
            // The leg: a fresh spot each frame, 20 cm apart.
            scan.merge(&one_beam_scan(1.0, 0.5 + 0.2 * i as f32));
            acc.push((0.0, 0.0, 0.0), scan);
        }
        let (pose, out) = acc.finish().expect("a composite");
        assert_eq!(pose, (0.0, 0.0, 0.0));
        assert_eq!(out.n_valid(), 10, "10 wall votes, 0 legs: {:?}", out.beams);
        for &(_, (ex, ey)) in &out.beams {
            assert!((ex - 1.0).abs() < 0.01 && ey.abs() < 0.01);
        }
    }

    /// Beams past the range cap are gone before they can vote.
    #[test]
    fn the_far_range_tail_is_dropped() {
        let mut acc = WindowAccumulator::new(AccumulatorConfig::default());
        for _ in 0..10 {
            let mut scan = one_beam_scan(0.0, 1.0);
            scan.merge(&one_beam_scan(0.5, 3.5)); // past max_range_m
            acc.push((0.0, 0.0, 0.0), scan);
        }
        let (_, out) = acc.finish().expect("a composite");
        assert!(
            out.beams
                .iter()
                .all(|&(_, (ex, ey))| (ex * ex + ey * ey).sqrt() < 2.1)
        );
    }

    /// A tiny window has nothing to vote with: it passes through.
    #[test]
    fn short_windows_pass_through() {
        let mut acc = WindowAccumulator::new(AccumulatorConfig::default());
        acc.push((0.0, 0.0, 0.0), one_beam_scan(0.0, 1.0));
        let (_, out) = acc.finish().expect("a composite");
        assert_eq!(out.n_valid(), 1);
    }

    /// Frames at different poses vote in world cells and merge into the
    /// representative frame exactly: a wall seen from two poses lands at
    /// one place.
    #[test]
    fn the_merge_is_exact_across_poses() {
        let mut acc = WindowAccumulator::new(AccumulatorConfig {
            min_frames: 3,
            ..AccumulatorConfig::default()
        });
        // Wall at world (1, 0); robot at origin and slightly left/right,
        // beam adjusted so the endpoint is the same world point.
        for i in 0..9 {
            let y = -0.01 + 0.0025 * i as f32;
            let pose = (0.0, y, 0.0);
            let angle = (-y / 1.0).atan2(1.0);
            let range = (1.0 + y * y).sqrt();
            acc.push(pose, one_beam_scan(angle, range));
        }
        let (rep, out) = acc.finish().expect("a composite");
        assert!(out.n_valid() >= 9);
        for &(_, (ex, ey)) in &out.beams {
            // Back to world through the representative pose.
            let (sy, cy) = rep.2.sin_cos();
            let wx = rep.0 + cy * ex - sy * ey;
            let wy = rep.1 + sy * ex + cy * ey;
            assert!(
                (wx - 1.0).abs() < 0.01 && wy.abs() < 0.02,
                "merged endpoint drifted: ({wx}, {wy})"
            );
        }
    }
}
