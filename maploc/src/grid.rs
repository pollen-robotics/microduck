//! The map as a picture: an occupancy grid rendered from the keyframes at their current poses.
//!
//! This is what a planner reads and what a viewer draws. It is derived, never accumulated: after
//! a loop closure it is rendered again from the moved keyframes, so it cannot disagree with the
//! graph — the first maploc composited submaps by adding clamped log-odds one after another, and
//! which submap came last decided whether a misregistered wall showed.

use crate::keyframe::Keyframe;
use crate::se2::Pose2;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Cell {
    Unknown = 0,
    Free = 1,
    Occupied = 2,
}

#[derive(Debug, Clone, PartialEq)]
pub struct OccupancyGrid {
    pub res: f64,
    /// Map coordinates of cell (0, 0)'s low corner.
    pub origin: [f64; 2],
    pub w: usize,
    pub h: usize,
    /// Row-major, row 0 at the lowest y.
    pub cells: Vec<Cell>,
}

impl OccupancyGrid {
    /// Render `keyframes` at `res`. `path` is the robot's own track between stops: floor it
    /// walked on is free whether or not the sensor saw it, and it is the best-proven free space
    /// there is — the ToF's blind zone right in front of the feet is exactly where it walks.
    pub fn render(
        keyframes: &[(Pose2, &Keyframe)],
        path: &[[Pose2; 2]],
        res: f64,
        footprint_m: f64,
    ) -> Option<Self> {
        let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
        for (pose, kf) in keyframes {
            for p in kf.occ_points().chain(kf.free_points()) {
                let q = pose.apply(p);
                lo = [lo[0].min(q[0]), lo[1].min(q[1])];
                hi = [hi[0].max(q[0]), hi[1].max(q[1])];
            }
        }
        if !lo[0].is_finite() {
            return None;
        }
        let margin = 2.0 * res;
        let origin = [lo[0] - margin, lo[1] - margin];
        let w = ((hi[0] - lo[0] + 2.0 * margin) / res).ceil() as usize + 1;
        let h = ((hi[1] - lo[1] + 2.0 * margin) / res).ceil() as usize + 1;
        let n = w * h;
        let idx = |q: [f64; 2]| -> Option<usize> {
            let i = ((q[0] - origin[0]) / res).floor();
            let j = ((q[1] - origin[1]) / res).floor();
            (i >= 0.0 && j >= 0.0 && (i as usize) < w && (j as usize) < h)
                .then(|| j as usize * w + i as usize)
        };
        let mut occ = vec![0u16; n];
        let mut free = vec![0u16; n];
        let mut stamp_o = vec![u32::MAX; n];
        let mut stamp_f = vec![u32::MAX; n];
        for (k, (pose, kf)) in keyframes.iter().enumerate() {
            let k = k as u32;
            for p in kf.occ_points() {
                if let Some(c) = idx(pose.apply(p))
                    && stamp_o[c] != k
                {
                    stamp_o[c] = k;
                    occ[c] += 1;
                }
            }
            for p in kf.free_points() {
                if let Some(c) = idx(pose.apply(p))
                    && stamp_f[c] != k
                {
                    stamp_f[c] = k;
                    free[c] += 1;
                }
            }
        }
        let mut walked = vec![false; n];
        for [a, b] in path {
            let len = a.dist(*b);
            let steps = ((len / (0.5 * res)).ceil() as usize).max(1);
            let r = (footprint_m / res).ceil() as isize;
            for s in 0..=steps {
                let u = s as f64 / steps as f64;
                let c = [a.x + u * (b.x - a.x), a.y + u * (b.y - a.y)];
                for dj in -r..=r {
                    for di in -r..=r {
                        let q = [c[0] + di as f64 * res, c[1] + dj as f64 * res];
                        if (di * di + dj * dj) as f64 * res * res <= footprint_m * footprint_m
                            && let Some(i) = idx(q)
                        {
                            walked[i] = true;
                        }
                    }
                }
            }
        }
        let cells = (0..n)
            .map(|c| {
                if walked[c] {
                    Cell::Free
                } else if occ[c] > 0 && occ[c] >= free[c] {
                    Cell::Occupied
                } else if free[c] > 0 {
                    Cell::Free
                } else {
                    Cell::Unknown
                }
            })
            .collect();
        Some(Self {
            res,
            origin,
            w,
            h,
            cells,
        })
    }

    pub fn count(&self, c: Cell) -> usize {
        self.cells.iter().filter(|&&x| x == c).count()
    }

    /// A binary PGM (P5), north up: unknown grey, free white, occupied black.
    pub fn to_pgm(&self) -> Vec<u8> {
        let mut out = format!("P5\n{} {}\n255\n", self.w, self.h).into_bytes();
        for j in (0..self.h).rev() {
            for i in 0..self.w {
                out.push(match self.cells[j * self.w + i] {
                    Cell::Unknown => 160,
                    Cell::Free => 255,
                    Cell::Occupied => 0,
                });
            }
        }
        out
    }
}
