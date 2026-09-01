//! Submap — local 2D occupancy grid + the world pose at which it was
//! anchored.
//!
//! All scans integrated into a submap are placed in *its local frame*:
//! local X/Y of (0, 0, 0) corresponds to the duck's pose at submap
//! creation. The global map is reconstructed by composing each submap's
//! local grid through its anchor pose (see `global_render`).
//!
//! For Phase 3 (single submap) we just create one Submap at world
//! origin and never close it. From Phase 4 onward `submap_manager`
//! owns the lifecycle.

use crate::grid::{GridConfig, OccupancyGrid};
use crate::pose_graph::wrap_pi;

/// Pose in SE(2): `(x, y, yaw)`.
pub type Pose2 = (f32, f32, f32);

/// One horizontal-plane scan: each beam an origin→endpoint pair in the
/// body frame.
///
/// Two decisions live in this shape, both departures from the prototype:
///
///   - **The origin is per-beam and explicit.** The prototype inked maps
///     from the *sensor* pose but scored MCL particles and relocalize
///     candidates from the *body* pose — a 10–15 cm systematic
///     map-vs-matcher disagreement on a head-mounted sensor. Every
///     consumer now derives world beams the same way:
///     `body_pose ∘ origin → body_pose ∘ endpoint`.
///
///   - **Scans compose.** A single 45° depth frame is a wedge — far too
///     ambiguous a signature to relocalize a robot in a blobby room. With
///     per-beam origins, frames taken at the same body pose but different
///     head yaws (a deliberate pan) merge into one wide-FOV scan.
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct Scan {
    /// `(origin, endpoint)` per beam, body frame, metres. Only valid,
    /// in-range beams are stored — construction filters.
    pub beams: Vec<((f32, f32), (f32, f32))>,
}

impl Scan {
    /// Every k-th beam, sized to land at or under `max_beams` — what the
    /// relocalize search and the loop closer probe with: their cost is
    /// O(candidates × beams), and a few hundred beams position as well as
    /// nine thousand.
    pub fn decimated(&self, max_beams: usize) -> Scan {
        let n = self.beams.len();
        if n <= max_beams.max(1) {
            return self.clone();
        }
        let step = n.div_ceil(max_beams.max(1));
        Scan {
            beams: self.beams.iter().copied().step_by(step).collect(),
        }
    }

    /// From parallel azimuth/range arrays measured at one sensor origin.
    /// Beams with non-finite or shorter-than-`min_range` values are
    /// dropped here, once, instead of in every consumer.
    pub fn from_polar(
        angles_body: &[f32],
        ranges: &[f32],
        sensor_in_body: (f32, f32),
        min_range: f32,
    ) -> Self {
        let (ox, oy) = sensor_in_body;
        Self {
            beams: angles_body
                .iter()
                .zip(ranges)
                .filter(|(a, r)| r.is_finite() && a.is_finite() && **r >= min_range)
                .map(|(a, &r)| ((ox, oy), (ox + r * a.cos(), oy + r * a.sin())))
                .collect(),
        }
    }

    /// Fold another scan (taken at the same body pose) into this one.
    pub fn merge(&mut self, other: &Scan) {
        self.beams.extend_from_slice(&other.beams);
    }

    /// Beam endpoints in the body frame — the one quantity every matcher
    /// and filter scores.
    pub fn endpoints_body(&self) -> impl Iterator<Item = (f32, f32)> + '_ {
        self.beams.iter().map(|&(_, e)| e)
    }

    /// How many beams the scan carries.
    pub fn n_valid(&self) -> usize {
        self.beams.len()
    }
}

/// One scan retained for loop-closure scan matching. Stored in the
/// submap's *local* frame (so it's independent of any later anchor
/// changes during pose-graph optimization).
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct RawScan {
    pub pose_in_submap: Pose2,
    pub scan: Scan,
}

/// SE(2) inverse-compose: returns the body pose expressed in the
/// anchor's frame. (x_local, y_local, yaw_local) such that composing
/// `anchor` with that local pose gives back `body_world`.
#[inline]
fn world_to_local(anchor: Pose2, body_world: Pose2) -> Pose2 {
    let (ax, ay, ayaw) = anchor;
    let (wx, wy, wyaw) = body_world;
    let dx = wx - ax;
    let dy = wy - ay;
    let ca = ayaw.cos();
    let sa = ayaw.sin();
    let xl = ca * dx + sa * dy;
    let yl = -sa * dx + ca * dy;
    let yawl = wrap_pi(wyaw - ayaw);
    (xl, yl, yawl)
}

/// Number of raw scans retained per submap for loop-closure matching.
/// 10 covers a few seconds of capture at 15 Hz; way more than enough
/// to give the matcher signal but cheap (10 × 64 beams × 8 B = 5 KB).
pub const MAX_RAW_SCANS: usize = 10;

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Submap {
    grid: OccupancyGrid,
    anchor_pose: Pose2,
    raw_scans: Vec<RawScan>,
}

impl Submap {
    /// Create a submap with its origin (local 0,0,0) at `anchor_pose` in
    /// the world frame. The grid is sized by `grid_cfg`.
    pub fn new_at(anchor_pose: Pose2, grid_cfg: GridConfig) -> Self {
        Self {
            grid: OccupancyGrid::new(grid_cfg),
            anchor_pose,
            raw_scans: Vec::with_capacity(MAX_RAW_SCANS),
        }
    }

    pub fn anchor_pose(&self) -> Pose2 {
        self.anchor_pose
    }
    pub fn grid(&self) -> &OccupancyGrid {
        &self.grid
    }
    pub fn grid_mut(&mut self) -> &mut OccupancyGrid {
        &mut self.grid
    }
    pub fn raw_scans(&self) -> &[RawScan] {
        &self.raw_scans
    }

    /// Whether anything was ever integrated. (Retention caps at
    /// [`MAX_RAW_SCANS`], so this is "has content", not a count.)
    pub fn has_content(&self) -> bool {
        !self.raw_scans.is_empty()
    }

    /// Update the anchor pose. Used by the pose-graph optimizer after
    /// loop closure: the submap's local content (grid + raw scans)
    /// stays untouched, only its anchor changes.
    pub fn set_anchor_pose(&mut self, new_anchor: Pose2) {
        self.anchor_pose = new_anchor;
    }

    /// Integrate one scan. `body_pose_world` is the duck's pose at scan
    /// capture (world frame); the beams start at the scan's own sensor
    /// origin, composed through that pose. Skipped beams: NaN/zero range,
    /// NaN angle, and sub-cell ranges — a return shorter than one grid
    /// cell puts origin and endpoint in the same cell, which would mark
    /// the sensor's *own* cell occupied (`at_end` wins over the
    /// free-space pass) and saturate it within a second at 15 Hz.
    pub fn integrate_scan(&mut self, body_pose_world: Pose2, scan: &Scan) {
        self.integrate_scan_weighted(body_pose_world, scan, 1);
    }

    /// Integrate one scan, applying its log-odds `passes` times — a vetted
    /// still-window composite is worth more than one raw frame — while
    /// remembering it as ONE raw scan. The old shape (call `integrate_scan`
    /// twice) stored duplicate raw scans, and byte-identical duplicates
    /// filled both of the loop closer's witness slots: its two-witness
    /// independence gate was cross-examining a scan against itself.
    pub fn integrate_scan_weighted(&mut self, body_pose_world: Pose2, scan: &Scan, passes: usize) {
        debug_assert!(
            body_pose_world.0.is_finite()
                && body_pose_world.1.is_finite()
                && body_pose_world.2.is_finite(),
            "non-finite body pose {body_pose_world:?}"
        );
        if !(body_pose_world.0.is_finite()
            && body_pose_world.1.is_finite()
            && body_pose_world.2.is_finite())
        {
            return;
        }
        // A beam shorter than one cell puts origin and endpoint in the
        // same cell, marking the sensor's own cell occupied — skip.
        let min_range_sq = self.grid.cell() * self.grid.cell();
        let pose_local = world_to_local(self.anchor_pose, body_pose_world);
        let (bx, by, byaw) = pose_local;
        let (cy, sy) = (byaw.cos(), byaw.sin());
        let mut touched = false;
        for _ in 0..passes.max(1) {
            for &((obx, oby), (ebx, eby)) in &scan.beams {
                let dbx = ebx - obx;
                let dby = eby - oby;
                if dbx * dbx + dby * dby < min_range_sq {
                    continue;
                }
                let ox = bx + cy * obx - sy * oby;
                let oy = by + sy * obx + cy * oby;
                let hx = bx + cy * ebx - sy * eby;
                let hy = by + sy * ebx + cy * eby;
                touched |= self.grid.integrate_ray(ox, oy, hx, hy, true);
            }
        }
        // Retain the first `MAX_RAW_SCANS` for loop closure — but only
        // scans that actually reached the grid: a fully clipped scan in a
        // witness slot is a witness that saw nothing, blocking the real
        // ones behind it, and `has_content()` calling such a submap
        // non-empty froze inkless husks into the pose graph.
        if touched && self.raw_scans.len() < MAX_RAW_SCANS {
            self.raw_scans.push(RawScan {
                pose_in_submap: pose_local,
                scan: scan.clone(),
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scan(angles: &[f32], ranges: &[f32]) -> Scan {
        Scan::from_polar(angles, ranges, (0.0, 0.0), 0.0)
    }

    #[test]
    fn world_to_local_is_inverse_of_compose() {
        // pick a non-trivial anchor and a body pose
        let anchor = (1.0_f32, -0.5, 0.6);
        let body = (2.5_f32, 0.3, 0.2);
        let local = world_to_local(anchor, body);
        // recompose: world = anchor ⊕ local
        let (ax, ay, ayaw) = anchor;
        let ca = ayaw.cos();
        let sa = ayaw.sin();
        let wx = ax + ca * local.0 - sa * local.1;
        let wy = ay + sa * local.0 + ca * local.1;
        let wyaw = wrap_pi(ayaw + local.2);
        assert!((wx - body.0).abs() < 1e-5);
        assert!((wy - body.1).abs() < 1e-5);
        assert!((wyaw - body.2).abs() < 1e-5);
    }

    #[test]
    fn integrating_a_perpendicular_wall_lights_up_the_right_cell() {
        // 4 m square grid, 5 cm cells, anchor at world origin, no rotation.
        let cfg = GridConfig {
            x_range: (-2.0, 2.0),
            y_range: (-2.0, 2.0),
            cell: 0.05,
        };
        let mut s = Submap::new_at((0.0, 0.0, 0.0), cfg);
        // Body at world origin too. One beam pointing +x at 1 m.
        s.integrate_scan((0.0, 0.0, 0.0), &scan(&[0.0], &[1.0]));
        // Cell at world (1.0, 0.0) should be occupied (positive log-odds).
        let (i, j) = s.grid().world_to_idx(1.0, 0.0).unwrap();
        assert!(
            s.grid().log_at(i, j) > 0,
            "wall cell wasn't marked occupied (log_odds={})",
            s.grid().log_at(i, j)
        );
        // Cell halfway along the ray should be free (negative log-odds).
        let (i, j) = s.grid().world_to_idx(0.5, 0.0).unwrap();
        assert!(
            s.grid().log_at(i, j) < 0,
            "free cell wasn't marked free (log_odds={})",
            s.grid().log_at(i, j)
        );
    }

    /// The port's reason to exist: a beam ranged 0.9 m from a sensor
    /// mounted 0.1 m ahead of the body must ink the wall at 1.0 m — from
    /// the sensor, not from the body.
    #[test]
    fn beams_start_at_the_sensor_not_the_body() {
        let cfg = GridConfig {
            x_range: (-2.0, 2.0),
            y_range: (-2.0, 2.0),
            cell: 0.05,
        };
        let mut s = Submap::new_at((0.0, 0.0, 0.0), cfg);
        let sc = Scan::from_polar(&[0.0], &[0.9], (0.1, 0.0), 0.0);
        for _ in 0..3 {
            s.integrate_scan((0.0, 0.0, 0.0), &sc);
        }
        let (i, j) = s.grid().world_to_idx(1.0, 0.0).unwrap();
        assert!(
            s.grid().log_at(i, j) > 0,
            "wall must land at sensor + range: log = {}",
            s.grid().log_at(i, j)
        );
        // And the naive body-origin endpoint (0.9, 0) stays free.
        let (i, j) = s.grid().world_to_idx(0.85, 0.0).unwrap();
        assert!(
            s.grid().log_at(i, j) < 0,
            "body-origin endpoint must be free space"
        );
    }

    #[test]
    fn integrating_at_offset_anchor_uses_local_frame() {
        let cfg = GridConfig {
            x_range: (-2.0, 2.0),
            y_range: (-2.0, 2.0),
            cell: 0.05,
        };
        // Anchor at world (5, 5, 0). Body at world (5, 5, 0) (same as anchor).
        // Scan should mark up the wall at LOCAL (1, 0) = WORLD (6, 5),
        // but the local grid's cell at LOCAL (1, 0) is what gets ink.
        let mut s = Submap::new_at((5.0, 5.0, 0.0), cfg);
        s.integrate_scan((5.0, 5.0, 0.0), &scan(&[0.0], &[1.0]));
        let (i, j) = s.grid().world_to_idx(1.0, 0.0).unwrap();
        assert!(s.grid().log_at(i, j) > 0);
    }
}
