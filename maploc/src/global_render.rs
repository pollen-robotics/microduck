//! Composite all submap grids into a single global occupancy grid.
//!
//! Each submap is a small local grid anchored at its world pose. To get
//! a global view we:
//!
//!   1. Compute a world-frame bounding box covering every submap's grid
//!      corners (after applying the submap's anchor pose).
//!   2. Allocate a fresh `OccupancyGrid` covering that bbox + a margin.
//!   3. For each submap, walk the *global* cells inside its rotated
//!      footprint; transform each global cell centre into the submap's
//!      local frame and sample the local cell there, adding its
//!      log-odds to the global cell (clamped to `[LO_MIN, LO_MAX]`).
//!
//! Inverse mapping (global → local sampling) matters: the previous
//! forward mapping pushed each *source* cell centre to one destination
//! cell, and a rotated regular lattice is not surjective onto the
//! destination lattice — solid walls in yaw-rotated submaps rendered
//! with periodic holes the planner could thread paths through.
//!
//! Naive O(total footprint cells) implementation. Fine at our scales
//! (a 4 m × 4 m × 5 cm submap covers ≤ ~13 k global cells; 50 submaps
//! = a few ms on the Radxa's A55s).

use crate::grid::{GridConfig, OccupancyGrid};
use crate::submap::Submap;

#[derive(Debug, Clone, Copy)]
pub struct GlobalRenderConfig {
    /// Cell size of the rendered global grid.
    pub cell_m: f32,
    /// Padding around the union bbox, metres.
    pub margin_m: f32,
    /// Per-submap cells are only composited when their |log-odds| is
    /// above this threshold. 0 = include every barely-positive cell
    /// (legacy, fuzzy walls). ~150 hides one-off ToF flickers without
    /// erasing genuine walls.
    pub min_hit_threshold_fp: i16,
}

impl Default for GlobalRenderConfig {
    fn default() -> Self {
        Self {
            cell_m: 0.05,
            margin_m: 0.5,
            min_hit_threshold_fp: 150,
        }
    }
}

/// Render the union of all submaps into a fresh global grid. Returns
/// `None` when given no submaps (no bbox to render). When all submaps
/// have empty grids, the resulting global grid is just an empty grid
/// covering their anchors.
pub fn render_global<'a>(
    submaps: impl IntoIterator<Item = &'a Submap>,
    cfg: &GlobalRenderConfig,
) -> Option<OccupancyGrid> {
    let submaps: Vec<&Submap> = submaps.into_iter().collect();
    if submaps.is_empty() {
        return None;
    }

    // Step 1 — bbox over all submap grid corners (in world frame).
    let mut min_x = f32::INFINITY;
    let mut max_x = f32::NEG_INFINITY;
    let mut min_y = f32::INFINITY;
    let mut max_y = f32::NEG_INFINITY;
    for s in &submaps {
        let cfg_l = s.grid().cfg();
        let (ax, ay, ayaw) = s.anchor_pose();
        let ca = ayaw.cos();
        let sa = ayaw.sin();
        for (xl, yl) in [
            (cfg_l.x_range.0, cfg_l.y_range.0),
            (cfg_l.x_range.0, cfg_l.y_range.1),
            (cfg_l.x_range.1, cfg_l.y_range.0),
            (cfg_l.x_range.1, cfg_l.y_range.1),
        ] {
            let xw = ax + ca * xl - sa * yl;
            let yw = ay + sa * xl + ca * yl;
            if xw < min_x {
                min_x = xw;
            }
            if xw > max_x {
                max_x = xw;
            }
            if yw < min_y {
                min_y = yw;
            }
            if yw > max_y {
                max_y = yw;
            }
        }
    }
    let m = cfg.margin_m;
    let global_cfg = GridConfig {
        x_range: (min_x - m, max_x + m),
        y_range: (min_y - m, max_y + m),
        cell: cfg.cell_m,
    };
    let mut global = OccupancyGrid::new(global_cfg);

    // Step 2 — for each submap, sweep the global cells inside its
    // rotated footprint; sample the submap at each global cell centre
    // (inverse mapping — see module doc) and accumulate the log-odds.
    let g_cfg = *global.cfg();
    let g_cell = g_cfg.cell;
    for s in &submaps {
        let local = s.grid();
        let cfg_l = local.cfg();
        let (ax, ay, ayaw) = s.anchor_pose();
        let ca = ayaw.cos();
        let sa = ayaw.sin();
        // Global-index bbox of this submap's rotated footprint (+1 cell
        // of slack for the centre-vs-corner sampling offset).
        let mut fx0 = f32::INFINITY;
        let mut fx1 = f32::NEG_INFINITY;
        let mut fy0 = f32::INFINITY;
        let mut fy1 = f32::NEG_INFINITY;
        for (xl, yl) in [
            (cfg_l.x_range.0, cfg_l.y_range.0),
            (cfg_l.x_range.0, cfg_l.y_range.1),
            (cfg_l.x_range.1, cfg_l.y_range.0),
            (cfg_l.x_range.1, cfg_l.y_range.1),
        ] {
            let xw = ax + ca * xl - sa * yl;
            let yw = ay + sa * xl + ca * yl;
            fx0 = fx0.min(xw);
            fx1 = fx1.max(xw);
            fy0 = fy0.min(yw);
            fy1 = fy1.max(yw);
        }
        let (i_lo, j_lo) = global.world_to_cell(fx0, fy0);
        let (i_hi, j_hi) = global.world_to_cell(fx1, fy1);
        let i_lo = (i_lo - 1).max(0) as usize;
        let j_lo = (j_lo - 1).max(0) as usize;
        let i_hi = ((i_hi + 1).max(0) as usize).min(global.height().saturating_sub(1));
        let j_hi = ((j_hi + 1).max(0) as usize).min(global.width().saturating_sub(1));
        for gi in i_lo..=i_hi {
            let yw = g_cfg.y_range.0 + (gi as f32 + 0.5) * g_cell;
            for gj in j_lo..=j_hi {
                let xw = g_cfg.x_range.0 + (gj as f32 + 0.5) * g_cell;
                // World → submap-local (inverse of the anchor pose).
                let dx = xw - ax;
                let dy = yw - ay;
                let xl = ca * dx + sa * dy;
                let yl = -sa * dx + ca * dy;
                let Some((li, lj)) = local.world_to_idx(xl, yl) else {
                    continue;
                };
                let v = local.log_at(li, lj);
                if v == 0 {
                    continue;
                }
                // Skip per-submap cells whose |log-odds| is below the
                // configured threshold — keeps the global render from
                // accumulating sub-threshold noise into a wall once it
                // crosses zero in aggregate.
                if v.unsigned_abs() < cfg.min_hit_threshold_fp.unsigned_abs() {
                    continue;
                }
                global.add_log_odds_at(gi, gj, v);
            }
        }
    }
    Some(global)
}

#[cfg(test)]
mod tests {

    fn sc(angles: &[f32], ranges: &[f32]) -> crate::submap::Scan {
        crate::submap::Scan::from_polar(angles, ranges, (0.0, 0.0), 0.0)
    }
    use super::*;
    use crate::grid::GridConfig;
    use crate::submap::Submap;

    #[test]
    fn render_two_overlapping_submaps_combines_their_marks() {
        let cfg = GridConfig {
            x_range: (-1.0, 1.0),
            y_range: (-1.0, 1.0),
            cell: 0.05,
        };
        // Submap A at (0, 0, 0): mark a wall at world (0.5, 0). Repeat
        // the same beam so the cell crosses the render confidence
        // threshold (a single hit would otherwise be filtered as noise).
        let mut a = Submap::new_at((0.0, 0.0, 0.0), cfg);
        for _ in 0..5 {
            a.integrate_scan((0.0, 0.0, 0.0), &sc(&[0.0], &[0.5]));
        }
        // Submap B anchored at (0.5, 0, 0): same world wall is at LOCAL (0, 0).
        let mut b = Submap::new_at((0.5, 0.0, 0.0), cfg);
        for _ in 0..5 {
            b.integrate_scan((0.5, 0.0, 0.0), &sc(&[0.0], &[0.05]));
        }

        let g = render_global([&a, &b], &GlobalRenderConfig::default()).unwrap();
        // World cell ~ (0.5, 0) should be marked occupied (positive log-odds)
        // — both submaps contributed there.
        let (i, j) = g.world_to_idx(0.5, 0.0).expect("inside global bbox");
        assert!(
            g.log_at(i, j) > 0,
            "world (0.5, 0) wasn't marked occupied (log = {})",
            g.log_at(i, j)
        );
    }

    /// Aliasing regression: a solid wall in a yaw-rotated submap must
    /// render as a solid wall. The old forward mapping (source cell →
    /// one destination cell) left periodic holes on rotated lattices —
    /// holes the planner then threaded paths through.
    #[test]
    fn rotated_submap_wall_renders_without_holes() {
        let cfg = GridConfig {
            x_range: (-2.0, 2.0),
            y_range: (-2.0, 2.0),
            cell: 0.05,
        };
        // Submap anchored at 30° yaw. Paint a dense wall along the
        // submap-local line x = 1.0, y in [-1, 1] (many repeats so every
        // wall cell clears the render threshold).
        let yaw = 30.0_f32.to_radians();
        let mut s = Submap::new_at((0.0, 0.0, yaw), cfg);
        for _ in 0..8 {
            let mut y = -1.0_f32;
            while y <= 1.0 {
                let a = y.atan2(1.0); // beam angle to (1.0, y)
                let r = (1.0 + y * y).sqrt(); // range to (1.0, y)
                s.integrate_scan((0.0, 0.0, yaw), &sc(&[a], &[r]));
                y += 0.02; // denser than the 5 cm cells → solid local wall
            }
        }
        let g = render_global([&s], &GlobalRenderConfig::default()).unwrap();
        // Walk the wall in WORLD frame and count holes: for each sample
        // point on the wall line, the containing global cell (or one of
        // its 8 neighbours, to absorb the lattice offset) must be
        // occupied.
        let (ca, sa) = (yaw.cos(), yaw.sin());
        let mut holes = 0usize;
        let mut samples = 0usize;
        let mut y = -0.9_f32;
        while y <= 0.9 {
            let xw = ca * 1.0 - sa * y;
            let yw = sa * 1.0 + ca * y;
            let (i, j) = g.world_to_idx(xw, yw).expect("wall inside bbox");
            let mut hit = false;
            for di in -1i32..=1 {
                for dj in -1i32..=1 {
                    let ii = i as i32 + di;
                    let jj = j as i32 + dj;
                    if ii < 0 || jj < 0 || ii as usize >= g.height() || jj as usize >= g.width() {
                        continue;
                    }
                    if g.log_at(ii as usize, jj as usize) > 0 {
                        hit = true;
                    }
                }
            }
            if !hit {
                holes += 1;
            }
            samples += 1;
            y += 0.05;
        }
        assert_eq!(
            holes, 0,
            "rotated wall rendered with {holes}/{samples} holes"
        );
    }
}
