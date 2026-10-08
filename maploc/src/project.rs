//! One depth frame → what it says about the floor plan, in the odometry frame.
//!
//! The map is 2D but the beams are not: the head sits ~25 cm up on a trunk that pitches and
//! rolls, and its 8×8 fan spans 45° of elevation. Flattening every return onto the floor plane —
//! what the first maploc did — gets two things wrong that matter for a robot this small:
//!
//!   - A beam that passes *over* a low box on its way to a far wall carved the box's cells free.
//!     Each column of the fan has one azimuth, so the rows that clear the box outvote the rows
//!     that hit it, and the box is erased.
//!   - Table tops and shelf undersides were inked as walls, so the map said the duck could not
//!     go where it walks every day.
//!
//! So every return is placed in 3D and judged by **height** against a band that is the duck's own
//! business: below the band is floor, above it is overhead and blocks nothing, inside it is an
//! obstacle. And a beam only proves *free floor* along the stretch where it runs low enough that
//! anything the duck could trip on would have stopped it — in practice the last few tens of
//! centimetres of a beam that lands on the floor. That is less free space per frame than carving
//! whole rays, and it is the only kind that is true.

use kinematics::Pose;
use serde::{Deserialize, Serialize};

use crate::input::{DepthFrame, StateSample};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ProjectConfig {
    /// Zone statuses whose distance is believed. ST's "valid" are 5 and 9; mapping stays strict
    /// because a believed-bad range here is a phantom wall that persists, not one wrong note
    /// (the theremin accepts more, for the opposite reason).
    pub statuses: Vec<u8>,
    /// Closer than this the sensor reports its own cover glass.
    pub min_range_m: f64,
    /// Further than this a return is too weak and too wide (one zone is ~10 cm across at 1 m) to
    /// draw a wall with.
    pub max_range_m: f64,
    /// Whether the sensor reports distance along its optical axis rather than along each zone's
    /// beam. Unsettled for the VL53L5CX/L8CX; the difference is 1 − cos(off-axis) — about 11 % at
    /// the corner zones, which bends a flat wall. The replay bench's flat-wall fit decides it.
    pub range_is_axial: bool,
    /// A return lower than `floor_margin_m + floor_margin_per_m × horizontal range` is the floor.
    /// The slope is there because pitch error grows into height error with range: 1.7 cm per
    /// metre per degree.
    pub floor_margin_m: f64,
    pub floor_margin_per_m: f64,
    /// A return higher than this is overhead — a table top, a shelf — and blocks nothing the
    /// duck does. Its full height with the head up, plus a margin.
    pub ceiling_m: f64,
    /// A beam proves floor-level free space only where it runs at or below this height.
    pub free_height_m: f64,
}

impl Default for ProjectConfig {
    fn default() -> Self {
        Self {
            statuses: vec![5, 9],
            min_range_m: 0.08,
            max_range_m: 2.0,
            range_is_axial: false,
            floor_margin_m: 0.03,
            floor_margin_per_m: 0.03,
            ceiling_m: 0.40,
            free_height_m: 0.04,
        }
    }
}

/// What a frame says, in the odometry frame's floor plane.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Projected {
    pub t_ns: u64,
    /// Obstacles: returns inside the height band.
    pub hits: Vec<[f64; 2]>,
    /// Stretches of floor the beams proved clear. `ends_at_hit` marks a stretch that ends on an
    /// obstacle, so the cells right before the obstacle are not called free — a beam grazing a
    /// wall at a shallow angle crosses cells the wall occupies.
    pub free: Vec<FreeRun>,
    /// Zones that produced anything at all, for diagnostics.
    pub n_used: usize,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FreeRun {
    pub from: [f64; 2],
    pub to: [f64; 2],
    pub ends_at_hit: bool,
}

pub struct Projector {
    cfg: ProjectConfig,
    beams: [[f64; 3]; 64],
}

impl Projector {
    pub fn new(cfg: ProjectConfig) -> Self {
        Self {
            cfg,
            beams: *kinematics::tof::Reprojector::alpha().beams(),
        }
    }

    pub fn config(&self) -> &ProjectConfig {
        &self.cfg
    }

    /// Place `frame` using the robot state at its capture instant.
    pub fn project(&self, frame: &DepthFrame, state: &StateSample) -> Projected {
        let sensor: Pose = state.trunk_pose() * state.tof_in_trunk;
        let o = sensor.pos;
        let mut out = Projected {
            t_ns: frame.t_ns,
            ..Projected::default()
        };
        for (i, beam) in self.beams.iter().enumerate() {
            if !self.cfg.statuses.contains(&frame.status[i]) || frame.distance_mm[i] <= 0 {
                continue;
            }
            let mut r = f64::from(frame.distance_mm[i]) / 1000.0;
            if self.cfg.range_is_axial {
                r /= beam[0];
            }
            if r < self.cfg.min_range_m || r > self.cfg.max_range_m {
                continue;
            }
            let d = sensor.quat.rotate(*beam);
            let p = [o[0] + r * d[0], o[1] + r * d[1], o[2] + r * d[2]];
            let h = (p[0] - o[0]).hypot(p[1] - o[1]);
            let floor = self.cfg.floor_margin_m + self.cfg.floor_margin_per_m * h;
            let is_floor = p[2] < floor;
            if p[2] > self.cfg.ceiling_m {
                continue;
            }
            out.n_used += 1;
            if !is_floor {
                out.hits.push([p[0], p[1]]);
            }
            // The low stretch of the beam: from where it descends through `free_height_m` to
            // where it ended. A beam that never gets that low proves nothing about the floor.
            let low_from = if o[2] <= self.cfg.free_height_m {
                Some(0.0)
            } else if d[2] < 0.0 {
                Some((self.cfg.free_height_m - o[2]) / d[2])
            } else {
                None
            };
            if let Some(t0) = low_from.filter(|&t0| t0 < r) {
                out.free.push(FreeRun {
                    from: [o[0] + t0 * d[0], o[1] + t0 * d[1]],
                    to: [p[0], p[1]],
                    ends_at_hit: !is_floor,
                });
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::input::Activity;
    use kinematics::Quat;

    /// A level sensor 25 cm up, pitched down by `pitch` radians.
    fn state(pitch: f64) -> StateSample {
        StateSample {
            t_ns: 1,
            trunk_pos: [0.0, 0.0, 0.12],
            odom_yaw: 0.0,
            trunk_quat: Quat::IDENTITY,
            tof_in_trunk: Pose::new(
                [0.08, 0.0, 0.13],
                Quat::from_axis_angle([0.0, 1.0, 0.0], pitch),
            ),
            activity: Activity::default(),
        }
    }

    fn frame(mm: i16) -> DepthFrame {
        DepthFrame {
            t_ns: 1,
            distance_mm: vec![mm; 64],
            status: vec![5; 64],
        }
    }

    #[test]
    fn rows_over_a_low_box_do_not_erase_it() {
        // The first maploc's failure: a 15 cm box at 0.6 m in front of a wall. Upper rows clear
        // the box and hit the wall; lower rows hit the box. Projected in 3D, the wall rows prove
        // no floor free in front of the wall — they never run below 4 cm — so nothing votes the
        // box's cells free.
        let p = Projector::new(ProjectConfig::default());
        let s = state(0.0);
        let mut f = frame(1500);
        let beams = *kinematics::tof::Reprojector::alpha().beams();
        for (i, b) in beams.iter().enumerate() {
            // Where along this beam does it reach 15 cm high at 0.6 m out?
            let t = 0.6 / b[0];
            if 0.25 + t * b[2] < 0.15 {
                f.distance_mm[i] = (t * 1000.0) as i16;
            }
        }
        let out = p.project(&f, &s);
        assert!(out.hits.iter().any(|h| (h[0] - 0.68).abs() < 0.05));
        for run in &out.free {
            // No free stretch may pass through the box's footprint (x in 0.6..0.75 m).
            assert!(run.to[0] < 0.68 + 0.05 || run.from[0] > 0.75, "{run:?}");
        }
    }

    #[test]
    fn a_table_top_is_overhead_not_a_wall() {
        let p = Projector::new(ProjectConfig::default());
        // Head tilted up ~34°: even the bottom row climbs 14°, and is above 40 cm by 0.8 m.
        let s = state(-0.6);
        let out = p.project(&frame(800), &s);
        // A table top must not become a wall the duck cannot walk under.
        assert!(out.hits.is_empty(), "{:?}", out.hits);
    }

    #[test]
    fn floor_returns_prove_the_floor_near_where_they_land() {
        let p = Projector::new(ProjectConfig::default());
        let s = state(0.5); // looking down ~29°
        let sensor = s.trunk_pose() * s.tof_in_trunk;
        let mut f = frame(0);
        let beams = *kinematics::tof::Reprojector::alpha().beams();
        for (i, b) in beams.iter().enumerate() {
            let d = sensor.quat.rotate(*b);
            f.distance_mm[i] = (-sensor.pos[2] / d[2] * 1000.0) as i16;
        }
        let out = p.project(&f, &s);
        assert!(
            out.hits.is_empty(),
            "floor read as obstacles: {:?}",
            out.hits
        );
        assert!(out.free.len() > 40);
        for run in &out.free {
            assert!(!run.ends_at_hit);
            // The low stretch is short: 4 cm of height at 7–51° down is 5–33 cm of floor.
            let len = (run.to[0] - run.from[0]).hypot(run.to[1] - run.from[1]);
            assert!(len < 0.35, "{len}");
        }
    }
}
