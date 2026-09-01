//! ToF reprojection: an 8×8 grid of distances becomes points in the trunk.
//!
//! `tofd` publishes what the sensor sees — slant ranges along fixed beams in
//! the sensor's own frame. What a consumer wants is geometry the robot can act
//! on: where each return sits in the trunk frame, with the two systematic
//! nuisances filtered the way the prototype runtime settled on:
//!
//!   - **Floor returns.** A head looking down sees the floor at every range;
//!     a beam whose slant range times its downward component reaches the
//!     sensor's height above the floor (times a safety factor, for pose error)
//!     hit the floor, not an obstacle.
//!   - **Too-close returns.** Below ~10 cm the sensor's readings stop being
//!     trustworthy — cover-glass crosstalk and pulse pile-up produce phantom
//!     short returns — so a reading in that band is discarded as noise.
//!
//! The sensor pose comes from [`HeadFk::tof_in_trunk`] per frame, and the
//! trunk's height above the floor from the MJCF itself
//! ([`Model::trunk_height_m`]) — no transcribed constants.
//!
//! This lives in `kinematics`, not the `tof` crate, on purpose: it is pure
//! geometry, and the `tof` crate carries the vendored ST C driver that clients
//! like `robotctl` must never have to link.

use crate::head::HeadFk;
use crate::{Model, Pose, Quat};

/// The grid every VL53L5CX/L8CX ranges in 8×8 mode.
pub const ROWS: usize = 8;
pub const COLS: usize = 8;
const N_ZONES: usize = ROWS * COLS;

/// The sensor's square field of view, degrees per axis — 45°×45° per ST's
/// datasheet for both generations, the value the prototype's beam table used.
const FOV_DEG: f64 = 45.0;

/// What one zone's return turned out to be, once it has a place in the world.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Zone {
    /// No usable return — nothing in range, or the sensor said the
    /// measurement is not to be trusted.
    Empty,
    /// A return inside the short-range noise band ([`Reprojector::MIN_RANGE_M`]).
    TooClose,
    /// The beam reached the floor before anything stood in its way.
    Floor {
        /// Where it touched, in the trunk frame — for drawing the ground the
        /// robot has actually confirmed, not for avoiding.
        point: [f64; 3],
    },
    /// Something is there.
    Hit {
        /// The return, in the trunk frame, metres.
        point: [f64; 3],
        /// Horizontal distance from the trunk origin's vertical axis — the
        /// number obstacle avoidance compares against a stop threshold.
        range: f64,
    },
}

/// What the trunk itself is doing — the half of the geometry the head joints
/// cannot know. The floor filter reasons about *world* down; a trunk pitched
/// forward carries the whole head chain with it, and ignoring that labels the
/// floor an obstacle exactly when the robot leans at it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Posture {
    /// Projected gravity in the trunk frame, as `robot.state` reports it.
    /// Upright is about `[0, 0, -1]`.
    pub gravity: [f64; 3],
    /// Measured trunk height above the floor, metres — contact odometry's Z is
    /// exactly this. `None` falls back to the model's standing rest height.
    pub trunk_height_m: Option<f64>,
}

impl Default for Posture {
    /// A robot standing upright at its rest height.
    fn default() -> Self {
        Self {
            gravity: [0.0, 0.0, -1.0],
            trunk_height_m: None,
        }
    }
}

/// A depth frame as a 2D scan — what a mapper consumes. See
/// [`Reprojector::flatten`].
#[derive(Debug, Clone)]
pub struct FlatScan {
    /// Beam azimuths about the levelled body Z, radians.
    pub angles_body: Vec<f32>,
    /// Horizontal range from the sensor, metres. Parallel to `angles_body`.
    pub ranges: Vec<f32>,
    /// The sensor's levelled body-frame position the ranges start at.
    pub sensor_xy: (f32, f32),
}

pub struct Reprojector {
    fk: HeadFk,
    /// Unit beam directions in the sensor frame (+x forward, +y left, +z up),
    /// row-major like the wire frame.
    beams: [[f64; 3]; N_ZONES],
    trunk_height_m: f64,
}

impl Reprojector {
    /// Fraction of the sensor's height above the floor a downward beam must
    /// cover before it counts as a floor hit. Below 1.0 so FK and posture
    /// error make returns *near* the floor read as floor, not as obstacles —
    /// the prototype's tuned value.
    pub const FLOOR_SAFETY: f64 = 0.85;

    /// Returns closer than this (horizontally) are discarded: under ~10 cm the
    /// sensor's own crosstalk produces phantom short readings.
    pub const MIN_RANGE_M: f64 = 0.10;

    pub fn new(model: &'static Model) -> Self {
        // Zone centres, evenly spread over the FOV with a half-zone inset at
        // the edges (an 8-zone row has 8 centres, not 9 fenceposts). Row 0 is
        // the top of the grid, column 0 the sensor's left — matching how the
        // prototype's streamer ordered the very same ULD buffer.
        let half = (FOV_DEG / 2.0 - FOV_DEG / (COLS as f64) / 2.0).to_radians();
        let step = 2.0 * half / (COLS as f64 - 1.0);
        let mut beams = [[0.0; 3]; N_ZONES];
        for (i, beam) in beams.iter_mut().enumerate() {
            let elevation = half - (i / COLS) as f64 * step;
            let azimuth = half - (i % COLS) as f64 * step;
            *beam = [
                elevation.cos() * azimuth.cos(),
                elevation.cos() * azimuth.sin(),
                elevation.sin(),
            ];
        }
        Self {
            fk: HeadFk::new(model),
            beams,
            trunk_height_m: model.trunk_height_m(),
        }
    }

    pub fn alpha() -> Self {
        Self::new(Model::alpha())
    }

    /// The unit beam directions, in the sensor's own frame, row-major like the wire's
    /// zones. Exposed for geometry that reasons about the beams themselves rather than
    /// about where they landed — [`crate::hand`]'s plane fit, which needs the direction a
    /// slant range was measured along.
    pub fn beams(&self) -> &[[f64; 3]; N_ZONES] {
        &self.beams
    }

    /// The sensor's pose in the trunk frame at these head joints — for a
    /// consumer that wants the raw geometry (a point-cloud view, a map).
    pub fn sensor_in_trunk(&self, head_joints: [f64; 4]) -> Pose {
        self.fk.tof_in_trunk(head_joints)
    }

    /// One frame flattened to a 2D scan for mapping: per-obstacle azimuth
    /// and horizontal range measured **from the sensor**, in the
    /// gravity-levelled body frame — the frame odometry's yaw lives in, so
    /// a mapper can compose the two without caring how the trunk leans or
    /// where on the head the sensor sits. Floor and too-close returns are
    /// already filtered out; `sensor_xy` is the levelled body-frame sensor
    /// position the ranges start at.
    pub fn flatten(
        &self,
        ranges_m: &[Option<f64>; N_ZONES],
        head_joints: [f64; 4],
        posture: &Posture,
    ) -> FlatScan {
        let zones = self.project(ranges_m, head_joints, posture);
        let level = level_from_gravity(posture.gravity);
        let sensor = level.rotate(self.fk.tof_in_trunk(head_joints).pos);

        let mut angles = Vec::with_capacity(N_ZONES);
        let mut ranges = Vec::with_capacity(N_ZONES);
        for zone in zones {
            let Zone::Hit { point, .. } = zone else {
                continue;
            };
            let p = level.rotate(point);
            let (dx, dy) = (p[0] - sensor[0], p[1] - sensor[1]);
            let range = (dx * dx + dy * dy).sqrt();
            if range <= 0.0 {
                continue;
            }
            angles.push(dy.atan2(dx) as f32);
            ranges.push(range as f32);
        }
        FlatScan {
            angles_body: angles,
            ranges,
            sensor_xy: (sensor[0] as f32, sensor[1] as f32),
        }
    }

    /// Reproject one frame.
    ///
    /// `ranges_m` is the frame's 64 slant ranges in row-major order, `None`
    /// where the sensor reported nothing usable (the caller interprets the
    /// wire's status codes — that contract lives with the protocol, not here).
    /// `head_joints` = `[neck_pitch, head_pitch, head_yaw, head_roll]`,
    /// measured, from the same `robot.state` tick as near the frame as the
    /// caller has; `posture` is the trunk's own tilt and height from the same
    /// state. Points come out in the trunk frame; the floor and range
    /// *verdicts* are computed in the gravity-levelled frame, because "down"
    /// is the IMU's to say, not the trunk's.
    pub fn project(
        &self,
        ranges_m: &[Option<f64>; N_ZONES],
        head_joints: [f64; 4],
        posture: &Posture,
    ) -> [Zone; N_ZONES] {
        let sensor = self.fk.tof_in_trunk(head_joints);
        // Trunk → levelled: the rotation that takes the measured gravity to
        // straight down, so the filter's Z axis is the world's.
        let level = level_from_gravity(posture.gravity);
        let sensor_level = level.rotate(sensor.pos);
        let above_floor = sensor_level[2] + posture.trunk_height_m.unwrap_or(self.trunk_height_m);
        let floor_threshold = above_floor * Self::FLOOR_SAFETY;

        let mut zones = [Zone::Empty; N_ZONES];
        for (i, range) in ranges_m.iter().enumerate() {
            let Some(r) = *range else { continue };
            let dir = sensor.quat.rotate(self.beams[i]);
            let dir_level = level.rotate(dir);
            // Positive when the beam looks below the *world* horizon.
            let downward = -dir_level[2];
            let point = [
                sensor.pos[0] + r * dir[0],
                sensor.pos[1] + r * dir[1],
                sensor.pos[2] + r * dir[2],
            ];
            if above_floor > 0.0 && downward > 0.0 && r * downward >= floor_threshold {
                zones[i] = Zone::Floor { point };
                continue;
            }
            let horizontal = r * (dir_level[0] * dir_level[0] + dir_level[1] * dir_level[1]).sqrt();
            if horizontal < Self::MIN_RANGE_M {
                zones[i] = Zone::TooClose;
                continue;
            }
            zones[i] = Zone::Hit {
                point,
                range: horizontal,
            };
        }
        zones
    }
}

/// The rotation taking the trunk-frame gravity to straight down. Identity for
/// a gravity too small to trust — an IMU that has not converged should level
/// nothing rather than something random.
fn level_from_gravity(gravity: [f64; 3]) -> Quat {
    let n = (gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]).sqrt();
    if n < 0.5 {
        return Quat::IDENTITY;
    }
    let g = [gravity[0] / n, gravity[1] / n, gravity[2] / n];
    let down = [0.0, 0.0, -1.0f64];
    let axis = [
        g[1] * down[2] - g[2] * down[1],
        g[2] * down[0] - g[0] * down[2],
        g[0] * down[1] - g[1] * down[0],
    ];
    let s = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
    let c = g[0] * down[0] + g[1] * down[1] + g[2] * down[2];
    if s < 1e-9 {
        return if c > 0.0 {
            Quat::IDENTITY
        } else {
            // Hanging exactly upside down: any horizontal axis will do.
            Quat::from_axis_angle([1.0, 0.0, 0.0], std::f64::consts::PI)
        };
    }
    Quat::from_axis_angle([axis[0] / s, axis[1] / s, axis[2] / s], s.atan2(c))
}

#[cfg(test)]
mod tests {
    use super::*;

    const LEVEL: [f64; 4] = [0.0; 4];
    /// Centre-ish zone: row 3, column 3 — slightly up-left of the optical axis.
    const CENTRE: usize = 3 * COLS + 3;

    fn one_return(zone: usize, r: f64) -> [Option<f64>; N_ZONES] {
        let mut ranges = [None; N_ZONES];
        ranges[zone] = Some(r);
        ranges
    }

    /// A metre-out return near the optical axis of a level head lands about a
    /// metre forward of the trunk, at roughly the sensor's height — the sanity
    /// anchor for every convention in the pipeline at once.
    #[test]
    fn a_forward_return_lands_a_metre_ahead() {
        let rp = Reprojector::alpha();
        let zones = rp.project(&one_return(CENTRE, 1.0), LEVEL, &Posture::default());
        let Zone::Hit { point, range } = zones[CENTRE] else {
            panic!("expected a hit, got {:?}", zones[CENTRE]);
        };
        let sensor = rp.sensor_in_trunk(LEVEL);
        assert!((0.8..=1.2).contains(&point[0]), "x: {point:?}");
        assert!(point[1].abs() < 0.3, "y: {point:?}");
        assert!((point[2] - sensor.pos[2]).abs() < 0.3, "z: {point:?}");
        assert!((0.8..=1.05).contains(&range), "range: {range}");
    }

    /// Head pitched down: a bottom-row beam at floor distance is the floor;
    /// the same beam interrupted early is an obstacle. This is the filter's
    /// whole job in two returns.
    #[test]
    fn the_floor_is_floor_and_a_thing_before_it_is_not() {
        let rp = Reprojector::alpha();
        let looking_down = [0.0, 0.3, 0.0, 0.0];
        // A low row, centre column: a beam that reaches the floor at a
        // walking-relevant distance rather than at the robot's own feet.
        let zone = 6 * COLS + 3;

        let sensor = rp.sensor_in_trunk(looking_down);
        let above_floor = sensor.pos[2] + Model::alpha().trunk_height_m();
        let dir = sensor.quat.rotate(rp.beams[zone]);
        assert!(
            dir[2] < -0.3,
            "the test beam must look well below the horizon"
        );
        let to_floor = above_floor / -dir[2];

        let at_floor = rp.project(
            &one_return(zone, to_floor),
            looking_down,
            &Posture::default(),
        );
        let Zone::Floor { point } = at_floor[zone] else {
            panic!("expected the floor, got {:?}", at_floor[zone]);
        };
        assert!(
            (point[2] + Model::alpha().trunk_height_m()).abs() < 0.02,
            "a floor touch sits at floor height: z = {}",
            point[2]
        );

        let interrupted = rp.project(
            &one_return(zone, to_floor * 0.75),
            looking_down,
            &Posture::default(),
        );
        assert!(
            matches!(interrupted[zone], Zone::Hit { .. }),
            "three quarters of the way to the floor is an obstacle: {:?}",
            interrupted[zone]
        );
    }

    /// A level head's top-row beams look *up*: however far the return, it can
    /// never be called floor.
    #[test]
    fn an_upward_beam_is_never_the_floor() {
        let rp = Reprojector::alpha();
        let zones = rp.project(&one_return(3, 4.0), LEVEL, &Posture::default());
        assert!(
            matches!(zones[3], Zone::Hit { .. }),
            "an upward 4 m return must be a hit: {:?}",
            zones[3]
        );
    }

    #[test]
    fn the_noise_band_and_the_silence_are_named() {
        let rp = Reprojector::alpha();
        let zones = rp.project(&one_return(CENTRE, 0.05), LEVEL, &Posture::default());
        assert_eq!(zones[CENTRE], Zone::TooClose);
        assert_eq!(zones[0], Zone::Empty, "no return is no zone");
    }

    /// The flattened scan measures from the sensor, drops non-obstacles,
    /// and keeps azimuths near the beam's own: a mapper composing it with
    /// odometry must get the wall where the wall is.
    #[test]
    fn flatten_measures_from_the_sensor_and_keeps_only_obstacles() {
        let rp = Reprojector::alpha();
        let mut ranges = one_return(CENTRE, 1.0);
        ranges[CENTRE + 1] = Some(0.05); // too close — must not survive
        let flat = rp.flatten(&ranges, LEVEL, &Posture::default());

        assert_eq!(flat.angles_body.len(), 1, "only the obstacle survives");
        let sensor = rp.sensor_in_trunk(LEVEL).pos;
        assert!((flat.sensor_xy.0 - sensor[0] as f32).abs() < 1e-3);
        // Levelled-frame horizontal range from the sensor: close to the
        // slant range for a near-axis beam.
        assert!(
            (0.9..=1.0).contains(&flat.ranges[0]),
            "range {} should be the horizontal projection of 1.0",
            flat.ranges[0]
        );
        assert!(
            flat.angles_body[0].abs() < 0.2,
            "near-axis beam stays near-axis"
        );
    }

    /// A level head on a trunk *leaned* forward stares at the floor even
    /// though its joints say otherwise — the IMU is the only witness, and the
    /// filter must believe it. This is the hold-the-robot-tilted-by-hand case.
    #[test]
    fn a_leaning_trunk_makes_a_forward_beam_floor() {
        let rp = Reprojector::alpha();

        // Pitch the trunk 0.6 rad nose-down: gravity as the IMU would project it.
        let lean = Quat::from_axis_angle([0.0, 1.0, 0.0], 0.6);
        let gravity = lean.conjugate().rotate([0.0, 0.0, -1.0]);
        let leaned = Posture {
            gravity,
            trunk_height_m: None,
        };

        // A centre beam return at roughly where the floor now is.
        let sensor = rp.sensor_in_trunk(LEVEL);
        let dir = level_from_gravity(gravity).rotate(sensor.quat.rotate(rp.beams[CENTRE]));
        assert!(dir[2] < -0.3, "the leaned beam must point at the floor");
        let above =
            level_from_gravity(gravity).rotate(sensor.pos)[2] + Model::alpha().trunk_height_m();
        let to_floor = above / -dir[2];

        let leaned_zones = rp.project(&one_return(CENTRE, to_floor), LEVEL, &leaned);
        assert!(
            matches!(leaned_zones[CENTRE], Zone::Floor { .. }),
            "the IMU says this is the floor: {:?}",
            leaned_zones[CENTRE]
        );

        // The same return on an upright trunk is a thing ahead, not floor.
        let upright = rp.project(&one_return(CENTRE, to_floor), LEVEL, &Posture::default());
        assert!(
            matches!(upright[CENTRE], Zone::Hit { .. }),
            "upright, the same range is an obstacle: {:?}",
            upright[CENTRE]
        );
    }

    /// Yawing the head left must swing the reprojected point left (+y) in the
    /// trunk frame — the FK is actually in the loop, not a fixed mount.
    #[test]
    fn the_head_pose_steers_the_points() {
        let rp = Reprojector::alpha();
        let ahead = rp.project(&one_return(CENTRE, 1.0), LEVEL, &Posture::default());
        let left = rp.project(
            &one_return(CENTRE, 1.0),
            [0.0, 0.0, 0.8, 0.0],
            &Posture::default(),
        );
        let (Zone::Hit { point: a, .. }, Zone::Hit { point: l, .. }) =
            (ahead[CENTRE], left[CENTRE])
        else {
            panic!("both should hit");
        };
        assert!(
            l[1] > a[1] + 0.3,
            "a yawed-left head must place the return further left: {a:?} vs {l:?}"
        );
    }
}
