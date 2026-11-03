//! The two things that keep a lap off the walls: a square on odometry, and the head ToF.
//!
//! Neither is navigation. A guard cuts a walking step short and points the robot back; the lap
//! goes on at its next step. Odometry drifts with every fall (contact odometry keeps integrating
//! a robot flailing on the floor), so the square is coarse; the ToF is the one that actually sees
//! the wall.

use duck_ipc_proto as proto;
use kinematics::tof::{Reprojector, Zone};

/// Half the width of the strip ahead that counts: the robot's own width and a margin.
pub const CORRIDOR_HALF_WIDTH: f64 = 0.15;

/// The nearest obstacle ahead, horizontal metres from the trunk: returns in front
/// (`x > 0`) and inside `half_width` of the centre line. Floor, too-close and empty zones are
/// not obstacles.
pub fn nearest_ahead(zones: &[Zone], half_width: f64) -> Option<f64> {
    zones
        .iter()
        .filter_map(|zone| match zone {
            Zone::Hit { point, range } if point[0] > 0.0 && point[1].abs() <= half_width => {
                Some(*range)
            }
            _ => None,
        })
        .min_by(f64::total_cmp)
}

/// A frame's zones in the trunk frame, through the head FK — `robotctl monitor`'s reduction.
pub fn classify(
    frame: &proto::TofFrame,
    state: &proto::RobotState,
    r: &Reprojector,
) -> Option<Vec<Zone>> {
    crate::monitor::classify_tof(frame, state, r).map(|zones| zones.to_vec())
}

/// Whether `pose` is outside the square of half-side `half` around `home`.
pub fn outside_square(pose: (f64, f64), home: (f64, f64), half: f64) -> bool {
    (pose.0 - home.0).abs() > half || (pose.1 - home.1).abs() > half
}

/// Odometry moving further than this between two reads is not the robot walking: it is
/// `robotd` restarting and integrating from zero again.
const ODOMETRY_JUMP: f64 = 0.5;

/// Whether odometry went from `before` to `after` in a way no walk could, between two reads.
pub fn odometry_jumped(before: (f64, f64), after: (f64, f64)) -> bool {
    (after.0 - before.0).hypot(after.1 - before.1) > ODOMETRY_JUMP
}

fn wrap(angle: f64) -> f64 {
    angle.sin().atan2(angle.cos())
}

/// The twist that walks from `pose` (x, y, yaw) to `target`: on the spot at full rate while the
/// heading is off by more than `turn_threshold` rad, else full speed with a proportional turn.
pub fn toward(
    pose: (f64, f64, f64),
    target: (f64, f64),
    speed: f64,
    vyaw: f64,
    turn_threshold: f64,
) -> (f64, f64) {
    let (x, y, yaw) = pose;
    let err = wrap((target.1 - y).atan2(target.0 - x) - yaw);
    if err.abs() > turn_threshold {
        (0.0, vyaw.copysign(err))
    } else {
        (speed, (2.5 * err).clamp(-vyaw, vyaw))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use kinematics::tof::Zone;

    fn hit(x: f64, y: f64) -> Zone {
        Zone::Hit {
            point: [x, y, 0.0],
            range: x.hypot(y),
        }
    }

    #[test]
    fn a_hit_straight_ahead_is_the_nearest() {
        let zones = [hit(0.40, 0.0), hit(0.18, 0.0), Zone::Empty];
        assert_eq!(nearest_ahead(&zones, 0.15), Some(0.18));
    }

    #[test]
    fn hits_off_the_corridor_and_behind_are_ignored() {
        assert_eq!(
            nearest_ahead(&[hit(0.10, 0.30), hit(-0.10, 0.0)], 0.15),
            None
        );
    }

    #[test]
    fn floor_and_too_close_are_not_walls() {
        let zones = [
            Zone::Floor {
                point: [0.1, 0.0, -0.2],
            },
            Zone::TooClose,
            Zone::Empty,
        ];
        assert_eq!(nearest_ahead(&zones, 0.15), None);
        assert_eq!(nearest_ahead(&[], 0.15), None);
    }

    #[test]
    fn the_square_is_half_a_metre_each_way() {
        assert!(!outside_square((0.49, 0.49), (0.0, 0.0), 0.5));
        assert!(outside_square((0.51, 0.0), (0.0, 0.0), 0.5));
        assert!(outside_square((0.0, -0.51), (0.0, 0.0), 0.5));
        assert!(!outside_square((1.2, 1.0), (1.0, 1.0), 0.5));
    }

    #[test]
    fn toward_turns_first_then_walks() {
        // Target behind: turn on the spot, full rate, the short way round.
        assert_eq!(
            toward((0.0, 0.0, 0.0), (-1.0, 0.1), 0.3, 1.5, 0.6),
            (0.0, 1.5)
        );
        assert_eq!(
            toward((0.0, 0.0, 0.0), (-1.0, -0.1), 0.3, 1.5, 0.6),
            (0.0, -1.5)
        );
        // Target ahead: full speed, small correction.
        let (vx, vyaw) = toward((0.0, 0.0, 0.0), (1.0, 0.1), 0.3, 1.5, 0.6);
        assert_eq!(vx, 0.3);
        assert!(vyaw > 0.0 && vyaw < 1.5);
    }

    #[test]
    fn a_jump_no_walk_could_make_is_an_odometry_reset() {
        // 50 ms apart at full stick is under 2 cm; a restarted robotd starts again from zero.
        assert!(!odometry_jumped((1.00, 0.50), (1.02, 0.50)));
        assert!(odometry_jumped((5.0, 3.0), (0.0, 0.0)));
        assert!(odometry_jumped((0.0, 0.0), (0.0, 0.6)));
    }
}
