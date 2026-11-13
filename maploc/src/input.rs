//! What the mapper consumes, and the short history that lets a depth frame meet the robot state
//! of its own instant.
//!
//! Both inputs come off sockets at their own rates — `robot.state` at the control loop's 50 Hz,
//! `tof.frame` at ~15 Hz — and both carry `t_ns` on `CLOCK_MONOTONIC`. The first maploc paired
//! each frame with whichever state sample happened to be newest, which during the stop's head
//! sweep (~0.6 rad/s) put walls 1–2° off in a direction that flipped with the sweep: every wall
//! drawn twice. Here a frame waits until the state history brackets its capture time and is
//! placed with the interpolated pose.

use std::collections::VecDeque;

use duck_ipc_proto::{RobotState, TofFrame};
use kinematics::{Pose, Quat};

use crate::se2::{Pose2, wrap};

/// What the robot is doing, as far as mapping cares.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Activity {
    /// Somebody is asking it to move. Not the same as moving — a pad stick idles slightly off
    /// zero, which is why this is decided with a threshold, and why stillness is also checked
    /// against odometry itself.
    pub commanded: bool,
    /// Seated, or on the way there or back — the sit/stand network is driving.
    pub sitting: bool,
    pub fallen: bool,
    /// The pick-up detector says somebody is holding it. Its legs are in the air, so odometry
    /// position is meaningless until it is put down; the gyro's heading still holds.
    pub picked_up: bool,
    /// No policy is driving (`held`): limp, paused, or a bench configuration.
    pub unpowered: bool,
}

/// One `robot.state` tick, reduced to what mapping uses.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StateSample {
    pub t_ns: u64,
    /// Contact odometry: trunk position in the odometry frame, z is height above the floor.
    pub trunk_pos: [f64; 3],
    /// Odometry heading — the IMU's integrated yaw.
    pub odom_yaw: f64,
    /// Trunk orientation, trunk → world, from the IMU. Its yaw is `odom_yaw`; the pitch and roll
    /// are what put a head-mounted beam on the floor or over it.
    pub trunk_quat: Quat,
    /// The ToF sensor in the trunk frame at this tick's measured head joints.
    pub tof_in_trunk: Pose,
    pub activity: Activity,
}

/// A commanded twist below these is a stick at rest, not a request to walk.
const TWIST_DEADBAND_LIN: f64 = 0.02;
const TWIST_DEADBAND_ANG: f64 = 0.05;

impl StateSample {
    /// `None` for a frame that cannot place a beam: a `robotd` predating the v24 telemetry, or a
    /// tick before the sensors were first read.
    pub fn from_proto(s: &RobotState) -> Option<Self> {
        let imu = s.imu?;
        let frames = s.frames?;
        if s.t_ns == 0 {
            return None;
        }
        let q = |w: [f64; 4]| Quat::new(w[0], w[1], w[2], w[3]).normalized();
        let r = s.movement.requested;
        let commanded = r[0].hypot(r[1]) > TWIST_DEADBAND_LIN || r[2].abs() > TWIST_DEADBAND_ANG;
        Some(Self {
            t_ns: s.t_ns,
            trunk_pos: s.odom.position,
            odom_yaw: s.odom.yaw,
            trunk_quat: q(imu.quat),
            tof_in_trunk: Pose::new(frames.tof.pos, q(frames.tof.quat)),
            activity: Activity {
                commanded,
                sitting: s.policy == "sit",
                fallen: s.safety.fallen,
                picked_up: s.safety.picked_up,
                unpowered: s.policy == "held",
            },
        })
    }

    /// Where the trunk stands on the floor, in the odometry frame.
    pub fn odom_pose(&self) -> Pose2 {
        Pose2::new(self.trunk_pos[0], self.trunk_pos[1], self.odom_yaw)
    }

    /// The trunk in the odometry frame, in 3D.
    pub fn trunk_pose(&self) -> Pose {
        Pose::new(self.trunk_pos, self.trunk_quat)
    }
}

/// One `tof.frame`, reduced to what mapping uses.
#[derive(Debug, Clone, PartialEq)]
pub struct DepthFrame {
    pub t_ns: u64,
    pub distance_mm: Vec<i16>,
    pub status: Vec<u8>,
}

impl DepthFrame {
    /// `None` for a frame this mapper cannot place: no capture clock (a `tofd` predating v24),
    /// or not the 8×8 grid the beam model describes.
    pub fn from_proto(f: &TofFrame) -> Option<Self> {
        let n = kinematics::tof::ROWS * kinematics::tof::COLS;
        if f.t_ns == 0 || f.distance_mm.len() != n || f.status.len() != n {
            return None;
        }
        Some(Self {
            t_ns: f.t_ns,
            distance_mm: f.distance_mm.clone(),
            status: f.status.clone(),
        })
    }
}

/// The last couple of seconds of state, for interpolating at a depth frame's capture time.
#[derive(Debug, Default)]
pub struct History {
    samples: VecDeque<StateSample>,
}

/// Longer than any depth frame can lag its state, short enough to stay a few hundred samples.
const HISTORY_NS: u64 = 2_000_000_000;

/// Two state samples further apart than this do not bracket anything — the stream stalled, and a
/// pose interpolated across the gap is invented.
const MAX_GAP_NS: u64 = 100_000_000;

impl History {
    pub fn push(&mut self, s: StateSample) {
        if let Some(last) = self.samples.back()
            && s.t_ns <= last.t_ns
        {
            // Out of order or duplicated: the stream is monotonic by contract, so this is a
            // restarted robotd. Start over rather than interleave two clocks' worth of poses.
            if s.t_ns < last.t_ns {
                self.samples.clear();
            } else {
                return;
            }
        }
        self.samples.push_back(s);
        while let Some(front) = self.samples.front() {
            if s.t_ns - front.t_ns > HISTORY_NS {
                self.samples.pop_front();
            } else {
                break;
            }
        }
    }

    pub fn latest(&self) -> Option<&StateSample> {
        self.samples.back()
    }

    /// Whether a frame captured at `t_ns` can still be bracketed later, or never will be.
    pub fn covers(&self, t_ns: u64) -> Coverage {
        match (self.samples.front(), self.samples.back()) {
            (Some(f), Some(b)) if t_ns >= f.t_ns && t_ns <= b.t_ns => Coverage::Now,
            (Some(_), Some(b)) if t_ns > b.t_ns => Coverage::Later,
            _ => Coverage::Never,
        }
    }

    /// The state at `t_ns`, interpolated between the two samples around it. Activity flags come
    /// from the earlier sample — they are levels, and the earlier one is what was true when the
    /// interval began.
    pub fn at(&self, t_ns: u64) -> Option<StateSample> {
        let idx = self.samples.partition_point(|s| s.t_ns <= t_ns);
        if idx == 0 {
            return None;
        }
        let a = &self.samples[idx - 1];
        if a.t_ns == t_ns {
            return Some(*a);
        }
        let b = self.samples.get(idx)?;
        if b.t_ns - a.t_ns > MAX_GAP_NS {
            return None;
        }
        let u = (t_ns - a.t_ns) as f64 / (b.t_ns - a.t_ns) as f64;
        let lerp = |p: [f64; 3], q: [f64; 3]| {
            [
                p[0] + u * (q[0] - p[0]),
                p[1] + u * (q[1] - p[1]),
                p[2] + u * (q[2] - p[2]),
            ]
        };
        Some(StateSample {
            t_ns,
            trunk_pos: lerp(a.trunk_pos, b.trunk_pos),
            odom_yaw: wrap(a.odom_yaw + u * wrap(b.odom_yaw - a.odom_yaw)),
            trunk_quat: nlerp(a.trunk_quat, b.trunk_quat, u),
            tof_in_trunk: Pose::new(
                lerp(a.tof_in_trunk.pos, b.tof_in_trunk.pos),
                nlerp(a.tof_in_trunk.quat, b.tof_in_trunk.quat, u),
            ),
            activity: a.activity,
        })
    }

    /// Planar odometry speed over roughly the last `window_ns`: distance between the newest
    /// sample and the oldest one still inside the window, over the time between them.
    pub fn recent_speed(&self, window_ns: u64) -> Option<(f64, f64)> {
        let b = self.samples.back()?;
        let a = self
            .samples
            .iter()
            .find(|s| b.t_ns - s.t_ns <= window_ns)
            .filter(|a| a.t_ns < b.t_ns)?;
        let dt = (b.t_ns - a.t_ns) as f64 * 1e-9;
        let lin = (b.trunk_pos[0] - a.trunk_pos[0]).hypot(b.trunk_pos[1] - a.trunk_pos[1]) / dt;
        let ang = wrap(b.odom_yaw - a.odom_yaw).abs() / dt;
        Some((lin, ang))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Coverage {
    Now,
    Later,
    Never,
}

/// Normalized linear interpolation — indistinguishable from slerp across the 20 ms between two
/// control ticks, and it never divides by a vanishing sine.
fn nlerp(a: Quat, b: Quat, u: f64) -> Quat {
    let dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    let s = if dot < 0.0 { -1.0 } else { 1.0 };
    Quat::new(
        a.w + u * (s * b.w - a.w),
        a.x + u * (s * b.x - a.x),
        a.y + u * (s * b.y - a.y),
        a.z + u * (s * b.z - a.z),
    )
    .normalized()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(t_ns: u64, x: f64, yaw: f64) -> StateSample {
        StateSample {
            t_ns,
            trunk_pos: [x, 0.0, 0.12],
            odom_yaw: yaw,
            trunk_quat: Quat::from_axis_angle([0.0, 0.0, 1.0], yaw),
            tof_in_trunk: Pose::IDENTITY,
            activity: Activity::default(),
        }
    }

    #[test]
    fn interpolates_between_bracketing_samples() {
        let mut h = History::default();
        h.push(sample(1_000_000_000, 0.0, 3.1));
        h.push(sample(1_020_000_000, 0.02, -3.1));
        let s = h.at(1_010_000_000).unwrap();
        assert!((s.trunk_pos[0] - 0.01).abs() < 1e-12);
        // Across ±π the midpoint is π, not 0: interpolating the raw numbers would spin the
        // robot round for one frame every time it faced backwards.
        assert!(wrap(s.odom_yaw - std::f64::consts::PI).abs() < 1e-9);
        assert!(wrap(s.trunk_quat.yaw() - std::f64::consts::PI).abs() < 1e-9);
    }

    #[test]
    fn a_frame_newer_than_the_history_waits_and_a_gap_is_not_bridged() {
        let mut h = History::default();
        h.push(sample(1_000_000_000, 0.0, 0.0));
        assert_eq!(h.covers(1_010_000_000), Coverage::Later);
        assert_eq!(h.covers(900_000_000), Coverage::Never);
        h.push(sample(1_500_000_000, 1.0, 0.0));
        // Half a second without state: whatever the robot did in between, a straight line is
        // not it.
        assert!(h.at(1_250_000_000).is_none());
    }
}
