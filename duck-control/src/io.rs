//! The seam between the control loop and the physical world.
//!
//! Everything between [`RobotIo::read`] and [`RobotIo::write`] is pure computation over
//! plain data, which is what makes the loop testable without a robot.
//!
//! [`Sensors`] carries joints *and* IMU together because that is what the hardware does:
//! the IMU board sits on the Dynamixel bus and is fetched in the same transaction as the
//! servos. A trait that split them would invent a distinction the bus does not have, and
//! would double the bus traffic to honour it.

use crate::imu::ImuData;
use crate::model::NUM_JOINTS;

/// One atomic sample of the robot.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sensors {
    /// Joint angles in radians, indexed as [`crate::model::JOINT_NAMES`].
    pub positions: [f64; NUM_JOINTS],
    /// Joint velocities, rad/s.
    pub velocities: [f64; NUM_JOINTS],
    /// Present current magnitude, mA. Sign is dropped — direction is inferable from
    /// velocity, and every consumer so far wants load, not direction.
    pub currents_ma: [f64; NUM_JOINTS],
    pub imu: ImuData,
}

impl Default for Sensors {
    fn default() -> Self {
        Self {
            positions: [0.0; NUM_JOINTS],
            velocities: [0.0; NUM_JOINTS],
            currents_ma: [0.0; NUM_JOINTS],
            imu: ImuData::default(),
        }
    }
}

/// What the loop commands. Position control only — alpha has no velocity-mode joints.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointTargets {
    pub positions: [f64; NUM_JOINTS],
}

impl JointTargets {
    pub fn new(positions: [f64; NUM_JOINTS]) -> Self {
        Self { positions }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum IoError {
    #[error("serial port {path}: {source}")]
    Port {
        path: String,
        #[source]
        source: std::io::Error,
    },
    #[error("bus transaction failed: {0}")]
    Bus(String),
    /// A `sync_read` that returns the wrong number of blocks, or a block of the wrong
    /// length, means a device did not answer. Reported rather than papered over: a
    /// silently short read would leave stale values in half the joint array.
    #[error("{what}: expected {expected}, got {got}")]
    ShortRead {
        what: &'static str,
        expected: usize,
        got: usize,
    },
    #[error("simulated failure")]
    Simulated,
}

pub type Result<T> = std::result::Result<T, IoError>;

pub trait RobotIo {
    /// One transaction: joints and IMU together.
    fn read(&mut self) -> Result<Sensors>;
    fn write(&mut self, targets: &JointTargets) -> Result<()>;
}

/// A robot made of nothing, for tests.
///
/// Always compiled — it is what the test suite runs against, and it is why `cargo test`
/// needs no hardware, no network and no Docker. Positions echo back whatever was last
/// written, so a loop driving it behaves like a servo that tracks perfectly.
pub struct FakeIo {
    sensors: Sensors,
    /// Set to make the next [`RobotIo::read`] fail, then cleared. For exercising the
    /// loop's error path without a flaky bus.
    pub fail_next_read: bool,
    pub last_written: Option<JointTargets>,
    pub reads: usize,
    pub writes: usize,
    /// When true, `read` reports the last written targets as the present positions.
    track_targets: bool,
}

impl Default for FakeIo {
    fn default() -> Self {
        Self::new()
    }
}

impl FakeIo {
    pub fn new() -> Self {
        Self {
            sensors: Sensors::default(),
            fail_next_read: false,
            last_written: None,
            reads: 0,
            writes: 0,
            track_targets: true,
        }
    }

    /// Start from a known pose — typically [`crate::model::DEFAULT_POSITION`].
    pub fn at(positions: [f64; NUM_JOINTS]) -> Self {
        let mut io = Self::new();
        io.sensors.positions = positions;
        io
    }

    /// Freeze reported positions so they ignore what is written. Models a robot whose
    /// servos are limp, or one being pushed around by hand.
    pub fn frozen(mut self) -> Self {
        self.track_targets = false;
        self
    }

    pub fn set_imu(&mut self, imu: ImuData) {
        self.sensors.imu = imu;
    }

    pub fn positions(&self) -> [f64; NUM_JOINTS] {
        self.sensors.positions
    }
}

impl RobotIo for FakeIo {
    fn read(&mut self) -> Result<Sensors> {
        if self.fail_next_read {
            self.fail_next_read = false;
            return Err(IoError::Simulated);
        }
        self.reads += 1;
        Ok(self.sensors)
    }

    fn write(&mut self, targets: &JointTargets) -> Result<()> {
        self.writes += 1;
        self.last_written = Some(*targets);
        if self.track_targets {
            self.sensors.positions = targets.positions;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::DEFAULT_POSITION;

    /// The loop's whole contract in slice 1: whatever it writes, it reads back. A FakeIo
    /// that ignored writes would make every hold-pose test pass vacuously.
    #[test]
    fn fake_io_tracks_what_was_written() {
        let mut io = FakeIo::at(DEFAULT_POSITION);
        assert_eq!(io.read().unwrap().positions, DEFAULT_POSITION);

        let mut moved = DEFAULT_POSITION;
        moved[0] = 0.5;
        io.write(&JointTargets::new(moved)).unwrap();
        assert_eq!(io.read().unwrap().positions, moved);
        assert_eq!(io.writes, 1);
    }

    /// A limp or hand-held robot does not follow commands. Slice 2's safety work needs to
    /// be testable against that, so the divergence has to be expressible.
    #[test]
    fn frozen_fake_io_ignores_writes() {
        let mut io = FakeIo::at(DEFAULT_POSITION).frozen();
        let mut moved = DEFAULT_POSITION;
        moved[0] = 0.5;
        io.write(&JointTargets::new(moved)).unwrap();
        assert_eq!(io.read().unwrap().positions, DEFAULT_POSITION);
    }

    /// A read failure must be a one-shot, or a test that injects one can never recover and
    /// the loop's retry path is untestable.
    #[test]
    fn simulated_read_failure_clears_itself() {
        let mut io = FakeIo::new();
        io.fail_next_read = true;
        assert!(io.read().is_err());
        assert!(io.read().is_ok());
    }
}
