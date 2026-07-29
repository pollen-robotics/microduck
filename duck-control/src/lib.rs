//! The robot control core: everything between reading the bus and writing it.
//!
//! Deliberately not a daemon. There is no tokio here, no socket, no systemd — `robotd`
//! owns all of that. The boundary is enforced by the compiler rather than by discipline,
//! which is what stops process concerns leaking into the code that drives motors.
//!
//! Slice 1 (`docs/robotd-design.md` §4) is the model, the bus, and the [`io::RobotIo`]
//! seam. Observations, the policy runner and the safety layer arrive with slice 2.

pub mod bus;
pub mod imu;
pub mod io;
pub mod model;

pub use imu::ImuData;
pub use io::{FakeIo, IoError, JointTargets, RobotIo, Sensors};
pub use model::{DEFAULT_POSITION, JOINT_IDS, JOINT_NAMES, NUM_JOINTS};
