//! Mapping and localization for the duck, onboard, from the head ToF and the legs.
//!
//! The sensor is a cane, not a lidar: 64 zones over 45°, most valid returns inside a metre. So
//! the design leans on what the duck has instead — odometry whose heading is the gyro's, the
//! ability to stop and look around, and patience. A **stop** is the unit of mapping: frames from
//! one standing place are voted into one rigid keyframe, odometry links consecutive keyframes
//! with an uncertainty that grows with what was walked, and scan matching adds edges only in the
//! directions a scan actually pins. The map is rendered from keyframes at their optimized poses,
//! never inked once and kept.
//!
//!   input    — `robot.state` and `tof.frame` reduced, and time-aligned on the shared clock
//!   project  — a depth frame into obstacles and floor-level free space, by height
//!   keyframe — a stop's frames voted into one local scan
//!   field    — a map region prepared for scoring a scan
//!   matcher  — correlative search with a per-direction covariance
//!   graph    — the pose graph, sparse Levenberg–Marquardt with a robust kernel
//!   mapper   — the loop: stops, tracking, loop closure, losing and finding itself, islands
//!   grid     — the occupancy grid a planner or a viewer reads
//!   sim      — a synthetic world with ground truth, for the tests
//!
//! The first design (`origin/maploc`, 2026-08) is the reason for most of these choices; each
//! module's doc says which of its failures it exists to prevent.

// The graph, field and matcher kernels index matrices and grids on purpose: the loops mirror the
// maths they implement (H[r][c], row-major sweeps), and iterator rewrites of numeric kernels hide
// the indices the comments speak in.
#![allow(clippy::needless_range_loop)]
pub mod field;
pub mod graph;
pub mod grid;
pub mod input;
pub mod keyframe;
pub mod mapper;
pub mod matcher;
pub mod project;
pub mod se2;
pub mod sim;
