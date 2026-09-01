//! Submap-based pose-graph SLAM + boot-time relocalization for the duck.
//!
//! Absorbed from the `microduck_maploc_rs` satellite. The algorithm is the
//! prototype's (see each module's doc): odometry tracks between submap
//! freezes, scan matching runs only at submap-to-submap granularity (loop
//! closure), a dense Gauss-Newton optimizer relaxes the SE(2) anchor graph,
//! and a particle filter relocalizes against a saved map on boot.
//!
//! What changed in the port, beyond the trimmings the manifest lists:
//! every scan-consuming API takes the beam origin explicitly (see
//! [`Scan`]) — the prototype inked maps from the *sensor* pose but scored
//! MCL particles and relocalize candidates from the *body* pose, a ~10–15 cm
//! systematic disagreement between map and matcher on a head-mounted sensor.
//!
//! Module map:
//!
//!   accumulator    — still-window frame voting: many noisy frames → one scan
//!   grid           — 2D log-odds occupancy + cached distance field
//!   submap         — local grid + SE(2) anchor pose + retained raw scans
//!   submap_manager — open / freeze submaps based on time + travel
//!   scan_matcher   — Hector-style GN scan-to-map matching
//!   pose_graph     — SE(2) nodes + relative-pose edges
//!   optimizer      — dense Gauss-Newton over the full graph
//!   pipeline       — the assembled SLAM loop a host drives (robotd, bench)
//!   loop_closer    — coarse-to-fine submap-to-submap loop matching
//!   global_render  — composite all submaps into one grid
//!   mcl            — particle-filter relocalize against a saved map
//!   relocalize     — one-shot brute-force pose search (diagnostic / seeding)
//!   planner        — A* + inflation + line-of-sight simplification
//!   follower       — turn-then-go waypoint follower (velocity output)
//!   session        — save/load the whole SLAM state (fsynced, atomic)
//!   replay         — read back the prototype's .mdlg logs for offline work
//!   rng            — the pinned RNG determinism rides on

// The optimizer, grid and matcher kernels iterate matrices and grids by
// index on purpose: the loops mirror the maths they implement (H[r][c],
// row-major sweeps), and iterator rewrites of numeric kernels hide the
// indices the comments and papers speak in.
#![allow(clippy::needless_range_loop)]

pub mod accumulator;
pub mod follower;
pub mod global_render;
pub mod grid;
pub mod loop_closer;
pub mod mapper;
pub mod mcl;
pub mod optimizer;
pub mod pipeline;
pub mod planner;
pub mod pose_graph;
pub mod record;
pub mod relocalize;
pub mod replay;
pub mod rng;
pub mod scan_matcher;
pub mod session;
pub mod submap;
pub mod submap_manager;

pub use grid::{GridConfig, OccupancyGrid};
pub use submap::{Pose2, Scan};
