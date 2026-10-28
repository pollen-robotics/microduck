//! Walking on planned footsteps: the planner loop, and the follower that feeds the footstep policy.
//!
//! The contract is microduck_rl's (`src/mjlab_microduck/footsteps.py`, Hub `apirrone/microduck-footstepnet`):
//!
//! - A pose is the planar `[x, y, yaw]` of a **sole centre** (7 mm ahead of the `*_foot` site), x forward.
//! - The planner network takes an 8-float observation of the goal and an obstacle as seen from the support
//!   foot, and returns a step `(dx, dy, dθ)` relative to the **neutral** landing pose (feet parallel,
//!   `feet_spacing` apart), written as if the support foot were the right one: a left support mirrors y and θ.
//! - The footstep policy reads, in its twist slot, the swing foot's target pose in the current support sole frame.
//! - The support foot is whichever foot the contact odometry anchors on; a switch is a touchdown.
//!
//! [`Planner`] runs the network in that loop; [`Follower`] replans at every touchdown toward a goal — a fixed pose,
//! or the joystick's carrot ([`carrot_goal`]) — and produces the policy command each tick.

mod follower;
mod goal;
mod pattern;
mod planner;
mod pose;

pub use follower::{CarrotParams, Follower, carrot_goal, stick_step};
pub use goal::{GoalOutcome, GoalRun};
pub use pattern::{Miss, PatternRun, stance_frame};
pub use planner::{Obstacle, Plan, Planner, PlannerError, PlannerOptions, StepNet};
pub use pose::{Foot, Pose2, compose, relative, wrap};
