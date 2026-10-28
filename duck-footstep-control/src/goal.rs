//! Walk to a goal pose: FootstepNet replanned at every touchdown, from wherever the feet actually landed.

use crate::follower::Follower;
use crate::pattern::stance_frame;
use crate::planner::{Planner, PlannerError, StepNet};
use crate::pose::{Foot, Pose2, compose, relative};

/// Touchdowns before a goal is given up on. FootstepNet reaches ±1 m goals in ~22 steps (99–100 % in training);
/// three times that is a plan going round in circles, not a long walk.
const MAX_TOUCHDOWNS: u32 = 70;

/// How the goal ended: reached (with the goal foot's final miss, m and rad) or given up.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GoalOutcome {
    Reached {
        distance: f64,
        heading: f64,
        touchdowns: u32,
    },
    GaveUp {
        touchdowns: u32,
    },
}

/// A goal in the robot's frame at the time it was asked: the stance frame (midway between the sole centres,
/// their mean heading — the pattern frame). `foot` places that foot's sole centre on the goal; `None` places the
/// stance centre there (the left foot beside it, then the right squares up).
#[derive(Debug, Clone)]
pub struct GoalRun {
    goal: Pose2,
    foot: Option<Foot>,
    /// `goal` is already in the odometry's world frame, not the stance frame at the start.
    in_world: bool,
    world: Option<(Pose2, Foot)>,
    touchdowns: u32,
    /// The goal foot landed on the goal; the other is stepping up beside it.
    squaring: bool,
    outcome: Option<GoalOutcome>,
}

impl GoalRun {
    pub fn new(goal: Pose2, foot: Option<Foot>) -> Self {
        Self {
            goal,
            foot,
            in_world: false,
            world: None,
            touchdowns: 0,
            squaring: false,
            outcome: None,
        }
    }

    /// A goal given in the odometry's world frame (where the soles are), e.g. a tag the camera located.
    pub fn in_world(goal: Pose2, foot: Option<Foot>) -> Self {
        Self {
            in_world: true,
            ..Self::new(goal, foot)
        }
    }

    /// How it ended, once [`Self::tick`] has returned `Ok(None)`.
    pub fn outcome(&self) -> Option<GoalOutcome> {
        self.outcome
    }

    /// The goal (world pose, foot), once anchored.
    pub fn world_goal(&self) -> Option<(Pose2, Foot)> {
        self.world
    }

    /// One control tick: the policy command, or `Ok(None)` when done (at a touchdown). Anchors on the first tick.
    ///
    /// At each touchdown FootstepNet plans from the support sole to the goal and the swing foot gets the plan's
    /// first step. A plan with no step means the goal foot is standing on the goal: the follower then steps the
    /// other foot to its neutral place beside it, and that touchdown ends the run.
    pub fn tick<N: StepNet>(
        &mut self,
        follower: &mut Follower,
        planner: &mut Planner<N>,
        support: Foot,
        soles: [Pose2; 2],
    ) -> Result<Option<[f64; 3]>, PlannerError> {
        let (goal, goal_foot) = *self.world.get_or_insert_with(|| {
            let anchor = if self.in_world {
                [0.0; 3]
            } else {
                stance_frame(soles)
            };
            follower.reset();
            match self.foot {
                Some(foot) => (compose(anchor, self.goal), foot),
                None => {
                    let half = planner.options().feet_spacing / 2.0;
                    (
                        compose(compose(anchor, self.goal), [0.0, half, 0.0]),
                        Foot::Left,
                    )
                }
            }
        });
        let command =
            follower.update(planner, support, soles, |_, _| (goal, goal_foot), None, 50)?;
        if !follower.switched() {
            return Ok(Some(command));
        }
        self.touchdowns += 1;
        if self.squaring {
            let miss = relative(goal, soles[goal_foot.index()]);
            self.outcome = Some(GoalOutcome::Reached {
                distance: miss[0].hypot(miss[1]),
                heading: miss[2].abs(),
                touchdowns: self.touchdowns,
            });
            return Ok(None);
        }
        if follower.plan().steps.is_empty() {
            self.squaring = true; // the goal foot is on the goal; the swing foot is going to its neutral place
        } else if self.touchdowns > MAX_TOUCHDOWNS {
            self.outcome = Some(GoalOutcome::GaveUp {
                touchdowns: self.touchdowns,
            });
            return Ok(None);
        }
        Ok(Some(command))
    }
}
