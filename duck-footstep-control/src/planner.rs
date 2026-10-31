//! FootstepNet's planning loop around a step network (gym_footsteps_planning's `FootstepsPlanningEnv`).

use serde::Deserialize;

use crate::pose::{Foot, Pose2, compose, relative};

/// The planner's geometry and limits — the `options` JSON in the planner ONNX's metadata, written by
/// microduck_rl `footstepnet/export_onnx.py`. The network was trained with exactly these, so they are read from
/// the file rather than configured.
#[derive(Debug, Clone, Deserialize)]
pub struct PlannerOptions {
    pub max_dx_forward: f64,
    pub max_dx_backward: f64,
    pub max_dy: f64,
    pub max_dtheta: f64,
    pub tolerance_distance: f64,
    pub tolerance_angle: f64,
    pub obstacle_max_radius: f64,
    pub foot_length: f64,
    pub foot_width: f64,
    pub feet_spacing: f64,
}

impl PlannerOptions {
    pub fn from_json(json: &str) -> Result<Self, PlannerError> {
        serde_json::from_str(json).map_err(|e| PlannerError::Options(e.to_string()))
    }

    /// A neutral-relative step clipped to these limits as FootstepNet's env clips them: the box (± forward
    /// limit, ± `max_dy`, ± `max_dtheta`), then the ellipsoid with the backward limit behind.
    pub fn clip(&self, step: [f64; 3]) -> [f64; 3] {
        let high = [self.max_dx_forward, self.max_dy, self.max_dtheta];
        let mut a = step;
        for (v, h) in a.iter_mut().zip(high) {
            *v = v.clamp(-h, h);
        }
        let factor = [
            if a[0] >= 0.0 {
                self.max_dx_forward
            } else {
                self.max_dx_backward
            },
            self.max_dy,
            self.max_dtheta,
        ];
        let norm = a
            .iter()
            .zip(factor)
            .map(|(v, f)| (v / f).powi(2))
            .sum::<f64>()
            .sqrt();
        if norm > 1.0 { a.map(|v| v / norm) } else { a }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum PlannerError {
    #[error("planner options: {0}")]
    Options(String),
    #[error("planner network: {0}")]
    Net(String),
}

/// The step network: observation (8) → neutral-relative step (3), deterministic.
pub trait StepNet {
    fn step(&mut self, observation: &[f32; 8]) -> Result<[f32; 3], PlannerError>;
}

/// A disc the plan must keep the feet out of, in the world frame. FootstepNet saw one obstacle per episode,
/// radius up to `obstacle_max_radius`; "no obstacle" is radius 0.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Obstacle {
    pub x: f64,
    pub y: f64,
    pub radius: f64,
}

/// A rollout to the goal.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Plan {
    /// Every planned landing, in order: the pose and the foot that lands there. The first is the swing foot's
    /// next target.
    pub steps: Vec<(Pose2, Foot)>,
    /// Whether the last pose satisfies the goal (or the start already did, with no steps).
    pub reached: bool,
}

pub struct Planner<N> {
    net: N,
    options: PlannerOptions,
}

impl<N: StepNet> Planner<N> {
    pub fn new(net: N, options: PlannerOptions) -> Self {
        Self { net, options }
    }

    pub fn options(&self) -> &PlannerOptions {
        &self.options
    }

    pub fn net(&self) -> &N {
        &self.net
    }

    /// Where the swing foot lands for a zero step: parallel to the support, `feet_spacing` to its side.
    pub fn neutral(&self, support: Pose2, support_foot: Foot) -> Pose2 {
        compose(
            support,
            [
                0.0,
                support_foot.other().side() * self.options.feet_spacing,
                0.0,
            ],
        )
    }

    /// `FootstepsPlanningEnv.get_observation()`: goal and obstacle in the support sole frame, a right support's
    /// view (a left support mirrors y and θ).
    pub fn observation(
        &self,
        support: Pose2,
        support_foot: Foot,
        goal: Pose2,
        goal_foot: Foot,
        obstacle: Option<Obstacle>,
    ) -> [f32; 8] {
        let g = relative(support, goal);
        let (sin, cos) = g[2].sin_cos();
        let o = obstacle.unwrap_or(Obstacle {
            x: goal[0],
            y: goal[1],
            radius: 0.0,
        });
        let ob = relative(support, [o.x, o.y, 0.0]);
        let m = if support_foot == Foot::Left {
            -1.0
        } else {
            1.0
        };
        [
            g[0],
            m * g[1],
            cos,
            m * sin,
            f64::from(u8::from(support_foot == goal_foot)),
            ob[0],
            m * ob[1],
            o.radius,
        ]
        .map(|v| v as f32)
    }

    /// The network's step, mirrored back to the support foot's side and clipped to the limits: the box, then
    /// FootstepNet's ellipsoid.
    pub fn step(
        &mut self,
        observation: &[f32; 8],
        support_foot: Foot,
    ) -> Result<[f64; 3], PlannerError> {
        let raw = self.net.step(observation)?;
        if !raw.iter().all(|v| v.is_finite()) {
            return Err(PlannerError::Net(format!("non-finite step {raw:?}")));
        }
        let mut a = raw.map(f64::from);
        if support_foot == Foot::Left {
            a[1] = -a[1];
            a[2] = -a[2];
        }
        Ok(self.options.clip(a))
    }

    /// Clip a neutral-relative step to the limits the planner was trained with: the box, then the ellipsoid.
    pub fn clip(&self, step: [f64; 3]) -> [f64; 3] {
        self.options.clip(step)
    }

    /// The goal foot within `tolerance_distance` and `tolerance_angle` of the goal pose.
    pub fn reached(
        &self,
        support: Pose2,
        support_foot: Foot,
        goal: Pose2,
        goal_foot: Foot,
    ) -> bool {
        let r = relative(support, goal);
        support_foot == goal_foot
            && r[0].hypot(r[1]) < self.options.tolerance_distance
            && r[2].abs() < self.options.tolerance_angle
    }

    /// Roll the planner out from the support foot until the goal is reached or `max_steps` are planned.
    pub fn plan(
        &mut self,
        support: Pose2,
        support_foot: Foot,
        goal: Pose2,
        goal_foot: Foot,
        obstacle: Option<Obstacle>,
        max_steps: usize,
    ) -> Result<Plan, PlannerError> {
        let mut steps = Vec::new();
        let (mut pose, mut foot) = (support, support_foot);
        for _ in 0..max_steps {
            if self.reached(pose, foot, goal, goal_foot) {
                return Ok(Plan {
                    steps,
                    reached: true,
                });
            }
            let observation = self.observation(pose, foot, goal, goal_foot, obstacle);
            let step = self.step(&observation, foot)?;
            pose = compose(self.neutral(pose, foot), step);
            foot = foot.other();
            steps.push((pose, foot));
        }
        let reached = self.reached(pose, foot, goal, goal_foot);
        Ok(Plan { steps, reached })
    }
}
