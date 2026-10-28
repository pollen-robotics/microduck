//! From a plan to the footstep policy's command, replanned at every touchdown.

use crate::planner::{Obstacle, Plan, Planner, PlannerError, PlannerOptions, StepNet};
use crate::pose::{Foot, Pose2, compose, relative};

/// The joystick's goal: where a constant twist would carry the stance in `horizon_s` seconds.
#[derive(Debug, Clone, Copy)]
pub struct CarrotParams {
    /// Look-ahead, seconds. Longer reaches the step limits sooner (the plan's first step saturates); shorter
    /// makes the robot hesitate between replans. 1 s ≈ four steps at the trained cadence.
    pub horizon_s: f64,
    /// Planned steps per replan. The policy only takes the first; the rest are for the log and the viewer, and
    /// each costs one tiny inference inside the control tick.
    pub max_steps: usize,
}

impl Default for CarrotParams {
    fn default() -> Self {
        Self {
            horizon_s: 1.0,
            max_steps: 8,
        }
    }
}

/// The goal for the swing foot when the operator holds `twist = [vx, vy, ωz]` (m/s, m/s, rad/s, robot frame).
///
/// The stance centre — halfway between the support sole and its neutral partner — travels the arc a constant
/// twist draws over `horizon_s`; the swing foot's goal is its place beside that centre. Zero twist puts the goal
/// on the neutral stance: one step to square up, then stepping in place.
pub fn carrot_goal(
    support: Pose2,
    support_foot: Foot,
    twist: [f64; 3],
    horizon_s: f64,
    feet_spacing: f64,
) -> (Pose2, Foot) {
    let swing = support_foot.other();
    let half = swing.side() * feet_spacing / 2.0;
    let centre = compose(support, [0.0, half, 0.0]);
    let [vx, vy, wz] = twist;
    let theta = wz * horizon_s;
    // Exact displacement of a constant body twist: ∫ R(ωt) v dt.
    let (sx, cx) = if theta.abs() < 1e-6 {
        (horizon_s, 0.0)
    } else {
        (theta.sin() / wz, (1.0 - theta.cos()) / wz)
    };
    let moved = [vx * sx - vy * cx, vx * cx + vy * sx, theta];
    (compose(compose(centre, moved), [0.0, half, 0.0]), swing)
}

/// The swing foot's step for a held stick: each axis of `twist` as a fraction of its full deflection `full`
/// (m/s, m/s, rad/s) times that axis's step limit, clipped like FootstepNet clips — a neutral-relative step in the
/// robot's own frame (x forward, y left, θ counter-clockwise), whichever foot swings.
///
/// Proportional on purpose. Planning toward a goal a stick pushes ahead (the first version) made every
/// deflection a full step: FootstepNet reaches any goal in the fewest steps, and below a step it only aims
/// within its 2 cm / 5° reached tolerance — 1 cm asked gave 0.5 cm, 2 cm gave 1.7 cm (seen on the robot: "a
/// small angle or a full one makes the same step size"). The policy was trained on exactly this target.
pub fn stick_step(twist: [f64; 3], full: [f64; 3], options: &PlannerOptions) -> [f64; 3] {
    let frac = |v: f64, f: f64| {
        if f > 0.0 {
            (v / f).clamp(-1.0, 1.0)
        } else {
            0.0
        }
    };
    let fx = frac(twist[0], full[0]);
    let dx = fx
        * if fx >= 0.0 {
            options.max_dx_forward
        } else {
            options.max_dx_backward
        };
    options.clip([
        dx,
        frac(twist[1], full[1]) * options.max_dy,
        frac(twist[2], full[2]) * options.max_dtheta,
    ])
}

/// Follows a plan one touchdown at a time.
#[derive(Debug, Clone, Default)]
pub struct Follower {
    support: Option<Foot>,
    target: Pose2,
    plan: Plan,
    switched: bool,
    last_error: Option<(f64, f64)>,
}

impl Follower {
    pub fn new() -> Self {
        Self::default()
    }

    /// Forget the support foot: the next [`Self::update`] replans as if starting.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// One control tick. `support_foot` and `soles` (left, right) come from the contact odometry; `goal` is asked
    /// for the goal pose and foot, given the support sole, only when a replan is due — at a touchdown, or on the
    /// first tick. Returns the policy command: the swing foot's target in the support sole frame.
    ///
    /// The target is fixed for the whole swing, as in training (the command term resamples on support switches
    /// only); only its expression in the support frame moves as the support foot does.
    pub fn update<N: StepNet>(
        &mut self,
        planner: &mut Planner<N>,
        support_foot: Foot,
        soles: [Pose2; 2],
        goal: impl FnOnce(Pose2, Foot) -> (Pose2, Foot),
        obstacle: Option<Obstacle>,
        max_steps: usize,
    ) -> Result<[f64; 3], PlannerError> {
        self.advance(support_foot, soles, |support| {
            let (goal_pose, goal_foot) = goal(support, support_foot);
            let plan = planner.plan(
                support,
                support_foot,
                goal_pose,
                goal_foot,
                obstacle,
                max_steps,
            )?;
            let target = match plan.steps.first() {
                Some((pose, _)) => *pose,
                None => planner.neutral(support, support_foot), // at the goal: step in place
            };
            Ok((target, plan))
        })
    }

    /// [`Self::update`] with the target chosen by the caller instead of planned: `target` is asked, at a
    /// touchdown, for the swing foot's next landing given the support sole and foot. A scripted pattern.
    pub fn update_scripted(
        &mut self,
        support_foot: Foot,
        soles: [Pose2; 2],
        target: impl FnOnce(Pose2, Foot) -> Pose2,
    ) -> [f64; 3] {
        let result = self.advance(support_foot, soles, |support| {
            Ok::<_, PlannerError>((target(support, support_foot), Plan::default()))
        });
        result.expect("a scripted target cannot fail")
    }

    fn advance(
        &mut self,
        support_foot: Foot,
        soles: [Pose2; 2],
        replan: impl FnOnce(Pose2) -> Result<(Pose2, Plan), PlannerError>,
    ) -> Result<[f64; 3], PlannerError> {
        let support = soles[support_foot.index()];
        self.switched = self.support != Some(support_foot);
        if self.switched {
            if self.support.is_some() {
                let r = relative(self.target, support);
                self.last_error = Some((r[0].hypot(r[1]), r[2].abs()));
            }
            self.support = Some(support_foot);
            (self.target, self.plan) = replan(support)?;
        }
        Ok(relative(support, self.target))
    }

    /// Whether the last update was a touchdown (or the first tick).
    pub fn switched(&self) -> bool {
        self.switched
    }

    /// The swing foot's target, world frame.
    pub fn target(&self) -> Pose2 {
        self.target
    }

    pub fn plan(&self) -> &Plan {
        &self.plan
    }

    /// The last touchdown's distance (m) and heading error (rad) from the target it was given.
    pub fn last_error(&self) -> Option<(f64, f64)> {
        self.last_error
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SPACING: f64 = 0.095;

    /// Released stick: the goal is the swing foot's neutral place, so the planner squares the feet up and then
    /// steps in place. A goal anywhere else here would walk a robot whose operator let go.
    #[test]
    fn zero_twist_goal_is_the_neutral_stance() {
        for (support, foot) in [
            ([0.1, -0.2, 0.7], Foot::Left),
            ([0.0, 0.0, -2.0], Foot::Right),
        ] {
            let (goal, goal_foot) = carrot_goal(support, foot, [0.0; 3], 1.0, SPACING);
            assert_eq!(goal_foot, foot.other());
            let r = relative(support, goal);
            assert!(
                r[0].abs() < 1e-12
                    && (r[1] - foot.other().side() * SPACING).abs() < 1e-12
                    && r[2].abs() < 1e-12,
                "{r:?}"
            );
        }
    }

    fn options() -> PlannerOptions {
        PlannerOptions {
            max_dx_forward: 0.05,
            max_dx_backward: 0.05,
            max_dy: 0.02,
            max_dtheta: 0.52,
            tolerance_distance: 0.02,
            tolerance_angle: 0.087,
            obstacle_max_radius: 0.12,
            foot_length: 0.054,
            foot_width: 0.041,
            feet_spacing: SPACING,
        }
    }

    /// The step is proportional to the stick on every axis — a quarter deflection is a quarter step — and full
    /// deflection on one axis is that axis's limit. The "every deflection is a full step" regression guard.
    #[test]
    fn stick_step_is_proportional() {
        let full = [0.4, 0.3, 1.0];
        let o = options();
        let s = stick_step([0.1, 0.0, 0.0], full, &o);
        assert!(
            (s[0] - 0.0125).abs() < 1e-12 && s[1] == 0.0 && s[2] == 0.0,
            "{s:?}"
        );
        let s = stick_step([0.4, 0.0, 0.0], full, &o);
        assert!((s[0] - 0.05).abs() < 1e-12, "{s:?}");
        let s = stick_step([-0.2, 0.0, 0.0], full, &o);
        assert!((s[0] + 0.025).abs() < 1e-12, "{s:?}");
        let s = stick_step([0.0, -0.15, 0.5], full, &o);
        assert!(
            (s[1] + 0.01).abs() < 1e-12 && (s[2] - 0.26).abs() < 1e-12,
            "{s:?}"
        );
        // Past full deflection it saturates; two full axes share the ellipsoid.
        let s = stick_step([0.8, 0.3, 0.0], full, &o);
        let norm = ((s[0] / 0.05).powi(2) + (s[1] / 0.02).powi(2)).sqrt();
        assert!((norm - 1.0).abs() < 1e-9, "{s:?}");
    }

    /// Forward stick moves the goal straight ahead by v·T; a pure turn rotates the stance about its centre. The
    /// arc formula's sign is what makes "left stick" mean left.
    #[test]
    fn carrot_follows_the_stick() {
        let support = [0.0, 0.0, 0.0];
        let (g, _) = carrot_goal(support, Foot::Right, [0.2, 0.0, 0.0], 1.0, SPACING);
        assert!(
            (g[0] - 0.2).abs() < 1e-12 && (g[1] - SPACING).abs() < 1e-12 && g[2].abs() < 1e-12,
            "{g:?}"
        );
        let (g, _) = carrot_goal(support, Foot::Right, [0.0, 0.1, 0.0], 1.0, SPACING);
        assert!(
            g[0].abs() < 1e-12 && (g[1] - SPACING - 0.1).abs() < 1e-12,
            "{g:?}"
        );
        // Turning in place: the centre stays, the left foot swings round it.
        let (g, _) = carrot_goal(
            support,
            Foot::Right,
            [0.0, 0.0, std::f64::consts::FRAC_PI_2],
            1.0,
            SPACING,
        );
        let centre = [0.0, SPACING / 2.0];
        assert!(
            (g[0] - (centre[0] - SPACING / 2.0)).abs() < 1e-9 && (g[1] - centre[1]).abs() < 1e-9,
            "{g:?}"
        );
        assert!((g[2] - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
        // Forward while turning left ends up left of straight ahead.
        let (g, _) = carrot_goal(support, Foot::Right, [0.2, 0.0, 0.5], 1.0, SPACING);
        assert!(g[1] > SPACING && g[0] < 0.2, "{g:?}");
    }
}
