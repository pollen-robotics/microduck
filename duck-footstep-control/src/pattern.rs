//! A scripted footstep pattern: a fixed list of landings, walked one touchdown at a time.
//!
//! For checking placement against marks on the floor (microduck_rl `scripts/footsteps_patterns.py` prints them
//! 1:1 on A4). The steps are given in the **start stance frame** — origin midway between the two sole centres,
//! x along their mean heading — and anchored to wherever the robot stands when the pattern starts, so standing
//! the robot on the printed start soles lines the pattern up with the paper.

use crate::follower::Follower;
use crate::pose::{Foot, Pose2, compose, wrap};

/// Touchdowns in place after the last step before the pattern hands back: the first squares the feet up, the
/// second lands the last step's miss in the log.
const TAIL_TOUCHDOWNS: u32 = 2;

/// One landing that missed its mark: pattern index, distance (m), heading error (rad).
pub type Miss = (usize, f64, f64);

#[derive(Debug, Clone)]
pub struct PatternRun {
    steps: Vec<(Pose2, Foot)>,
    world: Option<Vec<Pose2>>,
    next: usize,
    in_flight: Option<usize>,
    tail: u32,
    misses: Vec<Miss>,
}

/// Midway between the two sole centres, heading their mean heading.
pub fn stance_frame(soles: [Pose2; 2]) -> Pose2 {
    let [l, r] = soles;
    let yaw = (l[2].sin() + r[2].sin()).atan2(l[2].cos() + r[2].cos());
    [(l[0] + r[0]) / 2.0, (l[1] + r[1]) / 2.0, wrap(yaw)]
}

impl PatternRun {
    pub fn new(steps: Vec<(Pose2, Foot)>) -> Self {
        Self {
            steps,
            world: None,
            next: 0,
            in_flight: None,
            tail: 0,
            misses: Vec::new(),
        }
    }

    pub fn len(&self) -> usize {
        self.steps.len()
    }

    pub fn is_empty(&self) -> bool {
        self.steps.is_empty()
    }

    /// The misses so far, one per landed pattern step.
    pub fn misses(&self) -> &[Miss] {
        &self.misses
    }

    /// One control tick: the policy command, or `None` once the pattern is walked (at a touchdown).
    ///
    /// Anchors on its first tick. At each touchdown the swing foot gets the next pattern step if that step is
    /// its foot's, else it steps in place (`feet_spacing` beside the support) — which is how a robot whose first
    /// free foot is not the pattern's first foot falls into phase.
    pub fn tick(
        &mut self,
        follower: &mut Follower,
        support_foot: Foot,
        soles: [Pose2; 2],
        feet_spacing: f64,
    ) -> Option<[f64; 3]> {
        if self.world.is_none() {
            let anchor = stance_frame(soles);
            self.world = Some(
                self.steps
                    .iter()
                    .map(|(p, _)| compose(anchor, *p))
                    .collect(),
            );
            follower.reset();
        }
        let world = self.world.as_ref().expect("anchored above");
        let previous = self.in_flight;
        let (next, steps) = (self.next, &self.steps);
        let mut chosen = None;
        let command = follower.update_scripted(support_foot, soles, |support, foot| {
            let swing = foot.other();
            if next < steps.len() && steps[next].1 == swing {
                chosen = Some(next);
                world[next]
            } else {
                compose(support, [0.0, swing.side() * feet_spacing, 0.0])
            }
        });
        if follower.switched() {
            if let (Some(k), Some((distance, heading))) = (previous, follower.last_error()) {
                self.misses.push((k, distance, heading));
            }
            self.in_flight = chosen;
            match chosen {
                Some(k) => self.next = k + 1,
                None if self.next >= self.steps.len() => {
                    self.tail += 1;
                    if self.tail > TAIL_TOUCHDOWNS {
                        return None;
                    }
                }
                None => {}
            }
        }
        Some(command)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pose::relative;

    const SPACING: f64 = 0.095;

    /// A robot that lands every target exactly: the pattern must come out landing by landing, in the world the
    /// start stance defines, with zero misses, and hand back after squaring up. Starting on the "wrong" support
    /// foot costs one step in place, not a skipped mark.
    #[test]
    fn a_perfect_walker_lands_the_pattern_in_order() {
        let pattern: Vec<(Pose2, Foot)> = vec![
            ([0.05, SPACING / 2.0, 0.0], Foot::Left),
            ([0.10, -SPACING / 2.0, 0.0], Foot::Right),
            ([0.15, SPACING / 2.0, 0.3], Foot::Left),
        ];
        for first_support in [Foot::Right, Foot::Left] {
            let start: Pose2 = [1.0, 2.0, 0.5];
            let mut soles = [
                compose(start, [0.0, SPACING / 2.0, 0.0]),
                compose(start, [0.0, -SPACING / 2.0, 0.0]),
            ];
            let mut run = PatternRun::new(pattern.clone());
            let mut follower = Follower::new();
            let mut support = first_support;
            let mut landed = Vec::new();
            for _ in 0..20 {
                let Some(command) = run.tick(&mut follower, support, soles, SPACING) else {
                    break;
                };
                // The swing foot lands exactly on the command's target; it becomes the support.
                let swing = support.other();
                soles[swing.index()] = compose(soles[support.index()], command);
                landed.push(relative(start, soles[swing.index()]));
                support = swing;
            }
            // The marks, in order, as a subsequence of the landings (stepping in place may land on one again).
            let mut k = 0;
            for p in &landed {
                if k < pattern.len() && (0..3).all(|i| (p[i] - pattern[k].0[i]).abs() < 1e-9) {
                    k += 1;
                }
            }
            assert_eq!(k, 3, "support {first_support:?}: landed {landed:?}");
            assert_eq!(run.misses().len(), 3);
            assert!(
                run.misses().iter().all(|&(_, d, a)| d < 1e-9 && a < 1e-9),
                "{:?}",
                run.misses()
            );
            let in_place = landed.len() - 3;
            let expected =
                if first_support == Foot::Left { 1 } else { 0 } + TAIL_TOUCHDOWNS as usize;
            assert_eq!(in_place, expected, "support {first_support:?}: {landed:?}");
        }
    }

    #[test]
    fn stance_frame_is_the_midpoint_and_mean_heading() {
        let f = stance_frame([[0.0, 0.05, 3.1], [0.0, -0.05, -3.1]]);
        assert!(f[0].abs() < 1e-12 && f[1].abs() < 1e-12);
        assert!((f[2].abs() - std::f64::consts::PI).abs() < 1e-9, "{f:?}");
    }
}
