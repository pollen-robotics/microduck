//! Turning sensors and a command into joint targets.
//!
//! Everything here is pure computation between [`duck_control::io::RobotIo::read`] and the
//! safety layer's `apply`. It holds no IO handle — by construction it cannot command a
//! motor, only propose targets.
//!
//! The tick, in order:
//!
//! ```text
//! observation ← sensors + last action + command
//! standing?   ← velocity magnitude vs threshold
//! action      ← ONNX
//! targets     ← home pose + action_scale × action
//! filters     ← optional first-order low-pass on head and legs
//! ```
//!
//! The numeric defaults come from `microduck_runtime`, including which of them are off:
//! both low-pass filters are opt-in flags there, not enabled behaviour, so they are off
//! here too. Turning them on by default would be a silent change to how the robot moves.

use duck_control::model::{DEFAULT_POSITION, NUM_JOINTS};
use duck_control::obs::{ACTION_LEN, Command, Observation};
use duck_control::policy::{Policy, PolicyError};

/// Joint indices the head low-pass covers: neck_pitch, head_pitch, head_yaw, head_roll.
const HEAD_JOINTS: std::ops::Range<usize> = 5..9;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tuning {
    /// Scales raw policy output before it becomes a joint offset. The prototype's 0.7.
    pub action_scale: f64,
    /// The standing policy is trained to be applied whole.
    pub standing_action_scale: f64,
    /// Standing runs softer, at this fraction of the running gain.
    pub standing_gain_ratio: f64,
    pub gain: u16,
    /// First-order low-pass on the head joints. `None` is no filtering, which is the
    /// prototype's default — there it is behind `--head-low-pass`.
    pub head_lowpass: Option<f64>,
    /// Same, for the ten leg joints.
    pub legs_lowpass: Option<f64>,
}

impl Default for Tuning {
    fn default() -> Self {
        Self {
            action_scale: 0.7,
            standing_action_scale: 1.0,
            standing_gain_ratio: 0.6,
            gain: 200,
            head_lowpass: None,
            legs_lowpass: None,
        }
    }
}

/// One tick's worth of decisions, for the caller to act on and report.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Step {
    pub targets: [f64; NUM_JOINTS],
    /// Whether the standing policy drove this tick.
    pub standing: bool,
    /// What the gain should be while upright. Safety still overrides it on a fall.
    pub gain: u16,
}

pub struct Controller {
    policy: Policy,
    tuning: Tuning,
    /// Raw previous policy output, which the observation feeds back. Raw, not scaled: the
    /// policy was trained observing its own output, not the actuator command derived from
    /// it.
    last_action: [f32; ACTION_LEN],
    /// Previous filtered targets, kept only for the low-pass. `None` until the first tick,
    /// so the filter starts from reality rather than dragging up from zero.
    previous: Option<[f64; NUM_JOINTS]>,
}

impl Controller {
    pub fn new(policy: Policy, tuning: Tuning) -> Self {
        Self {
            policy,
            tuning,
            last_action: [0.0; ACTION_LEN],
            previous: None,
        }
    }

    /// Reset the feedback state.
    ///
    /// Called when the policy is re-enabled, so a robot that sat disabled for a minute does
    /// not resume with a stale action in its observation and a filter anchored to wherever
    /// it was before.
    pub fn reset(&mut self) {
        self.last_action = [0.0; ACTION_LEN];
        self.previous = None;
    }

    pub fn step(
        &mut self,
        sensors: &duck_control::Sensors,
        command: &Command,
    ) -> Result<Step, PolicyError> {
        let observation = Observation::build(
            &sensors.imu,
            &sensors.positions,
            &sensors.velocities,
            &DEFAULT_POSITION,
            &self.last_action,
            command,
        );

        let standing = self.policy.will_stand(command.twist_magnitude());
        let action = self.policy.infer(&observation, standing)?;
        self.last_action = action;

        let scale = if standing {
            self.tuning.standing_action_scale
        } else {
            self.tuning.action_scale
        };
        let offsets = Observation::scatter_action(&action);

        let mut targets = [0.0; NUM_JOINTS];
        for joint in 0..NUM_JOINTS {
            targets[joint] = DEFAULT_POSITION[joint] + scale * offsets[joint];
        }

        if let Some(previous) = self.previous {
            if let Some(alpha) = self.tuning.head_lowpass {
                for joint in HEAD_JOINTS {
                    targets[joint] = alpha * targets[joint] + (1.0 - alpha) * previous[joint];
                }
            }
            if let Some(alpha) = self.tuning.legs_lowpass {
                for (joint, target) in targets.iter_mut().enumerate() {
                    if HEAD_JOINTS.contains(&joint) || joint == duck_control::model::MOUTH_INDEX {
                        continue;
                    }
                    *target = alpha * *target + (1.0 - alpha) * previous[joint];
                }
            }
        }
        self.previous = Some(targets);

        let gain = if standing {
            (self.tuning.gain as f64 * self.tuning.standing_gain_ratio).round() as u16
        } else {
            self.tuning.gain
        };

        Ok(Step {
            targets,
            standing,
            gain,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The prototype's numbers, and — just as important — which of them are off. Both
    /// low-pass filters are `--flag`s there, so defaulting them on here would silently
    /// change how the robot moves relative to the thing it is replacing.
    #[test]
    fn the_defaults_match_the_prototype() {
        let t = Tuning::default();
        assert_eq!(t.action_scale, 0.7);
        assert_eq!(t.standing_action_scale, 1.0);
        assert_eq!(t.standing_gain_ratio, 0.6);
        assert_eq!(t.head_lowpass, None, "the prototype ships this off");
        assert_eq!(t.legs_lowpass, None, "the prototype ships this off");
    }

    /// Standing must drop the gain. Running the standing policy at walking stiffness is a
    /// visibly different robot, and the ratio is the prototype's.
    #[test]
    fn standing_softens_the_gain() {
        let t = Tuning::default();
        let standing_gain = (t.gain as f64 * t.standing_gain_ratio).round() as u16;
        assert_eq!(standing_gain, 120);
        assert!(standing_gain < t.gain);
    }
}
