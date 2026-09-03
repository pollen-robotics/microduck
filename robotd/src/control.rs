//! Turning sensors and a command into joint targets — and scheduling the skills.
//!
//! Everything here is pure computation between [`duck_control::io::RobotIo::read`] and the
//! safety layer's `apply`. It holds no IO handle — by construction it cannot command a
//! motor, only propose targets.
//!
//! The tick, in order:
//!
//! ```text
//! skill windows ← advance / expire (roulade window, kick timer, ground-pick phase, sit↔stand rise)
//! command      ← the caller's smoothed command, re-encoded for the active skill
//! net          ← roulade > kick > ground pick > sit/rise > stand-by-magnitude > walk
//! action       ← ONNX
//! targets      ← home pose + action_scale × action
//! filters      ← optional first-order low-pass on head and legs
//! ```
//!
//! The priority chain and every numeric default come from `microduck_runtime`'s
//! `control_step`, which this replaces. Two of its subtleties are worth naming because they
//! are easy to "fix" by accident:
//!
//!  - **A kick window runs at standing tuning.** The kick's observation carries an all-zero
//!    command, and in the prototype the standing transition fires on exactly that — so a
//!    kick runs at `standing_action_scale` and the softened standing gain. Kept, because
//!    the kicks were tuned against it.
//!  - **The sitstand *rise* also runs at the standing gain** (its command is all-zero),
//!    while the *sit* does not (its posture flag makes the twist magnitude 1). Same
//!    mechanism, same reason.
//!
//! One deliberate divergence: the prototype tracks the standing action scale by
//! saving/restoring `action_scale` on transitions, which can leave a stale value behind
//! after a sit→stand cycle until the next walk. Here scale and gain are recomputed from
//! the active state every tick — same values on every path that matters, no leftovers.

use duck_control::model::{DEFAULT_POSITION, NUM_JOINTS};
use duck_control::obs::{ACTION_LEN, Command, Observation};
use duck_control::policy::{Net, Policy, PolicyError};

/// Joint indices the head low-pass covers: neck_pitch, head_pitch, head_yaw, head_roll.
const HEAD_JOINTS: std::ops::Range<usize> = 5..9;

/// The ground pick hands back at this fraction of its cycle — the prototype's cutoff.
const GROUND_PICK_END_PHASE: f64 = 0.7;

/// How long the sitstand network rises (posture flag 0) before the main policy takes over.
/// 1 s is enough on the robot — velstand owns the tail of the rise fine.
const RISE_SECS: f64 = 1.0;

/// How recently a roulade request must have arrived, at the end of a roll, for another to
/// chain. The prototype chains on "X still held at the window boundary"; here the client
/// holds the button by re-sending the request every tick, so "held" is "a request landed
/// within the last few ticks". 150 ms is seven ticks — generous against a dropped packet,
/// far too short to mistake a fresh press for a hold.
const ROULADE_CHAIN_WINDOW: f64 = 0.15;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tuning {
    /// Scales raw policy output before it becomes a joint offset. The prototype's current
    /// alpha default.
    pub action_scale: f64,
    /// The standing policy is trained to be applied whole.
    pub standing_action_scale: f64,
    /// Standing runs softer, at this fraction of the running gain. `--standing-kp-ratio`.
    pub standing_gain_ratio: f64,
    pub gain: u16,
    /// First-order low-pass on the head joints. `None` is no filtering. The alpha policies
    /// are trained with 0.5 — it must match training or transfer degrades.
    pub head_lowpass: Option<f64>,
    /// Same, for the ten leg joints. Trained with 0.7.
    pub legs_lowpass: Option<f64>,
}

impl Default for Tuning {
    fn default() -> Self {
        Self {
            action_scale: 0.9,
            standing_action_scale: 1.0,
            standing_gain_ratio: 0.8,
            gain: 200,
            head_lowpass: Some(0.5),
            legs_lowpass: Some(0.7),
        }
    }
}

/// The scripted-skill numbers, resolved per mode by `params`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkillTuning {
    /// One ground-pick cycle, seconds.
    pub ground_pick_period: f64,
    pub ground_pick_action_scale: f64,
    /// Gain multiplier while the pick runs.
    pub ground_pick_gain_ratio: f64,
    /// How long a kick window stays on the kick network, seconds.
    pub kick_duration: f64,
    /// One roulade — one forward roll, seconds. The prototype's measured single-roll time.
    pub roulade_duration: f64,
    /// Action scale while a roulade runs.
    pub roulade_action_scale: f64,
    /// Gain multiplier while a roulade runs.
    pub roulade_gain_ratio: f64,
}

impl Default for SkillTuning {
    fn default() -> Self {
        Self {
            ground_pick_period: 4.0,
            ground_pick_action_scale: 1.0,
            ground_pick_gain_ratio: 1.0,
            kick_duration: 0.5,
            roulade_duration: 1.0,
            roulade_action_scale: 1.0,
            roulade_gain_ratio: 1.0,
        }
    }
}

/// One tick's worth of decisions, for the caller to act on and report.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Step {
    pub targets: [f64; NUM_JOINTS],
    /// Which network drove, as the wire label: `walk`, `stand`, `ground_pick`, `kick_left`,
    /// `kick_right`, `sit`, `rise` — or `external` when an off-robot controller is driving the
    /// joints directly ([`Step::external`]).
    pub label: &'static str,
    /// What the gain should be for this tick.
    pub gain: u16,
    /// A scripted move is mid-flight — the robot is moving regardless of the twist, so
    /// restarting the daemon now would put it on the floor.
    pub busy: bool,
}

impl Step {
    /// The **External** drive path: joint targets streamed by an off-robot controller (via
    /// `robot.setJoints`) in place of the on-device policy's output.
    ///
    /// External mode skips the whole policy stage — `observe → infer → scatter → scale →
    /// low-pass` — so there is nothing to run here: the tick *is* the supplied targets. They are
    /// carried straight to the safety layer's external chokepoint,
    /// [`duck_control::safety::Safety::apply_external`], which owns the per-joint anatomical
    /// limits, the per-tick step clamp and the dedicated external deadman. Nothing is clamped
    /// here — this is a decision object, and like every other producer in this module it holds
    /// no IO handle: it proposes targets, it does not command a motor.
    ///
    /// `busy` is false: External is a drive mode a client enters and leaves deliberately, not a
    /// scripted move mid-flight, so it does not block a restart the way a roulade does — the mode
    /// gate, not this flag, is what refuses `robot.setJoints` outside External.
    pub fn external(targets: [f64; NUM_JOINTS], gain: u16) -> Self {
        Self {
            targets,
            label: "external",
            gain,
            busy: false,
        }
    }
}

/// Where the robot is in the sit↔stand cycle.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Sit {
    Up,
    /// The sitstand network holds the seat (posture flag 1).
    Sitting,
    /// The sitstand network rises (posture flag 0) for the remaining seconds, then the
    /// main policy takes over.
    Rising {
        remaining: f64,
    },
}

pub struct Controller {
    policy: Policy,
    tuning: Tuning,
    skills: SkillTuning,
    /// Raw previous policy output, which the observation feeds back. Raw, not scaled: the
    /// policy was trained observing its own output, not the actuator command derived from
    /// it. Shared across every network, as the prototype shares it.
    last_action: [f32; ACTION_LEN],
    /// Previous filtered targets, kept only for the low-pass. `None` until the first tick,
    /// so the filter starts from reality rather than dragging up from zero.
    previous: Option<[f64; NUM_JOINTS]>,
    /// Ground-pick phase, 0..[`GROUND_PICK_END_PHASE`]. `None` when inactive.
    ground_pick: Option<f64>,
    /// An active kick window: which leg, and seconds remaining.
    kick: Option<(bool, f64)>,
    /// An active roulade: seconds remaining in the current roll.
    roulade: Option<f64>,
    /// Seconds since a roulade request would still count as "the button is held" — counts
    /// down every tick, refreshed by each request that arrives while a roll runs. At the
    /// end of a roll, positive means chain another.
    roulade_chain: f64,
    sit: Sit,
}

impl Controller {
    pub fn new(policy: Policy, tuning: Tuning, skills: SkillTuning) -> Self {
        Self {
            policy,
            tuning,
            skills,
            last_action: [0.0; ACTION_LEN],
            previous: None,
            ground_pick: None,
            kick: None,
            roulade: None,
            roulade_chain: 0.0,
            sit: Sit::Up,
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

    pub fn has_sitstand(&self) -> bool {
        self.policy.has_sitstand()
    }

    pub fn is_sitting(&self) -> bool {
        self.sit == Sit::Sitting
    }

    /// A scripted move is mid-flight. Sitting itself is not busy — a seated robot is
    /// parked, not travelling.
    pub fn busy(&self) -> bool {
        self.ground_pick.is_some()
            || self.kick.is_some()
            || self.roulade.is_some()
            || matches!(self.sit, Sit::Rising { .. })
    }

    /// Start a one-shot ground pick. The prototype gates the trigger on nothing but the
    /// network existing and the move not already running — a pick can even preempt a kick's
    /// tail, and that stays as it was.
    pub fn start_ground_pick(&mut self) -> Result<(), &'static str> {
        if !self.policy.has_ground_pick() {
            return Err("no ground-pick policy loaded");
        }
        if self.ground_pick.is_some() {
            return Err("ground pick already running");
        }
        self.ground_pick = Some(0.0);
        Ok(())
    }

    /// Start a kick window. Blocked while any scripted move runs, as the prototype blocks it.
    pub fn start_kick(&mut self, left: bool) -> Result<(), &'static str> {
        if !self.policy.has_kick(left) {
            return Err("no kick policy loaded for that leg");
        }
        if self.kick.is_some() || self.ground_pick.is_some() || self.roulade.is_some() {
            return Err("a scripted move is already running");
        }
        self.kick = Some((left, self.skills.kick_duration));
        Ok(())
    }

    /// A roulade request: start a roll, or — arriving while one runs — keep the chain alive.
    ///
    /// `Ok(true)` started a roll; `Ok(false)` refreshed a running one (the caller should
    /// stay quiet: a held button lands here fifty times a second). The prototype gates the
    /// X press on nothing but the ground pick — a roulade can preempt a kick's tail or roll
    /// out of the seat, and both stay as they were.
    pub fn request_roulade(&mut self) -> Result<bool, &'static str> {
        if self.roulade.is_some() {
            self.roulade_chain = ROULADE_CHAIN_WINDOW;
            return Ok(false);
        }
        if !self.policy.has_roulade() {
            return Err("no roulade policy loaded");
        }
        if self.ground_pick.is_some() {
            return Err("a ground pick is running");
        }
        self.roulade = Some(self.skills.roulade_duration);
        self.roulade_chain = 0.0;
        Ok(true)
    }

    /// Sit if standing, stand if sitting. Refused mid-rise, as the prototype refuses it
    /// while a stand transition is in flight.
    pub fn sit_toggle(&mut self) -> Result<&'static str, &'static str> {
        match self.sit {
            Sit::Up => {
                if !self.policy.has_sitstand() {
                    return Err("no sitstand policy loaded");
                }
                self.sit = Sit::Sitting;
                Ok("sit")
            }
            Sit::Sitting => {
                self.sit = Sit::Rising {
                    remaining: RISE_SECS,
                };
                Ok("stand")
            }
            Sit::Rising { .. } => Err("already standing up"),
        }
    }

    /// Engage the sit for the shutdown sequence. The caller owns the timing (sit for a few
    /// seconds, then cut torque and power off); this just puts the sitstand network in
    /// charge with the posture flag at 1.
    pub fn begin_shutdown_sit(&mut self) {
        self.sit = Sit::Sitting;
    }

    /// Seated boot: the robot powered on already sitting, so rise via the sitstand network
    /// instead of dragging the legs through a linear ramp to the standing pose.
    pub fn begin_boot_rise(&mut self) {
        self.sit = Sit::Rising {
            remaining: RISE_SECS,
        };
    }

    /// One tick.
    ///
    /// `body_active` says a client is holding the body-pose mode: the twist is zeroed and
    /// the standing network drives (by magnitude where it is selectable, forced where it is
    /// reserved), exactly as the prototype's B-button mode behaves.
    ///
    /// `scale_mult` multiplies the action scale — voltage adaptation, 1.0 when off.
    pub fn step(
        &mut self,
        sensors: &duck_control::Sensors,
        command: &Command,
        body_active: bool,
        dt: f64,
        scale_mult: f64,
    ) -> Result<Step, PolicyError> {
        // Expire windows first, so a tick after the deadline runs the next thing rather
        // than one more frame of a finished move — the prototype checks its timers at the
        // same point relative to inference.
        if let Some((_, remaining)) = self.kick
            && remaining <= 0.0
        {
            self.kick = None;
        }
        // The end of a roll is a fork, as the prototype forks it: the button still held
        // (a request landed within the chain window) restarts the window — the policy
        // re-initiates a roll from wherever it landed — and released hands back.
        if let Some(remaining) = self.roulade
            && remaining <= 0.0
        {
            self.roulade = if self.roulade_chain > 0.0 {
                Some(self.skills.roulade_duration)
            } else {
                None
            };
        }
        if let Sit::Rising { remaining } = self.sit
            && remaining <= 0.0
        {
            self.sit = Sit::Up;
        }

        // Re-encode the command for the active skill and pick the network. The priority
        // chain is the prototype's: roulade > kick > ground pick > sit/rise > stand > walk.
        let (net, effective, label) = if self.roulade.is_some() {
            // Trained with every command slot at zero; it rolls as soon as it is switched
            // in, so being selected IS the trigger.
            (Net::Roulade, Command::default(), "roulade")
        } else if let Some((left, _)) = self.kick {
            // The kick networks are trained with every command slot at zero — head and
            // body included, whatever the client is holding.
            let net = if left { Net::KickLeft } else { Net::KickRight };
            let label = if left { "kick_left" } else { "kick_right" };
            (net, Command::default(), label)
        } else if let Some(phase) = self.ground_pick {
            // The twist slots carry the phase encoding; head and body are zero-padded,
            // mirroring the training env's `zero_command_padding`.
            let angle = std::f64::consts::TAU * phase;
            let c = Command {
                twist: [angle.cos(), angle.sin(), 0.0],
                ..Command::default()
            };
            (Net::GroundPick, c, "ground_pick")
        } else {
            let mut c = *command;
            match self.sit {
                // The posture flag rides the twist vx slot: 1 = sit, 0 = stand. Head and
                // body slots stay live — the prototype keeps them in the buffer too.
                Sit::Sitting => {
                    c.twist = [1.0, 0.0, 0.0];
                    (Net::SitStand, c, "sit")
                }
                Sit::Rising { .. } => {
                    c.twist = [0.0; 3];
                    (Net::SitStand, c, "rise")
                }
                Sit::Up => {
                    if body_active {
                        c.twist = [0.0; 3];
                    }
                    let standing = self.policy.will_stand(c.twist_magnitude())
                        || (body_active && self.policy.has_standing());
                    if standing {
                        (Net::Stand, c, "stand")
                    } else {
                        (Net::Walk, c, "walk")
                    }
                }
            }
        };

        let observation = Observation::build(
            &sensors.imu,
            &sensors.positions,
            &sensors.velocities,
            &DEFAULT_POSITION,
            &self.last_action,
            &effective,
        );

        let action = self.policy.infer(&observation, net)?;
        self.last_action = action;

        // Scale and gain follow the active state, recomputed every tick. "Standing tuning"
        // applies whenever the *effective* command is inside the standing threshold and the
        // standing network exists — which is how a kick window and the sitstand rise end up
        // at standing gain in the prototype, so they do here too.
        let standing_tuned = matches!(net, Net::Stand)
            || (matches!(net, Net::KickLeft | Net::KickRight | Net::SitStand)
                && self.policy.will_stand(effective.twist_magnitude()));
        let (scale, gain) = match net {
            Net::Roulade => (
                self.skills.roulade_action_scale,
                (self.tuning.gain as f64 * self.skills.roulade_gain_ratio).round() as u16,
            ),
            Net::GroundPick => (
                self.skills.ground_pick_action_scale,
                (self.tuning.gain as f64 * self.skills.ground_pick_gain_ratio).round() as u16,
            ),
            Net::SitStand => (
                // The prototype's `start_sit_toggle` pins the scale at 1.0 for the whole
                // sit/rise cycle.
                1.0,
                if standing_tuned {
                    (self.tuning.gain as f64 * self.tuning.standing_gain_ratio).round() as u16
                } else {
                    self.tuning.gain
                },
            ),
            _ if standing_tuned => (
                self.tuning.standing_action_scale,
                (self.tuning.gain as f64 * self.tuning.standing_gain_ratio).round() as u16,
            ),
            _ => (self.tuning.action_scale, self.tuning.gain),
        };
        let scale = scale * scale_mult;

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

        // Advance the windows, after the tick that used them — the prototype advances its
        // phase after the motor write.
        if let Some(phase) = self.ground_pick.as_mut() {
            *phase += dt / self.skills.ground_pick_period;
            if *phase >= GROUND_PICK_END_PHASE {
                self.ground_pick = None;
            }
        }
        if let Some((_, remaining)) = self.kick.as_mut() {
            *remaining -= dt;
        }
        if let Some(remaining) = self.roulade.as_mut() {
            *remaining -= dt;
            self.roulade_chain = (self.roulade_chain - dt).max(0.0);
        }
        if let Sit::Rising { remaining } = &mut self.sit {
            *remaining -= dt;
        }

        Ok(Step {
            targets,
            label,
            gain,
            busy: self.busy(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The prototype's **current alpha configuration** — its built-in defaults, which the
    /// installer deliberately passes no flags to override. The filters are ON at the values
    /// the policies are trained with; changing any of these silently changes how the robot
    /// moves relative to the thing it replaces.
    #[test]
    fn the_defaults_match_the_prototype() {
        let t = Tuning::default();
        assert_eq!(t.action_scale, 0.9);
        assert_eq!(t.standing_action_scale, 1.0);
        assert_eq!(t.standing_gain_ratio, 0.8);
        assert_eq!(
            t.head_lowpass,
            Some(0.5),
            "trained with ACTION_LOW_PASS_HEAD_ALPHA"
        );
        assert_eq!(
            t.legs_lowpass,
            Some(0.7),
            "trained with ACTION_LOW_PASS_LEG_ALPHA"
        );

        let s = SkillTuning::default();
        assert_eq!(s.ground_pick_period, 4.0);
        assert_eq!(s.ground_pick_action_scale, 1.0);
        assert_eq!(s.ground_pick_gain_ratio, 1.0);
        assert_eq!(s.kick_duration, 0.5);
        assert_eq!(s.roulade_duration, 1.0, "one roll, the measured time");
        assert_eq!(s.roulade_action_scale, 1.0);
        assert_eq!(
            s.roulade_gain_ratio, 1.0,
            "a roll runs at full walking gain"
        );
    }

    /// Standing must drop the gain. Running the standing policy at walking stiffness is a
    /// visibly different robot, and the ratio is the prototype's.
    #[test]
    fn standing_softens_the_gain() {
        let t = Tuning::default();
        let standing_gain = (t.gain as f64 * t.standing_gain_ratio).round() as u16;
        assert_eq!(standing_gain, 160);
        assert!(standing_gain < t.gain);
    }

    /// The ground pick ends at 70% of its cycle — ending at 100% replays the reach on the
    /// way out, which is the prototype bug the 0.7 cutoff fixed there.
    #[test]
    fn the_ground_pick_cutoff_is_the_prototypes() {
        assert_eq!(GROUND_PICK_END_PHASE, 0.7);
        assert_eq!(RISE_SECS, 1.0);
    }

    /// The External branch produces exactly the supplied targets — the whole point of a
    /// passthrough is that it does not reinterpret them. The clamping (anatomical limits, the
    /// per-tick step, the deadman) is the safety layer's job downstream, verified there; here
    /// what matters is that the Runtime hands the joint vector through untouched, labels the
    /// tick `external`, carries the caller's gain, and is not `busy`.
    ///
    /// Tested via `Step::external` rather than a full `Controller`, because constructing a
    /// `Controller` needs a loaded `Policy` (ONNX Runtime + a model file), which the External
    /// path deliberately never touches — skipping the policy stage is the point of the mode.
    #[test]
    fn the_external_branch_returns_the_supplied_targets() {
        let mut targets = DEFAULT_POSITION;
        targets[0] = 0.3;
        targets[duck_control::model::MOUTH_INDEX] = 0.2;

        let step = Step::external(targets, 175);

        assert_eq!(
            step.targets, targets,
            "the joint vector passes through unchanged"
        );
        assert_eq!(step.label, "external");
        assert_eq!(step.gain, 175);
        assert!(
            !step.busy,
            "External is a mode, not a scripted move mid-flight"
        );
    }
}
