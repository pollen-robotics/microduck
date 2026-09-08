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

/// How recently a request must have arrived, at the end of a chaining skill's window, for
/// another to start. The prototype chains roulade on "X still held at the window boundary";
/// here the client holds the button by re-sending the request every tick, so "held" is "a
/// request landed within the last few ticks". 150 ms is seven ticks — generous against a
/// dropped packet, far too short to mistake a fresh press for a hold.
const CHAIN_WINDOW: f64 = 0.15;

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

/// The scripted-skill numbers, resolved per mode by `params` — from the installed set's
/// manifest where it says, and from the prototype's literals where it does not.
#[derive(Debug, Clone, PartialEq)]
pub struct SkillTuning {
    /// One ground-pick cycle, seconds.
    pub ground_pick_period: f64,
    /// The ground pick hands back at this fraction of its cycle — the prototype's cutoff is
    /// 0.7. Ending at 1.0 replays the reach on the way out.
    pub ground_pick_end_phase: f64,
    pub ground_pick_action_scale: f64,
    /// Gain multiplier while the pick runs.
    pub ground_pick_gain_ratio: f64,
    /// How long the sitstand network rises (posture flag 0) before the main policy takes over.
    /// 1 s is enough on the robot — velstand owns the tail of the rise fine.
    pub sitstand_rise_s: f64,
    /// How long the seat takes to settle after the posture flag flips: the ~2 s glide the
    /// network is trained on. The shutdown sit waits twice this before cutting torque.
    pub sitstand_ramp_s: f64,
    /// The one-shot skills, in priority order — name, duration, whether holding chains, and
    /// what each changes about the robot while it runs. Config, resolved over the built-ins.
    pub skills: Vec<robotd_params::SkillDef>,
}

impl Default for SkillTuning {
    fn default() -> Self {
        Self {
            ground_pick_period: 4.0,
            ground_pick_end_phase: robotd_params::DEFAULT_GROUND_PICK_END_PHASE,
            ground_pick_action_scale: 1.0,
            ground_pick_gain_ratio: 1.0,
            sitstand_rise_s: robotd_params::DEFAULT_SITSTAND_RISE_S,
            sitstand_ramp_s: robotd_params::DEFAULT_SITSTAND_RAMP_S,
            skills: Vec::new(),
        }
    }
}

/// One tick's worth of decisions, for the caller to act on and report.
#[derive(Debug, Clone, PartialEq)]
pub struct Step {
    pub targets: [f64; NUM_JOINTS],
    /// Which network drove, as the wire label: `walk`, `stand`, `ground_pick`, `sit`, `rise`,
    /// or a configured skill's own name.
    ///
    /// Borrowed for the fixed set and owned for a skill, whose name comes from config rather
    /// than from this build — which is the whole point of a skill being config.
    pub label: std::borrow::Cow<'static, str>,
    /// What the gain should be for this tick.
    pub gain: u16,
    /// A scripted move is mid-flight — the robot is moving regardless of the twist, so
    /// restarting the daemon now would put it on the floor.
    pub busy: bool,
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

/// A one-shot skill mid-flight.
#[derive(Debug, Clone, Copy)]
struct ActiveSkill {
    /// Index into the resolved skill list, which is also [`Net::Skill`]'s index.
    index: usize,
    /// Whether it is doing the thing or coming back from it.
    phase: SkillPhase,
    /// Seconds left in this phase.
    remaining: f64,
    /// Seconds left during which another request still counts as the button being held.
    /// Roulade's chaining, generalised: counted down every tick, refreshed by each request
    /// that lands while the skill runs, and positive at the end of a window means start
    /// another.
    chain: f64,
}

/// Which half of a skill is running.
///
/// Most skills only ever have the first. The second exists for a policy that does not end
/// itself — one that holds until told otherwise — where handing straight back to walk would
/// give it a robot mid-pose.
#[derive(Debug, Clone, Copy, PartialEq)]
enum SkillPhase {
    /// Driving `command`, for `duration`.
    Holding,
    /// Driving `unwind`, for `unwind_s`, before handing back.
    Unwinding,
}

/// Which network drove the robot on the last step, as a policy change needs to know it.
///
/// [`Net`] with the skill index replaced by the skill's name, because an index is only meaningful
/// against one skill list and a change may be about to install another.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Driving {
    Walk,
    Stand,
    /// The sitstand network, in motion: rising, or the tick the flag flipped.
    SitStand,
    /// The sitstand network holding the seat. Parked, not travelling — the one driving state a
    /// network can be swapped under without the robot noticing, which is why it is its own
    /// variant rather than a flag beside `SitStand`.
    Seated,
    GroundPick,
    /// A one-shot skill, by its config name.
    Skill(String),
}

impl std::fmt::Display for Driving {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Driving::Walk => f.write_str("walk"),
            Driving::Stand => f.write_str("stand"),
            Driving::SitStand => f.write_str("sitstand"),
            Driving::Seated => f.write_str("seated"),
            Driving::GroundPick => f.write_str("ground_pick"),
            Driving::Skill(name) => f.write_str(name),
        }
    }
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
    /// Ground-pick phase, 0..`skills.ground_pick_end_phase`. `None` when inactive.
    ground_pick: Option<f64>,
    /// **A running skill switches the fall reflex off**, because `busy()` is what gates the
    /// limp-fall predictor and any active skill makes it true. That was uncontroversial when
    /// every one-shot was under a second; a skill configured to hold for ten is a robot with no
    /// fall reflex for ten seconds, which wants deciding rather than inheriting.
    ///
    /// The one-shot skill driving right now: which one, and how long it has left.
    ///
    /// One field where there were three, because a kick and a roulade were the same thing
    /// with different numbers.
    active: Option<ActiveSkill>,
    sit: Sit,
    /// The network the last [`Self::step`] ran. `None` before the first.
    last_net: Option<Net>,
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
            active: None,
            sit: Sit::Up,
            last_net: None,
        }
    }

    /// The network that drove on the last step, or `None` before there was one.
    pub fn driving(&self) -> Option<Driving> {
        Some(match self.last_net? {
            Net::Walk => Driving::Walk,
            Net::Stand => Driving::Stand,
            Net::SitStand if self.sit == Sit::Sitting => Driving::Seated,
            Net::SitStand => Driving::SitStand,
            Net::GroundPick => Driving::GroundPick,
            Net::Skill(index) => Driving::Skill(
                self.skills
                    .skills
                    .get(index)
                    .map_or_else(|| index.to_string(), |d| d.name.clone()),
            ),
        })
    }

    /// Pick up where `from` left off.
    ///
    /// For a controller built to replace one whose driving network it has *not* changed — a new
    /// `walk` under a robot that is sitting, a new `stand` under one that is walking. The seat,
    /// the skill mid-flight, the ground-pick phase, the last action the observation feeds back
    /// and the low-pass anchor all carry across, so the swap is invisible: the next tick runs the
    /// same network from the same state, and the replaced one is simply there when it is next
    /// selected. A fresh controller in its place would start `Sit::Up`, which under a seated
    /// robot is a stand-up nobody asked for.
    ///
    /// The caller has checked that the network driving is unchanged, and that the skill list is
    /// unchanged if a skill is what is driving — `active` addresses that list by index. The one
    /// exception is a seated robot, which carries across whatever changed: the seat is a static
    /// pose on a constant flag, and any sitstand network asked to hold it holds it.
    pub fn carry_over(&mut self, from: &Controller) {
        self.last_action = from.last_action;
        self.previous = from.previous;
        self.ground_pick = from.ground_pick;
        self.active = from.active;
        self.sit = from.sit;
        self.last_net = from.last_net;
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

    /// How long a shutdown sit gets before torque is cut: twice the seat's settle time, which
    /// is the prototype's four seconds over its ~2 s glide.
    pub fn shutdown_sit_secs(&self) -> f64 {
        2.0 * self.skills.sitstand_ramp_s
    }

    pub fn is_sitting(&self) -> bool {
        self.sit == Sit::Sitting
    }

    /// A scripted move is mid-flight. Sitting itself is not busy — a seated robot is
    /// parked, not travelling.
    pub fn busy(&self) -> bool {
        self.ground_pick.is_some()
            || self.active.is_some()
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

    /// Every one-shot skill this robot has, in priority order — what a client may ask for.
    pub fn skill_names(&self) -> Vec<String> {
        self.skills
            .skills
            .iter()
            .take(self.policy.skill_count())
            .map(|skill| skill.name.clone())
            .collect()
    }

    /// Start a one-shot skill, or — for a chaining one already running — keep the chain alive.
    ///
    /// `Ok(true)` started it; `Ok(false)` refreshed a running one, and the caller should stay
    /// quiet about that, because a held button lands here fifty times a second.
    ///
    /// The gating is the prototype's, generalised. A scripted move blocks another, except that
    /// a chaining skill may preempt one that is not chaining — which is how the X press could
    /// always roll out of a kick's tail or out of the seat. A ground pick blocks everything, as
    /// it always has.
    pub fn start_skill(&mut self, index: usize) -> Result<bool, &'static str> {
        let Some(def) = self.skills.skills.get(index) else {
            return Err("no such skill on this robot");
        };
        let (duration, chains) = (def.duration, def.chain);

        if let Some(active) = &mut self.active {
            if active.index == index && chains {
                active.chain = CHAIN_WINDOW;
                return Ok(false);
            }
            if !chains {
                return Err("a scripted move is already running");
            }
        }
        if self.ground_pick.is_some() {
            return Err("a ground pick is running");
        }
        self.active = Some(ActiveSkill {
            index,
            phase: SkillPhase::Holding,
            remaining: duration,
            chain: 0.0,
        });
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
                    remaining: self.skills.sitstand_rise_s,
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
            remaining: self.skills.sitstand_rise_s,
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
        // The end of a window is a fork, as the prototype forks a roll: the button still held
        // — a request landed within the chain window — restarts it, and released hands back.
        // Only a chaining skill can take the first branch, so a kick still ends when it ends.
        if let Some(active) = self.active
            && active.remaining <= 0.0
        {
            let def = self.skills.skills.get(active.index);
            self.active = match active.phase {
                // A skill that does not end itself comes back first, so walk never inherits a
                // robot mid-pose. `unwind_s` of zero — the common case — skips straight past.
                SkillPhase::Holding if def.is_some_and(|d| d.unwind_s > 0.0) => Some(ActiveSkill {
                    phase: SkillPhase::Unwinding,
                    remaining: def.map_or(0.0, |d| d.unwind_s),
                    ..active
                }),
                // The end of a window is a fork, as the prototype forks a roll: the button still
                // held — a request landed within the chain window — restarts it, and released
                // hands back. Only a chaining skill can take that branch, so a kick still ends
                // when it ends, and a skill that unwound is finished either way.
                SkillPhase::Holding | SkillPhase::Unwinding => {
                    let chains = active.phase == SkillPhase::Holding
                        && def.is_some_and(|d| d.chain)
                        && active.chain > 0.0;
                    chains.then(|| ActiveSkill {
                        phase: SkillPhase::Holding,
                        remaining: def.map_or(0.0, |d| d.duration),
                        chain: 0.0,
                        ..active
                    })
                }
            };
        }
        if let Sit::Rising { remaining } = self.sit
            && remaining <= 0.0
        {
            self.sit = Sit::Up;
        }

        // Re-encode the command for the active skill and pick the network. The priority chain
        // is the prototype's, with its three one-shots now one entry: skill > ground pick >
        // sit/rise > stand-by-magnitude > walk, and the skills themselves ordered by config.
        let (net, effective, label) = if let Some(active) = self.active {
            // Head and body are zeroed whatever the phase — every one-shot published so far
            // declares them unused, and a policy trained with `zero_command_padding` expects
            // exactly that. Only the twist differs, and for most skills it is zero too, which is
            // what made the kick and roulade arms the same arm.
            let def = self.skills.skills.get(active.index);
            let twist = def.map_or([0.0; 3], |d| match active.phase {
                SkillPhase::Holding => d.command,
                SkillPhase::Unwinding => d.unwind,
            });
            let label = def.map_or(std::borrow::Cow::Borrowed("skill"), |d| {
                std::borrow::Cow::Owned(match active.phase {
                    SkillPhase::Holding => d.name.clone(),
                    SkillPhase::Unwinding => format!("{}:unwind", d.name),
                })
            });
            let c = Command {
                twist,
                ..Command::default()
            };
            (Net::Skill(active.index), c, label)
        } else if let Some(phase) = self.ground_pick {
            // The twist slots carry the phase encoding; head and body are zero-padded,
            // mirroring the training env's `zero_command_padding`.
            let angle = std::f64::consts::TAU * phase;
            let c = Command {
                twist: [angle.cos(), angle.sin(), 0.0],
                ..Command::default()
            };
            (Net::GroundPick, c, "ground_pick".into())
        } else {
            let mut c = *command;
            match self.sit {
                // The posture flag rides the twist vx slot: 1 = sit, 0 = stand. Head and
                // body slots stay live — the prototype keeps them in the buffer too.
                Sit::Sitting => {
                    c.twist = [1.0, 0.0, 0.0];
                    (Net::SitStand, c, "sit".into())
                }
                Sit::Rising { .. } => {
                    c.twist = [0.0; 3];
                    (Net::SitStand, c, "rise".into())
                }
                Sit::Up => {
                    if body_active {
                        c.twist = [0.0; 3];
                    }
                    let standing = self.policy.will_stand(c.twist_magnitude())
                        || (body_active && self.policy.has_standing());
                    if standing {
                        (Net::Stand, c, "stand".into())
                    } else {
                        (Net::Walk, c, "walk".into())
                    }
                }
            }
        };

        self.last_net = Some(net);

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
            || (matches!(net, Net::Skill(_) | Net::SitStand)
                && self.policy.will_stand(effective.twist_magnitude()));
        let (scale, gain) = match net {
            // A skill's own overrides, falling back to the gait's — which is how a kick keeps
            // running at the standing tuning it was tuned against while a roulade can ask for
            // something else.
            Net::Skill(index) => {
                let overrides = self.skills.skills.get(index).map(|d| &d.params);
                let scale = overrides
                    .and_then(|o| o.action_scale)
                    .unwrap_or(if standing_tuned {
                        self.tuning.standing_action_scale
                    } else {
                        self.tuning.action_scale
                    });
                let ratio = overrides.and_then(|o| o.gain_ratio).unwrap_or({
                    if standing_tuned {
                        self.tuning.standing_gain_ratio
                    } else {
                        1.0
                    }
                });
                (scale, (self.tuning.gain as f64 * ratio).round() as u16)
            }
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
            if *phase >= self.skills.ground_pick_end_phase {
                self.ground_pick = None;
            }
        }
        if let Some(active) = self.active.as_mut() {
            active.remaining -= dt;
            active.chain = (active.chain - dt).max(0.0);
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
        assert_eq!(s.ground_pick_end_phase, 0.7);
        assert_eq!(s.ground_pick_action_scale, 1.0);
        assert_eq!(s.ground_pick_gain_ratio, 1.0);
        assert_eq!(s.sitstand_rise_s, 1.0);
        assert_eq!(s.sitstand_ramp_s, 2.0);
        // The one-shots' numbers live with the skills now — `robotd_params` owns the built-in
        // three and asserts their durations, and this struct simply carries the resolved list.
        assert!(
            s.skills.is_empty(),
            "a bare tuning has no skills of its own"
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
    /// way out, which is the prototype bug the 0.7 cutoff fixed there. The cutoff and the rise
    /// come from the set's manifest now; these are what a board with no manifest gets.
    #[test]
    fn the_ground_pick_cutoff_is_the_prototypes() {
        assert_eq!(robotd_params::DEFAULT_GROUND_PICK_END_PHASE, 0.7);
        assert_eq!(robotd_params::DEFAULT_SITSTAND_RISE_S, 1.0);
    }
}
