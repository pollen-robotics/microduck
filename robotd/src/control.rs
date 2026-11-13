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
//!
//! A second: **one move at a time, and none from the seat** ([`Controller::move_blocked`]). The
//! prototype let a pick preempt a kick's tail and a chaining skill roll out of a kick or the seat;
//! here each is refused, and a seated robot accepts only standing up.

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
    /// network is trained on. The shutdown sit waits this plus a second before easing into the
    /// rest pose.
    pub sitstand_ramp_s: f64,
    /// The one-shot skills, in priority order — name, duration, whether holding chains, and
    /// what each changes about the robot while it runs. Config, resolved over the built-ins.
    pub skills: Vec<robotd_params::SkillDef>,
    /// Where the seat is the ground-pick network held mid-cycle rather than the sitstand
    /// network: the ground-pick phase it stops at. Roller mode — the sitstand network cannot
    /// sit a robot on wheels, and the roller crouch is the lowest pose that one has.
    /// See [`ROLLER_SEAT_PHASE`].
    pub crouch_seat: Option<f64>,
}

/// Where a roller robot's seat holds the crouch: the middle of its crouched plateau. The roller
/// crouch is trained as one cycle — down by phase 0.10, crouched to 0.50, up by 0.60
/// (`microduck_roller_crouch_env_cfg.py` in `microduck_rl`) — and this is a phase every episode
/// passes through, held rather than passed. Standing up resumes the cycle from here, so the rise
/// is the one the network was trained on.
pub const ROLLER_SEAT_PHASE: f64 = 0.3;

/// The part of the roller crouch's cycle that is nearer the bottom than the top: from halfway
/// down (the descent ends at 0.10) to halfway up (the rise runs 0.50–0.60). A stop asked for in
/// here stays down — see [`Controller::crouching_low`].
pub const ROLLER_LOW_PHASES: std::ops::RangeInclusive<f64> = 0.05..=0.55;

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
            crouch_seat: None,
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
    /// Seconds left of the glide down into the seat after a sit was asked for. The seat is
    /// `Sit::Sitting` from the first tick, but the robot is still travelling, and standing it
    /// back up halfway down is one move cut into another.
    sit_settle: f64,
    /// Stand up by itself once `sit_settle` runs out — [`Self::settle_then_rise`].
    auto_rise: bool,
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
            sit_settle: 0.0,
            auto_rise: false,
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
        self.policy.carry_over(&from.policy);
        self.last_action = from.last_action;
        self.previous = from.previous;
        self.ground_pick = from.ground_pick;
        self.active = from.active;
        self.sit = from.sit;
        self.sit_settle = from.sit_settle;
        self.auto_rise = from.auto_rise;
        self.last_net = from.last_net;
    }

    /// Reset the feedback state.
    ///
    /// Called when the policy is re-enabled, so a robot that sat disabled for a minute does
    /// not resume with a stale action in its observation and a filter anchored to wherever
    /// it was before. Recurrent networks also discard their episode memory.
    pub fn reset(&mut self) {
        self.policy.reset();
        self.last_action = [0.0; ACTION_LEN];
        self.previous = None;
    }

    /// The policy was stopped on purpose: end whatever move is in flight, and keep the seat.
    ///
    /// A move cut short is not resumed on the next Start — its window would pick up mid-way, on
    /// a robot that has been standing at home in between. A rise cut short counts as standing,
    /// since that is where it was going. The seat is kept because the robot is still sitting:
    /// the caller holds it there, and the next Start hands it back to the sitstand network
    /// rather than to a gait that would try to walk out of a chair.
    pub fn stop_moves(&mut self) {
        self.ground_pick = None;
        self.active = None;
        self.sit_settle = 0.0;
        self.auto_rise = false;
        if matches!(self.sit, Sit::Rising { .. }) {
            self.sit = Sit::Up;
        }
    }

    /// Forget everything about where the robot was: the seat, any move in flight, and the
    /// feedback state. For when torque went away — a relax, a servo reboot — and whatever the
    /// robot was doing before is no longer what it is doing. The next bring-up starts from a
    /// standing robot's state, as after a boot.
    pub fn forget(&mut self) {
        self.reset();
        self.ground_pick = None;
        self.active = None;
        self.sit = Sit::Up;
        self.sit_settle = 0.0;
        self.auto_rise = false;
    }

    /// Why a new move cannot start now, or `None` when it can.
    ///
    /// One move at a time, and none from the seat: a kick or a pick from a sitting robot
    /// throws it over, and one move started inside another hands the second network a robot
    /// mid-pose it was never trained from.
    fn move_blocked(&self) -> Option<&'static str> {
        if self.ground_pick.is_some() && self.sit == Sit::Up {
            return Some("a ground pick is running");
        }
        if self.active.is_some() {
            return Some("a scripted move is already running");
        }
        match self.sit {
            Sit::Sitting => Some("the robot is sitting — stand it up first"),
            Sit::Rising { .. } => Some("the robot is standing up"),
            Sit::Up => None,
        }
    }

    /// The phase a crouch seat holds at, when the seat is a crouch and the crouch is loaded.
    fn crouch_seat(&self) -> Option<f64> {
        self.skills
            .crouch_seat
            .filter(|_| self.policy.has_ground_pick())
    }

    /// Whether this robot can sit at all: the sitstand network, or on wheels the held crouch.
    pub fn has_seat(&self) -> bool {
        match self.skills.crouch_seat {
            Some(_) => self.policy.has_ground_pick(),
            None => self.policy.has_sitstand(),
        }
    }

    /// How long the seat takes to reach: the sitstand glide, or the crouch's way down.
    fn seat_settle_secs(&self) -> f64 {
        match self.crouch_seat() {
            Some(hold) => hold * self.skills.ground_pick_period,
            None => self.skills.sitstand_ramp_s,
        }
    }

    /// How long the policy holds the shutdown sit before the joints ease into the rest pose:
    /// the seat's settle time, then a second sitting still.
    pub fn shutdown_sit_secs(&self) -> f64 {
        self.seat_settle_secs() + 1.0
    }

    pub fn is_sitting(&self) -> bool {
        self.sit == Sit::Sitting
    }

    /// A roller crouch is running and is nearer its bottom than its top. Start then holds it
    /// down as the seat rather than stopping the policy: cutting the policy there sends a
    /// crouched robot on wheels straight to the standing home pose, and over.
    pub fn crouching_low(&self) -> bool {
        self.crouch_seat().is_some()
            && self.sit == Sit::Up
            && self
                .ground_pick
                .is_some_and(|p| ROLLER_LOW_PHASES.contains(&p))
    }

    /// A scripted move is mid-flight. Sitting itself is not busy — a seated robot is
    /// parked, not travelling, and that includes one parked in a held crouch.
    pub fn busy(&self) -> bool {
        (self.ground_pick.is_some() && self.sit == Sit::Up)
            || self.active.is_some()
            || matches!(self.sit, Sit::Rising { .. })
    }

    /// Start a one-shot ground pick. Refused while any other move runs or the robot is
    /// sitting — see [`Self::move_blocked`]. The prototype let a pick preempt a kick's tail;
    /// that was a pick starting from a robot on one leg.
    pub fn start_ground_pick(&mut self) -> Result<(), &'static str> {
        if !self.policy.has_ground_pick() {
            return Err("no ground-pick policy loaded");
        }
        if let Some(reason) = self.move_blocked() {
            return Err(reason);
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
    /// One move at a time and none from the seat — see [`Self::move_blocked`]. The one thing a
    /// running skill accepts is a request for itself when it chains: that is the button being
    /// held, not a second move. The prototype also let a chaining skill preempt a kick's tail or
    /// roll out of the seat; both were a move started from a pose it was not trained from.
    pub fn start_skill(&mut self, index: usize) -> Result<bool, &'static str> {
        let Some(def) = self.skills.skills.get(index) else {
            return Err("no such skill on this robot");
        };
        let (duration, chains) = (def.duration, def.chain);

        if let Some(active) = &mut self.active
            && active.index == index
            && chains
        {
            active.chain = CHAIN_WINDOW;
            return Ok(false);
        }
        if let Some(reason) = self.move_blocked() {
            return Err(reason);
        }
        self.active = Some(ActiveSkill {
            index,
            phase: SkillPhase::Holding,
            remaining: duration,
            chain: 0.0,
        });
        Ok(true)
    }

    /// Sit if standing, stand if sitting. Refused while any other move runs, mid-rise as the
    /// prototype refuses it, and on the way down into the seat.
    ///
    /// On wheels the seat is the crouch held at [`ROLLER_SEAT_PHASE`]: sitting runs the crouch
    /// down to it and stops there, and standing lets the crouch finish its cycle. Sitting during
    /// a crouch already running keeps it down — back to the hold if it had passed it.
    pub fn sit_toggle(&mut self) -> Result<&'static str, &'static str> {
        if let (Some(hold), Some(phase), Sit::Up) = (self.crouch_seat(), self.ground_pick, self.sit)
        {
            self.sit = Sit::Sitting;
            self.ground_pick = Some(phase.min(hold));
            self.sit_settle = (hold - phase).max(0.0) * self.skills.ground_pick_period;
            return Ok("sit");
        }
        if self.ground_pick.is_some() && self.sit == Sit::Up {
            return Err("a ground pick is running");
        }
        if self.active.is_some() {
            return Err("a scripted move is already running");
        }
        match self.sit {
            Sit::Up => {
                if !self.has_seat() {
                    return Err(if self.skills.crouch_seat.is_some() {
                        "no crouch policy loaded to sit with"
                    } else {
                        "no sitstand policy loaded"
                    });
                }
                self.sit = Sit::Sitting;
                self.sit_settle = self.seat_settle_secs();
                if self.crouch_seat().is_some() {
                    self.ground_pick = Some(0.0);
                }
                Ok("sit")
            }
            Sit::Sitting if self.sit_settle > 0.0 => Err("still sitting down"),
            // The crouch stands up by finishing its own cycle, as a ground pick.
            Sit::Sitting if self.crouch_seat().is_some() => {
                self.sit = Sit::Up;
                Ok("stand")
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
    ///
    /// On wheels that is the crouch, from the top unless one is already on its way down.
    pub fn begin_shutdown_sit(&mut self) {
        if let Some(hold) = self.crouch_seat()
            && self.sit == Sit::Up
        {
            self.ground_pick = Some(self.ground_pick.filter(|&p| p <= hold).unwrap_or(0.0));
        }
        self.sit = Sit::Sitting;
    }

    /// The robot was found sitting as torque came on: it is held where it is, and this is the seat
    /// it is in. Already settled — it did not just sit down, so standing up is not refused as
    /// "still sitting down".
    pub fn enter_seat(&mut self) {
        self.sit = Sit::Sitting;
        self.sit_settle = 0.0;
    }

    /// The robot is not in a seat after all — measured standing as the policy took over: forget
    /// the one this controller believed in, so the gait rather than the sitstand network drives.
    pub fn leave_seat(&mut self) {
        if self.sit == Sit::Sitting {
            self.sit = Sit::Up;
        }
        self.sit_settle = 0.0;
        self.auto_rise = false;
    }

    /// The robot was found sitting as the policy took over: settle into the seat with the
    /// sitstand network for the seat's settle time, then rise, then the gait.
    ///
    /// The same sequence as a rise while the robot runs, which is the one that works: there the
    /// network has held the seat for a while and rises from *its* seat with a warm history. Rising
    /// on the first tick instead started it cold, from a seat it had not chosen.
    pub fn settle_then_rise(&mut self) {
        self.sit = Sit::Sitting;
        self.sit_settle = self.seat_settle_secs();
        self.auto_rise = true;
    }

    /// Start the feedback state from a pose the robot is holding rather than from zero.
    ///
    /// [`Self::reset`] zeroes the previous action and drops the low-pass anchor, which is right
    /// for a robot standing at home — a zero action *is* the home pose — and wrong for one held
    /// anywhere else: the network would observe "I just commanded home" from a seat, and its
    /// first targets would go out unfiltered. Here the previous action is the offset that would
    /// have produced `pose` (the sitstand network runs at action scale 1), and the filter starts
    /// from it.
    pub fn seed_from_pose(&mut self, pose: &[f64; NUM_JOINTS]) {
        let offsets: [f64; NUM_JOINTS] = std::array::from_fn(|j| pose[j] - DEFAULT_POSITION[j]);
        self.last_action = Observation::gather_action(&offsets);
        self.previous = Some(*pose);
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
                    if chains {
                        // A chained replay is a new episode even though Net is unchanged.
                        self.policy.reset();
                    }
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
        if self.auto_rise && self.sit == Sit::Sitting && self.sit_settle <= 0.0 {
            self.auto_rise = false;
            self.sit = Sit::Rising {
                remaining: self.skills.sitstand_rise_s,
            };
        }
        // On wheels the seat is the crouch, whichever way the robot came to it. A seat with no
        // crouch phase is one the robot is already in — kept across a stop, or found at
        // bring-up — so it resumes at the hold; and a rise is the crouch finishing its cycle.
        if let Some(hold) = self.crouch_seat() {
            match self.sit {
                Sit::Sitting => {
                    self.ground_pick.get_or_insert(hold);
                }
                Sit::Rising { .. } => {
                    self.sit = Sit::Up;
                    self.ground_pick = Some(self.ground_pick.map_or(hold, |p| p.max(hold)));
                }
                Sit::Up => {}
            }
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
        let seat_hold = self.crouch_seat().filter(|_| self.sit == Sit::Sitting);
        if let Some(phase) = self.ground_pick.as_mut() {
            *phase += dt / self.skills.ground_pick_period;
            if let Some(hold) = seat_hold {
                *phase = phase.min(hold);
            } else if *phase >= self.skills.ground_pick_end_phase {
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
        self.sit_settle = (self.sit_settle - dt).max(0.0);

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

    #[test]
    #[ignore = "requires ONNX Runtime >= 1.23"]
    fn recurrent_memory_resets_with_controller_feedback() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../duck-control/tests/fixtures/lstm.onnx");
        let policy = Policy::load(
            &duck_control::policy::PolicyPaths {
                walk: path,
                ..Default::default()
            },
            0.05,
        )
        .unwrap();
        let mut controller = Controller::new(policy, Tuning::default(), SkillTuning::default());
        let sensors = duck_control::Sensors::default();
        let command = Command::default();
        let first = controller
            .step(&sensors, &command, false, 0.02, 1.0)
            .unwrap()
            .targets;
        let next = controller
            .step(&sensors, &command, false, 0.02, 1.0)
            .unwrap()
            .targets;
        assert_ne!(first, next);
        controller.reset();
        assert_eq!(
            first,
            controller
                .step(&sensors, &command, false, 0.02, 1.0)
                .unwrap()
                .targets
        );
    }

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

    /// A controller with every slot loaded — the feedforward fixture stands in for each net, which
    /// is all the gating needs — and two skills: a kick that does not chain and a roll that does.
    fn full_controller() -> Controller {
        let net = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../duck-control/tests/fixtures/feedforward.onnx");
        let policy = Policy::load(
            &duck_control::policy::PolicyPaths {
                walk: net.clone(),
                sitstand: Some(net.clone()),
                ground_pick: Some(net.clone()),
                skills: vec![net.clone(), net],
                ..Default::default()
            },
            0.05,
        )
        .unwrap();
        let skill = |name: &str, duration: f64, chain: bool| robotd_params::SkillDef {
            name: name.to_owned(),
            duration,
            chain,
            ..Default::default()
        };
        Controller::new(
            policy,
            Tuning::default(),
            SkillTuning {
                skills: vec![skill("kick_left", 0.5, false), skill("roulade", 1.0, true)],
                ..SkillTuning::default()
            },
        )
    }

    fn tick(controller: &mut Controller, seconds: f64) {
        let sensors = duck_control::Sensors::default();
        for _ in 0..(seconds / 0.02).round() as usize {
            controller
                .step(&sensors, &Command::default(), false, 0.02, 1.0)
                .unwrap();
        }
    }

    /// **Nothing starts from the seat but standing up.** A kick or a pick from a sitting robot
    /// throws it over.
    #[test]
    #[ignore = "requires ONNX Runtime >= 1.23"]
    fn a_seated_robot_refuses_every_move_but_standing_up() {
        let mut c = full_controller();
        assert_eq!(c.sit_toggle(), Ok("sit"));
        tick(&mut c, 3.0);
        assert!(c.is_sitting());

        assert!(c.start_ground_pick().is_err());
        assert!(c.start_skill(0).is_err(), "no kick from the seat");
        assert!(c.start_skill(1).is_err(), "no roll out of the seat either");
        assert!(c.is_sitting() && !c.busy(), "and the seat is untouched");

        assert_eq!(c.sit_toggle(), Ok("stand"));
        assert!(c.start_ground_pick().is_err(), "not while it rises");
        tick(&mut c, 1.1);
        assert_eq!(c.start_ground_pick(), Ok(()), "standing again, it may");
    }

    /// **On wheels the seat is the crouch, held.** Sitting runs the crouch down to
    /// [`ROLLER_SEAT_PHASE`] and stays there for as long as it is seated; standing lets the
    /// crouch finish its cycle and hands back to the gait. The rest and shutdown sits take the
    /// same seat, and a stop keeps it.
    #[test]
    #[ignore = "requires ONNX Runtime >= 1.23"]
    fn on_wheels_the_seat_is_the_crouch_held() {
        let mut c = full_controller();
        c.skills.crouch_seat = Some(ROLLER_SEAT_PHASE);
        c.skills.ground_pick_period = 5.0;
        assert!(c.has_seat());
        assert_eq!(c.shutdown_sit_secs(), 0.3 * 5.0 + 1.0);

        assert_eq!(c.sit_toggle(), Ok("sit"));
        assert_eq!(c.sit_toggle(), Err("still sitting down"));
        tick(&mut c, 10.0);
        assert!(c.is_sitting() && !c.busy(), "parked in the crouch");
        assert_eq!(
            c.driving(),
            Some(Driving::GroundPick),
            "the crouch holds it"
        );
        assert_eq!(
            c.ground_pick,
            Some(ROLLER_SEAT_PHASE),
            "at the hold, however long"
        );
        assert!(c.start_ground_pick().is_err(), "no pick from the seat");

        c.stop_moves();
        assert!(c.is_sitting(), "a stop keeps the seat");
        tick(&mut c, 0.02);
        assert_eq!(
            c.ground_pick,
            Some(ROLLER_SEAT_PHASE),
            "and resumes it at the hold"
        );

        assert_eq!(c.sit_toggle(), Ok("stand"));
        assert!(
            !c.is_sitting() && c.busy(),
            "rising: the crouch finishes its cycle"
        );
        tick(&mut c, (0.7 - ROLLER_SEAT_PHASE) * 5.0 + 0.1);
        assert!(
            !c.busy() && c.ground_pick.is_none(),
            "standing: the gait has it"
        );
        assert_eq!(c.driving(), Some(Driving::Walk));

        c.begin_shutdown_sit();
        tick(&mut c, 3.0);
        assert!(c.is_sitting());
        assert_eq!(
            c.ground_pick,
            Some(ROLLER_SEAT_PHASE),
            "the shutdown sit is the crouch"
        );
    }

    /// **A sit during a running crouch keeps it down.** Low in the cycle the crouch stops at the
    /// hold — back to it if it had passed it — and is the seat; high in it, it is not low, and
    /// Start stops the policy as ever.
    #[test]
    #[ignore = "requires ONNX Runtime >= 1.23"]
    fn on_wheels_a_sit_mid_crouch_holds_it_down() {
        let mut c = full_controller();
        c.skills.crouch_seat = Some(ROLLER_SEAT_PHASE);
        c.skills.ground_pick_period = 5.0;

        assert_eq!(c.start_ground_pick(), Ok(()));
        assert!(!c.crouching_low(), "at the top of the cycle");
        tick(&mut c, 0.1 * 5.0);
        assert!(c.crouching_low(), "down");
        tick(&mut c, 0.35 * 5.0);
        assert!(c.crouching_low(), "still down, past the hold");
        assert_eq!(c.sit_toggle(), Ok("sit"));
        assert!(c.is_sitting() && !c.crouching_low());
        tick(&mut c, 10.0);
        assert_eq!(
            c.ground_pick,
            Some(ROLLER_SEAT_PHASE),
            "back at the hold, and held"
        );
        assert_eq!(c.sit_toggle(), Ok("stand"));
        tick(&mut c, 3.0);
        assert!(!c.busy() && !c.is_sitting(), "risen: the gait has it");

        assert_eq!(c.start_ground_pick(), Ok(()));
        tick(&mut c, 0.58 * 5.0);
        assert!(!c.crouching_low(), "nearly up: not low");
    }

    /// The glide down into the seat is a move too: standing up halfway down is refused.
    #[test]
    #[ignore = "requires ONNX Runtime >= 1.23"]
    fn sitting_down_cannot_be_cut_short() {
        let mut c = full_controller();
        assert_eq!(c.sit_toggle(), Ok("sit"));
        tick(&mut c, 1.0);
        assert!(c.sit_toggle().is_err());
        tick(&mut c, 1.1);
        assert_eq!(c.sit_toggle(), Ok("stand"));
    }

    /// **One move at a time.** No kick inside a pick, no pick inside a kick, no sit inside
    /// either, and a chaining skill no longer preempts a kick's tail.
    #[test]
    #[ignore = "requires ONNX Runtime >= 1.23"]
    fn a_move_in_flight_blocks_every_other() {
        let mut c = full_controller();
        assert_eq!(c.start_ground_pick(), Ok(()));
        assert!(c.start_skill(0).is_err(), "no kick during a pick");
        assert!(c.sit_toggle().is_err(), "no sit during a pick");
        tick(&mut c, 3.0);
        assert!(!c.busy(), "the pick has ended");

        assert_eq!(c.start_skill(0), Ok(true));
        assert!(c.start_ground_pick().is_err(), "no pick during a kick");
        assert!(c.start_skill(1).is_err(), "no roll cutting into a kick");
        assert!(c.sit_toggle().is_err(), "no sit during a kick");
        tick(&mut c, 0.6);

        // The held button is still the held button: a chaining skill refreshes itself.
        assert_eq!(c.start_skill(1), Ok(true));
        assert_eq!(c.start_skill(1), Ok(false), "a hold, not a second move");
    }

    /// A seat found at torque-on is already settled: it refuses moves like any seat, and stands
    /// up at once rather than as "still sitting down".
    #[test]
    #[ignore = "requires ONNX Runtime >= 1.23"]
    fn a_seat_found_at_torque_on_is_settled() {
        let mut c = full_controller();
        c.enter_seat();
        assert!(c.is_sitting());
        assert!(c.start_ground_pick().is_err(), "no pick from it");
        assert_eq!(c.sit_toggle(), Ok("stand"), "no settle to wait out");
    }

    /// Found sitting at the start: it sits (settles) first, refusing everything meanwhile, then
    /// rises by itself and ends standing.
    #[test]
    #[ignore = "requires ONNX Runtime >= 1.23"]
    fn a_seat_found_at_the_start_settles_then_rises_by_itself() {
        let mut c = full_controller();
        c.settle_then_rise();
        tick(&mut c, 1.0);
        assert!(c.is_sitting(), "still settling");
        assert!(c.sit_toggle().is_err(), "no toggling mid-settle");
        tick(&mut c, 1.1);
        assert!(!c.is_sitting() && c.busy(), "rising by itself");
        tick(&mut c, 1.1);
        assert!(!c.is_sitting() && !c.busy(), "standing: the gait has it");
    }

    /// Seeding from a pose makes the previous action the offset that pose is from home.
    #[test]
    #[ignore = "requires ONNX Runtime >= 1.23"]
    fn seeding_from_a_pose_sets_the_previous_action() {
        let mut c = full_controller();
        let mut pose = DEFAULT_POSITION;
        pose[3] += 1.0;
        c.seed_from_pose(&pose);
        assert_eq!(c.previous, Some(pose));
        assert!((c.last_action[3] - 1.0).abs() < 1e-6);
        assert!(
            c.last_action
                .iter()
                .enumerate()
                .all(|(i, a)| i == 3 || *a == 0.0)
        );
    }

    /// A deliberate stop keeps the seat and drops the move in flight; a reset forgets both.
    #[test]
    #[ignore = "requires ONNX Runtime >= 1.23"]
    fn a_stop_keeps_the_seat_and_a_reset_forgets_it() {
        let mut c = full_controller();
        assert_eq!(c.sit_toggle(), Ok("sit"));
        tick(&mut c, 3.0);
        c.stop_moves();
        assert!(c.is_sitting(), "still sitting after a stop");

        c.forget();
        assert!(!c.is_sitting(), "a reset forgets the seat");
        assert_eq!(c.start_ground_pick(), Ok(()), "and moves are allowed again");

        c.stop_moves();
        assert!(
            !c.busy(),
            "a stop ends the pick rather than resuming it later"
        );
    }
}
