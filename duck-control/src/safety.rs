//! The safety authority.
//!
//! **It owns the only write handle to the robot.** Nothing above it — not the policy, not
//! the arbiter, not a client — holds a [`RobotIo`], so nothing above it *can* command a
//! motor. The invariant is enforced by the borrow checker rather than by a rule someone has
//! to remember when adding the eighth skill the night before a demo.
//!
//! That is the same argument the updater makes about its recovery path: code that only runs
//! once something has already gone wrong is the code most likely to be quietly broken, so
//! make the broken state unrepresentable instead.
//!
//! Two rules, unconditional:
//!
//!  - **Non-finite rejection.** A `NaN` target is not clamped, it is refused outright.
//!  - **Range clamp.** Targets are held inside the actuator's travel.
//!
//! Plus a deadman on the command itself: if intents stop arriving, the velocity goes to
//! zero. **Stop is not limp** — losing comms makes the robot *stand still*, because standing
//! is the safe state for a biped; losing balance is a different event, and it is not this
//! layer's to answer.
//!
//! **The fall verdict is a report, not a rule.** [`Safety::fallen`] is tracked every tick
//! and published, and it preempts nothing: a fallen robot keeps being driven and the
//! humans stay in charge. That is the prototype's behaviour, and it is the only one — a
//! robot that yields the moment gravity misreads a lean is a robot that keeps sitting down
//! while someone handles it.
//!
//! What to *do* about a fall is a control decision, and it lives above this layer:
//! `robotd`'s limp-fall mode drops the gain and rides the robot down, on
//! [`crate::fall::FallPredictor`] rather than on this verdict. It commands through `apply`
//! like anything else, with no exemption and no back door — which is the point of the
//! write handle living here.

use std::time::Duration;

use crate::io::{IoError, JointTargets, RobotIo, Sensors};
use crate::model::NUM_JOINTS;
use crate::obs::Command;

/// The XL330's position range: one turn, centred, from the count↔radian conversion.
///
/// This is the *actuator's* travel, not a per-joint anatomical limit — the alpha robot's
/// real joint limits live in the MJCF, which is not vendored here. So this catches a policy
/// emitting `NaN`, an absurd action scale, or a garbage tensor; it will not stop a joint
/// being driven somewhere mechanically unwise. Recorded plainly rather than dressed up,
/// because a limit that looks per-joint but is not would imply protection nobody has.
pub const ACTUATOR_MIN: f64 = -std::f64::consts::PI;
pub const ACTUATOR_MAX: f64 = std::f64::consts::PI;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SafetyConfig {
    /// Projected-gravity z above which the robot counts as falling. Upright reads about
    /// -1.0 and a robot on its side reads near 0.
    pub fall_gravity_z: f64,
    /// How long that must hold before it counts. Debounced so a hard footfall is not a
    /// fall.
    pub fall_debounce: Duration,
    /// Intent age past which the velocity command is zeroed.
    pub deadman: Duration,
    /// Gain while running.
    pub gain_running: u16,
    /// Gain to yield at rather than fight the floor. Nothing here applies it — `robotd`
    /// commands it during limp-fall — but it lives with the other safety numbers because
    /// it is one: the gain at which the robot stops pushing back.
    pub gain_limp: u16,
}

impl Default for SafetyConfig {
    fn default() -> Self {
        Self {
            // The prototype's numbers.
            fall_gravity_z: -0.5,
            fall_debounce: Duration::from_millis(200),
            deadman: Duration::from_millis(500),
            gain_running: 200,
            gain_limp: 50,
        }
    }
}

/// Why a commanded value was not applied as asked. Surfaced so a client can be told,
/// rather than watching the robot ignore it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Limit {
    /// Intents went stale; the velocity was zeroed.
    Deadman,
    /// A target was outside the actuator's travel.
    Range,
    /// A target was `NaN` or infinite.
    NotFinite,
}

/// What safety did with a tick's worth of targets.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Applied {
    pub limits: Vec<Limit>,
}

impl Applied {
    pub fn limited_by(&self, limit: Limit) -> bool {
        self.limits.contains(&limit)
    }
}

/// How often a limit that keeps firing is said again.
///
/// The first tick a limit trips always reports — a clamp that happens once is news, and
/// before this none of it was visible at all. After that, one line per fifty: at 50 Hz that
/// is one a second, which shows a fault is still going on without writing a line per tick
/// into the journal. The same shape as the bus-read warning in `robotd`, which reports the
/// first failure and then every tenth.
const LIMIT_LOG_EVERY: u64 = 50;

/// How many ticks running each limit has tripped, which is what the journal is keyed off.
///
/// Three counters rather than one because the three limits are independent events: a
/// deadman that fires for a second says nothing about whether a joint is being clamped, and
/// one shared counter would let whichever fires most starve the other two of a line.
///
/// Each is cleared by a tick that reaches that limit's check and does *not* trip it, so the
/// number reads "how long has this been going on" — which is the number worth saying, and
/// the one a reader has to be able to distinguish from "how many times has it ever
/// happened".
///
/// A tick refused as non-finite returns before the clamp is ever reached, so it leaves
/// `range` standing rather than clearing it. That is deliberate, and the alternative is
/// worse: a target alternating between `NaN` and out-of-range would end the range run on
/// every second tick, and both limits would then report on every tick they tripped — the
/// line per tick this counting exists to prevent. So the number is trips in a run, not
/// strictly consecutive ticks.
#[derive(Debug, Default, PartialEq)]
struct LimitRuns {
    deadman: u64,
    range: u64,
    not_finite: u64,
}

impl LimitRuns {
    /// Whether a run of `n` has reached a point worth writing down.
    ///
    /// Kept here rather than at each call site so the three limits cannot drift into
    /// three different ideas of "often enough".
    fn worth_logging(n: u64) -> bool {
        n == 1 || n.is_multiple_of(LIMIT_LOG_EVERY)
    }
}

pub struct Safety<T: RobotIo> {
    io: T,
    config: SafetyConfig,
    /// How long gravity has been past the threshold. Reset by any upright sample.
    falling_for: Duration,
    fallen: bool,
    /// Tracks the last gain written so an unchanged one is not rewritten every tick — that
    /// would be fifteen bus writes per tick for no reason.
    gain: Option<u16>,
    /// Trip counts, for rate-limiting what the layer says about itself.
    runs: LimitRuns,
    /// Whether an intent has ever arrived inside the deadman, which is what makes a stale
    /// one news. Until one has, the robot has no driver to have lost. See
    /// [`Self::deadman_is_news`].
    deadman_armed: bool,
}

impl<T: RobotIo> Safety<T> {
    pub fn new(io: T, config: SafetyConfig) -> Self {
        Self {
            io,
            config,
            falling_for: Duration::ZERO,
            fallen: false,
            gain: None,
            runs: LimitRuns::default(),
            deadman_armed: false,
        }
    }

    pub fn read(&mut self) -> Result<Sensors, IoError> {
        self.io.read()
    }

    /// Bus reads that are not part of the tick, passed through rather than exposing the IO.
    ///
    /// Safety owns the only `RobotIo` handle — that is what makes "nothing commands a motor
    /// except through here" a fact the borrow checker enforces rather than a convention. Slow
    /// sensors and IMU diagnostics read rather than write, so they are no threat to that, but
    /// handing out the handle to reach them would be.
    pub fn slow_sensors(&mut self) -> Result<crate::io::SlowSensors, IoError> {
        self.io.slow_sensors()
    }

    pub fn imu_stale(&self) -> crate::io::ImuStale {
        self.io.imu_stale()
    }

    pub fn imu_ready(&self) -> bool {
        self.io.imu_ready()
    }

    pub fn fallen(&self) -> bool {
        self.fallen
    }

    /// Power the joints, so the positions this writes can actually be held.
    ///
    /// Through here because [`Safety`] owns the only [`RobotIo`] handle — the same reason
    /// [`Self::slow_sensors`] is a passthrough. Unlike that one this *does* affect the robot, so it
    /// is worth being explicit about what it is and is not:
    ///
    ///  - It is called when a human enables the policy on a limp robot, once, not per tick.
    ///  - It is **not** called at startup. A `robotd` restarted by an update must leave a standing
    ///    robot standing, and nothing here changes that.
    ///  - It does not bypass anything. Torque decides whether the motors hold what
    ///    [`Self::apply`] writes; every clamp, the fall gate and the limp gain still apply to *what*
    ///    gets written. A fallen robot with torque on is still commanded `hold` at `gain_limp`.
    pub fn set_torque(&mut self, on: bool) -> Result<(), IoError> {
        tracing::warn!(on, "torque");
        self.io.set_torque(on)
    }

    /// The gain last written to the servos, or `None` before the first write. This is what
    /// the robot is running at, which is not always what the caller asked for.
    pub fn gain(&self) -> Option<u16> {
        self.gain
    }

    /// Update fall state from a fresh sample. Call every tick, before [`Self::apply`].
    ///
    /// Debounced in both directions: a robot has to be down for `fall_debounce` to count as
    /// fallen, and one upright sample clears the accumulator. Without the debounce, the
    /// impulse from a firm footfall reads as a fall.
    pub fn observe(&mut self, sensors: &Sensors, dt: Duration) {
        // An orientation filter that has not converged does not get a vote.
        //
        // The SFLP filter needs a few seconds of samples before its quaternion means
        // anything, and until then projected gravity is not `[0, 0, -1]` — it is whatever the
        // filter is mid-way through deciding, which reads as "above the fall threshold", which
        // reads as "on its side". Two hundred milliseconds of that and an upright robot on a
        // bench latches `fallen` at startup: `apply` writes `gain_limp`, the policy is refused
        // with "the robot is down; stand it up first", and it clears itself a few seconds
        // later leaving the gain behind at 50 with nothing to explain it.
        //
        // Observed on a board: a joint set to 137 by hand read back 50 after five seconds of
        // robotd, while `robotctl monitor` — sampled later, after convergence — reported `ok`.
        //
        // Holding the previous verdict is the safe default in both directions. At startup that
        // is "not fallen", which is what the robot standing on the bench actually is; and a
        // filter that stops being ready mid-run leaves a fallen robot fallen.
        if !self.io.imu_ready() {
            return;
        }

        let was_fallen = self.fallen;
        let down = sensors.imu.gravity[2] > self.config.fall_gravity_z;
        if down {
            self.falling_for = self.falling_for.saturating_add(dt);
            if self.falling_for >= self.config.fall_debounce {
                self.fallen = true;
            }
        } else {
            self.falling_for = Duration::ZERO;
            self.fallen = false;
        }

        // Not rate-limited, unlike the limits below — though a flip is less rare than the
        // debounce suggests. Latching takes `fall_debounce` of down, but one upright sample
        // clears it, so a robot held near the threshold on a noisy IMU can cycle in roughly
        // that period and report a few times a second. Still not rate-limited, because what
        // is being said here is the flip itself: a limit reports "this is still wrong",
        // where this reports "this just changed", and dropping all but one in fifty of those
        // would drop the one that says when.
        //
        // It is also the only *event* there is. `fallen` otherwise shows up in two places
        // and neither is a record of the fall: a subscribed client's frame — which on a
        // robot is usually nobody, since `robotd` only assembles one when it has a
        // receiver — and the loop summary, which samples the boolean every five minutes.
        // A fall that comes and goes inside one of those windows leaves nothing at all,
        // and one that persists has no time on it. This fires on the flip, so it has both.
        if self.fallen != was_fallen {
            tracing::warn!(
                fallen = self.fallen,
                gravity_z = sensors.imu.gravity[2],
                "the fall verdict changed"
            );
        }
    }

    /// Apply the deadman to a command.
    ///
    /// Zeroes the *twist* only. Head targets are left alone deliberately: a stale head pose
    /// is harmless, while a stale velocity walks the robot into a wall.
    ///
    /// Takes `&mut self` because how long the intents have been stale is this layer's to
    /// remember: a robot standing still because comms dropped is a state, not a moment, and
    /// the journal has to be able to say how long it has lasted without saying so 50 times
    /// a second. See [`LimitRuns`].
    pub fn gate(&mut self, command: Command, intent_age: Duration) -> (Command, Option<Limit>) {
        if intent_age <= self.config.deadman {
            self.runs.deadman = 0;
            self.deadman_armed = true;
            return (command, None);
        }

        self.runs.deadman += 1;
        if self.deadman_is_news() {
            tracing::warn!(
                intent_age_ms = intent_age.as_millis(),
                deadman_ms = self.config.deadman.as_millis(),
                tripped = self.runs.deadman,
                "intents went stale — the velocity command is zeroed"
            );
        }

        let mut stopped = command;
        stopped.twist = [0.0; 3];
        (stopped, Some(Limit::Deadman))
    }

    /// Whether a stale intent this tick is worth saying anything about.
    ///
    /// Never, on a robot that has not had a driver since it started, however long it has been
    /// idle. This runs every tick whether or not anything is driving the robot, and an intent
    /// that has never been written reads as maximally stale from the moment it boots — so
    /// without this, a robot nobody has sent a twist to trips the deadman about half a second
    /// after start and reports it once a second forever, with the count climbing without
    /// bound. That is a bench robot, and any robot driven through skills or `robotctl` rather
    /// than a pad: the common case, and eighty thousand lines a day describing it.
    ///
    /// A robot with no driver is not being ignored, and that is not news. Losing one is.
    fn deadman_is_news(&self) -> bool {
        self.deadman_armed && LimitRuns::worth_logging(self.runs.deadman)
    }

    /// The only path to the motors.
    ///
    /// `hold` is what to command when the policy must not drive — normally the pose the
    /// robot is already in.
    /// `running_gain` is what the caller wants — the standing policy runs softer than the
    /// walking one, and limp-fall softer still. All of those are control decisions, not
    /// safety ones, so they are passed in rather than second-guessed here.
    pub fn apply(
        &mut self,
        targets: [f64; NUM_JOINTS],
        hold: [f64; NUM_JOINTS],
        running_gain: u16,
    ) -> Result<Applied, IoError> {
        let mut applied = Applied::default();

        // Note what is *not* here: a fall gate. Being down does not stop the caller
        // driving, because the verdict is a report (see the module docs). What a fall is
        // worth doing about is decided above, and arrives as ordinary targets and an
        // ordinary gain.
        self.set_gain(running_gain)?;

        // Non-finite is refused, not clamped. Clamping `NaN` silently produces a boundary
        // value, which is a plausible-looking joint angle — far worse than declining to
        // move, because the robot would lurch to a limit rather than hold still.
        if targets.iter().any(|v| !v.is_finite()) {
            // Returned before the clamp below is reached, so `range` is left standing — see
            // [`LimitRuns`] for why that is the right way round.
            self.runs.not_finite += 1;
            if LimitRuns::worth_logging(self.runs.not_finite) {
                tracing::warn!(
                    tripped = self.runs.not_finite,
                    "targets refused: not finite — holding the pose the robot is already in"
                );
            }
            applied.limits.push(Limit::NotFinite);
            self.io.write(&JointTargets::new(hold))?;
            return Ok(applied);
        }
        self.runs.not_finite = 0;

        // Counted, not merely detected: one joint at its limit and nine are different
        // faults wearing the same `Limit::Range` on the wire, and the lowest index out is
        // what someone goes to look up. `first_clamped` doubles as "any were clamped",
        // which is what keeps the run counter honest on a tick that clamps nothing.
        let mut safe = targets;
        let mut clamped_joints = 0u32;
        let mut first_clamped = None;
        for (joint, value) in safe.iter_mut().enumerate() {
            let clamped = value.clamp(ACTUATOR_MIN, ACTUATOR_MAX);
            if clamped != *value {
                clamped_joints += 1;
                first_clamped.get_or_insert(joint);
                *value = clamped;
            }
        }

        if let Some(first) = first_clamped {
            self.runs.range += 1;
            applied.limits.push(Limit::Range);
            if LimitRuns::worth_logging(self.runs.range) {
                tracing::warn!(
                    joints = clamped_joints,
                    first_joint = first,
                    tripped = self.runs.range,
                    "joint targets clamped to the actuator's travel"
                );
            }
        } else {
            self.runs.range = 0;
        }

        self.io.write(&JointTargets::new(safe))?;
        Ok(applied)
    }

    fn set_gain(&mut self, kp: u16) -> Result<(), IoError> {
        if self.gain == Some(kp) {
            return Ok(());
        }
        self.io.set_gain(kp)?;
        self.gain = Some(kp);
        Ok(())
    }

    /// Borrow the wrapped IO. Test-only, and deliberately not public: handing this out in
    /// production would defeat the point of safety owning the writer.
    #[cfg(test)]
    fn io(&self) -> &T {
        &self.io
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::imu::ImuData;
    use crate::io::FakeIo;
    use crate::model::DEFAULT_POSITION;

    fn upright() -> Sensors {
        Sensors::default() // gravity defaults to [0, 0, -1]
    }

    fn on_its_side() -> Sensors {
        Sensors {
            imu: ImuData {
                gravity: [-1.0, 0.0, 0.0],
                ..ImuData::default()
            },
            ..Sensors::default()
        }
    }

    fn safety() -> Safety<FakeIo> {
        Safety::new(FakeIo::at(DEFAULT_POSITION), SafetyConfig::default())
    }

    /// A hard footfall spikes gravity briefly. Treating that as a fall would drop the robot
    /// mid-stride — which is itself how you cause a fall.
    #[test]
    fn a_brief_tilt_is_not_a_fall() {
        let mut s = safety();
        s.observe(&on_its_side(), Duration::from_millis(100));
        assert!(!s.fallen(), "100ms is under the 200ms debounce");
        s.observe(&upright(), Duration::from_millis(20));
        s.observe(&on_its_side(), Duration::from_millis(100));
        assert!(!s.fallen(), "an upright sample must reset the accumulator");
    }

    /// Sustained means fallen.
    #[test]
    fn a_sustained_tilt_is_a_fall() {
        let mut s = safety();
        for _ in 0..11 {
            s.observe(&on_its_side(), Duration::from_millis(20));
        }
        assert!(s.fallen());
    }

    /// Falling must drop the gain, not merely stop commanding. Refusing to command only
    /// freezes the robot in the pose it fell in; going soft lets it yield.
    /// **The regression.** An orientation filter that has not converged must not be able to
    /// declare a robot fallen.
    ///
    /// On a board this cost an afternoon: an upright robot latched `fallen` within 200 ms of
    /// startup, `apply` wrote `gain_limp`, `padd` was refused with "the robot is down; stand
    /// it up first", and a few seconds later it cleared itself — leaving the servos at kP 50
    /// with `robotctl monitor` reporting `ok`, because the gain is only written when it
    /// changes. Nothing in the observable state explained it.
    #[test]
    fn an_unconverged_imu_cannot_declare_a_fall() {
        let mut io = FakeIo::at(DEFAULT_POSITION);
        io.imu_ready = false;
        let mut safety = Safety::new(io, SafetyConfig::default());

        // Gravity well above the threshold — what an unconverged filter reports, and what
        // a robot on its side reports. The difference is only whether the filter is ready.
        let mut sensors = Sensors::default();
        sensors.imu.gravity = [0.0, 0.0, 0.0];

        for _ in 0..50 {
            safety.observe(&sensors, Duration::from_millis(20));
        }
        assert!(
            !safety.fallen(),
            "a robot was called fallen on an orientation filter that had not converged"
        );
    }

    /// And once it has converged, the same sample must be believed — otherwise the guard
    /// above would simply disable fall detection.
    #[test]
    fn a_converged_imu_still_detects_a_fall() {
        let mut io = FakeIo::at(DEFAULT_POSITION);
        io.imu_ready = true;
        let mut safety = Safety::new(io, SafetyConfig::default());

        let mut sensors = Sensors::default();
        sensors.imu.gravity = [0.0, 0.0, 0.0];

        for _ in 0..50 {
            safety.observe(&sensors, Duration::from_millis(20));
        }
        assert!(
            safety.fallen(),
            "a converged filter must still report a fall"
        );
    }

    /// With the gate OFF — the default, the prototype's behaviour — a fall changes
    /// nothing about what gets written: the verdict is reported, the policy keeps
    /// driving, the gain stays the caller's. Going limp under someone adjusting the
    /// robot's lean is exactly the annoyance this default removes.
    #[test]
    fn by_default_a_fall_reports_but_does_not_preempt() {
        let mut s = safety();
        for _ in 0..11 {
            s.observe(&on_its_side(), Duration::from_millis(20));
        }
        assert!(s.fallen(), "the verdict is still tracked");

        let mut wanted = DEFAULT_POSITION;
        wanted[0] = 0.9;
        let applied = s
            .apply(
                wanted,
                DEFAULT_POSITION,
                SafetyConfig::default().gain_running,
            )
            .unwrap();
        assert!(applied.limits.is_empty(), "{:?}", applied.limits);
        assert_eq!(
            s.io().last_written.unwrap().positions,
            wanted,
            "the policy keeps driving"
        );
        assert_eq!(s.io().last_gain, Some(SafetyConfig::default().gain_running));
    }

    /// **The fall verdict preempts nothing.** It is tracked, it is published, and it does
    /// not touch the motors: a fallen robot is driven exactly like an upright one, at the
    /// gain the caller asked for.
    ///
    /// This is the contract that lets limp-fall live entirely above this layer — it drops
    /// the gain by *asking*, through the same `apply` as everything else, with no exemption
    /// to special-case. A gate here would have to be bypassed for the pose ramp to move a
    /// robot lying on the floor, and a safety rule with a bypass is not one.
    #[test]
    fn a_fall_does_not_preempt_the_caller() {
        let mut s = safety();
        for _ in 0..11 {
            s.observe(&on_its_side(), Duration::from_millis(20));
        }
        assert!(s.fallen(), "the verdict is still tracked");

        let mut wanted = DEFAULT_POSITION;
        wanted[0] = 0.9;
        let applied = s
            .apply(
                wanted,
                DEFAULT_POSITION,
                SafetyConfig::default().gain_running,
            )
            .unwrap();

        assert!(applied.limits.is_empty(), "{:?}", applied.limits);
        assert_eq!(
            s.io().last_written.unwrap().positions,
            wanted,
            "the caller keeps driving a robot that is down"
        );
        assert_eq!(s.io().last_gain, Some(SafetyConfig::default().gain_running));
    }

    /// The other half: a caller that *wants* to go soft says so, and it goes straight
    /// through. This is exactly what `robotd` does during limp-fall.
    #[test]
    fn a_caller_that_asks_for_the_limp_gain_gets_it() {
        let mut s = safety();
        s.observe(&upright(), Duration::from_millis(20));
        let limp = SafetyConfig::default().gain_limp;

        s.apply(DEFAULT_POSITION, DEFAULT_POSITION, limp).unwrap();
        assert_eq!(s.io().last_gain, Some(limp));
        assert_eq!(
            s.gain(),
            Some(limp),
            "and it is reported as what is running"
        );
    }

    /// `NaN` is refused outright. Clamping it would yield a boundary value — a plausible
    /// joint angle — and the robot would lurch to a limit instead of holding still.
    #[test]
    fn a_non_finite_target_is_refused_not_clamped() {
        let mut s = safety();
        s.observe(&upright(), Duration::from_millis(20));

        let mut poisoned = DEFAULT_POSITION;
        poisoned[3] = f64::NAN;
        let applied = s
            .apply(
                poisoned,
                DEFAULT_POSITION,
                SafetyConfig::default().gain_running,
            )
            .unwrap();

        assert!(applied.limited_by(Limit::NotFinite));
        assert!(!applied.limited_by(Limit::Range), "must not be clamped");
        assert_eq!(s.io().last_written.unwrap().positions, DEFAULT_POSITION);
    }

    /// Out-of-range targets are clamped and reported. Reported matters: a client whose
    /// command was silently altered has no way to know why the robot is not doing as asked.
    #[test]
    fn out_of_range_targets_are_clamped_and_reported() {
        let mut s = safety();
        s.observe(&upright(), Duration::from_millis(20));

        let mut wild = DEFAULT_POSITION;
        wild[2] = 100.0;
        wild[7] = -100.0;
        let applied = s
            .apply(wild, DEFAULT_POSITION, SafetyConfig::default().gain_running)
            .unwrap();

        assert!(applied.limited_by(Limit::Range));
        let written = s.io().last_written.unwrap().positions;
        assert_eq!(written[2], ACTUATOR_MAX);
        assert_eq!(written[7], ACTUATOR_MIN);
    }

    /// An ordinary tick must pass through untouched, or the clamp is silently mangling
    /// normal operation and every other test here proves nothing.
    #[test]
    fn an_ordinary_target_passes_through_unchanged() {
        let mut s = safety();
        s.observe(&upright(), Duration::from_millis(20));

        let applied = s
            .apply(
                DEFAULT_POSITION,
                DEFAULT_POSITION,
                SafetyConfig::default().gain_running,
            )
            .unwrap();
        assert!(applied.limits.is_empty(), "{:?}", applied.limits);
        assert_eq!(s.io().last_written.unwrap().positions, DEFAULT_POSITION);
        assert_eq!(s.io().last_gain, Some(SafetyConfig::default().gain_running));
    }

    /// The deadman zeroes the twist and nothing else. Losing comms should make the robot
    /// stand still — not collapse, and not forget where its head was pointing.
    #[test]
    fn the_deadman_zeroes_the_twist_only() {
        let mut s = safety();
        let command = Command {
            twist: [0.5, 0.0, 0.3],
            head: [0.1, 0.2, 0.3, 0.4],
            ..Command::default()
        };

        let (fresh, limit) = s.gate(command, Duration::from_millis(100));
        assert_eq!(fresh, command, "a fresh intent passes through");
        assert!(limit.is_none());

        let (stale, limit) = s.gate(command, Duration::from_secs(5));
        assert_eq!(stale.twist, [0.0; 3], "velocity must stop");
        assert_eq!(stale.head, command.head, "head is harmless when stale");
        assert_eq!(limit, Some(Limit::Deadman));
    }

    /// The gain is written once per transition, not once per tick. At 50 Hz the naive
    /// version would be 750 extra bus writes a second, on the bus the control loop needs.
    #[test]
    fn the_gain_is_only_written_when_it_changes() {
        let mut s = safety();
        s.observe(&upright(), Duration::from_millis(20));
        s.apply(
            DEFAULT_POSITION,
            DEFAULT_POSITION,
            SafetyConfig::default().gain_running,
        )
        .unwrap();
        s.apply(
            DEFAULT_POSITION,
            DEFAULT_POSITION,
            SafetyConfig::default().gain_running,
        )
        .unwrap();
        s.apply(
            DEFAULT_POSITION,
            DEFAULT_POSITION,
            SafetyConfig::default().gain_running,
        )
        .unwrap();

        // Three applies, three position writes, but only the first gain write.
        assert_eq!(s.io().writes, 3);
        assert_eq!(s.io().last_gain, Some(SafetyConfig::default().gain_running));
        assert_eq!(s.gain, Some(SafetyConfig::default().gain_running));
    }

    fn gain() -> u16 {
        SafetyConfig::default().gain_running
    }

    /// Every joint asked to go past the actuator's travel.
    fn over_travel() -> [f64; NUM_JOINTS] {
        [ACTUATOR_MAX + 1.0; NUM_JOINTS]
    }

    /// A limit that trips once is reported; one that trips forever is reported every
    /// [`LIMIT_LOG_EVERY`] ticks and not on any tick in between.
    ///
    /// This is the failure the run counters exist to prevent: at 50 Hz a policy emitting
    /// NaN writes a line a tick, and fifty identical lines a second is what teaches
    /// everyone to stop reading the journal — the same reason the stale-IMU warning in
    /// `bus.rs` says nothing until a run is long enough to mean something.
    #[test]
    fn a_limit_that_keeps_firing_is_reported_periodically_not_per_tick() {
        assert!(LimitRuns::worth_logging(1), "the first trip is always news");
        assert!(!LimitRuns::worth_logging(2), "not the second");
        assert!(!LimitRuns::worth_logging(LIMIT_LOG_EVERY - 1));
        assert!(LimitRuns::worth_logging(LIMIT_LOG_EVERY));
        assert!(!LimitRuns::worth_logging(LIMIT_LOG_EVERY + 1));
        assert!(LimitRuns::worth_logging(LIMIT_LOG_EVERY * 3));
    }

    /// A run must measure *how long this has been going on*, not how many times it has
    /// ever happened.
    ///
    /// Otherwise the count climbs for the life of the process and `50` stops meaning
    /// anything: a clamp on Tuesday and a clamp on Friday would read the same as a
    /// second-long one now. The debounce on the fall verdict is reset the same way, and
    /// for the same reason.
    #[test]
    fn a_limit_run_resets_when_the_limit_stops_firing() {
        let mut s = safety();

        s.apply(over_travel(), DEFAULT_POSITION, gain()).unwrap();
        s.apply(over_travel(), DEFAULT_POSITION, gain()).unwrap();
        assert_eq!(s.runs.range, 2, "two clamped ticks in a row");

        s.apply(DEFAULT_POSITION, DEFAULT_POSITION, gain()).unwrap();
        assert_eq!(s.runs.range, 0, "a clean tick ends the run");

        // A deadman run clears the same way, when a fresh intent arrives again.
        s.gate(Command::default(), Duration::from_secs(5));
        s.gate(Command::default(), Duration::from_secs(5));
        assert_eq!(s.runs.deadman, 2);
        s.gate(Command::default(), Duration::from_millis(10));
        assert_eq!(s.runs.deadman, 0, "fresh intents end the run");
    }

    /// The three counts must be independent. One shared counter would let whichever limit
    /// fires most starve the other two of a line entirely — a robot whose joints are being
    /// clamped every tick would say nothing about it, because the deadman kept resetting
    /// the number that both were writing into.
    #[test]
    fn the_three_limits_are_counted_separately() {
        let mut s = safety();

        s.apply(over_travel(), DEFAULT_POSITION, gain()).unwrap();
        s.apply(over_travel(), DEFAULT_POSITION, gain()).unwrap();
        s.apply([f64::NAN; NUM_JOINTS], DEFAULT_POSITION, gain())
            .unwrap();
        s.gate(Command::default(), Duration::from_secs(5));
        s.gate(Command::default(), Duration::from_secs(5));
        s.gate(Command::default(), Duration::from_secs(5));

        assert_eq!(s.runs.range, 2, "clamped twice, then refused as NaN");
        assert_eq!(s.runs.not_finite, 1, "the NaN tick is its own run");
        assert_eq!(s.runs.deadman, 3);
    }

    /// An ordinary tick trips nothing and reports nothing — the common case, and the one
    /// that has to stay silent for any of this to be worth reading.
    #[test]
    fn a_plain_tick_trips_no_limit() {
        let mut s = safety();
        s.observe(&upright(), Duration::from_millis(20));

        let applied = s.apply(DEFAULT_POSITION, DEFAULT_POSITION, gain()).unwrap();
        let (command, limit) = s.gate(Command::default(), Duration::from_millis(10));

        assert!(applied.limits.is_empty(), "{:?}", applied.limits);
        assert!(limit.is_none());
        assert_eq!(s.runs, LimitRuns::default());
        assert_eq!(command, Command::default());
        assert!(!s.fallen());
    }

    /// Refusing non-finite targets must still leave the robot where it was, and clamping
    /// must still land every joint inside the actuator's travel.
    ///
    /// The run counters and the journal lines were added to a path that already worked;
    /// this is what says they did not change what it decides. Both are the difference
    /// between a robot that holds still and one that lurches to a limit.
    #[test]
    fn limits_are_reported_without_changing_what_is_applied() {
        let mut s = safety();

        // Refused: the write is the hold pose, not the NaN.
        s.apply([f64::NAN; NUM_JOINTS], DEFAULT_POSITION, gain())
            .unwrap();
        assert_eq!(
            s.io().last_written.unwrap().positions,
            DEFAULT_POSITION,
            "a refused target must not reach the servos"
        );

        // Clamped: every joint lands on the boundary, none beyond it.
        s.apply(over_travel(), DEFAULT_POSITION, gain()).unwrap();
        let written = s.io().last_written.unwrap().positions;
        assert!(
            written.iter().all(|v| *v <= ACTUATOR_MAX),
            "nothing past the actuator's travel: {written:?}"
        );
        assert_eq!(written, [ACTUATOR_MAX; NUM_JOINTS]);
    }

    /// A robot nobody has driven since it started says nothing about the deadman, however
    /// long it has been idle.
    ///
    /// This is the failure the arming exists to prevent. `gate` runs every tick whatever the
    /// robot is doing, and an intent that was never written reads as maximally stale from
    /// boot, so an idle robot trips the deadman about half a second in and never stops: one
    /// line a second, eighty thousand a day, the count climbing without bound. That is a
    /// bench robot, and any robot driven through skills or `robotctl` instead of a pad.
    ///
    /// Losing a driver is news. Never having had one is not.
    #[test]
    fn a_robot_that_has_never_been_driven_reports_no_deadman() {
        let mut s = safety();

        for _ in 0..200 {
            assert!(!s.deadman_is_news(), "never driven: nothing to say");
            s.gate(Command::default(), Duration::from_secs(5));
        }
        assert_eq!(s.runs.deadman, 200, "counted, but not worth saying");

        // One fresh intent arms it, and from then on going quiet is news again.
        s.gate(Command::default(), Duration::from_millis(10));
        assert!(s.deadman_is_news(), "a driver that goes quiet is different");
    }

    /// A tick refused as non-finite returns before the clamp is reached, so it leaves the
    /// range run standing rather than ending it.
    ///
    /// Deliberate, and the alternative is noisier: a target alternating between `NaN` and
    /// out-of-range would end the range run on every other tick, and then both limits report
    /// on the first tick of every run — a line per tick, which is what the counting is
    /// there to prevent. See [`LimitRuns`].
    #[test]
    fn a_refused_tick_does_not_end_a_range_run() {
        let mut s = safety();

        s.apply(over_travel(), DEFAULT_POSITION, gain()).unwrap();
        assert_eq!(s.runs.range, 1);

        s.apply([f64::NAN; NUM_JOINTS], DEFAULT_POSITION, gain())
            .unwrap();
        assert_eq!(s.runs.range, 1, "the refused tick leaves the run standing");
        assert_eq!(s.runs.not_finite, 1, "and is its own run");
    }
}
