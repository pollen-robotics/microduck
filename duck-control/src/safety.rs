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

pub struct Safety<T: RobotIo> {
    io: T,
    config: SafetyConfig,
    /// How long gravity has been past the threshold. Reset by any upright sample.
    falling_for: Duration,
    fallen: bool,
    /// Tracks the last gain written so an unchanged one is not rewritten every tick — that
    /// would be fifteen bus writes per tick for no reason.
    gain: Option<u16>,
}

impl<T: RobotIo> Safety<T> {
    pub fn new(io: T, config: SafetyConfig) -> Self {
        Self {
            io,
            config,
            falling_for: Duration::ZERO,
            fallen: false,
            gain: None,
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

    /// Reboot these servos, and forget the gain cache: a rebooted servo comes back at its EEPROM
    /// gains, and a cache that still says the running gain is written would leave it there. The
    /// next [`Self::apply`] rewrites the gains on every servo.
    pub fn reboot_motors(&mut self, ids: &[u8]) -> Result<(), IoError> {
        for &id in ids {
            tracing::warn!(id, "rebooting servo");
            self.io.reboot(id)?;
        }
        self.gain = None;
        Ok(())
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
    }

    /// Apply the deadman to a command.
    ///
    /// Zeroes the *twist* only. Head targets are left alone deliberately: a stale head pose
    /// is harmless, while a stale velocity walks the robot into a wall.
    pub fn gate(&self, command: Command, intent_age: Duration) -> (Command, Option<Limit>) {
        if intent_age <= self.config.deadman {
            return (command, None);
        }
        let mut stopped = command;
        stopped.twist = [0.0; 3];
        (stopped, Some(Limit::Deadman))
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
            applied.limits.push(Limit::NotFinite);
            self.io.write(&JointTargets::new(hold))?;
            return Ok(applied);
        }

        let mut safe = targets;
        for value in safe.iter_mut() {
            let clamped = value.clamp(ACTUATOR_MIN, ACTUATOR_MAX);
            if clamped != *value {
                if !applied.limited_by(Limit::Range) {
                    applied.limits.push(Limit::Range);
                }
                *value = clamped;
            }
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
    /// A reboot forgets the gain cache, so the rebooted servos get their gains back on the next
    /// apply instead of running at whatever their EEPROM says.
    #[test]
    fn rebooting_motors_rewrites_the_gain_on_the_next_apply() {
        use crate::io::FakeIo;
        use crate::model::DEFAULT_POSITION;
        let mut s = Safety::new(FakeIo::at(DEFAULT_POSITION), SafetyConfig::default());
        s.apply(DEFAULT_POSITION, DEFAULT_POSITION, 200).unwrap();
        assert_eq!(s.gain(), Some(200));
        s.reboot_motors(&[3, 11]).unwrap();
        assert_eq!(s.io().reboots, vec![3, 11]);
        assert_eq!(
            s.gain(),
            None,
            "the gain cache must be forgotten after a reboot"
        );
        s.apply(DEFAULT_POSITION, DEFAULT_POSITION, 200).unwrap();
        assert_eq!(s.gain(), Some(200));
    }

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
        let s = safety();
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
}
