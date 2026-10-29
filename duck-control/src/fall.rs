//! Seeing a fall *start*, rather than confirming one that already finished.
//!
//! [`crate::safety::Safety`] has a fall verdict already, and it is the right one for what
//! it does: projected gravity past `fall_gravity_z` (about 60° from upright) held for
//! `fall_debounce` (200 ms). By the time that latches the robot is on the floor, or a few
//! tens of milliseconds from it. That is fine for "refuse to enable a robot lying on its
//! side" and useless for "drop the gains before it lands" — the whole value of limping is
//! spent in the window this predictor exists to find.
//!
//! So this is a second, deliberately separate detector. It answers a different question —
//! *is the robot on its way down right now* — and it answers it early enough to act on,
//! which means it answers on a rate rather than on a position.
//!
//! # How
//!
//! Projected gravity in the trunk frame rotates with the trunk, so its derivative is
//! exactly `ġ = −ω × g` for body-frame angular velocity `ω`. Only the z component matters
//! (that is what the fall threshold reads), which is one multiply-subtract:
//!
//! ```text
//! ġz = −(ωx·gy − ωy·gx)
//! ```
//!
//! No filtering, no differentiation of the quaternion: the gyro is a direct measurement
//! arriving in the same 12-byte block, and differentiating the SFLP quaternion instead
//! would add the filter's own lag to the very number whose whole point is to be early.
//!
//! Extrapolate that linearly over [`FallPredictor`]'s lookahead and the test is "where
//! will gravity be a quarter-second from now" — trigger when *that* is past the point of
//! no return. Three conditions, all required:
//!
//!  - **Already tilted** past `tilt_z`. A gyro spike on an upright robot is a footfall, a
//!    shove that it will absorb, or somebody picking it up. None of those is a fall, and
//!    a predictor with no position gate calls all three one.
//!  - **Still tipping over** (`ġz > 0`) — going *down*, not recovering from a lean.
//!  - **Predicted past `predicted_z`** at the lookahead horizon.
//!
//! and then debounced for a few ticks, because one noisy sample must not cost the robot
//! its gains mid-stride.
//!
//! # Armed only from standing
//!
//! A fall is something that happens to a robot that was up. So the predictor only fires once
//! it has seen the robot inside the tilt gate for `arm`, and disarms when it fires or when the
//! caller resets it. Without that, a robot lying on its front at about 58° — which is what the
//! limp-fall pose leaves after a fall forwards, a hair inside `fall_gravity_z`, so not
//! `fallen` — sagging slowly as the policy takes it is tilted, tipping and predicted past the
//! threshold: a fall, by every other test. It limped, posed, handed back and limped again, a
//! loop that only broke when the fall verdict happened to latch (seen on a robot, 2026-10-27).
//!
//! # The tuning is the whole feature
//!
//! Too early and the robot limps out of leans it would have walked off — every false
//! positive is a fall the robot *caused*, which is strictly worse than the fall being
//! dampened. Too late and it lands stiff and the mode has bought nothing. The defaults sit
//! deliberately on the late side of that: `tilt_z = -0.90` is about 26° from upright,
//! which ordinary walking does not reach, and 60 ms of debounce is three ticks at 50 Hz —
//! longer than any footfall impulse, short enough to leave most of the fall to limp
//! through.

use std::time::Duration;

use crate::imu::ImuData;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FallPredictorConfig {
    /// Projected-gravity z the robot must *already* be past before a prediction counts.
    /// Upright is -1.0; -0.90 is about 26° of tilt.
    pub tilt_z: f64,
    /// Where the extrapolation has to reach to count as a fall in progress. The same sense
    /// as `SafetyConfig::fall_gravity_z`, and by default the same number: "it will be
    /// what safety calls fallen, in `lookahead`".
    pub predicted_z: f64,
    /// How far ahead to extrapolate.
    pub lookahead: Duration,
    /// How long the verdict must hold before it fires.
    pub debounce: Duration,
    /// How long the robot must be inside `tilt_z` — standing — before a fall can fire.
    pub arm: Duration,
}

impl Default for FallPredictorConfig {
    fn default() -> Self {
        Self {
            tilt_z: -0.90,
            predicted_z: -0.5,
            lookahead: Duration::from_millis(300),
            debounce: Duration::from_millis(60),
            arm: Duration::from_millis(200),
        }
    }
}

/// Watches the IMU for a fall that has started but not landed.
///
/// Edge-triggered: [`Self::observe`] returns `true` on the tick the debounce completes and
/// then stays `false` until the robot has stood again for `arm`. The caller owns what to do
/// about it and how long to keep doing it — this only reports the moment.
#[derive(Debug, Clone)]
pub struct FallPredictor {
    config: FallPredictorConfig,
    /// How long the three conditions have held together. Any tick that fails one resets it.
    falling_for: Duration,
    /// How long the robot has been inside the tilt gate, uninterrupted.
    upright_for: Duration,
    /// Whether a fall can fire: the robot has stood for `arm` since the last trigger or
    /// reset. Firing disarms, so a caller polling every tick gets one edge per fall.
    armed: bool,
}

impl FallPredictor {
    pub fn new(config: FallPredictorConfig) -> Self {
        Self {
            config,
            falling_for: Duration::ZERO,
            upright_for: Duration::ZERO,
            armed: false,
        }
    }

    /// Rate of change of projected-gravity z, per second, from `ġ = −ω × g`.
    ///
    /// Positive means the trunk is tipping *away* from upright — gravity z climbing from
    /// -1 towards 0. Public because it is the number to look at when tuning the
    /// thresholds against a recording, and it is not otherwise recoverable from outside.
    pub fn gravity_z_rate(imu: &ImuData) -> f64 {
        -(imu.gyro[0] * imu.gravity[1] - imu.gyro[1] * imu.gravity[0])
    }

    /// Where gravity z is heading, `lookahead` from now, at the current rate.
    pub fn predicted_z(&self, imu: &ImuData) -> f64 {
        imu.gravity[2] + Self::gravity_z_rate(imu) * self.config.lookahead.as_secs_f64()
    }

    /// One tick. `true` exactly once per fall, when the debounce completes.
    pub fn observe(&mut self, imu: &ImuData, dt: Duration) -> bool {
        if imu.gravity[2] <= self.config.tilt_z {
            self.upright_for = self.upright_for.saturating_add(dt);
            if self.upright_for >= self.config.arm {
                self.armed = true;
            }
        } else {
            self.upright_for = Duration::ZERO;
        }

        let rate = Self::gravity_z_rate(imu);
        let going_down = imu.gravity[2] > self.config.tilt_z
            && rate > 0.0
            && self.predicted_z(imu) > self.config.predicted_z;

        if !going_down || !self.armed {
            self.falling_for = Duration::ZERO;
            return false;
        }

        self.falling_for = self.falling_for.saturating_add(dt);
        if self.falling_for >= self.config.debounce {
            self.armed = false;
            self.falling_for = Duration::ZERO;
            return true;
        }
        false
    }

    /// Forget the accumulated verdict — the caller has taken charge (it is limping, or the
    /// policy stopped driving), so the next fall must be detected from scratch, from a robot
    /// seen standing again.
    pub fn reset(&mut self) {
        self.falling_for = Duration::ZERO;
        self.upright_for = Duration::ZERO;
        self.armed = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: Duration = Duration::from_millis(20);

    /// Gravity for a trunk pitched `tilt` radians forward, and a pitch rate about y.
    fn tipping(tilt: f64, pitch_rate: f64) -> ImuData {
        ImuData {
            gyro: [0.0, pitch_rate, 0.0],
            gravity: [tilt.sin(), 0.0, -tilt.cos()],
            ..ImuData::default()
        }
    }

    /// A predictor that has watched the robot stand for longer than `arm` — the state every
    /// fall starts from.
    fn standing() -> FallPredictor {
        let mut p = FallPredictor::new(FallPredictorConfig::default());
        for _ in 0..10 {
            assert!(!p.observe(&tipping(0.0, 0.0), DT));
        }
        p
    }

    /// The derivative has to be the real one, or every threshold above it is guesswork.
    /// Pitching forward at 1 rad/s from 30° must raise gravity z at sin(30°) = 0.5 per
    /// second — the analytic value, not something close to it.
    #[test]
    fn the_rate_is_the_analytic_derivative() {
        let imu = tipping(std::f64::consts::FRAC_PI_6, 1.0);
        assert!((FallPredictor::gravity_z_rate(&imu) - 0.5).abs() < 1e-12);
    }

    /// Tipping the other way reads negative — recovering from a lean must never look like
    /// a fall, whatever the magnitude of the rate.
    #[test]
    fn recovering_from_a_lean_never_fires() {
        let mut p = standing();
        // Well past the tilt gate, rotating back towards upright, fast.
        for _ in 0..50 {
            assert!(!p.observe(&tipping(0.9, -4.0), DT));
        }
    }

    /// An upright robot taking a hard footfall: a big gyro transient with no tilt behind
    /// it. This is the false positive that matters — it happens every step.
    #[test]
    fn a_footfall_on_an_upright_robot_never_fires() {
        let mut p = standing();
        for _ in 0..10 {
            // 8° of lean, 3 rad/s of impulse. The tilt gate is what refuses this.
            assert!(!p.observe(&tipping(0.14, 3.0), DT));
        }
    }

    /// A real fall: tilted past the gate and rotating over at a rate that puts it on the
    /// floor. It must fire, and it must fire *once*.
    #[test]
    fn a_fall_fires_once_after_the_debounce() {
        let mut p = standing();
        let falling = tipping(0.6, 3.0);
        // 34° of tilt clears the -0.90 gate; the extrapolation clears -0.5.
        assert!(falling.gravity[2] > -0.90);
        assert!(p.predicted_z(&falling) > -0.5);

        // Debounce is 60 ms — three ticks. The first two must stay quiet.
        assert!(!p.observe(&falling, DT));
        assert!(!p.observe(&falling, DT));
        assert!(p.observe(&falling, DT), "the debounce completes on tick 3");
        for _ in 0..20 {
            assert!(!p.observe(&falling, DT), "one edge per fall, not a stream");
        }
    }

    /// A tilt that is not going anywhere — the robot held at an angle by hand — is not a
    /// fall however far it is tipped. Position alone must not trigger this; that verdict
    /// belongs to `Safety`, on its own thresholds.
    #[test]
    fn a_static_tilt_is_not_a_fall() {
        let mut p = standing();
        for _ in 0..50 {
            assert!(!p.observe(&tipping(1.2, 0.0), DT));
        }
    }

    /// A slow topple that has not committed yet — tilted, tipping, but slowly enough that
    /// the standing policy has a quarter-second to catch it. Waiting is the right answer:
    /// limping here would *cause* the fall it is predicting.
    #[test]
    fn a_slow_lean_waits() {
        let mut p = standing();
        // 26° and 0.3 rad/s: predicted z is about -0.86, nowhere near the -0.5 floor.
        let leaning = tipping(0.46, 0.3);
        assert!(leaning.gravity[2] > -0.90, "past the tilt gate");
        for _ in 0..50 {
            assert!(!p.observe(&leaning, DT));
        }
    }

    /// The trigger re-arms only once the robot has stood again for `arm`, so a second fall
    /// is caught but the tail of the first one is not re-reported.
    #[test]
    fn it_rearms_once_the_robot_has_stood_again() {
        let mut p = standing();
        let falling = tipping(0.6, 3.0);
        for _ in 0..3 {
            p.observe(&falling, DT);
        }
        assert!(!p.observe(&falling, DT), "already fired");
        // One upright tick is a swing through vertical, not standing: still disarmed.
        assert!(!p.observe(&tipping(0.0, 0.0), DT));
        for _ in 0..5 {
            assert!(
                !p.observe(&falling, DT),
                "not re-armed by a single upright tick"
            );
        }
        // Standing for `arm`: a fresh fall fires again, after its own debounce.
        for _ in 0..10 {
            assert!(!p.observe(&tipping(0.0, 0.0), DT));
        }
        assert!(!p.observe(&falling, DT));
        assert!(!p.observe(&falling, DT));
        assert!(p.observe(&falling, DT));
    }

    /// `reset` drops a debounce in progress and disarms: the caller took charge, so the next
    /// fall is detected from scratch, from a robot seen standing again.
    #[test]
    fn reset_drops_a_debounce_in_progress_and_disarms() {
        let mut p = standing();
        let falling = tipping(0.6, 3.0);
        p.observe(&falling, DT);
        p.observe(&falling, DT);
        p.reset();
        for _ in 0..10 {
            assert!(!p.observe(&falling, DT), "disarmed until the robot stands");
        }
        for _ in 0..10 {
            p.observe(&tipping(0.0, 0.0), DT);
        }
        assert!(!p.observe(&falling, DT), "the count restarted");
        assert!(!p.observe(&falling, DT));
        assert!(p.observe(&falling, DT));
    }

    /// The loop seen on a robot (2026-10-27): after a fall forwards the limp-fall pose leaves
    /// it lying on its front at gravity z ≈ -0.52 — not `fallen` — and handed back, it sags
    /// at +0.2..0.4 /s. Tilted, tipping, predicted past -0.5: a fall by every other test, so
    /// it limped, posed and limped again. A robot that has not stood since must not fire.
    #[test]
    fn a_robot_that_has_not_stood_never_fires() {
        let mut p = FallPredictor::new(FallPredictorConfig::default());
        let sagging = ImuData {
            gyro: [0.0, 0.35, 0.0],
            gravity: [0.854, 0.0, -0.52],
            ..ImuData::default()
        };
        assert!(sagging.gravity[2] > -0.90, "past the tilt gate");
        assert!(FallPredictor::gravity_z_rate(&sagging) > 0.0, "tipping");
        assert!(
            p.predicted_z(&sagging) > -0.5,
            "predicted past the fall threshold"
        );
        for _ in 0..100 {
            assert!(!p.observe(&sagging, DT));
        }
        // Same after the hand-back's reset, with the robot never having stood.
        p.reset();
        for _ in 0..100 {
            assert!(!p.observe(&sagging, DT));
        }
    }

    /// Arming needs `arm` of *uninterrupted* standing: a rocking stand-up that passes through
    /// vertical for a tick at a time does not arm it.
    #[test]
    fn brief_upright_ticks_do_not_arm() {
        let mut p = FallPredictor::new(FallPredictorConfig::default());
        let falling = tipping(0.6, 3.0);
        for _ in 0..20 {
            p.observe(&tipping(0.0, 0.0), DT);
            p.observe(&tipping(0.6, -1.0), DT);
        }
        for _ in 0..10 {
            assert!(!p.observe(&falling, DT));
        }
    }
}
