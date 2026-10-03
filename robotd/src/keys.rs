//! What the head does while the duck is being played from a keyboard.
//!
//! The voice and the beak are `sound.rs` and the loop; this is only the *body language*, kept here
//! so it can be tested as plain functions of time.
//!
//! **Singing** is the chorale's sway ([`crate::chorale::head_expression`]) on a clock of its own,
//! since a keyboard has no score to take a beat from: a slow sway, a bob, and the head lifting on the
//! high notes. "High" is relative to what the player has been playing ([`Range`]), because a duck
//! whose head only moved above C6 would be a duck that ignored most of the keyboard.
//!
//! **Between notes** it waits, and waiting is where it gets to be cute: it breathes, and every few
//! seconds it cocks its head to one side, perks up a little, and comes back — a duck listening for
//! the next note. Only while the instrument is up: when the player goes away, so does this, and a
//! duck nobody is playing does not move on its own.
//!
//! Amplitudes are the chorale's order of magnitude for the same reason it gives — the head carries
//! the ToF and the balance policy has opinions about mass that high up. The pitch sign assumes
//! negative is up, as the chorale's does.

use std::f64::consts::{PI, TAU};

/// The sway's tempo, in beats a second. A keyboard has no score to read one from, and the sway is
/// not meant to keep time with the player — 70 bpm is slow enough to read as swaying rather than
/// nodding along.
const SWAY_BEATS_PER_S: f64 = 70.0 / 60.0;

/// The narrowest span [`Range::reach`] spreads a note across, in semitones. Two notes a tone apart
/// should not send the head from the bottom of its travel to the top.
const MIN_SPAN: f64 = 12.0;

/// Breathing: a slow pitch bob, radians and seconds.
const BREATH_PITCH: f64 = 0.02;
const BREATH_PERIOD_S: f64 = 4.0;

/// The head-tilt: one every [`TILT_EVERY_S`], lasting [`TILT_FOR_S`], to a side picked per window.
const TILT_EVERY_S: f64 = 6.5;
const TILT_FOR_S: f64 = 2.4;
const TILT_ROLL: f64 = 0.14;
const TILT_YAW: f64 = 0.07;
/// How far it perks up during a tilt. Negative is up.
const TILT_PERK: f64 = -0.04;

/// The notes the player has been using, so "high" means high for *this* tune.
#[derive(Debug, Clone, Copy, Default)]
pub struct Range {
    low_high: Option<(u8, u8)>,
}

impl Range {
    /// Forget the tune: the instrument was put down.
    pub fn reset(&mut self) {
        self.low_high = None;
    }

    /// Note one being played, and say where it sits in the range so far: 0 low, 1 high.
    pub fn reach(&mut self, midi: u8) -> f64 {
        let (low, high) = match self.low_high {
            Some((low, high)) => (low.min(midi), high.max(midi)),
            None => (midi, midi),
        };
        self.low_high = Some((low, high));
        let centre = (f64::from(low) + f64::from(high)) / 2.0;
        let span = (f64::from(high) - f64::from(low)).max(MIN_SPAN);
        ((f64::from(midi) - centre) / span + 0.5).clamp(0.0, 1.0)
    }
}

/// Head offsets while a note sounds: `[neck_pitch, head_pitch, head_yaw, head_roll]`.
pub fn singing(t: f64, reach: f64) -> [f64; 4] {
    crate::chorale::head_expression(t * SWAY_BEATS_PER_S, reach)
}

/// Head offsets between notes: breathing, and now and then a curious tilt.
pub fn waiting(t: f64) -> [f64; 4] {
    let breath = BREATH_PITCH * (TAU * t / BREATH_PERIOD_S).sin();

    let window = (t / TILT_EVERY_S).floor();
    let into = t - window * TILT_EVERY_S;
    // Eased in and out — sin² over the tilt — so it starts and lands without a jerk.
    let envelope = if into < TILT_FOR_S {
        (PI * into / TILT_FOR_S).sin().powi(2)
    } else {
        0.0
    };
    let side = if side_of(window as i64) { 1.0 } else { -1.0 };

    [
        0.0,
        breath + TILT_PERK * envelope,
        side * TILT_YAW * envelope,
        side * TILT_ROLL * envelope,
    ]
}

/// Which way a window tilts. Hashed rather than alternating, because a duck that always tilts
/// left-right-left reads as a metronome; and from the window number rather than a random draw, so
/// the motion is a function of time alone and the loop holds no state for it.
fn side_of(window: i64) -> bool {
    let mut x = window as u64 ^ 0x9e37_79b9_7f4a_7c15;
    x = (x ^ (x >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    x = (x ^ (x >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    (x ^ (x >> 31)) & 1 == 1
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The head lifts with the note, relative to what is being played, and two neighbouring notes
    /// do not throw it across its whole travel.
    #[test]
    fn reach_is_relative_to_the_tune() {
        let mut range = Range::default();
        assert_eq!(
            range.reach(72),
            0.5,
            "a first note is the middle of nothing"
        );
        let a_tone_up = range.reach(74);
        assert!(a_tone_up > 0.5 && a_tone_up < 0.7, "{a_tone_up}");
        range.reach(60);
        assert!(range.reach(84) > range.reach(72));
        assert_eq!(range.reach(60), 0.0);
        range.reset();
        assert_eq!(range.reach(60), 0.5);
    }

    /// Waiting is small, smooth, and actually tilts both ways over a minute.
    #[test]
    fn waiting_stays_small_tilts_both_ways_and_never_jerks() {
        let dt = 0.02;
        let mut previous = waiting(0.0);
        let (mut left, mut right) = (false, false);
        for i in 1..3000 {
            let now = waiting(f64::from(i) * dt);
            for (axis, (a, b)) in now.iter().zip(previous).enumerate() {
                assert!(a.abs() <= 0.2, "axis {axis} at {a}");
                assert!((a - b).abs() < 0.01, "axis {axis} jumped {b} → {a}");
            }
            left |= now[3] > 0.1;
            right |= now[3] < -0.1;
            previous = now;
        }
        assert!(left && right, "a minute of waiting tilted only one way");
    }

    /// Between tilts the head is level — the tilt is an event, not a lean.
    #[test]
    fn between_tilts_only_the_breath_moves() {
        let t = TILT_FOR_S + 1.0;
        let [neck, pitch, yaw, roll] = waiting(t);
        assert_eq!([neck, yaw, roll], [0.0, 0.0, 0.0]);
        assert!(pitch.abs() <= BREATH_PITCH);
    }
}
