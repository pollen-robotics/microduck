//! Phrase-mix twist for the locomoting hop-dance.
//!
//! Must match play mix in `docs/ideas/dance-gait/microduck_dance_loco_env_cfg.py`.
//! The director does not know about pads or pose — the caller passes `applies`.
//! Phrase length is 1–2 beats (sampled per phrase). Overlay drop (`applies =
//! false`) resets; the next lock is a new phrase.

use crate::beat::BeatState;

/// Play / robot mix. Train still uses 15/55/20/10; the policy tracks twist so
/// this table can be livelier without a retrain.
pub const MIX_INPLACE: f64 = 0.20;
pub const MIX_TRANSLATE: f64 = 0.28;
pub const MIX_SIDESTEP: f64 = 0.27;
pub const MIX_SPIN: f64 = 0.25;
pub const PHRASE_BEATS_MIN: u32 = 1;
pub const PHRASE_BEATS_MAX: u32 = 2;
pub const VX_MIN: f64 = -0.10;
pub const VX_MAX: f64 = 0.20;
pub const VY_ABS: f64 = 0.12;
pub const VYAW_SMALL: f64 = 0.25;
pub const VYAW_SPIN_MIN: f64 = 0.40;
pub const VYAW_SPIN_MAX: f64 = 0.80;

const ENERGY_SILENT: f32 = 1e-4;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kind {
    Inplace,
    Translate,
    Sidestep,
    Spin,
}

/// Tick-local dance twist. Seeded so tests (and two ducks) can replay.
pub struct DanceDirector {
    rng: u64,
    phrase_twist: [f64; 3],
    phrase_start: Option<u32>,
    pub(crate) phrase_len: u32,
    active: bool,
}

impl DanceDirector {
    pub fn new(seed: u64) -> Self {
        Self {
            rng: mix64(seed),
            phrase_twist: [0.0; 3],
            phrase_start: None,
            phrase_len: PHRASE_BEATS_MIN,
            active: false,
        }
    }

    pub fn tick(&mut self, beat: &BeatState, applies: bool) -> [f64; 3] {
        if !applies {
            self.reset();
            return [0.0, 0.0, 0.0];
        }
        let due = match self.phrase_start {
            None => true,
            Some(start) => beat.onset_seq.saturating_sub(start) >= self.phrase_len,
        };
        if !self.active || due {
            self.phrase_twist = sample_mix(&mut self.rng).0;
            self.phrase_len = sample_phrase_len(&mut self.rng);
            self.phrase_start = Some(beat.onset_seq);
            self.active = true;
        }
        if beat.energy <= ENERGY_SILENT {
            [0.0, 0.0, 0.0]
        } else {
            self.phrase_twist
        }
    }

    pub fn reset(&mut self) {
        self.phrase_twist = [0.0; 3];
        self.phrase_start = None;
        self.phrase_len = PHRASE_BEATS_MIN;
        self.active = false;
    }
}

fn mix64(mut z: u64) -> u64 {
    z = z.wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    let out = z ^ (z >> 31);
    if out == 0 {
        1
    } else {
        out
    }
}

fn next_u64(state: &mut u64) -> u64 {
    let mut x = *state;
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    *state = x;
    x
}

fn uniform01(state: &mut u64) -> f64 {
    (next_u64(state) >> 11) as f64 / ((1u64 << 53) as f64)
}

fn uniform(state: &mut u64, lo: f64, hi: f64) -> f64 {
    lo + (hi - lo) * uniform01(state)
}

fn sample_phrase_len(rng: &mut u64) -> u32 {
    let span = u64::from(PHRASE_BEATS_MAX - PHRASE_BEATS_MIN + 1);
    PHRASE_BEATS_MIN + (next_u64(rng) % span) as u32
}

fn sample_mix(rng: &mut u64) -> ([f64; 3], Kind) {
    let u = uniform01(rng);
    if u < MIX_INPLACE {
        ([0.0, 0.0, 0.0], Kind::Inplace)
    } else if u < MIX_INPLACE + MIX_TRANSLATE {
        (
            [
                uniform(rng, VX_MIN, VX_MAX),
                uniform(rng, -VY_ABS, VY_ABS),
                uniform(rng, -VYAW_SMALL, VYAW_SMALL),
            ],
            Kind::Translate,
        )
    } else if u < MIX_INPLACE + MIX_TRANSLATE + MIX_SIDESTEP {
        (
            [
                0.0,
                uniform(rng, -VY_ABS, VY_ABS),
                uniform(rng, -VYAW_SMALL, VYAW_SMALL),
            ],
            Kind::Sidestep,
        )
    } else {
        let mag = uniform(rng, VYAW_SPIN_MIN, VYAW_SPIN_MAX);
        let yaw = if uniform01(rng) < 0.5 { -mag } else { mag };
        ([0.0, 0.0, yaw], Kind::Spin)
    }
}

#[cfg(test)]
fn in_command_box(twist: [f64; 3], kind: Kind) -> bool {
    let [vx, vy, yaw] = twist;
    let xy_ok = (-VY_ABS - 1e-12..=VY_ABS + 1e-12).contains(&vy)
        && (VX_MIN - 1e-12..=VX_MAX + 1e-12).contains(&vx);
    match kind {
        Kind::Inplace => vx == 0.0 && vy == 0.0 && yaw == 0.0,
        Kind::Translate => xy_ok && yaw.abs() <= VYAW_SMALL + 1e-12,
        Kind::Sidestep => vx == 0.0 && xy_ok && yaw.abs() <= VYAW_SMALL + 1e-12,
        Kind::Spin => {
            vx == 0.0
                && vy == 0.0
                && yaw.abs() >= VYAW_SPIN_MIN - 1e-12
                && yaw.abs() <= VYAW_SPIN_MAX + 1e-12
        }
    }
}

#[cfg(test)]
fn live_beat(onset_seq: u32, energy: f32) -> BeatState {
    BeatState {
        t: u64::from(onset_seq) + 1,
        locked: true,
        bpm: 100.0,
        phase: 0.0,
        onset_seq,
        energy,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_seed_replays() {
        let mut a = DanceDirector::new(7);
        let mut b = DanceDirector::new(7);
        for k in 0..32u32 {
            let beat = live_beat(k, 1.0);
            assert_eq!(a.tick(&beat, true), b.tick(&beat, true));
        }
    }

    #[test]
    fn ten_k_samples_match_the_mix_table_and_stay_in_box() {
        let mut rng = mix64(1);
        let mut counts = [0u32; 4];
        for _ in 0..10_000 {
            let (twist, kind) = sample_mix(&mut rng);
            assert!(in_command_box(twist, kind), "{twist:?} {kind:?}");
            counts[kind as usize] += 1;
        }
        let expected = [
            MIX_INPLACE,
            MIX_TRANSLATE,
            MIX_SIDESTEP,
            MIX_SPIN,
        ];
        for (got, exp) in counts.iter().zip(expected) {
            let p = f64::from(*got) / 10_000.0;
            assert!(
                (p - exp).abs() < 0.02,
                "mix {counts:?} proportion {p} vs {exp}"
            );
        }
    }

    #[test]
    fn applies_false_zeros_and_later_true_starts_a_fresh_phrase() {
        let beat = live_beat(0, 1.0);
        let mut d = DanceDirector::new(3);
        let first = d.tick(&beat, true);
        assert_eq!(d.tick(&beat, false), [0.0, 0.0, 0.0]);
        let after = d.tick(&beat, true);

        let mut twin = DanceDirector::new(3);
        assert_eq!(twin.tick(&beat, true), first);
        let second = twin.tick(&live_beat(twin.phrase_len, 1.0), true);
        assert_eq!(after, second, "drop must sample again, not hold the old twist");
    }

    #[test]
    fn resamples_when_phrase_len_beats_elapse_not_on_repeats() {
        let mut d = DanceDirector::new(11);
        let first = d.tick(&live_beat(0, 1.0), true);
        let len = d.phrase_len;
        assert!((PHRASE_BEATS_MIN..=PHRASE_BEATS_MAX).contains(&len));
        for onset in 0..len {
            for _ in 0..8 {
                assert_eq!(d.tick(&live_beat(onset, 1.0), true), first);
            }
        }
        let next = d.tick(&live_beat(len, 1.0), true);
        let mut twin = DanceDirector::new(11);
        twin.tick(&live_beat(0, 1.0), true);
        assert_eq!(twin.tick(&live_beat(len, 1.0), true), next);
        assert_eq!(d.tick(&live_beat(len, 1.0), true), next);
    }

    #[test]
    fn first_lock_tick_resamples_immediately() {
        let mut d = DanceDirector::new(13);
        let mut twin = DanceDirector::new(13);
        let got = d.tick(&live_beat(8, 1.0), true);
        let expect = twin.tick(&live_beat(0, 1.0), true);
        assert_eq!(got, expect);
        assert_eq!(d.tick(&live_beat(8, 1.0), true), got);
    }

    #[test]
    fn energy_zero_snaps_twist_without_waiting_for_a_phrase_boundary() {
        let mut d = DanceDirector::new(17);
        let moving = d.tick(&live_beat(0, 1.0), true);
        assert_eq!(d.tick(&live_beat(0, 0.0), true), [0.0, 0.0, 0.0]);
        assert_eq!(
            d.tick(&live_beat(0, 1.0), true),
            moving,
            "same phrase must resume after silence"
        );
    }

    #[test]
    fn phrase_len_stays_in_play_range() {
        let mut rng = mix64(9);
        for _ in 0..200 {
            let n = sample_phrase_len(&mut rng);
            assert!((PHRASE_BEATS_MIN..=PHRASE_BEATS_MAX).contains(&n), "{n}");
        }
    }
}
