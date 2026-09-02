//! Scripted head expressions on the bumpers: little timed head-intent sequences the policy
//! plays through the command block, so the sticks keep driving underneath.
//!
//! LB = *curious*: a slow head tilt one way, then the other, with a small dip of the neck. The
//! "what is this thing?" look. RB = *peck*: the head goes forward twice — neck and head pitched
//! down together, held long enough for the network to follow (the alpha policies track a head
//! step in roughly half a second; shorter pulses barely move it), then back up.
//!
//! An expression is a pure function of the time since it started, so it needs no state beyond
//! that instant, and it ends on its own. The amplitudes are the tracked ranges measured in
//! simulation on the shipped policies: head_pitch and head_yaw follow ±1 rad one to one, the neck
//! only follows downward (about half the command), head_roll follows ±0.27 rad.

use std::time::{Duration, Instant};

use duck_ipc_proto as proto;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    Curious,
    Peck,
    /// Startled: the head snaps up and the duck backs away a step. The scream is on the trigger.
    Startled,
}

#[derive(Debug, Clone, Copy)]
pub struct Expression {
    kind: Kind,
    started: Instant,
}

/// One peck: down for this long, then back up for the rest of `PECK_PERIOD`.
const PECK_DOWN: f64 = 0.45;
const PECK_PERIOD: f64 = 0.85;
const PECK_COUNT: u32 = 2;
/// The peck's amplitudes: neck command (tracks to about -0.85 rad) and the head-pitch counter-tilt.
const PECK_NECK: f64 = 1.5;
const PECK_HEAD: f64 = 0.6;
/// Curious: tilt right, tilt left, centre. Each leg this long.
const CURIOUS_LEG: f64 = 0.7;
/// Startled: head up at once, then back away from `STARTLED_BACK_FROM` to `STARTLED_BACK_TO`
/// (the calmest backward walk command the alpha gait has), head settling by the end.
const STARTLED_LEN: f64 = 2.0;
const STARTLED_BACK_FROM: f64 = 0.3;
const STARTLED_BACK_TO: f64 = 1.5;
const STARTLED_BACK: proto::MoveParams = proto::MoveParams {
    vx: -0.35,
    vy: 0.0,
    vyaw: 0.22,
};

impl Expression {
    pub fn start(kind: Kind, now: Instant) -> Self {
        Self { kind, started: now }
    }

    pub fn duration(kind: Kind) -> Duration {
        Duration::from_secs_f64(match kind {
            Kind::Peck => PECK_PERIOD * PECK_COUNT as f64,
            Kind::Curious => CURIOUS_LEG * 3.0,
            Kind::Startled => STARTLED_LEN,
        })
    }

    /// The twist this expression wants at `now`: `None` when it leaves the sticks in charge.
    pub fn twist_at(&self, now: Instant) -> Option<proto::MoveParams> {
        let t = now.duration_since(self.started).as_secs_f64();
        twist_at(self.kind, t)
    }

    /// The head intent at `now`, or `None` once the expression is over.
    pub fn head_at(&self, now: Instant) -> Option<proto::HeadParams> {
        let t = now.duration_since(self.started).as_secs_f64();
        if t >= Self::duration(self.kind).as_secs_f64() {
            return None;
        }
        Some(head_at(self.kind, t))
    }
}

/// Smooth 0→1 ramp over `len` seconds (half a cosine): no snap for the servos to chase.
fn ramp(t: f64, len: f64) -> f64 {
    let x = (t / len).clamp(0.0, 1.0);
    0.5 - 0.5 * (std::f64::consts::PI * x).cos()
}

pub fn twist_at(kind: Kind, t: f64) -> Option<proto::MoveParams> {
    match kind {
        Kind::Startled if (STARTLED_BACK_FROM..STARTLED_BACK_TO).contains(&t) => {
            Some(STARTLED_BACK)
        }
        _ => None,
    }
}

pub fn head_at(kind: Kind, t: f64) -> proto::HeadParams {
    match kind {
        Kind::Startled => {
            // Head pitch negative is beak up (measured on the shipped policies). Up fast, back
            // to level over the last half second.
            let up = ramp(t, 0.15) * (1.0 - ramp(t - (STARTLED_LEN - 0.5), 0.5));
            proto::HeadParams {
                neck_pitch: 0.0,
                head_pitch: -up,
                head_yaw: 0.0,
                head_roll: 0.0,
            }
        }
        Kind::Peck => {
            let phase = t % PECK_PERIOD;
            // Forward = the neck pitched down (it tracks only about half, so it is asked for
            // more than its range) while the head pitches the other way to keep the beak level:
            // a chicken's jab, about 3 cm of travel on this neck. Head pitch negative = beak up,
            // measured in simulation; the two together read as "the head went forward", not
            // "the beak dropped".
            let down = if phase < PECK_DOWN {
                ramp(phase, 0.12)
            } else {
                1.0 - ramp(phase - PECK_DOWN, 0.15)
            };
            proto::HeadParams {
                neck_pitch: -PECK_NECK * down,
                head_pitch: -PECK_HEAD * down,
                head_yaw: 0.0,
                head_roll: 0.0,
            }
        }
        Kind::Curious => {
            // Tilt one way, then the other, then back — with a small dip of the neck that reads
            // as "looking at it". Rolls are within the joint's ±0.27 rad.
            let roll = if t < CURIOUS_LEG {
                0.27 * ramp(t, 0.3)
            } else if t < 2.0 * CURIOUS_LEG {
                0.27 - 0.54 * ramp(t - CURIOUS_LEG, 0.35)
            } else {
                -0.27 + 0.27 * ramp(t - 2.0 * CURIOUS_LEG, 0.3)
            };
            let dip = ramp(t, 0.4) * (1.0 - ramp(t - 2.0 * CURIOUS_LEG, 0.4));
            proto::HeadParams {
                neck_pitch: -0.5 * dip,
                head_pitch: 0.0,
                head_yaw: 0.0,
                head_roll: roll,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_peck_goes_forward_twice_and_comes_back() {
        let downs: Vec<f64> = (0..(PECK_PERIOD * PECK_COUNT as f64 * 100.0) as usize)
            .map(|i| -head_at(Kind::Peck, i as f64 / 100.0).neck_pitch / PECK_NECK)
            .collect();
        // two separate excursions past 0.9, and back near zero between them and at the end
        let mut excursions = 0;
        let mut inside = false;
        for &d in &downs {
            if d > 0.9 && !inside {
                excursions += 1;
                inside = true;
            } else if d < 0.1 {
                inside = false;
            }
        }
        assert_eq!(excursions, 2, "two pecks");
        assert!(
            downs.last().unwrap().abs() < 0.05,
            "ends with the head back up"
        );
        assert!(downs.iter().all(|d| (0.0..=1.0).contains(d)));
    }

    #[test]
    fn curious_tilts_both_ways_within_the_roll_range_and_ends_centred() {
        let rolls: Vec<f64> = (0..(CURIOUS_LEG * 3.0 * 100.0) as usize)
            .map(|i| head_at(Kind::Curious, i as f64 / 100.0).head_roll)
            .collect();
        assert!(rolls.iter().cloned().fold(f64::MIN, f64::max) > 0.25);
        assert!(rolls.iter().cloned().fold(f64::MAX, f64::min) < -0.25);
        assert!(rolls.iter().all(|r| r.abs() <= 0.27 + 1e-9));
        assert!(rolls.last().unwrap().abs() < 0.02);
        let e = Expression::start(Kind::Curious, Instant::now());
        assert!(
            e.head_at(e.started + Expression::duration(Kind::Curious))
                .is_none()
        );
        assert!(e.head_at(e.started).is_some());
    }

    #[test]
    fn startled_looks_up_then_backs_away_then_settles() {
        assert!(
            head_at(Kind::Startled, 0.2).head_pitch < -0.9,
            "head up at once"
        );
        assert_eq!(
            twist_at(Kind::Startled, 0.1),
            None,
            "no walking before the head is up"
        );
        assert_eq!(twist_at(Kind::Startled, 1.0), Some(STARTLED_BACK));
        assert_eq!(twist_at(Kind::Startled, 1.8), None);
        assert!(
            head_at(Kind::Startled, STARTLED_LEN - 0.01)
                .head_pitch
                .abs()
                < 0.05,
            "level at the end"
        );
        assert_eq!(twist_at(Kind::Peck, 0.5), None);
        assert_eq!(twist_at(Kind::Curious, 0.5), None);
    }

    #[test]
    fn no_expression_touches_the_yaw_or_the_body() {
        for kind in [Kind::Peck, Kind::Curious, Kind::Startled] {
            for i in 0..300 {
                let h = head_at(kind, i as f64 / 100.0);
                assert_eq!(h.head_yaw, 0.0);
                assert!(h.neck_pitch <= 0.0, "the neck only tracks downward");
            }
        }
    }
}
