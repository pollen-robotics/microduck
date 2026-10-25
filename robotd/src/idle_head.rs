//! The robot looks around when nothing else is happening.
//!
//! A robot standing perfectly still reads as switched off, and a head sweeping on a sinusoid reads
//! as a machine. This is closer to an animal: after `[idle] after_s` with no motion asked for and
//! nobody steering the head, it
//!
//! - **glances**: every second or few it picks somewhere to look — mostly left or right, a little
//!   up or down, sometimes back to the middle, never right where it already was — moves there
//!   on an ease-in-ease-out curve, and holds the look;
//! - **keeps looking** while it holds: the gaze shifts a few degrees around the spot every half
//!   second or so, the way eyes re-fixate on something they are looking at;
//! - **leans its neck** a little forward or back with each new look, so the whole head shifts
//!   rather than only the face turning, and the beak opens a touch or closes with it;
//! - **tilts its head** now and then on a glance, the curious sideways tilt;
//! - **never quite freezes**: a slow, smooth drift of a degree or two rides on every axis, and
//!   the body breathes — a millimetre of height on a few-second cycle;
//! - **calms down**: right after it stops it is lively, and the longer nothing happens the slower
//!   and smaller its glances get, as if dozing off, and now and then it **yawns** — the beak
//!   opens slowly and the head tips back. Silent. Any motion wakes it.
//!
//! All of it is added on top of whatever the head and body were last asked to do, the way the
//! chorale's sway is, and fades in and out over `fade_s`, so it never jumps. Every number is
//! `[idle]` in `robotd.toml` ([`IdleParams`]).
//!
//! A piece of the autonomous behaviour stack's LookAround/Chill states
//! (`docs/ideas/autonomous_behavior.md`), not the stack itself: nothing here looks *at* anything.

use std::f64::consts::TAU;
use std::time::{Duration, Instant};

use robotd_params::IdleParams;

/// What the idle behaviour adds this tick.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct IdleOffset {
    /// Neck pitch, head pitch, head yaw, head roll — the head command's order.
    pub head: [f64; 4],
    /// Body height, metres. Only meaningful to a network trained on the body pose; the caller
    /// decides whether to apply it.
    pub body_z: f64,
    /// Mouth opening, 0..1 — the yawn. The caller decides who else has the mouth.
    pub mouth: f64,
}

/// One move and the pause after it: from where, to where, when it started, how long the move
/// takes, until when it is held. Yaw, pitch, roll, neck, and the beak's opening.
#[derive(Debug, Clone, Copy)]
struct Glance {
    from: [f64; 5],
    to: [f64; 5],
    start: f64,
    travel: f64,
    until: f64,
}

/// Where it has chosen to look, and until when — the gaze shifts around `anchor` until then.
#[derive(Debug, Clone, Copy)]
struct Look {
    anchor: [f64; 5],
    until: f64,
}

/// The idle behaviour's state, ticked once per control tick.
#[derive(Debug)]
pub struct IdleHead {
    tuning: IdleParams,
    rng: Rng,
    /// Since when nothing has asked the robot to move. `None` while something does.
    still_since: Option<Instant>,
    /// How much of the behaviour is applied, 0..1, faded towards 1 while idle and 0 otherwise.
    gain: f64,
    /// When this idle spell's clock started; `None` once it has faded out.
    started: Option<Instant>,
    /// The look being held, in seconds on the spell's clock.
    look: Option<Look>,
    /// The move in progress, in seconds on the spell's clock.
    glance: Option<Glance>,
    /// When the yawn in progress started, on the spell's clock.
    yawn: Option<f64>,
    /// Random phases for the drift and the breath, so two robots side by side do not move in step.
    phases: [f64; 4],
}

impl IdleHead {
    /// A new idle behaviour tuned by `tuning`, drawing from `seed`.
    pub fn new(tuning: IdleParams, seed: u64) -> Self {
        let mut rng = Rng::new(seed);
        let phases = [0; 4].map(|_| rng.unit() * TAU);
        Self {
            tuning,
            rng,
            still_since: None,
            gain: 0.0,
            started: None,
            look: None,
            glance: None,
            yawn: None,
            phases,
        }
    }

    /// How long the robot has to be still before this starts.
    ///
    /// Validation only promises finite and nonnegative, and past ~1.8e19 s `from_secs_f64`
    /// panics — on the control thread, the first time the robot stands still. Longer than a
    /// `Duration` holds is never.
    pub fn after(&self) -> Duration {
        Duration::try_from_secs_f64(self.tuning.after_s).unwrap_or(Duration::MAX)
    }

    /// One tick. `still` says nothing is asking for motion right now — the caller decides what
    /// counts.
    pub fn tick(&mut self, now: Instant, still: bool, dt: Duration) -> IdleOffset {
        let p = self.tuning;
        self.still_since = if still && p.enabled {
            Some(self.still_since.unwrap_or(now))
        } else {
            None
        };
        let idle = self
            .still_since
            .is_some_and(|since| now.duration_since(since) >= self.after());

        let step = (dt.as_secs_f64() / p.fade_s).clamp(0.0, 1.0);
        self.gain = if idle {
            (self.gain + step).min(1.0)
        } else {
            (self.gain - step).max(0.0)
        };

        if idle && self.started.is_none() {
            self.started = Some(now);
            tracing::info!("idle look-around started");
        }
        let Some(started) = self.started else {
            return IdleOffset::default();
        };
        if self.gain == 0.0 {
            // Faded out: the next spell starts afresh, at the middle, wide awake.
            let spell_s = now.duration_since(started).as_secs_f64();
            tracing::info!(
                spell_s = format_args!("{spell_s:.1}"),
                "idle look-around stopped"
            );
            self.started = None;
            self.look = None;
            self.glance = None;
            self.yawn = None;
            return IdleOffset::default();
        }

        let t = now.duration_since(started).as_secs_f64();
        let energy = p.doze_floor + (1.0 - p.doze_floor) * (-t / p.doze_tau_s).exp();

        // A yawn now and then, once it has been idle a while: a coin per tick, so they come
        // about `yawn_every_s` apart without a rhythm to them.
        if self.yawn.is_some_and(|start| t - start >= p.yawn_s) {
            self.yawn = None;
        }
        if idle
            && self.yawn.is_none()
            && p.yawn_every_s > 0.0
            && t >= p.yawn_after_s
            && self.rng.unit() < dt.as_secs_f64() / p.yawn_every_s
        {
            self.yawn = Some(t);
        }
        let yawn = self
            .yawn
            .map_or(0.0, |start| yawn_shape((t - start) / p.yawn_s));

        // Moves only while idle, and not mid-yawn: fading out, the current one is simply let go
        // of; yawning, it is held.
        if idle && self.yawn.is_none() && self.glance.is_none_or(|g| t >= g.until) {
            let from = self.glance.map_or([0.0; 5], |g| g.to);
            self.glance = Some(self.next_glance(from, t, energy));
        }
        let [yaw, pitch, roll, neck, beak] = self.glance.map_or([0.0; 5], |g| {
            let s = ease((t - g.start) / g.travel);
            [0, 1, 2, 3, 4].map(|i| g.from[i] + (g.to[i] - g.from[i]) * s)
        });

        // The drift: two incommensurate slow waves per axis, so it never visibly repeats.
        let drift = |phase: f64, f: f64| {
            p.drift
                * 0.5
                * ((TAU * f * t + phase).sin() + (TAU * f * 1.618 * t + 2.0 * phase).sin())
        };
        let breath = p.breath_m * (TAU * t / p.breath_period_s + self.phases[3]).sin();

        let g = self.gain;
        IdleOffset {
            head: [
                g * neck,
                g * (pitch + p.yawn_pitch * yawn + drift(self.phases[0], p.drift_hz * 1.2)),
                g * (yaw + drift(self.phases[1], p.drift_hz * 0.8)),
                g * (roll + drift(self.phases[2], p.drift_hz)),
            ],
            body_z: g * breath,
            mouth: g * (beak + p.yawn_open * yawn).min(1.0),
        }
    }

    /// The next move, from `from`, at spell time `t`: a small shift around the look being held,
    /// or, once that look is over, a glance somewhere new.
    fn next_glance(&mut self, from: [f64; 5], t: f64, energy: f64) -> Glance {
        let p = self.tuning;
        let holding = self.look.filter(|look| t < look.until);
        let to = match holding {
            Some(look) => [
                look.anchor[0] + p.nudge_yaw * energy * self.rng.between(-1.0, 1.0),
                look.anchor[1] + p.nudge_pitch * energy * self.rng.between(-1.0, 1.0),
                look.anchor[2],
                look.anchor[3],
                look.anchor[4],
            ],
            None => self.somewhere_new(energy),
        };
        // Angles only: the beak is an opening, not radians travelled.
        let distance = (0..4)
            .map(|i| (to[i] - from[i]).powi(2))
            .sum::<f64>()
            .sqrt();
        // Dozing: slower moves, longer holds, longer gaps.
        let travel = (p.move_min_s + p.move_s_per_rad * distance) / energy.sqrt();
        let look_until = match holding {
            Some(look) => look.until,
            None => {
                let hold = self.rng.between(p.hold_min_s, p.hold_max_s) / energy;
                let until = t + travel + hold;
                self.look = Some(Look { anchor: to, until });
                until
            }
        };
        let gap = self.rng.between(p.nudge_every_min_s, p.nudge_every_max_s) / energy;
        Glance {
            from,
            to,
            start: t,
            travel,
            // A move always finishes, and the look always ends when it said it would.
            until: (t + travel + gap).min(look_until).max(t + travel),
        }
    }

    /// A new place to look, at least `glance_min` from the last one. Drawn again when it lands
    /// too close; after a few misses the furthest draw is taken rather than looping.
    fn somewhere_new(&mut self, energy: f64) -> [f64; 5] {
        let last = self.look.map_or([0.0; 5], |look| look.anchor);
        let away = |to: [f64; 5]| (to[0] - last[0]).hypot(to[1] - last[1]);
        let mut best = self.draw(energy);
        for _ in 0..8 {
            if away(best) >= self.tuning.glance_min {
                break;
            }
            let next = self.draw(energy);
            if away(next) > away(best) {
                best = next;
            }
        }
        best
    }

    /// Anywhere to look: the middle, or somewhere within reach.
    fn draw(&mut self, energy: f64) -> [f64; 5] {
        let p = self.tuning;
        if self.rng.unit() < p.centre_chance {
            return [0.0; 5];
        }
        let reach = 0.5 + 0.5 * energy;
        let tilt = if self.rng.unit() < p.tilt_chance {
            p.tilt_roll * self.rng.sign()
        } else {
            0.0
        };
        [
            p.glance_yaw * reach * self.rng.between(-1.0, 1.0),
            p.glance_pitch * reach * self.rng.between(-1.0, 1.0),
            tilt,
            p.glance_neck * reach * self.rng.between(-1.0, 1.0),
            // Not scaled by `reach`, and never less than half: the first seventh of the mouth’s
            // travel only eases off how hard it is held shut, so a smaller opening is no
            // opening at all.
            p.glance_mouth * self.rng.between(0.5, 1.0),
        ]
    }
}

/// Minimum-jerk ease from 0 to 1: still at both ends, quickest in the middle — how a head moves
/// to look at something, rather than how a motor ramps.
fn ease(x: f64) -> f64 {
    let x = x.clamp(0.0, 1.0);
    x * x * x * (10.0 + x * (-15.0 + 6.0 * x))
}

/// A yawn's beak opening over its length, 0..1: slow open, a hold at the top, a quicker close.
fn yawn_shape(x: f64) -> f64 {
    match x {
        x if x < 0.4 => ease(x / 0.4),
        x if x < 0.7 => 1.0,
        x => 1.0 - ease((x - 0.7) / 0.3),
    }
}

/// A small xorshift generator: the glances want variety, not cryptography, and a dependency for
/// this would be heavier than the thing it draws for.
#[derive(Debug)]
struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        // Never zero, which is xorshift's one fixed point.
        Self(seed.max(1))
    }

    fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }

    /// Uniform in [0, 1).
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }

    fn between(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }

    fn sign(&mut self) -> f64 {
        if self.unit() < 0.5 { -1.0 } else { 1.0 }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: Duration = Duration::from_millis(20);

    /// Run `seconds` of ticks from `from`, returning the time reached and every offset.
    fn run(
        idle: &mut IdleHead,
        from: Instant,
        seconds: f64,
        still: bool,
    ) -> (Instant, Vec<IdleOffset>) {
        let mut now = from;
        let mut out = Vec::new();
        for _ in 0..(seconds / DT.as_secs_f64()).round() as usize {
            now += DT;
            out.push(idle.tick(now, still, DT));
        }
        (now, out)
    }

    /// `after_s = 1e20` passes validation, and asking how long that is panicked the control
    /// thread. It is a wait that never ends.
    #[test]
    fn a_delay_past_what_a_duration_holds_is_never() {
        let p = IdleParams {
            after_s: 1e20,
            ..IdleParams::default()
        };
        assert_eq!(IdleHead::new(p, 1).after(), Duration::MAX);
    }

    /// Nothing before the idle delay; after it, it looks around within its bounds, in yaw and
    /// pitch and sometimes roll, with the neck leaning under it.
    #[test]
    fn it_looks_around_only_after_the_robot_has_been_still() {
        let p = IdleParams::default();
        let mut idle = IdleHead::new(IdleParams::default(), 7);
        let t0 = Instant::now();
        let (now, early) = run(&mut idle, t0, p.after_s - 0.1, true);
        assert!(early.iter().all(|o| *o == IdleOffset::default()), "not yet");

        let (_, later) = run(&mut idle, now, 60.0, true);
        let peak = |i: usize| later.iter().map(|o| o.head[i].abs()).fold(0.0, f64::max);
        assert!(peak(0) > 0.05, "the neck leans too: {}", peak(0));
        assert!(peak(0) <= p.glance_neck + 1e-9);
        let beak = later.iter().map(|o| o.mouth).fold(0.0, f64::max);
        assert!(beak > 0.02, "the beak moves: {beak}");
        assert!(peak(2) > 0.2, "it really glances sideways: {}", peak(2));
        assert!(peak(2) <= p.glance_yaw + p.nudge_yaw + p.drift + 1e-9);
        assert!(peak(1) <= p.glance_pitch + p.nudge_pitch + p.drift + 1e-9);
        assert!(peak(3) <= p.tilt_roll + p.drift + 1e-9);
        assert!(later.iter().any(|o| o.body_z != 0.0), "it breathes");
    }

    /// **No jumps**: from one tick to the next the head moves a little, whatever it is doing —
    /// fading in, mid-glance, fading out.
    #[test]
    fn it_never_jumps() {
        let mut idle = IdleHead::new(IdleParams::default(), 42);
        let t0 = Instant::now();
        let (now, a) = run(&mut idle, t0, 40.0, true);
        let (_, b) = run(&mut idle, now, 3.0, false);
        let all: Vec<_> = a.into_iter().chain(b).collect();
        for pair in all.windows(2) {
            for i in 0..4 {
                let step = (pair[1].head[i] - pair[0].head[i]).abs();
                assert!(step < 0.03, "a {step} rad step on axis {i}");
            }
            let step = (pair[1].mouth - pair[0].mouth).abs();
            assert!(step < 0.05, "a {step} mouth step");
        }
    }

    /// Asked to move, it fades back out completely, and the idle delay starts over.
    #[test]
    fn motion_fades_it_out_and_restarts_the_wait() {
        let p = IdleParams::default();
        let mut idle = IdleHead::new(IdleParams::default(), 3);
        let t0 = Instant::now();
        let (now, _) = run(&mut idle, t0, 12.0, true);
        let (now, faded) = run(&mut idle, now, p.fade_s + 0.1, false);
        assert_eq!(*faded.last().unwrap(), IdleOffset::default(), "gone");

        let (_, again) = run(&mut idle, now, p.after_s - 0.2, true);
        assert!(
            again.iter().all(|o| *o == IdleOffset::default()),
            "the wait starts over"
        );
    }

    /// The longer it is left alone, the calmer it gets: fewer glances per minute late than early.
    #[test]
    fn it_dozes_off() {
        let p = IdleParams::default();
        let mut idle = IdleHead::new(IdleParams::default(), 11);
        let t0 = Instant::now();
        let (now, _) = run(&mut idle, t0, p.after_s, true);
        let glances = |idle: &mut IdleHead, from: Instant| {
            let mut count = 0;
            let mut last = None;
            let mut now = from;
            for _ in 0..(60.0 / DT.as_secs_f64()) as usize {
                now += DT;
                idle.tick(now, true, DT);
                let current = idle.glance.map(|g| g.start);
                if current != last {
                    count += 1;
                    last = current;
                }
            }
            (now, count)
        };
        let (now, early) = glances(&mut idle, now);
        let (now, _) = run(&mut idle, now, 240.0, true);
        let (_, late) = glances(&mut idle, now);
        assert!(late < early, "{late} glances late against {early} early");
    }

    /// Holding a look is not standing still: the gaze keeps shifting around it, several times
    /// per look, and stays near it.
    #[test]
    fn a_held_look_keeps_moving() {
        let p = IdleParams::default();
        let mut idle = IdleHead::new(p, 5);
        let t0 = Instant::now();
        let (mut now, _) = run(&mut idle, t0, p.after_s + 0.1, true);
        let (mut moves, mut looks) = (0, 0);
        let (mut last_move, mut last_look) = (None, None);
        for _ in 0..(30.0 / DT.as_secs_f64()) as usize {
            now += DT;
            idle.tick(now, true, DT);
            let (glance, look) = (idle.glance.unwrap(), idle.look.unwrap());
            if last_move != Some(glance.start) {
                moves += 1;
                last_move = Some(glance.start);
                assert!(
                    (glance.to[0] - look.anchor[0]).abs() <= p.nudge_yaw + 1e-9,
                    "a shift stays near the look"
                );
            }
            if last_look != Some(look.until) {
                looks += 1;
                last_look = Some(look.until);
            }
        }
        assert!(moves >= 2 * looks, "{moves} moves over {looks} looks");
    }

    /// A new look is never right where the last one was.
    #[test]
    fn a_new_look_goes_somewhere() {
        let p = IdleParams::default();
        let mut idle = IdleHead::new(p, 13);
        let mut now = Instant::now();
        let mut anchors: Vec<[f64; 5]> = Vec::new();
        for _ in 0..(300.0 / DT.as_secs_f64()) as usize {
            now += DT;
            idle.tick(now, true, DT);
            if let Some(look) = idle.look
                && anchors.last() != Some(&look.anchor)
            {
                anchors.push(look.anchor);
            }
        }
        assert!(anchors.len() > 20, "{} looks", anchors.len());
        for pair in anchors.windows(2) {
            let away = (pair[1][0] - pair[0][0]).hypot(pair[1][1] - pair[0][1]);
            assert!(away >= p.glance_min, "a look {away} rad from the last");
        }
    }

    /// No yawn early on; once it has been idle a while it yawns, opening the mouth and tipping
    /// the head, and closes it again.
    #[test]
    fn it_yawns_once_it_has_been_idle_a_while() {
        // The glances' own beak movement off, so the mouth is the yawn's alone.
        let p = IdleParams {
            glance_mouth: 0.0,
            ..IdleParams::default()
        };
        let mut idle = IdleHead::new(p, 17);
        let t0 = Instant::now();
        let (now, early) = run(&mut idle, t0, p.after_s + p.yawn_after_s - 1.0, true);
        assert!(early.iter().all(|o| o.mouth == 0.0), "not yet");
        let (_, later) = run(&mut idle, now, 10.0 * p.yawn_every_s, true);
        let peak = later.iter().map(|o| o.mouth).fold(0.0, f64::max);
        assert!(peak > 0.9 * p.yawn_open && peak <= p.yawn_open, "{peak}");
        assert!(
            later.iter().any(|o| o.mouth == 0.0 && o.head != [0.0; 4]),
            "it closes"
        );

        let never = IdleParams {
            yawn_every_s: 0.0,
            ..p
        };
        let mut idle = IdleHead::new(never, 17);
        let (_, all) = run(&mut idle, t0, 600.0, true);
        assert!(all.iter().all(|o| o.mouth == 0.0), "0 is never");
    }

    /// Off is off, however still the robot is.
    #[test]
    fn disabled_it_does_nothing() {
        let p = IdleParams {
            enabled: false,
            ..IdleParams::default()
        };
        let mut idle = IdleHead::new(p, 9);
        let (_, all) = run(&mut idle, Instant::now(), 30.0, true);
        assert!(all.iter().all(|o| *o == IdleOffset::default()));
    }

    #[test]
    fn a_yawn_starts_and_ends_closed() {
        assert_eq!(yawn_shape(0.0), 0.0);
        assert_eq!(yawn_shape(0.5), 1.0);
        assert_eq!(yawn_shape(1.0), 0.0);
    }

    #[test]
    fn the_ease_is_still_at_both_ends() {
        assert_eq!(ease(0.0), 0.0);
        assert_eq!(ease(1.0), 1.0);
        assert!((ease(0.5) - 0.5).abs() < 1e-12);
        assert!(ease(0.01) < 0.001 && ease(0.99) > 0.999);
    }
}
