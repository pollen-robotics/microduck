//! A room of ducks on a bad radio, in one process, deterministically.
//!
//! The chorale's failures on the floor were never in one duck's logic: each duck did something
//! locally reasonable with what it had heard, and what it had heard differed. So the test is a
//! room, not a unit — several [`Chorale`]s driven through the same tick loop `robotd` runs, with a
//! radio between them that is as bad as BLE is:
//!
//!  - **Discovery is slow and per pair.** A duck takes up to `discovery` to be noticed by each other
//!    duck, independently, timed from when both are on. One pair finding each other early while
//!    another is still deaf is the asymmetry that splits a flock; a single shared delay cannot.
//!  - **Deliveries are lost**, at `loss`, independently per scan.
//!  - **A payload change reaches the air late**, somewhere in the advertising-slot window, which is
//!    the jitter the beat lock is built to average out.
//!  - **Some links are one-way or dead**, for as long as the scenario says.
//!  - **Ducks are switched off and on** in the middle of things, as `robotctl chorale` on a bench is.
//!
//! Everything comes from a seed, so a failing scenario is a number that reproduces it. `duck-ether`
//! does the same thing across containers and real daemons; this does it a thousand times a second.
//!
//! What is checked is what a person in the room would notice: two songs at once, a duck sitting
//! out while the others sing, ducks audibly off the beat, two ducks on one line.

use std::time::{Duration, Instant};

use duck_ipc_proto as proto;

use super::Chorale;

/// The simulation's step. Deliveries land on it; the control loop ticks every other one, at 50 Hz
/// like `robotd`'s.
const STEP_MS: u64 = 10;

/// A splitmix64 stream — short, well distributed, no dependency, and the same on every machine.
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self(seed ^ 0x9E37_79B9_7F4A_7C15)
    }

    pub fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut x = self.0;
        x = (x ^ (x >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        x = (x ^ (x >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        x ^ (x >> 31)
    }

    /// Uniform in `[0, 1)`.
    pub fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Uniform in `[low, high]`.
    pub fn range(&mut self, low: u64, high: u64) -> u64 {
        if high <= low {
            return low;
        }
        low + self.next() % (high - low + 1)
    }

    pub fn chance(&mut self, p: f64) -> bool {
        self.unit() < p
    }
}

/// How bad the radio is.
#[derive(Debug, Clone)]
pub struct Weather {
    /// Each directed pair takes up to this long to notice each other, ms.
    pub discovery_ms: u64,
    /// Fraction of scans that miss.
    pub loss: f64,
    /// Chance per step that a discovered duck's advertisement is caught at all, before loss — the
    /// scanner's duty cycle and the advertising interval, folded into one number.
    pub hit: f64,
    /// When a new payload reaches the air after it is handed to the radio, ms.
    pub air_ms: (u64, u64),
    /// Directed links that carry nothing until a time: `(from, to, until ms)`. `u64::MAX` is a link
    /// that never works.
    pub deaf: Vec<(usize, usize, u64)>,
}

impl Weather {
    /// Whether `from` cannot reach `to` at `ms`.
    pub fn deaf(&self, from: usize, to: usize, ms: u64) -> bool {
        self.deaf
            .iter()
            .any(|(f, t, until)| *f == from && *t == to && ms < *until)
    }

    /// Two groups that cannot hear each other until `until` — two halves of a room that each
    /// elect a conductor before they meet.
    pub fn islands(mut self, a: &[usize], b: &[usize], until: u64) -> Self {
        for x in a {
            for y in b {
                self.deaf.push((*x, *y, until));
                self.deaf.push((*y, *x, until));
            }
        }
        self
    }

    pub fn perfect() -> Self {
        Self {
            discovery_ms: 0,
            loss: 0.0,
            hit: 1.0,
            air_ms: (100, 150),
            deaf: Vec::new(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    /// `robotctl chorale` on this duck.
    On(usize),
    /// `robotctl chorale --off`, or Ctrl-C.
    Off(usize),
}

#[derive(Debug, Clone)]
pub struct Scenario {
    /// Each duck's pitch centre and voice seed.
    pub ducks: Vec<(f64, u32)>,
    /// What happens when, ms from the start, in order.
    pub events: Vec<(u64, Event)>,
    pub weather: Weather,
    pub duration_ms: u64,
    /// From when the room has to be right, ms.
    pub judge_from_ms: u64,
}

/// What [`Chorale::sim_view`] reports: the performance as `(conductor, round, piece)`, the part and
/// position of the last tick, and the piece's beat length and length in beats.
pub type View = (
    Option<(u16, u8, u8)>,
    Option<(sounds::chorale::Part, f64)>,
    f64,
    f64,
);

/// What one duck was doing at one instant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Seen {
    pub active: bool,
    /// The performance it is in — `(conductor, round, piece)` — whether or not it is audible yet.
    pub performance: Option<(u16, u8, u8)>,
    /// Its part and how far into the score, when it is singing.
    pub singing: Option<(sounds::chorale::Part, f64)>,
    /// Whether a note is actually due now: inside the piece, not in the count-in or the breath.
    pub audible: bool,
    /// Whether this duck holds the beat for its performance.
    pub conducting: bool,
    pub beat_s: f64,
}

/// What went wrong, counted over the judged part of a run.
#[derive(Debug, Clone, Default)]
pub struct Verdict {
    pub samples: usize,
    /// Two different performances audible in the room at once.
    pub split: usize,
    /// A duck that is on and could be singing sat out while others sang.
    pub sitting_out: usize,
    /// Two audible followers more than [`Verdict::DISCORD_S`] apart, or a follower more than
    /// [`Verdict::CONDUCTOR_S`] from its conductor.
    pub discord: usize,
    /// Two audible ducks on the same line.
    pub doubled: usize,
    /// The widest gap between two audible followers on the same performance, seconds.
    pub worst_spread_s: f64,
    /// First instant the room was right, ms after `judge_from_ms`. `None` if it never was.
    pub first_good_ms: Option<u64>,
    pub log: Vec<String>,
}

impl Verdict {
    /// Audibly off the beat — well past the ±20 ms an ensemble wants, so that this flags a broken
    /// lock rather than radio jitter.
    pub const DISCORD_S: f64 = 0.08;

    /// The same, against the conductor — looser, because how far a follower sits from its
    /// conductor is mostly how far the radio's real latency is from `beat::SLOT_MEAN_S`, a constant
    /// the conductor compensates by. That is a calibration for hardware to settle; what the
    /// protocol owns is that followers agree with *each other*, wherever they took the beat from.
    pub const CONDUCTOR_S: f64 = 0.15;

    pub fn bad(&self) -> usize {
        self.split + self.sitting_out + self.discord + self.doubled
    }
}

struct Duck {
    chorale: Chorale,
    active: bool,
    /// When this duck switched on, ms — discovery is timed from here.
    on_since: Option<u64>,
    /// What it has handed the radio: `(air time ms, beacon)`, in air order.
    air: Vec<(u64, proto::ChoraleBeacon)>,
}

impl Duck {
    /// What the radio is broadcasting for this duck now.
    fn on_air(&self, ms: u64) -> Option<&proto::ChoraleBeacon> {
        self.air
            .iter()
            .rev()
            .find(|(at, _)| *at <= ms)
            .map(|(_, beacon)| beacon)
    }
}

/// Run a scenario and judge it.
pub fn run(scenario: &Scenario, seed: u64) -> Verdict {
    let mut rng = Rng::new(seed);
    let t0 = Instant::now();
    let at = |ms: u64| t0 + Duration::from_millis(ms);
    let n = scenario.ducks.len();
    let mut ducks: Vec<Duck> = scenario
        .ducks
        .iter()
        .map(|(hz, voice)| Duck {
            chorale: Chorale::with_round(*hz, *voice, None, rng.next() as u8),
            active: false,
            on_since: None,
            air: Vec::new(),
        })
        .collect();
    // Per directed pair, how long after both are on before `to` hears `from`. Re-drawn whenever
    // either switches back on, since a scanner restarting is a new discovery.
    let mut discovery = vec![vec![0u64; n]; n];
    let mut verdict = Verdict::default();
    let mut events = scenario.events.iter().peekable();

    let mut ms = 0;
    while ms <= scenario.duration_ms {
        while let Some((when, event)) = events.peek() {
            if *when > ms {
                break;
            }
            match event {
                Event::On(i) => {
                    ducks[*i].chorale.set_active(true, at(ms), None);
                    ducks[*i].active = true;
                    ducks[*i].on_since = Some(ms);
                    // Both directions of each pair, drawn in this order so a seed stays a room.
                    #[allow(clippy::needless_range_loop)]
                    for other in 0..n {
                        discovery[*i][other] = rng.range(0, scenario.weather.discovery_ms);
                        discovery[other][*i] = rng.range(0, scenario.weather.discovery_ms);
                    }
                    verdict.log.push(format!("{ms:>7} ms  duck {i} on"));
                }
                Event::Off(i) => {
                    ducks[*i].chorale.set_active(false, at(ms), None);
                    ducks[*i].active = false;
                    ducks[*i].on_since = None;
                    verdict.log.push(format!("{ms:>7} ms  duck {i} off"));
                }
            }
            events.next();
        }

        // The radio: every discovered, live link, a scan that may or may not catch what is on air.
        for to in 0..n {
            if !ducks[to].active {
                continue;
            }
            for from in 0..n {
                if from == to || scenario.weather.deaf(from, to, ms) {
                    continue;
                }
                let (Some(a), Some(b)) = (ducks[from].on_since, ducks[to].on_since) else {
                    continue;
                };
                if ms < a.max(b) + discovery[from][to] {
                    continue;
                }
                if !rng.chance(scenario.weather.hit) || rng.chance(scenario.weather.loss) {
                    continue;
                }
                let Some(beacon) = ducks[from].on_air(ms).cloned() else {
                    continue;
                };
                let heard = proto::ChoraleHeard {
                    beacon,
                    from: format!("{:02X}:00:00:00:00:{from:02X}", rng.next() as u8),
                    age_us: 0,
                };
                ducks[to].chorale.heard(&heard, at(ms));
            }
        }

        // The control loop, at 50 Hz.
        if ms % (2 * STEP_MS) == 0 {
            for (i, duck) in ducks.iter_mut().enumerate() {
                let before = (
                    duck.chorale.sim_view().0,
                    duck.chorale.sim_view().1.is_some(),
                );
                let tick = duck.chorale.tick(at(ms));
                if let Some(advertise) = tick.advertise.clone() {
                    match advertise.beacon {
                        Some(beacon) => {
                            let (low, high) = scenario.weather.air_ms;
                            let last = duck.air.last().map_or(0, |(t, _)| *t);
                            duck.air
                                .push(((ms + rng.range(low, high)).max(last), beacon));
                            // Only the recent past can still be on the air.
                            if duck.air.len() > 64 {
                                duck.air.drain(..32);
                            }
                        }
                        None => duck.air.clear(),
                    }
                }
                let after = (duck.chorale.sim_view().0, tick.singing.is_some());
                if before != after {
                    verdict.log.push(format!(
                        "{ms:>7} ms  duck {i} now {:?} singing {:?}",
                        after.0, tick.singing
                    ));
                }
            }
        }

        if ms >= scenario.judge_from_ms && ms % 250 == 0 {
            let seen: Vec<Seen> = ducks
                .iter_mut()
                .map(|duck| {
                    let (performance, singing, beat_s, total) = duck.chorale.sim_view();
                    let audible = singing.is_some_and(|(_, beats)| beats >= 0.0 && beats < total);
                    Seen {
                        conducting: performance.is_some_and(|p| p.0 == duck.chorale.id),
                        active: duck.active,
                        performance,
                        singing,
                        audible,
                        beat_s,
                    }
                })
                .collect();
            judge(&seen, ms, scenario, &mut verdict);
        }
        ms += STEP_MS;
    }
    for (i, duck) in ducks.iter().enumerate() {
        verdict
            .log
            .insert(0, format!("final duck {i}: {}", duck.chorale.sim_debug()));
    }
    verdict
}

fn judge(seen: &[Seen], ms: u64, scenario: &Scenario, verdict: &mut Verdict) {
    verdict.samples += 1;
    let audible: Vec<(usize, &Seen)> = seen
        .iter()
        .enumerate()
        .filter(|(_, s)| s.active && s.audible)
        .collect();
    let mut bad = false;

    let mut performances: Vec<(u16, u8, u8)> =
        audible.iter().filter_map(|(_, s)| s.performance).collect();
    performances.sort_unstable();
    performances.dedup();
    if performances.len() > 1 {
        verdict.split += 1;
        bad = true;
        verdict
            .log
            .push(format!("{ms:>7} ms  SPLIT {performances:?}"));
    }

    // Sitting out: someone is singing and this duck, on and able to hear somebody, is not — and
    // the room is not full. A fifth duck listening is the design, not a failure.
    let on = seen.iter().filter(|s| s.active).count();
    if !audible.is_empty() && on <= proto::ChoraleBeacon::MAX_ROSTER {
        for (i, s) in seen.iter().enumerate() {
            // Not singing at all — a duck in its count-in or a beat past the last note, a few
            // milliseconds from the others, is not sitting out.
            if s.active && s.singing.is_none() && hears_anyone(i, seen, scenario, ms) {
                verdict.sitting_out += 1;
                bad = true;
                verdict.log.push(format!(
                    "{ms:>7} ms  duck {i} sitting out: {:?} {:?}",
                    s.performance, s.singing
                ));
            }
        }
    }

    for (x, (i, a)) in audible.iter().enumerate() {
        for (j, b) in audible.iter().skip(x + 1) {
            if a.performance != b.performance {
                continue;
            }
            let (Some((pa, ba)), Some((pb, bb))) = (a.singing, b.singing) else {
                continue;
            };
            let spread = ((ba - bb) * a.beat_s).abs();
            if !a.conducting && !b.conducting {
                verdict.worst_spread_s = verdict.worst_spread_s.max(spread);
            }
            let limit = if a.conducting || b.conducting {
                Verdict::CONDUCTOR_S
            } else {
                Verdict::DISCORD_S
            };
            if spread > limit {
                verdict.discord += 1;
                bad = true;
                verdict.log.push(format!(
                    "{ms:>7} ms  ducks {i},{j} {:.0} ms apart in {:?} at {ba:.2}/{bb:.2}",
                    spread * 1e3,
                    a.performance
                ));
            }
            if pa == pb && audible.len() <= proto::ChoraleBeacon::MAX_ROSTER {
                verdict.doubled += 1;
                bad = true;
                verdict
                    .log
                    .push(format!("{ms:>7} ms  ducks {i},{j} both on {pa:?}"));
            }
        }
    }
    if !bad && !audible.is_empty() && verdict.first_good_ms.is_none() {
        verdict.first_good_ms = Some(ms - scenario.judge_from_ms);
    }
}

/// Whether duck `i` has any working link in from a duck that is on — a duck nobody can reach is
/// not expected to sing.
fn hears_anyone(i: usize, seen: &[Seen], scenario: &Scenario, ms: u64) -> bool {
    (0..seen.len())
        .any(|from| from != i && seen[from].active && !scenario.weather.deaf(from, i, ms))
}

/// A random room: two to four ducks, started together or apart, a radio anywhere from perfect to
/// awful, and sometimes a one-way link, a late arrival or a restart.
pub fn random_scenario(seed: u64) -> Scenario {
    let mut rng = Rng::new(seed.wrapping_mul(0xA24B_AED4_963E_E407));
    let n = [2usize, 3, 4, 4, 4][rng.range(0, 4) as usize];
    let ducks: Vec<(f64, u32)> = (0..n)
        .map(|_| (rng.range(150, 420) as f64, rng.next() as u32))
        .collect();
    let spread = [0u64, 400, 3_000, 15_000][rng.range(0, 3) as usize];
    let mut events: Vec<(u64, Event)> = (0..n)
        .map(|i| (rng.range(0, spread), Event::On(i)))
        .collect();
    // A latecomer, arriving into a piece already going.
    if n > 2 && rng.chance(0.3) {
        let late = rng.range(0, n as u64 - 1) as usize;
        events.retain(|(_, e)| *e != Event::On(late));
        events.push((rng.range(30_000, 60_000), Event::On(late)));
    }
    // A restart: `robotctl chorale` killed and run again, conductor or not.
    if rng.chance(0.3) {
        let who = rng.range(0, n as u64 - 1) as usize;
        let off = rng.range(20_000, 70_000);
        events.push((off, Event::Off(who)));
        events.push((off + rng.range(500, 6_000), Event::On(who)));
    }
    events.sort_by_key(|(t, _)| *t);
    let discovery_ms = [0u64, 3_000, 10_000, 30_000][rng.range(0, 3) as usize];
    let mut deaf = Vec::new();
    // One dead directed link, in a room big enough that the duck can still hear somebody.
    if n > 2 && rng.chance(0.3) {
        let from = rng.range(0, n as u64 - 1) as usize;
        let to = (from + 1 + rng.range(0, n as u64 - 2) as usize) % n;
        deaf.push((from, to, u64::MAX));
    }
    let weather = Weather {
        discovery_ms,
        loss: [0.0, 0.2, 0.4, 0.6][rng.range(0, 3) as usize],
        hit: [0.25, 0.5, 1.0][rng.range(0, 2) as usize],
        air_ms: (100, 150),
        deaf,
    };
    let last = events.last().map_or(0, |(t, _)| *t);
    // Room to discover, to elect, and to merge — then judged over two minutes, which spans more
    // than one piece, so the change of piece is judged as well.
    let judge_from_ms = last + discovery_ms + 20_000;
    Scenario {
        ducks,
        events,
        weather,
        duration_ms: judge_from_ms + 120_000,
        judge_from_ms,
    }
}
