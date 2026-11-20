//! Taking an object held out to the beak: the decisions, as a pure state machine.
//!
//! Shared by `robotctl object-offer` (the bench prototype) and `autod` (the duck on its own), so
//! the gesture means the same thing in both. Inputs are time, the hand ([`crate::hand`]'s tracker
//! over the head ToF) and the jaw's measured angle; output is the mouth opening and one-shot
//! events. No sockets, no clocks of its own.
//!
//! **The gesture, as the ToF sees it.** The nearest return in front of the beak is the hand.
//!
//! 1. *Arming.* Nothing within [`Config::offer_m`] for [`Config::clear_for`]. An offer is
//!    something that *arrives*, so a wall or a table edge sitting in front of the duck never
//!    triggers: it was never clear. While the caller says it is not accepting offers, the machine
//!    stays here.
//! 2. *Noticed → Open.* Something comes within `offer_m` and stays [`Config::dwell`]: the beak
//!    opens.
//! 3. *Open → Closing.* The sensor cannot see the last few centimetres (cover-glass crosstalk
//!    below ~10 cm, see [`crate::hand::Config::near_m`]), so "the object is at the beak" is
//!    either the hand held still within [`Config::insert_m`] (range moving less than
//!    [`Config::still_m`]) for [`Config::still_for`] — a person holding the object in the beak
//!    keeps their hand in view — or the hand having come within `insert_m` and then vanished
//!    from the band for [`Config::gone_after`]. A hand that recedes past [`Config::leave_m`] was
//!    not giving anything: the beak closes and the machine re-arms. Optionally, a load on the
//!    open jaw closes it at once — [`Config::push_ma`].
//! 4. *Closing → Holding or missed.* After [`Config::settle`], the jaw stopped short of the
//!    angle it closes to on nothing (see [`Machine::set_closed`]) by
//!    [`Config::hold_margin_rad`] means something is between the pads. Fully closed is a miss —
//!    it reopens [`Config::retries`] times, then gives up.
//! 5. *Holding.* The jaw falling back to closed means the object was taken or dropped. A hand
//!    that leaves and comes back for [`Config::release_dwell`] (while accepting) gets it back,
//!    and [`Machine::release`] lets go on the caller's say-so.

use std::time::{Duration, Instant};

use crate::hand::{self, Hand};

/// Thresholds and timings. Every one of them is a flag, because a bench session is where they
/// get their real values.
#[derive(Debug, Clone, PartialEq)]
pub struct Config {
    /// A hand nearer than this is an offer, metres.
    pub offer_m: f64,
    /// A hand that came nearer than this before vanishing was putting something in the beak.
    pub insert_m: f64,
    /// A hand past this while the beak is open is leaving, metres.
    pub leave_m: f64,
    /// How long nothing must be within `offer_m` before an offer can be noticed.
    pub clear_for: Duration,
    /// How long an offer must stay before the beak opens.
    pub dwell: Duration,
    /// How long a near hand must have vanished before the beak closes on what it left.
    pub gone_after: Duration,
    /// How long a hand held still within `insert_m` closes the beak. Zero is off.
    pub still_for: Duration,
    /// How much the range may move and still count as still, metres.
    pub still_m: f64,
    /// How long a hand that vanished from farther than `insert_m` is waited for.
    pub lost_after: Duration,
    /// The beak gives up and closes after being open this long.
    pub open_timeout: Duration,
    /// After the close command, how long before the jaw angle is believed.
    pub settle: Duration,
    /// How far short of the empty-closed angle the jaw must stop to be holding, radians.
    pub hold_margin_rad: f64,
    /// Opening, 0..1 of the mouth's travel.
    pub open: f64,
    /// Jaw current above its open-and-still baseline that closes the beak at once, mA. 0 is off.
    pub push_ma: f64,
    /// Reopenings after a miss before giving up.
    pub retries: u32,
    /// How long a returning hand must stay before a held object is let go.
    pub release_dwell: Duration,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            // 30 cm fired on people walking past; an offer is close to the head.
            offer_m: 0.20,
            insert_m: 0.16,
            leave_m: 0.40,
            clear_for: Duration::from_millis(500),
            dwell: Duration::from_millis(500),
            gone_after: Duration::from_millis(300),
            still_for: Duration::from_millis(800),
            still_m: 0.02,
            lost_after: Duration::from_millis(1000),
            open_timeout: Duration::from_secs(8),
            settle: Duration::from_millis(600),
            hold_margin_rad: 4f64.to_radians(),
            open: 1.0,
            push_ma: 0.0,
            retries: 1,
            release_dwell: Duration::from_millis(1000),
        }
    }
}

/// The jaw, as `robot.state` measured it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Jaw {
    pub angle_rad: f64,
    /// `None` when the robot does not report load.
    pub current_ma: Option<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Phase {
    /// Waiting for the space in front of the beak to be clear.
    Arming { clear_since: Option<Instant> },
    /// Clear: the next arrival is an offer.
    Ready,
    /// Something arrived; is it staying?
    Noticed { since: Instant },
    /// Beak open, waiting for the object.
    Open {
        since: Instant,
        /// The hand has been within `insert_m` since the beak opened.
        came_near: bool,
        /// When the hand stopped being seen, if it has.
        lost_since: Option<Instant>,
        /// The range the hand has stayed near, and since when, while within `insert_m`.
        still: Option<(f64, Instant)>,
        /// Jaw current once the open beak settled.
        baseline_ma: Option<f64>,
        retries_left: u32,
    },
    /// Closing on whatever is there.
    Closing { since: Instant, retries_left: u32 },
    /// Something is in the beak.
    Holding {
        /// The space in front has been clear since the grab (the giving hand has gone).
        cleared: bool,
        /// When a returning hand arrived.
        hand_since: Option<Instant>,
        /// When the jaw first read as empty.
        empty_since: Option<Instant>,
    },
    /// Letting go.
    Releasing {
        since: Instant,
        gone_since: Option<Instant>,
    },
}

impl Phase {
    pub fn name(&self) -> &'static str {
        match self {
            Phase::Arming { .. } => "arming",
            Phase::Ready => "ready",
            Phase::Noticed { .. } => "noticed",
            Phase::Open { .. } => "open",
            Phase::Closing { .. } => "closing",
            Phase::Holding { .. } => "holding",
            Phase::Releasing { .. } => "releasing",
        }
    }
}

/// Something worth a line in the log and, usually, a sound.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    /// The beak opened for an offer.
    Opened,
    /// The hand left without giving anything, or the wait timed out.
    Withdrawn,
    /// Closing on an object.
    Closing,
    /// Closed on something.
    Grabbed,
    /// Closed on nothing; reopening.
    Missed,
    /// Closed on nothing and out of retries.
    GaveUp,
    /// The held object was taken or fell.
    Lost,
    /// Letting a held object go.
    Released,
}

impl Event {
    pub fn describe(self) -> &'static str {
        match self {
            Event::Opened => "an offer — beak open",
            Event::Withdrawn => "the hand left — beak closed",
            Event::Closing => "something at the beak — closing",
            Event::Grabbed => "got it",
            Event::Missed => "closed on nothing — open again",
            Event::GaveUp => "closed on nothing — giving up",
            Event::Lost => "the beak is empty again (taken or dropped)",
            Event::Released => "a hand came back — letting go",
        }
    }
}

/// What one step decided.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Step {
    /// Mouth opening to command, 0..1.
    pub mouth: f64,
    pub event: Option<Event>,
}

pub struct Machine {
    config: Config,
    /// The jaw angle with the beak closed on nothing, radians.
    closed_rad: f64,
    phase: Phase,
}

impl Machine {
    pub fn new(config: Config, closed_rad: f64) -> Self {
        Self {
            config,
            closed_rad,
            phase: Phase::Arming { clear_since: None },
        }
    }

    pub fn phase(&self) -> Phase {
        self.phase
    }

    pub fn holding(&self) -> bool {
        matches!(self.phase, Phase::Holding { .. })
    }

    /// Whether this mode has the beak: from the offer to the end of a release. Outside it the
    /// mode sends nothing, so another client (the photo tag opening the beak for its display)
    /// is not fought.
    pub fn engaged(&self) -> bool {
        matches!(
            self.phase,
            Phase::Open { .. }
                | Phase::Closing { .. }
                | Phase::Holding { .. }
                | Phase::Releasing { .. }
        )
    }

    /// Whether the idle look-around must be kept away: its yawn opens the beak, which would drop
    /// what is in it. Only then is the head steered, because steering it all the time fought
    /// every other client that moves it (the photo tag's look-up oscillated against it).
    pub fn guards_beak(&self) -> bool {
        matches!(self.phase, Phase::Closing { .. } | Phase::Holding { .. })
    }

    fn open_phase(&self, now: Instant, retries_left: u32) -> Phase {
        Phase::Open {
            since: now,
            came_near: false,
            lost_since: None,
            still: None,
            baseline_ma: None,
            retries_left,
        }
    }

    /// The jaw angle with the beak closed on nothing, radians: the reference a grab is judged
    /// against. It differs from the commanded closed angle (the servo squeezes past where the
    /// lips meet), so it is measured: `robotctl` closes the beak once at start, `autod` takes it
    /// whenever the beak has been commanded shut for a while.
    pub fn set_closed(&mut self, closed_rad: f64) {
        self.closed_rad = closed_rad;
    }

    /// Let go of what the beak holds, as if a hand had come back for it. Nothing otherwise.
    pub fn release(&mut self, now: Instant) -> Option<Event> {
        if !self.holding() {
            return None;
        }
        self.phase = Phase::Releasing {
            since: now,
            gone_since: None,
        };
        Some(Event::Released)
    }

    /// Abandon an offer under way (somebody else took the robot): back to arming, beak shut. A
    /// held object is kept — the beak stays closed on it.
    pub fn cancel(&mut self) {
        if !self.holding() {
            self.phase = Phase::Arming { clear_since: None };
        }
    }

    fn holds_something(&self, jaw: Jaw) -> bool {
        jaw.angle_rad > self.closed_rad + self.config.hold_margin_rad
    }

    /// One decision. `hand` is this frame's tracked hand (already bridged over dropouts by the
    /// tracker); `jaw` is the latest measurement, `None` if the robot is not reporting.
    ///
    /// `accepting` is whether a new offer may open the beak, and a returning hand take a held
    /// object back — the caller's judgement of whether the duck is in a state to interact (the
    /// duck on its own only takes offers standing still). An offer already under way finishes
    /// either way. Feed the real hand even while not accepting: it is what keeps a wall the duck
    /// stopped in front of from ever counting as an arrival.
    pub fn step(
        &mut self,
        now: Instant,
        hand: Option<Hand>,
        jaw: Option<Jaw>,
        accepting: bool,
    ) -> Step {
        let c = &self.config;
        let near = |d: f64| hand.is_some_and(|h| h.range_m <= d);
        let mut event = None;

        self.phase = match self.phase {
            Phase::Arming { clear_since } => {
                if near(c.offer_m) {
                    Phase::Arming { clear_since: None }
                } else {
                    let since = clear_since.unwrap_or(now);
                    if now.duration_since(since) >= c.clear_for {
                        Phase::Ready
                    } else {
                        Phase::Arming {
                            clear_since: Some(since),
                        }
                    }
                }
            }
            Phase::Ready | Phase::Noticed { .. } if !accepting => {
                Phase::Arming { clear_since: None }
            }
            Phase::Ready => {
                if near(c.offer_m) {
                    Phase::Noticed { since: now }
                } else {
                    Phase::Ready
                }
            }
            Phase::Noticed { since } => {
                if !near(c.offer_m) {
                    Phase::Ready
                } else if now.duration_since(since) >= c.dwell {
                    event = Some(Event::Opened);
                    self.open_phase(now, c.retries)
                } else {
                    Phase::Noticed { since }
                }
            }
            Phase::Open {
                since,
                mut came_near,
                lost_since,
                still,
                mut baseline_ma,
                retries_left,
            } => {
                let open_for = now.duration_since(since);
                // The baseline is taken once the beak has finished opening.
                if baseline_ma.is_none() && open_for >= c.settle {
                    baseline_ma = jaw.and_then(|j| j.current_ma);
                }
                let pushed = c.push_ma > 0.0
                    && match (baseline_ma, jaw.and_then(|j| j.current_ma)) {
                        (Some(base), Some(now_ma)) => now_ma > base + c.push_ma,
                        _ => false,
                    };
                let lost_since = match hand {
                    Some(_) => None,
                    None => Some(lost_since.unwrap_or(now)),
                };
                let lost_for = lost_since.map(|t| now.duration_since(t));
                came_near |= near(c.insert_m);
                // Held still near the beak: the range stays within `still_m` of where it
                // settled. Any bigger move restarts the clock; leaving `insert_m` clears it.
                let still = match hand.filter(|h| h.range_m <= c.insert_m) {
                    None => None,
                    Some(h) => match still {
                        Some((at, t)) if (h.range_m - at).abs() <= c.still_m => Some((at, t)),
                        _ => Some((h.range_m, now)),
                    },
                };
                let held_still = !c.still_for.is_zero()
                    && still.is_some_and(|(_, t)| now.duration_since(t) >= c.still_for);

                if pushed
                    || held_still
                    || (came_near && lost_for.is_some_and(|d| d >= c.gone_after))
                {
                    event = Some(Event::Closing);
                    Phase::Closing {
                        since: now,
                        retries_left,
                    }
                } else if hand.is_some_and(|h| h.range_m > c.leave_m)
                    || (!came_near && lost_for.is_some_and(|d| d >= c.lost_after))
                    || open_for >= c.open_timeout
                {
                    event = Some(Event::Withdrawn);
                    Phase::Arming { clear_since: None }
                } else {
                    Phase::Open {
                        since,
                        came_near,
                        lost_since,
                        still,
                        baseline_ma,
                        retries_left,
                    }
                }
            }
            Phase::Closing {
                since,
                retries_left,
            } => {
                if now.duration_since(since) < c.settle {
                    Phase::Closing {
                        since,
                        retries_left,
                    }
                } else if jaw.is_none_or(|j| self.holds_something(j)) {
                    // A robot that does not report the jaw is believed: the person watching
                    // can see whether it is holding.
                    event = Some(Event::Grabbed);
                    Phase::Holding {
                        cleared: false,
                        hand_since: None,
                        empty_since: None,
                    }
                } else if retries_left > 0 {
                    event = Some(Event::Missed);
                    self.open_phase(now, retries_left - 1)
                } else {
                    event = Some(Event::GaveUp);
                    Phase::Arming { clear_since: None }
                }
            }
            Phase::Holding {
                mut cleared,
                hand_since,
                empty_since,
            } => {
                // Half the margin, so a jaw sitting right at the threshold does not flicker
                // between holding and lost.
                let empty = jaw.is_some_and(|j| {
                    j.angle_rad <= self.closed_rad + self.config.hold_margin_rad / 2.0
                });
                let empty_since = if empty {
                    Some(empty_since.unwrap_or(now))
                } else {
                    None
                };
                let offered = accepting && near(c.offer_m);
                cleared |= !offered;
                let hand_since = if cleared && offered {
                    Some(hand_since.unwrap_or(now))
                } else {
                    None
                };
                if empty_since.is_some_and(|t| now.duration_since(t) >= c.gone_after) {
                    event = Some(Event::Lost);
                    Phase::Arming { clear_since: None }
                } else if hand_since.is_some_and(|t| now.duration_since(t) >= c.release_dwell) {
                    event = Some(Event::Released);
                    Phase::Releasing {
                        since: now,
                        gone_since: None,
                    }
                } else {
                    Phase::Holding {
                        cleared,
                        hand_since,
                        empty_since,
                    }
                }
            }
            Phase::Releasing { since, gone_since } => {
                let gone_since = if near(c.offer_m) {
                    None
                } else {
                    Some(gone_since.unwrap_or(now))
                };
                if gone_since.is_some_and(|t| now.duration_since(t) >= c.clear_for)
                    || now.duration_since(since) >= c.open_timeout
                {
                    Phase::Arming { clear_since: None }
                } else {
                    Phase::Releasing { since, gone_since }
                }
            }
        };

        let mouth = match self.phase {
            Phase::Open { .. } | Phase::Releasing { .. } => self.config.open,
            _ => 0.0,
        };
        Step { mouth, event }
    }
}

/// The hand tracker this mode uses: the theremin's, with a shorter band (an offer is close) and
/// three zones rather than two, so a single flier is not a hand.
pub fn tracker_config(far_m: f64) -> hand::Config {
    hand::Config {
        far_m,
        min_zones: 3,
        ..hand::Config::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CLOSED: f64 = -0.05;
    const MS: Duration = Duration::from_millis(1);

    fn hand_at(range_m: f64) -> Option<Hand> {
        Some(Hand {
            range_m,
            closeness: 0.5,
            zones: 10,
            held: false,
        })
    }

    fn jaw(deg_above_closed: f64) -> Option<Jaw> {
        Some(Jaw {
            angle_rad: CLOSED + deg_above_closed.to_radians(),
            current_ma: Some(20.0),
        })
    }

    /// Drives a machine through time in 20 ms ticks.
    struct Rig {
        m: Machine,
        t: Instant,
        events: Vec<Event>,
        mouth: f64,
        accepting: bool,
    }

    impl Rig {
        fn new() -> Self {
            Self {
                m: Machine::new(Config::default(), CLOSED),
                t: Instant::now(),
                events: Vec::new(),
                mouth: 0.0,
                accepting: true,
            }
        }

        fn run(&mut self, ms: u64, hand: Option<Hand>, jaw: Option<Jaw>) {
            for _ in 0..ms / 20 {
                self.t += 20 * MS;
                let step = self.m.step(self.t, hand, jaw, self.accepting);
                self.mouth = step.mouth;
                self.events.extend(step.event);
            }
        }

        /// Clear, then a hand arriving and staying: the beak is open.
        fn opened(mut self) -> Self {
            self.run(600, None, jaw(0.0));
            self.run(600, hand_at(0.18), jaw(30.0));
            assert_eq!(self.events, vec![Event::Opened]);
            assert_eq!(self.mouth, 1.0);
            self
        }

        /// From open: the hand comes close and disappears into the sensor's blind spot.
        fn inserted(mut self) -> Self {
            self.run(300, hand_at(0.12), jaw(30.0));
            self.run(400, None, jaw(30.0));
            self
        }
    }

    #[test]
    fn a_hand_arriving_and_staying_opens_the_beak() {
        Rig::new().opened();
    }

    /// The point of arming: something already in front of the duck when it starts — a wall, a
    /// box — never opens the beak, because it never arrived.
    #[test]
    fn something_that_was_always_there_never_opens_it() {
        let mut rig = Rig::new();
        rig.run(5_000, hand_at(0.20), jaw(0.0));
        assert!(rig.events.is_empty());
        assert_eq!(rig.mouth, 0.0);
    }

    #[test]
    fn a_hand_passing_by_does_not_open_it() {
        let mut rig = Rig::new();
        rig.run(600, None, jaw(0.0));
        rig.run(200, hand_at(0.18), jaw(0.0));
        rig.run(600, None, jaw(0.0));
        assert!(rig.events.is_empty());
    }

    #[test]
    fn something_put_in_the_beak_is_grabbed() {
        let mut rig = Rig::new().opened().inserted();
        assert_eq!(rig.events, vec![Event::Opened, Event::Closing]);
        assert_eq!(rig.mouth, 0.0);
        // The jaw stops 12° short of closed: an object.
        rig.run(800, None, jaw(12.0));
        assert_eq!(rig.events.last(), Some(&Event::Grabbed));
        assert!(rig.m.holding());
        assert_eq!(rig.mouth, 0.0);
    }

    /// Holding the object in the beak keeps the hand in view: holding it still is the signal.
    #[test]
    fn a_hand_held_still_at_the_beak_closes_it() {
        let mut rig = Rig::new().opened();
        rig.run(700, hand_at(0.12), jaw(30.0));
        assert_eq!(rig.events, vec![Event::Opened], "not yet");
        rig.run(200, hand_at(0.125), jaw(30.0));
        assert_eq!(rig.events.last(), Some(&Event::Closing));
    }

    #[test]
    fn a_hand_moving_at_the_beak_does_not_close_it() {
        let mut rig = Rig::new().opened();
        for i in 0..10 {
            let range = if i % 2 == 0 { 0.10 } else { 0.14 };
            rig.run(300, hand_at(range), jaw(30.0));
        }
        assert_eq!(rig.events, vec![Event::Opened]);
    }

    /// The old default fired on people walking past at 25 cm.
    #[test]
    fn something_at_25_cm_is_not_an_offer() {
        let mut rig = Rig::new();
        rig.run(600, None, jaw(0.0));
        rig.run(2_000, hand_at(0.25), jaw(0.0));
        assert!(rig.events.is_empty());
    }

    #[test]
    fn closing_on_nothing_reopens_once_then_gives_up() {
        let mut rig = Rig::new().opened().inserted();
        rig.run(800, None, jaw(0.5));
        assert_eq!(rig.events.last(), Some(&Event::Missed));
        assert_eq!(rig.mouth, 1.0);
        rig.run(300, hand_at(0.12), jaw(30.0));
        rig.run(400, None, jaw(30.0));
        rig.run(800, None, jaw(0.5));
        assert_eq!(rig.events.last(), Some(&Event::GaveUp));
        assert_eq!(rig.mouth, 0.0);
    }

    #[test]
    fn a_hand_that_backs_away_closes_the_beak() {
        let mut rig = Rig::new().opened();
        rig.run(200, hand_at(0.45), jaw(30.0));
        assert_eq!(rig.events.last(), Some(&Event::Withdrawn));
        assert_eq!(rig.mouth, 0.0);
    }

    /// Vanishing from far away is not "at the beak": that hand never came close.
    #[test]
    fn a_hand_vanishing_from_afar_is_not_a_give() {
        let mut rig = Rig::new().opened();
        rig.run(1_200, None, jaw(30.0));
        assert_eq!(rig.events.last(), Some(&Event::Withdrawn));
    }

    #[test]
    fn the_wait_times_out() {
        let mut rig = Rig::new().opened();
        rig.run(9_000, hand_at(0.25), jaw(30.0));
        assert_eq!(rig.events.last(), Some(&Event::Withdrawn));
        assert_eq!(rig.mouth, 0.0);
    }

    #[test]
    fn a_push_on_the_open_jaw_closes_it_when_enabled() {
        let mut rig = Rig::new();
        rig.m.config.push_ma = 80.0;
        let mut rig = rig.opened();
        rig.run(800, hand_at(0.20), jaw(30.0));
        let pushed = Some(Jaw {
            angle_rad: CLOSED + 30f64.to_radians(),
            current_ma: Some(150.0),
        });
        rig.run(40, hand_at(0.20), pushed);
        assert_eq!(rig.events.last(), Some(&Event::Closing));
    }

    #[test]
    fn a_held_object_taken_away_is_noticed() {
        let mut rig = Rig::new().opened().inserted();
        rig.run(800, None, jaw(12.0));
        rig.run(600, None, jaw(0.5));
        assert_eq!(rig.events.last(), Some(&Event::Lost));
    }

    /// The giving hand is still there right after the grab; it must leave and come back before
    /// it means "give it back".
    #[test]
    fn a_hand_must_leave_and_return_to_get_it_back() {
        let mut rig = Rig::new().opened().inserted();
        rig.run(800, hand_at(0.20), jaw(12.0));
        rig.run(2_000, hand_at(0.20), jaw(12.0));
        assert_eq!(rig.events.last(), Some(&Event::Grabbed), "still holding");
        rig.run(200, None, jaw(12.0));
        rig.run(1_100, hand_at(0.20), jaw(12.0));
        assert_eq!(rig.events.last(), Some(&Event::Released));
        assert_eq!(rig.mouth, 1.0);
        rig.run(700, None, jaw(30.0));
        assert_eq!(rig.mouth, 0.0);
        assert!(matches!(rig.m.phase(), Phase::Arming { .. }));
    }

    /// Waiting for an offer, the mode leaves the head and beak to whoever else has them; it
    /// steers the head only while something is in the beak.
    #[test]
    fn it_takes_the_beak_and_head_only_when_it_needs_them() {
        let mut rig = Rig::new();
        rig.run(600, None, jaw(0.0));
        assert!(!rig.m.engaged() && !rig.m.guards_beak(), "ready");
        let mut rig = rig.opened();
        assert!(rig.m.engaged() && !rig.m.guards_beak(), "open");
        rig = rig.inserted();
        rig.run(800, None, jaw(12.0));
        assert!(rig.m.holding() && rig.m.guards_beak(), "holding");
    }

    /// Not accepting (the duck is walking, napping...): nothing opens the beak, and becoming
    /// accepting with something already in front does not count it as an arrival.
    #[test]
    fn not_accepting_holds_it_at_arming() {
        let mut rig = Rig::new();
        rig.accepting = false;
        rig.run(600, None, jaw(0.0));
        rig.run(2_000, hand_at(0.15), jaw(0.0));
        assert!(rig.events.is_empty());
        rig.accepting = true;
        rig.run(2_000, hand_at(0.15), jaw(0.0));
        assert!(
            rig.events.is_empty(),
            "was there before it accepted: not an arrival"
        );
        rig.run(600, None, jaw(0.0));
        rig.run(600, hand_at(0.15), jaw(0.0));
        assert_eq!(rig.events, vec![Event::Opened]);
    }

    #[test]
    fn release_lets_go_of_a_held_object() {
        let mut rig = Rig::new().opened().inserted();
        rig.run(800, None, jaw(12.0));
        assert!(rig.m.holding());
        let t = rig.t;
        assert_eq!(rig.m.release(t), Some(Event::Released));
        rig.run(20, None, jaw(12.0));
        assert_eq!(rig.mouth, 1.0);
        rig.run(700, None, jaw(30.0));
        assert_eq!(rig.mouth, 0.0);
        let t = rig.t;
        assert_eq!(rig.m.release(t), None, "nothing held");
    }

    #[test]
    fn cancel_closes_an_open_beak_but_keeps_a_held_object() {
        let mut rig = Rig::new().opened();
        rig.m.cancel();
        rig.run(20, hand_at(0.18), jaw(30.0));
        assert_eq!(rig.mouth, 0.0);
        let mut rig = Rig::new().opened().inserted();
        rig.run(800, None, jaw(12.0));
        rig.m.cancel();
        assert!(rig.m.holding());
    }

    #[test]
    fn a_robot_not_reporting_the_jaw_is_believed() {
        let mut rig = Rig::new().opened().inserted();
        rig.run(800, None, None);
        assert_eq!(rig.events.last(), Some(&Event::Grabbed));
    }
}
