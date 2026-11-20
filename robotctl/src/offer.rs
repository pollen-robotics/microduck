//! `robotctl object-offer` — hold something out to the duck and it takes it in its beak.
//!
//! A prototype, run from the board's shell, built only from what robotd already serves: the head
//! ToF's depth frames (`tofd`), the jaw's measured angle and current (`robot.state`), and the
//! mouth intent (`robot.mouth`). Nothing in robotd changes for it, so it is as safe to try as
//! `robotctl theremin`.
//!
//! **The gesture, as the ToF sees it.** The nearest return in front of the beak
//! ([`kinematics::hand`], the theremin's tracker) is the hand.
//!
//! 1. *Arming.* Nothing within [`Config::offer_m`] for [`Config::clear_for`]. An offer is
//!    something that *arrives*, so a wall or a table edge sitting in front of the duck never
//!    triggers: it was never clear.
//! 2. *Noticed → Open.* Something comes within `offer_m` and stays [`Config::dwell`]: the beak
//!    opens (with a rising "inquire").
//! 3. *Open → Closing.* The sensor cannot see the last few centimetres (cover-glass crosstalk
//!    below ~10 cm, see [`kinematics::hand::Config::near_m`]), so "the object is at the beak"
//!    is either the hand having come within [`Config::insert_m`] and then vanishing from the band
//!    for [`Config::gone_after`], or the hand held still within `insert_m` (range moving less
//!    than [`Config::still_m`]) for [`Config::still_for`] — a person holding the object in the
//!    beak keeps their hand in view, so vanishing alone never fires for them. A hand that recedes past [`Config::leave_m`] instead was not
//!    giving anything: the beak closes and the duck re-arms. Optionally, a load on the open jaw
//!    (somebody pushing the object onto it) closes it at once — [`Config::push_ma`].
//! 4. *Closing → Holding or missed.* The beak closes and, after [`Config::settle`], the jaw's
//!    measured angle says what happened: stopped short of the angle it closes to on nothing
//!    (calibrated at start) by [`Config::hold_margin_rad`] means something is between the pads.
//!    Fully closed is a miss — it reopens [`Config::retries`] times, then gives up.
//! 5. *Holding.* The jaw falling back to closed means the object was taken or dropped. To give it
//!    back, take your hand away and bring it back in front of the beak for
//!    [`Config::release_dwell`]: the beak opens ("bwep") and closes once the hand has gone.
//!
//! The decisions are [`Machine::step`], a pure function of time, the tracked hand and the jaw,
//! so they are tested here without a robot; [`run`] is the plumbing around it.

use std::io::{BufRead, Write};
use std::path::Path;
use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant};

use duck_ipc_proto as proto;
use kinematics::hand::{self, Hand, Tracker};

use crate::{Client, Failure, exit};

/// Index of the mouth in `robot.state`'s joint arrays.
const MOUTH: usize = 9;

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

    pub fn sound(self) -> Option<proto::SoundTag> {
        match self {
            Event::Opened => Some(proto::SoundTag::Inquire),
            Event::Grabbed => Some(proto::SoundTag::Chirp),
            Event::GaveUp => Some(proto::SoundTag::Peck),
            Event::Released => Some(proto::SoundTag::Bwep),
            Event::Withdrawn | Event::Closing | Event::Missed | Event::Lost => None,
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

    fn holds_something(&self, jaw: Jaw) -> bool {
        jaw.angle_rad > self.closed_rad + self.config.hold_margin_rad
    }

    /// One decision. `hand` is this frame's tracked hand (already bridged over dropouts by the
    /// tracker); `jaw` is the latest measurement, `None` if the robot is not reporting.
    pub fn step(&mut self, now: Instant, hand: Option<Hand>, jaw: Option<Jaw>) -> Step {
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
                let offered = near(c.offer_m);
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

/// What the reader threads hand to the loop.
enum Update {
    Tof(Box<proto::TofFrame>),
    State(Box<proto::RobotState>),
    Lost(String),
}

/// How often the head is resent: the idle look-around (which yawns, opening the beak) starts
/// after two seconds without a `robot.head`.
const HOLD_HEAD_EVERY: Duration = Duration::from_millis(400);
/// The mouth intent is resent this often even when unchanged, so a robotd restart mid-run picks
/// it back up.
const MOUTH_EVERY: Duration = Duration::from_millis(500);
/// A depth frame older than this is no frame.
const FRAME_STALE: Duration = Duration::from_millis(500);
/// Releasing a held object on the way out.
const LET_GO_ON_EXIT: Duration = Duration::from_millis(1500);

pub struct Options {
    pub config: Config,
    pub quiet: bool,
    /// Keep the head straight ahead, which also keeps the idle look-around (and its yawn) away.
    pub hold_head: bool,
}

/// `robotctl object-offer`: run the machine until Ctrl-C.
pub fn run(robot_socket: &Path, tof_socket: &Path, options: Options) -> Result<(), Failure> {
    let mut control = Client::connect_to("robotd", robot_socket)?;
    control.hello()?;

    let (tx, rx) = mpsc::channel();
    {
        let tx = tx.clone();
        let socket = robot_socket.to_path_buf();
        thread::spawn(move || read_state(&socket, &tx));
    }
    {
        let socket = tof_socket.to_path_buf();
        thread::spawn(move || read_tof(&socket, &tx));
    }

    // SAFETY: a handler whose whole body is one relaxed atomic store.
    unsafe {
        libc::signal(
            libc::SIGINT,
            crate::note_interrupt as *const () as libc::sighandler_t,
        );
    }

    let mut out = Out {
        control,
        quiet: options.quiet,
        hold_head: options.hold_head,
        last_mouth: None,
        last_head: None,
    };

    // Calibrate: the jaw closed on nothing.
    println!("closing the beak to measure it…");
    let mut latest_state: Option<Box<proto::RobotState>> = None;
    let mut latest_frame: Option<(Box<proto::TofFrame>, Instant)> = None;
    let calibrate_until = Instant::now() + Duration::from_millis(800);
    while Instant::now() < calibrate_until {
        out.mouth(0.0)?;
        out.head()?;
        drain(
            &rx,
            Duration::from_millis(50),
            &mut latest_state,
            &mut latest_frame,
        );
        if interrupted() {
            return Ok(());
        }
    }
    let Some(state) = latest_state.as_ref() else {
        return Err(Failure::new(
            exit::UNREACHABLE,
            "no robot.state from robotd in 0.8 s".to_owned(),
        ));
    };
    let Some(&closed_rad) = state.joints.get(MOUTH) else {
        return Err(Failure::new(
            exit::FAILED,
            "robot.state carries no mouth angle".to_owned(),
        ));
    };
    if state.policy == "held" {
        println!(
            "⚠ no policy is driving (policy: held) — robotd only moves the mouth while one is; \
             stand the robot up first (Start, or `robotctl enable`)"
        );
    }
    println!(
        "closed beak at {:+.1}° · offer within {:.0} cm · Ctrl-C to stop",
        closed_rad.to_degrees(),
        options.config.offer_m * 100.0
    );

    let mut tracker = Tracker::new(tracker_config(options.config.leave_m + 0.10));
    let mut machine = Machine::new(options.config, closed_rad);
    let mut last_drawn = Instant::now() - Duration::from_secs(1);
    let mut no_frames_said = false;

    while !interrupted() {
        drain(
            &rx,
            Duration::from_millis(30),
            &mut latest_state,
            &mut latest_frame,
        );
        let now = Instant::now();

        let fresh = latest_frame
            .as_ref()
            .filter(|(_, at)| now.duration_since(*at) < FRAME_STALE);
        let short = fresh.map_or(0, |(frame, _)| short_returns(frame));
        let hand =
            fresh.and_then(|(frame, _)| tracker.track(&frame.distance_mm, &frame.status, now));
        if fresh.is_none() && !no_frames_said {
            println!("\r⚠ no depth frames from tofd — is it running?{:40}", "");
            no_frames_said = true;
        } else if fresh.is_some() {
            no_frames_said = false;
        }
        let jaw = latest_state.as_ref().and_then(|s| {
            Some(Jaw {
                angle_rad: *s.joints.get(MOUTH)?,
                current_ma: s.currents_ma.get(MOUTH).copied(),
            })
        });

        let step = machine.step(now, hand, jaw);
        out.mouth(step.mouth)?;
        out.head()?;
        if let Some(event) = step.event {
            println!("\r{:<78}", event.describe());
            if let Some(tag) = event.sound() {
                out.sound(tag);
            }
        }

        if now.duration_since(last_drawn) >= Duration::from_millis(66) {
            last_drawn = now;
            let seen = match hand {
                Some(h) => format!(
                    "{:4.1} cm {:2} zones{}",
                    h.range_m * 100.0,
                    h.zones,
                    if h.held { "~" } else { " " }
                ),
                None => "  — nothing —      ".to_owned(),
            };
            let beak = match jaw {
                Some(j) => format!(
                    "jaw {:+5.1}°{}",
                    (j.angle_rad - closed_rad).to_degrees(),
                    j.current_ma
                        .map(|ma| format!(" {ma:4.0} mA"))
                        .unwrap_or_default()
                ),
                None => "jaw ?".to_owned(),
            };
            let policy = latest_state.as_ref().map_or("?", |s| s.policy.as_str());
            print!(
                "\r  {:<9} {seen} <10cm {short:2}  {beak}  policy {policy:<6}   ",
                machine.phase().name()
            );
            let _ = std::io::stdout().flush();
        }
    }

    println!();
    if machine.holding() {
        println!("letting go on the way out");
        out.sound(proto::SoundTag::Bwep);
        let until = Instant::now() + LET_GO_ON_EXIT;
        while Instant::now() < until {
            out.mouth(1.0)?;
            thread::sleep(Duration::from_millis(100));
        }
    }
    out.last_mouth = None;
    out.mouth(0.0)?;
    Ok(())
}

/// Zones reporting a valid range under 10 cm — below the hand tracker's floor, so not part of any
/// decision. Shown because whether the sensor sees an object sitting in the beak (rather than
/// cover-glass crosstalk) is the open question for a better close trigger.
fn short_returns(frame: &proto::TofFrame) -> usize {
    frame
        .distance_mm
        .iter()
        .zip(&frame.status)
        .filter(|&(&mm, &code)| mm > 0 && mm < 100 && matches!(code, 5 | 9))
        .count()
}

fn interrupted() -> bool {
    crate::INTERRUPTED.load(std::sync::atomic::Ordering::Relaxed)
}

/// Take everything the readers sent, waiting up to `wait` for the first of it.
fn drain(
    rx: &mpsc::Receiver<Update>,
    wait: Duration,
    state: &mut Option<Box<proto::RobotState>>,
    frame: &mut Option<(Box<proto::TofFrame>, Instant)>,
) {
    let mut next = rx.recv_timeout(wait).ok();
    while let Some(update) = next {
        match update {
            Update::Tof(f) => *frame = Some((f, Instant::now())),
            Update::State(s) => *state = Some(s),
            Update::Lost(why) => println!("\r⚠ {why}{:40}", ""),
        }
        next = rx.try_recv().ok();
    }
}

/// The commands going to robotd, with their resend cadence.
struct Out {
    control: Client,
    quiet: bool,
    hold_head: bool,
    last_mouth: Option<(f64, Instant)>,
    last_head: Option<Instant>,
}

impl Out {
    fn mouth(&mut self, open: f64) -> Result<(), Failure> {
        let due = match self.last_mouth {
            Some((was, at)) => was != open || at.elapsed() >= MOUTH_EVERY,
            None => true,
        };
        if due {
            self.control
                .send(&proto::Request::notify(&proto::Call::RobotMouth(
                    proto::MouthParams { open },
                )))?;
            self.last_mouth = Some((open, Instant::now()));
        }
        Ok(())
    }

    fn head(&mut self) -> Result<(), Failure> {
        if !self.hold_head
            || self
                .last_head
                .is_some_and(|t| t.elapsed() < HOLD_HEAD_EVERY)
        {
            return Ok(());
        }
        self.control
            .send(&proto::Request::notify(&proto::Call::RobotHead(
                proto::HeadParams::default(),
            )))?;
        self.last_head = Some(Instant::now());
        Ok(())
    }

    /// Fire and forget: a sound is decoration, and a refusal (no voice) is not worth stopping for.
    fn sound(&mut self, tag: proto::SoundTag) {
        if self.quiet {
            return;
        }
        let _ = self
            .control
            .send(&proto::Request::notify(&proto::Call::RobotSound(
                proto::SoundParams { tag, hold: None },
            )));
    }
}

/// How long a reader waits before reconnecting a stream that ended.
const RETRY: Duration = Duration::from_secs(2);

fn read_state(socket: &Path, tx: &mpsc::Sender<Update>) {
    loop {
        let why = stream_state(socket, tx).unwrap_or_else(|why| why);
        if tx
            .send(Update::Lost(format!("robot.state: {why}")))
            .is_err()
        {
            return;
        }
        thread::sleep(RETRY);
    }
}

fn stream_state(socket: &Path, tx: &mpsc::Sender<Update>) -> Result<String, String> {
    let mut client = Client::connect_to("robotd", socket).map_err(|e| e.message)?;
    client.hello().map_err(|e| e.message)?;
    client
        .send(&proto::Request::call(
            proto::Id::Number(1),
            &proto::Call::RobotSubscribe(proto::SubscribeParams { hz: Some(25) }),
        ))
        .map_err(|e| e.message)?;
    let mut line = String::new();
    loop {
        line.clear();
        match client.reader.read_line(&mut line) {
            Err(e) => return Err(format!("the stream stopped: {e}")),
            Ok(0) => return Err("robotd closed the stream".to_owned()),
            Ok(_) => {}
        }
        if let Some(state) = serde_json::from_str::<proto::Request>(&line)
            .ok()
            .and_then(|r| r.as_state())
            && tx.send(Update::State(Box::new(state))).is_err()
        {
            return Ok(String::new());
        }
    }
}

fn read_tof(socket: &Path, tx: &mpsc::Sender<Update>) {
    loop {
        let why = stream_tof(socket, tx).unwrap_or_else(|why| why);
        if tx.send(Update::Lost(format!("depth: {why}"))).is_err() {
            return;
        }
        thread::sleep(RETRY);
    }
}

fn stream_tof(socket: &Path, tx: &mpsc::Sender<Update>) -> Result<String, String> {
    let mut client = Client::connect_to("tofd", socket).map_err(|e| e.message)?;
    client
        .send(&proto::Request::call(
            proto::Id::Number(1),
            &proto::Call::TofStream,
        ))
        .map_err(|e| e.message)?;
    let mut line = String::new();
    loop {
        line.clear();
        match client.reader.read_line(&mut line) {
            Err(e) => return Err(format!("the stream stopped: {e}")),
            Ok(0) => return Err("tofd closed the stream".to_owned()),
            Ok(_) => {}
        }
        // The answer names the sensor; say so if there is none.
        if let Ok(response) = serde_json::from_str::<proto::Response>(&line)
            && let Ok(status) = response.result_as::<proto::TofStreamResult>()
        {
            if status.sensor.is_none() {
                let why = status.unavailable.unwrap_or_else(|| "no sensor".to_owned());
                return Err(format!("tofd has no sensor: {why}"));
            }
            continue;
        }
        if let Some(frame) = serde_json::from_str::<proto::Request>(&line)
            .ok()
            .and_then(|r| r.as_tof_frame())
            && tx.send(Update::Tof(Box::new(frame))).is_err()
        {
            return Ok(String::new());
        }
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
    }

    impl Rig {
        fn new() -> Self {
            Self {
                m: Machine::new(Config::default(), CLOSED),
                t: Instant::now(),
                events: Vec::new(),
                mouth: 0.0,
            }
        }

        fn run(&mut self, ms: u64, hand: Option<Hand>, jaw: Option<Jaw>) {
            for _ in 0..ms / 20 {
                self.t += 20 * MS;
                let step = self.m.step(self.t, hand, jaw);
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

    #[test]
    fn a_robot_not_reporting_the_jaw_is_believed() {
        let mut rig = Rig::new().opened().inserted();
        rig.run(800, None, None);
        assert_eq!(rig.events.last(), Some(&Event::Grabbed));
    }
}
