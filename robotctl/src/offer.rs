//! `robotctl object-offer` — hold something out to the duck and it takes it in its beak.
//!
//! A prototype, run from the board's shell, built only from what robotd already serves: the head
//! ToF's depth frames (`tofd`), the jaw's measured angle and current (`robot.state`), and the
//! mouth intent (`robot.mouth`). Nothing in robotd changes for it, so it is as safe to try as
//! `robotctl theremin`.
//!
//! The decisions are [`kinematics::offer`], shared with `autod` (which takes offers on its own);
//! this file is the plumbing around them and the live readout.

use std::io::{BufRead, Write};
use std::path::Path;
use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant};

use duck_ipc_proto as proto;
use kinematics::hand::Tracker;
pub use kinematics::offer::Config;
use kinematics::offer::{Event, Jaw, Machine, tracker_config};

use crate::{Client, Failure, exit};

/// Index of the mouth in `robot.state`'s joint arrays.
const MOUTH: usize = 9;

/// What each event sounds like on the bench.
fn sound_of(event: Event) -> Option<proto::SoundTag> {
    match event {
        Event::Opened => Some(proto::SoundTag::Inquire),
        Event::Grabbed => Some(proto::SoundTag::Chirp),
        Event::GaveUp => Some(proto::SoundTag::Peck),
        Event::Released => Some(proto::SoundTag::Bwep),
        Event::Withdrawn | Event::Closing | Event::Missed | Event::Lost => None,
    }
}

/// What the reader threads hand to the loop.
enum Update {
    Tof(Box<proto::TofFrame>),
    State(Box<proto::RobotState>),
    Lost(String),
}

/// How often the head is resent while the beak holds something: the idle look-around (which
/// yawns, opening the beak) starts after two seconds without a `robot.head`.
const HOLD_HEAD_EVERY: Duration = Duration::from_millis(400);
/// While engaged, the mouth intent is resent this often even when unchanged, so a robotd restart
/// mid-run picks it back up.
const MOUTH_EVERY: Duration = Duration::from_millis(500);
/// A depth frame older than this is no frame.
const FRAME_STALE: Duration = Duration::from_millis(500);
/// Releasing a held object on the way out.
const LET_GO_ON_EXIT: Duration = Duration::from_millis(1500);

pub struct Options {
    pub config: Config,
    pub quiet: bool,
    /// Keep the head straight ahead while holding something, which keeps the idle look-around
    /// (and its yawn) away.
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
    out.mouth(0.0, false)?;
    while Instant::now() < calibrate_until {
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

        let step = machine.step(now, hand, jaw, true);
        out.mouth(step.mouth, machine.engaged())?;
        out.head(machine.guards_beak())?;
        if let Some(event) = step.event {
            println!("\r{:<78}", event.describe());
            if let Some(tag) = sound_of(event) {
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
            out.mouth(1.0, true)?;
            thread::sleep(Duration::from_millis(100));
        }
    }
    if machine.engaged() {
        out.last_mouth = None;
        out.mouth(0.0, false)?;
    }
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
    /// Send the mouth when it changes; with `keep`, also resend it on [`MOUTH_EVERY`].
    fn mouth(&mut self, open: f64, keep: bool) -> Result<(), Failure> {
        let due = match self.last_mouth {
            Some((was, at)) => was != open || (keep && at.elapsed() >= MOUTH_EVERY),
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

    /// Hold the head straight ahead while `active`; send nothing otherwise.
    fn head(&mut self, active: bool) -> Result<(), Failure> {
        if !active
            || !self.hold_head
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
