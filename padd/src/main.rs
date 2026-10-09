//! `padd` — a gamepad, as an intent client.
//!
//! It has no privileged access to the robot. It reads a pad, turns sticks and buttons into
//! intents, and sends them over `robotd`'s socket like any other client.
//!
//! That is the point of it being a separate process rather than a thread inside `robotd`.
//! The intent API is the path the app, the SDK and any remote client will use, and here it
//! gets exercised every day by whoever is working on the robot — so it cannot quietly rot
//! the way an API only the phone app uses inevitably would. The cost is a socket hop: tens
//! of microseconds against a 20 ms tick.
//!
//! ## The mapping
//!
//! Laid out so that each part of the pad has one job: the face buttons and bumpers run skills,
//! the D-pad picks what the sticks mean, and the two small buttons in the middle are holds —
//! nothing that cuts torque or powers the robot off fires on a brush of the thumb.
//!
//! The six one-shot buttons are `[pad]` in `robotd.toml`, so a robot that has learned a new skill
//! can put it on a button without a release. Only those six: `Start`, the D-pad and held `Select`
//! are not `robot.do` calls, and the button that powers a robot off is the one binding worth not
//! being able to lose to a config edit.
//!
//! This daemon still knows nothing about what a skill *is*. It reads which button went down,
//! looks up the name beside it, and sends that name; `robotd` decides whether the robot has such
//! a thing and answers with the list it does have when it does not.
//!
//! ```text
//! A (South)       sit ↔ stand
//! B (East)        ground pick
//! X / Y           nothing, until a skill is bound there
//! LB / RB         left / right kick
//! RT / LT         mouth (either trigger; the max wins) · RT quacks · LT rides the wheee
//! D-pad up        head mode — the sticks pose the head, the body holds still
//! D-pad right     head + move — left stick walks and turns, right stick looks around
//! D-pad left      move — the sticks walk, strafe and turn
//! D-pad down      body + head — left stick crouches and leans sideways, right stick looks around
//! Home            flashlight on / off (a board with one)
//! Start           first press stands up, then toggles the policy
//! Start, 1.5 s    home pose, motors stiff, policy off — a seated robot stays seated
//! Select, 2–4 s   let go: sit, rest pose, then torque off and every servo rebooted
//! Select, 4 s     sit, rest pose, then power off
//! ```
//!
//! The D-pad *selects* a mode rather than toggling one, so a press always lands where its arrow
//! says whatever mode the robot was in. Head and body + head mode both zero the velocity while
//! active — a robot that keeps walking because you started posing its head is a bad surprise —
//! and leaving a mode puts back what it moved: the body snaps to nominal, the head re-centres.
//!
//! Smoothing lives in `robotd` (`[control] cmd_alpha` / `head_alpha`), not here: this
//! process sends raw targets, so every client gets the same feel.
//!
//! ## Roller mode
//!
//! At startup this asks `robot.mode`. On a roller robot the stick mapping becomes the
//! prototype's roller preset — asymmetric forward/brake (0.6 / 0.5), no strafe, ±0.3 rad/s
//! heading — and B triggers the crouch that lives in the ground-pick slot. The other
//! skills ride along on wheels, as the rebased roller line has them. Switching between walk and
//! roller is `robot.setMode`; it is no longer on the pad.
//!
//! ## On the robot, this runs itself
//!
//! `padd.service` starts at boot and stays up whether or not a pad is present, so driving takes one
//! step and it is a pairing step: `sudo robotctl pad pair`, with the pad in pairing mode. The
//! pad is bonded *and trusted*, so it reconnects by itself afterwards, and this process picks it up
//! within a tick.
//!
//! Waiting with no pad is deliberately cheap — no stick command is sent, and `robotd`'s deadman
//! stops the walking on its own; inventing a zero command instead would mask a disconnected pad
//! as someone's decision to stop. A pad that was *driving* and goes away is different: the robot
//! asks where it went (a sound), and after a second sits down, so it is not left standing in the
//! middle of a room. The pad coming back is greeted; A stands the robot up.
//!
//! Pairing is **not** done here: bonding a device needs root and BlueZ, and a `padd` holding
//! either would stop being the unprivileged client whose whole value is having no special
//! access. It lives in `configd`, next to wifi.
//!
//! ## It also hands out the pad's raw input
//!
//! One socket, read-only, for `pad.input` and nothing else — `src/tap.rs`, and it does not make this
//! a privileged process. It exists because `padd` is the reason a stalled radio is invisible: the
//! sticks are *polled*, so the last known value keeps being sent — at the full rate, since a stick
//! reading anything but centre is never held back — whether or not the pad is still talking, and
//! every surface downstream then shows a robot with a live driver. The event
//! stream one layer below has the evidence, so it is passed out unaltered rather than summarised.
//! `robotctl monitor` draws it; `docs/robot/pair-a-gamepad.md` says how to read it.
//!
//! For development against a board: `ssh -L /tmp/robotd.sock:/run/robotd.sock duck`, then
//! point `--socket` at the forwarded path. Pad on your laptop, robot on the bench, no code.
//! `systemctl stop padd` first, or two processes fight over the sticks. Run that way, `--tap-socket`
//! wants a path you can write — `/run/padd/` belongs to the unit — and on a Mac there is no tap at
//! all, since it reads evdev.

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use clap::Parser;
use duck_ipc_proto as proto;
use gilrs::{Axis, Button, GilrsBuilder, MappingSource};

mod mappings;

#[cfg(target_os = "linux")]
mod tap;

/// The raw tap, on a platform with no evdev to read.
///
/// A `padd` on a Mac still drives a pad — that is the bench setup in the crate docs above, and it
/// would be a poor trade to lose it over a debug facility. It serves no tap, and `robotctl monitor`
/// finds no socket and says so, which is the truth rather than an empty stream.
#[cfg(not(target_os = "linux"))]
mod tap {
    pub struct Tap;

    impl Tap {
        pub fn serve(_socket: &std::path::Path) -> std::io::Result<Self> {
            Err(std::io::Error::other(
                "the raw pad tap reads evdev, which only Linux has",
            ))
        }

        pub fn watch(&self, _pad: &gilrs::Gamepad<'_>) {}

        pub fn idle(&self) {}

        pub fn imu_control(&self, _on: bool) {}

        pub fn has_imu(&self) -> bool {
            false
        }

        pub fn attitude(&self) -> Option<[f32; 4]> {
            None
        }
    }
}

#[derive(Parser, Debug)]
#[command(name = "padd", about = "Drive the robot from a gamepad", version)]
struct Args {
    /// `robotd`'s socket.
    #[arg(long, default_value = "/run/robotd.sock")]
    socket: PathBuf,

    /// Where the button bindings are read from — the same file everything else is configured in.
    #[arg(long, default_value = robotd_params::DEFAULT_PATH)]
    config: PathBuf,

    /// How often to read the pad, 1–1000 Hz. Matching the control rate exactly buys nothing —
    /// the loop reads the latest value once per tick — but staying at or above it keeps the
    /// added latency under one tick.
    ///
    /// Not quite how often intents are *sent*: a frame identical to the last one and asking
    /// for no motion is held back, down to [`HEARTBEAT`]. See [`Continuous`].
    // Bounded both ways, and refused rather than clamped so the flag says what it did.
    // Zero reaches `1.0 / 0.0` and `Duration::from_secs_f64` panics on infinity. The ceiling
    // is the other half of the same line: a rate this loop cannot keep gives a period of 0 ns,
    // `checked_sub` never has anything left to sleep on, and the pad spins on robotd's socket —
    // the same busy loop `robotctl monitor` clamps for, and the range `control.hz` already
    // rejects outside.
    #[arg(long, default_value_t = 50, value_parser = clap::value_parser!(u32).range(1..=1000))]
    hz: u32,

    /// Deflection below this counts as centre. Analogue sticks rarely rest at exactly zero,
    /// and without this the robot creeps. The prototype's value.
    #[arg(long, default_value_t = 0.1)]
    deadzone: f64,

    /// Full-deflection head travel, radians. The head command feeds the policy's
    /// observation rather than a servo directly, so this is the prototype's generous 2.5 —
    /// the network itself decides how far the head actually goes.
    #[arg(long, default_value_t = 2.5)]
    max_head: f64,

    /// Where to serve the raw input tap: the pad's own event stream, for `robotctl monitor`.
    ///
    /// Read-only, and nothing on the driving path depends on it — if the socket cannot be created
    /// `padd` says so once and drives anyway.
    #[arg(long, default_value = proto::socket::PAD)]
    tap_socket: PathBuf,
}

/// How long to wait between checks when there is no pad.
///
/// Longer than a control tick on purpose. This process now runs from boot on every robot, and most
/// of the time there is no pad connected at all — spinning at the control rate to discover that
/// again is a wakeup every 20 ms, forever, for nothing. Half a second is imperceptible when someone
/// switches a pad on and is not a background load.
const IDLE_POLL: Duration = Duration::from_millis(500);

/// The pad gone this long sits the robot down. Past the deadman, which has already stopped any
/// walking at half a second, and short enough that a robot left standing in the middle of a room
/// is sitting before anybody wonders why it froze.
const PAD_LOST_SIT: Duration = Duration::from_secs(1);

/// How long to keep asking for that sit when the robot refuses it — mid-kick, mid-pick, mid-rise.
/// Each refused move ends within a few seconds; past this, something else is going on and the
/// journal says so rather than the pad asking forever.
const PAD_LOST_SIT_TRIES: Duration = Duration::from_secs(5);

/// How often to ask, while a sit is due. Faster than [`IDLE_POLL`] so the one-second promise holds.
const PAD_LOST_POLL: Duration = Duration::from_millis(100);

/// What to do about a pad that went away, tick by tick.
///
/// Only a pad that was *driving* can be lost: a `padd` that starts with no pad has nothing to
/// react to, and must not sit a robot somebody else is driving.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
enum PadLoss {
    /// The pad is here, or never was.
    #[default]
    Present,
    /// Gone since then; the sit is not due yet, or is being asked for.
    Lost { since: Instant },
    /// Gone, and the sit has been dealt with — done, or given up on.
    Settled,
}

/// What [`PadLoss::tick`] asks the loop to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PadLossAction {
    Nothing,
    /// Try to sit the robot now; [`PadLoss::settle`] once it is, or ask again next poll.
    Sit,
    /// The tries ran out.
    GiveUp,
}

impl PadLoss {
    fn lost(&mut self, now: Instant) {
        *self = Self::Lost { since: now };
    }

    /// The pad is back. True when it had been lost, for the greeting.
    fn found(&mut self) -> bool {
        let was_lost = *self != Self::Present;
        *self = Self::Present;
        was_lost
    }

    fn tick(&self, now: Instant) -> PadLossAction {
        match *self {
            Self::Lost { since } => {
                let gone = now.duration_since(since);
                if gone >= PAD_LOST_SIT + PAD_LOST_SIT_TRIES {
                    PadLossAction::GiveUp
                } else if gone >= PAD_LOST_SIT {
                    PadLossAction::Sit
                } else {
                    PadLossAction::Nothing
                }
            }
            Self::Present | Self::Settled => PadLossAction::Nothing,
        }
    }

    /// The robot is sitting, or there was nothing to sit: stop asking.
    fn settle(&mut self) {
        if matches!(self, Self::Lost { .. }) {
            *self = Self::Settled;
        }
    }

    /// Whether the loop should poll quickly, because a sit is due or about to be.
    fn pending(&self) -> bool {
        matches!(self, Self::Lost { .. })
    }
}

/// Start held this long brings the robot to its home pose with the motors stiff and the policy
/// off: the "put everything back" button. Long enough that a press meant for the policy toggle
/// never reaches it, short enough to be the obvious thing to do when the robot is somewhere odd.
const HOME_HOLD: Duration = Duration::from_millis(1500);

/// Select let go after this long, and before [`SHUTDOWN_HOLD`], puts the robot down for a rest:
/// `robot.rest` — it sits if it is driving, eases into the rest pose, then torque goes off and
/// every servo reboots. Decided on the release, because until then the hold may still become a
/// power-off, and both start the same way.
const REST_HOLD: Duration = Duration::from_secs(2);

/// Select held this long powers the robot off: `robot.shutdown`, the same sit and rest pose ending
/// in a power-off. Sent the moment the hold gets here; the release after it does nothing.
const SHUTDOWN_HOLD: Duration = Duration::from_secs(4);

/// A button that does different things depending on how long it is held.
///
/// Each threshold fires once, on the tick the hold crosses it, so a longer hold walks through
/// every shorter one on the way — Select cuts torque at two seconds and then powers off at four.
/// A release that crossed no threshold is a [`HoldAction::Tap`]; the release after a hold that
/// did fire says nothing, because the thumb coming off is not a second request.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
struct HoldButton {
    /// When the current hold began. `None` between presses.
    held_since: Option<Instant>,
    /// How many thresholds this hold has crossed.
    fired: usize,
    /// The pad went away during a hold that had already fired. Whatever is still held when it
    /// comes back is the tail of that hold, and does nothing until the release.
    spent: bool,
}

/// What a [`HoldButton`] asks for this tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum HoldAction {
    Nothing,
    /// Pressed and let go before the first threshold.
    Tap,
    /// The hold just crossed this threshold, by index.
    Reached(usize),
    /// Let go after a hold whose furthest threshold was this one, by index. For a button whose
    /// action depends on where the hold stopped, which is only known at the release.
    ReleasedAfter(usize),
}

impl HoldButton {
    /// One tick: is the button down now, and did it come up since the last tick.
    ///
    /// `released` is the edge from the event queue, because a press and release inside one tick
    /// leaves `pressed` false on both sides and would otherwise be a tap nobody saw.
    fn tick(
        &mut self,
        pressed: bool,
        released: bool,
        now: Instant,
        thresholds: &[Duration],
    ) -> HoldAction {
        if pressed {
            let since = *self.held_since.get_or_insert(now);
            if !self.spent
                && let Some(&next) = thresholds.get(self.fired)
                && now.duration_since(since) >= next
            {
                self.fired += 1;
                return HoldAction::Reached(self.fired - 1);
            }
            return HoldAction::Nothing;
        }
        let was_held = released || self.held_since.is_some();
        let (fired, spent) = (self.fired, self.spent);
        *self = Self::default();
        match (fired, spent) {
            // The tail of a hold cut by a dropout says nothing — see `reset`.
            (_, true) => HoldAction::Nothing,
            (0, false) if released => HoldAction::Tap,
            (0, false) => HoldAction::Nothing,
            (n, false) if was_held => HoldAction::ReleasedAfter(n - 1),
            (_, false) => HoldAction::Nothing,
        }
    }

    /// Forget a hold in flight. Called when the pad goes away: the hold's start was measured
    /// against *that* pad's button, and carrying it onto the next pad would turn a button still
    /// held across a long dropout into its longest action on the first tick back.
    ///
    /// A hold that already fired is marked spent rather than forgotten, because what it did is a
    /// fact about the robot rather than about the pad: the release that follows must stay silent,
    /// and the rest of the hold must not reach the next threshold from a fresh start.
    fn reset(&mut self) {
        self.held_since = None;
        if self.fired > 0 {
            self.spent = true;
        }
    }
}

/// Body-pose stick ranges, from the training env via the prototype: z is asymmetric
/// (little headroom up at the standing height, more crouch down), angles capped at ~15°.
const BODY_MAX_Z_UP: f64 = 0.010;
const BODY_MAX_Z_DOWN: f64 = 0.025;
const BODY_MAX_ANGLE: f64 = 0.2618;

/// The prototype's roller-mode stick shaping: push and brake are asymmetric, there is no
/// strafe, and heading is capped at 0.3 rad/s regardless of the walking limits — the
/// roller launch line's `--max-angular-vel 0.3`, unchanged across both of its eras.
const ROLLER_PUSH: f64 = 0.6;
const ROLLER_BRAKE: f64 = 0.5;
const ROLLER_YAW: f64 = 0.3;

/// What the sticks drive, picked on the D-pad. Modal because two sticks cannot express nine
/// degrees of freedom.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    /// D-pad left. Left stick walks and strafes, right stick turns.
    Drive,
    /// D-pad up. All four axes pose the head; the body holds still.
    Head,
    /// D-pad right. Left stick walks and turns, right stick looks around — or, with
    /// `[pad_imu_head_control]` on a pad that has an IMU, the pad's tilt poses the head and the
    /// sticks keep the whole [`Mode::Drive`] mapping.
    HeadDrive,
    /// D-pad down. Body + head: the left stick crouches and leans the standing robot sideways,
    /// the right stick looks around. The body does not walk.
    BodyPose,
}

impl Mode {
    /// Whether this mode sends head poses, so leaving it has a head to put back.
    fn poses_head(self) -> bool {
        matches!(self, Self::Head | Self::HeadDrive | Self::BodyPose)
    }
}

/// What has to be sent when the sticks stop meaning `from` and start meaning `to`.
///
/// Leaving a mode puts back what it moved, because nothing else will: a body left leaning or a
/// head left turned stays that way, and the next mode does not send the joints the last one did.
/// Moving between modes that all pose the head keeps the head, since the next one goes on posing
/// it.
fn mode_exit_calls(from: Mode, to: Mode) -> Vec<proto::Call> {
    let mut calls = Vec::new();
    if from == Mode::BodyPose && to != Mode::BodyPose {
        calls.push(proto::Call::RobotPose(proto::PoseParams {
            active: false,
            ..Default::default()
        }));
    }
    if from.poses_head() && !to.poses_head() {
        calls.push(proto::Call::RobotHead(proto::HeadParams::default()));
    }
    calls
}

/// How often to look for a rewritten config. See the loop.
const BINDINGS_POLL: Duration = Duration::from_secs(1);

/// The button bindings, the IMU head switch and the walking speeds, or the defaults.
///
/// A file that will not parse is never a reason to leave somebody without a pad: the defaults
/// are a working robot, and the reason is logged. That matters more here than elsewhere because
/// this is re-read while running — a half-saved file caught mid-write must not take the buttons
/// away, and the next read a second later gets the finished one.
fn read_bindings(
    path: &Path,
) -> (
    robotd_params::PadParams,
    robotd_params::PadImuHeadControlParams,
    robotd_params::PadDriveParams,
) {
    match robotd_params::Params::load(path, false) {
        Ok(params) => {
            let pad = params.pad;
            let imu_head = params.pad_imu_head_control;
            let drive = params.pad_drive;
            tracing::info!(
                a = %pad.a, b = %pad.b, x = %pad.x, y = %pad.y, lb = %pad.lb, rb = %pad.rb,
                pad_imu_head_control = imu_head.enabled, pad_imu_head_gain = imu_head.gain,
                vx = ?(drive.vx_min, drive.vx_max), vy = ?(drive.vy_min, drive.vy_max),
                vyaw = ?(drive.vyaw_min, drive.vyaw_max),
                "button bindings"
            );
            (pad, imu_head, drive)
        }
        Err(e) => {
            tracing::warn!(
                error = %e,
                path = %path.display(),
                "cannot read the button bindings; using the default mapping"
            );
            (
                robotd_params::PadParams::default(),
                robotd_params::PadImuHeadControlParams::default(),
                robotd_params::PadDriveParams::default(),
            )
        }
    }
}

/// The head pose for a pad attitude relative to its reference.
///
/// `relative` is body → world of the pad now, in the frame of the reference — [`pad_imu::relative`].
/// Its pitch, roll and yaw become the head's, scaled by `gain` and clamped to `max_head`. The signs
/// are the ones that made the head copy the pad on the robot (2026-09-09): pad nose-up is a
/// positive `head_pitch`, pad yaw to the left a positive `head_yaw`, and the pad rolling right
/// (left side up) a negative `head_roll`. Pitch and roll came out opposite to the stick mapping's
/// guess, which is worth knowing: the sticks' signs describe "stick up looks up", not the joint
/// axes, and the pad frame is the joints'. The neck stays at zero: one pitch joint is enough to
/// follow a wrist.
fn head_from_pad(relative: [f32; 4], gain: f64, max_head: f64) -> proto::HeadParams {
    let [pitch, roll, yaw] = pad_imu::euler_deg(relative);
    let angle = |degrees: f32| (f64::from(degrees).to_radians() * gain).clamp(-max_head, max_head);
    proto::HeadParams {
        neck_pitch: 0.0,
        head_pitch: angle(pitch),
        head_yaw: angle(yaw),
        head_roll: -angle(roll),
    }
}

/// When the config was last written, for spotting a change. `None` for a file that is not there,
/// which is a real state and compares equal to itself.
fn config_mtime(path: &Path) -> Option<std::time::SystemTime> {
    std::fs::metadata(path).ok()?.modified().ok()
}

fn main() -> std::process::ExitCode {
    let args = Args::parse();
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_writer(std::io::stderr)
        .init();

    // Before anything that can fail, and before the gamepad subsystem especially: `padd` was the
    // one daemon whose journal could not say which build was running, which came up while chasing
    // exactly that question across all five.
    duck_ipc_proto::log_startup_identity!("padd");

    // Mappings for pads `gilrs` ships none for, handed over *before* its own database so the
    // database still wins wherever it knows a pad — see `mappings` for why a pad can be missing.
    let mut gilrs = match GilrsBuilder::new()
        .add_mappings(&mappings::xbox_series_ble())
        .build()
    {
        Ok(gilrs) => gilrs,
        Err(e) => {
            tracing::error!(error = %e, "no gamepad subsystem");
            return std::process::ExitCode::FAILURE;
        }
    };

    // Before robotd's socket on purpose: a `padd` that cannot reach `robotd` exits and is retried
    // by systemd, and the tap is the one thing here that could have told someone why the pad looked
    // dead. Its own failure is logged and stepped over — see `--tap-socket`.
    let tap = match tap::Tap::serve(&args.tap_socket) {
        Ok(tap) => Some(tap),
        Err(e) => {
            tracing::warn!(
                error = %e, socket = %args.tap_socket.display(),
                "no raw pad tap — `robotctl monitor` cannot show the pad's own event stream"
            );
            None
        }
    };

    let mut stream = match UnixStream::connect(&args.socket) {
        Ok(stream) => stream,
        Err(e) => {
            tracing::error!(error = %e, socket = %args.socket.display(), "cannot reach robotd");
            return std::process::ExitCode::FAILURE;
        }
    };

    let mut next_id = 1u64;

    // Which robot is this? A roller duck wants the roller stick shaping. Asked at startup and then
    // again with the config check below, so a `robot.setMode` from anywhere — the pad no longer
    // switches modes itself — changes the stick shaping within a second.
    let mut roller = match ask_roller(&mut stream, &mut next_id) {
        Ok(roller) => roller.unwrap_or(false),
        Err(e) => {
            tracing::error!(error = %e, "mode request failed");
            return std::process::ExitCode::FAILURE;
        }
    };
    tracing::warn!(
        socket = %args.socket.display(),
        hz = args.hz,
        roller,
        "driving — A sit, B ground pick, LB/RB kicks, triggers mouth; D-pad up head, \
         right head + move, left move, down body + head; Start stands up then toggles the policy, \
         Start (1.5s) home pose; Select (2-4s) rest, Select (4s) power off"
    );

    let period = Duration::from_secs_f64(1.0 / args.hz as f64);
    // The button bindings, read once like every other daemon reads its config. A file that will
    // not parse is not a reason to leave somebody without a pad: the default mapping is the
    // fallback, and the reason is logged.
    let (mut bindings, mut imu_head_cfg, mut drive) = read_bindings(&args.config);
    // When the file was last written, so a change is picked up without a restart. `padd` holds
    // no motor control and no session state — the whole of it is this table — so re-reading is a
    // swap between two ticks rather than anything to sequence.
    let mut bindings_at = config_mtime(&args.config);
    let mut bindings_checked = Instant::now();

    let mut mode = Mode::Drive;
    // The pad attitude that reads as "head centred" while head + move follows the pad's IMU.
    // `None` whenever it is not following — another mode, the feature off, no IMU, or the pad
    // gone: a reference taken against one pad means nothing to the next.
    let mut imu_reference: Option<[f32; 4]> = None;
    // Whether a pad was there last tick, so appearing and disappearing are each logged once.
    let mut driving = false;
    let mut start = HoldButton::default();
    let mut select = HoldButton::default();
    // A driving pad that went away: the robot sits after a second — see `PadLoss`.
    let mut pad_loss = PadLoss::default();
    // Trigger levels last tick, for the sound edges: RT quacks on its rising edge, LT
    // starts the wheee ride. The prototype's threshold.
    let mut prev_rt = 0.0f64;
    let mut prev_lt = 0.0f64;
    // Does this pad think the robot is up (torque on, at the home pose)? When not, Start sends
    // `robot.init` (stand up and hold) instead of enabling the policy; the next Start enables it.
    // Starts false: padd starts with the robot, and a robotd restart restarts padd too.
    let mut up = false;
    // The continuous intents, and the buffer this tick's are built in. Both live across
    // ticks so a steady state neither allocates nor re-sends — see [`Continuous`].
    let mut continuous = Continuous::default();
    let mut frame: Vec<proto::Call> = Vec::with_capacity(2);

    loop {
        let tick = Instant::now();

        // Once a second, not every tick: a `stat` at 50 Hz to catch a file somebody edits by
        // hand a few times a week is work for nothing, and a second is faster than typing the
        // next command.
        if tick.duration_since(bindings_checked) >= BINDINGS_POLL {
            bindings_checked = tick;
            let now = config_mtime(&args.config);
            if now != bindings_at {
                bindings_at = now;
                (bindings, imu_head_cfg, drive) = read_bindings(&args.config);
                tracing::warn!("button bindings reloaded");
            }
            match ask_roller(&mut stream, &mut next_id) {
                Ok(Some(now)) if now != roller => {
                    roller = now;
                    tracing::warn!(roller, "drive mode changed — stick shaping follows");
                }
                Ok(_) => {}
                Err(e) => {
                    tracing::error!(error = %e, "mode request failed");
                    return std::process::ExitCode::FAILURE;
                }
            }
        }

        // Drain the queue so axis polling below sees present state, and catch button
        // *edges* — a held D-pad must select once, not fifty times a second.
        //
        // The driving pad is the first one, and only its events may act: with two pads
        // connected, a Start or Select from the *other* one would otherwise steer a robot
        // whose sticks belong to somebody else.
        let pad_id = gilrs.gamepads().next().map(|(id, _)| id);
        let mut wanted_mode: Option<Mode> = None;
        // Which bindable buttons went down this tick, by their config name. A list rather than
        // a flag apiece, because what each one runs is config now and this loop no longer knows.
        let mut pressed: Vec<&'static str> = Vec::new();
        let mut start_released = false;
        let mut select_released = false;
        let mut flashlight = false;
        while let Some(event) = gilrs.next_event() {
            if Some(event.id) != pad_id {
                continue;
            }
            match event.event {
                // Start and Select are holds: what they do depends on how long they stay down,
                // so they are read off their state every tick and their release here.
                gilrs::EventType::ButtonReleased(Button::Start, _) => start_released = true,
                gilrs::EventType::ButtonReleased(Button::Select, _) => select_released = true,
                gilrs::EventType::ButtonPressed(button, _) => match button {
                    // The six bindable ones. What each runs is `[pad]` in the config; this
                    // only knows which physical control was pressed.
                    //
                    // gilrs names the *bumpers* `LeftTrigger`/`RightTrigger`; the analog
                    // triggers are `LeftTrigger2`/`RightTrigger2`. Getting that backwards binds
                    // a skill to a control nobody presses.
                    Button::South => pressed.push("a"),
                    Button::East => pressed.push("b"),
                    Button::West => pressed.push("x"),
                    Button::North => pressed.push("y"),
                    Button::LeftTrigger => pressed.push("lb"),
                    Button::RightTrigger => pressed.push("rb"),
                    // The D-pad picks what the sticks mean.
                    Button::DPadUp => wanted_mode = Some(Mode::Head),
                    Button::DPadRight => wanted_mode = Some(Mode::HeadDrive),
                    Button::DPadLeft => wanted_mode = Some(Mode::Drive),
                    Button::DPadDown => wanted_mode = Some(Mode::BodyPose),
                    // Home. Not bindable: it is not a skill, and the robot owns the toggle.
                    Button::Mode => flashlight = true,
                    _ => {}
                },
                _ => {}
            }
        }

        // Re-read the pad by id after the drain: a `Disconnected` dequeued above may have
        // taken it, and asking for the first pad again would silently hand the robot the
        // *other* pad's sticks. One tick of "pad gone" beats that.
        let Some(pad) = pad_id.and_then(|id| gilrs.connected_gamepad(id)) else {
            // No pad. Send nothing: `robotd`'s deadman stops the robot on its own, which is
            // exactly the wanted behaviour, and inventing a zero command here would mask a
            // disconnected pad as a deliberate stop.
            //
            // Logged once per transition, at `warn` so it survives `RUST_LOG=warn` on a board.
            // "The pad went away" is the single most useful line in the journal when the robot
            // stops responding mid-drive, and one line per tick would bury it.
            if driving {
                tracing::warn!(
                    "pad gone — robotd's deadman stops the walking; sitting down in {PAD_LOST_SIT:?}"
                );
                driving = false;
                pad_loss.lost(tick);
                // Heard, because whoever lost the pad is looking at the robot, not the journal.
                sound(&mut stream, proto::SoundTag::Inquire);
            }
            match pad_loss.tick(tick) {
                PadLossAction::Nothing => {}
                PadLossAction::Sit => match sit_if_standing(&mut stream, &mut next_id) {
                    Ok(true) => pad_loss.settle(),
                    // Refused — mid-move, typically. Asked again on the next poll.
                    Ok(false) => {}
                    Err(e) => {
                        tracing::error!(error = %e, "sit request failed");
                        return std::process::ExitCode::FAILURE;
                    }
                },
                PadLossAction::GiveUp => {
                    tracing::warn!(
                        "pad gone: the robot would not sit after {PAD_LOST_SIT_TRIES:?} of asking — leaving it as it is"
                    );
                    pad_loss.settle();
                }
            }
            // A hold in flight was measured against the pad that just left: drop it, or a
            // Select still down when the pad returns lands its full hold time at once — a
            // power-off nobody asked for.
            start.reset();
            select.reset();
            imu_reference = None;
            if let Some(tap) = tap.as_ref() {
                tap.idle();
            }
            std::thread::sleep(if pad_loss.pending() {
                PAD_LOST_POLL
            } else {
                IDLE_POLL
            });
            continue;
        };

        if !driving {
            tracing::warn!(pad = pad.name(), "pad connected — driving");
            driving = true;
            // `gilrs` says "No mapping found for UUID" once, without the pad's name or what it costs.
            // A pad it has no layout for still moves its left stick and presses its buttons, and
            // looks like one that is merely unresponsive: the right stick and triggers are the part
            // that goes missing.
            match pad.mapping_source() {
                MappingSource::SdlMappings => {}
                MappingSource::Driver => tracing::info!(
                    pad = pad.name(),
                    guid = %mappings::guid_text(pad.uuid()),
                    "pad has no SDL mapping; using gilrs's default layout"
                ),
                MappingSource::None => tracing::warn!(
                    pad = pad.name(),
                    guid = %mappings::guid_text(pad.uuid()),
                    "pad has no SDL mapping, so its sticks and triggers will mostly not read — \
                     add one with SDL_GAMECONTROLLERCONFIG (docs/robot/pair-a-gamepad.md)"
                ),
            }
            if pad_loss.found() {
                sound(&mut stream, proto::SoundTag::Greet);
            }
        }

        // Every tick rather than on the transition above: a pad that drops and comes back between
        // two ticks never clears `driving`, and it comes back as a different event node often
        // enough that a tap following the old one would report the rest of the session as silence.
        if let Some(tap) = tap.as_ref() {
            tap.watch(&pad);
            // Every tick, like the bindings: switching the feature on in the config has to start
            // the IMU reader without a restart, and off has to let it go.
            tap.imu_control(imu_head_cfg.enabled);
        }

        // The pad's attitude this tick, when the feature is on and the pad has an IMU that has
        // said something believable. `None` is every other case, and head + move then poses the
        // head from the right stick.
        let attitude = if imu_head_cfg.enabled {
            tap.as_ref().and_then(|tap| tap.attitude())
        } else {
            None
        };
        if attitude.is_none() && imu_reference.is_some() {
            // The feature went off, or the IMU went away under us. Not silent: a head that stops
            // following mid-turn wants a line in the journal saying why.
            tracing::info!("IMU head control off — the right stick poses the head");
            imu_reference = None;
        }

        // Start: a tap stands the robot up, then toggles the policy; held, it is the way home.
        let mut go_home = false;
        match start.tick(
            pad.is_pressed(Button::Start),
            start_released,
            tick,
            &[HOME_HOLD],
        ) {
            // Start's one threshold acts when it is reached; its release says nothing more.
            HoldAction::Nothing | HoldAction::ReleasedAfter(_) => {}
            HoldAction::Reached(_) => go_home = true,
            HoldAction::Tap if !up => {
                tracing::warn!("Start — robot.init: standing up. Press Start again to drive");
                match request(&mut stream, &mut next_id, &proto::Call::RobotInit) {
                    Err(e) => {
                        tracing::error!(error = %e, "init failed");
                        return std::process::ExitCode::FAILURE;
                    }
                    Ok(_) => up = true,
                }
            }
            HoldAction::Tap => {
                // The robot owns the toggle. A local on/off belief here drifts from the
                // robot's the moment anything else moves it — robot.relax, the shutdown
                // sequence, either side restarting — and a stale belief turns Start into a
                // button that does nothing every other press. `toggle` flips the robot's own
                // state; turning OFF returns it to the home pose (the prototype's "returning
                // to default pose"), so turning on always starts the policy from home.
                let call = proto::Call::RobotEnable(proto::EnableParams {
                    on: false,
                    toggle: true,
                });
                match request(&mut stream, &mut next_id, &call) {
                    Err(e) => {
                        tracing::error!(error = %e, "enable failed");
                        return std::process::ExitCode::FAILURE;
                    }
                    Ok(response) => {
                        // The robot names the state it ended in; that is the log, since padd
                        // no longer has a belief of its own to report.
                        let outcome = response
                            .and_then(|r| r.result_as::<proto::IntentResult>().ok())
                            .and_then(|r| r.reason)
                            .unwrap_or_else(|| "toggled".to_owned());
                        tracing::warn!(%outcome, "policy");
                    }
                }
            }
        }

        if go_home {
            // Everything back: the sticks to plain driving, the policy off, and the robot ramped
            // to its home pose with torque on — from limp, from mid-walk or from a crouch alike.
            // The disable comes first so the policy cannot pick the robot up again the moment the
            // ramp ends, which is what `robot.init` alone does on a robot that is driving.
            tracing::warn!("Start held — home pose, motors stiff, policy off");
            for call in mode_exit_calls(mode, Mode::Drive) {
                if let Err(e) = notify(&mut stream, &call) {
                    tracing::error!(error = %e, "send failed");
                    return std::process::ExitCode::FAILURE;
                }
            }
            mode = Mode::Drive;
            imu_reference = None;
            let disable = proto::Call::RobotEnable(proto::EnableParams {
                on: false,
                toggle: false,
            });
            for call in [disable, proto::Call::RobotInit] {
                if let Err(e) = request(&mut stream, &mut next_id, &call) {
                    tracing::error!(error = %e, "home request failed");
                    return std::process::ExitCode::FAILURE;
                }
            }
            up = true;
        }

        if let Some(next) = wanted_mode {
            if next != mode {
                for call in mode_exit_calls(mode, next) {
                    if let Err(e) = notify(&mut stream, &call) {
                        tracing::error!(error = %e, "send failed");
                        return std::process::ExitCode::FAILURE;
                    }
                }
                mode = next;
                tracing::info!(?mode, "mode");
            }
            // Head + move follows the pad's IMU when it can, from wherever the pad is at the
            // press — and pressing it again re-centres, which is how a person beats the gyro's
            // yaw drift without a magnetometer.
            imu_reference = if mode == Mode::HeadDrive {
                attitude
            } else {
                None
            };
            if mode == Mode::HeadDrive {
                if imu_reference.is_some() {
                    tracing::info!("IMU head control: following the pad from here");
                } else if imu_head_cfg.enabled && tap.as_ref().is_some_and(|tap| tap.has_imu()) {
                    // The IMU is there and has not spoken yet — a second after connecting,
                    // typically. Saying so beats silently doing the other thing.
                    tracing::warn!(
                        "the pad's IMU has no attitude yet; the right stick has the head this once"
                    );
                }
            }
        }

        // One-shot skills. Answered, because "refused, and here is why" is a real outcome — a
        // skill this robot does not have, one mid-flight, or the policy not driving.
        //
        // The name comes from config and is sent as it was written. `padd` does not check it
        // against anything: which skills exist is the robot's to know, and it answers an unknown
        // one with the list it does have, which is a better error than this side could give.
        for button in &pressed {
            // An empty binding is a button switched off on purpose, not a fault.
            let skill = bindings.skill(button).unwrap_or_default();
            if skill.is_empty() {
                tracing::debug!(button, "no skill bound");
                continue;
            }
            let call = proto::Call::RobotDo(proto::DoParams {
                skill: skill.to_owned(),
            });
            if let Err(e) = request(&mut stream, &mut next_id, &call) {
                tracing::error!(error = %e, "skill request failed");
                return std::process::ExitCode::FAILURE;
            }
        }

        // A request, so a board without a flashlight says so in the journal rather than not at all.
        if flashlight
            && let Err(e) = request(
                &mut stream,
                &mut next_id,
                &proto::Call::RobotFlashlight(proto::FlashlightParams {
                    toggle: true,
                    ..Default::default()
                }),
            )
        {
            tracing::error!(error = %e, "flashlight request failed");
            return std::process::ExitCode::FAILURE;
        }

        // X held: keep a chaining skill going. The robot starts another when a request lands
        // near the end of the current one, so "held" is spelled "resent every tick" — as a
        // notification, because fifty answered requests a second would spend their time waiting
        // on replies, and the press above already got the real answer.
        //
        // Whatever X is bound to, nothing by default: a skill that does not chain simply refuses
        // the resend, which costs a notification nobody reads. Only X, because it is the button
        // the prototype held the roulade on.
        let held = bindings.skill("x").unwrap_or_default();
        if pad.is_pressed(Button::West)
            && !pressed.contains(&"x")
            && !held.is_empty()
            && let Err(e) = notify(
                &mut stream,
                &proto::Call::RobotDo(proto::DoParams {
                    skill: held.to_owned(),
                }),
            )
        {
            tracing::error!(error = %e, "send failed");
            return std::process::ExitCode::FAILURE;
        }

        // Select: nothing on a tap. Let go between two and four seconds, a rest; held to four, a
        // power-off. Both sit and ease into the rest pose before torque goes, which is why the
        // rest waits for the release: until then the same hold may still become a power-off.
        match select.tick(
            pad.is_pressed(Button::Select),
            select_released,
            tick,
            &[REST_HOLD, SHUTDOWN_HOLD],
        ) {
            HoldAction::ReleasedAfter(0) => {
                tracing::warn!(
                    "Select released — robot.rest: sit, rest pose, then torque off and servo reboot"
                );
                // A reboot at the end rather than a bare relax: a servo that tripped its overload
                // comes back with it, so this is also the way out of a tripped servo without
                // pulling the battery. The robot ends limp, so the next Start stands it up again
                // rather than toggling the policy on a robot that is lying on the floor.
                up = false;
                if let Err(e) = request(&mut stream, &mut next_id, &proto::Call::RobotRest) {
                    tracing::error!(error = %e, "rest request failed");
                    return std::process::ExitCode::FAILURE;
                }
            }
            HoldAction::Nothing | HoldAction::ReleasedAfter(_) => {}
            HoldAction::Tap => {
                tracing::info!("Select tapped — hold it 2 s to rest, 4 s to power off")
            }
            HoldAction::Reached(0) => {
                tracing::warn!("Select held 2 s — let go to rest, keep holding to power off")
            }
            HoldAction::Reached(_) => {
                up = false;
                tracing::warn!("Select held on — asking the robot to power off");
                if let Err(e) = request(&mut stream, &mut next_id, &proto::Call::RobotShutdown) {
                    tracing::error!(error = %e, "shutdown request failed");
                    return std::process::ExitCode::FAILURE;
                }
            }
        }

        let deadzone = |v: f32| {
            let v = v as f64;
            if v.abs() < args.deadzone { 0.0 } else { v }
        };
        let left_x = deadzone(pad.value(Axis::LeftStickX));
        let left_y = deadzone(pad.value(Axis::LeftStickY));
        let right_x = deadzone(pad.value(Axis::RightStickX));
        let right_y = deadzone(pad.value(Axis::RightStickY));

        // Either trigger opens the mouth; the max wins, as in the prototype — where RT
        // also chirps and LT rides the wheee, which they now do here too.
        let trigger = |b: Button| pad.button_data(b).map(|d| d.value()).unwrap_or(0.0) as f64;
        let rt = trigger(Button::RightTrigger2);
        let lt = trigger(Button::LeftTrigger2);
        let mouth = rt.max(lt);
        if let Err(e) = notify(
            &mut stream,
            &proto::Call::RobotMouth(proto::MouthParams { open: mouth }),
        ) {
            tracing::error!(error = %e, "send failed");
            return std::process::ExitCode::FAILURE;
        }

        // Chirp on the right trigger's rising edge; the robot cuts off a still-playing
        // sound, so rapid pulses quack rapidly. The wheee rides the left trigger: start on
        // press, then a hold notification per tick — the robot treats the hold as a level
        // that decays, so a padd that dies mid-ride leaves a ride that lands. Release cuts
        // it instantly, as the prototype does.
        const SOUND_THRESHOLD: f64 = 0.3;
        let mut sound_calls: Vec<proto::SoundParams> = Vec::new();
        if prev_rt < SOUND_THRESHOLD && rt >= SOUND_THRESHOLD {
            sound_calls.push(proto::SoundParams {
                tag: proto::SoundTag::Chirp,
                hold: None,
            });
        }
        if lt >= SOUND_THRESHOLD {
            sound_calls.push(proto::SoundParams {
                tag: proto::SoundTag::Wheee,
                hold: Some(true),
            });
        } else if prev_lt >= SOUND_THRESHOLD {
            sound_calls.push(proto::SoundParams {
                tag: proto::SoundTag::Wheee,
                hold: Some(false),
            });
        }
        prev_rt = rt;
        prev_lt = lt;
        for params in sound_calls {
            if let Err(e) = notify(&mut stream, &proto::Call::RobotSound(params)) {
                tracing::error!(error = %e, "send failed");
                return std::process::ExitCode::FAILURE;
            }
        }

        let limits = DriveLimits {
            roller,
            drive: &drive,
        };

        // This tick's continuous intents, as one frame. Reused rather than built fresh:
        // a `Vec` per tick is an allocation fifty times a second to say what the sticks
        // were doing, which is the shape of thing this loop is meant not to do.
        frame.clear();
        match mode {
            Mode::Drive => frame.push(proto::Call::RobotMove(limits.walk(left_y, left_x, right_x))),
            Mode::HeadDrive => match (imu_reference, attitude) {
                // The pad's tilt has the head, so the sticks keep the whole drive mapping. In the
                // same frame: the pad's tilt and the sticks describe one instant.
                (Some(reference), Some(now)) => {
                    frame.push(proto::Call::RobotMove(limits.walk(left_y, left_x, right_x)));
                    frame.push(proto::Call::RobotHead(head_from_pad(
                        pad_imu::relative(reference, now),
                        imu_head_cfg.gain,
                        args.max_head,
                    )));
                }
                // Left stick walks and turns — no strafe, the right stick is busy — and the right
                // stick looks around, with the signs head mode uses for its left stick.
                _ => {
                    frame.push(proto::Call::RobotMove(limits.walk(left_y, 0.0, left_x)));
                    frame.push(proto::Call::RobotHead(proto::HeadParams {
                        neck_pitch: 0.0,
                        head_pitch: -right_y * args.max_head,
                        head_yaw: -right_x * args.max_head,
                        head_roll: 0.0,
                    }));
                }
            },
            Mode::Head => {
                // The body must not keep its last velocity while the sticks are posing the
                // head. The deadman would catch it eventually; a robot that keeps walking
                // because you started moving its head is a bad enough surprise to be
                // explicit about.
                //
                // In the same frame as the head rather than a notification of its own: the
                // two describe one instant, and sending them separately was two `write_all`
                // and two `flush` syscalls a tick to say so.
                frame.push(proto::Call::RobotMove(proto::MoveParams::default()));
                // The prototype's alpha mapping, signs included (its head_pitch/head_yaw
                // joint axes are inverted relative to stick direction — verified on
                // hardware there, kept verbatim here).
                frame.push(proto::Call::RobotHead(proto::HeadParams {
                    neck_pitch: right_y * args.max_head,
                    head_pitch: -left_y * args.max_head,
                    head_yaw: -left_x * args.max_head,
                    head_roll: right_x * args.max_head,
                }));
            }
            Mode::BodyPose => {
                frame.push(proto::Call::RobotMove(proto::MoveParams::default()));
                frame.push(proto::Call::RobotPose(proto::PoseParams {
                    z: left_y
                        * if left_y >= 0.0 {
                            BODY_MAX_Z_UP
                        } else {
                            BODY_MAX_Z_DOWN
                        },
                    // No forward/back tilt here: the right stick has the head. The side lean
                    // keeps the old body mode's sign, moved from the right stick to the left.
                    pitch: 0.0,
                    roll: left_x * BODY_MAX_ANGLE,
                    active: true,
                }));
                // The same look-around as head + move, so the right stick means one thing
                // wherever it poses the head.
                frame.push(proto::Call::RobotHead(proto::HeadParams {
                    neck_pitch: 0.0,
                    head_pitch: -right_y * args.max_head,
                    head_yaw: -right_x * args.max_head,
                    head_roll: 0.0,
                }));
            }
        }

        if let Err(e) = continuous.send(&mut stream, &frame, tick) {
            tracing::error!(error = %e, "send failed");
            return std::process::ExitCode::FAILURE;
        }

        if let Some(remaining) = period.checked_sub(tick.elapsed()) {
            std::thread::sleep(remaining);
        }
    }
}

/// How full deflection maps to velocity, for the modes that walk.
#[derive(Debug, Clone, Copy)]
struct DriveLimits<'a> {
    roller: bool,
    /// `[pad_drive]`: each direction of each axis onto its own signed bound.
    drive: &'a robotd_params::PadDriveParams,
}

impl DriveLimits<'_> {
    /// A velocity from three stick axes: forward/back, strafe and turn. Which physical axis is
    /// which depends on the mode; the shaping does not.
    fn walk(&self, forward: f64, strafe: f64, turn: f64) -> proto::MoveParams {
        if self.roller {
            // The prototype's roller shaping: push harder than you can brake, no strafe,
            // heading capped independently of the walking limits.
            return proto::MoveParams {
                vx: forward
                    * if forward >= 0.0 {
                        ROLLER_PUSH
                    } else {
                        ROLLER_BRAKE
                    },
                vy: 0.0,
                vyaw: -turn * ROLLER_YAW,
            };
        }
        let scale = robotd_params::PadDriveParams::scale;
        let d = self.drive;
        proto::MoveParams {
            vx: scale(forward, d.vx_min, d.vx_max),
            // `vy` is positive to the left; stick-left reads negative on every pad gilrs
            // normalises.
            vy: scale(-strafe, d.vy_min, d.vy_max),
            vyaw: scale(-turn, d.vyaw_min, d.vyaw_max),
        }
    }
}

/// Whether this robot is on wheels, by asking it. `None` for an answer that did not say.
fn ask_roller(stream: &mut UnixStream, next_id: &mut u64) -> std::io::Result<Option<bool>> {
    let answer = request(stream, next_id, &proto::Call::RobotMode)?;
    Ok(answer
        .and_then(|answer| answer.result_as::<proto::ModeResult>().ok())
        .map(|mode| mode.mode == "roller"))
}

/// Play one of the robot's sounds, best effort: a cue, not something worth stopping for.
fn sound(stream: &mut UnixStream, tag: proto::SoundTag) {
    let call = proto::Call::RobotSound(proto::SoundParams { tag, hold: None });
    if let Err(e) = notify(stream, &call) {
        tracing::debug!(error = %e, "sound not sent");
    }
}

/// Sit the robot down unless it is sitting already. `Ok(true)` when there is nothing left to do —
/// sitting, sat down, or a policy that is not driving and so has nothing to sit with (the robot
/// is then standing still at home, stiff, which is where a lost pad should leave it anyway).
/// `Ok(false)` when the robot refused for a reason that passes, mid-move typically.
///
/// Asked rather than toggled blind: `sit_toggle` stands a seated robot up, and a pad dropping
/// out next to a sitting robot must not do that.
fn sit_if_standing(stream: &mut UnixStream, next_id: &mut u64) -> std::io::Result<bool> {
    let sitting = request(stream, next_id, &proto::Call::RobotPolicies)?
        .and_then(|answer| answer.result_as::<proto::PoliciesResult>().ok())
        .and_then(|policies| policies.sitting);
    if sitting == Some(true) {
        tracing::info!("pad gone: the robot is already sitting");
        return Ok(true);
    }
    let call = proto::Call::RobotDo(proto::DoParams {
        skill: "sit_toggle".to_owned(),
    });
    let result = request(stream, next_id, &call)?
        .and_then(|answer| answer.result_as::<proto::IntentResult>().ok());
    match result {
        Some(result) if result.accepted => {
            tracing::warn!("pad gone: sitting the robot down");
            Ok(true)
        }
        // Not driving: nothing will sit it, and nothing needs to.
        Some(result)
            if result
                .reason
                .as_deref()
                .is_some_and(|r| r.contains("not driving")) =>
        {
            Ok(true)
        }
        _ => Ok(false),
    }
}

/// Send a continuous intent: no `id`, no reply, nothing to wait for.
fn notify(stream: &mut UnixStream, call: &proto::Call) -> std::io::Result<()> {
    let mut line = serde_json::to_vec(&proto::Request::notify(call))?;
    line.push(b'\n');
    stream.write_all(&line)?;
    stream.flush()
}

/// How long an unchanged frame may go unsent while the robot is being asked to stand still.
///
/// The sticks are *polled*, so an untouched pad re-sent the same three zeros fifty times a
/// second — a `serde_json` encode, a `write_all`, a `flush`, and a `serde_json` parse on
/// `robotd`'s side, a hundred messages a second in the modes that send two, to say nothing
/// changed. Ten a second says it as well.
///
/// **Nothing about the robot's safety rests on this number**, which is why it can be picked
/// for legibility rather than argued against `[safety] deadman_ms` — a value this daemon
/// cannot read and does not know. A frame is only ever held back when it is byte-identical
/// to the last one sent *and* asks for no motion ([`Continuous::may_hold`]); a stick that is
/// doing anything goes out on every tick as it always did. What the heartbeat buys is the
/// report: without it `robotd`'s twist would age past the deadman while a pad sat connected
/// and idle, and `robot.state` would carry `limited_by: ["deadman"]` for a robot that is
/// stationary because it was asked to be.
const HEARTBEAT: Duration = Duration::from_millis(100);

/// The continuous intents, encoded once a tick and sent when they say something new.
#[derive(Default)]
struct Continuous {
    /// This tick's frame. Kept across ticks so the encode reuses its buffer.
    line: Vec<u8>,
    /// The bytes last put on the socket, to compare this tick's against.
    last: Vec<u8>,
    /// When that was. `None` until the first send, which therefore always happens.
    at: Option<Instant>,
}

impl Continuous {
    /// Put this tick's intents on the socket, unless they are the ones already there.
    ///
    /// One write for the whole frame: the calls describe a single instant, and a peer that
    /// read half of one would be acting on a head pose without the velocity that came with
    /// it. `robotd` splits the buffer back into lines on its own read.
    fn send(
        &mut self,
        stream: &mut UnixStream,
        calls: &[proto::Call],
        now: Instant,
    ) -> std::io::Result<()> {
        self.line.clear();
        for call in calls {
            serde_json::to_writer(&mut self.line, &proto::Request::notify(call))?;
            self.line.push(b'\n');
        }

        if self.line == self.last && self.may_hold(calls, now) {
            return Ok(());
        }

        stream.write_all(&self.line)?;
        stream.flush()?;
        // Swapped rather than cloned: the buffer this displaces becomes next tick's
        // scratch, so a steady state allocates nothing at all.
        std::mem::swap(&mut self.last, &mut self.line);
        self.at = Some(now);
        Ok(())
    }

    /// Whether an unchanged frame may be left unsent this tick.
    ///
    /// Only while it asks for no velocity. The deadman zeroes the twist and nothing else, so
    /// on a frame that already commands zero, letting it fire changes nothing about what the
    /// robot does — and on a frame that commands motion it would stop a robot whose stick is
    /// still held. That is the whole of the argument, and it holds whatever `deadman_ms` is
    /// set to.
    ///
    /// A held stick therefore keeps sending at the full rate. That is the case where the
    /// robot is walking and the daemon has something to say; this is about the one where it
    /// is not and does not.
    fn may_hold(&self, calls: &[proto::Call], now: Instant) -> bool {
        let asks_for_motion = calls.iter().any(|call| match call {
            proto::Call::RobotMove(p) => p.vx != 0.0 || p.vy != 0.0 || p.vyaw != 0.0,
            _ => false,
        });
        !asks_for_motion && self.at.is_some_and(|at| now.duration_since(at) < HEARTBEAT)
    }
}

/// Send a discrete intent and read its answer.
///
/// Answered, unlike the continuous ones, because "refused, and here is why" is a real
/// outcome — a skill with no policy loaded, a sound with no bank — and a client that
/// ignored it would leave the operator wondering why nothing happened.
fn request(
    stream: &mut UnixStream,
    next_id: &mut u64,
    call: &proto::Call,
) -> std::io::Result<Option<proto::Response>> {
    let id = proto::Id::Number(*next_id);
    *next_id += 1;
    let mut line = serde_json::to_vec(&proto::Request::call(id, call))?;
    line.push(b'\n');
    stream.write_all(&line)?;
    stream.flush()?;

    // One line per request, in order, on a connection nothing else uses.
    let mut reader = BufReader::new(stream.try_clone()?);
    let mut answer = String::new();
    reader.read_line(&mut answer)?;

    match serde_json::from_str::<proto::Response>(&answer) {
        Ok(response) => {
            if let Some(error) = &response.error {
                tracing::warn!(code = error.code, message = %error.message, "refused");
            } else if let Ok(result) = response.result_as::<proto::IntentResult>()
                && !result.accepted
            {
                tracing::warn!(reason = ?result.reason, "not accepted");
            }
            Ok(Some(response))
        }
        Err(e) => {
            tracing::warn!(error = %e, raw = %answer.trim(), "unparsable answer");
            Ok(None)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Read;

    /// A socket pair standing in for `robotd`, and what came out of it.
    ///
    /// Non-blocking on the reading end so a test can assert that *nothing* was sent, which
    /// is the assertion most of these are making.
    fn socket() -> (UnixStream, UnixStream) {
        let (ours, theirs) = UnixStream::pair().expect("a socket pair");
        theirs.set_nonblocking(true).expect("non-blocking");
        (ours, theirs)
    }

    fn drain(stream: &mut UnixStream) -> String {
        let mut buffer = Vec::new();
        let mut chunk = [0u8; 4096];
        while let Ok(n) = stream.read(&mut chunk) {
            if n == 0 {
                break;
            }
            buffer.extend_from_slice(&chunk[..n]);
        }
        String::from_utf8(buffer).expect("utf-8")
    }

    fn moving() -> Vec<proto::Call> {
        vec![proto::Call::RobotMove(proto::MoveParams {
            vx: 0.2,
            vy: 0.0,
            vyaw: 0.0,
        })]
    }

    fn still() -> Vec<proto::Call> {
        vec![proto::Call::RobotMove(proto::MoveParams::default())]
    }

    /// The sticks are polled, so an untouched pad produces the same frame fifty times a
    /// second. Sending it fifty times is what this stops.
    #[test]
    fn an_unchanged_stationary_frame_is_not_resent() {
        let (mut ours, mut theirs) = socket();
        let mut continuous = Continuous::default();
        let at = Instant::now();

        continuous.send(&mut ours, &still(), at).expect("first");
        let first = drain(&mut theirs);
        assert!(first.contains("robot.move"), "the first frame must go out");

        for tick in 1..5 {
            let now = at + Duration::from_millis(20 * tick);
            continuous.send(&mut ours, &still(), now).expect("held");
        }
        assert_eq!(drain(&mut theirs), "", "an idle pad must say nothing");
    }

    /// **The safety property, and the reason the heartbeat needs no argument about
    /// `deadman_ms`.** A stick that is asking the robot to walk is re-sent every tick
    /// whatever the clock says, because a deadman that fires on a held stick stops a robot
    /// somebody is driving.
    #[test]
    fn a_frame_that_asks_for_motion_is_always_sent() {
        let (mut ours, mut theirs) = socket();
        let mut continuous = Continuous::default();
        let at = Instant::now();

        continuous.send(&mut ours, &moving(), at).expect("first");
        drain(&mut theirs);

        // The same bytes, one tick later — well inside the heartbeat.
        continuous
            .send(&mut ours, &moving(), at + Duration::from_millis(20))
            .expect("second");
        assert!(
            drain(&mut theirs).contains("robot.move"),
            "a held stick must keep being sent"
        );
    }

    /// The heartbeat is what keeps `robotd` from reporting a deadman on a robot that is
    /// standing still because it was asked to.
    #[test]
    fn a_stationary_frame_goes_out_again_on_the_heartbeat() {
        let (mut ours, mut theirs) = socket();
        let mut continuous = Continuous::default();
        let at = Instant::now();

        continuous.send(&mut ours, &still(), at).expect("first");
        drain(&mut theirs);

        continuous
            .send(
                &mut ours,
                &still(),
                at + HEARTBEAT - Duration::from_millis(1),
            )
            .expect("held");
        assert_eq!(drain(&mut theirs), "", "not due yet");

        continuous
            .send(&mut ours, &still(), at + HEARTBEAT)
            .expect("heartbeat");
        assert!(drain(&mut theirs).contains("robot.move"), "due");
    }

    /// A stick that moves is heard on the tick it moved, not on the next heartbeat.
    #[test]
    fn a_changed_frame_is_sent_at_once() {
        let (mut ours, mut theirs) = socket();
        let mut continuous = Continuous::default();
        let at = Instant::now();

        continuous.send(&mut ours, &still(), at).expect("first");
        drain(&mut theirs);

        continuous
            .send(&mut ours, &moving(), at + Duration::from_millis(20))
            .expect("changed");
        assert!(
            drain(&mut theirs).contains("robot.move"),
            "a stick that moved must not wait for a heartbeat"
        );
    }

    /// Head mode's two intents describe one instant and go out in one write. Split across
    /// two, a reader could act on a head pose without the velocity that came with it.
    #[test]
    fn a_two_call_frame_is_one_write_and_two_lines() {
        let (mut ours, mut theirs) = socket();
        let mut continuous = Continuous::default();
        let calls = vec![
            proto::Call::RobotMove(proto::MoveParams::default()),
            proto::Call::RobotHead(proto::HeadParams {
                neck_pitch: 0.1,
                head_pitch: 0.0,
                head_yaw: 0.0,
                head_roll: 0.0,
            }),
        ];

        continuous
            .send(&mut ours, &calls, Instant::now())
            .expect("sent");

        let sent = drain(&mut theirs);
        let lines: Vec<&str> = sent.lines().collect();
        assert_eq!(lines.len(), 2, "two intents, two lines: {sent:?}");
        assert!(lines[0].contains("robot.move"));
        assert!(lines[1].contains("robot.head"));
        assert!(sent.ends_with('\n'), "every line must be terminated");
    }

    /// A head frame carries a zero velocity, so an untouched pad in head mode holds too —
    /// which is the mode that was sending a hundred messages a second.
    #[test]
    fn an_untouched_pad_in_head_mode_holds_both_intents() {
        let (mut ours, mut theirs) = socket();
        let mut continuous = Continuous::default();
        let at = Instant::now();
        let calls = vec![
            proto::Call::RobotMove(proto::MoveParams::default()),
            proto::Call::RobotHead(proto::HeadParams::default()),
        ];

        continuous.send(&mut ours, &calls, at).expect("first");
        drain(&mut theirs);

        continuous
            .send(&mut ours, &calls, at + Duration::from_millis(20))
            .expect("held");
        assert_eq!(drain(&mut theirs), "");
    }

    /// The bug this catches: `--hz 0` used to reach `Duration::from_secs_f64(1.0 / 0.0)`, and
    /// that panics on infinity rather than giving a very long period. So a typo killed the
    /// daemon at startup with a panic instead of saying which flag was wrong.
    ///
    /// The ceiling is the same line's other half, and the worse failure of the two: a period
    /// that rounds to 0 ns leaves `checked_sub` nothing to sleep on, so the loop stops being
    /// paced and spins on robotd's socket. A panic is at least loud.
    #[test]
    fn a_rate_this_loop_cannot_run_at_is_refused_rather_than_divided_by() {
        assert!(Args::try_parse_from(["padd", "--hz", "0"]).is_err());
        assert!(Args::try_parse_from(["padd", "--hz", "1"]).is_ok());
        assert!(Args::try_parse_from(["padd", "--hz", "1000"]).is_ok());
        assert!(Args::try_parse_from(["padd", "--hz", "1001"]).is_err());
        assert!(
            Args::try_parse_from(["padd", "--hz", "4294967295"]).is_err(),
            "the top of a u32 is a 0 ns period, which is a spin loop"
        );
        assert!(
            Args::try_parse_from(["padd"]).is_ok(),
            "the default still parses"
        );
    }

    /// Head + move with an IMU pad re-centres on every press: the reference is the pad's attitude
    /// at the press, so a re-press after a minute of drift starts the head at centre again.
    #[test]
    fn the_pad_attitude_at_the_press_reads_as_centre() {
        // Yawed 30° by the time of the press — drift, or a turn; the pad cannot tell.
        let yawed = [
            (15.0f32.to_radians()).cos(),
            0.0,
            0.0,
            (15.0f32.to_radians()).sin(),
        ];
        let head = head_from_pad(pad_imu::relative(yawed, yawed), 1.0, 2.5);
        assert!(
            head.head_yaw.abs() < 1e-4 && head.head_pitch.abs() < 1e-4,
            "{head:?}"
        );
    }

    /// The pad's tilt becomes the head's pose with the signs verified on the robot: nose up is a
    /// positive head_pitch, yaw left a positive head_yaw, rolled right a negative head_roll. Gain
    /// scales, the travel limit clamps, the neck stays put.
    #[test]
    fn the_head_follows_the_pad_with_the_sticks_signs_gain_and_limit() {
        let half = 15.0f32.to_radians();
        // 30° nose up: a rotation about +Y.
        let nose_up = [half.cos(), 0.0, half.sin(), 0.0];
        let head = head_from_pad(nose_up, 1.0, 2.5);
        assert!(
            (head.head_pitch - 30.0f64.to_radians()).abs() < 0.01,
            "{head:?}"
        );
        assert!(
            head.head_yaw.abs() < 0.01 && head.head_roll.abs() < 0.01,
            "{head:?}"
        );
        assert_eq!(head.neck_pitch, 0.0);

        // 30° rolled right (left side up): about +X. The head rolls the other sign.
        let rolled = [half.cos(), half.sin(), 0.0, 0.0];
        let head = head_from_pad(rolled, 1.0, 2.5);
        assert!(
            (head.head_roll - (-30.0f64.to_radians())).abs() < 0.01,
            "{head:?}"
        );

        // 30° yaw left: about +Z.
        let left = [half.cos(), 0.0, 0.0, half.sin()];
        let head = head_from_pad(left, 1.0, 2.5);
        assert!(
            (head.head_yaw - 30.0f64.to_radians()).abs() < 0.01,
            "{head:?}"
        );

        // Gain 2 doubles it; a limit of 0.5 rad clamps it.
        let head = head_from_pad(left, 2.0, 2.5);
        assert!(
            (head.head_yaw - 60.0f64.to_radians()).abs() < 0.01,
            "{head:?}"
        );
        let head = head_from_pad(left, 2.0, 0.5);
        assert!((head.head_yaw - 0.5).abs() < 1e-6, "{head:?}");
    }

    const SELECT: [Duration; 2] = [REST_HOLD, SHUTDOWN_HOLD];
    const START: [Duration; 1] = [HOME_HOLD];

    /// Select: a tap does nothing to the robot. Let go between two and four seconds, a rest —
    /// decided at the release, since the hold could still have become a power-off. Held to four,
    /// the power-off goes out at four, and its release adds nothing.
    #[test]
    fn select_rests_on_a_release_between_two_and_four_and_powers_off_at_four() {
        let t0 = Instant::now();
        let at = |ms: u64| t0 + Duration::from_millis(ms);
        let mut select = HoldButton::default();
        let mut tick = |pressed, released, ms| select.tick(pressed, released, at(ms), &SELECT);

        // A tap: reported, so the log can say what a hold would do.
        assert_eq!(tick(true, false, 0), HoldAction::Nothing);
        assert_eq!(tick(false, true, 300), HoldAction::Tap);

        // Let go at 1.9 s: still a tap.
        assert_eq!(tick(true, false, 1_000), HoldAction::Nothing);
        assert_eq!(tick(false, true, 2_900), HoldAction::Tap);

        // Let go at 3 s: two seconds reached, four not — a rest, at the release only.
        assert_eq!(tick(true, false, 10_000), HoldAction::Nothing);
        assert_eq!(tick(true, false, 12_000), HoldAction::Reached(0));
        assert_eq!(tick(true, false, 12_500), HoldAction::Nothing);
        assert_eq!(tick(false, true, 13_000), HoldAction::ReleasedAfter(0));
        assert_eq!(tick(false, false, 13_020), HoldAction::Nothing);

        // Held to four: the power-off at four, once, and a release that is not a rest.
        assert_eq!(tick(true, false, 20_000), HoldAction::Nothing);
        assert_eq!(tick(true, false, 22_000), HoldAction::Reached(0));
        assert_eq!(tick(true, false, 24_000), HoldAction::Reached(1));
        assert_eq!(tick(true, false, 24_020), HoldAction::Nothing);
        assert_eq!(tick(false, true, 25_000), HoldAction::ReleasedAfter(1));
    }

    /// Start: a tap is the stand-up / policy toggle, a 1.5 s hold is the way home — and a hold
    /// is never also a tap, or going home would toggle the policy straight back on.
    #[test]
    fn start_taps_toggle_and_a_long_hold_goes_home_only() {
        let t0 = Instant::now();
        let at = |ms: u64| t0 + Duration::from_millis(ms);
        let mut start = HoldButton::default();

        assert_eq!(start.tick(true, false, at(0), &START), HoldAction::Nothing);
        assert_eq!(start.tick(false, true, at(200), &START), HoldAction::Tap);

        // Press and release inside one tick: the state never read as down, the edge did.
        assert_eq!(start.tick(false, true, at(1_000), &START), HoldAction::Tap);

        assert_eq!(
            start.tick(true, false, at(2_000), &START),
            HoldAction::Nothing
        );
        assert_eq!(
            start.tick(true, false, at(3_490), &START),
            HoldAction::Nothing
        );
        assert_eq!(
            start.tick(true, false, at(3_500), &START),
            HoldAction::Reached(0)
        );
        assert_eq!(
            start.tick(true, false, at(9_000), &START),
            HoldAction::Nothing
        );
        assert_eq!(
            start.tick(false, true, at(9_020), &START),
            HoldAction::ReleasedAfter(0),
            "reported, and Start acts on nothing at its release"
        );
    }

    /// Select held when the pad drops, back three seconds later with Select still down: that
    /// is a reconnection, not a long hold — the hold was measured against the pad that left.
    /// Pinned both ways, because the `tick` arithmetic alone would call it a torque cut.
    #[test]
    fn a_hold_does_not_survive_the_pad_going_away() {
        let t0 = Instant::now();
        let mut select = HoldButton::default();
        assert_eq!(select.tick(true, false, t0, &SELECT), HoldAction::Nothing);

        select.reset();
        assert_eq!(
            select.tick(
                true,
                false,
                t0 + REST_HOLD + Duration::from_secs(1),
                &SELECT
            ),
            HoldAction::Nothing,
            "a hold older than the pad's absence does nothing"
        );

        // Without the reset that same tick is the torque cut — the arithmetic is why the
        // reset exists.
        let mut stale = HoldButton::default();
        assert_eq!(stale.tick(true, false, t0, &SELECT), HoldAction::Nothing);
        assert_eq!(
            stale.tick(
                true,
                false,
                t0 + REST_HOLD + Duration::from_secs(1),
                &SELECT
            ),
            HoldAction::Reached(0)
        );
    }

    /// A pad dropping out after the torque cut does not let the rest of that hold power the
    /// robot off from a fresh start, and its release is not a tap either.
    #[test]
    fn a_pad_dropout_after_a_threshold_spends_the_rest_of_the_hold() {
        let t0 = Instant::now();
        let mut select = HoldButton::default();
        assert_eq!(select.tick(true, false, t0, &SELECT), HoldAction::Nothing);
        assert_eq!(
            select.tick(true, false, t0 + REST_HOLD, &SELECT),
            HoldAction::Reached(0)
        );

        // Pad gone, pad back with Select still down for longer than the whole sequence.
        select.reset();
        let back = t0 + REST_HOLD + Duration::from_secs(3);
        assert_eq!(select.tick(true, false, back, &SELECT), HoldAction::Nothing);
        assert_eq!(
            select.tick(true, false, back + SHUTDOWN_HOLD, &SELECT),
            HoldAction::Nothing,
            "the tail of a spent hold powers nothing off"
        );
        assert_eq!(
            select.tick(false, true, back + SHUTDOWN_HOLD, &SELECT),
            HoldAction::Nothing,
            "and its release is not a tap"
        );

        // Once that release has been seen, Select is an ordinary hold again.
        let t1 = back + Duration::from_secs(10);
        assert_eq!(select.tick(true, false, t1, &SELECT), HoldAction::Nothing);
        assert_eq!(
            select.tick(true, false, t1 + REST_HOLD, &SELECT),
            HoldAction::Reached(0)
        );
    }

    /// Leaving a mode puts back what it moved, and only that: body + head releases the body,
    /// leaving every head-posing mode for plain driving re-centres the head, and moving between
    /// head-posing modes keeps it.
    #[test]
    fn leaving_a_mode_puts_back_what_it_moved() {
        let is_pose_off = |c: &proto::Call| matches!(c, proto::Call::RobotPose(p) if !p.active);
        let centre = proto::HeadParams::default();
        let is_head_centre =
            |c: &proto::Call| matches!(c, proto::Call::RobotHead(h) if *h == centre);

        let calls = mode_exit_calls(Mode::BodyPose, Mode::Drive);
        assert!(
            calls.len() == 2 && is_pose_off(&calls[0]) && is_head_centre(&calls[1]),
            "{calls:?}"
        );
        for to in [Mode::Head, Mode::HeadDrive] {
            let calls = mode_exit_calls(Mode::BodyPose, to);
            assert!(
                calls.len() == 1 && is_pose_off(&calls[0]),
                "→ {to:?}: {calls:?}"
            );
        }

        for head in [Mode::Head, Mode::HeadDrive] {
            let calls = mode_exit_calls(head, Mode::Drive);
            assert!(
                calls.len() == 1 && is_head_centre(&calls[0]),
                "{head:?}: {calls:?}"
            );
            assert!(mode_exit_calls(head, Mode::BodyPose).is_empty());
        }

        assert!(mode_exit_calls(Mode::Head, Mode::HeadDrive).is_empty());
        assert!(mode_exit_calls(Mode::HeadDrive, Mode::Head).is_empty());
        assert!(mode_exit_calls(Mode::Drive, Mode::Head).is_empty());
        assert!(mode_exit_calls(Mode::Drive, Mode::BodyPose).is_empty());
        assert!(mode_exit_calls(Mode::BodyPose, Mode::BodyPose).is_empty());
    }

    /// Head + move drives with the left stick alone — forward and turn, no strafe — and on wheels
    /// it takes the roller shaping like every other walking mode.
    #[test]
    fn head_and_move_walks_and_turns_from_the_left_stick() {
        let drive = robotd_params::PadDriveParams {
            vx_min: -0.2,
            ..Default::default()
        };
        let walking = DriveLimits {
            roller: false,
            drive: &drive,
        };
        // Left stick up and to the left: forward, turning left.
        let twist = walking.walk(1.0, 0.0, -1.0);
        assert_eq!(twist.vx, 0.3);
        assert_eq!(twist.vy, 0.0);
        assert_eq!(twist.vyaw, 1.5, "stick left turns left (positive yaw)");
        assert_eq!(
            walking.walk(-1.0, 0.0, 0.0).vx,
            -0.2,
            "reverse has its own cap"
        );

        let rolling = DriveLimits {
            roller: true,
            ..walking
        };
        let twist = rolling.walk(-1.0, 1.0, -1.0);
        assert_eq!(twist.vx, -ROLLER_BRAKE);
        assert_eq!(twist.vy, 0.0, "no strafe on wheels");
        assert_eq!(twist.vyaw, ROLLER_YAW);
    }

    /// A pad lost while driving: nothing for a second, then sit — asked again until the robot
    /// takes it, for a few seconds, then given up on. Settled, it asks nothing more.
    #[test]
    fn a_lost_pad_sits_the_robot_after_a_second() {
        let t0 = Instant::now();
        let at = |ms: u64| t0 + Duration::from_millis(ms);
        let mut loss = PadLoss::default();
        assert_eq!(loss.tick(at(5_000)), PadLossAction::Nothing, "never lost");

        loss.lost(t0);
        assert!(loss.pending());
        assert_eq!(loss.tick(at(900)), PadLossAction::Nothing);
        assert_eq!(loss.tick(at(1_000)), PadLossAction::Sit);
        assert_eq!(
            loss.tick(at(3_000)),
            PadLossAction::Sit,
            "refused: ask again"
        );
        assert_eq!(loss.tick(at(6_000)), PadLossAction::GiveUp);

        loss.settle();
        assert!(!loss.pending());
        assert_eq!(loss.tick(at(9_000)), PadLossAction::Nothing);
    }

    /// The pad coming back is greeted only when it had been lost, and stops any sit still due.
    #[test]
    fn a_pad_coming_back_is_greeted_once() {
        let t0 = Instant::now();
        let mut loss = PadLoss::default();
        assert!(!loss.found(), "a pad that was never lost is not a return");

        loss.lost(t0);
        assert!(loss.found(), "back before the sit");
        assert_eq!(
            loss.tick(t0 + Duration::from_secs(2)),
            PadLossAction::Nothing,
            "and no sit after it is back"
        );

        loss.lost(t0);
        loss.settle();
        assert!(loss.found(), "back after the sit");
        assert!(!loss.found(), "once");
    }
}
