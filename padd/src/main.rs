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
//! Waiting with no pad is deliberately cheap and deliberately silent — nothing is sent, and
//! `robotd`'s deadman holds the robot on its own. Inventing a zero command instead would mask a
//! disconnected pad as someone's decision to stop.
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
use gilrs::{Axis, Button, Gilrs};
use pad_map::{BINDINGS_POLL, Buttons, Config, Continuous, Mapper, Out, PadFrame, read_bindings};

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

    /// Exit 0 if this daemon should run — `[netpad] enabled` is off — and 1 if `netpadd` has the
    /// pad. For `ExecCondition=`; nothing else is started.
    #[arg(long)]
    should_run: bool,

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

/// The gilrs name for each wire bit. gilrs calls the *bumpers* `LeftTrigger`/`RightTrigger` and
/// the analogue triggers `LeftTrigger2`/`RightTrigger2` — getting that backwards binds a skill to
/// a control nobody presses.
const GILRS: [(Button, Buttons); 12] = [
    (Button::South, Buttons::A),
    (Button::East, Buttons::B),
    (Button::West, Buttons::X),
    (Button::North, Buttons::Y),
    (Button::LeftTrigger, Buttons::LB),
    (Button::RightTrigger, Buttons::RB),
    (Button::Start, Buttons::START),
    (Button::Select, Buttons::SELECT),
    (Button::DPadUp, Buttons::UP),
    (Button::DPadDown, Buttons::DOWN),
    (Button::DPadLeft, Buttons::LEFT),
    (Button::DPadRight, Buttons::RIGHT),
];

fn bit(button: Button) -> Buttons {
    GILRS
        .iter()
        .find(|(b, _)| *b == button)
        .map_or(Buttons::NONE, |(_, bit)| *bit)
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

    if args.should_run {
        return if pad_map::udp_selected(&args.config) {
            std::process::ExitCode::from(1)
        } else {
            std::process::ExitCode::SUCCESS
        };
    }

    // Before anything that can fail, and before the gamepad subsystem especially: `padd` was the
    // one daemon whose journal could not say which build was running, which came up while chasing
    // exactly that question across all five.
    duck_ipc_proto::log_startup_identity!("padd");

    let mut gilrs = match Gilrs::new() {
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
    let roller = match ask_roller(&mut stream, &mut next_id) {
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
    let (bindings, imu_head, drive) = read_bindings(&args.config);
    // When the file was last written, so a change is picked up without a restart. `padd` holds
    // no motor control and no session state — the whole of it is this table — so re-reading is a
    // swap between two ticks rather than anything to sequence.
    let mut bindings_at = config_mtime(&args.config);
    let mut bindings_checked = Instant::now();

    // Everything the mapping reads that is not the pad. Rebuilt — a field at a time — whenever the
    // bindings reload or the drive mode changes; the rest is flags fixed at startup.
    let mut config = Config {
        bindings,
        imu_head,
        drive,
        deadzone: args.deadzone,
        max_head: args.max_head,
        roller,
    };
    // What the pad means: the mode, the holds, the trigger edges and whether the robot is up.
    // `pad_map`, so that `netpadd` maps a pad exactly as this does.
    let mut mapper = Mapper::new();
    // Whether a pad was there last tick, so appearing and disappearing are each logged once.
    let mut driving = false;
    // The continuous intents, and the buffer this tick's are built in. Both live across
    // ticks so a steady state neither allocates nor re-sends — see [`Continuous`].
    let mut continuous = Continuous::default();
    let mut frame: Vec<proto::Call> = Vec::with_capacity(2);
    // This tick's discrete intents, in the order they go out. Reused like `frame`.
    let mut out: Vec<Out> = Vec::new();

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
                (config.bindings, config.imu_head, config.drive) = read_bindings(&args.config);
                tracing::warn!("button bindings reloaded");
            }
            match ask_roller(&mut stream, &mut next_id) {
                Ok(Some(now)) if now != config.roller => {
                    config.roller = now;
                    tracing::warn!(
                        roller = config.roller,
                        "drive mode changed — stick shaping follows"
                    );
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
        // Which buttons went down and came up this tick. Edges rather than state: a press and
        // release inside one tick leaves the state unchanged on both sides, and Start and Select
        // are holds read off their state every tick *and* their release here.
        let mut pressed = Buttons::NONE;
        let mut released = Buttons::NONE;
        while let Some(event) = gilrs.next_event() {
            if Some(event.id) != pad_id {
                continue;
            }
            match event.event {
                gilrs::EventType::ButtonPressed(button, _) => pressed |= bit(button),
                gilrs::EventType::ButtonReleased(button, _) => released |= bit(button),
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
                tracing::warn!("pad gone — sending nothing; robotd's deadman holds the robot");
                driving = false;
            }
            // A hold in flight was measured against the pad that just left: drop it, or a
            // Select still down when the pad returns lands its full hold time at once — a
            // power-off nobody asked for.
            mapper.pad_gone();
            if let Some(tap) = tap.as_ref() {
                tap.idle();
            }
            std::thread::sleep(IDLE_POLL);
            continue;
        };

        if !driving {
            tracing::warn!(pad = pad.name(), "pad connected — driving");
            driving = true;
        }

        // Every tick rather than on the transition above: a pad that drops and comes back between
        // two ticks never clears `driving`, and it comes back as a different event node often
        // enough that a tap following the old one would report the rest of the session as silence.
        if let Some(tap) = tap.as_ref() {
            tap.watch(&pad);
            // Every tick, like the bindings: switching the feature on in the config has to start
            // the IMU reader without a restart, and off has to let it go.
            tap.imu_control(config.imu_head.enabled);
        }

        // The pad's attitude this tick, when the feature is on and the pad has an IMU that has
        // said something believable. `None` is every other case, and head + move then poses the
        // head from the right stick.
        let attitude = if config.imu_head.enabled {
            tap.as_ref().and_then(|tap| tap.attitude())
        } else {
            None
        };

        let held = GILRS
            .iter()
            .filter(|(b, _)| pad.is_pressed(*b))
            .fold(Buttons::NONE, |acc, (_, bit)| acc | *bit);
        let trigger = |b: Button| pad.button_data(b).map(|d| d.value()).unwrap_or(0.0) as f64;
        let frame_in = PadFrame {
            left_x: pad.value(Axis::LeftStickX) as f64,
            left_y: pad.value(Axis::LeftStickY) as f64,
            right_x: pad.value(Axis::RightStickX) as f64,
            right_y: pad.value(Axis::RightStickY) as f64,
            lt: trigger(Button::LeftTrigger2),
            rt: trigger(Button::RightTrigger2),
            held,
            pressed,
            released,
            attitude,
            has_imu: tap.as_ref().is_some_and(|t| t.has_imu()),
        };

        // The mapping says what to send; this sends it, in its order. A failed write ends the
        // process as it always has, and systemd brings back a `padd` with a fresh connection.
        mapper.tick(&frame_in, &config, tick, &mut out, &mut frame);
        for o in out.drain(..) {
            let (call, result) = match &o {
                Out::Notify(call) => (call, notify(&mut stream, call).map(|()| None)),
                Out::Request(call) => (call, request(&mut stream, &mut next_id, call)),
            };
            match result {
                Ok(response) => {
                    if let Out::Request(_) = &o {
                        pad_map::report(call, response.as_ref());
                    }
                }
                Err(e) => {
                    // Named by the call, because the line this replaced named what failed —
                    // "init failed", "skill request failed" — and the journal is all there is.
                    tracing::error!(error = %e, call = call.method(), "send failed");
                    return std::process::ExitCode::FAILURE;
                }
            }
        }
        if let Some(bytes) = continuous.next(&frame, tick)
            && let Err(e) = stream.write_all(bytes).and_then(|()| stream.flush())
        {
            tracing::error!(error = %e, "send failed");
            return std::process::ExitCode::FAILURE;
        }

        if let Some(remaining) = period.checked_sub(tick.elapsed()) {
            std::thread::sleep(remaining);
        }
    }
}

/// Whether this robot is on wheels, by asking it. `None` for an answer that did not say.
fn ask_roller(stream: &mut UnixStream, next_id: &mut u64) -> std::io::Result<Option<bool>> {
    let answer = request(stream, next_id, &proto::Call::RobotMode)?;
    pad_map::report(&proto::Call::RobotMode, answer.as_ref());
    Ok(answer
        .and_then(|answer| answer.result_as::<proto::ModeResult>().ok())
        .map(|mode| mode.mode == "roller"))
}

/// Send a continuous intent: no `id`, no reply, nothing to wait for.
fn notify(stream: &mut UnixStream, call: &proto::Call) -> std::io::Result<()> {
    let mut line = serde_json::to_vec(&proto::Request::notify(call))?;
    line.push(b'\n');
    stream.write_all(&line)?;
    stream.flush()
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
        Ok(response) => Ok(Some(response)),
        Err(e) => {
            tracing::warn!(error = %e, raw = %answer.trim(), "unparsable answer");
            Ok(None)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
}
