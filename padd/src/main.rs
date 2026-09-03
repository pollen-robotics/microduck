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
//! ## The mapping is the prototype's, and now it is config
//!
//! Muscle memory carries over from `microduck_runtime`, and the five one-shot buttons are
//! `[pad]` in `robotd.toml` — so a robot that has learned a new skill can put it on a button
//! without a release. The defaults are exactly the mapping below, so a robot with no `[pad]`
//! section behaves as it always has.
//!
//! Only those five. `Start`, the two mode toggles, held `Select` and held `D-pad up` are not
//! `robot.do` calls, and the button that powers a robot off is the one binding worth not being
//! able to lose to a config edit.
//!
//! This daemon still knows nothing about what a skill *is*. It reads which button went down,
//! looks up the name beside it, and sends that name; `robotd` decides whether the robot has such
//! a thing and answers with the list it does have when it does not.
//!
//! ```text
//! Start        toggle the policy
//! Y (North)    head mode — sticks pose the head
//! B (East)     body-pose mode — sticks lean and crouch the standing robot
//! A (South)    ground pick
//! LB / RB      left / right kick
//! DPad-Down    sit ↔ stand
//! RT / LT      mouth (either trigger; the max wins) · RT quacks · LT rides the wheee
//! Select, 2 s  sit down, then power off
//! ```
//!
//! Head and body-pose mode both zero the velocity while active, as the prototype does — a
//! robot that keeps walking because you started posing its head is a bad surprise.
//!
//! Smoothing lives in `robotd` (`[control] cmd_alpha` / `head_alpha`), not here: this
//! process sends raw targets, so every client gets the same feel.
//!
//! ## Roller mode
//!
//! At startup this asks `robot.mode`. On a roller robot the stick mapping becomes the
//! prototype's roller preset — asymmetric forward/brake (0.6 / 0.5), no strafe, ±0.3 rad/s
//! heading — and A triggers the crouch that lives in the ground-pick slot. The other
//! skills ride along on wheels, as the rebased roller line has them.
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

    /// Full-deflection forward/strafe speed, m/s. The prototype's alpha default.
    #[arg(long, default_value_t = 0.3)]
    max_linear: f64,

    /// Full-deflection backward speed, m/s — the prototype caps reverse separately.
    #[arg(long, default_value_t = 0.3)]
    max_linear_backward: f64,

    /// Full-deflection turn rate, rad/s.
    #[arg(long, default_value_t = 1.5)]
    max_angular: f64,

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

/// Select held this long sits the robot down and powers it off.
const SHUTDOWN_HOLD: Duration = Duration::from_secs(2);

/// D-pad up held this long switches drive mode, walk ⇄ roller.
///
/// Three seconds, longer than the shutdown hold, and the prototype's number. D-pad up is a
/// direction anybody might lean on for a moment while driving; the mode switch takes the robot
/// home and reloads its policies, so it has to be a hold nobody performs by accident.
const MODE_HOLD: Duration = Duration::from_secs(3);

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

/// What the sticks drive. Head and body-pose are modal because two sticks cannot express
/// nine degrees of freedom; the toggles are the prototype's Y and B buttons.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    Drive,
    Head,
    BodyPose,
}

/// How often to look for a rewritten config. See the loop.
const BINDINGS_POLL: Duration = Duration::from_secs(1);

/// The button bindings, or the mapping the prototype had.
///
/// A file that will not parse is never a reason to leave somebody without a pad: the defaults
/// are a working robot, and the reason is logged. That matters more here than elsewhere because
/// this is re-read while running — a half-saved file caught mid-write must not take the buttons
/// away, and the next read a second later gets the finished one.
fn read_bindings(path: &Path) -> robotd_params::PadParams {
    match robotd_params::Params::load(path, false) {
        Ok(params) => {
            let pad = params.pad;
            tracing::info!(
                a = %pad.a, x = %pad.x, lb = %pad.lb, rb = %pad.rb,
                dpad_down = %pad.dpad_down,
                "button bindings"
            );
            pad
        }
        Err(e) => {
            tracing::warn!(
                error = %e,
                path = %path.display(),
                "cannot read the button bindings; using the default mapping"
            );
            robotd_params::PadParams::default()
        }
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

    // Which robot is this? A roller duck wants the roller stick shaping. Asked once at startup,
    // then kept in step by the D-pad-up switch below — which is this process asking for the
    // change, so it knows the answer without asking again.
    let mut roller = match request(&mut stream, &mut next_id, &proto::Call::RobotMode) {
        Ok(Some(answer)) => match answer.result_as::<proto::ModeResult>() {
            Ok(mode) => mode.mode == "roller",
            Err(_) => false,
        },
        _ => false,
    };
    tracing::warn!(
        socket = %args.socket.display(),
        hz = args.hz,
        roller,
        "driving — Start toggles the policy, Y head mode, B body pose, A ground pick, \
         LB/RB kicks, DPad-Down sit, triggers mouth, DPad-Up (3s) walk/roller, \
         Select (2s) shutdown"
    );

    let period = Duration::from_secs_f64(1.0 / args.hz as f64);
    // The button bindings, read once like every other daemon reads its config. A file that will
    // not parse is not a reason to leave somebody without a pad: the mapping the prototype had is
    // the fallback, and the reason is logged.
    let mut bindings = read_bindings(&args.config);
    // When the file was last written, so a change is picked up without a restart. `padd` holds
    // no motor control and no session state — the whole of it is this table — so re-reading is a
    // swap between two ticks rather than anything to sequence.
    let mut bindings_at = config_mtime(&args.config);
    let mut bindings_checked = Instant::now();

    let mut mode = Mode::Drive;
    // Whether a pad was there last tick, so appearing and disappearing are each logged once.
    let mut driving = false;
    let mut select_held_since: Option<Instant> = None;
    let mut dpad_up_held_since: Option<Instant> = None;
    let mut mode_switch_sent = false;
    let mut shutdown_sent = false;
    // Trigger levels last tick, for the sound edges: RT quacks on its rising edge, LT
    // starts the wheee ride. The prototype's threshold.
    let mut prev_rt = 0.0f64;
    let mut prev_lt = 0.0f64;
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
                bindings = read_bindings(&args.config);
                tracing::warn!("button bindings reloaded");
            }
        }

        // Drain the queue so axis polling below sees present state, and catch button
        // *edges* — a held Start must toggle once, not fifty times a second.
        let mut toggle_enable = false;
        let mut toggle_head = false;
        let mut toggle_body = false;
        // Which bindable buttons went down this tick, by their config name. A list rather than
        // a flag apiece, because what each one runs is config now and this loop no longer knows.
        let mut pressed: Vec<&'static str> = Vec::new();
        while let Some(event) = gilrs.next_event() {
            if let gilrs::EventType::ButtonPressed(button, _) = event.event {
                match button {
                    Button::Start => toggle_enable = true,
                    Button::North => toggle_head = true,
                    Button::East => toggle_body = true,
                    // The five bindable ones. What each runs is `[pad]` in the config; this
                    // only knows which physical control was pressed.
                    //
                    // gilrs names the *bumpers* `LeftTrigger`/`RightTrigger`; the analog
                    // triggers are `LeftTrigger2`/`RightTrigger2`. Getting that backwards binds
                    // a skill to a control nobody presses.
                    Button::South => pressed.push("a"),
                    Button::West => pressed.push("x"),
                    Button::LeftTrigger => pressed.push("lb"),
                    Button::RightTrigger => pressed.push("rb"),
                    Button::DPadDown => pressed.push("dpad_down"),
                    _ => {}
                }
            }
        }

        let Some((_, pad)) = gilrs.gamepads().next() else {
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
        }

        if toggle_head {
            mode = if mode == Mode::Head {
                Mode::Drive
            } else {
                Mode::Head
            };
            tracing::info!(?mode, "mode");
        }
        if toggle_body {
            let leaving = mode == Mode::BodyPose;
            mode = if leaving { Mode::Drive } else { Mode::BodyPose };
            tracing::info!(?mode, "mode");
            if leaving {
                // The prototype's B-button exit snaps the body back to nominal at once.
                if let Err(e) = notify(
                    &mut stream,
                    &proto::Call::RobotPose(proto::PoseParams {
                        active: false,
                        ..Default::default()
                    }),
                ) {
                    tracing::error!(error = %e, "send failed");
                    return std::process::ExitCode::FAILURE;
                }
            }
        }

        if toggle_enable {
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

        // X held: keep a chaining skill going. The robot starts another when a request lands
        // near the end of the current one, so "held" is spelled "resent every tick" — as a
        // notification, because fifty answered requests a second would spend their time waiting
        // on replies, and the press above already got the real answer.
        //
        // Whatever X is bound to, not the word "roulade": a skill that does not chain simply
        // refuses the resend, which costs a notification nobody reads. Only X, because it is the
        // button the prototype held and the only one anybody holds.
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

        // Select held two seconds: sit down, then power off. Sent once per hold — the
        // robot owns the sequence from there, and a second request would be a no-op anyway.
        if pad.is_pressed(Button::Select) {
            let held = select_held_since.get_or_insert(tick);
            if tick.duration_since(*held) >= SHUTDOWN_HOLD && !shutdown_sent {
                shutdown_sent = true;
                tracing::warn!("Select held — asking the robot to sit and power off");
                if let Err(e) = request(&mut stream, &mut next_id, &proto::Call::RobotShutdown) {
                    tracing::error!(error = %e, "shutdown request failed");
                    return std::process::ExitCode::FAILURE;
                }
            }
        } else {
            select_held_since = None;
            shutdown_sent = false;
        }

        // D-pad up held three seconds: switch drive mode, which is what somebody who has just
        // put wheels on the duck (or taken them off) wants. Sent once per hold, and the target is
        // named rather than toggled — so a request that crosses a switch from somewhere else asks
        // for a mode rather than for "the other one", which could be either by the time it lands.
        if pad.is_pressed(Button::DPadUp) {
            let held = dpad_up_held_since.get_or_insert(tick);
            if tick.duration_since(*held) >= MODE_HOLD && !mode_switch_sent {
                mode_switch_sent = true;
                let target = if roller { "walk" } else { "roller" };
                tracing::warn!(
                    target,
                    "DPad-Up held — asking the robot to switch drive mode"
                );
                let call = proto::Call::RobotSetMode(proto::SetModeParams {
                    mode: target.to_owned(),
                });
                match request(&mut stream, &mut next_id, &call) {
                    Ok(Some(answer)) => match answer.result_as::<proto::IntentResult>() {
                        // The stick shaping follows the robot, and only when it agreed: a refused
                        // switch that changed the mapping here would leave the pad driving a
                        // walking duck with roller curves.
                        Ok(result) if result.accepted => {
                            roller = target == "roller";
                            tracing::warn!(roller, "drive mode switched");
                        }
                        Ok(result) => tracing::warn!(
                            reason = result.reason.as_deref().unwrap_or("no reason given"),
                            "the robot refused the mode switch"
                        ),
                        Err(e) => tracing::error!(error = %e, "unreadable answer to the switch"),
                    },
                    Ok(None) => tracing::warn!("no answer to the mode switch"),
                    Err(e) => {
                        tracing::error!(error = %e, "mode switch request failed");
                        return std::process::ExitCode::FAILURE;
                    }
                }
            }
        } else {
            dpad_up_held_since = None;
            mode_switch_sent = false;
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

        // This tick's continuous intents, as one frame. Reused rather than built fresh:
        // a `Vec` per tick is an allocation fifty times a second to say what the sticks
        // were doing, which is the shape of thing this loop is meant not to do.
        frame.clear();
        match mode {
            Mode::Drive if roller => frame.push(proto::Call::RobotMove(proto::MoveParams {
                // The prototype's roller shaping: push harder than you can brake, no
                // strafe, heading capped independently of the walking limits.
                vx: left_y
                    * if left_y >= 0.0 {
                        ROLLER_PUSH
                    } else {
                        ROLLER_BRAKE
                    },
                vy: 0.0,
                vyaw: -right_x * ROLLER_YAW,
            })),
            Mode::Drive => frame.push(proto::Call::RobotMove(proto::MoveParams {
                vx: left_y
                    * if left_y >= 0.0 {
                        args.max_linear
                    } else {
                        args.max_linear_backward
                    },
                // `vy` is positive to the left; stick-left reads negative on every pad
                // gilrs normalises.
                vy: -left_x * args.max_linear,
                vyaw: -right_x * args.max_angular,
            })),
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
                    pitch: right_y * BODY_MAX_ANGLE,
                    roll: right_x * BODY_MAX_ANGLE,
                    active: true,
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
}
