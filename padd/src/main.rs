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
//! ## The mapping is the prototype's
//!
//! Muscle memory carries over from `microduck_runtime`:
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
//! sticks are *polled*, so the last known value keeps being sent at 50 Hz whether or not the pad is
//! still talking, and every surface downstream then shows a robot with a live driver. The event
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
use std::path::PathBuf;
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

    /// How often to send intents. Matching the control rate exactly buys nothing — the loop
    /// reads the latest value once per tick — but staying at or above it keeps the added
    /// latency under one tick.
    #[arg(long, default_value_t = 50)]
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
         LB/RB kicks, DPad-Down sit, triggers mouth, DPad-Up reboot servos, DPad-Up (3s) \
         walk/roller, Select (2s) shutdown"
    );

    let period = Duration::from_secs_f64(1.0 / args.hz as f64);
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

    loop {
        let tick = Instant::now();

        // Drain the queue so axis polling below sees present state, and catch button
        // *edges* — a held Start must toggle once, not fifty times a second.
        let mut toggle_enable = false;
        let mut toggle_head = false;
        let mut toggle_body = false;
        let mut ground_pick = false;
        let mut kick_left = false;
        let mut kick_right = false;
        let mut sit_toggle = false;
        let mut roulade = false;
        let mut reboot_motors = false;
        while let Some(event) = gilrs.next_event() {
            if let gilrs::EventType::ButtonPressed(button, _) = event.event {
                match button {
                    Button::Start => toggle_enable = true,
                    Button::North => toggle_head = true,
                    Button::East => toggle_body = true,
                    Button::South => ground_pick = true,
                    // X on an Xbox pad. Press = one roulade; holding it chains rolls,
                    // which the resend below carries.
                    Button::West => roulade = true,
                    // gilrs names the bumpers `LeftTrigger`/`RightTrigger`; the analog
                    // triggers are `LeftTrigger2`/`RightTrigger2`.
                    Button::LeftTrigger => kick_left = true,
                    Button::RightTrigger => kick_right = true,
                    Button::DPadDown => sit_toggle = true,
                    // Reboot the servos: the way back from a tripped overload without pulling
                    // the battery.
                    Button::DPadUp => reboot_motors = true,
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

        // One-shot skills. Answered, because "refused, and here is why" is a real outcome —
        // there may be no kick policy on this robot, or another move mid-flight.
        for (fired, skill) in [
            (ground_pick, proto::Skill::GroundPick),
            (kick_left, proto::Skill::KickLeft),
            (kick_right, proto::Skill::KickRight),
            (sit_toggle, proto::Skill::SitToggle),
            (roulade, proto::Skill::Roulade),
        ] {
            if fired {
                let call = proto::Call::RobotDo(proto::DoParams { skill });
                if let Err(e) = request(&mut stream, &mut next_id, &call) {
                    tracing::error!(error = %e, "skill request failed");
                    return std::process::ExitCode::FAILURE;
                }
            }
        }

        // X held: keep the roulade chain alive. The robot chains another roll when a
        // request lands near the end of the current one, so "held" is spelled "resent every
        // tick" — as a notification, because fifty answered requests a second would spend
        // their time waiting on replies, and the press above already got the real answer.
        if pad.is_pressed(Button::West)
            && !roulade
            && let Err(e) = notify(
                &mut stream,
                &proto::Call::RobotDo(proto::DoParams {
                    skill: proto::Skill::Roulade,
                }),
            )
        {
            tracing::error!(error = %e, "send failed");
            return std::process::ExitCode::FAILURE;
        }

        if reboot_motors {
            tracing::warn!("DPad-Up — asking the robot to reboot its servos (robot.rebootMotors)");
            let call = proto::Call::RobotRebootMotors(proto::RebootMotorsParams::default());
            if let Err(e) = request(&mut stream, &mut next_id, &call) {
                tracing::error!(error = %e, "reboot request failed");
                return std::process::ExitCode::FAILURE;
            }
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

        let call = match mode {
            Mode::Drive if roller => proto::Call::RobotMove(proto::MoveParams {
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
            }),
            Mode::Drive => proto::Call::RobotMove(proto::MoveParams {
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
            }),
            Mode::Head => {
                // The body must not keep its last velocity while the sticks are posing the
                // head. The deadman would catch it eventually; a robot that keeps walking
                // because you started moving its head is a bad enough surprise to be
                // explicit about.
                if let Err(e) = notify(
                    &mut stream,
                    &proto::Call::RobotMove(proto::MoveParams::default()),
                ) {
                    tracing::error!(error = %e, "send failed");
                    return std::process::ExitCode::FAILURE;
                }
                // The prototype's alpha mapping, signs included (its head_pitch/head_yaw
                // joint axes are inverted relative to stick direction — verified on
                // hardware there, kept verbatim here).
                proto::Call::RobotHead(proto::HeadParams {
                    neck_pitch: right_y * args.max_head,
                    head_pitch: -left_y * args.max_head,
                    head_yaw: -left_x * args.max_head,
                    head_roll: right_x * args.max_head,
                })
            }
            Mode::BodyPose => {
                if let Err(e) = notify(
                    &mut stream,
                    &proto::Call::RobotMove(proto::MoveParams::default()),
                ) {
                    tracing::error!(error = %e, "send failed");
                    return std::process::ExitCode::FAILURE;
                }
                proto::Call::RobotPose(proto::PoseParams {
                    z: left_y
                        * if left_y >= 0.0 {
                            BODY_MAX_Z_UP
                        } else {
                            BODY_MAX_Z_DOWN
                        },
                    pitch: right_y * BODY_MAX_ANGLE,
                    roll: right_x * BODY_MAX_ANGLE,
                    active: true,
                })
            }
        };

        if let Err(e) = notify(&mut stream, &call) {
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
