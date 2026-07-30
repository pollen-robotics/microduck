//! `padd` — a gamepad, as an intent client.
//!
//! It has no privileged access to the robot. It reads a pad, turns sticks into velocity and
//! head targets, and sends them over `robotd`'s socket like any other client.
//!
//! That is the point of it being a separate process rather than a thread inside `robotd`.
//! The intent API is the path the app, the SDK and any remote client will use, and here it
//! gets exercised every day by whoever is working on the robot — so it cannot quietly rot
//! the way an API only the phone app uses inevitably would. The cost is a socket hop: tens
//! of microseconds against a 20 ms tick.
//!
//! For development against a board: `ssh -L /tmp/robotd.sock:/run/robotd.sock duck`, then
//! point `--socket` at the forwarded path. Pad on your laptop, robot on the bench, no code.

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use clap::Parser;
use duck_ipc_proto as proto;
use gilrs::{Axis, Button, Gilrs};

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
    /// and without this the robot creeps.
    #[arg(long, default_value_t = 0.12)]
    deadzone: f64,

    /// Full-deflection forward/strafe speed, m/s.
    #[arg(long, default_value_t = 0.15)]
    max_linear: f64,

    /// Full-deflection turn rate, rad/s.
    #[arg(long, default_value_t = 1.0)]
    max_angular: f64,

    /// Full-deflection head travel, radians.
    #[arg(long, default_value_t = 0.5)]
    max_head: f64,
}

/// Sticks drive either the body or the head, never both — the prototype does the same, on
/// its X button. Two sticks cannot express five degrees of freedom, and a modal toggle is
/// clearer than a chord.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    Body,
    Head,
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

    let mut gilrs = match Gilrs::new() {
        Ok(gilrs) => gilrs,
        Err(e) => {
            tracing::error!(error = %e, "no gamepad subsystem");
            return std::process::ExitCode::FAILURE;
        }
    };

    let mut stream = match UnixStream::connect(&args.socket) {
        Ok(stream) => stream,
        Err(e) => {
            tracing::error!(error = %e, socket = %args.socket.display(), "cannot reach robotd");
            return std::process::ExitCode::FAILURE;
        }
    };
    tracing::warn!(
        socket = %args.socket.display(),
        hz = args.hz,
        "driving — Start toggles the policy, North toggles head mode, East stops"
    );

    let period = Duration::from_secs_f64(1.0 / args.hz as f64);
    let mut mode = Mode::Body;
    let mut enabled = false;
    let mut next_id = 1u64;

    loop {
        let tick = Instant::now();

        // Drain the queue so axis polling below sees present state, and catch button
        // *edges* — a held Start must toggle once, not fifty times a second.
        let mut toggle_enable = false;
        let mut toggle_mode = false;
        let mut stop = false;
        while let Some(event) = gilrs.next_event() {
            if let gilrs::EventType::ButtonPressed(button, _) = event.event {
                match button {
                    Button::Start => toggle_enable = true,
                    Button::North => toggle_mode = true,
                    Button::East => stop = true,
                    _ => {}
                }
            }
        }

        let Some((_, pad)) = gilrs.gamepads().next() else {
            // No pad. Send nothing: `robotd`'s deadman stops the robot on its own, which is
            // exactly the wanted behaviour, and inventing a zero command here would mask a
            // disconnected pad as a deliberate stop.
            std::thread::sleep(period);
            continue;
        };

        if toggle_mode {
            mode = match mode {
                Mode::Body => Mode::Head,
                Mode::Head => Mode::Body,
            };
            tracing::info!(?mode, "mode");
        }

        if toggle_enable {
            enabled = !enabled;
            let call = proto::Call::RobotEnable(proto::EnableParams { on: enabled });
            if let Err(e) = request(&mut stream, &mut next_id, &call) {
                tracing::error!(error = %e, "enable failed");
                return std::process::ExitCode::FAILURE;
            }
            tracing::warn!(enabled, "policy");
        }

        if stop {
            if let Err(e) = request(&mut stream, &mut next_id, &proto::Call::RobotStop) {
                tracing::error!(error = %e, "stop failed");
                return std::process::ExitCode::FAILURE;
            }
            tracing::warn!("stop");
        }

        let deadzone = |v: f32| {
            let v = v as f64;
            if v.abs() < args.deadzone { 0.0 } else { v }
        };
        let left_y = deadzone(pad.value(Axis::LeftStickY));
        let left_x = deadzone(pad.value(Axis::LeftStickX));
        let right_y = deadzone(pad.value(Axis::RightStickY));
        let right_x = deadzone(pad.value(Axis::RightStickX));

        let call = match mode {
            Mode::Body => proto::Call::RobotMove(proto::MoveParams {
                vx: left_y * args.max_linear,
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
                proto::Call::RobotHead(proto::HeadParams {
                    neck_pitch: left_y * args.max_head,
                    head_pitch: right_y * args.max_head,
                    head_yaw: right_x * args.max_head,
                    head_roll: left_x * args.max_head,
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
/// outcome — safety declines to enable a policy on a fallen robot — and a client that
/// ignored it would leave the operator wondering why nothing happened.
fn request(stream: &mut UnixStream, next_id: &mut u64, call: &proto::Call) -> std::io::Result<()> {
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
            if let Some(error) = response.error {
                tracing::warn!(code = error.code, message = %error.message, "refused");
            } else if let Ok(result) = response.result_as::<proto::IntentResult>()
                && !result.accepted
            {
                tracing::warn!(reason = ?result.reason, "not accepted");
            }
        }
        Err(e) => tracing::warn!(error = %e, raw = %answer.trim(), "unparsable answer"),
    }
    Ok(())
}
