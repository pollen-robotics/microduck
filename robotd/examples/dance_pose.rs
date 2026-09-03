//! Attended dance client for phases 0B and 1 of `docs/ideas/dance-to-music.md`.
//!
//! Streams the stand-policy crouch waveform (`z` most negative at phase 0) at a fixed
//! BPM, or — with `--stdin` — from a beat tracker on raw `S16_LE` 16 kHz mono (laptop
//! mic, not a second `arecord` on the duck). Trap SIGINT / SIGTERM and send
//! `active: false` plus nominal head/mouth: `robot.enable` off does **not** clear pose.
//!
//! ```text
//! cargo run -p robotd --example dance-pose -- --socket /tmp/robotd.sock --bpm 100
//! arecord -f S16_LE -r 16000 -c 1 -t raw | cargo run -p robotd --example dance-pose -- --stdin
//! ```
//!
//! Stop `padd` (or leave it up with no pad) so a publishing twist cannot fight the pose.

use std::io::{self, BufRead, Read, Write};
use std::os::unix::net::UnixStream;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use clap::Parser;
use duck_ipc_proto as proto;
use pet_detect::beat::{BeatState, BeatTracker};
use pet_detect::i16_to_f32;
use pet_detect::mapper::{self, OverlayGate};

static STOP: AtomicBool = AtomicBool::new(false);

extern "C" fn on_signal(_: libc::c_int) {
    STOP.store(true, Ordering::SeqCst);
}

#[derive(Parser, Debug)]
#[command(name = "dance-pose", about = "Attended crouch-waveform client for a standing duck")]
struct Args {
    /// robotd Unix socket. Forward with `ssh -L /tmp/robotd.sock:/run/robotd.sock`.
    #[arg(long, default_value = proto::socket::ROBOT)]
    socket: PathBuf,
    /// Scripted BPM when not reading `--stdin`.
    #[arg(long, default_value_t = 100.0)]
    bpm: f32,
    /// Crouch energy 0..1 for the scripted path.
    #[arg(long, default_value_t = 1.0)]
    energy: f32,
    /// Raw S16_LE 16 kHz mono on stdin (laptop mic). Phase 1.
    #[arg(long)]
    stdin: bool,
    /// Send `robot.enable on` once at start.
    #[arg(long)]
    enable: bool,
}

fn main() {
    unsafe {
        libc::signal(libc::SIGINT, on_signal as libc::sighandler_t);
        libc::signal(libc::SIGTERM, on_signal as libc::sighandler_t);
    }

    let args = Args::parse();
    let mut stream = match UnixStream::connect(&args.socket) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("connect {}: {e}", args.socket.display());
            std::process::exit(1);
        }
    };

    let mut next_id = 1u64;
    if args.enable {
        if let Err(e) = request(
            &mut stream,
            &mut next_id,
            &proto::Call::RobotEnable(proto::EnableParams {
                on: true,
                toggle: false,
            }),
        ) {
            eprintln!("enable: {e}");
            std::process::exit(1);
        }
    }

    let beat_slot: Arc<Mutex<BeatState>> = Arc::new(Mutex::new(BeatState::default()));
    let stdin_thread = args.stdin.then(|| {
        let slot = beat_slot.clone();
        thread::spawn(move || pump_stdin(slot))
    });

    let period = Duration::from_millis(20);
    let started = Instant::now();
    let mut last_t = 0u64;
    let mut last_t_at = Instant::now();
    let mut released = false;

    while !STOP.load(Ordering::SeqCst) {
        let tick = started.elapsed();
        let (overlay, release) = if args.stdin {
            let beat = beat_slot.lock().map(|g| *g).unwrap_or_default();
            if beat.t != last_t {
                last_t = beat.t;
                last_t_at = Instant::now();
            }
            let t_fresh = beat.t > 0 && last_t_at.elapsed() <= mapper::stale_after(beat.bpm);
            let gate = OverlayGate {
                opt_in: true,
                walk_mode: true,
                enabled: true,
                has_standing: true,
                locked: beat.locked,
                t_fresh,
                ..OverlayGate::default()
            };
            if gate.external_must_release() {
                (mapper::Overlay::default(), true)
            } else {
                (mapper::map_beat(&beat), false)
            }
        } else {
            let bpm = args.bpm.clamp(60.0, 140.0);
            let phase = (tick.as_secs_f32() * bpm / 60.0).fract();
            (mapper::map_body(phase, args.energy.clamp(0.0, 1.0)), false)
        };

        let send = if release {
            released = true;
            notify_release(&mut stream)
        } else {
            released = false;
            notify(
                &mut stream,
                &proto::Call::RobotPose(proto::PoseParams {
                    z: overlay.z,
                    roll: overlay.roll,
                    pitch: overlay.pitch,
                    active: true,
                }),
            )
        };
        if let Err(e) = send {
            eprintln!("send: {e}");
            break;
        }
        thread::sleep(period);
    }

    if !released {
        let _ = notify_release(&mut stream);
    }
    if let Some(t) = stdin_thread {
        let _ = t.join();
    }
}

fn pump_stdin(slot: Arc<Mutex<BeatState>>) {
    let mut tracker = BeatTracker::new();
    let mut bytes = [0u8; 2048];
    let mut stdin = io::stdin();
    while !STOP.load(Ordering::SeqCst) {
        let n = match stdin.read(&mut bytes) {
            Ok(0) => {
                tracker.capture_lost();
                if let Ok(mut g) = slot.lock() {
                    *g = tracker.state();
                }
                break;
            }
            Ok(n) => n,
            Err(_) => break,
        };
        if !n.is_multiple_of(2) {
            continue;
        }
        let mut i16s = Vec::with_capacity(n / 2);
        for chunk in bytes[..n].chunks_exact(2) {
            i16s.push(i16::from_le_bytes([chunk[0], chunk[1]]));
        }
        let state = tracker.push(&i16_to_f32(&i16s));
        if let Ok(mut g) = slot.lock() {
            *g = state;
        }
    }
}

fn notify_release(stream: &mut UnixStream) -> io::Result<()> {
    notify(
        stream,
        &proto::Call::RobotPose(proto::PoseParams {
            z: 0.0,
            roll: 0.0,
            pitch: 0.0,
            active: false,
        }),
    )?;
    notify(
        stream,
        &proto::Call::RobotHead(proto::HeadParams::default()),
    )?;
    notify(
        stream,
        &proto::Call::RobotMouth(proto::MouthParams { open: 0.0 }),
    )
}

fn notify(stream: &mut UnixStream, call: &proto::Call) -> io::Result<()> {
    let mut line = serde_json::to_vec(&proto::Request::notify(call))?;
    line.push(b'\n');
    stream.write_all(&line)?;
    stream.flush()
}

fn request(stream: &mut UnixStream, next_id: &mut u64, call: &proto::Call) -> io::Result<()> {
    let id = proto::Id::Number(*next_id);
    *next_id += 1;
    let mut line = serde_json::to_vec(&proto::Request::call(id, call))?;
    line.push(b'\n');
    stream.write_all(&line)?;
    stream.flush()?;
    let mut reader = io::BufReader::new(stream.try_clone()?);
    let mut answer = String::new();
    reader.read_line(&mut answer)?;
    Ok(())
}
