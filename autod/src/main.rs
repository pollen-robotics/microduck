//! `autod` — the autonomous brain, as an intent client.
//!
//! `docs/design/autonomous.md` is the design. In one paragraph: with `[autonomous] enabled`, this
//! stands the robot up at boot with no pad, then reads `robot.state` and the head ToF and sends
//! ordinary intents, driven by a state machine (`brain.rs`) and an obstacle model
//! (`obstacle.rs`). It steps aside whenever `robot.state.move.source` says the pad is driving.
//! Switched off, it parks and touches nothing.
//!
//! Like `padd`, it holds no privileged access: it is a socket client in group `robot`, and the
//! deadman stops the robot within `[safety] deadman_ms` of this process dying.
//!
//! Three connections:
//!
//! - **commands** to `robotd`, answered requests and notifications, used by the tick loop alone;
//! - **state** from `robotd`, a `robot.subscribe` stream read by a thread of its own;
//! - **depth** from `tofd`, a `tof.stream` read by another, retried while `tofd` is down.
//!   A duck with no depth never advances: the brain only walks a path it has seen.
//!
//! A broken connection to `robotd` ends the process and systemd starts it again, as with `padd`.

mod brain;
mod obstacle;

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use clap::Parser;
use duck_ipc_proto as proto;

#[derive(Parser, Debug)]
#[command(name = "autod", about = "The robot's own brain", version)]
struct Args {
    /// `robotd`'s socket.
    #[arg(long, default_value = proto::socket::ROBOT)]
    socket: PathBuf,

    /// `tofd`'s socket.
    #[arg(long, default_value = proto::socket::TOF)]
    tof_socket: PathBuf,

    /// The robot's configuration: `[autonomous]` decides whether this runs at all.
    #[arg(long, default_value = robotd_params::DEFAULT_PATH)]
    config: PathBuf,

    /// Where "the robot was stood up on this boot" is remembered: the unit's `StateDirectory`.
    #[arg(long, default_value = "/var/lib/autod")]
    state_dir: PathBuf,

    /// Brain ticks per second. The control rate: a target sent faster is never read.
    #[arg(long, default_value_t = 50, value_parser = clap::value_parser!(u32).range(1..=200))]
    hz: u32,
}

/// The pad's twist must have been gone this long before the brain takes back over: a pad that is
/// still being driven heartbeats every 100 ms, and this rides over a radio hiccup.
const PAD_GRACE: Duration = Duration::from_secs(1);

/// The duck's own sounds come back through its microphone. A noise or a voice heard within this
/// long after the brain asked for a sound is its own, and is dropped — it must never answer itself.
const SELF_AUDIO: Duration = Duration::from_millis(1500);

/// How long to wait at boot for the robot to come up after `robot.enable`.
const STAND_UP: Duration = Duration::from_secs(20);

fn main() -> std::process::ExitCode {
    let args = Args::parse();
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_writer(std::io::stderr)
        .init();
    duck_ipc_proto::log_startup_identity!("autod");

    let params = match robotd_params::Params::load(&args.config, false) {
        Ok(params) => params,
        Err(e) => {
            // The same file `robotd` refuses to start on, so there is no robot to drive either.
            tracing::error!(error = %e, "cannot read the configuration");
            return std::process::ExitCode::FAILURE;
        }
    };
    if !params.autonomous.enabled {
        // Parked rather than exited: the unit is `Restart=always`, so an exit would be a restart
        // loop, and an idle process costs nothing. `robotctl configure` restarts this unit when
        // the switch changes.
        tracing::info!("autonomy is off ([autonomous] enabled = false) — idle");
        loop {
            std::thread::park();
        }
    }

    match run(&args, &params) {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(e) => {
            tracing::error!(error = %e, "lost robotd");
            std::process::ExitCode::FAILURE
        }
    }
}

/// What the state thread last saw, and what the depth thread has made of the room.
#[derive(Default)]
struct Shared {
    history: obstacle::History,
    model: obstacle::Model,
    latest: Option<Latest>,
    /// The state stream ended: `robotd` is gone.
    lost: bool,
}

/// The fields of a `robot.state` frame the tick loop reads.
#[derive(Debug, Clone)]
struct Latest {
    sample: obstacle::Sample,
    policy: String,
    fallen: bool,
    source: Option<String>,
    deadman: bool,
    hearing: Option<proto::HearingState>,
}

/// What the robot has, from the subscribe ack.
#[derive(Debug, Clone, Copy)]
struct Abilities {
    ground_pick: bool,
    sit_stand: bool,
    /// The robot speaks v59: `robot.head` takes `alpha`.
    head_alpha: bool,
}

fn run(args: &Args, params: &robotd_params::Params) -> std::io::Result<()> {
    let mut commands = UnixStream::connect(&args.socket)?;
    let mut next_id = 1u64;

    let hello = proto::Call::Hello(proto::HelloParams {
        api_version: proto::API_VERSION,
        client: Some("autod".to_owned()),
    });
    let api = request(&mut commands, &mut next_id, &hello)?
        .and_then(|r| r.result_as::<proto::HelloResult>().ok())
        .map_or(0, |h| h.api_version);
    if api < 59 {
        // Logged and served, never refused: this robot cannot say who is driving, so the brain
        // cannot see a pad take over. Updating moves every daemon together.
        tracing::warn!(
            api,
            "robotd predates v59 — the pad's takeover is invisible to the brain on this robot"
        );
    }

    let shared = Arc::new(Mutex::new(Shared::default()));
    let abilities = subscribe(&args.socket, &shared, api >= 59)?;
    tracing::warn!(?abilities, "autonomous — the duck is on its own");
    spawn_depth(args.tof_socket.clone(), Arc::clone(&shared));

    stand_up_once(&mut commands, &mut next_id, &shared, &args.state_dir)?;
    drive(
        args,
        params,
        &mut commands,
        &mut next_id,
        &shared,
        abilities,
    )
}

/// Open the state stream, and read it on a thread of its own from then on.
fn subscribe(
    socket: &Path,
    shared: &Arc<Mutex<Shared>>,
    head_alpha: bool,
) -> std::io::Result<Abilities> {
    let mut stream = UnixStream::connect(socket)?;
    let call = proto::Request::call(
        proto::Id::Number(1),
        &proto::Call::RobotSubscribe(proto::SubscribeParams { hz: None }),
    );
    let mut line = serde_json::to_vec(&call)?;
    line.push(b'\n');
    stream.write_all(&line)?;
    let mut reader = BufReader::new(stream);
    // The ack is the first line that is not a notification; a state frame can beat it.
    let ack = loop {
        let mut raw = String::new();
        if reader.read_line(&mut raw)? == 0 {
            return Err(std::io::Error::other("robotd closed the state stream"));
        }
        if let Ok(response) = serde_json::from_str::<proto::Response>(&raw)
            && response.id.is_some()
        {
            break response.result_as::<proto::SubscribeResult>().ok();
        }
    };
    let abilities = match ack {
        Some(ack) => Abilities {
            ground_pick: ack.ground_pick.is_some(),
            sit_stand: ack.sitstand.is_some(),
            head_alpha,
        },
        None => Abilities {
            ground_pick: false,
            sit_stand: false,
            head_alpha,
        },
    };

    let shared = Arc::clone(shared);
    std::thread::spawn(move || {
        let mut raw = String::new();
        loop {
            raw.clear();
            match reader.read_line(&mut raw) {
                Ok(0) | Err(_) => break,
                Ok(_) => {}
            }
            let Ok(request) = serde_json::from_str::<proto::Request>(&raw) else {
                continue;
            };
            let Some(state) = request.as_state() else {
                continue;
            };
            let latest = Latest {
                sample: obstacle::Sample::of(&state),
                policy: state.policy,
                fallen: state.safety.fallen,
                source: state.movement.source,
                deadman: state.movement.limited_by.iter().any(|l| l == "deadman"),
                hearing: state.hearing,
            };
            let mut shared = shared.lock().unwrap_or_else(|p| p.into_inner());
            shared.history.push(latest.sample);
            shared.latest = Some(latest);
        }
        shared.lock().unwrap_or_else(|p| p.into_inner()).lost = true;
    });
    Ok(abilities)
}

/// Read depth frames into the obstacle model, forever. `tofd` being down is not fatal — the duck
/// simply stops advancing, because the path is never observed — and it is retried.
fn spawn_depth(socket: PathBuf, shared: Arc<Mutex<Shared>>) {
    std::thread::spawn(move || {
        let mut warned = false;
        loop {
            if let Err(e) = depth(&socket, &shared) {
                if !warned {
                    tracing::warn!(
                        error = %e, socket = %socket.display(),
                        "no depth frames — the duck will look around but not walk until they come"
                    );
                    warned = true;
                }
                std::thread::sleep(Duration::from_secs(5));
            }
        }
    });
}

fn depth(socket: &Path, shared: &Mutex<Shared>) -> std::io::Result<()> {
    let mut stream = UnixStream::connect(socket)?;
    let call = proto::Request::call(proto::Id::Number(1), &proto::Call::TofStream);
    let mut line = serde_json::to_vec(&call)?;
    line.push(b'\n');
    stream.write_all(&line)?;
    let mut reader = BufReader::new(stream);
    let mut raw = String::new();
    let mut announced = false;
    loop {
        raw.clear();
        if reader.read_line(&mut raw)? == 0 {
            return Err(std::io::Error::other("tofd closed the depth stream"));
        }
        let Ok(request) = serde_json::from_str::<proto::Request>(&raw) else {
            // The ack, or a refusal: a sensor that is unavailable says so here.
            if let Ok(response) = serde_json::from_str::<proto::Response>(&raw)
                && let Ok(ack) = response.result_as::<proto::TofStreamResult>()
                && !ack.accepted
            {
                return Err(std::io::Error::other(format!(
                    "tofd refused the stream: {}",
                    ack.unavailable.unwrap_or_default()
                )));
            }
            continue;
        };
        let Some(frame) = request.as_tof_frame() else {
            continue;
        };
        if !announced {
            tracing::info!("depth frames arriving — obstacle avoidance on");
            announced = true;
        }
        let mut shared = shared.lock().unwrap_or_else(|p| p.into_inner());
        if let Some(at) = shared.history.nearest(frame.t_ns) {
            shared.model.observe(&frame, &at);
        }
    }
}

fn latest(shared: &Mutex<Shared>) -> (Option<Latest>, bool) {
    let shared = shared.lock().unwrap_or_else(|p| p.into_inner());
    (shared.latest.clone(), shared.lost)
}

/// Driving labels: the policy is on and the robot is the brain's to move.
fn standing(policy: &str) -> bool {
    matches!(policy, "walk" | "stand")
}

/// Parked on the sitstand network. The loop labels a robot sat down by `sit_toggle` `sit` for as
/// long as it stays down (`rise` is the way back up, and busy); `seated` is the same network as the
/// driving state names it. Head and mouth stay live while seated, which is what the nap animates.
fn seated(policy: &str) -> bool {
    matches!(policy, "sit" | "seated")
}

/// Not driving at all: limp, homing, or the policy is off. The brain sends nothing — it never
/// re-enables what somebody else disabled.
fn off(policy: &str) -> bool {
    matches!(policy, "held" | "homing" | "limp_fall" | "limp_pose")
}

/// This boot's identity, from the kernel. `None` off Linux, where every start is a first one.
fn boot_id() -> Option<String> {
    std::fs::read_to_string("/proc/sys/kernel/random/boot_id")
        .ok()
        .map(|id| id.trim().to_owned())
}

/// Whether the marker says the robot was already stood up on the boot `boot`.
fn stood_up_on(marker: &Path, boot: Option<&str>) -> bool {
    match (std::fs::read_to_string(marker), boot) {
        (Ok(recorded), Some(boot)) => recorded.trim() == boot,
        _ => false,
    }
}

/// Stand the robot up, once per boot. `docs/design/autonomous.md` §2: a restart of this unit
/// later in the same boot must not stand back up a robot somebody stopped.
fn stand_up_once(
    commands: &mut UnixStream,
    next_id: &mut u64,
    shared: &Mutex<Shared>,
    state_dir: &Path,
) -> std::io::Result<()> {
    let marker = state_dir.join("started");
    let boot = boot_id();
    if stood_up_on(&marker, boot.as_deref()) {
        tracing::info!("already stood up on this boot — leaving the policy as it is");
        return Ok(());
    }
    let deadline = Instant::now() + STAND_UP;
    let first = loop {
        let (state, lost) = latest(shared);
        if lost {
            return Err(std::io::Error::other("robotd closed the state stream"));
        }
        if let Some(state) = state {
            break state;
        }
        if Instant::now() > deadline {
            return Err(std::io::Error::other("no robot.state from robotd"));
        }
        std::thread::sleep(Duration::from_millis(50));
    };
    if off(&first.policy) {
        tracing::warn!("standing up — no pad needed (robot.enable)");
        let call = proto::Call::RobotEnable(proto::EnableParams {
            on: true,
            toggle: false,
        });
        let accepted = request(commands, next_id, &call)?
            .and_then(|r| r.result_as::<proto::IntentResult>().ok())
            .is_some_and(|r| r.accepted);
        if !accepted {
            // Not marked: the next start of this unit tries again.
            return Ok(());
        }
        while Instant::now() < deadline {
            let (state, lost) = latest(shared);
            if lost {
                return Err(std::io::Error::other("robotd closed the state stream"));
            }
            if state.is_some_and(|s| !off(&s.policy)) {
                break;
            }
            std::thread::sleep(Duration::from_millis(100));
        }
    }
    if let Err(e) = std::fs::write(&marker, boot.unwrap_or_default()) {
        tracing::warn!(error = %e, path = %marker.display(), "cannot remember the stand-up");
    }
    Ok(())
}

/// The tick loop.
fn drive(
    args: &Args,
    params: &robotd_params::Params,
    commands: &mut UnixStream,
    next_id: &mut u64,
    shared: &Mutex<Shared>,
    abilities: Abilities,
) -> std::io::Result<()> {
    let seed = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0x5eed, |d| d.as_nanos() as u64);
    // The pad's full stick, so the duck on its own never goes faster — or slower — than the
    // person holding the pad could make it. Read once, like the rest of `[autonomous]`.
    let mut brain = brain::Brain::new(params.pad_drive.vx_max, params.pad_drive.vyaw_max, seed);
    let period = Duration::from_secs_f64(1.0 / f64::from(args.hz));
    let mut last_tick = Instant::now();
    // The pad: when its twist was last live, and whether the brain is standing aside for it.
    let mut pad_at: Option<Instant> = None;
    let mut yielded = false;
    // What the microphone had counted, and until when what it hears is the duck itself.
    let mut heard_before: Option<proto::HearingState> = None;
    let mut quiet_until = Instant::now();
    let mut wheee_on = false;
    let mut posing = false;
    let mut idle_reason: Option<&'static str> = None;
    let mut frame: Vec<u8> = Vec::with_capacity(512);

    loop {
        let tick = Instant::now();
        let dt = tick.duration_since(last_tick).as_secs_f64().min(0.1);
        last_tick = tick;

        let (state, lost) = latest(shared);
        if lost {
            return Err(std::io::Error::other("robotd closed the state stream"));
        }
        let Some(state) = state else {
            std::thread::sleep(period);
            continue;
        };

        // The microphone's counters move whether or not the brain is listening; keep up with them
        // so a backlog is never delivered as a burst of fresh events later.
        let heard = match (heard_before, state.hearing) {
            (Some(before), Some(now)) if tick >= quiet_until => {
                if now.noises != before.noises {
                    Some(brain::Heard::Noise)
                } else if now.voices != before.voices {
                    Some(brain::Heard::Voice)
                } else {
                    None
                }
            }
            _ => None,
        };
        heard_before = state.hearing;

        // The pad is driving: say nothing at all, so it has the head, mouth and voice too.
        let pad_live = state.source.as_deref() == Some("padd") && !state.deadman;
        if pad_live {
            pad_at = Some(tick);
        }
        if pad_at.is_some_and(|at| tick.duration_since(at) < PAD_GRACE) {
            if !yielded {
                tracing::warn!("the pad is driving — standing aside");
                yielded = true;
                wheee_on = false;
                posing = false;
            }
            sleep_rest(period, tick);
            continue;
        }
        if yielded {
            tracing::warn!("the pad let go — the duck takes back over");
            yielded = false;
            brain.reset();
        }

        if off(&state.policy) {
            if idle_reason != Some("off") {
                tracing::warn!(
                    policy = %state.policy,
                    "the policy is not driving — waiting (Start on the pad turns it on)"
                );
                idle_reason = Some("off");
            }
            sleep_rest(period, tick);
            continue;
        }
        idle_reason = None;

        let sitting = seated(&state.policy);
        let busy = state.fallen || !(standing(&state.policy) || sitting);
        let obstacle = {
            let shared = shared.lock().unwrap_or_else(|p| p.into_inner());
            shared.model.summary(&state.sample)
        };
        let world = brain::World {
            busy,
            petting: state.hearing.is_some_and(|h| h.petting),
            heard,
            can_ground_pick: abilities.ground_pick,
            can_sit_stand: abilities.sit_stand,
            sitting,
            obstacle,
            pose: state.sample.pose,
        };
        let step = brain.step(dt, &world);

        if busy {
            // Something else owns the robot. A head target now would yank it mid-skill.
            sleep_rest(period, tick);
            continue;
        }

        // One write for the tick's continuous intents: they describe one instant.
        frame.clear();
        let mut push = |call: &proto::Call| -> std::io::Result<()> {
            serde_json::to_writer(&mut frame, &proto::Request::notify(call))?;
            frame.push(b'\n');
            Ok(())
        };
        push(&proto::Call::RobotMove(proto::MoveParams {
            vx: step.cmd[0],
            vy: step.cmd[1],
            vyaw: step.cmd[2],
        }))?;
        push(&proto::Call::RobotHead(proto::HeadParams {
            neck_pitch: step.head[0],
            // The brain's pitch is "positive = up"; the alpha's +head_pitch looks down.
            head_pitch: -step.head[1],
            head_yaw: step.head[2],
            head_roll: step.head[3],
            alpha: abilities.head_alpha.then_some(step.head_alpha),
        }))?;
        push(&proto::Call::RobotMouth(proto::MouthParams {
            open: step.mouth,
        }))?;
        // Body pose works on the velstand gait as well as on a standing network, so it goes out
        // whenever the brain poses, and once more with `active: false` when it stops.
        if step.body_active || posing {
            push(&proto::Call::RobotPose(proto::PoseParams {
                z: step.body[0],
                pitch: step.body[1],
                roll: step.body[2],
                active: step.body_active,
            }))?;
            posing = step.body_active;
        }
        if let Some(sound) = step.sound {
            quiet_until = tick + SELF_AUDIO;
            push(&proto::Call::RobotSound(proto::SoundParams {
                tag: match sound {
                    brain::Sound::Chirp => proto::SoundTag::Chirp,
                    brain::Sound::Coo => proto::SoundTag::Coo,
                },
                hold: None,
            }))?;
        }
        if step.wheee || wheee_on {
            if step.wheee {
                quiet_until = tick + SELF_AUDIO;
            }
            push(&proto::Call::RobotSound(proto::SoundParams {
                tag: proto::SoundTag::Wheee,
                hold: Some(step.wheee),
            }))?;
            wheee_on = step.wheee;
        }
        commands.write_all(&frame)?;
        commands.flush()?;

        if step.ground_pick {
            skill(commands, next_id, "ground_pick")?;
        }
        if step.sit_toggle {
            skill(commands, next_id, "sit_toggle")?;
        }

        sleep_rest(period, tick);
    }
}

fn sleep_rest(period: Duration, tick: Instant) {
    if let Some(remaining) = period.checked_sub(tick.elapsed()) {
        std::thread::sleep(remaining);
    }
}

/// Ask for a skill, answered: a refusal is logged with its reason by [`request`].
fn skill(commands: &mut UnixStream, next_id: &mut u64, name: &str) -> std::io::Result<()> {
    let call = proto::Call::RobotDo(proto::DoParams {
        skill: name.to_owned(),
    });
    request(commands, next_id, &call).map(|_| ())
}

/// Send a request and read its answer, on a connection that carries nothing else.
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

    let mut reader = BufReader::new(stream.try_clone()?);
    let mut answer = String::new();
    if reader.read_line(&mut answer)? == 0 {
        return Err(std::io::Error::other("robotd closed the connection"));
    }
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

    /// The marker is per boot: the same boot is remembered, a new one is not, and a machine that
    /// cannot say which boot it is on stands up every time rather than never.
    #[test]
    fn standing_up_is_remembered_per_boot() {
        let dir = std::env::temp_dir().join(format!("autod-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let marker = dir.join("started");
        assert!(!stood_up_on(&marker, Some("a")), "no marker yet");
        std::fs::write(&marker, "a\n").unwrap();
        assert!(stood_up_on(&marker, Some("a")));
        assert!(
            !stood_up_on(&marker, Some("b")),
            "a new boot stands up again"
        );
        assert!(!stood_up_on(&marker, None));
        std::fs::remove_dir_all(&dir).unwrap();
    }

    /// The labels the loop publishes, sorted into the three things the brain does about them.
    /// `held` is "no policy drove this tick" — the policy is off — and the brain must not treat it
    /// as busy-and-coming-back, or it would wait to drive a robot somebody switched off.
    #[test]
    fn policy_labels_sort_into_drive_wait_and_off() {
        for label in ["walk", "stand"] {
            assert!(standing(label) && !off(label), "{label}");
        }
        for label in ["held", "homing", "limp_fall", "limp_pose"] {
            assert!(off(label) && !standing(label), "{label}");
        }
        for label in ["ground_pick", "rise", "roulade", "roulade:unwind", "sit"] {
            assert!(!off(label) && !standing(label), "{label}");
        }
        // Seated is neither busy nor off: the nap runs there, and it must be able to stand back up.
        // The twin caught this one — reading `sit` as busy left a napping duck seated for good.
        assert!(seated("sit") && !seated("rise") && !seated("walk"));
    }
}
