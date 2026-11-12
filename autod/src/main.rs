//! `autod` — the autonomous brain, as an intent client.
//!
//! `docs/design/autonomous.md` is the design. In one paragraph: while `robotd`'s autonomy switch
//! is on (`robot.state.autonomous` — `[autonomous] enabled` at boot, `robot.autonomous` or an NFC
//! tag after), this stands the robot up with no pad, then reads `robot.state` and the head ToF and
//! sends ordinary intents, driven by a state machine (`brain.rs`) and an obstacle model
//! (`obstacle.rs`). It switches laser tracking and autonomous soccer on and off as the brain
//! wants, and stands aside while `robotd` drives the dot or the ball. It steps aside entirely
//! whenever `robot.state.move.source` says the pad is driving. Switched off, it touches nothing.
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
mod getup;
mod map;
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

    /// `mapd`'s socket. No `mapd`, or one not mapping, and the brain walks on odometry alone.
    #[arg(long, default_value = proto::socket::MAP)]
    map_socket: PathBuf,

    /// The robot's configuration, for `[pad_drive]`'s speeds.
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

/// After a fall, what `autod` is doing about the get-up — `getup.rs` decides, this acts.
enum Recovery {
    Idle,
    /// The policy is working the robot upright; `since` is when the watch started.
    Watching {
        g: getup::Getup,
        since: Instant,
    },
    /// The policy cut by `autod`, the robot settling into its init pose until `until`.
    Cut {
        g: getup::Getup,
        since: Instant,
        until: Instant,
    },
}

/// A laser or soccer switch `robotd` did not take is asked again after this, not every tick.
const SWITCH_RETRY: Duration = Duration::from_secs(3);

/// `map.stream` says something four times a second; silence this long is no map.
const MAP_STALE: Duration = Duration::from_secs(2);
/// A map pose is paired with the odometry sample nearest its `t_ns`, if one is this close.
const MAP_PAIR_NS: u64 = 60_000_000;
/// How often the grid is fetched while the map knows where the robot is. A few milliseconds of
/// `mapd`'s time and tens of kilobytes; the room does not change faster than this.
const GRID_EVERY: Duration = Duration::from_secs(20);
/// The grid's cell size for the brain: goals are judged at 15 cm from a wall, so finer buys
/// nothing.
const GRID_RES_M: f64 = 0.1;
/// How often the novelty grid is written to the state directory, while it is in the map's frame.
const VISITS_EVERY: Duration = Duration::from_secs(60);

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
    /// What `mapd` last said, and when.
    map: Option<(brain::MapFix, Instant)>,
    /// The newest grid, and a counter bumped with each, so the loop hands each one over once.
    grid: Option<(Arc<map::Grid>, u64)>,
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
    /// `robotd`'s autonomy switch: whether this process should be doing anything at all.
    autonomous: bool,
    laser: brain::Laser,
    ball: brain::Ball,
    /// Soccer mode is on at all — by hand or playing by itself.
    soccer_mode: bool,
    /// When this `robotd` started, whole seconds of `CLOCK_MONOTONIC`: which `robotd` it is. A new
    /// one comes up with the robot limp, which nobody chose.
    robotd_started_s: i64,
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
    spawn_map(args.map_socket.clone(), Arc::clone(&shared));

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
                autonomous: state.autonomous,
                laser: brain::Laser {
                    on: state.laser.is_some(),
                    dot: state.laser.as_ref().is_some_and(|l| l.target_m.is_some()),
                },
                ball: state.soccer.as_ref().filter(|s| s.autonomous).map_or_else(
                    brain::Ball::default,
                    |s| brain::Ball {
                        on: true,
                        seen: s.target_m.is_some(),
                        kicks: s.kicks,
                    },
                ),
                soccer_mode: state.soccer.is_some(),
                robotd_started_s: (state.t_ns as f64 / 1e9 - state.t).round() as i64,
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

/// Read `map.stream` into what the brain needs, forever, reconnecting while `mapd` is away; and
/// fetch the grid every [`GRID_EVERY`] while the map knows where the robot is.
fn spawn_map(socket: PathBuf, shared: Arc<Mutex<Shared>>) {
    let stream_socket = socket.clone();
    let stream_shared = Arc::clone(&shared);
    std::thread::spawn(move || {
        let mut warned = false;
        loop {
            if let Err(e) = map_stream(&stream_socket, &stream_shared)
                && !warned
            {
                tracing::info!(error = %e, "no map — walking on odometry alone");
                warned = true;
            }
            std::thread::sleep(Duration::from_secs(5));
        }
    });
    std::thread::spawn(move || {
        let mut seq = 0u64;
        loop {
            std::thread::sleep(GRID_EVERY);
            let known = shared
                .lock()
                .map(|s| matches!(s.map, Some((brain::MapFix::Known(_), _))))
                .unwrap_or(false);
            if !known {
                continue;
            }
            match fetch_grid(&socket) {
                Ok(Some(grid)) => {
                    seq += 1;
                    if let Ok(mut s) = shared.lock() {
                        s.grid = Some((Arc::new(grid), seq));
                    }
                }
                Ok(None) => {}
                Err(e) => tracing::debug!(error = %e, "no grid from mapd"),
            }
        }
    });
}

fn map_stream(socket: &Path, shared: &Mutex<Shared>) -> std::io::Result<()> {
    let mut stream = UnixStream::connect(socket)?;
    stream.write_all(&line_of(&proto::Request::call(
        proto::Id::Number(1),
        &proto::Call::MapStream,
    ))?)?;
    let mut reader = BufReader::new(stream);
    let mut raw = String::new();
    loop {
        raw.clear();
        if reader.read_line(&mut raw)? == 0 {
            if let Ok(mut s) = shared.lock() {
                s.map = None;
            }
            return Err(std::io::Error::other("mapd closed the map stream"));
        }
        let Some(state) = serde_json::from_str::<proto::Request>(&raw)
            .ok()
            .and_then(|r| r.as_map_state())
        else {
            continue;
        };
        let mut s = shared.lock().unwrap_or_else(|p| p.into_inner());
        let fix = match (state.pose, state.lost) {
            (Some(pose), _) => {
                // Paired with the odometry of the same instant; a pose that cannot be paired is
                // skipped rather than paired with the wrong one — the next comes in 250 ms.
                match s
                    .history
                    .nearest(state.t_ns)
                    .filter(|o| o.t_ns.abs_diff(state.t_ns) <= MAP_PAIR_NS)
                {
                    Some(odom) => {
                        brain::MapFix::Known(map::offset([pose.x, pose.y, pose.yaw], odom.pose))
                    }
                    None => continue,
                }
            }
            (None, Some(_)) => brain::MapFix::Lost,
            // Neither a pose nor a reason: `mapd` is not mapping.
            (None, None) => brain::MapFix::Off,
        };
        s.map = Some((fix, Instant::now()));
    }
}

fn fetch_grid(socket: &Path) -> std::io::Result<Option<map::Grid>> {
    let mut stream = UnixStream::connect(socket)?;
    stream.set_read_timeout(Some(Duration::from_secs(10)))?;
    let mut next_id = 1;
    let call = proto::Call::MapGrid(proto::MapGridParams {
        res_m: Some(GRID_RES_M),
    });
    Ok(request(&mut stream, &mut next_id, &call)?
        .and_then(|r| r.result_as::<proto::MapGridResult>().ok())
        .and_then(|g| map::Grid::of(&g)))
}

/// The novelty grid as it is kept on disk: map-frame cells, and when it was written.
#[derive(serde::Serialize, serde::Deserialize)]
struct SavedVisits {
    saved_at: u64,
    cells: Vec<(i32, i32, f32)>,
}

fn unix_now() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs())
}

fn save_visits(path: &Path, visits: Vec<((i32, i32), f32)>) {
    let saved = SavedVisits {
        saved_at: unix_now(),
        cells: visits.into_iter().map(|((i, j), w)| (i, j, w)).collect(),
    };
    let tmp = path.with_extension("tmp");
    let result = serde_json::to_vec(&saved)
        .map_err(std::io::Error::other)
        .and_then(|bytes| std::fs::write(&tmp, bytes))
        .and_then(|()| std::fs::rename(&tmp, path));
    if let Err(e) = result {
        tracing::debug!(error = %e, path = %path.display(), "visits not saved");
    }
}

/// The novelty grid from an earlier run, faded by the time since — the same 3 % per 30 s the brain
/// fades it by while running, so a place visited yesterday is new again.
fn load_visits(path: &Path) -> Vec<((i32, i32), f32)> {
    let Ok(bytes) = std::fs::read(path) else {
        return Vec::new();
    };
    let Ok(saved) = serde_json::from_slice::<SavedVisits>(&bytes) else {
        tracing::warn!(path = %path.display(), "unreadable visits — starting afresh");
        return Vec::new();
    };
    let fade = 0.97f32.powf(unix_now().saturating_sub(saved.saved_at) as f32 / 30.0);
    saved
        .cells
        .into_iter()
        .map(|(i, j, w)| ((i, j), w * fade))
        .filter(|(_, w)| *w > 0.05)
        .collect()
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

/// Which `robotd` stood the robot up: the boot, and when that `robotd` started on it.
fn instance(boot: &str, robotd_started_s: i64) -> String {
    format!("{boot} {robotd_started_s}")
}

/// Whether the marker says this very `robotd`, on this boot, was already stood up. A `robotd` that
/// restarted since (an update) came back limp by itself, so it is not the same one: the robot did
/// not stop because somebody stopped it. A marker from before `robotd` was part of it matches
/// nothing, which stands up once.
fn stood_up_on(marker: &Path, boot: Option<&str>, robotd_started_s: i64) -> bool {
    let (Ok(recorded), Some(boot)) = (std::fs::read_to_string(marker), boot) else {
        return false;
    };
    let mut parts = recorded.split_whitespace();
    let same_boot = parts.next() == Some(boot);
    // Two seconds of slack: `t` and `t_ns` are read a few microseconds apart, and rounding.
    let same_robotd = parts
        .next()
        .and_then(|s| s.parse::<i64>().ok())
        .is_some_and(|s| s.abs_diff(robotd_started_s) <= 2);
    same_boot && same_robotd
}

/// Stand the robot up: `robot.enable`, then wait until it is driving. `Ok(false)` when
/// `robotd` would not.
fn stand_up(
    commands: &mut UnixStream,
    next_id: &mut u64,
    shared: &Mutex<Shared>,
    policy: &str,
) -> std::io::Result<bool> {
    if !off(policy) {
        return Ok(true);
    }
    tracing::warn!("standing up — no pad needed (robot.enable)");
    let call = proto::Call::RobotEnable(proto::EnableParams {
        on: true,
        toggle: false,
    });
    let accepted = request(commands, next_id, &call)?
        .and_then(|r| r.result_as::<proto::IntentResult>().ok())
        .is_some_and(|r| r.accepted);
    if !accepted {
        return Ok(false);
    }
    let deadline = Instant::now() + STAND_UP;
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
    Ok(true)
}

/// The duck on its own was just switched on. Stands the robot up — except the first time this
/// process sees the switch on, with the same `robotd` it already stood the robot up with: that is
/// this unit alone restarting (a crash), and somebody may have stopped the robot since. A `robotd`
/// that restarted (an update restarts both) came back limp on its own, and is stood up again.
/// `docs/design/autonomous.md` §2.
fn switched_on(
    commands: &mut UnixStream,
    next_id: &mut u64,
    shared: &Mutex<Shared>,
    state_dir: &Path,
    state: &Latest,
    first_sight: bool,
) -> std::io::Result<()> {
    let marker = state_dir.join("started");
    let boot = boot_id();
    if first_sight && stood_up_on(&marker, boot.as_deref(), state.robotd_started_s) {
        tracing::warn!(
            policy = %state.policy,
            "already stood up with this robotd — leaving the policy as it is (Start on the pad, or \
             robotctl autonomous off then on, stands it up)"
        );
        return Ok(());
    }
    let mark = instance(boot.as_deref().unwrap_or_default(), state.robotd_started_s);
    // Not marked when refused: the next switch-on, or the next start of this unit, tries again.
    if stand_up(commands, next_id, shared, &state.policy)?
        && let Err(e) = std::fs::write(&marker, mark)
    {
        tracing::warn!(error = %e, path = %marker.display(), "cannot remember the stand-up");
    }
    Ok(())
}

/// Ask `robotd` for a switch, answered. A refusal is logged by [`request`].
fn switch(commands: &mut UnixStream, next_id: &mut u64, call: proto::Call) -> std::io::Result<()> {
    request(commands, next_id, &call).map(|_| ())
}

fn laser_switch(active: bool) -> proto::Call {
    proto::Call::RobotLaserTrack(proto::LaserTrackParams { active })
}

fn soccer_switch(on: bool) -> proto::Call {
    proto::Call::RobotSoccer(proto::SoccerParams {
        active: on,
        autonomous: on,
    })
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
    // The autonomy switch as last seen; `None` before the first state frame.
    let mut was_on: Option<bool> = None;
    // Autonomous soccer was switched on by the brain, so it is the brain's to switch off. A game
    // somebody else started (the pad's D-pad down) is theirs, and the brain stands aside for it.
    let mut soccer_mine = false;
    let mut external_soccer = false;
    // When each switch was last asked for, so a refusal is not asked again fifty times a second.
    let mut laser_asked: Option<Instant> = None;
    let mut soccer_asked: Option<Instant> = None;
    // After a fall: watching the get-up, or resetting it. See [`Recovery`].
    let mut recovery = Recovery::Idle;
    // The map: the last grid handed to the brain, and the novelty grid kept across reboots.
    let mut grid_seq = 0u64;
    let visits_path = args.state_dir.join("visits.json");
    let restored = load_visits(&visits_path);
    if !restored.is_empty() {
        tracing::info!(
            cells = restored.len(),
            "remembering where the duck has been"
        );
        brain.restore_visits(restored);
    }
    let mut visits_saved = Instant::now();

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

        // The switch. Edges only: switched on stands the robot up, switched off stops it and
        // hands back the camera; in between, off is silence.
        if was_on.is_none() && state.ball.on {
            // A game already running when this process starts is most likely its own, from before
            // a restart — nobody would know to switch it off otherwise. Adopted, so the brain
            // (not in BallPlay) switches it off.
            soccer_mine = true;
        }
        if was_on != Some(state.autonomous) {
            if state.autonomous {
                tracing::warn!("the duck is on its own");
                switched_on(
                    commands,
                    next_id,
                    shared,
                    &args.state_dir,
                    &state,
                    was_on.is_none(),
                )?;
                brain.reset();
            } else if was_on.is_some() {
                tracing::warn!("the duck on its own is switched off — standing still");
                for call in [
                    proto::Call::RobotMove(proto::MoveParams::default()),
                    proto::Call::RobotMouth(proto::MouthParams { open: 0.0 }),
                ] {
                    commands.write_all(&line_of(&proto::Request::notify(&call))?)?;
                }
                if posing {
                    let rest = proto::Call::RobotPose(proto::PoseParams {
                        active: false,
                        ..Default::default()
                    });
                    commands.write_all(&line_of(&proto::Request::notify(&rest))?)?;
                }
                commands.flush()?;
                if soccer_mine && state.soccer_mode {
                    switch(commands, next_id, soccer_switch(false))?;
                }
                if state.laser.on {
                    switch(commands, next_id, laser_switch(false))?;
                }
                soccer_mine = false;
                posing = false;
                wheee_on = false;
                recovery = Recovery::Idle;
            } else {
                tracing::info!(
                    "the duck on its own is off — waiting for it to be switched on \
                     (robotctl autonomous on, or an autonomous_mode tag)"
                );
            }
            was_on = Some(state.autonomous);
        }
        if !state.autonomous {
            sleep_rest(period, tick);
            continue;
        }

        // The pad is driving: say nothing at all, so it has the head, mouth and voice too.
        let pad_live = state.source.as_deref() == Some("padd") && !state.deadman;
        if pad_live {
            pad_at = Some(tick);
        }
        if pad_at.is_some_and(|at| tick.duration_since(at) < PAD_GRACE) {
            if !yielded {
                tracing::warn!("the pad is driving — standing aside");
                yielded = true;
                // Whatever the get-up was, the pad's person has the robot now.
                recovery = Recovery::Idle;
                wheee_on = false;
                posing = false;
                // A game the brain started would take the legs back the moment the stick is
                // centred: the pad's person did not ask for it.
                if soccer_mine {
                    switch(commands, next_id, soccer_switch(false))?;
                    soccer_mine = false;
                }
            }
            sleep_rest(period, tick);
            continue;
        }
        if yielded {
            tracing::warn!("the pad let go — the duck takes back over");
            yielded = false;
            brain.reset();
        }
        // Somebody else's ball game — the pad's D-pad down — is theirs until it ends.
        if state.ball.on && !soccer_mine {
            if !external_soccer {
                tracing::warn!("playing soccer by somebody else's say — standing aside");
                external_soccer = true;
                wheee_on = false;
                posing = false;
            }
            sleep_rest(period, tick);
            continue;
        }
        if external_soccer {
            tracing::warn!("the game is over — the duck takes back over");
            external_soccer = false;
            brain.reset();
        }

        // Down after a fall, with the policy trying to get it up. Given `getup.rs`'s timeout,
        // then the policy cut so the robot settles into its init pose, then given back for a
        // second try; after that, torque off and a call for help. Before the `off` check below:
        // during the cut, the policy is off because `autod` turned it off.
        let secs = |since: Instant| tick.duration_since(since).as_secs_f64();
        recovery = match recovery {
            Recovery::Idle if state.fallen && standing(&state.policy) => {
                tracing::warn!("down — the policy is getting the duck up");
                Recovery::Watching {
                    g: getup::Getup::new(getup::Timing::default()),
                    since: tick,
                }
            }
            Recovery::Idle => Recovery::Idle,
            // Somebody else turned the policy off (Start, Select): theirs now.
            Recovery::Watching { .. } if matches!(state.policy.as_str(), "held" | "homing") => {
                tracing::warn!(policy = %state.policy, "the policy went off during the get-up — leaving it");
                Recovery::Idle
            }
            Recovery::Watching { mut g, since } => {
                // A robot going over again mid-try is down, whatever the label says.
                match g.tick(secs(since), state.fallen || !standing(&state.policy)) {
                    getup::Verdict::Waiting => Recovery::Watching { g, since },
                    getup::Verdict::GotUp {
                        seconds,
                        after_reset,
                    } => {
                        tracing::warn!(seconds, after_reset, "up again");
                        brain.reset();
                        Recovery::Idle
                    }
                    getup::Verdict::Reset => {
                        tracing::warn!(
                            timeout_s = g.timing.timeout,
                            reset_s = g.timing.reset,
                            "still down — back to the init pose, then trying again"
                        );
                        let off = proto::Call::RobotEnable(proto::EnableParams {
                            on: false,
                            toggle: false,
                        });
                        switch(commands, next_id, off)?;
                        let until = tick + Duration::from_secs_f64(g.timing.reset);
                        Recovery::Cut { g, since, until }
                    }
                    getup::Verdict::GaveUp => {
                        tracing::warn!(
                            "still down after the reset — torque off; it needs a hand (Start on the \
                             pad once it is back on its feet)"
                        );
                        switch(commands, next_id, proto::Call::RobotRelax)?;
                        let ask = proto::Call::RobotSound(proto::SoundParams {
                            tag: proto::SoundTag::Inquire,
                            hold: None,
                        });
                        commands.write_all(&line_of(&proto::Request::notify(&ask))?)?;
                        commands.flush()?;
                        Recovery::Idle
                    }
                }
            }
            Recovery::Cut { g, since, until } if tick < until => Recovery::Cut { g, since, until },
            Recovery::Cut { mut g, since, .. } => {
                let on = proto::Call::RobotEnable(proto::EnableParams {
                    on: true,
                    toggle: false,
                });
                switch(commands, next_id, on)?;
                g.resumed(secs(since));
                Recovery::Watching { g, since }
            }
        };
        if !matches!(recovery, Recovery::Idle) {
            sleep_rest(period, tick);
            continue;
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
        let fix = {
            let mut s = shared.lock().unwrap_or_else(|p| p.into_inner());
            if let Some((grid, seq)) = s.grid.as_ref()
                && *seq != grid_seq
            {
                grid_seq = *seq;
                brain.set_grid(Arc::clone(grid));
            }
            match s.map {
                Some((fix, at)) if tick.duration_since(at) < MAP_STALE => fix,
                Some(_) => {
                    s.map = None;
                    brain::MapFix::Off
                }
                None => brain::MapFix::Off,
            }
        };
        if tick.duration_since(visits_saved) >= VISITS_EVERY {
            visits_saved = tick;
            if let Some(visits) = brain.visits_in_map() {
                save_visits(&visits_path, visits);
            }
        }
        let world = brain::World {
            busy,
            petting: state.hearing.is_some_and(|h| h.petting),
            heard,
            can_ground_pick: abilities.ground_pick,
            can_sit_stand: abilities.sit_stand,
            sitting,
            obstacle,
            pose: state.sample.pose,
            laser: state.laser,
            ball: state.ball,
            map: fix,
        };
        let step = brain.step(dt, &world);

        if busy {
            // Something else owns the robot. A head target now would yank it mid-skill.
            sleep_rest(period, tick);
            continue;
        }

        // The camera's two jobs, held to what the brain wants. Soccer first: switching it off is
        // what lets the laser come back on (`robotd` runs one at a time).
        let retry =
            |asked: Option<Instant>| asked.is_none_or(|at| tick.duration_since(at) > SWITCH_RETRY);
        if step.soccer && !state.ball.on && retry(soccer_asked) {
            tracing::info!("looking for the ball (autonomous soccer on)");
            switch(commands, next_id, soccer_switch(true))?;
            soccer_mine = true;
            soccer_asked = Some(tick);
        } else if !step.soccer && soccer_mine {
            if state.soccer_mode {
                switch(commands, next_id, soccer_switch(false))?;
            }
            soccer_mine = false;
        }
        if step.laser != state.laser.on && !step.soccer && retry(laser_asked) {
            tracing::info!(on = step.laser, "laser tracking");
            switch(commands, next_id, laser_switch(step.laser))?;
            laser_asked = Some(tick);
        }

        // One write for the tick's continuous intents: they describe one instant.
        frame.clear();
        let mut push = |call: &proto::Call| -> std::io::Result<()> {
            serde_json::to_writer(&mut frame, &proto::Request::notify(call))?;
            frame.push(b'\n');
            Ok(())
        };
        // While `robotd` chases the dot or the ball, the twist and the head go out as exact zeros:
        // `robotd` reads a zero twist and a centred head as nobody asking, and gives both to the
        // chase at once — where staying silent would leave the last nonzero command winning until
        // it went stale. The quack's head toss stays out of it for the same reason.
        let head = if step.hands_off { [0.0; 4] } else { step.head };
        push(&proto::Call::RobotMove(proto::MoveParams {
            vx: step.cmd[0],
            vy: step.cmd[1],
            vyaw: step.cmd[2],
        }))?;
        push(&proto::Call::RobotHead(proto::HeadParams {
            neck_pitch: head[0],
            // The brain's pitch is "positive = up"; the alpha's +head_pitch looks down.
            head_pitch: -head[1],
            head_yaw: head[2],
            head_roll: head[3],
            alpha: (abilities.head_alpha && !step.hands_off).then_some(step.head_alpha),
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

/// One notification or request as the line it goes out as.
fn line_of(request: &proto::Request) -> std::io::Result<Vec<u8>> {
    let mut line = serde_json::to_vec(request)?;
    line.push(b'\n');
    Ok(line)
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

    /// The marker is per boot and per `robotd`: the same pair is remembered; a new boot, or a
    /// `robotd` restarted since (an update), stands up again; and a machine that cannot say which
    /// boot it is on stands up every time rather than never.
    #[test]
    fn standing_up_is_remembered_per_robotd() {
        let dir = std::env::temp_dir().join(format!("autod-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let marker = dir.join("started");
        assert!(!stood_up_on(&marker, Some("a"), 100), "no marker yet");
        std::fs::write(&marker, instance("a", 100)).unwrap();
        assert!(stood_up_on(&marker, Some("a"), 100));
        assert!(stood_up_on(&marker, Some("a"), 101), "rounding slack");
        assert!(
            !stood_up_on(&marker, Some("a"), 640),
            "robotd restarted: stands up again"
        );
        assert!(
            !stood_up_on(&marker, Some("b"), 100),
            "a new boot stands up again"
        );
        assert!(!stood_up_on(&marker, None, 100));
        std::fs::write(&marker, "a\n").unwrap();
        assert!(
            !stood_up_on(&marker, Some("a"), 100),
            "a marker from before robotd was in it matches nothing"
        );
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
