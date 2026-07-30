//! `robotd` — the robot control daemon.
//!
//! **Slice 1** (`docs/robotd-design.md` §4): a control loop that drives the real bus at the
//! real rate and holds the pose it started in. No observations, no policy, no intents.
//!
//! It is not walking yet because that is not what it is for yet. The update engine is
//! finished and has never run on hardware, and its auto-rollback is only meaningful if
//! `robot.health` means something — today it means "the loop ticked once", so every
//! rollback tested so far tested a placeholder. Slice 1 makes health honest: **the loop is
//! meeting its deadline**. A loop running at 60% of target is alive, answers every request,
//! and is badly broken.
//!
//! Holding a pose is also the right thing to be doing while someone deliberately breaks
//! releases at a bench: the bus sees the real load at the real rate, and nothing falls over
//! when a bad build lands.
//!
//! Every one of the four methods must be answerable *while the robot is in a bad state*,
//! since that is exactly when it is asked. So the IPC side reads atomics the control loop
//! publishes and never calls into the loop — a wedged loop reports itself unhealthy rather
//! than hanging the caller.

mod control;
mod intents;
mod params;

use std::path::PathBuf;
use std::process::ExitCode;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::time::{Duration, Instant};

use arc_swap::ArcSwapOption;
use clap::{Parser, Subcommand};
use duck_control::io::RobotIo;
use duck_control::policy::{DEFAULT_STANDING_THRESHOLD, Policy};
use duck_control::safety::{Safety, SafetyConfig};
use duck_control::{DEFAULT_POSITION, FakeIo, NUM_JOINTS};
use duck_ipc_proto as proto;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::{UnixListener, UnixStream};

use control::{Controller, Tuning};
use intents::Intents;
use params::Params;

/// Model API version this build implements (`updater-design.md` §5.5). Bump when the
/// sensor-input / actuator-output contract a model sees changes.
const MODEL_API: u32 = 1;

/// Socket mode. Same reasoning as `updaterd`'s: the group decides who may ask.
const SOCKET_MODE: u32 = 0o660;

const MAX_LINE: usize = 64 * 1024;

/// How often the loop logs a summary at `info`.
///
/// Per-tick logging would be ~4.3M lines a day at 50 Hz. That is not merely noise: under a
/// journal size cap it is what *evicts* the logs support needs.
const LOOP_SUMMARY_INTERVAL: Duration = Duration::from_secs(300);

/// Window over which the achieved rate is measured, and therefore how quickly a degraded
/// loop becomes visible to the health gate.
const RATE_WINDOW: Duration = Duration::from_secs(1);

#[derive(Parser, Debug)]
#[command(name = "robotd", about = "Robot control daemon", version)]
struct Args {
    /// Socket to serve the `robot.*` API on. `updaterd --robot-socket` must match.
    #[arg(long, default_value = "/run/robotd.sock")]
    socket: PathBuf,

    /// Params file. Defaults to `/etc/robot/robotd.toml`, which may be absent — an
    /// unprovisioned board comes up on defaults. A path given here must exist.
    #[arg(long)]
    params: Option<PathBuf>,

    /// Serial port override, for a board wired differently from the shipped default.
    #[arg(long)]
    port: Option<String>,

    /// Run against a robot made of nothing. For laptop development and tests — there is no
    /// simulator yet, and this is what stands in for one.
    #[arg(long)]
    fake: bool,

    /// Do not load a policy: run the loop and hold the startup pose.
    ///
    /// Distinct from a policy that failed to load, which is unhealthy. This is the
    /// configuration to use when the thing under test is the updater rather than the gait —
    /// nothing falls over when a deliberately broken release lands.
    #[arg(long)]
    no_policy: bool,

    /// Report unhealthy. For exercising the updater's rollback path on a bench robot
    /// without having to break a real build.
    #[arg(long)]
    unhealthy: bool,

    /// Report that it is not safe to restart, as if the robot were moving.
    #[arg(long)]
    busy: bool,

    #[command(subcommand)]
    command: Option<Command>,
}

#[derive(Subcommand, Debug)]
enum Command {
    /// Enable torque and move to the home pose, then exit.
    ///
    /// Explicit, and separate from running the daemon, because the control loop must never
    /// move the robot on its own: `robotd` restarting during an update would otherwise make
    /// a standing robot lurch, which is both a fall risk and a confounder when the thing
    /// under test is the updater.
    Init {
        #[arg(long, default_value = "2s", value_parser = parse_duration)]
        duration: Duration,
    },
}

fn parse_duration(raw: &str) -> Result<Duration, String> {
    let (value, scale) = match raw.strip_suffix("ms") {
        Some(v) => (v, 1u64),
        None => match raw.strip_suffix('s') {
            Some(v) => (v, 1000),
            None => (raw, 1000),
        },
    };
    value
        .parse::<u64>()
        .map(|n| Duration::from_millis(n * scale))
        .map_err(|_| format!("expected e.g. 500ms or 2s, got {raw:?}"))
}

/// What the control loop publishes about itself.
///
/// Atomics rather than a mutex on purpose: the IPC side must never be able to block on the
/// control loop. A robot whose loop is wedged still has to be able to say "I am not
/// healthy" — if answering required the loop's lock, the one situation where `updaterd`
/// needs an answer is the situation it would hang in.
struct RobotState {
    /// Epoch for every timestamp below. `Instant` so the clock cannot go backwards.
    started: Instant,
    ticks: AtomicU64,
    /// Ticks whose work overran the period. Cumulative, for diagnosis; the rate check is
    /// what catches a sustained problem.
    missed: AtomicU64,
    /// Microseconds since `started` at the last completed tick.
    last_tick_us: AtomicU64,
    /// Achieved rate over the last window, as `f64::to_bits`. Zero until the first window
    /// closes, which is why the stall check carries the first second.
    achieved_hz: AtomicU64,
    /// Consecutive failed bus reads. Reset by any success.
    consecutive_errors: AtomicU32,
    shutdown: AtomicBool,
    /// Why the policy is not loaded, if it is not. Set once at startup; the loop keeps
    /// running and holds the pose, so a broken bundle is a rollback rather than a crash.
    policy_error: ArcSwapOption<String>,
    /// Published by the loop so the IPC side can answer without consulting it.
    fallen: AtomicBool,
    /// The policy is driving and has been asked for a non-zero velocity.
    moving: AtomicBool,

    period_us: u64,
    min_achieved_hz: f64,
    stall_periods: u32,
    max_consecutive_errors: u32,
    force_unhealthy: bool,
    force_busy: bool,
}

impl RobotState {
    fn new(params: &Params, force_unhealthy: bool, force_busy: bool) -> Self {
        Self {
            started: Instant::now(),
            ticks: AtomicU64::new(0),
            missed: AtomicU64::new(0),
            last_tick_us: AtomicU64::new(0),
            achieved_hz: AtomicU64::new(0),
            consecutive_errors: AtomicU32::new(0),
            shutdown: AtomicBool::new(false),
            policy_error: ArcSwapOption::empty(),
            fallen: AtomicBool::new(false),
            moving: AtomicBool::new(false),
            period_us: params.period().as_micros() as u64,
            min_achieved_hz: params.health.min_achieved_hz,
            stall_periods: params.health.stall_periods,
            max_consecutive_errors: params.health.max_consecutive_errors,
            force_unhealthy,
            force_busy,
        }
    }

    fn health(&self) -> proto::HealthResult {
        let unhealthy = |reason: String| proto::HealthResult {
            healthy: false,
            reason: Some(reason),
        };

        if self.force_unhealthy {
            return unhealthy("forced unhealthy by --unhealthy".into());
        }

        // "Starting" is not "started". The gate polls, so it will see the transition.
        if self.ticks.load(Ordering::Relaxed) == 0 {
            return unhealthy("control loop has not completed a cycle yet".into());
        }

        // A daemon that came up but cannot run its policy is not healthy, however well the
        // loop is ticking. This is what makes the updater roll back a release whose bundle
        // is wrong, instead of leaving a robot that holds a pose and never walks again.
        if let Some(reason) = self.policy_error.load_full() {
            return unhealthy(format!("policy unavailable: {reason}"));
        }

        let errors = self.consecutive_errors.load(Ordering::Relaxed);
        if errors >= self.max_consecutive_errors {
            return unhealthy(format!("{errors} consecutive bus read failures"));
        }

        // A wedged loop stops stamping. This is what turns a hung control thread into an
        // honest answer instead of a socket that keeps saying "healthy" forever.
        let now_us = self.started.elapsed().as_micros() as u64;
        let stale_us = now_us.saturating_sub(self.last_tick_us.load(Ordering::Relaxed));
        let stall_limit_us = self.period_us.saturating_mul(self.stall_periods as u64);
        if stale_us > stall_limit_us {
            return unhealthy(format!("control loop stalled for {} ms", stale_us / 1000));
        }

        let hz = f64::from_bits(self.achieved_hz.load(Ordering::Relaxed));
        if hz > 0.0 && hz < self.min_achieved_hz {
            return unhealthy(format!(
                "control loop at {hz:.1} Hz, below the {:.1} Hz floor",
                self.min_achieved_hz
            ));
        }

        proto::HealthResult {
            healthy: true,
            reason: None,
        }
    }

    fn safe_to_restart(&self) -> proto::SafeToRestartResult {
        if self.force_busy {
            return proto::SafeToRestartResult {
                safe: false,
                reason: Some("forced busy by --busy".into()),
            };
        }
        // Restarting motor control mid-stride is how a robot falls over
        // (`updater-design.md` §7.2). A robot that is merely standing, or already down, is
        // safe to interrupt — it is going nowhere either way.
        if self.moving.load(Ordering::Relaxed) && !self.fallen.load(Ordering::Relaxed) {
            return proto::SafeToRestartResult {
                safe: false,
                reason: Some("the robot is walking".into()),
            };
        }
        proto::SafeToRestartResult {
            safe: true,
            reason: None,
        }
    }
}

/// The first line each daemon logs, before anything that can fail.
///
/// At `warn` so it survives `RUST_LOG=warn` on a long-running board: identifying the running
/// build is not a debug-level concern. `exe` is here because after an update `updaterd` is
/// still running the *previous* binary by design, so which release directory a process came
/// from cannot be inferred (`docs/architecture.md` §8).
fn log_startup_identity(service: &str) {
    tracing::warn!(
        service,
        build = %proto::build_info!(),
        exe = %std::env::current_exe()
            .map(|p| p.display().to_string())
            .unwrap_or_else(|_| "unknown".into()),
        pid = std::process::id(),
        "starting"
    );
}

#[tokio::main]
async fn main() -> ExitCode {
    // Rust ignores SIGPIPE, which turns `robotd ... | head` into a panic.
    unsafe { libc::signal(libc::SIGPIPE, libc::SIG_DFL) };

    let args = Args::parse();

    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_writer(std::io::stderr)
        .init();

    log_startup_identity("robotd");

    let explicit = args.params.is_some();
    let params_path = args
        .params
        .clone()
        .unwrap_or_else(|| PathBuf::from(params::DEFAULT_PATH));
    let mut params = match Params::load(&params_path, explicit) {
        Ok(params) => params,
        Err(e) => {
            tracing::error!(error = %e, "bad params");
            return ExitCode::FAILURE;
        }
    };
    if let Some(port) = args.port.clone() {
        params.bus.port = port;
    }
    if args.no_policy {
        params.policy.enabled = false;
    }

    if let Some(Command::Init { duration }) = args.command {
        return run_init(&params, duration);
    }

    let state = Arc::new(RobotState::new(&params, args.unhealthy, args.busy));

    if args.unhealthy {
        tracing::warn!("--unhealthy: will report unhealthy, so updates will roll back");
    }
    if args.busy {
        tracing::warn!("--busy: will refuse restarts, so updates will be held off");
    }

    let intents = Arc::new(Intents::new());

    let control =
        match spawn_control_thread(&args, &params, Arc::clone(&state), Arc::clone(&intents)) {
            Ok(handle) => handle,
            Err(e) => {
                tracing::error!(error = %e, "cannot start the control loop");
                return ExitCode::FAILURE;
            }
        };

    let serving = serve(
        Arc::clone(&state),
        Arc::clone(&intents),
        args.socket.clone(),
    );
    let mut code = ExitCode::SUCCESS;
    tokio::select! {
        result = serving => {
            if let Err(e) = result {
                tracing::error!(error = %e, "IPC server stopped");
                code = ExitCode::FAILURE;
            }
        }
        _ = shutdown() => tracing::info!("shutting down"),
    }

    // Ask the loop to stop and let it finish the tick it is in, rather than aborting
    // mid-transaction and leaving a half-written packet on the bus.
    state.shutdown.store(true, Ordering::Relaxed);
    let _ = control.join();
    let _ = std::fs::remove_file(&args.socket);
    code
}

/// Enable torque and ramp to the home pose.
#[cfg(target_os = "linux")]
fn run_init(params: &Params, duration: Duration) -> ExitCode {
    let mut io = match duck_control::bus::DynamixelIo::open(&params.bus.port) {
        Ok(io) => io,
        Err(e) => {
            tracing::error!(error = %e, port = %params.bus.port, "cannot open the bus");
            return ExitCode::FAILURE;
        }
    };
    if let Err(e) = io.check_registers() {
        tracing::error!(error = %e, "motor register check failed");
        return ExitCode::FAILURE;
    }
    if let Err(e) = io.set_torque(true) {
        tracing::error!(error = %e, "cannot enable torque");
        return ExitCode::FAILURE;
    }
    if let Err(e) = io.interpolate_to(&DEFAULT_POSITION, duration, Duration::from_millis(20)) {
        tracing::error!(error = %e, "interpolation to the home pose failed");
        return ExitCode::FAILURE;
    }
    tracing::warn!(?duration, "at home pose, torque enabled");
    ExitCode::SUCCESS
}

#[cfg(not(target_os = "linux"))]
fn run_init(_params: &Params, _duration: Duration) -> ExitCode {
    tracing::error!("init needs a real bus; this build is not on the robot");
    ExitCode::FAILURE
}

/// Start the control loop on its own OS thread, with its own current-thread runtime.
///
/// Its own thread because the bus read is *blocking* serial I/O — on a shared runtime it
/// would occupy a worker for the duration of every transaction. Its own runtime so IPC work
/// can never be scheduled in front of a tick. This mirrors the prototype, where the loop
/// likewise had a runtime to itself and everything else lived on threads.
fn spawn_control_thread(
    args: &Args,
    params: &Params,
    state: Arc<RobotState>,
    intents: Arc<Intents>,
) -> std::io::Result<std::thread::JoinHandle<()>> {
    let period = params.period();
    let fake = args.fake;
    let port = params.bus.port.clone();
    let params = params.clone();

    std::thread::Builder::new()
        .name("control".into())
        .spawn(move || {
            let runtime = match tokio::runtime::Builder::new_current_thread()
                .enable_time()
                .build()
            {
                Ok(runtime) => runtime,
                Err(e) => {
                    tracing::error!(error = %e, "cannot build the control runtime");
                    return;
                }
            };

            if fake {
                tracing::warn!("--fake: no bus, no robot");
                runtime.block_on(control_loop(
                    FakeIo::at(DEFAULT_POSITION),
                    state,
                    intents,
                    params,
                    period,
                ));
                return;
            }

            if let Some(io) = open_bus(&port) {
                runtime.block_on(control_loop(io, state, intents, params, period));
            }
        })
}

/// Open and verify the bus, or explain why not.
#[cfg(target_os = "linux")]
fn open_bus(port: &str) -> Option<duck_control::bus::DynamixelIo> {
    let mut io = match duck_control::bus::DynamixelIo::open(port) {
        Ok(io) => io,
        Err(e) => {
            tracing::error!(error = %e, port, "cannot open the bus");
            return None;
        }
    };
    match io.check_registers() {
        Ok(0) => tracing::info!("motor registers already correct"),
        Ok(n) => tracing::warn!(corrected = n, "motor registers corrected"),
        Err(e) => {
            tracing::error!(error = %e, "motor register check failed");
            return None;
        }
    }
    Some(io)
}

#[cfg(not(target_os = "linux"))]
fn open_bus(_port: &str) -> Option<FakeIo> {
    tracing::error!("no bus on this platform; use --fake");
    None
}

/// The tick.
///
/// ```text
/// read → observe (fall) → gate (deadman) → policy → safety.apply
/// ```
///
/// Safety holds the only `RobotIo`, so everything above it can propose targets and nothing
/// above it can command a motor.
///
/// A policy that failed to load is survivable, and deliberately so: the loop keeps running
/// at rate, holds its pose, and `robot.health` says why. The updater then rolls the release
/// back. The alternative — refusing to start — becomes a crashloop under
/// `Restart=always` and reaches the health gate as `Unreachable`, which blames the wrong
/// thing in the journal.
async fn control_loop<T: RobotIo>(
    io: T,
    state: Arc<RobotState>,
    intents: Arc<Intents>,
    params: Params,
    period: Duration,
) {
    let mut safety = Safety::new(
        io,
        SafetyConfig {
            fall_gravity_z: params.safety.fall_gravity_z,
            fall_debounce: Duration::from_millis(params.safety.fall_debounce_ms),
            deadman: Duration::from_millis(params.safety.deadman_ms),
            gain_running: params.policy.gain,
            gain_limp: params.safety.gain_limp,
        },
    );

    // Adopt the pose the robot is already in. Never move on start: the servos hold their
    // last commanded goal while this process is dead, so a restart mid-update leaves a
    // standing robot standing, with no gap.
    let mut hold = match safety.read() {
        Ok(sensors) => sensors.positions,
        Err(e) => {
            tracing::error!(error = %e, "first bus read failed; not commanding anything");
            return;
        }
    };

    // A policy that was *not wanted* is healthy; one that was wanted and could not be
    // loaded is not. Collapsing those two would either make a bench robot look broken or
    // let a release with an unusable bundle pass the health gate.
    let mut controller = if !params.policy.enabled {
        tracing::warn!("policy disabled; holding the startup pose");
        None
    } else {
        let tuning = Tuning {
            action_scale: params.policy.action_scale,
            standing_action_scale: params.policy.standing_action_scale,
            standing_gain_ratio: params.policy.standing_gain_ratio,
            gain: params.policy.gain,
            head_lowpass: params.policy.head_lowpass,
            legs_lowpass: params.policy.legs_lowpass,
        };
        match Policy::load(
            &params.policy.walk,
            params.policy.stand.as_deref(),
            DEFAULT_STANDING_THRESHOLD,
        ) {
            Ok(policy) => {
                tracing::warn!(
                    walk = %params.policy.walk.display(),
                    stand = ?params.policy.stand.as_ref().map(|p| p.display().to_string()),
                    "policy loaded"
                );
                Some(Controller::new(policy, tuning))
            }
            Err(e) => {
                tracing::error!(error = %e, "policy unavailable; holding the pose");
                state.policy_error.store(Some(Arc::new(e.to_string())));
                None
            }
        }
    };

    tracing::warn!(
        joints = NUM_JOINTS,
        hz = 1.0 / period.as_secs_f64(),
        driving = controller.is_some(),
        "control loop running"
    );

    let mut ticker = tokio::time::interval(period);
    // Skipped ticks must not be replayed in a burst: a loop that fell behind should continue
    // at its target rate, not fire the backlog back to back and stack motor commands.
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

    let mut window_start = Instant::now();
    let mut window_ticks = 0u64;
    let mut last_summary = Instant::now();
    let mut was_driving = false;

    while !state.shutdown.load(Ordering::Relaxed) {
        ticker.tick().await;
        let tick_start = Instant::now();

        let sensors = match safety.read() {
            Ok(sensors) => {
                state.consecutive_errors.store(0, Ordering::Relaxed);
                Some(sensors)
            }
            Err(e) => {
                let n = state.consecutive_errors.fetch_add(1, Ordering::Relaxed) + 1;
                // One dropped transaction is ordinary on a serial bus; a run of them is not.
                // Log the first and then every tenth, so a persistent fault is visible
                // without a wall of identical lines.
                if n == 1 || n.is_multiple_of(10) {
                    tracing::warn!(error = %e, consecutive = n, "bus read failed");
                }
                None
            }
        };

        if let Some(sensors) = sensors.as_ref() {
            safety.observe(sensors, period);
        }
        state.fallen.store(safety.fallen(), Ordering::Relaxed);

        let snapshot = intents.snapshot();
        let (command, _deadman) = safety.gate(snapshot.command, snapshot.twist_age);

        // Drive only with a sample to drive from: a tick whose read failed has no
        // observation to build, and inventing one would feed the policy a stale robot.
        let driving =
            snapshot.enabled && controller.is_some() && !safety.fallen() && sensors.is_some();

        if driving && !was_driving {
            // Starting fresh: a stale previous action in the observation, or a filter
            // anchored to where the robot was a minute ago, would both show up as a lurch.
            if let Some(controller) = controller.as_mut() {
                controller.reset();
            }
        }
        if was_driving && !driving {
            // Freeze where it is rather than snapping back to the startup pose. Captured
            // once, not re-read each tick, or the hold target would sag under gravity.
            if let Some(sensors) = sensors.as_ref() {
                hold = sensors.positions;
            }
        }
        was_driving = driving;

        let (targets, gain, moving) = match (driving, sensors.as_ref()) {
            (true, Some(sensors)) => {
                let controller = controller.as_mut().expect("driving implies a controller");
                match controller.step(sensors, &command) {
                    Ok(step) => (step.targets, step.gain, command.twist_magnitude() > 0.0),
                    Err(e) => {
                        tracing::warn!(error = %e, "inference failed; holding");
                        (hold, params.policy.gain, false)
                    }
                }
            }
            _ => (hold, params.policy.gain, false),
        };
        state.moving.store(moving, Ordering::Relaxed);

        if let Err(e) = safety.apply(targets, hold, gain) {
            tracing::warn!(error = %e, "bus write failed");
        }

        let ticks = state.ticks.fetch_add(1, Ordering::Relaxed) + 1;
        state.last_tick_us.store(
            state.started.elapsed().as_micros() as u64,
            Ordering::Relaxed,
        );
        if tick_start.elapsed() > period {
            state.missed.fetch_add(1, Ordering::Relaxed);
        }

        window_ticks += 1;
        let window = window_start.elapsed();
        if window >= RATE_WINDOW {
            let hz = window_ticks as f64 / window.as_secs_f64();
            state.achieved_hz.store(hz.to_bits(), Ordering::Relaxed);
            window_start = Instant::now();
            window_ticks = 0;

            if last_summary.elapsed() >= LOOP_SUMMARY_INTERVAL {
                tracing::info!(
                    total = ticks,
                    hz = format!("{hz:.1}"),
                    missed = state.missed.load(Ordering::Relaxed),
                    driving,
                    fallen = safety.fallen(),
                    "control loop"
                );
                last_summary = Instant::now();
            }
        }
    }
    tracing::info!("control loop stopped");
}

async fn serve(
    state: Arc<RobotState>,
    intents: Arc<Intents>,
    socket_path: PathBuf,
) -> std::io::Result<()> {
    // A leftover socket from a killed process must not stop us coming up.
    if socket_path.exists() {
        tracing::warn!(path = %socket_path.display(), "removing stale socket");
        let _ = std::fs::remove_file(&socket_path);
    }
    if let Some(parent) = socket_path.parent() {
        let _ = std::fs::create_dir_all(parent);
    }

    let listener = UnixListener::bind(&socket_path)?;

    use std::os::unix::fs::PermissionsExt;
    std::fs::set_permissions(&socket_path, std::fs::Permissions::from_mode(SOCKET_MODE))?;

    tracing::info!(
        path = %socket_path.display(),
        mode = format!("{SOCKET_MODE:o}"),
        model_api = MODEL_API,
        "serving robot IPC"
    );

    loop {
        let (stream, _) = match listener.accept().await {
            Ok(pair) => pair,
            Err(e) => {
                tracing::warn!(error = %e, "accept failed");
                continue;
            }
        };
        let state = Arc::clone(&state);
        let intents = Arc::clone(&intents);
        tokio::spawn(async move {
            if let Err(e) = handle(state, intents, stream).await {
                tracing::debug!(error = %e, "connection ended");
            }
        });
    }
}

async fn handle(
    state: Arc<RobotState>,
    intents: Arc<Intents>,
    stream: UnixStream,
) -> std::io::Result<()> {
    let (read_half, mut write_half) = stream.into_split();
    let mut lines = BufReader::new(read_half).lines();

    while let Some(line) = lines.next_line().await? {
        if line.trim().is_empty() {
            continue;
        }
        if line.len() > MAX_LINE {
            let response = proto::Response::err(
                None,
                proto::Error::new(proto::code::INVALID_REQUEST, "request too large"),
            );
            write_line(&mut write_half, &response).await?;
            continue;
        }

        let request: proto::Request = match serde_json::from_str(&line) {
            Ok(request) => request,
            Err(e) => {
                let response = proto::Response::err(
                    None,
                    proto::Error::new(proto::code::PARSE_ERROR, e.to_string()),
                );
                write_line(&mut write_half, &response).await?;
                continue;
            }
        };

        let call = request.as_call();

        // Notifications get no reply, per the spec. Continuous intents arrive this way —
        // at 50 Hz a response per message would be pure overhead, and there is nothing
        // useful to say about a velocity that is superseded 20 ms later.
        let Some(id) = request.id.clone() else {
            if let Ok(call) = call {
                apply_intent(&intents, &call);
            }
            continue;
        };

        let response = match call {
            Ok(call) => dispatch(&state, &intents, id, &call),
            Err(e) => proto::Response::err(Some(id), e),
        };
        write_line(&mut write_half, &response).await?;
    }
    Ok(())
}

/// Answer one request.
///
/// Synchronous and allocation-light on purpose: these answers must be available even when
/// everything else is broken.
/// Apply a continuous intent. Shared by the notification path and the request path, so a
/// client that sends `robot.move` with an `id` is not silently ignored — the spec permits
/// either, and refusing one because of a framing choice would be a surprise.
fn apply_intent(intents: &Intents, call: &proto::Call) -> bool {
    match call {
        proto::Call::RobotMove(p) => {
            intents.set_twist([p.vx, p.vy, p.vyaw]);
            true
        }
        proto::Call::RobotHead(p) => {
            intents.set_head([p.neck_pitch, p.head_pitch, p.head_yaw, p.head_roll]);
            true
        }
        _ => false,
    }
}

fn dispatch(
    state: &RobotState,
    intents: &Intents,
    id: proto::Id,
    call: &proto::Call,
) -> proto::Response {
    match call {
        proto::Call::RobotMove(_) | proto::Call::RobotHead(_) => {
            apply_intent(intents, call);
            proto::Response::ok(Some(id), &proto::IntentResult::accepted())
        }

        proto::Call::RobotStop => {
            intents.stop();
            proto::Response::ok(Some(id), &proto::IntentResult::accepted())
        }

        // Refusing to enable a fallen robot is a normal answer with a reason, not an
        // error: the client asked something reasonable and safety declined.
        proto::Call::RobotEnable(p) => {
            let result = if p.on && state.fallen.load(Ordering::Relaxed) {
                proto::IntentResult::refused("the robot is down; stand it up first")
            } else {
                intents.set_enabled(p.on);
                proto::IntentResult::accepted()
            };
            proto::Response::ok(Some(id), &result)
        }

        proto::Call::RobotHealth => proto::Response::ok(Some(id), &state.health()),
        proto::Call::RobotSafeToRestart => proto::Response::ok(Some(id), &state.safe_to_restart()),
        proto::Call::RobotModelApi => proto::Response::ok(
            Some(id),
            &proto::ModelApiResult {
                model_api: MODEL_API,
            },
        ),
        // No media stack, so no session can be live. `mediad` owns the real answer
        // (architecture.md §5.2); reporting `false` here is honest for now, and the updater
        // treats unknown as false anyway.
        proto::Call::RobotRemoteSessionActive => {
            proto::Response::ok(Some(id), &proto::SessionActiveResult { active: false })
        }
        proto::Call::Hello(_) => proto::Response::ok(
            Some(id),
            &proto::HelloResult {
                api_version: proto::API_VERSION,
                daemon_version: proto::semver::Version::parse(env!("CARGO_PKG_VERSION")).ok(),
                revision: proto::build_info!().revision.map(str::to_owned),
            },
        ),
        // `update.*` is `updaterd`'s namespace. A client reaching here aimed at the wrong
        // socket, so say that rather than report a generic failure.
        other => proto::Response::err(
            Some(id),
            proto::Error::new(
                proto::code::METHOD_NOT_FOUND,
                format!("{} is not served by robotd", other.method()),
            ),
        ),
    }
}

async fn write_line<T: serde::Serialize>(
    out: &mut tokio::net::unix::OwnedWriteHalf,
    message: &T,
) -> std::io::Result<()> {
    let mut line = serde_json::to_vec(message)
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
    line.push(b'\n');
    out.write_all(&line).await?;
    out.flush().await
}

/// Resolve on SIGTERM (systemd stop) or SIGINT (Ctrl-C).
async fn shutdown() {
    use tokio::signal::unix::{SignalKind, signal};
    let mut term = match signal(SignalKind::terminate()) {
        Ok(s) => s,
        Err(e) => {
            tracing::warn!(error = %e, "cannot listen for SIGTERM");
            return std::future::pending().await;
        }
    };
    let mut int = match signal(SignalKind::interrupt()) {
        Ok(s) => s,
        Err(_) => return std::future::pending().await,
    };
    tokio::select! {
        _ = term.recv() => {}
        _ = int.recv() => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn state() -> RobotState {
        RobotState::new(&Params::default(), false, false)
    }

    /// Mark the loop as having just ticked, `ticks` times.
    fn ticked(s: &RobotState, ticks: u64) {
        s.ticks.store(ticks, Ordering::Relaxed);
        s.last_tick_us
            .store(s.started.elapsed().as_micros() as u64, Ordering::Relaxed);
    }

    /// Before the loop has run, health must be false. Claiming readiness early would let an
    /// update commit against a robot that never actually started.
    #[test]
    fn not_healthy_until_the_loop_has_ticked() {
        let s = state();
        assert!(!s.health().healthy);
        assert!(s.health().reason.unwrap().contains("not completed a cycle"));

        ticked(&s, 1);
        assert!(s.health().healthy);
    }

    /// **The point of slice 1.** A loop that ticked once and then wedged must report
    /// unhealthy, not stay healthy forever on the strength of that one tick. This is what
    /// the updater's auto-rollback actually gates on.
    #[test]
    fn a_stalled_loop_reports_unhealthy() {
        // A short window so the test does not sleep for the real 500 ms default. Two
        // periods at 50 Hz is 40 ms.
        let mut params = Params::default();
        params.health.stall_periods = 2;
        let s = RobotState::new(&params, false, false);

        s.ticks.store(1, Ordering::Relaxed);
        // Last tick stamped at time zero while `started` keeps advancing — the shape of a
        // loop that stopped.
        s.last_tick_us.store(0, Ordering::Relaxed);
        std::thread::sleep(Duration::from_millis(60));

        let health = s.health();
        assert!(!health.healthy);
        assert!(
            health.reason.as_deref().unwrap().contains("stalled"),
            "{:?}",
            health.reason
        );
    }

    /// **Regression.** The stall check must not fire on ordinary scheduler jitter.
    ///
    /// It was originally three periods — 60 ms at 50 Hz — which a loaded machine exceeds
    /// routinely. That failed the gate test outright, and on a board it would report a
    /// perfectly good release unhealthy and roll it back: exactly the false positive the
    /// health gate exists not to produce. Stall detects a *wedged* loop; `min_achieved_hz`
    /// owns degradation, and conflating them makes both worse.
    #[test]
    fn a_late_tick_is_not_a_stalled_loop() {
        let s = state();
        let params = Params::default();
        s.ticks.store(100, Ordering::Relaxed);

        // 100 ms late: five whole periods, far past anything the old threshold tolerated,
        // and still nowhere near a loop that has died.
        let late_by = Duration::from_millis(100);
        assert!(
            late_by.as_micros() as u64 > params.period().as_micros() as u64 * 3,
            "the jitter under test must exceed the old three-period threshold"
        );
        let now_us = s.started.elapsed().as_micros() as u64;
        s.last_tick_us.store(
            now_us.saturating_sub(late_by.as_micros() as u64),
            Ordering::Relaxed,
        );

        let health = s.health();
        assert!(
            health.healthy,
            "a merely late loop must stay healthy, got {:?}",
            health.reason
        );
    }

    /// A loop running at 60% of target is alive and answers every request. Rate is the only
    /// thing that distinguishes it from a healthy one.
    #[test]
    fn a_slow_loop_reports_unhealthy() {
        let s = state();
        ticked(&s, 100);
        s.achieved_hz.store(30.0f64.to_bits(), Ordering::Relaxed);

        let health = s.health();
        assert!(!health.healthy);
        let reason = health.reason.unwrap();
        assert!(reason.contains("30.0"), "{reason}");
        assert!(reason.contains("45.0"), "{reason}");
    }

    /// The rate is unknown until the first window closes, and unknown must not read as
    /// failing — otherwise every startup would report unhealthy for its first second and the
    /// update gate would roll back a perfectly good release.
    #[test]
    fn an_unmeasured_rate_is_not_treated_as_a_slow_one() {
        let s = state();
        ticked(&s, 5);
        assert_eq!(s.achieved_hz.load(Ordering::Relaxed), 0);
        assert!(s.health().healthy);
    }

    /// One dropped transaction is ordinary; a sustained run of them means the bus is gone,
    /// and a robot that cannot read its own joints is not healthy whatever the loop rate.
    #[test]
    fn sustained_bus_failures_report_unhealthy() {
        let s = state();
        ticked(&s, 100);
        s.consecutive_errors.store(9, Ordering::Relaxed);
        assert!(
            s.health().healthy,
            "9 errors is under the default floor of 10"
        );

        s.consecutive_errors.store(10, Ordering::Relaxed);
        let health = s.health();
        assert!(!health.healthy);
        assert!(health.reason.unwrap().contains("consecutive"));
    }

    /// `--unhealthy` must win over a healthy loop: it exists to exercise rollback.
    #[test]
    fn forced_unhealthy_overrides_a_running_loop() {
        let s = RobotState::new(&Params::default(), true, false);
        ticked(&s, 100);
        assert!(!s.health().healthy);
        assert!(s.health().reason.unwrap().contains("--unhealthy"));
    }

    #[test]
    fn safe_to_restart_unless_forced_busy() {
        assert!(state().safe_to_restart().safe);
        let busy = RobotState::new(&Params::default(), false, true).safe_to_restart();
        assert!(!busy.safe);
        assert!(busy.reason.unwrap().contains("--busy"));
    }

    /// Every method must come back off `dispatch` in the shape the updater parses.
    ///
    /// The health tests call `state.health()` directly, which type-checks but says nothing
    /// about what goes over the socket — `dispatch` could answer with a completely different
    /// JSON shape and they would all still pass. `tests/updater_gate.rs` catches that
    /// against a live process; this runs in microseconds and fails on the exact method.
    #[test]
    fn dispatch_answers_every_method_in_the_typed_shape() {
        let s = state();
        ticked(&s, 1);
        let id = || proto::Id::Number(1);

        let health: proto::HealthResult =
            dispatch(&s, &Intents::new(), id(), &proto::Call::RobotHealth)
                .result_as()
                .expect("robot.health must deserialize as HealthResult");
        assert!(health.healthy);

        let safe: proto::SafeToRestartResult =
            dispatch(&s, &Intents::new(), id(), &proto::Call::RobotSafeToRestart)
                .result_as()
                .expect("robot.safeToRestart must deserialize as SafeToRestartResult");
        assert!(safe.safe);

        let session: proto::SessionActiveResult = dispatch(
            &s,
            &Intents::new(),
            id(),
            &proto::Call::RobotRemoteSessionActive,
        )
        .result_as()
        .expect("robot.remoteSessionActive must deserialize as SessionActiveResult");
        assert!(!session.active);
    }

    /// `update.*` is a valid call that this daemon does not serve. It must be refused with a
    /// message naming the right daemon, not answered with something invented.
    #[test]
    fn calls_belonging_to_updaterd_are_refused() {
        let s = state();
        let response = dispatch(
            &s,
            &Intents::new(),
            proto::Id::Number(1),
            &proto::Call::Status,
        );
        let error = response.error.expect("update.status must be refused");
        assert_eq!(error.code, proto::code::METHOD_NOT_FOUND);
        assert!(error.message.contains("robotd"), "{}", error.message);
    }

    #[test]
    fn model_api_is_reported() {
        let s = state();
        let response = dispatch(
            &s,
            &Intents::new(),
            proto::Id::Number(1),
            &proto::Call::RobotModelApi,
        );
        let result: proto::ModelApiResult = response.result_as().unwrap();
        assert_eq!(result.model_api, MODEL_API);
    }

    #[test]
    fn durations_accept_seconds_and_millis() {
        assert_eq!(parse_duration("2s").unwrap(), Duration::from_secs(2));
        assert_eq!(parse_duration("500ms").unwrap(), Duration::from_millis(500));
        assert_eq!(parse_duration("3").unwrap(), Duration::from_secs(3));
        assert!(parse_duration("soon").is_err());
    }

    /// **The startup invariant.** The loop must command the pose it *found*, not the home
    /// pose and nothing interpolated — an update restarting `robotd` while the robot stands
    /// must not move it.
    ///
    /// `frozen()` so the fake robot does not follow commands: if the loop were re-reading
    /// and re-adopting each tick, a tracking fake would hide the bug.
    #[tokio::test]
    async fn the_loop_holds_the_pose_it_started_in() {
        let mut resting = DEFAULT_POSITION;
        resting[0] = 0.42; // deliberately not the home pose
        let io = FakeIo::at(resting).frozen();

        let s = Arc::new(RobotState::new(&Params::default(), false, false));
        let (tx, rx) = std::sync::mpsc::channel();
        let loop_state = Arc::clone(&s);
        let handle = tokio::spawn(async move {
            let mut io = io;
            control_loop_probe(&mut io, loop_state, Duration::from_millis(2)).await;
            tx.send(io.last_written).unwrap();
        });

        while s.ticks.load(Ordering::Relaxed) < 5 {
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        s.shutdown.store(true, Ordering::Relaxed);
        handle.await.unwrap();

        let written = rx.recv().unwrap().expect("the loop must command something");
        assert_eq!(
            written.positions, resting,
            "the loop moved the robot instead of holding where it found it"
        );
    }

    /// **The policy-failure contract.** A policy that cannot load must not stop the robot
    /// working: the loop keeps ticking at rate, holds its pose, and health says why.
    ///
    /// This is the branch that makes a broken bundle a rollback instead of an outage. It
    /// nearly did not work at all — `ort` does not return an error when ONNX Runtime is
    /// missing, it `expect`s deep inside a lazy init, so the control thread died, no tick
    /// ever landed, and health reported "the loop has not completed a cycle" forever. The
    /// daemon looked wedged rather than saying what was wrong.
    ///
    /// Works whether or not ONNX Runtime is installed: with it, the bogus path fails to
    /// load; without it, the runtime probe fails first. Either way the contract is the same.
    #[tokio::test]
    async fn an_unloadable_policy_holds_the_pose_and_reports_why() {
        let mut params = Params::default();
        params.policy.walk = PathBuf::from("/nonexistent/definitely-not-a-policy.onnx");
        params.policy.stand = None;

        let resting = DEFAULT_POSITION;
        let s = Arc::new(RobotState::new(&params, false, false));
        let intents = Arc::new(Intents::new());
        // Enabled, so this is not passing merely because nothing asked the robot to move.
        intents.set_enabled(true);
        intents.set_twist([0.4, 0.0, 0.0]);

        let loop_state = Arc::clone(&s);
        let handle = tokio::spawn(control_loop(
            FakeIo::at(resting).frozen(),
            loop_state,
            Arc::clone(&intents),
            params,
            Duration::from_millis(2),
        ));

        let deadline = Instant::now() + Duration::from_secs(5);
        while s.ticks.load(Ordering::Relaxed) < 5 && Instant::now() < deadline {
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        let ticks = s.ticks.load(Ordering::Relaxed);
        let health = s.health();
        s.shutdown.store(true, Ordering::Relaxed);
        handle.await.unwrap();

        assert!(
            ticks >= 5,
            "the loop must keep running without a policy, got {ticks} ticks"
        );
        assert!(!health.healthy, "a robot that cannot walk is not healthy");
        let reason = health.reason.unwrap_or_default();
        assert!(
            reason.contains("policy"),
            "health must name the policy as the cause, got {reason:?}"
        );
        assert!(
            !s.moving.load(Ordering::Relaxed),
            "nothing should be reported as moving"
        );
    }

    /// `control_loop` takes its IO by value, which makes the fake unreachable afterwards.
    /// This borrows instead so a test can inspect what was written.
    async fn control_loop_probe<T: RobotIo>(io: &mut T, state: Arc<RobotState>, period: Duration) {
        struct Borrowed<'a, T>(&'a mut T);
        impl<T: RobotIo> RobotIo for Borrowed<'_, T> {
            fn read(&mut self) -> duck_control::io::Result<duck_control::Sensors> {
                self.0.read()
            }
            fn write(&mut self, t: &duck_control::JointTargets) -> duck_control::io::Result<()> {
                self.0.write(t)
            }
            fn set_gain(&mut self, kp: u16) -> duck_control::io::Result<()> {
                self.0.set_gain(kp)
            }
        }
        control_loop(
            Borrowed(io),
            state,
            Arc::new(Intents::new()),
            Params::default(),
            period,
        )
        .await
    }
}
