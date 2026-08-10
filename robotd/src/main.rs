//! `robotd` — the robot control daemon.
//!
//! **Slice 1** (`docs/design/robotd-design.md` §4): a control loop that drives the real bus at the
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
mod soc;

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

/// How far a subscriber may fall behind before it starts losing frames.
///
/// Five seconds at 50 Hz. State is advisory: a client that cannot keep up gets a gap, never
/// backpressure onto the control loop. Same rule the updater applies to progress.
const STATE_BUFFER: usize = 256;

/// Window over which the achieved rate is measured, and therefore how quickly a degraded
/// loop becomes visible to the health gate. Doubles as the slow-sensor sampling interval
/// ([`publish_slow_sensors`]).
const RATE_WINDOW: Duration = Duration::from_secs(1);

/// Smoothing on the reported battery voltage. At one sample per [`RATE_WINDOW`] this is a
/// ~10 s time constant, which is what makes the number readable: the raw voltage sags
/// several tenths of a volt on every step and recovers between them, so an unsmoothed
/// reading swings while the pack is doing nothing unusual. Borrowed, with the figure, from
/// `microduck_runtime`.
const BATTERY_EMA_ALPHA: f64 = 0.1;

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
    /// Failed attempts to bring the bus up — opening it, verifying its registers, or
    /// reading the startup pose. Non-zero means the loop is still waiting for a robot to
    /// answer and has never commanded anything.
    startup_bus_failures: AtomicU32,
    /// Motor-bus voltage, EMA-smoothed, as `f64::to_bits`. Zero means *not read yet* — a
    /// distinction that has to survive to the wire, since zero volts and unknown volts look
    /// nothing alike to whoever is deciding whether to charge the robot.
    battery_v: AtomicU64,
    /// Hottest servo of the last thermal sample: temperature as `f64::to_bits`, and which
    /// joint it was. Zero means *not read yet*, same as the battery.
    motor_max_c: AtomicU64,
    motor_mean_c: AtomicU64,
    /// Index into [`duck_control::JOINT_NAMES`] of the hottest joint.
    motor_hottest: AtomicU32,
    /// Hottest board thermal zone, as `f64::to_bits`. Zero means no reading — off Linux, or a
    /// kernel with no thermal sysfs ([`soc`]).
    cpu_temp_c: AtomicU64,
    /// Mirrors of the bus's own IMU diagnostics, refreshed with the thermal sample. Held here
    /// so the IPC side can report them without touching the loop's IO.
    imu_stale_blocks: AtomicU64,
    /// Stale reads in a row as of the last sample. Sampled rather than watched, which is fine
    /// for the fault it describes: a board that has stopped refreshing keeps repeating, so its
    /// run is still growing whenever the next sample lands.
    imu_stale_run: AtomicU64,
    imu_ready: AtomicBool,
    shutdown: AtomicBool,
    /// Fan-out for `robot.state`. Bounded and lossy by design — see [`STATE_BUFFER`].
    state_tx: tokio::sync::broadcast::Sender<proto::RobotState>,
    /// Why the policy is not loaded, if it is not. Set once at startup; the loop keeps
    /// running and holds the pose, so a broken bundle is a rollback rather than a crash.
    policy_error: ArcSwapOption<String>,
    /// Which policy files this process was configured with, as file names. `None` when the
    /// policy is disabled.
    ///
    /// From the params rather than from the loaded network, and therefore known before the
    /// control thread has finished loading anything — a client that subscribes during startup
    /// gets the answer rather than a race. What *failed* to load is `policy_error`; this is
    /// what was asked for, and the pair is what distinguishes "no policy wanted" from "the
    /// policy this release ships would not load".
    policy_walk: Option<String>,
    policy_stand: Option<String>,
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
            startup_bus_failures: AtomicU32::new(0),
            battery_v: AtomicU64::new(0),
            motor_max_c: AtomicU64::new(0),
            motor_mean_c: AtomicU64::new(0),
            motor_hottest: AtomicU32::new(0),
            cpu_temp_c: AtomicU64::new(0),
            imu_stale_blocks: AtomicU64::new(0),
            imu_stale_run: AtomicU64::new(0),
            imu_ready: AtomicBool::new(false),
            shutdown: AtomicBool::new(false),
            state_tx: tokio::sync::broadcast::Sender::new(STATE_BUFFER),
            policy_error: ArcSwapOption::empty(),
            policy_walk: params
                .policy
                .enabled
                .then(|| file_name(&params.policy.walk))
                .flatten(),
            policy_stand: params
                .policy
                .enabled
                .then(|| params.policy.stand.as_deref().and_then(file_name))
                .flatten(),
            fallen: AtomicBool::new(false),
            moving: AtomicBool::new(false),
            period_us: params.period().as_micros() as u64,
            min_achieved_hz: params.update_gate.min_achieved_hz,
            stall_periods: params.update_gate.stall_periods,
            max_consecutive_errors: params.update_gate.max_consecutive_errors,
            force_unhealthy,
            force_busy,
        }
    }

    fn health(&self) -> proto::HealthResult {
        // Everything the robot can say about itself, attached to every answer whatever the
        // verdict — and consulted by none of the checks below.
        //
        // Two separate jobs in one method, deliberately. `healthy`/`degraded` are the update
        // system's inputs and may only reflect what a *release* can be blamed for. The rest
        // is a description of the robot for whoever is looking at it, and it travels on the
        // same answer because a robot behaving oddly is asked exactly one question, once.
        let describe =
            |healthy: bool, degraded: bool, reason: Option<String>| proto::HealthResult {
                healthy,
                degraded,
                reason,
                battery: self.battery(),
                motors: self.motor_thermal(),
                cpu_temp_c: {
                    // Zero is "never read", the same sentinel the battery uses: a board at
                    // 0 °C is a sensor that is not there, not a cold robot.
                    let c = f64::from_bits(self.cpu_temp_c.load(Ordering::Relaxed));
                    (c > 0.0).then_some(c)
                },
                control_loop: Some(self.loop_health()),
                bus: proto::BusHealth {
                    consecutive_errors: self.consecutive_errors.load(Ordering::Relaxed),
                    startup_failures: self.startup_bus_failures.load(Ordering::Relaxed),
                },
                imu: Some(proto::ImuHealth {
                    ready: self.imu_ready.load(Ordering::Relaxed),
                    stale_blocks: self.imu_stale_blocks.load(Ordering::Relaxed),
                    consecutive_stale_blocks: self.imu_stale_run.load(Ordering::Relaxed),
                }),
            };

        let unhealthy = |reason: String| describe(false, false, Some(reason));
        // Not healthy, but not the release's fault either — see `HealthResult::degraded`.
        let degraded = |reason: String| describe(false, true, Some(reason));

        if self.force_unhealthy {
            return unhealthy("forced unhealthy by --unhealthy".into());
        }

        // "Starting" is not "started". The gate polls, so it will see the transition.
        if self.ticks.load(Ordering::Relaxed) == 0 {
            // Distinguish "starting" from "cannot see a robot". Both mean no ticks, but only
            // one of them is going to resolve on its own, and the update system quotes this
            // string as the reason it rolled a release back — so it has to name the cause.
            let waiting = self.startup_bus_failures.load(Ordering::Relaxed);
            if waiting > 0 {
                // Degraded, not unhealthy: an unpowered bench board must not roll back every
                // release shipped to it. The bus not answering is the same before and after.
                return degraded(format!(
                    "no robot on the motor bus after {waiting} attempts; \
                     is servo power on and the bus wired?"
                ));
            }
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

        describe(true, false, None)
    }

    /// The loop's own numbers, as the readout reports them.
    fn loop_health(&self) -> proto::LoopHealth {
        let hz = f64::from_bits(self.achieved_hz.load(Ordering::Relaxed));
        let now_us = self.started.elapsed().as_micros() as u64;
        proto::LoopHealth {
            target_hz: 1_000_000.0 / self.period_us as f64,
            // Zero is the "no window has closed yet" sentinel, not a measured rate.
            achieved_hz: (hz > 0.0).then_some(hz),
            ticks: self.ticks.load(Ordering::Relaxed),
            missed: self.missed.load(Ordering::Relaxed),
            last_tick_age_ms: now_us.saturating_sub(self.last_tick_us.load(Ordering::Relaxed))
                / 1000,
        }
    }

    /// The hottest servo of the last thermal sample, or `None` before the first one.
    fn motor_thermal(&self) -> Option<proto::MotorThermal> {
        let max_c = f64::from_bits(self.motor_max_c.load(Ordering::Relaxed));
        if max_c <= 0.0 {
            return None;
        }
        let hottest = self.motor_hottest.load(Ordering::Relaxed) as usize;
        Some(proto::MotorThermal {
            hottest: duck_control::JOINT_NAMES
                .get(hottest)
                .unwrap_or(&"unknown")
                .to_string(),
            max_c,
            mean_c: f64::from_bits(self.motor_mean_c.load(Ordering::Relaxed)),
        })
    }

    /// The last battery reading, mapped to a percentage — or `None` if there has not been
    /// one.
    ///
    /// Zero is the "never read" sentinel rather than a measurement: the atomic starts there,
    /// and a robot whose bus cannot answer never leaves it. Reporting that as `0.00 V, 0%`
    /// would put a flat-battery warning in front of anyone whose robot has been up for less
    /// than a second.
    fn battery(&self) -> Option<proto::Battery> {
        let volts = f64::from_bits(self.battery_v.load(Ordering::Relaxed));
        (volts > 0.0).then(|| proto::Battery {
            volts,
            percent: duck_control::battery_percent(volts),
        })
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
/// from cannot be inferred (`docs/design/architecture.md` §8).
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
    // Before the ramp, not after: position_p_gain is a RAM register that survives this
    // process, so whatever was written last is what the robot stands up with. A previous fall
    // leaves `gain_limp` (50) there, and `init` would then take the robot to its home pose at
    // a third of the intended stiffness — soft enough to be a fall risk in the one command
    // whose whole job is establishing a known state.
    //
    // `robotd`'s control loop sets its own gain on the first tick, so this only governs the
    // ramp and the window before the daemon starts — which is exactly the window where the
    // robot is standing up unsupported.
    if let Err(e) = io.set_gain(params.policy.gain) {
        tracing::error!(error = %e, gain = params.policy.gain, "cannot set the position gain");
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

            // Waiting, not one shot. `open_bus` verifies motor registers, which means it
            // talks to the servos — so on an unpowered board it fails and this used to fall
            // straight off the end of the thread. No control loop was ever created, and
            // because nothing had been *attempted* the health reason was the bland "control
            // loop has not completed a cycle yet", forever, whatever happened to the robot
            // afterwards. Retrying the read alone was not enough: execution never got there.
            runtime.block_on(async move {
                if let Some(io) = open_bus_waiting(&port, &state).await {
                    control_loop(io, state, intents, params, period).await;
                }
            });
        })
}

/// The real bus on the board; a fake elsewhere, so `open_bus_waiting` has one signature.
#[cfg(target_os = "linux")]
type BusIo = duck_control::bus::DynamixelIo;
#[cfg(not(target_os = "linux"))]
type BusIo = FakeIo;

/// Open and verify the bus, waiting for a robot to answer.
///
/// Same reasoning as [`adopt_startup_pose`], one step earlier: an unpowered board cannot
/// pass `check_registers`, and that is a condition someone fixes by flipping a switch, not
/// one to abandon the control loop over.
///
/// Returns `None` only if shutdown is requested while waiting.
async fn open_bus_waiting(port: &str, state: &RobotState) -> Option<BusIo> {
    let mut attempt = 0u32;

    while !state.shutdown.load(Ordering::Relaxed) {
        // Logging lives in `open_bus`, which is chatty by design on the first attempt and
        // quiet thereafter — a board waiting overnight must not fill the journal.
        if let Some(io) = open_bus(port, attempt) {
            state.startup_bus_failures.store(0, Ordering::Relaxed);
            return Some(io);
        }
        attempt += 1;
        // Published before sleeping, so `robot.health` can name the cause immediately.
        state.startup_bus_failures.store(attempt, Ordering::Relaxed);

        // Nothing to retry on a platform that has no bus at all.
        if !cfg!(target_os = "linux") {
            return None;
        }
        tokio::time::sleep(STARTUP_RETRY_INTERVAL).await;
    }

    None
}

/// Open and verify the bus, or explain why not.
#[cfg(target_os = "linux")]
fn open_bus(port: &str, attempt: u32) -> Option<BusIo> {
    // First attempt and every thirtieth — about one line per 30 s while waiting.
    let loud = attempt == 0 || attempt.is_multiple_of(STARTUP_READ_LOG_EVERY);

    let mut io = match duck_control::bus::DynamixelIo::open(port) {
        Ok(io) => io,
        Err(e) => {
            if loud {
                tracing::error!(error = %e, port, attempt, "cannot open the bus; waiting");
            }
            return None;
        }
    };
    match io.check_registers() {
        Ok(0) => tracing::info!("motor registers already correct"),
        Ok(n) => tracing::warn!(corrected = n, "motor registers corrected"),
        Err(e) => {
            if loud {
                tracing::error!(
                    error = %e,
                    attempt,
                    "motor register check failed; waiting, is servo power on?"
                );
            }
            return None;
        }
    }
    Some(io)
}

#[cfg(not(target_os = "linux"))]
fn open_bus(_port: &str, _attempt: u32) -> Option<BusIo> {
    tracing::error!("no bus on this platform; use --fake");
    None
}

/// How long to wait between attempts to read the startup pose.
///
/// A second, not a control period: the read itself already carries a 30 ms bus timeout, and
/// a board waiting for someone to switch servo power on is not in a hurry. Fast enough that
/// powering the robot brings it up while your hand is still on the switch.
const STARTUP_RETRY_INTERVAL: Duration = Duration::from_secs(1);

/// Log one waiting line, then one every this many attempts — about one per 30 s.
const STARTUP_READ_LOG_EVERY: u32 = 30;

/// Adopt the pose the robot is already in, waiting for the bus to answer.
///
/// Never move on start: the servos hold their last commanded goal while this process is
/// dead, so a restart mid-update leaves a standing robot standing, with no gap. That
/// requires a successful read, so there is nothing to command until one lands.
///
/// This used to be a single read that logged and returned on failure — which killed the
/// control thread for the life of the process. `robotd` stayed up and kept answering the
/// socket, so a board booted before its servos were powered was permanently inert: powering
/// them changed nothing and only `systemctl restart robotd` helped, with no hint anywhere
/// that a restart was what was needed. Retrying makes the ordinary order of operations —
/// power the board, then power the servos — just work.
///
/// Read through `Safety` rather than the bus directly: safety owns the only `RobotIo`, so
/// this is the only way to reach it, and going through it keeps that invariant intact even
/// for the one read that happens before the loop starts.
///
/// Returns `None` only if shutdown is requested while waiting.
async fn adopt_startup_pose<T: RobotIo>(
    safety: &mut Safety<T>,
    state: &RobotState,
    period: Duration,
) -> Option<[f64; NUM_JOINTS]> {
    let mut attempt = 0u32;

    while !state.shutdown.load(Ordering::Relaxed) {
        match safety.read() {
            Ok(sensors) => {
                state.startup_bus_failures.store(0, Ordering::Relaxed);
                if attempt > 0 {
                    // Only worth a line if it had to wait — otherwise "control loop running"
                    // below already says everything, and this would just double it.
                    tracing::warn!(
                        attempt,
                        hz = 1.0 / period.as_secs_f64(),
                        "the motor bus answered; holding the pose found at startup"
                    );
                }
                return Some(sensors.positions);
            }
            Err(e) => {
                attempt += 1;
                // Published before sleeping, so `robot.health` can name the cause on the
                // very first failure rather than after the first log line.
                state.startup_bus_failures.store(attempt, Ordering::Relaxed);
                if attempt == 1 || attempt.is_multiple_of(STARTUP_READ_LOG_EVERY) {
                    tracing::warn!(
                        error = %e,
                        attempt,
                        "no answer from the motor bus; waiting, not commanding anything"
                    );
                }
                tokio::time::sleep(STARTUP_RETRY_INTERVAL).await;
            }
        }
    }

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

    let Some(mut hold) = adopt_startup_pose(&mut safety, &state, period).await else {
        return;
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
    // `Skip`, not `Burst` and not `Delay`.
    //
    // `Burst` replays a backlog back to back, stacking motor commands — clearly wrong. But
    // `Delay` is wrong too, and less obviously: it schedules the next tick at *now + period*
    // after each one, so every tick's wakeup latency is added to the period rather than
    // absorbed. A few milliseconds of scheduler jitter becomes a permanent rate loss.
    //
    // Measured, not reasoned about: with `Delay` this loop reported 43.1 Hz against a 50 Hz
    // target and `missed = 0` — not overrunning its work, just being rescheduled late every
    // time. With a real bus read costing 3–8 ms it would have been nearer 35 Hz, and it
    // would have looked like a hardware problem.
    //
    // `Skip` keeps the original schedule and drops missed ticks, which is what a control
    // loop wants: no backlog, no drift.
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

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
        let (command, deadman) = safety.gate(snapshot.command, snapshot.twist_age);
        let mut limits: Vec<duck_control::safety::Limit> = deadman.into_iter().collect();

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

        let (targets, gain, moving, policy_label) = match (driving, sensors.as_ref()) {
            (true, Some(sensors)) => {
                let controller = controller.as_mut().expect("driving implies a controller");
                match controller.step(sensors, &command) {
                    Ok(step) => (
                        step.targets,
                        step.gain,
                        command.twist_magnitude() > 0.0,
                        if step.standing { "stand" } else { "walk" },
                    ),
                    Err(e) => {
                        tracing::warn!(error = %e, "inference failed; holding");
                        (hold, params.policy.gain, false, "held")
                    }
                }
            }
            _ => (hold, params.policy.gain, false, "held"),
        };
        state.moving.store(moving, Ordering::Relaxed);

        match safety.apply(targets, hold, gain) {
            Ok(applied) => limits.extend(applied.limits),
            Err(e) => tracing::warn!(error = %e, "bus write failed"),
        }

        // Only assemble a frame when somebody is subscribed. On a robot nobody usually is,
        // and this would otherwise be a per-tick allocation on the thread that should not
        // be visiting the allocator without a reason.
        if state.state_tx.receiver_count() > 0
            && let Some(sensors) = sensors.as_ref()
        {
            let _ = state.state_tx.send(proto::RobotState {
                t: state.started.elapsed().as_secs_f64(),
                movement: proto::MoveState {
                    requested: snapshot.command.twist,
                    applied: command.twist,
                    limited_by: limits.iter().map(|l| limit_name(*l).to_owned()).collect(),
                },
                head: command.head,
                policy: policy_label.to_owned(),
                safety: proto::SafetyState {
                    fallen: safety.fallen(),
                    limp: safety.fallen(),
                    gravity: sensors.imu.gravity,
                    gain: safety.gain(),
                },
                control_loop: proto::LoopState {
                    hz: f64::from_bits(state.achieved_hz.load(Ordering::Relaxed)),
                    missed: state.missed.load(Ordering::Relaxed),
                },
                joints: sensors.positions.to_vec(),
                targets: targets.to_vec(),
            });
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

            publish_slow_sensors(&mut safety, &state);

            if last_summary.elapsed() >= LOOP_SUMMARY_INTERVAL {
                tracing::info!(
                    total = ticks,
                    hz = format!("{hz:.1}"),
                    missed = state.missed.load(Ordering::Relaxed),
                    driving,
                    fallen = safety.fallen(),
                    battery_v = format!(
                        "{:.2}",
                        f64::from_bits(state.battery_v.load(Ordering::Relaxed))
                    ),
                    motor_max_c = format!(
                        "{:.0}",
                        f64::from_bits(state.motor_max_c.load(Ordering::Relaxed))
                    ),
                    cpu_c = format!(
                        "{:.0}",
                        f64::from_bits(state.cpu_temp_c.load(Ordering::Relaxed))
                    ),
                    "control loop"
                );
                last_summary = Instant::now();
            }
        }
    }
    tracing::info!("control loop stopped");
}

/// Stable wire names for the reasons a command was altered.
///
/// Spelled out rather than `Debug`-formatted: this goes over the wire, and a client
/// branching on it must not break because a variant was renamed in Rust.
/// A path's file name, for reporting which policy is loaded. `None` for a path that ends in
/// something that is not a file name — reported as unknown rather than as an empty string,
/// which would read as "no policy".
fn file_name(path: &std::path::Path) -> Option<String> {
    Some(path.file_name()?.to_string_lossy().into_owned())
}

fn limit_name(limit: duck_control::safety::Limit) -> &'static str {
    use duck_control::safety::Limit;
    match limit {
        Limit::Deadman => "deadman",
        Limit::Range => "joint_range",
        Limit::NotFinite => "not_finite",
        Limit::Fallen => "fallen",
    }
}

/// Sample and publish everything that does not need sampling every tick, once per
/// [`RATE_WINDOW`].
///
/// Not part of the tick. The voltage/temperature registers are a second bus transaction —
/// about a millisecond — which is nothing once a second and would be 5% of the budget at
/// 50 Hz. A second is also faster than a pack can drain or a servo can heat up.
///
/// Called from the loop thread because that thread owns the IO, and nothing else may touch
/// the bus: a transaction issued from the IPC side would interleave bytes with a tick and
/// corrupt both. The IMU counters come from the same `io`, so they are mirrored here rather
/// than reached for from the socket.
fn publish_slow_sensors<T: RobotIo>(io: &mut Safety<T>, state: &RobotState) {
    let stale = io.imu_stale();
    state.imu_stale_blocks.store(stale.total, Ordering::Relaxed);
    state.imu_stale_run.store(stale.run, Ordering::Relaxed);
    state.imu_ready.store(io.imu_ready(), Ordering::Relaxed);

    // Before the bus read, and unconditionally: this is a `sysfs` read that owes the motor bus
    // nothing, and a board cooking behind a blocked vent is *more* likely to be worth seeing on
    // a robot whose servos have stopped answering, not less.
    if let Some(celsius) = soc::hottest_zone_c() {
        state.cpu_temp_c.store(celsius.to_bits(), Ordering::Relaxed);
    }

    match io.slow_sensors() {
        Ok(slow) => {
            let previous = f64::from_bits(state.battery_v.load(Ordering::Relaxed));
            // Seed from the first reading rather than blending up from zero, which would
            // spend ten seconds reporting a battery flatter than it is.
            let smoothed = if previous > 0.0 {
                BATTERY_EMA_ALPHA * slow.volts + (1.0 - BATTERY_EMA_ALPHA) * previous
            } else {
                slow.volts
            };
            state.battery_v.store(smoothed.to_bits(), Ordering::Relaxed);

            // Temperature is not smoothed: a servo's case is already a slow signal, and an
            // EMA would only delay the one reading anybody cares about — the joint climbing
            // towards its overheat shutdown.
            let (hottest, max_c) = slow.temps_c.iter().enumerate().fold(
                (0usize, f64::MIN),
                |(best, high), (joint, &t)| {
                    if t > high { (joint, t) } else { (best, high) }
                },
            );
            let mean_c = slow.temps_c.iter().sum::<f64>() / slow.temps_c.len() as f64;
            state.motor_max_c.store(max_c.to_bits(), Ordering::Relaxed);
            state
                .motor_mean_c
                .store(mean_c.to_bits(), Ordering::Relaxed);
            state.motor_hottest.store(hottest as u32, Ordering::Relaxed);
        }
        // Keep the last sample. A single failed transaction is ordinary on a serial bus, and
        // dropping to "unknown" over one would make the reported battery flicker. A bus that
        // is really gone already shows up in the verdict and in `bus.consecutive_errors`.
        Err(e) => tracing::debug!(error = %e, "slow-sensor read failed; keeping the last sample"),
    }
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

    // `None` until the client subscribes. Once set, the connection is both a request
    // channel and a state stream, so the loop below waits on whichever speaks first.
    let mut states: Option<tokio::sync::broadcast::Receiver<proto::RobotState>> = None;
    let mut decimate = Duration::ZERO;
    let mut last_sent: Option<Instant> = None;

    loop {
        let line = match states.as_mut() {
            None => lines.next_line().await?,
            Some(rx) => {
                tokio::select! {
                    line = lines.next_line() => line?,
                    received = rx.recv() => {
                        match received {
                            Ok(state) => {
                                // Decimate per subscriber: a dashboard asking for 10 Hz
                                // should not cost what a digital twin asking for 50 does.
                                let due = last_sent
                                    .map(|at| at.elapsed() >= decimate)
                                    .unwrap_or(true);
                                if due {
                                    last_sent = Some(Instant::now());
                                    write_line(&mut write_half, &proto::Request::notify_state(&state))
                                        .await?;
                                }
                            }
                            // Lagged: the client fell behind and lost frames. That is the
                            // designed behaviour — state is advisory and must never apply
                            // backpressure to the control loop — so carry on from the newest.
                            Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                                tracing::debug!(dropped = n, "state subscriber fell behind");
                            }
                            Err(tokio::sync::broadcast::error::RecvError::Closed) => return Ok(()),
                        }
                        continue;
                    }
                }
            }
        };
        let Some(line) = line else { return Ok(()) };

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

        if let Ok(proto::Call::RobotSubscribe(params)) = &call {
            decimate = params
                .hz
                .filter(|hz| *hz > 0)
                .map(|hz| Duration::from_secs_f64(1.0 / hz as f64))
                .unwrap_or(Duration::ZERO);
            // Subscribing again replaces the rate rather than opening a second stream.
            states = Some(state.state_tx.subscribe());
            last_sent = None;
        }

        let response = match call {
            Ok(call) => dispatch(&state, &intents, id, &call),
            Err(e) => proto::Response::err(Some(id), e),
        };
        write_line(&mut write_half, &response).await?;
    }
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

        // Handled by the caller, which owns the connection; answering here keeps the
        // request/response pairing in one place.
        // The acknowledgement carries the policy identity: it is constant for the life of the
        // process, so sending it once here costs nothing, where putting it on every frame
        // would allocate two strings per tick on the control thread.
        proto::Call::RobotSubscribe(_) => proto::Response::ok(
            Some(id),
            &proto::SubscribeResult {
                accepted: true,
                walk: state.policy_walk.clone(),
                stand: state.policy_stand.clone(),
                unavailable: state.policy_error.load_full().map_or_else(
                    || {
                        state
                            .policy_walk
                            .is_none()
                            .then(|| "no policy configured; holding the startup pose".to_owned())
                    },
                    |e| Some(format!("policy would not load: {e}")),
                ),
            },
        ),

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
        params.update_gate.stall_periods = 2;
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

    /// Subscribing answers with the policy this process is running.
    ///
    /// Sent once, in the acknowledgement, rather than on every frame: it cannot change while
    /// the process lives, and two strings per tick on the control thread is a cost paid fifty
    /// times a second for an answer that never differs.
    #[test]
    fn subscribing_names_the_policy() {
        let mut params = Params::default();
        params.policy.enabled = true;
        params.policy.walk = "/opt/robot/releases/7/alpha_walking.onnx".into();
        params.policy.stand = Some("/opt/robot/releases/7/alpha_stand.onnx".into());
        let s = Arc::new(RobotState::new(&params, false, false));

        let result: proto::SubscribeResult = dispatch(
            &s,
            &Intents::new(),
            proto::Id::Number(1),
            &proto::Call::RobotSubscribe(proto::SubscribeParams { hz: Some(10) }),
        )
        .result_as()
        .expect("robot.subscribe must deserialize as SubscribeResult");

        assert!(result.accepted);
        // File names, not paths: the directory is what `robotctl version` reports, and the
        // name is the part that differs between two builds someone is comparing.
        assert_eq!(result.walk.as_deref(), Some("alpha_walking.onnx"));
        assert_eq!(result.stand.as_deref(), Some("alpha_stand.onnx"));
        assert_eq!(result.unavailable, None);
    }

    /// A policy that was wanted and would not load is a different situation from one that was
    /// never wanted, and both end up as `policy: "held"` on the stream. The acknowledgement is
    /// where they are told apart.
    #[test]
    fn subscribing_distinguishes_no_policy_from_a_broken_one() {
        let mut params = Params::default();
        params.policy.enabled = false;
        let disabled = Arc::new(RobotState::new(&params, false, false));
        let result: proto::SubscribeResult = dispatch(
            &disabled,
            &Intents::new(),
            proto::Id::Number(1),
            &proto::Call::RobotSubscribe(proto::SubscribeParams::default()),
        )
        .result_as()
        .unwrap();
        assert_eq!(result.walk, None);
        assert!(
            result
                .unavailable
                .as_deref()
                .is_some_and(|u| u.contains("no policy configured")),
            "{:?}",
            result.unavailable
        );

        params.policy.enabled = true;
        let broken = Arc::new(RobotState::new(&params, false, false));
        broken
            .policy_error
            .store(Some(Arc::new("ONNX Runtime not loadable".to_owned())));
        let result: proto::SubscribeResult = dispatch(
            &broken,
            &Intents::new(),
            proto::Id::Number(1),
            &proto::Call::RobotSubscribe(proto::SubscribeParams::default()),
        )
        .result_as()
        .unwrap();
        // The name it tried is still reported: "which policy failed" is the question.
        assert!(result.walk.is_some(), "{result:?}");
        assert!(
            result
                .unavailable
                .as_deref()
                .is_some_and(|u| u.contains("ONNX Runtime not loadable")),
            "{:?}",
            result.unavailable
        );
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
        // The detail, not just the category. The updater quotes this string as the reason it
        // rolled a release back, so "policy unavailable" on its own is not actionable — that
        // is the same failure as the useless "loop has not completed a cycle" this branch
        // exists to avoid. Which detail arrives depends on the machine: the bogus path where
        // ONNX Runtime is installed, the runtime's own diagnosis where it is not.
        assert!(
            reason.contains("definitely-not-a-policy.onnx") || reason.contains("ONNX Runtime"),
            "health must carry the underlying cause, got {reason:?}"
        );
        assert!(
            !s.moving.load(Ordering::Relaxed),
            "nothing should be reported as moving"
        );
    }

    /// **The reporting claim.** Safety says it reports what it refused rather than silently
    /// altering commands — that is only true if the reason reaches the state stream.
    ///
    /// The deadman is the easiest limit to provoke: intents start maximally stale, so a
    /// loop with the policy enabled and nothing driving it must publish a frame whose twist
    /// was zeroed and whose `limited_by` says why. Without this, a client watching the robot
    /// ignore its command has no way to tell a limit from a bug.
    #[tokio::test]
    async fn the_state_stream_reports_why_a_command_was_refused() {
        let params = Params {
            policy: params::PolicyParams {
                enabled: false,
                ..params::PolicyParams::default()
            },
            ..Params::default()
        };
        let s = Arc::new(RobotState::new(&params, false, false));
        let mut states = s.state_tx.subscribe();

        let intents = Arc::new(Intents::new());
        intents.set_enabled(true);
        // Asked for, but never refreshed — so already past the deadman.
        intents.set_twist([0.4, 0.0, 0.0]);
        tokio::time::sleep(Duration::from_millis(params.safety.deadman_ms + 20)).await;

        let loop_state = Arc::clone(&s);
        let handle = tokio::spawn(control_loop(
            FakeIo::at(DEFAULT_POSITION),
            loop_state,
            Arc::clone(&intents),
            params,
            Duration::from_millis(2),
        ));

        let frame = tokio::time::timeout(Duration::from_secs(5), states.recv())
            .await
            .expect("a frame within five seconds")
            .expect("the stream stayed open");

        s.shutdown.store(true, Ordering::Relaxed);
        handle.await.unwrap();

        assert_eq!(
            frame.movement.requested,
            [0.4, 0.0, 0.0],
            "what the client asked for must survive to the stream"
        );
        assert_eq!(
            frame.movement.applied, [0.0; 3],
            "a stale twist must be zeroed"
        );
        assert!(
            frame.movement.limited_by.contains(&"deadman".to_owned()),
            "the reason must be named, got {:?}",
            frame.movement.limited_by
        );
        assert_eq!(frame.policy, "held", "no policy was loaded");
        assert_eq!(frame.joints.len(), NUM_JOINTS);
    }

    /// Assembling a frame allocates, on the thread that should not be visiting the
    /// allocator without reason. With nobody subscribed — the normal case on a robot —
    /// nothing should be built at all.
    #[tokio::test]
    async fn no_subscribers_means_no_frames() {
        let params = Params {
            policy: params::PolicyParams {
                enabled: false,
                ..params::PolicyParams::default()
            },
            ..Params::default()
        };
        let s = Arc::new(RobotState::new(&params, false, false));
        assert_eq!(s.state_tx.receiver_count(), 0);

        let loop_state = Arc::clone(&s);
        let handle = tokio::spawn(control_loop(
            FakeIo::at(DEFAULT_POSITION),
            loop_state,
            Arc::new(Intents::new()),
            params,
            Duration::from_millis(2),
        ));
        while s.ticks.load(Ordering::Relaxed) < 5 {
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        s.shutdown.store(true, Ordering::Relaxed);
        handle.await.unwrap();

        // Subscribing afterwards must find an empty channel: nothing was published while
        // no one was listening.
        let mut late = s.state_tx.subscribe();
        assert!(
            late.try_recv().is_err(),
            "frames were built with nobody subscribed"
        );
    }

    /// **The regression.** A board powered on before its servos gets no answer from the bus.
    /// That used to kill the control thread outright — `robotd` stayed up, kept serving the
    /// socket, and never ticked again no matter what happened to the robot afterwards. Only
    /// `systemctl restart robotd` recovered it, and nothing said so.
    ///
    /// So: fail the first few reads, then answer, and require the loop to be running.
    #[tokio::test(start_paused = true)]
    async fn the_loop_waits_for_the_bus_rather_than_giving_up() {
        let mut resting = DEFAULT_POSITION;
        resting[0] = 0.42;
        // Three failures is arbitrary; one is enough to have broken the old code.
        let io = FakeIo::at(resting).failing_reads(3).frozen();

        let s = Arc::new(RobotState::new(&Params::default(), false, false));
        let (tx, rx) = std::sync::mpsc::channel();
        let loop_state = Arc::clone(&s);
        let handle = tokio::spawn(async move {
            let mut io = io;
            control_loop_probe(&mut io, loop_state, Duration::from_millis(2)).await;
            tx.send(io.last_written).unwrap();
        });

        // Bounded, so a regression fails the test instead of hanging CI forever.
        for _ in 0..10_000 {
            if s.ticks.load(Ordering::Relaxed) >= 3 {
                break;
            }
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        assert!(
            s.ticks.load(Ordering::Relaxed) >= 3,
            "the loop never started ticking; it gave up on the bus instead of waiting"
        );

        s.shutdown.store(true, Ordering::Relaxed);
        handle.await.unwrap();

        // And it still adopted the pose it found rather than the home pose — waiting must
        // not cost the startup invariant.
        let written = rx.recv().unwrap().expect("the loop must command something");
        assert_eq!(written.positions, resting);
    }

    /// **The invariant the battery field lives or dies by.** A flat pack must be reported and
    /// must not touch the verdict.
    ///
    /// If it ever did, updating a robot on a low battery would roll the release back — and the
    /// replacement would be judged on the same low battery, so the robot could not be updated
    /// at all until someone noticed and charged it. The whole reason `degraded` exists is to
    /// keep board conditions out of the rollback decision; a battery that gated would walk
    /// straight back into it.
    #[test]
    fn a_flat_battery_is_reported_and_changes_no_verdict() {
        let s = state();
        ticked(&s, 100);
        // Below BATTERY_EMPTY_V: the pack is done and the robot is struggling.
        s.battery_v.store(6.1f64.to_bits(), Ordering::Relaxed);

        let health = s.health();
        let battery = health.battery.expect("a flat battery is still a reading");
        assert!(battery.volts < duck_control::BATTERY_EMPTY_V);
        assert_eq!(battery.percent, 0.0);

        assert!(health.healthy, "{:?}", health.reason);
        assert!(!health.degraded);
    }

    /// Zero volts is what the atomic holds before the first read lands, and it must reach the
    /// wire as absent rather than as a pack at 0 V — otherwise `robotctl health` announces a
    /// flat battery on every robot that has been up for less than a second.
    #[test]
    fn an_unread_battery_is_absent_not_empty() {
        let s = state();
        ticked(&s, 1);
        assert_eq!(s.battery_v.load(Ordering::Relaxed), 0);
        assert!(s.health().battery.is_none());
    }

    /// The reading travels on every answer, not only the healthy one — a robot that is
    /// unhealthy *because* it is out of power is exactly when someone wants to see the pack.
    #[test]
    fn battery_is_reported_alongside_an_unhealthy_verdict() {
        let s = state();
        s.startup_bus_failures.store(4, Ordering::Relaxed);
        s.battery_v.store(7.5f64.to_bits(), Ordering::Relaxed);
        s.motor_max_c.store(48.0f64.to_bits(), Ordering::Relaxed);

        let health = s.health();
        assert!(!health.healthy);
        assert!(
            health.battery.is_some(),
            "battery dropped from a bad answer"
        );
        // The rest of the description too: an unhealthy robot is exactly when someone needs
        // the whole picture, not a verdict on its own.
        assert!(health.motors.is_some(), "thermals dropped");
        assert!(health.control_loop.is_some(), "loop section dropped");
        assert!(health.imu.is_some(), "imu section dropped");
        // And the number *this* verdict was based on: "no robot on the motor bus" is only
        // actionable next to the count of attempts behind it.
        assert_eq!(health.bus.startup_failures, 4);
    }

    /// The loop section must carry the numbers the verdict was decided from, so a reader can
    /// check it rather than take it on faith.
    ///
    /// That distinction has already paid once: 43.9 Hz with `missed = 0` is a loop being
    /// *woken* late, not a loop doing too much, and the two have entirely different fixes.
    #[test]
    fn the_loop_section_reports_what_the_verdict_used() {
        let s = state();
        ticked(&s, 2490);
        s.achieved_hz.store(49.8f64.to_bits(), Ordering::Relaxed);
        s.missed.store(3, Ordering::Relaxed);

        let l = s.health().control_loop.expect("loop section");
        assert_eq!(l.achieved_hz, Some(49.8));
        assert_eq!(l.target_hz, 50.0);
        assert_eq!(l.ticks, 2490);
        assert_eq!(l.missed, 3);

        // Unmeasured stays unmeasured rather than becoming 0 Hz — that would describe a
        // stopped loop, which is the opposite of "started less than a second ago".
        s.achieved_hz.store(0, Ordering::Relaxed);
        assert_eq!(s.health().control_loop.unwrap().achieved_hz, None);
    }

    /// The hottest joint is named, not merely measured. "48 °C" prompts "which one?", and the
    /// answer decides whether it is the knee holding the robot up or something wrong.
    #[test]
    fn thermals_name_the_hottest_joint() {
        let s = state();
        ticked(&s, 100);
        let knee = duck_control::JOINT_NAMES
            .iter()
            .position(|n| *n == "left_knee")
            .unwrap();
        s.motor_max_c.store(48.0f64.to_bits(), Ordering::Relaxed);
        s.motor_mean_c.store(36.0f64.to_bits(), Ordering::Relaxed);
        s.motor_hottest.store(knee as u32, Ordering::Relaxed);

        let motors = s.health().motors.expect("thermals");
        assert_eq!(motors.hottest, "left_knee");
        assert_eq!(motors.max_c, 48.0);
        assert_eq!(motors.mean_c, 36.0);

        // A servo cooking must not change the verdict, for the same reason a flat pack must
        // not: it is a fact about the robot, not evidence about the release.
        assert!(s.health().healthy);
    }

    /// Unread thermals are absent, not 0 °C — which would read as a robot in a freezer.
    #[test]
    fn unread_thermals_are_absent() {
        let s = state();
        ticked(&s, 1);
        assert!(s.health().motors.is_none());
        assert!(s.health().cpu_temp_c.is_none());
    }

    /// Board and servo temperatures are separate readings, and the case that justifies both is
    /// them disagreeing: a board cooking behind a blocked vent while the motors sit idle and
    /// cool. One number could not express it.
    #[test]
    fn a_hot_board_and_cool_motors_are_both_reported() {
        let s = state();
        ticked(&s, 100);
        s.cpu_temp_c.store(84.0f64.to_bits(), Ordering::Relaxed);
        s.motor_max_c.store(31.0f64.to_bits(), Ordering::Relaxed);
        s.motor_mean_c.store(30.0f64.to_bits(), Ordering::Relaxed);

        let health = s.health();
        assert_eq!(health.cpu_temp_c, Some(84.0));
        assert_eq!(health.motors.expect("thermals").max_c, 31.0);
        // And neither touches the verdict — a warm afternoon is not a bad release.
        assert!(health.healthy);
    }

    /// While waiting, health must say *why*. The update system quotes this string as the
    /// reason it rolled a release back, and "control loop has not completed a cycle yet"
    /// describes a robot that is about to start, not one that cannot see its servos.
    #[test]
    fn health_names_the_bus_while_waiting_for_it() {
        let s = RobotState::new(&Params::default(), false, false);
        s.startup_bus_failures.store(4, Ordering::Relaxed);

        let health = s.health();
        assert!(!health.healthy);
        let reason = health.reason.unwrap();
        assert!(
            reason.contains("motor bus") && reason.contains("servo power"),
            "unactionable reason: {reason}"
        );
    }

    /// **The regression.** A bus that cannot be *opened* — or whose register check fails,
    /// which is what an unpowered board does — used to fall off the end of the control
    /// thread. No loop was created and nothing had been recorded, so health fell back to
    /// "control loop has not completed a cycle yet" for the life of the process: the one
    /// message that says nothing about the cause. Retrying the first *read* did not help,
    /// because execution never reached it.
    #[tokio::test(start_paused = true)]
    async fn a_bus_that_cannot_be_opened_is_reported_rather_than_abandoned() {
        let s = Arc::new(RobotState::new(&Params::default(), false, false));
        let waiter_state = Arc::clone(&s);
        let handle = tokio::spawn(async move {
            open_bus_waiting("/dev/definitely-not-a-bus", &waiter_state)
                .await
                .is_none()
        });

        // Bounded, so a regression fails rather than hanging CI.
        for _ in 0..10_000 {
            if s.startup_bus_failures.load(Ordering::Relaxed) > 0 {
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        assert!(
            s.startup_bus_failures.load(Ordering::Relaxed) > 0,
            "an unopenable bus must be recorded, or health cannot explain the silence"
        );
        // Which is exactly what health needs to name it and to pass the update gate.
        assert!(s.health().degraded);

        s.shutdown.store(true, Ordering::Relaxed);
        assert!(handle.await.unwrap(), "must give up only on shutdown");
    }

    /// A silent bus must be *degraded*, not unhealthy: it reports the same before and after
    /// a swap, so rolling a release back cannot fix it and only wastes an update. An
    /// unpowered bench board is the case that has to keep updating.
    #[test]
    fn a_silent_bus_is_degraded_rather_than_unhealthy() {
        let s = RobotState::new(&Params::default(), false, false);
        s.startup_bus_failures.store(4, Ordering::Relaxed);

        let health = s.health();
        assert!(!health.healthy);
        assert!(
            health.degraded,
            "an unpowered board would roll back releases"
        );
    }

    /// The other unhealthy states *are* evidence about the release, so they must not be
    /// degraded — otherwise auto-rollback stops working for the cases it exists for.
    #[test]
    fn a_broken_control_loop_is_not_degraded() {
        let s = RobotState::new(&Params::default(), false, false);
        s.ticks.store(1, Ordering::Relaxed);
        s.consecutive_errors
            .store(s.max_consecutive_errors, Ordering::Relaxed);

        let health = s.health();
        assert!(!health.healthy);
        assert!(!health.degraded, "this must still roll back");
    }

    /// Before any read is attempted there is nothing to blame, so the plain starting-up
    /// message is still the honest one.
    #[test]
    fn health_says_merely_starting_before_the_first_read_fails() {
        let s = RobotState::new(&Params::default(), false, false);
        let reason = s.health().reason.unwrap();
        assert!(reason.contains("not completed a cycle"), "{reason}");
    }

    /// A robot whose bus never answers must still shut down promptly. Waiting forever is
    /// correct; ignoring `systemctl stop` while doing it is not.
    #[tokio::test(start_paused = true)]
    async fn waiting_for_the_bus_still_honours_shutdown() {
        let io = FakeIo::at(DEFAULT_POSITION).failing_reads(u32::MAX);

        let s = Arc::new(RobotState::new(&Params::default(), false, false));
        let loop_state = Arc::clone(&s);
        let handle = tokio::spawn(async move {
            let mut io = io;
            control_loop_probe(&mut io, loop_state, Duration::from_millis(2)).await;
        });

        // Let it fail at least once, so shutdown lands mid-wait rather than before the start.
        for _ in 0..10_000 {
            if s.startup_bus_failures.load(Ordering::Relaxed) > 0 {
                break;
            }
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        assert!(s.startup_bus_failures.load(Ordering::Relaxed) > 0);

        s.shutdown.store(true, Ordering::Relaxed);
        handle
            .await
            .expect("the loop must exit when asked, even with no bus");
        assert_eq!(
            s.ticks.load(Ordering::Relaxed),
            0,
            "nothing should have been commanded without a successful read"
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
            fn slow_sensors(&mut self) -> duck_control::io::Result<duck_control::SlowSensors> {
                self.0.slow_sensors()
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
