//! `robotd` — the robot control daemon.
//!
//! The 50 Hz control loop, the `robot.*` socket, and the health the updater gates on
//! (`docs/design/robotd-design.md` §1.4, §3.4).
//!
//! **Health is why the daemon has the shape it does.** The updater's auto-rollback is only
//! meaningful if `robot.health` means something, and it used to mean "the loop ticked once" —
//! so every rollback was tested against a placeholder. It now means **the loop is meeting its
//! deadline**: a loop running at 60% of target is alive, answers every request, and is badly
//! broken.
//!
//! Holding the pose with no policy (`policy.enabled = false`) stays the right thing to be doing
//! while someone deliberately breaks releases at a bench: the bus sees the real load at the
//! real rate, and nothing falls over when a bad build lands.
//!
//! Every method must be answerable *while the robot is in a bad state*,
//! since that is exactly when it is asked. So the IPC side reads atomics the control loop
//! publishes and never calls into the loop — a wedged loop reports itself unhealthy rather
//! than hanging the caller.

mod chorale;
mod control;
mod intents;
mod params;
mod soc;
mod sound;
mod theremin;

use std::path::PathBuf;
use std::process::ExitCode;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, AtomicU64, Ordering};
use std::time::{Duration, Instant};

use arc_swap::{ArcSwap, ArcSwapOption};
use clap::{Parser, Subcommand};
use duck_control::fall::{FallPredictor, FallPredictorConfig};
use duck_control::io::RobotIo;
use duck_control::obs::{BodyPose, Command as PolicyCommand};
use duck_control::policy::{DEFAULT_STANDING_THRESHOLD, Policy, PolicyPaths};
use duck_control::safety::{Safety, SafetyConfig};
use duck_control::{DEFAULT_POSITION, FakeIo, NUM_JOINTS};
use duck_ipc_proto as proto;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::{UnixListener, UnixStream};

use control::{Controller, SkillTuning, Tuning};
use intents::Intents;
use params::{Mode, Params};

/// What to do when the shutdown sequence completes. Injected so the tests can observe the
/// call instead of powering off the machine running them.
type PowerOff = Arc<dyn Fn() + Send + Sync>;

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

/// How fast the beak follows the vowel being sung, as a time constant.
///
/// A vowel is a step — `ah` opens the mouth to 0.90 and `mm` to 0.02 — and a servo asked to jump
/// between them on a 20 ms tick twitches rather than sings. This is roughly how fast a jaw moves.
const CHORALE_MOUTH_TAU_S: f64 = 0.09;

/// How far a subscriber may fall behind before it starts losing frames.
///
/// Five seconds at 50 Hz. State is advisory: a client that cannot keep up gets a gap, never
/// backpressure onto the control loop. Same rule the updater applies to progress.
const STATE_BUFFER: usize = 256;

/// Window over which the achieved rate is measured, and therefore how quickly a degraded
/// loop becomes visible to the health gate. Doubles as the slow-sensor sampling interval
/// ([`publish_slow_sensors`]).
const RATE_WINDOW: Duration = Duration::from_secs(1);

/// How long the ramp to the home pose takes when the policy is first enabled.
///
/// The same two seconds `robotd init` uses, and for the same reason: fast enough that nobody wonders
/// whether the button did anything, slow enough that a robot going from limp to standing does not
/// snap. Not a parameter yet — one number with no evidence that anyone wants a different one.
const HOME_RAMP: Duration = Duration::from_secs(2);

/// Smoothing on the reported battery voltage. At one sample per [`RATE_WINDOW`] this is a
/// ~10 s time constant, which is what makes the number readable: the raw voltage sags
/// several tenths of a volt on every step and recovers between them, so an unsmoothed
/// reading swings while the pack is doing nothing unusual. Borrowed, with the figure, from
/// `microduck_runtime`.
const BATTERY_EMA_ALPHA: f64 = 0.1;

/// How long the shutdown sit gets before torque is cut and the machine powers off. The
/// sitstand descent is a deliberate ~2 s glide; the prototype gives it four seconds.
const SHUTDOWN_SIT: Duration = Duration::from_secs(4);

/// How many consecutive failed bus reads the policy may drive through on the last good
/// sample before it stops.
///
/// **One dropped Dynamixel transaction is ordinary** — `robotd` says so itself when it logs
/// one — and it must therefore be *invisible*. It was not: a failed read used to stop the
/// policy for that tick, which commanded the hold pose and reset the controller, so every
/// ordinary dropped read produced a visible twitch. Measured on a bench robot at ~8 drops a
/// minute with a monitor attached, which is exactly the reported "random tiny spasms".
///
/// Coasting is safe because the observation is *already* a tick old by construction: at
/// 50 Hz the policy is trained on data of exactly this age, and a second tick of it is
/// inside that. Three ticks (60 ms) covers a drop, a retry and a slow tick; past that the
/// robot genuinely cannot see, and holding still is the honest answer.
const COAST_TICKS: u32 = 3;

/// How long driving must have been interrupted before the controller is reset.
///
/// The reset zeroes the action history the policy observes and drops the low-pass anchor,
/// which is right after a real pause and is itself a discontinuity after a 20 ms hiccup —
/// the very jolt it exists to prevent.
const RESET_AFTER_PAUSE: Duration = Duration::from_millis(200);

/// Mean leg-joint deviation from the home pose above which a boot counts as seated —
/// hips and knees folded far from standing. The prototype's threshold.
const SEATED_BOOT_RAD: f64 = 0.30;

/// Where the limp-fall sequence is (`[safety] limp_fall`).
///
/// The daemon's only answer to a fall, and it runs *during* one rather than after it: the
/// trigger is [`duck_control::fall::FallPredictor`], not the `fallen` verdict, because a
/// verdict that waits for the robot to be down cannot make it soft before it lands.
///
/// The verdict is still tracked and published throughout — it just does not gate anything.
#[derive(Debug, Clone, Copy, PartialEq)]
enum LimpFall {
    Idle,
    /// Gains at `gain_limp`, joints commanded to wherever they already are — the softest
    /// thing the servos can do short of cutting torque, which would drop the head on the
    /// floor and lose the pose the next phase ramps from.
    Limp {
        since: Instant,
        landing: Landing,
    },
    /// Ramping from where the robot landed back to the standing pose, so the standing
    /// policy starts from the still, known posture it stands up from cleanly.
    Posing {
        from: [f64; NUM_JOINTS],
        since: Instant,
    },
}

/// Watching a limping robot for the moment it stops moving.
///
/// The end of the limp is read from the gyro rather than timed, because the falls are not
/// all the same length: a stumble is over in a few hundred milliseconds and a topple off a
/// table takes most of a second, and ending the limp early is landing stiff after all.
/// Debounced, because a tumbling robot passes through instants of near-zero rate on its
/// way over — one quiet sample is not a landing.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct Landing {
    still_for: Duration,
}

impl Landing {
    /// One tick. `rate` is the gyro magnitude in rad/s, or `None` for a tick with no fresh
    /// sample — which resets rather than accumulates: a robot nobody can read is not a
    /// robot anybody has seen go still.
    fn observe(&mut self, rate: Option<f64>, period: Duration, below: f64, held: Duration) -> bool {
        self.still_for = match rate {
            Some(rate) if rate < below => self.still_for.saturating_add(period),
            _ => Duration::ZERO,
        };
        self.still_for >= held
    }
}

impl LimpFall {
    /// The interpolated pose target for this tick, or `None` once the ramp is done.
    ///
    /// The same linear shape as [`Bringup::homing_target`], and separate from it on
    /// purpose: that ramp is bring-up from limp at the policy gain over a fixed two
    /// seconds, this one is a landed robot being put back into shape, and both its
    /// duration and its gain are tunable because both depend on how the robot lands.
    fn pose_target(&self, now: Instant, over: Duration) -> Option<[f64; NUM_JOINTS]> {
        let LimpFall::Posing { from, since } = self else {
            return None;
        };
        let t = now.duration_since(*since).as_secs_f64() / over.as_secs_f64();
        if t >= 1.0 {
            return None;
        }
        let mut target = [0.0; NUM_JOINTS];
        for (i, slot) in target.iter_mut().enumerate() {
            *slot = from[i] + (DEFAULT_POSITION[i] - from[i]) * t;
        }
        Some(target)
    }
}

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
/// [`Mode`] as the byte [`RobotState::mode`] holds, and back.
///
/// Two tiny functions rather than a numeric cast, so the mapping is written down once: a mode
/// added later gets a code here and nowhere else, and an unknown byte reads as walking rather
/// than panicking on the IPC thread.
fn mode_code(mode: Mode) -> u8 {
    match mode {
        Mode::Walk => 0,
        Mode::Roller => 1,
    }
}

fn mode_of(code: u8) -> Mode {
    match code {
        1 => Mode::Roller,
        _ => Mode::Walk,
    }
}

/// The policy file names for one mode, as a set.
///
/// `None` in a skill slot doubles as "this robot cannot do that", which is what lets `dispatch`
/// refuse a `robot.do` for a skill this mode has no network for instead of queueing a request the
/// loop will drop — and roller mode is exactly that case: no standing network, a crouch where the
/// ground pick was.
#[derive(Debug, Clone, Default)]
struct PolicyNames {
    walk: Option<String>,
    stand: Option<String>,
    sitstand: Option<String>,
    ground_pick: Option<String>,
    kick_left: Option<String>,
    kick_right: Option<String>,
    roulade: Option<String>,
}

impl PolicyNames {
    /// The names for `policy`, or all-`None` when no policy is wanted.
    fn of(policy: &params::ResolvedPolicy) -> Self {
        if !policy.enabled {
            return Self::default();
        }
        let name = |path: &Option<std::path::PathBuf>| path.as_deref().and_then(file_name);
        Self {
            walk: file_name(&policy.walk),
            stand: name(&policy.stand),
            sitstand: name(&policy.sitstand),
            ground_pick: name(&policy.ground_pick),
            kick_left: name(&policy.kick_left),
            kick_right: name(&policy.kick_right),
            roulade: name(&policy.roulade),
        }
    }
}

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
    /// What `btd` should be advertising, published when it changes.
    ///
    /// A broadcast channel like the state stream, and for the same reason: `btd` subscribes, and a
    /// `robotd` with nobody subscribed must not be blocked by that. Small buffer — a subscriber
    /// that has fallen behind on beacons wants the newest one, not a backlog of beats that have
    /// already passed.
    chorale_tx: tokio::sync::broadcast::Sender<proto::ChoraleAdvertise>,
    /// Why the policy is not loaded, if it is not. Set once at startup; the loop keeps
    /// running and holds the pose, so a broken bundle is a rollback rather than a crash.
    policy_error: ArcSwapOption<String>,
    /// Which policy files this process is running, as file names. `None` when the policy is
    /// disabled.
    ///
    /// From the params rather than from the loaded network, and therefore known before the
    /// control thread has finished loading anything — a client that subscribes during startup
    /// gets the answer rather than a race. What *failed* to load is `policy_error`; this is
    /// what was asked for, and the pair is what distinguishes "no policy wanted" from "the
    /// policy this release ships would not load".
    ///
    /// **One swap for the whole set rather than seven fields**, because a mode switch replaces
    /// all of them at once: a reader that caught `walk` from roller beside `stand` from walking
    /// would be told about a robot that does not exist. Swapped by the control loop, which is
    /// the only thing that loads a policy.
    policies: ArcSwap<PolicyNames>,
    /// Whether this robot can make a sound at all: audio enabled, and a bank with wavs in
    /// it. Same job as the `policy_*` fields above — false is "this robot cannot do that",
    /// which is what lets `dispatch` refuse a `robot.sound` with a reason instead of
    /// accepting it into silence. Read once at startup, like the policies: the postinstall
    /// renders the bank and restarts robotd, so a bank cannot appear under a running one.
    has_voice: bool,
    /// Whether a theremin can be picked up: the params allow one, and the depth stream is
    /// actually delivering frames. Published by the loop rather than read once at startup,
    /// because unlike a voice bank the sensor comes and goes — `tofd` restarts, the ToF
    /// drops off the I²C bus — and an accepted theremin on a duck with no depth is a
    /// feature that silently does nothing.
    theremin_ready: AtomicBool,
    /// Whether this robot's config allows it to sing with others. Read once at startup, like the
    /// policies: `[chorale] accept`, false by default.
    chorale_accepted: bool,
    /// `walk` or `roller`, as `robot.mode` reports it.
    ///
    /// Not constant any more: `robot.setMode` switches it while the robot runs, so this is stored
    /// rather than read from the params it started as. An `AtomicU8` over [`Mode`] because it is
    /// read on the IPC side and written by the control loop, and there is nothing to allocate.
    mode: AtomicU8,
    /// Published by the loop so the IPC side can answer without consulting it.
    fallen: AtomicBool,
    /// The policy is driving and has been asked for a non-zero velocity.
    moving: AtomicBool,
    /// Torque is on and the joints are at the home pose, so the policy can drive.
    ///
    /// False covers two different states on purpose — never brought up, and ramping — because a
    /// client can act on neither: the answer to both is "wait, or press Start". The journal
    /// distinguishes them.
    homed: AtomicBool,

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
            chorale_tx: tokio::sync::broadcast::Sender::new(8),
            policy_error: ArcSwapOption::empty(),
            policies: ArcSwap::from_pointee(PolicyNames::of(&params.policy.resolved())),
            has_voice: params.audio.enabled && has_any_wav(&params.audio.bank),
            theremin_ready: AtomicBool::new(false),
            chorale_accepted: params.chorale.accept,
            mode: AtomicU8::new(mode_code(params.policy.mode)),
            fallen: AtomicBool::new(false),
            moving: AtomicBool::new(false),
            homed: AtomicBool::new(false),
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

    duck_ipc_proto::log_startup_identity!("robotd");

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

    // The real thing. `setsid` detaches the command from this process's cgroup, so the
    // poweroff proceeds while systemd is busy killing robotd itself.
    let poweroff: PowerOff = Arc::new(|| {
        let result = std::process::Command::new("setsid")
            .args(["sh", "-c", "systemctl poweroff"])
            .spawn();
        if let Err(e) = result {
            tracing::error!(error = %e, "cannot power off");
        }
    });

    let control = match spawn_control_thread(
        &args,
        &params,
        Arc::clone(&state),
        Arc::clone(&intents),
        poweroff,
    ) {
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
    poweroff: PowerOff,
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
                    poweroff,
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
                    control_loop(io, state, intents, params, period, poweroff).await;
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
/// How far the robot has got towards being drivable.
///
/// This is what made pressing Start on a fresh robot do nothing visible: the policy ran, the loop
/// wrote positions, and the servos ignored them because they had no torque. Torque came from
/// `robotd init` — a separate subcommand that opens the motor bus itself, so it needs the daemon
/// stopped, and which appeared in no documentation.
///
/// The invariant the old design protected is narrower than "never touch torque", and it survives
/// intact: **nothing here runs because a process started.** A `robotd` restarted by an update finds
/// `Limp`, writes nothing new, and leaves a standing robot standing. Only an explicit
/// `robot.enable` moves it on, which is a human pressing Start.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Bringup {
    /// No torque asked for yet. The loop still reads, publishes and holds — it just cannot make the
    /// robot do anything, which is the correct state for a robot nobody has asked to move.
    Limp,
    /// Torque is on and the joints are ramping to the home pose. The policy does not drive yet: it
    /// would be stepping from wherever the robot was slumped, which is exactly the lurch the ramp
    /// exists to avoid.
    Homing {
        from: [f64; NUM_JOINTS],
        since: Instant,
    },
    /// At the home pose with torque on. The policy drives when enabled.
    Ready,
}

impl Bringup {
    /// The interpolated target for this tick, or `None` once the ramp is done.
    ///
    /// Linear, like `DynamixelIo::interpolate_to` which `robotd init` uses — same shape, except this
    /// one is computed per tick instead of blocking the thread, because here the loop is running.
    fn homing_target(&self, now: Instant) -> Option<[f64; NUM_JOINTS]> {
        let Bringup::Homing { from, since } = self else {
            return None;
        };
        let t = now.duration_since(*since).as_secs_f64() / HOME_RAMP.as_secs_f64();
        if t >= 1.0 {
            return None;
        }
        let mut target = [0.0; NUM_JOINTS];
        for (i, slot) in target.iter_mut().enumerate() {
            *slot = from[i] + (DEFAULT_POSITION[i] - from[i]) * t;
        }
        Some(target)
    }
}

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
/// Load one mode's policy bundle, or `None` when there is nothing to load.
///
/// A function rather than a block inline in the loop's preamble, because it runs twice: once at
/// startup, and again on every `robot.setMode` — which is a mode's whole difference, since the
/// networks and the tuning are all a mode *is* by the time params are resolved.
///
/// A policy that was *not wanted* is healthy; one that was wanted and could not be loaded is not.
/// Collapsing those two would either make a bench robot look broken or let a release with an
/// unusable bundle pass the health gate.
fn build_controller(
    policy_cfg: &params::ResolvedPolicy,
    limp_fall: bool,
    state: &RobotState,
) -> Option<Controller> {
    if !policy_cfg.enabled {
        tracing::warn!("policy disabled; holding the startup pose");
        return None;
    }
    {
        let tuning = Tuning {
            action_scale: policy_cfg.action_scale,
            standing_action_scale: policy_cfg.standing_action_scale,
            standing_gain_ratio: policy_cfg.standing_gain_ratio,
            gain: policy_cfg.gain,
            head_lowpass: policy_cfg.head_lowpass,
            legs_lowpass: policy_cfg.legs_lowpass,
        };
        let skills = SkillTuning {
            ground_pick_period: policy_cfg.ground_pick_period,
            ground_pick_action_scale: policy_cfg.ground_pick_action_scale,
            ground_pick_gain_ratio: policy_cfg.ground_pick_gain_ratio,
            kick_duration: policy_cfg.kick_duration,
            roulade_duration: policy_cfg.roulade_duration,
            roulade_action_scale: policy_cfg.roulade_action_scale,
            roulade_gain_ratio: policy_cfg.roulade_gain_ratio,
        };
        let paths = PolicyPaths {
            walk: policy_cfg.walk.clone(),
            stand: policy_cfg.stand.clone(),
            sitstand: policy_cfg.sitstand.clone(),
            ground_pick: policy_cfg.ground_pick.clone(),
            kick_left: policy_cfg.kick_left.clone(),
            kick_right: policy_cfg.kick_right.clone(),
            roulade: policy_cfg.roulade.clone(),
        };
        match Policy::load(&paths, DEFAULT_STANDING_THRESHOLD) {
            Ok(mut policy) => {
                // Roller mode has no standing network — command magnitude stops selecting
                // it. Nothing else reserves it: limp-fall hands back by *letting* the
                // standing network be selected, which is what stands the robot up.
                if policy_cfg.mode == Mode::Roller {
                    policy.set_standing_disabled(true);
                }
                tracing::warn!(
                    mode = policy_cfg.mode.as_str(),
                    walk = %policy_cfg.walk.display(),
                    stand = ?policy_cfg.stand.as_ref().map(|p| p.display().to_string()),
                    sitstand = ?policy_cfg.sitstand.as_ref().map(|p| p.display().to_string()),
                    ground_pick = ?policy_cfg.ground_pick.as_ref().map(|p| p.display().to_string()),
                    kicks = policy_cfg.kick_left.is_some() || policy_cfg.kick_right.is_some(),
                    roulade = ?policy_cfg.roulade.as_ref().map(|p| p.display().to_string()),
                    limp_fall,
                    "policy loaded"
                );
                Some(Controller::new(policy, tuning, skills))
            }
            Err(e) => {
                tracing::error!(error = %e, "policy unavailable; holding the pose");
                state.policy_error.store(Some(Arc::new(e.to_string())));
                None
            }
        }
    }
}

async fn control_loop<T: RobotIo>(
    io: T,
    state: Arc<RobotState>,
    intents: Arc<Intents>,
    params: Params,
    period: Duration,
    poweroff: PowerOff,
) {
    // `mut` because a mode switch replaces it: the resolved policy *is* the mode, once the
    // per-mode defaults have been applied.
    let mut policy_cfg = params.policy.resolved();
    let mut safety = Safety::new(
        io,
        SafetyConfig {
            fall_gravity_z: params.safety.fall_gravity_z,
            fall_debounce: Duration::from_millis(params.safety.fall_debounce_ms),
            deadman: Duration::from_millis(params.safety.deadman_ms),
            gain_running: policy_cfg.gain,
            gain_limp: params.safety.gain_limp,
            // The external per-joint stream's deadman and step clamp default here; wiring them
            // to dedicated params is a `robotd-params` follow-up (see the RFC's External mode).
            ..SafetyConfig::default()
        },
    );

    let Some(mut hold) = adopt_startup_pose(&mut safety, &state, period).await else {
        return;
    };

    // Was the robot powered on already sitting? A seated duck has hips and knees folded
    // far from the standing pose. If so, the first bring-up rises via the sitstand network
    // instead of dragging the legs through the linear ramp — the ramp is for a robot that
    // is roughly standing.
    const LEG_JOINTS: [usize; 10] = [0, 1, 2, 3, 4, 10, 11, 12, 13, 14];
    let leg_deviation = LEG_JOINTS
        .iter()
        .map(|&j| (hold[j] - DEFAULT_POSITION[j]).abs())
        .sum::<f64>()
        / LEG_JOINTS.len() as f64;
    let mut seated_boot = leg_deviation > SEATED_BOOT_RAD;
    if seated_boot {
        tracing::warn!(
            deviation = format!("{leg_deviation:.2}"),
            "seated boot detected — will stand up via the sitstand policy"
        );
    }

    // Loaded once here and again on a mode switch — see `build_controller`.
    let mut controller = build_controller(&policy_cfg, params.safety.limp_fall, &state);

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
    // Whether External drive actually drove last tick, so its falling edge can clear the safety
    // layer's external step-baseline (a later re-entry then rate-limits from the current pose,
    // not a stale target from this session).
    let mut was_external_drive = false;
    let mut bringup = Bringup::Limp;
    // A mode switch in flight: the mode to end up in, once the robot is home. `None` the rest of
    // the time, which is nearly always.
    let mut mode_change: Option<Mode> = None;
    // The policy params this loop is running, which a mode switch replaces. Owned rather than
    // borrowed from `params` because the mode is no longer what the file said.
    let mut policy_params = params.policy.clone();

    // Command smoothing, per the prototype: `cmd += α × (target − cmd)` at the tick rate.
    // A stick snap becomes a ramp the gait can follow; the state lives here because it is
    // per-tick, which the intent slots must not be.
    let dt = period.as_secs_f64();
    let cmd_alpha = params.control.cmd_alpha.clamp(0.0, 1.0);
    let head_alpha = params.control.head_alpha.clamp(0.0, 1.0);
    let mut twist_ema = [0.0f64; 3];
    let mut head_ema = [0.0f64; 4];
    let mut body_ema = [0.0f64; 3];

    // Coasting over a dropped bus read, and where the robot actually is.
    //
    // `known_positions` is updated from every sample that arrives and is what a fallback
    // holds. It used to be `hold`, which is only assigned when driving *stops* — and the
    // assignment needed a sample, which is the one thing a failed read does not have. So a
    // drop mid-stride held whatever `hold` was last set to, which after bring-up is the home
    // pose: one tick of "snap to standing" at full gain, mid-motion.
    let mut coast = Coast::new();
    let mut stopped_driving_at: Option<Instant> = None;

    // Contact odometry, ticked at the loop's own rate on the sample the loop already
    // read — the prototype ran it at 100 Hz on extra bus reads, which is exactly the
    // bus pressure the spasms investigation taught this loop not to add.
    let mut odometry = odometry::Odometry::alpha();

    // The sit-then-power-off sequence.
    let mut shutdown_sit: Option<Instant> = None;
    let mut powered_off = false;
    let mut warned_imu_warming = false;

    // Limp-fall (`[safety] limp_fall`): the predictor that sees a fall start, and where the
    // sequence it drives has got to. Built whether or not the mode is on — it costs a
    // multiply-subtract per tick, and having the numbers computed on every robot is what
    // makes the thresholds tunable against a recording from a robot that was not running
    // the mode.
    let mut falling = FallPredictor::new(FallPredictorConfig {
        tilt_z: params.safety.limp_fall_tilt_z,
        predicted_z: params.safety.limp_fall_predict_z,
        lookahead: Duration::from_millis(params.safety.limp_fall_lookahead_ms),
        debounce: Duration::from_millis(params.safety.limp_fall_debounce_ms),
    });
    let limp_fall_still = Duration::from_millis(params.safety.limp_fall_still_ms);
    let limp_fall_max = Duration::from_millis(params.safety.limp_fall_max_ms);
    let limp_fall_pose = Duration::from_millis(params.safety.limp_fall_pose_ms);
    let mut limp_fall = LimpFall::Idle;

    // The voice, and the ear. Both optional equipment: a robot without a codec or a bank
    // walks identically — the player degrades to a debug line, and the mic worker is only
    // spawned when configured, with its own retry loop when arecord flaps.
    let mut voice = params
        .audio
        .enabled
        .then(|| sound::Sound::new(params.audio.bank.clone(), params.audio.device.clone()));
    let pet: Option<pet_detect::worker::PetHandle> = if params.audio.enabled
        && params.audio.pet_detect_resolved(params.policy.mode)
        && let Some(model) = params.audio.pet_model_resolved()
        && model.exists()
    {
        match pet_detect::worker::PetHandle::spawn(pet_detect::worker::PetConfig {
            alsa_device: params.audio.capture_device(),
            model_path: model.clone(),
            enter_threshold: params.audio.pet_enter_threshold,
            exit_threshold: params.audio.pet_exit_threshold,
        }) {
            Ok(handle) => {
                tracing::warn!(model = %model.display(), "petting detection listening");
                Some(handle)
            }
            Err(e) => {
                // Not unhealthy: the classifier is a feature, not the robot. A missing
                // model on a release that ships one is caught by the packaging tripwires.
                tracing::warn!(error = %e, "petting detection unavailable");
                None
            }
        }
    } else {
        // Configured on but nothing to run: a bench robotd, a board that predates the
        // model, or audio off. Quietly so on purpose — the mic is optional equipment.
        None
    };

    // The theremin. Its depth reader starts now and parks on `tofd`'s socket whether or not
    // anyone ever asks for an instrument: one blocked read costs nothing, and connecting
    // lazily would make the first arming window wait for a connection as well as for
    // frames — a second of silence that reads as a broken feature. Off entirely when the
    // params say so, or when audio is off, since a theremin with no voice is a mouth
    // opening for no reason.
    let mut theremin = (params.theremin.enabled && params.audio.enabled)
        .then(|| theremin::Theremin::spawn(params.theremin.socket.clone(), params.theremin.hand()));
    // How far the beak is open for the chorale, slewed across ticks — see where it is written.
    let mut chorale_mouth = 0.0f64;
    // And how the head sways while singing, applied to the next tick's command.
    let mut chorale_head = [0.0f64; 4];

    // The note the theremin is holding, kept across ticks so a hand leaving the frame fades
    // the note at its own pitch instead of gliding to the bottom of the range on its way out.
    let mut theremin_hz = 0.0f64;

    // The chorale. Off unless the config opted in — see `[chorale] accept`, and note that off
    // means putting nothing on the air rather than declining politely.
    let mut chorale = (params.chorale.accept && params.audio.enabled)
        .then(|| {
            let personality = voice.as_mut().and_then(|v| v.personality())?;
            // `DUCK_CHORALE_PIECE=<id>` pins the conductor's pick — a bench lever for testing
            // one piece, read once like everything else about this process's configuration.
            let forced_piece = std::env::var("DUCK_CHORALE_PIECE")
                .ok()
                .and_then(|v| v.parse::<u8>().ok());
            Some(chorale::Chorale::new(
                personality.pitch_center_hz,
                personality.seed,
                forced_piece,
            ))
        })
        .flatten();
    if chorale.is_some() {
        tracing::warn!("chorale: this robot will sing with others");
    }

    // Say hello in this robot's own voice as the control loop comes up, as the prototype
    // does — the greet is also the audible "robotd is running" on a headless board. Its
    // own switch, because the reason to want it gone (restarting the daemon all day) is
    // not a reason to give up the triggers and the mic that `audio.enabled = false` takes
    // with it.
    if let Some(voice) = voice.as_mut()
        && params.audio.greet
    {
        voice.play("greet", false);
    }

    while !state.shutdown.load(Ordering::Relaxed) {
        ticker.tick().await;
        let tick_start = Instant::now();

        let fresh = match safety.read() {
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

        // Coast over a dropped read on the last good sample — see [`COAST_TICKS`]. The
        // distinction between "fresh" and "what the policy steps from" matters twice below:
        // safety only observes fresh samples, so a repeat cannot feed the fall debounce, and
        // `known_positions` only advances on fresh ones, so a coasted tick cannot pretend to
        // know where the joints have moved to.
        let sensors = coast.sample(fresh);

        if let Some(fresh) = fresh.as_ref() {
            safety.observe(fresh, period);
            // Only once the orientation filter has converged: seeding the anchor from a
            // quaternion that is still swinging would put the world origin somewhere the
            // robot never was. A coasted tick is skipped too — repeating a stale sample
            // into the estimator would tell it the robot froze, which it did not.
            if safety.imu_ready() {
                odometry.update(&fresh.positions, fresh.imu.quat);
            }
        }
        state.fallen.store(safety.fallen(), Ordering::Relaxed);

        let snapshot = intents.snapshot();
        let (gated, deadman) = safety.gate(snapshot.command, snapshot.twist_age);
        let mut limits: Vec<duck_control::safety::Limit> = deadman.into_iter().collect();

        // An explicit `robot.init` / `robot.relax`, taken once.
        //
        // Before the enable-driven bring-up below, so a `relax` that arrives in the same tick as a
        // still-set `enabled` flag wins — `request_relax` clears that flag, and reading the request
        // first means the order cannot invert.
        match intents.take_power_request() {
            Some(intents::PowerRequest::Init) => match (bringup, sensors.as_ref()) {
                // Unlike `enable`, this needs no policy: "stand up" is a reasonable thing to ask of
                // a robot with no walking network at all, and it is what a bench robot needs before
                // anything else can be tested.
                (Bringup::Limp, Some(sensors)) => match safety.set_torque(true) {
                    Ok(()) => {
                        if seated_boot && controller.as_ref().is_some_and(|c| c.has_sitstand()) {
                            seated_boot = false;
                            tracing::warn!(
                                "robot.init: seated boot — rising via the sitstand policy"
                            );
                            controller
                                .as_mut()
                                .expect("checked above")
                                .begin_boot_rise();
                            bringup = Bringup::Ready;
                        } else {
                            tracing::warn!(?HOME_RAMP, "robot.init: torque on, ramping to home");
                            bringup = Bringup::Homing {
                                from: sensors.positions,
                                since: tick_start,
                            };
                        }
                    }
                    Err(e) => tracing::warn!(error = %e, "cannot enable torque"),
                },
                // Already up: ramp back to home from wherever the joints are, as the
                // prototype's init_position always does — Start on a robot stopped
                // mid-crouch must not hand the policy that crouch as its starting pose.
                // The policy holds off while Homing runs and resumes at Ready.
                (Bringup::Ready, Some(sensors)) => {
                    tracing::warn!(?HOME_RAMP, "robot.init: re-homing from the current pose");
                    bringup = Bringup::Homing {
                        from: sensors.positions,
                        since: tick_start,
                    };
                }
                // Mid-ramp, or no sample to ramp from: nothing to do, and saying so beats
                // a silent no-op.
                (state, _) => tracing::info!(?state, "robot.init: nothing to bring up"),
            },
            Some(intents::PowerRequest::Relax) => match safety.set_torque(false) {
                Ok(()) => {
                    tracing::warn!("robot.relax: torque off");
                    // Back to the start, so the next `init` or Start ramps from wherever the robot
                    // ends up rather than assuming it is still at the home pose.
                    bringup = Bringup::Limp;
                    was_driving = false;
                }
                Err(e) => {
                    tracing::warn!(error = %e, "cannot cut torque; the robot is still powered")
                }
            },
            None => {}
        }

        // One-shot skill requests, taken once per tick like the power request. They need a
        // driving robot — the prototype's buttons likewise did nothing until the policy ran.
        let requests = intents.take_skills();
        if requests.any() {
            match controller.as_mut() {
                Some(controller)
                    if snapshot.enabled && bringup == Bringup::Ready && shutdown_sit.is_none() =>
                {
                    let outcome = |what: &str, result: Result<(), &'static str>| match result {
                        Ok(()) => tracing::warn!(skill = what, "skill started"),
                        Err(reason) => tracing::warn!(skill = what, reason, "skill refused"),
                    };
                    if requests.ground_pick {
                        outcome("ground_pick", controller.start_ground_pick());
                    }
                    if requests.kick_left {
                        outcome("kick_left", controller.start_kick(true));
                    }
                    if requests.kick_right {
                        outcome("kick_right", controller.start_kick(false));
                    }
                    if requests.sit_toggle {
                        match controller.sit_toggle() {
                            Ok(direction) => tracing::warn!(direction, "sit toggle"),
                            Err(reason) => tracing::warn!(reason, "sit toggle refused"),
                        }
                    }
                    if requests.roulade {
                        // A held button lands here every tick; only the start of a roll is
                        // journal-worthy. `Ok(false)` is a chain refresh, and stays quiet.
                        match controller.request_roulade() {
                            Ok(true) => tracing::warn!(skill = "roulade", "skill started"),
                            Ok(false) => {}
                            Err(reason) => {
                                tracing::debug!(skill = "roulade", reason, "skill refused");
                            }
                        }
                    }
                }
                _ => tracing::info!("skill request ignored: the policy is not driving"),
            }
        }

        // Sounds: the wheee hold as a level, then the one-shot tags queued by clients. In
        // that order, because the ride owns the PCM while it lasts — the one-shots have to
        // see the ride this tick started (they wait) or the release this tick did (they
        // play). The hold is held only while `padd`'s re-notifications stay fresh; a client
        // that dies mid-ride leaves a ride that lands rather than one that loops forever.
        if let Some(voice) = voice.as_mut() {
            let hold = if powered_off {
                intents::WheeeHold::Released
            } else {
                intents.wheee_hold()
            };
            voice.theremin_settle();
            voice.wheee(hold);
            for tag in intents.take_sounds() {
                voice.play(tag.as_str(), false);
            }

            // Petting: coo, exactly when the prototype coos — not fallen, no scripted move
            // in flight. The verdict is used bare here (not the armed fall gate): this is a
            // sound cue, and cooing while face-down would be worse than staying quiet.
            if let Some(pet) = pet.as_ref() {
                while let Some(ev) = pet.try_recv_event() {
                    match ev {
                        pet_detect::PettingEvent::Start => {
                            let calm =
                                !safety.fallen() && controller.as_ref().is_none_or(|c| !c.busy());
                            if calm {
                                tracing::info!("petting started");
                                voice.play("coo", false);
                            } else {
                                tracing::debug!("petting detected (ignored: busy or down)");
                            }
                        }
                        pet_detect::PettingEvent::End => tracing::debug!("petting ended"),
                    }
                }
                // Ambient sound events have no consumer until the autonomous brain arrives;
                // surfaced at debug so mic tuning on a bench has data to look at.
                while let Some(ev) = pet.try_recv_sound() {
                    tracing::debug!(event = ?ev, "ambient sound");
                }
            }
        }

        // A drive-mode switch: `robot.setMode`, which the pad's held D-pad up sends.
        //
        // The robot goes back to its home pose first and the policies load there, for the reason
        // the prototype relaunched into pose 0: swapping the network under a moving gait means the
        // next tick is a different policy's idea of what the legs were doing. Torque stays on
        // throughout — the robot holds itself up across the swap rather than sitting down.
        if let Some(code) = intents.take_mode_switch() {
            let target = mode_of(code);
            if target == policy_params.mode {
                tracing::info!(
                    mode = target.as_str(),
                    "already in that mode; nothing to switch"
                );
            } else if mode_change.is_some() {
                tracing::warn!(mode = target.as_str(), "a mode switch is already in flight");
            } else {
                tracing::warn!(
                    from = policy_params.mode.as_str(),
                    to = target.as_str(),
                    "mode switch: going home before loading the other policies"
                );
                // The prototype's cue, and worth keeping: one quack for walking, two for roller,
                // so the robot says which mode it is going to without anybody reading a log.
                if let Some(voice) = voice.as_mut() {
                    voice.play("chirp", false);
                    if target == Mode::Roller {
                        voice.play("chirp", false);
                    }
                }
                mode_change = Some(target);
                // Home the robot with the machinery `init` and a fall recovery already use: it
                // ramps per tick, and `driving` is false until it reaches Ready.
                if let Some(sensors) = sensors.as_ref() {
                    bringup = Bringup::Homing {
                        from: sensors.positions,
                        since: tick_start,
                    };
                }
            }
        }

        // The sit-then-power-off sequence: `robot.shutdown`, or a genuinely empty pack.
        // The battery reading is a ~10 s EMA refreshed once a second, so a load sag cannot
        // reach the floor — a pack that gets there is spent.
        let battery_v = f64::from_bits(state.battery_v.load(Ordering::Relaxed));
        let battery_empty = params.safety.battery_empty_shutdown
            && battery_v > 0.0
            && battery_v <= duck_control::model::BATTERY_EMPTY_V;
        if !powered_off && shutdown_sit.is_none() && (intents.take_shutdown() || battery_empty) {
            let can_sit = snapshot.enabled
                && bringup == Bringup::Ready
                && !safety.fallen()
                && controller.as_ref().is_some_and(|c| c.has_sitstand());
            if can_sit {
                tracing::warn!(battery_empty, "shutdown: sitting down before power off");
                // Goodbye peck; non-blocking — it plays out during the sit.
                if let Some(voice) = voice.as_mut() {
                    voice.play("peck", false);
                }
                controller
                    .as_mut()
                    .expect("can_sit checked the controller")
                    .begin_shutdown_sit();
                shutdown_sit = Some(tick_start);
            } else {
                tracing::warn!(battery_empty, "shutdown: cutting torque and powering off");
                if let Err(e) = safety.set_torque(false) {
                    tracing::warn!(error = %e, "cannot cut torque before power off");
                }
                // Goodbye peck; blocking, so it is heard before the poweroff kills this
                // process's cgroup. The one deliberate block in the loop, on its last tick.
                if let Some(voice) = voice.as_mut() {
                    voice.play("peck", true);
                }
                intents.set_enabled(false);
                powered_off = true;
                poweroff();
            }
        }
        if let Some(started) = shutdown_sit
            && !powered_off
            && tick_start.duration_since(started) >= SHUTDOWN_SIT
        {
            tracing::warn!("sit complete: cutting torque and powering off");
            if let Err(e) = safety.set_torque(false) {
                tracing::warn!(error = %e, "cannot cut torque before power off");
            }
            intents.set_enabled(false);
            bringup = Bringup::Limp;
            powered_off = true;
            poweroff();
        }

        // Limp-fall (`[safety] limp_fall`): catch the fall on the way down.
        //
        // The standing policy is a good stand-up-er and a bad faller. It stands a still
        // robot up cleanly from face-down or face-up; asked to do it out of a dynamic fall
        // it tries, fails, tries again — at walking gain, against the floor — and the
        // motors pay for every attempt. So take the fall away from it: go soft before the
        // landing, let the robot arrive limp, put it back into the standing pose, and only
        // then hand it over. What the policy then sees is the case it is good at.
        //
        // Everything here is off the policy's path by construction: `driving` is false for
        // the whole sequence, so the controller is not stepped and the targets below come
        // from this block. Nothing in `safety` needs telling: the fall verdict gates
        // nothing, so the pose ramp moves a robot lying on the floor like any other tick.
        if params.safety.limp_fall {
            // Abandoned mid-sequence: someone cut the torque, disabled the policy, or the
            // shutdown sit started. Whatever took over owns the robot now — drop out rather
            // than finishing a pose ramp nobody asked for, and hold where the robot is
            // rather than snapping back to a pose it collapsed out of a second ago.
            if limp_fall != LimpFall::Idle
                && !(snapshot.enabled
                    && bringup == Bringup::Ready
                    && shutdown_sit.is_none()
                    && !powered_off)
            {
                tracing::warn!("limp-fall abandoned — something else took the robot");
                limp_fall = LimpFall::Idle;
                falling.reset();
                hold = coast.known_positions(hold);
            }
            match limp_fall {
                LimpFall::Idle => {
                    // Only on a fresh sample: a coasted one is the same reading twice, and
                    // feeding it to a debounce would let one bad tick count three times.
                    let eligible = was_driving
                        // Already down. There is no fall left to catch, and this is what
                        // stops the sequence firing at a robot on the floor working its way
                        // upright: a stand-up rocks, and a rock is a tilt with a rate on it.
                        // Limping through that would be an endless limp-pose-retry loop.
                        && !safety.fallen()
                        && shutdown_sit.is_none()
                        && !powered_off
                        // A roulade tips the robot over on purpose, a ground pick pitches it
                        // forward, and a sit is not a fall. Limping through any of them would
                        // break a move that was working.
                        && controller
                            .as_ref()
                            .is_some_and(|c| !c.busy() && !c.is_sitting());
                    match fresh.as_ref() {
                        Some(fresh) if eligible && safety.imu_ready() => {
                            if falling.observe(&fresh.imu, period) {
                                tracing::warn!(
                                    gravity_z = format!("{:.2}", fresh.imu.gravity[2]),
                                    rate =
                                        format!("{:.2}", FallPredictor::gravity_z_rate(&fresh.imu)),
                                    gain = params.safety.gain_limp,
                                    "falling — going limp to land soft"
                                );
                                limp_fall = LimpFall::Limp {
                                    since: tick_start,
                                    landing: Landing::default(),
                                };
                            }
                        }
                        // Not eligible, or blind for this tick. Drop any debounce in
                        // progress rather than resuming it later against a robot that has
                        // moved on.
                        _ => falling.reset(),
                    }
                }
                LimpFall::Limp { since, mut landing } => {
                    let rate = fresh
                        .as_ref()
                        .map(|s| s.imu.gyro.iter().map(|w| w * w).sum::<f64>().sqrt());
                    let landed = landing.observe(
                        rate,
                        period,
                        params.safety.limp_fall_still_rate,
                        limp_fall_still,
                    );
                    // Whatever the gyro says: a robot held in someone's hands, or resting
                    // against something that keeps nudging it, must not stay limp forever.
                    let timed_out = tick_start.duration_since(since) >= limp_fall_max;
                    if landed || timed_out {
                        // From where the robot actually is. `hold` is where it was when
                        // driving stopped, which is a pose it has since collapsed out of —
                        // ramping from that would start with a jump.
                        let from = coast.known_positions(hold);
                        tracing::warn!(
                            limp_ms = tick_start.duration_since(since).as_millis(),
                            timed_out,
                            pose_ms = limp_fall_pose.as_millis(),
                            "landed — posing back to standing"
                        );
                        limp_fall = LimpFall::Posing {
                            from,
                            since: tick_start,
                        };
                    } else {
                        limp_fall = LimpFall::Limp { since, landing };
                    }
                }
                LimpFall::Posing { since, .. } => {
                    if tick_start.duration_since(since) >= limp_fall_pose {
                        tracing::warn!("posed — handing back to the standing policy");
                        limp_fall = LimpFall::Idle;
                        falling.reset();
                        hold = DEFAULT_POSITION;
                        // Hand back to the policy, and let it choose. The twist has been
                        // held at zero through the whole sequence, so with nobody driving
                        // the standing network is what command magnitude selects — which
                        // is the stand-up. A client that *is* driving gets its walk back;
                        // the humans stay in charge, here as everywhere else.
                        if let Some(controller) = controller.as_mut() {
                            controller.reset();
                        }
                    }
                }
            }
        }
        let in_limp_fall = limp_fall != LimpFall::Idle;

        // Smooth the command. The limp-fall sequence holds the twist at zero outright, so
        // the robot is not handed back mid-command; and leaving body-pose mode snaps the
        // body back to nominal rather than gliding, which is its B-button exit.
        let twist_target = if in_limp_fall { [0.0; 3] } else { gated.twist };
        if in_limp_fall {
            twist_ema = [0.0; 3];
        }
        for (ema, target) in twist_ema.iter_mut().zip(twist_target) {
            *ema += cmd_alpha * (target - *ema);
        }
        for (ema, target) in head_ema.iter_mut().zip(gated.head) {
            *ema += head_alpha * (target - *ema);
        }
        if snapshot.pose.active {
            for (ema, target) in body_ema.iter_mut().zip(snapshot.pose.body) {
                *ema += cmd_alpha * (target - *ema);
            }
        } else {
            body_ema = [0.0; 3];
        }
        let command = PolicyCommand {
            twist: twist_ema,
            // The chorale's sway rides on top of whatever the head was asked to do, computed
            // last tick (20 ms stale, invisible at sway speed) and slewed to zero when the
            // singing stops so the head settles rather than snaps.
            head: [
                head_ema[0] + chorale_head[0],
                head_ema[1] + chorale_head[1],
                head_ema[2] + chorale_head[2],
                head_ema[3] + chorale_head[3],
            ],
            body: BodyPose {
                z: body_ema[0],
                roll: body_ema[1],
                pitch: body_ema[2],
            },
        };

        // Bring the robot up when someone asks it to drive and it has no torque yet.
        //
        // Not gated on the fall: a fallen robot ramps like any other, as the prototype
        // does. Being down is a report, and the humans stay in charge.
        //
        // Needs a sample: `from` is where the joints actually are, and starting a ramp from a
        // position nobody read would be the lurch this exists to avoid.
        // `controller.is_some()` as well, because this call means "enable the policy": powering the
        // joints to run a policy that is disabled or would not load is work towards nothing, and it
        // would make a release whose bundle is broken stand the robot up and then hold. A robot that
        // should stand without a policy is what `robotd init` is for.
        if let (Bringup::Limp, true, true, Some(sensors)) = (
            bringup,
            snapshot.enabled,
            controller.is_some(),
            sensors.as_ref(),
        ) {
            match safety.set_torque(true) {
                Ok(()) => {
                    if seated_boot && controller.as_ref().is_some_and(|c| c.has_sitstand()) {
                        // Seated boot: hold the seat and rise via the sitstand network —
                        // the linear ramp would drag folded legs sideways through the floor.
                        seated_boot = false;
                        tracing::warn!("seated boot — rising via the sitstand policy");
                        controller
                            .as_mut()
                            .expect("checked above")
                            .begin_boot_rise();
                        bringup = Bringup::Ready;
                    } else {
                        tracing::warn!(
                            ?HOME_RAMP,
                            "enabling the policy: torque on, ramping to home"
                        );
                        bringup = Bringup::Homing {
                            from: sensors.positions,
                            since: tick_start,
                        };
                    }
                }
                // Reported, not fatal, and it stays `Limp` so the next tick tries again: a bus that
                // dropped one transaction is ordinary, and a robot that refused to ever come up
                // because of it would be worse than one that keeps asking.
                Err(e) => tracing::warn!(error = %e, "cannot enable torque; the robot stays limp"),
            }
        }

        // The ramp finishing is what makes the policy eligible to drive.
        if let Bringup::Homing { .. } = bringup
            && bringup.homing_target(tick_start).is_none()
        {
            // Home, and a switch waiting: load the other mode's bundle here, where the robot is
            // standing still at a known pose with torque on. `Policy::load` validates and warms
            // up each network, so this blocks the loop for a moment — deliberately, and in the one
            // place where a stalled command stream costs nothing, because the robot is holding a
            // pose rather than mid-stride. Missed ticks in that window are expected.
            if let Some(target) = mode_change.take() {
                policy_params.mode = target;
                let cfg = policy_params.resolved();
                state.policy_error.store(None);
                controller = build_controller(&cfg, params.safety.limp_fall, &state);
                // Published together with the mode, and only after the load: a client that reads
                // `robot.mode` and gets the new one must not then be told the old mode's networks.
                state.policies.store(Arc::new(PolicyNames::of(&cfg)));
                state.mode.store(mode_code(target), Ordering::Relaxed);
                policy_cfg = cfg;
                tracing::warn!(
                    mode = target.as_str(),
                    loaded = controller.is_some(),
                    "mode switch complete"
                );
            }
            tracing::warn!("at the home pose; the policy has the robot");
            bringup = Bringup::Ready;
            hold = DEFAULT_POSITION;
        }
        state
            .homed
            .store(bringup == Bringup::Ready, Ordering::Relaxed);

        // The policy must not start on an unconverged orientation filter — the first
        // seconds of projected gravity are whatever the filter is mid-way through deciding,
        // and a gait stepping against that horizon is the prototype's "crazy start".
        let imu_warm = safety.imu_ready();
        if snapshot.enabled && !imu_warm && !warned_imu_warming {
            tracing::warn!(
                "IMU converging (keep the robot still) — the policy starts when it is ready"
            );
            warned_imu_warming = true;
        }

        // Drive only with a sample to drive from: a tick whose read failed has no
        // observation to build, and inventing one would feed the policy a stale robot.
        //
        // And only once the ramp is done, or the policy's first step would come from wherever the
        // robot was slumped. A fall does not stop the driving, as the prototype does not
        // stop it: the policy keeps going and the humans stay in charge.
        //
        // External drive: an off-robot controller streaming absolute joint targets
        // (robot.setJoints) in place of the policy. It owns the joints when engaged AND the robot
        // can hold them — torque on and homed (Ready), not mid limp-fall, not powered off. It
        // needs no IMU warmup and no policy: the tick *is* the supplied targets, clamped by
        // Safety::apply_external (anatomical limits, per-tick step, external deadman). The
        // dispatch door refuses setJoints unless homed, so a stale frame cannot arrive here.
        let external = snapshot.external;
        let external_drive =
            external.is_some() && bringup == Bringup::Ready && !in_limp_fall && !powered_off;

        let driving = snapshot.enabled
            && bringup == Bringup::Ready
            && controller.is_some()
            // The limp-fall sequence owns the robot for its duration: the whole point is
            // that the policy is *not* driving while the robot falls, lands and is posed.
            && !in_limp_fall
            && sensors.is_some()
            && imu_warm
            && !powered_off
            // External drive supersedes the policy: while a controller streams joints, the
            // on-device policy must not also be producing targets for the same servos.
            && external.is_none();

        if driving && !was_driving {
            // Starting fresh: a stale previous action in the observation, or a filter
            // anchored to where the robot was a minute ago, would both show up as a lurch.
            //
            // Only after a real pause, though. Past `COAST_TICKS` of a bad bus this edge
            // fires again within a few ticks, and resetting then re-introduces exactly the
            // discontinuity the reset is for: a zeroed action the policy observes, and a
            // low-pass filter with no anchor.
            if reset_on_resume(stopped_driving_at, tick_start)
                && let Some(controller) = controller.as_mut()
            {
                controller.reset();
            }
        }
        if was_driving && !driving {
            stopped_driving_at = Some(tick_start);
            if !snapshot.enabled {
                // A deliberate stop returns to the home pose — the prototype's Start-off
                // ("policy DISABLED - returning to default pose"). Commanded directly, no
                // ramp: the servos do the travel at their own speed, and the robot is
                // standing at home when Start next hands it to the policy.
                hold = DEFAULT_POSITION;
            } else {
                // Any other stop — IMU cooling, a blind bus, the armed fall gate — freezes
                // where the robot *is*, from the last sample that arrived. Captured once,
                // not re-read each tick, or the hold target would sag under gravity.
                //
                // Deliberately not gated on a sample arriving this tick: the stop that
                // matters most is the one caused by a read failing, and requiring a reading
                // then is requiring the thing that just failed.
                hold = coast.known_positions(hold);
            }
        }
        was_driving = driving;

        if was_external_drive && !external_drive {
            // Left the External stream — forget the last written target, so a later re-entry
            // rate-limits from the robot's current pose rather than a stale command from this
            // session (Safety::clear_external drops the step baseline).
            safety.clear_external();
        }
        was_external_drive = external_drive;

        // Voltage adaptation: the servos' effective kP tracks their supply, so scaling the
        // action by (nominal / measured) holds the robot's response steady as the pack
        // sags. The EMA is clamped to a plausible band so a bad reading cannot become a
        // wild scale.
        let scale_mult = if policy_cfg.voltage_adapt && battery_v > 0.0 {
            policy_cfg.nominal_voltage / battery_v.clamp(6.0, 9.5)
        } else {
            1.0
        };

        let (mut targets, gain, moving, policy_label) = match (driving, sensors.as_ref()) {
            // The limp-fall sequence, before anything else — `driving` is false throughout,
            // so without this it would fall through to the hold branch and the robot would
            // be commanded its pre-fall pose at walking gain, which is precisely the thing
            // the mode exists to stop.
            //
            // `moving` stays true for the whole sequence: the joints are travelling (down,
            // then back to the pose), and `safeToRestart` must not say yes in the middle of
            // a fall.
            _ if in_limp_fall => match limp_fall {
                // Command the joints where they already are, at limp gain. Following the
                // measurement rather than holding a fixed pose is what makes it soft: a
                // fixed target grows an error as the robot collapses, and an error at any
                // gain is a motor pushing back against the floor.
                LimpFall::Limp { .. } => (
                    coast.known_positions(hold),
                    params.safety.gain_limp,
                    true,
                    "limp_fall",
                ),
                LimpFall::Posing { .. } => (
                    // Past the end of the ramp the target is the pose itself — the state
                    // machine above clears `Posing` on the same tick, so this is the one
                    // frame where the two can disagree.
                    limp_fall
                        .pose_target(tick_start, limp_fall_pose)
                        .unwrap_or(DEFAULT_POSITION),
                    params.safety.limp_fall_pose_gain,
                    true,
                    "limp_pose",
                ),
                LimpFall::Idle => unreachable!("in_limp_fall excludes Idle"),
            },
            // External drive, ahead of the policy arms: `driving` is already false while a
            // controller streams (it yields to External above), so the tick is simply the
            // supplied targets. `moving` is true — the joints are travelling to a commanded
            // pose and a restart mid-stream would drop the robot, so `safeToRestart` must not
            // say yes here. The gain is the client's per-frame ask, or the mode's running gain.
            // Everything the frame can violate is clamped downstream by apply_external.
            _ if external_drive => {
                let ext = external.expect("external_drive implies Some");
                let step =
                    control::Step::external(ext.targets, ext.gain.unwrap_or(policy_cfg.gain));
                (step.targets, step.gain, true, step.label)
            }
            (true, Some(sensors)) => {
                let controller = controller.as_mut().expect("driving implies a controller");
                match controller.step(sensors, &command, snapshot.pose.active, dt, scale_mult) {
                    Ok(step) => (
                        step.targets,
                        step.gain,
                        // A scripted move is motion whatever the twist says; so is walking.
                        step.busy || command.twist_magnitude() > 0.0,
                        step.label,
                    ),
                    Err(e) => {
                        tracing::warn!(error = %e, "inference failed; holding");
                        (hold, policy_cfg.gain, false, "held")
                    }
                }
            }
            // Ramping to the home pose. `moving` is true, because it is: the joints are travelling,
            // and `safeToRestart` must not say yes in the middle of it.
            _ if bringup.homing_target(tick_start).is_some() => (
                bringup
                    .homing_target(tick_start)
                    .expect("just checked it is Some"),
                policy_cfg.gain,
                true,
                "homing",
            ),
            _ => (hold, policy_cfg.gain, false, "held"),
        };
        state.moving.store(moving, Ordering::Relaxed);

        // The theremin: a hand's distance in front of the beak, turned into a note and a
        // mouth opening. Before the mouth is written, because while an instrument is up it
        // *is* what the mouth is doing — the intent from a client is not competing with it.
        let mut theremin_state = None;
        if let Some(instrument) = theremin.as_mut() {
            // Whether an instrument could be picked up right now, for the IPC side to
            // refuse on. Republished every tick because the sensor can go away under a
            // running daemon.
            state.theremin_ready.store(
                instrument.has_frames() && voice.is_some(),
                Ordering::Relaxed,
            );
            if let Some(active) = intents.take_theremin_request() {
                instrument.set_active(active);
            }
            // Nothing takes the instrument away but asking. Walking used to drop it, because
            // a captured background is a picture of one spot — with no background there is
            // nothing to invalidate, and a duck that plays while it walks is a feature.
            let note = instrument.tick(tick_start);
            match note {
                Some(note) => {
                    let mut block = note.state;
                    // Silence keeps the last pitch rather than gliding to the bottom of the
                    // range: a fade at the note you were playing is a note ending, and a fade
                    // on the way down is a note falling over.
                    let level = match note.closeness {
                        Some(closeness) => {
                            if let Some(voice) = voice.as_mut()
                                && let Some(hz) = voice.theremin_hz_at(closeness)
                            {
                                theremin_hz = hz;
                            }
                            block.note_hz = Some(theremin_hz);
                            1.0
                        }
                        None => 0.0,
                    };
                    if let Some(voice) = voice.as_mut() {
                        // Idempotent, so this is also what picks the instrument up on the
                        // first tick after the request — there is no separate start edge to
                        // get wrong.
                        if voice.theremin_start() {
                            voice.theremin_set(theremin_hz, level, note.mouth);
                        }
                    }
                    // Whatever the policy is doing, unlike the mouth *intent* below. The
                    // mouth is not part of any policy, and a duck playing a theremin while
                    // sitting — which is how anyone will first try this — has to be able to
                    // open its beak: gating on `driving` made the visible half of the whole
                    // gesture silently absent on a sitting robot.
                    if snapshot.enabled && bringup == Bringup::Ready {
                        targets[duck_control::model::MOUTH_INDEX] =
                            duck_control::model::mouth_target(note.mouth);
                    }
                    theremin_state = Some(block);
                }
                // The instrument is down. Putting the voice down too is idempotent, so this
                // covers both "it was just dropped" and "there has never been one".
                None => {
                    if let Some(voice) = voice.as_mut() {
                        voice.theremin_stop();
                    }
                }
            }
        }

        // The chorale: where in the piece the ensemble is, and this duck's line of it. Before the
        // mouth, like the theremin, because while a duck is singing its beak is doing that.
        let mut chorale_state = None;
        if let Some(ensemble) = chorale.as_mut() {
            if let Some((active, piece_pin)) = intents.take_chorale_request() {
                ensemble.set_active(active, tick_start, piece_pin);
            }
            for heard in intents.take_chorale_heard() {
                ensemble.heard(&heard, tick_start);
            }
            let tick = ensemble.tick(tick_start);
            if let Some(advertise) = tick.advertise {
                // Handed to whatever `btd` connection is subscribed. Only when it changes, which
                // is about once a beat rather than fifty times a second.
                let _ = state.chorale_tx.send(advertise);
            }
            // The mouth: a target from the vowel being sung, written on every tick **while a
            // chorale is up** — which is the bug this replaced. Writing it only while a note
            // actually sounded left the target at whatever the hold pose had put there between
            // notes, so a beak could simply stay open.
            //
            // "While a chorale is up" and not "while this robot may sing", which was the *second*
            // bug: owning the mouth whenever `[chorale] accept` was true meant the trigger could
            // not open it any more, on a robot that was not singing and might never sing. Opting in
            // grants the chorale nothing until it is actually running.
            let mut head_target = [0.0f64; 4];
            let mouth_target = match tick.singing {
                Some((part, beats)) => {
                    if let Some(voice) = voice.as_mut()
                        && voice.sing_start(ensemble.score(), part)
                    {
                        voice.sing_at(beats, true);
                    }
                    let line = ensemble.score().line(part);
                    let (mut low, mut high) = (127.0f64, 0.0f64);
                    let mut current = None;
                    for note in line {
                        low = low.min(f64::from(note.midi));
                        high = high.max(f64::from(note.midi));
                        if beats >= note.start_beat && beats < note.end_beat() {
                            current = Some(*note);
                        }
                    }
                    match current {
                        Some(note) => {
                            // Where this note sits in the duck's own line, for the head lift.
                            let reach = ((f64::from(note.midi) - low) / (high - low).max(1.0))
                                .clamp(0.0, 1.0);
                            head_target = chorale::head_expression(beats, reach);
                            // The audio releases the last 8% of a note so a repeated pitch
                            // re-articulates; the beak does the same, visibly — without this a
                            // run of same-vowel notes reads as one long weird note.
                            let sung = (beats - note.start_beat) / note.beats;
                            if sung < 0.92 {
                                note.vowel.open()
                            } else {
                                note.vowel.open() * 0.2
                            }
                        }
                        // Between notes: beak closed, but keep swaying — a singer breathing is
                        // still part of the choir.
                        None => {
                            head_target = chorale::head_expression(beats, 0.0);
                            0.0
                        }
                    }
                }
                None => {
                    if let Some(voice) = voice.as_mut() {
                        voice.sing_stop();
                    }
                    0.0
                }
            };
            if ensemble.active() {
                // Slewed, not snapped. A vowel is a step — `ah` is 0.90 and `mm` is 0.05 — and a
                // servo asked to jump between them on a 20 ms tick twitches rather than sings.
                // The head rides the same slew: the sway targets are already smooth sinusoids,
                // and the slew is what makes the *start and stop* of singing gentle.
                let alpha = (period.as_secs_f64() / CHORALE_MOUTH_TAU_S).clamp(0.0, 1.0);
                chorale_mouth += (mouth_target - chorale_mouth) * alpha;
                for (offset, target) in chorale_head.iter_mut().zip(head_target) {
                    *offset += (target - *offset) * alpha;
                }
                if snapshot.enabled && bringup == Bringup::Ready {
                    targets[duck_control::model::MOUTH_INDEX] =
                        duck_control::model::mouth_target(chorale_mouth);
                }
                chorale_state = Some(proto::ChoraleState {
                    listening: true,
                    part: tick.singing.map(|(part, _)| part.as_str().to_owned()),
                    beats: tick.singing.map(|(_, beats)| beats),
                    joining: tick.joining,
                    voices: tick.voices as u32,
                });
            } else {
                // Nothing on the wire and nothing on the servo: the mouth belongs to whoever else
                // wants it, which is the trigger — and the head settles back to where it was
                // asked to look, through the same slew rather than a snap.
                chorale_mouth = 0.0;
                let alpha = (period.as_secs_f64() / CHORALE_MOUTH_TAU_S).clamp(0.0, 1.0);
                for offset in chorale_head.iter_mut() {
                    *offset += (0.0 - *offset) * alpha;
                }
            }
        }

        // The mouth is not part of any policy; the intent is the only thing that moves it.
        // Only while driving — a held or homing robot keeps whatever its hold pose says, so
        // a restart cannot snap a mouth.
        if driving && theremin_state.is_none() && chorale_state.is_none() {
            targets[duck_control::model::MOUTH_INDEX] =
                duck_control::model::mouth_target(snapshot.mouth);
        }

        // The External stream goes through the dedicated chokepoint: apply_external adds the
        // per-joint anatomical clamp, the per-tick step limit and the external deadman (which
        // holds `hold` and drops toward limp on a dropped controller) on top of the ordinary
        // actuator clamp. Everything else — policy, homing, limp-fall, hold — uses apply.
        let written = if external_drive {
            let age = external.map(|e| e.age).unwrap_or_default();
            safety.apply_external(targets, hold, age, gain)
        } else {
            safety.apply(targets, hold, gain)
        };
        match written {
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
                    // Limp means "the robot is actually at limp gain" — which now happens
                    // in exactly one place, the limp-fall sequence riding a fall down. A
                    // bare fall verdict is a report, not a state.
                    limp: matches!(limp_fall, LimpFall::Limp { .. }),
                    gravity: sensors.imu.gravity,
                    gain: safety.gain(),
                },
                control_loop: proto::LoopState {
                    hz: f64::from_bits(state.achieved_hz.load(Ordering::Relaxed)),
                    missed: state.missed.load(Ordering::Relaxed),
                },
                joints: sensors.positions.to_vec(),
                targets: targets.to_vec(),
                odom: proto::OdomState {
                    position: odometry.position(),
                    yaw: odometry.yaw(),
                },
                theremin: theremin_state.clone(),
                chorale: chorale_state.clone(),
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

/// The sample a tick steps from, and how stale it may get.
///
/// Exists as a type rather than three variables in the loop because it *was* three variables
/// in the loop, and the bug lived in the gap between them: a failed read stopped the policy
/// for one tick, which commanded `hold` — a pose only ever assigned when driving stops, and
/// the assignment needed a sample, which is the one thing a failed read does not have. Mid
/// stride that meant one tick of "snap to the home pose" at full gain, then a controller
/// reset on the way back in. Roughly eight times a minute on a loaded board.
struct Coast {
    last: Option<duck_control::Sensors>,
    ticks: u32,
}

impl Coast {
    fn new() -> Self {
        Self {
            last: None,
            ticks: 0,
        }
    }

    /// Feed a tick's read result; get back what the policy may step from.
    fn sample(&mut self, fresh: Option<duck_control::Sensors>) -> Option<duck_control::Sensors> {
        match fresh {
            Some(sensors) => {
                self.ticks = 0;
                self.last = Some(sensors);
                Some(sensors)
            }
            None if self.ticks < COAST_TICKS => {
                self.ticks += 1;
                self.last
            }
            None => None,
        }
    }

    /// Where the joints were the last time anything was actually read — what a fallback
    /// holds. `fallback` covers the ticks before the first successful read.
    fn known_positions(&self, fallback: [f64; NUM_JOINTS]) -> [f64; NUM_JOINTS] {
        self.last.map_or(fallback, |sensors| sensors.positions)
    }
}

/// Whether resuming from `stopped_at` warrants resetting the controller.
///
/// The reset zeroes the action history the policy observes and drops the low-pass anchor.
/// After a real pause that is right — the robot may be somewhere else entirely. After a
/// 20 ms bus hiccup it is itself the discontinuity the reset exists to prevent.
fn reset_on_resume(stopped_at: Option<Instant>, now: Instant) -> bool {
    stopped_at.is_none_or(|since| now.duration_since(since) >= RESET_AFTER_PAUSE)
}

/// Whether a voice bank has anything to play. One directory walk at startup: the bank is a
/// tree of per-tag directories, and an empty one (postinstall's `ensure-bank` failed, which
/// the hook deliberately downgrades to a warning) is exactly the case a `robot.sound` must
/// not answer "accepted" to.
fn has_any_wav(bank: &std::path::Path) -> bool {
    let Ok(tags) = std::fs::read_dir(bank) else {
        return false;
    };
    tags.filter_map(Result::ok).any(|tag| {
        std::fs::read_dir(tag.path()).is_ok_and(|mut wavs| {
            wavs.any(|w| w.is_ok_and(|w| w.path().extension().is_some_and(|ext| ext == "wav")))
        })
    })
}

fn limit_name(limit: duck_control::safety::Limit) -> &'static str {
    use duck_control::safety::Limit;
    match limit {
        Limit::Deadman => "deadman",
        Limit::Range => "joint_range",
        Limit::Step => "external_step",
        Limit::NotFinite => "not_finite",
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
    // What to advertise, once `btd` has subscribed. A second stream on the same connection, which
    // is why the read loop selects over a pair of options rather than one receiver.
    let mut beacons: Option<tokio::sync::broadcast::Receiver<proto::ChoraleAdvertise>> = None;
    let mut decimate = Duration::ZERO;
    let mut last_sent: Option<Instant> = None;

    loop {
        // Three things can happen: a request arrives, a state frame is due for a subscriber, or a
        // beacon is due for `btd`. Written as nested selects rather than one, because the two
        // streams are independent options and a client normally has neither.
        let line = match (states.as_mut(), beacons.as_mut()) {
            (None, None) => lines.next_line().await?,
            (None, Some(rx)) => {
                tokio::select! {
                    line = lines.next_line() => line?,
                    received = rx.recv() => {
                        match received {
                            Ok(advertise) => {
                                write_line(
                                    &mut write_half,
                                    &proto::Request::notify(&proto::Call::ChoraleBeaconSet(advertise)),
                                )
                                .await?;
                            }
                            // Lagged: `btd` fell behind on beats. The newest beacon is the only
                            // one worth having — an old beat is a beat that has already passed.
                            Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                                tracing::debug!(dropped = n, "chorale subscriber fell behind");
                            }
                            Err(tokio::sync::broadcast::error::RecvError::Closed) => return Ok(()),
                        }
                        continue;
                    }
                }
            }
            (Some(rx), beacon_rx) => {
                let mut pending = beacon_rx;
                tokio::select! {
                    line = lines.next_line() => line?,
                    received = async {
                        match pending.as_mut() {
                            Some(rx) => rx.recv().await,
                            // Never resolves, so the select falls to the other arms.
                            None => std::future::pending().await,
                        }
                    } => {
                        match received {
                            Ok(advertise) => {
                                write_line(
                                    &mut write_half,
                                    &proto::Request::notify(&proto::Call::ChoraleBeaconSet(advertise)),
                                )
                                .await?;
                            }
                            Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                                tracing::debug!(dropped = n, "chorale subscriber fell behind");
                            }
                            Err(tokio::sync::broadcast::error::RecvError::Closed) => return Ok(()),
                        }
                        continue;
                    }
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

        if let Ok(proto::Call::ChoraleSubscribe) = &call {
            // `btd` asking what to put on the air. One connection carries both directions: this
            // stream down, and `chorale.heard` notifications up.
            beacons = Some(state.chorale_tx.subscribe());
        }

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
        proto::Call::RobotPose(p) => {
            intents.set_pose(intents::PoseIntent {
                body: [p.z, p.roll, p.pitch],
                active: p.active,
            });
            true
        }
        proto::Call::RobotMouth(p) => {
            intents.set_mouth(p.open);
            true
        }
        // A beacon `btd` heard. A notification because it is one-way and frequent, and because
        // there is nothing to say back about a beat.
        proto::Call::ChoraleHeard(p) => {
            intents.heard_chorale(p.clone());
            true
        }
        // A skill as a notification: how a client spells "the button is held" — `padd`
        // resends `roulade` every tick to keep a chain alive, and an answer per resend
        // would be pure overhead. The one-shot press stays a request (`dispatch` refuses
        // it with a reason); the loop arbitrates either way.
        proto::Call::RobotDo(p) => {
            intents.request_skill(p.skill);
            true
        }
        // Same story for the wheee hold, which arrives per tick while the trigger is down.
        proto::Call::RobotSound(p) => {
            intents.request_sound(*p);
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
        proto::Call::RobotMove(_)
        | proto::Call::RobotHead(_)
        | proto::Call::RobotPose(_)
        | proto::Call::RobotMouth(_) => {
            apply_intent(intents, call);
            proto::Response::ok(Some(id), &proto::IntentResult::accepted())
        }

        // Gaze as a point: the IK runs here, against the same MJCF the policies train on,
        // and the answer is the joints the head was sent to — so a client can hold the gaze
        // by resending them as `robot.head`, or notice `clamped` and move the robot instead.
        // Never refused: an aim is an intent like `robot.head`, and the closest-possible
        // gaze at a clamped target is still the most useful thing the head can do.
        proto::Call::RobotLook(p) => {
            static HEAD_FK: std::sync::LazyLock<kinematics::head::HeadFk> =
                std::sync::LazyLock::new(kinematics::head::HeadFk::alpha);
            let gaze = HEAD_FK.look_at([p.x, p.y, p.z], p.neck_pitch);
            intents.set_head(gaze.joints);
            let [neck_pitch, head_pitch, head_yaw, head_roll] = gaze.joints;
            proto::Response::ok(
                Some(id),
                &proto::LookResult {
                    head: proto::HeadParams {
                        neck_pitch,
                        head_pitch,
                        head_yaw,
                        head_roll,
                    },
                    clamped: gaze.clamped,
                },
            )
        }

        // A skill request is queued for the loop's next tick. Refused here only for what
        // this side can already know — the skill was never configured, or the robot is
        // down; the loop still arbitrates against whatever move is mid-flight, exactly as
        // the prototype's buttons did.
        proto::Call::RobotDo(p) => {
            let policies = state.policies.load();
            let configured = match p.skill {
                // One load for the whole decision: these are the *current* mode's networks, and
                // reading them field by field could straddle a mode switch.
                proto::Skill::GroundPick => policies.ground_pick.is_some(),
                proto::Skill::KickLeft => policies.kick_left.is_some(),
                proto::Skill::KickRight => policies.kick_right.is_some(),
                proto::Skill::SitToggle => policies.sitstand.is_some(),
                proto::Skill::Roulade => policies.roulade.is_some(),
            };
            // Not refused for being down. A skill on a fallen robot is the human's call,
            // and refusing it is how a robot ends up unable to do the thing that would
            // have righted it.
            let result = if !configured {
                proto::IntentResult::refused("no policy configured for that skill")
            } else {
                intents.request_skill(p.skill);
                proto::IntentResult::accepted()
            };
            proto::Response::ok(Some(id), &result)
        }

        // Picking the theremin up, or putting it down. Refused here for what this side can
        // already know — no voice, no depth frames, the feature switched off — and *not* for
        // circumstance: the loop puts the instrument down by itself when the robot starts
        // walking or goes over, which is a thing that happens after the answer.
        //
        // Note what the answer does not promise: arming. That takes about half a second of
        // depth frames, because the theremin's zero is whatever is in front of the duck at
        // that moment, and one frame is not a background. Whether it took, and the one
        // refusal a player will actually hit — a hand already in front of the beak — arrive
        // in `robot.state`'s theremin block.
        proto::Call::RobotTheremin(p) => {
            let result = if !state.has_voice {
                proto::ThereminResult {
                    accepted: false,
                    reason: Some("this robot has no voice to play a theremin in".to_owned()),
                }
            } else if p.active && !state.theremin_ready.load(Ordering::Relaxed) {
                proto::ThereminResult {
                    accepted: false,
                    reason: Some(
                        "no depth frames — the ToF sensor is not delivering (is tofd running?)"
                            .to_owned(),
                    ),
                }
            } else {
                intents.request_theremin(p.active);
                proto::ThereminResult {
                    accepted: true,
                    reason: None,
                }
            };
            proto::Response::ok(Some(id), &result)
        }

        // A sound is queued for the loop's next tick. Never refused for *circumstance* —
        // unlike a skill it moves nothing, and a chirp out of a fallen robot is
        // diagnostics, not danger — but refused when this robot has no voice at all, the
        // same way `robot.do` refuses a skill that was never configured.
        //
        // That refusal is the whole value of `robotctl quack`: its job is to tell you which
        // duck you are talking to, and an accepted-then-silent quack inverts the answer —
        // you hear nothing, conclude you are on the wrong duck, and go looking.
        proto::Call::RobotSound(p) => {
            let result = if state.has_voice {
                intents.request_sound(*p);
                proto::IntentResult::accepted()
            } else {
                proto::IntentResult::refused(
                    "this robot has no voice: audio is disabled, or its bank is empty \
                     (run `sounds ensure-bank`)",
                )
            };
            proto::Response::ok(Some(id), &result)
        }

        // Sit, then power the machine off. Never refused for being inconvenient — a robot
        // that cannot sit (no sitstand policy, not driving) cuts torque and powers off
        // directly, which is still what was asked for.
        proto::Call::RobotShutdown => {
            intents.request_shutdown();
            proto::Response::ok(Some(id), &proto::IntentResult::accepted())
        }

        // Switch drive mode. Refused here for the two things this side knows: a mode nobody has
        // heard of, and a robot with no policy to switch between — the loop arbitrates the rest.
        //
        // "Already in that mode" is an acceptance rather than a refusal: the caller asked for a
        // state and that state is what they get, and a pad held a beat too long should not report
        // an error.
        proto::Call::RobotSetMode(p) => {
            // "external" is a drive *source*, not a policy mode: it hands the joints to an
            // off-robot controller via robot.setJoints, needs no policy loaded, and does not
            // touch the walk/roller mode machinery. walk/roller leave External and hand the
            // joints back to the policy.
            let result = if p.mode.as_str() == "external" {
                intents.set_external_mode(true);
                proto::IntentResult::accepted()
            } else {
                let target = match p.mode.as_str() {
                    "walk" => Some(Mode::Walk),
                    "roller" => Some(Mode::Roller),
                    _ => None,
                };
                match target {
                    None => proto::IntentResult::refused(
                        "mode must be \"walk\", \"roller\" or \"external\"",
                    ),
                    Some(_) if state.policies.load().walk.is_none() => {
                        proto::IntentResult::refused(
                            "no policy on this robot, so there is nothing to switch between",
                        )
                    }
                    Some(mode) => {
                        // Handing back to the policy: leave External first, so the loop stops
                        // driving external targets the moment the switch is requested.
                        intents.set_external_mode(false);
                        intents.request_mode_switch(mode_code(mode));
                        proto::IntentResult::accepted()
                    }
                }
            };
            proto::Response::ok(Some(id), &result)
        }

        proto::Call::RobotMode => proto::Response::ok(
            Some(id),
            &proto::ModeResult {
                // External is a drive source layered over the policy mode: while it is engaged
                // it is what is actually driving the joints, so it is what `robot.mode` reports.
                mode: if intents.external_engaged() {
                    "external".to_owned()
                } else {
                    mode_of(state.mode.load(Ordering::Relaxed))
                        .as_str()
                        .to_owned()
                },
            },
        ),

        // Handled by the caller, which owns the connection; answering here keeps the
        // request/response pairing in one place.
        // The acknowledgement carries the policy identity: it is constant for the life of the
        // process, so sending it once here costs nothing, where putting it on every frame
        // would allocate two strings per tick on the control thread.
        // Start or stop looking for other ducks. Refused for what this side can know: no voice to
        // sing with, and — the one that matters — a robot whose config has not opted in. A chorale
        // moves the mouth and the head, so an un-opted-in robot does not merely decline, it never
        // goes on the air at all.
        proto::Call::RobotChorale(p) => {
            let result = if !state.chorale_accepted {
                proto::ChoraleResult {
                    accepted: false,
                    reason: Some(
                        "this robot has not opted in to singing with others (`[chorale] accept` \
                         in robotd.toml)"
                            .to_owned(),
                    ),
                }
            } else if !state.has_voice {
                proto::ChoraleResult {
                    accepted: false,
                    reason: Some("this robot has no voice to sing with".to_owned()),
                }
            } else if let Some(id) = p.piece.filter(|id| !chorale::known_piece(*id)) {
                // Refused at the door with the catalogue, rather than accepted into the coin:
                // a pin that silently did not pin is exactly the confusion it exists to end.
                proto::ChoraleResult {
                    accepted: false,
                    reason: Some(format!(
                        "piece {id} is not on this robot — it has {}",
                        chorale::piece_catalogue()
                    )),
                }
            } else {
                intents.request_chorale(p.active, p.piece);
                proto::ChoraleResult {
                    accepted: true,
                    reason: None,
                }
            };
            proto::Response::ok(Some(id), &result)
        }

        // `btd` subscribing to what to advertise. The stream itself is set up by the caller; this
        // only acknowledges, and is deliberately not refused for a robot that has not opted in —
        // `btd` may subscribe at boot and the answer can change without it reconnecting.
        proto::Call::ChoraleSubscribe => proto::Response::ok(
            Some(id),
            &proto::ChoraleResult {
                accepted: true,
                reason: None,
            },
        ),

        proto::Call::RobotSubscribe(_) => {
            let policies = state.policies.load();
            proto::Response::ok(
                Some(id),
                &proto::SubscribeResult {
                    accepted: true,
                    walk: policies.walk.clone(),
                    stand: policies.stand.clone(),
                    sitstand: policies.sitstand.clone(),
                    ground_pick: policies.ground_pick.clone(),
                    kick_left: policies.kick_left.clone(),
                    kick_right: policies.kick_right.clone(),
                    roulade: policies.roulade.clone(),
                    unavailable: state.policy_error.load_full().map_or_else(
                        || {
                            policies.walk.is_none().then(|| {
                                "no policy configured; holding the startup pose".to_owned()
                            })
                        },
                        |e| Some(format!("policy would not load: {e}")),
                    ),
                },
            )
        }

        proto::Call::RobotStop => {
            intents.stop();
            proto::Response::ok(Some(id), &proto::IntentResult::accepted())
        }

        // Stream absolute joint targets from an off-robot controller. Accepting a frame enters
        // External drive implicitly (the RFC's `setMode external` is the explicit door). Refused,
        // with a named reason, when the request cannot be honoured safely:
        //  - the wrong number of targets — the vector is positional (`JOINT_NAMES` order), so a
        //    short or long one is a client bug, refused rather than zero-padded into a lurch;
        //  - a non-finite target — refused here for a clear error, and again in the safety layer;
        //  - the joints not powered and at home (`homed`) — a limp robot cannot hold a commanded
        //    position, so streaming to it is a silent no-op dressed as motion; `robot.init` first.
        // Everything the frame *can* violate once accepted — anatomical limits, per-tick step,
        // the external deadman — is the safety layer's job (`Safety::apply_external`), not this
        // door's.
        proto::Call::RobotSetJoints(p) => {
            use duck_control::NUM_JOINTS;
            let result = if p.targets.len() != NUM_JOINTS {
                proto::IntentResult::refused(format!(
                    "robot.setJoints needs exactly {NUM_JOINTS} joint targets in JOINT_NAMES \
                     order, got {}",
                    p.targets.len()
                ))
            } else if p.targets.iter().any(|v| !v.is_finite()) {
                proto::IntentResult::refused("robot.setJoints targets must all be finite")
            } else if !state.homed.load(Ordering::Relaxed) {
                proto::IntentResult::refused(
                    "robot.setJoints needs the joints powered and at the home pose — call \
                     robot.init first (a limp robot cannot hold a commanded position)",
                )
            } else {
                let mut targets = [0.0f64; NUM_JOINTS];
                targets.copy_from_slice(&p.targets);
                intents.set_external(targets, p.gain);
                proto::IntentResult::accepted()
            };
            proto::Response::ok(Some(id), &result)
        }

        // A refusal here is a normal answer with a reason, not an error: the client asked
        // something reasonable and the daemon declined. Gravity is never one of those
        // reasons — see below.
        proto::Call::RobotEnable(p) => {
            // `toggle` flips the robot's own state — the pad's Start. Evaluated here, not
            // in the client, because a client-side belief drifts (relax, shutdown, either
            // side restarting) and a stale one turns Start into a no-op every other press.
            let on = if p.toggle { !intents.enabled() } else { p.on };
            // Enabling the policy hands the joints back from any External drive — the two are
            // alternatives, and a stale external target must not shadow the policy that is now
            // asked to drive.
            if on {
                intents.set_external_mode(false);
            }
            // Never refused for being down. Start on a robot lying on the floor is exactly
            // how someone asks it to stand back up, and it brings the robot up and hands it
            // to the standing policy like any other enable.
            let result = {
                intents.set_enabled(on);
                // No init here: stopping is what returns the robot to its home pose (the
                // loop commands it directly on the disable — the prototype's "returning
                // to default pose"), so the next start already begins from home. From
                // limp, the enable-triggered bring-up still ramps torque on as ever.
                proto::IntentResult {
                    accepted: true,
                    reason: Some(
                        if on {
                            "enabled — driving"
                        } else {
                            "disabled — returning to the home pose"
                        }
                        .to_owned(),
                    ),
                }
            };
            proto::Response::ok(Some(id), &result)
        }

        // Power to the joints, which is the pair `robot.enable` is not: enabling asks the *policy*
        // to drive and brings a limp robot up on the way, while these two are the decision itself.
        //
        // Both only *ask*. The control loop owns the only `RobotIo` handle, so nothing here touches
        // the bus — which is also why `robotd init` needs the daemon stopped and these do not.
        proto::Call::RobotInit => {
            // Never refused for gravity: init works whatever the robot is lying on, which
            // is the prototype's behaviour and what a bench actually needs.
            let result = {
                intents.request_init();
                proto::IntentResult::accepted()
            };
            proto::Response::ok(Some(id), &result)
        }

        // Never refused. Going limp is always safe *for the robot* — it is the people around it who
        // need to know, which is why `robotctl` asks for `--yes` and BLE cannot reach this at all.
        proto::Call::RobotRelax => {
            intents.request_relax();
            proto::Response::ok(Some(id), &proto::IntentResult::accepted())
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

    /// The limp-fall pose ramp: starts where the robot landed, ends at the standing pose,
    /// and reports itself finished rather than pinning at the end — the state machine reads
    /// `None` as "hand back to the policy".
    #[test]
    fn the_limp_fall_pose_ramp_ends_at_the_standing_pose() {
        let landed = [0.7; NUM_JOINTS];
        let over = Duration::from_secs(1);
        let since = Instant::now();
        let posing = LimpFall::Posing {
            from: landed,
            since,
        };

        let start = posing.pose_target(since, over).expect("t = 0");
        assert_eq!(start, landed, "the ramp starts from where the robot landed");

        let half = posing
            .pose_target(since + over / 2, over)
            .expect("mid-ramp");
        for (i, value) in half.iter().enumerate() {
            let expected = landed[i] + (DEFAULT_POSITION[i] - landed[i]) * 0.5;
            assert!((value - expected).abs() < 1e-9);
        }

        assert!(
            posing.pose_target(since + over, over).is_none(),
            "a finished ramp is None, not the endpoint held forever"
        );
    }

    /// Only the posing phase has a ramp. Asking the limp phase for one must not hand back
    /// a target — that phase follows the joints down, it does not drive them anywhere.
    #[test]
    fn only_the_posing_phase_ramps() {
        let over = Duration::from_secs(1);
        let now = Instant::now();
        assert!(LimpFall::Idle.pose_target(now, over).is_none());
        assert!(
            LimpFall::Limp {
                since: now,
                landing: Landing::default(),
            }
            .pose_target(now, over)
            .is_none()
        );
    }

    /// The landing detector. The limp ends when the robot has been still for the full hold,
    /// not on the first quiet sample — a robot tumbling over passes through instants of
    /// near-zero rate on the way, and ending the limp on one of those is landing stiff
    /// after all.
    #[test]
    fn the_landing_needs_the_robot_to_stay_still() {
        let period = Duration::from_millis(20);
        let held = Duration::from_millis(60);
        let mut landing = Landing::default();

        // Still, but not for long enough yet.
        assert!(!landing.observe(Some(0.1), period, 1.0, held));
        assert!(!landing.observe(Some(0.1), period, 1.0, held));
        // One tumbling sample restarts the count from zero.
        assert!(!landing.observe(Some(4.0), period, 1.0, held));
        assert!(!landing.observe(Some(0.1), period, 1.0, held));
        assert!(!landing.observe(Some(0.1), period, 1.0, held));
        assert!(
            landing.observe(Some(0.1), period, 1.0, held),
            "three ticks still"
        );
    }

    /// A tick with no fresh sample is not evidence of stillness. A robot nobody can read is
    /// not a robot anybody has seen stop moving, and a bad bus must not end the limp.
    #[test]
    fn a_blind_tick_is_not_a_landing() {
        let period = Duration::from_millis(20);
        let held = Duration::from_millis(60);
        let mut landing = Landing::default();

        assert!(!landing.observe(Some(0.1), period, 1.0, held));
        assert!(!landing.observe(Some(0.1), period, 1.0, held));
        assert!(
            !landing.observe(None, period, 1.0, held),
            "blind, not still"
        );
        assert!(
            !landing.observe(Some(0.1), period, 1.0, held),
            "counting again"
        );
    }

    /// Limp-fall ships ON, and it must refuse a fallen robot nothing.
    ///
    /// This is the contract that answers "I booted it face-down and pressed Start": enable
    /// and init are never refused for gravity, whatever the mode is set to. It matters more
    /// now the mode is a default than it did when it was opt-in — every robot has it.
    #[test]
    fn limp_fall_ships_on_and_refuses_nothing() {
        let params = Params::default();
        assert!(params.safety.limp_fall, "on by default, fleet-wide");

        let s = RobotState::new(&params, false, false);
        let intents = Arc::new(Intents::new());
        let id = || proto::Id::Number(1);
        s.fallen.store(true, Ordering::Relaxed);

        let enabled: proto::IntentResult = dispatch(
            &s,
            &intents,
            id(),
            &proto::Call::RobotEnable(proto::EnableParams {
                on: true,
                toggle: false,
            }),
        )
        .result_as()
        .unwrap();
        assert!(
            enabled.accepted,
            "Start on a robot lying on the floor is how someone asks it to stand up"
        );
        assert!(intents.enabled());

        let init: proto::IntentResult = dispatch(&s, &intents, id(), &proto::Call::RobotInit)
            .result_as()
            .unwrap();
        assert!(init.accepted, "nor may init refuse for gravity");
    }

    fn state() -> RobotState {
        RobotState::new(&Params::default(), false, false)
    }

    /// A poweroff that powers nothing off — the loop under test must never reach the real
    /// one, and a test that wants to observe the call builds its own counter.
    fn noop_poweroff() -> PowerOff {
        Arc::new(|| {})
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
    /// `robot.setMode` is accepted, refused or a no-op — and never a parse error.
    ///
    /// The refusals are what this test is for. A robot with no policy has nothing to switch
    /// between, and a mode nobody has heard of must come back naming the two that exist rather
    /// than as a decode failure the caller cannot act on.
    #[test]
    fn setting_the_mode_accepts_refuses_and_says_which() {
        let params = Params::default();
        assert_eq!(params.policy.mode, Mode::Walk, "the shipped default");
        let s = RobotState::new(&params, false, false);
        let intents = Arc::new(Intents::new());
        let id = || proto::Id::Number(1);
        let set = |mode: &str| -> proto::IntentResult {
            dispatch(
                &s,
                &intents,
                id(),
                &proto::Call::RobotSetMode(proto::SetModeParams {
                    mode: mode.to_owned(),
                }),
            )
            .result
            .expect("a result")
            .as_object()
            .map(|o| serde_json::from_value(serde_json::Value::Object(o.clone())).expect("shape"))
            .expect("an object")
        };

        // Default params name a walking bundle, so there is something to switch between.
        assert!(set("roller").accepted, "a real mode is accepted");
        assert_eq!(
            intents.take_mode_switch(),
            Some(mode_code(Mode::Roller)),
            "the loop is the thing that switches; dispatch only queues it"
        );

        // The same mode is an acceptance, not a refusal: the caller asked for a state.
        assert!(set("walk").accepted);
        assert_eq!(intents.take_mode_switch(), Some(mode_code(Mode::Walk)));

        let refused = set("hovercraft");
        assert!(!refused.accepted);
        let reason = refused.reason.unwrap_or_default();
        assert!(
            reason.contains("walk") && reason.contains("roller"),
            "{reason}"
        );
        assert_eq!(
            intents.take_mode_switch(),
            None,
            "a refused switch must not reach the loop"
        );

        // "external" is a drive source, not a policy mode: accepted, engages External, and
        // does NOT queue a walk/roller mode switch.
        assert!(set("external").accepted, "external is a valid mode");
        assert!(intents.external_engaged(), "external must engage the mode");
        assert_eq!(
            intents.take_mode_switch(),
            None,
            "external is not a policy-mode switch"
        );

        // Switching back to a policy mode leaves External and hands the joints back.
        assert!(set("walk").accepted);
        assert!(!intents.external_engaged(), "walk must leave External");
        assert_eq!(intents.take_mode_switch(), Some(mode_code(Mode::Walk)));

        // A robot with no policy at all: nothing to switch between, and saying so beats homing
        // the robot for a swap that would load nothing.
        let mut bare = Params::default();
        bare.policy.enabled = false;
        let s = RobotState::new(&bare, false, false);
        let response = dispatch(
            &s,
            &intents,
            id(),
            &proto::Call::RobotSetMode(proto::SetModeParams {
                mode: "roller".to_owned(),
            }),
        );
        let result: proto::IntentResult =
            serde_json::from_value(response.result.expect("a result")).expect("shape");
        assert!(!result.accepted);
        assert!(
            result.reason.unwrap_or_default().contains("no policy"),
            "the reason must name the cause"
        );
    }

    /// `robot.setJoints`: the dispatch door for the External stream. It engages External and
    /// stores the frame only when the request is well-formed AND the robot can hold it (powered
    /// and homed); otherwise it refuses with a named reason and touches nothing. Everything the
    /// frame can violate once accepted is the safety layer's job, not this test's.
    #[test]
    fn robot_set_joints_gates_the_external_stream() {
        use duck_control::model::{DEFAULT_POSITION, NUM_JOINTS};

        let s = RobotState::new(&Params::default(), false, false);
        let intents = Arc::new(Intents::new());
        let id = || proto::Id::Number(1);
        let call = |targets: Vec<f64>, gain: Option<u16>| -> proto::IntentResult {
            dispatch(
                &s,
                &intents,
                id(),
                &proto::Call::RobotSetJoints(proto::SetJointsParams { targets, gain }),
            )
            .result_as()
            .expect("IntentResult")
        };

        // A limp robot (not homed) refuses — a commanded position it cannot hold is a silent
        // no-op, so say so rather than pretend to move.
        assert!(!s.homed.load(Ordering::Relaxed));
        let limp = call(DEFAULT_POSITION.to_vec(), None);
        assert!(!limp.accepted);
        assert!(
            limp.reason.unwrap_or_default().contains("robot.init"),
            "the refusal must point at the fix"
        );
        assert!(
            !intents.external_engaged(),
            "a refused frame must not engage External"
        );

        // Power and home it; now the door is open.
        s.homed.store(true, Ordering::Relaxed);

        // Wrong arity is refused, with both counts, and still engages nothing.
        let short = call(vec![0.0; NUM_JOINTS - 1], None);
        assert!(!short.accepted);
        let reason = short.reason.unwrap_or_default();
        assert!(reason.contains(&NUM_JOINTS.to_string()), "{reason}");
        assert!(!intents.external_engaged());

        // A non-finite target is refused at the door too.
        let mut poisoned = DEFAULT_POSITION.to_vec();
        poisoned[3] = f64::NAN;
        let nan = call(poisoned, None);
        assert!(!nan.accepted);
        assert!(nan.reason.unwrap_or_default().contains("finite"));
        assert!(!intents.external_engaged());

        // A well-formed frame on a ready robot is accepted, engages External implicitly, and the
        // targets + gain reach the snapshot the loop reads.
        let mut targets = DEFAULT_POSITION;
        targets[0] += 0.1;
        let ok = call(targets.to_vec(), Some(180));
        assert!(ok.accepted, "a valid frame on a ready robot is accepted");
        assert!(
            intents.external_engaged(),
            "accepting a frame enters External implicitly"
        );
        let ext = intents.snapshot().external.expect("External is engaged");
        assert_eq!(ext.targets, targets);
        assert_eq!(ext.gain, Some(180));
    }

    /// Roller mode has no standing network, and the published set must say so.
    ///
    /// This is the reason the names are swapped as a set rather than left at what startup
    /// resolved: `dispatch` refuses a `robot.do` for a skill whose slot is `None`, so a stale set
    /// would have the robot accepting a kick in a mode with no kick network — or refusing the
    /// crouch that roller mode does have.
    #[test]
    fn the_published_policy_names_are_one_modes_answer() {
        let walk = PolicyNames::of(&Params::default().policy.resolved());
        assert!(walk.stand.is_some(), "walking has a standing network");

        let mut rolling = Params::default();
        rolling.policy.mode = Mode::Roller;
        let roller = PolicyNames::of(&rolling.policy.resolved());
        assert!(
            roller.stand.is_none(),
            "roller mode loads no standing network"
        );
        assert_ne!(
            walk.walk, roller.walk,
            "the driving network differs by mode"
        );
        assert_ne!(
            walk.ground_pick, roller.ground_pick,
            "the ground-pick slot holds the crouch in roller mode"
        );

        // Disabled means every slot empty, which is what `robot.subscribe` reports as "no policy
        // configured" rather than as a load failure.
        let mut off = Params::default();
        off.policy.enabled = false;
        let none = PolicyNames::of(&off.policy.resolved());
        assert!(none.walk.is_none() && none.roulade.is_none());
    }

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

        let mode: proto::ModeResult = dispatch(&s, &Intents::new(), id(), &proto::Call::RobotMode)
            .result_as()
            .expect("robot.mode must deserialize as ModeResult");
        assert_eq!(mode.mode, "walk");
    }

    /// `robot.look` is a promise with two halves: the head actually moves (the intent is
    /// set), and the answer names the joints it moves to — so a client can hold the gaze by
    /// resending them as `robot.head`. A left-of-robot target must come back with a
    /// left-turning yaw, or the IK's sign conventions broke between the crate and the wire.
    #[test]
    fn robot_look_moves_the_head_and_answers_with_the_joints() {
        let intents = Intents::new();
        let state = RobotState::new(&Params::default(), false, false);
        let look: proto::LookResult = dispatch(
            &state,
            &intents,
            proto::Id::Number(1),
            &proto::Call::RobotLook(proto::LookParams {
                x: 0.5,
                y: 0.5,
                z: 0.0,
                neck_pitch: 0.1,
            }),
        )
        .result_as()
        .expect("robot.look must answer with a LookResult");

        assert!(!look.clamped, "an ahead-left point is well inside reach");
        assert!(look.head.head_yaw > 0.2, "left target, leftward yaw");
        assert!((look.head.neck_pitch - 0.1).abs() < 1e-9, "posture is held");

        let sent = intents.snapshot().command.head;
        assert_eq!(
            sent,
            [
                look.head.neck_pitch,
                look.head.head_pitch,
                look.head.head_yaw,
                look.head.head_roll
            ],
            "the answer must be exactly what the head was sent"
        );
    }

    /// `robotctl quack` exists to answer "which duck am I talking to", and it answers by
    /// making a noise. A robot that cannot make one must say so — accepting the call and
    /// staying silent inverts the answer, sending whoever asked off to look for a duck they
    /// were already connected to.
    #[test]
    fn robot_sound_is_refused_by_a_robot_with_no_voice() {
        let intents = Intents::new();
        let id = || proto::Id::Number(1);
        let quack = || {
            proto::Call::RobotSound(proto::SoundParams {
                tag: proto::SoundTag::Chirp,
                hold: None,
            })
        };

        // Audio off: the robot is quiet by configuration, and says so.
        let mut params = Params::default();
        params.audio.enabled = false;
        let mute = RobotState::new(&params, false, false);
        let refused: proto::IntentResult = dispatch(&mute, &intents, id(), &quack())
            .result_as()
            .unwrap();
        assert!(!refused.accepted);
        assert!(refused.reason.is_some(), "a refusal must say why");
        assert!(intents.take_sounds().is_empty(), "a refusal must not queue");

        // Audio on, but the bank never rendered (`ensure-bank` failed, which the
        // postinstall downgrades to a warning): same silence, so the same answer.
        let empty = std::env::temp_dir().join(format!("bank-empty-{}", std::process::id()));
        std::fs::create_dir_all(&empty).unwrap();
        let mut params = Params::default();
        params.audio.bank = empty.clone();
        let bankless = RobotState::new(&params, false, false);
        let refused: proto::IntentResult = dispatch(&bankless, &intents, id(), &quack())
            .result_as()
            .unwrap();
        assert!(!refused.accepted, "an empty bank is a robot with no voice");

        // A rendered bank: accepted, and actually queued for the loop to play.
        std::fs::create_dir_all(empty.join("chirp")).unwrap();
        std::fs::write(empty.join("chirp/chirp_a.wav"), b"RIFF").unwrap();
        let voiced = RobotState::new(&params, false, false);
        let accepted: proto::IntentResult = dispatch(&voiced, &intents, id(), &quack())
            .result_as()
            .unwrap();
        assert!(accepted.accepted);
        assert_eq!(intents.take_sounds(), vec![proto::SoundTag::Chirp]);
        std::fs::remove_dir_all(&empty).ok();
    }

    /// A skill whose network was never configured is refused at the door with a reason —
    /// not queued for a loop that would silently drop it. And a configured one is queued:
    /// the request must reach the intents for the loop to take.
    #[test]
    fn robot_do_refuses_what_is_not_configured_and_queues_what_is() {
        let s = state(); // default params: every walk-mode skill configured
        let intents = Intents::new();
        let id = || proto::Id::Number(1);

        let accepted: proto::IntentResult = dispatch(
            &s,
            &intents,
            id(),
            &proto::Call::RobotDo(proto::DoParams {
                skill: proto::Skill::GroundPick,
            }),
        )
        .result_as()
        .unwrap();
        assert!(accepted.accepted);
        assert!(
            intents.take_skills().ground_pick,
            "the request must be queued"
        );

        // A slot disabled with the `"none"` sentinel is a robot without that skill; asking
        // must refuse, not queue. (The roller preset no longer serves as the example — its
        // rebased line carries the kicks and the sit like walking does.)
        let mut params = Params::default();
        params.policy.kick_left = Some("none".into());
        let unconfigured = RobotState::new(&params, false, false);
        let refused: proto::IntentResult = dispatch(
            &unconfigured,
            &intents,
            id(),
            &proto::Call::RobotDo(proto::DoParams {
                skill: proto::Skill::KickLeft,
            }),
        )
        .result_as()
        .unwrap();
        assert!(!refused.accepted);
        assert!(!intents.take_skills().kick_left, "a refusal must not queue");

        // A fall never refuses a skill: `fallen` is a report, not a wall. Refusing here is
        // how a robot ends up unable to do the thing that would have righted it.
        s.fallen.store(true, Ordering::Relaxed);
        let down: proto::IntentResult = dispatch(
            &s,
            &intents,
            id(),
            &proto::Call::RobotDo(proto::DoParams {
                skill: proto::Skill::SitToggle,
            }),
        )
        .result_as()
        .unwrap();
        assert!(down.accepted, "fallen must not refuse a skill");
        assert!(intents.take_skills().sit_toggle);
    }

    /// The pose and mouth intents land in their slots like move and head do — including via
    /// the notification path, which is how they actually arrive at 50 Hz.
    #[test]
    fn pose_and_mouth_intents_reach_their_slots() {
        let intents = Intents::new();
        assert!(apply_intent(
            &intents,
            &proto::Call::RobotPose(proto::PoseParams {
                z: -0.01,
                roll: 0.1,
                pitch: -0.2,
                active: true,
            }),
        ));
        assert!(apply_intent(
            &intents,
            &proto::Call::RobotMouth(proto::MouthParams { open: 0.5 }),
        ));

        let snap = intents.snapshot();
        assert_eq!(snap.pose.body, [-0.01, 0.1, -0.2]);
        assert!(snap.pose.active);
        assert_eq!(snap.mouth, 0.5);
    }

    /// `robot.shutdown` on a robot that cannot sit (no policy driving) cuts torque and
    /// powers off — the request must never be silently lost. The sit-first path needs a
    /// policy and therefore ONNX Runtime, so it is exercised on a board rather than here.
    #[tokio::test(start_paused = true)]
    async fn a_shutdown_request_without_a_sit_cuts_torque_and_powers_off() {
        let mut params = Params::default();
        params.policy.enabled = false;
        let s = Arc::new(RobotState::new(&params, false, false));
        let intents = Arc::new(Intents::new());
        intents.request_shutdown();

        let powered = Arc::new(AtomicBool::new(false));
        let observed = Arc::clone(&powered);
        let poweroff: PowerOff = Arc::new(move || observed.store(true, Ordering::Relaxed));

        // The battery must read as healthy, or this would also exercise the empty-pack path.
        let loop_state = Arc::clone(&s);
        let handle = tokio::spawn(control_loop(
            FakeIo::at(DEFAULT_POSITION),
            loop_state,
            Arc::clone(&intents),
            params,
            Duration::from_millis(2),
            poweroff,
        ));

        let deadline = Instant::now() + Duration::from_secs(5);
        while !powered.load(Ordering::Relaxed) && Instant::now() < deadline {
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        s.shutdown.store(true, Ordering::Relaxed);
        handle.await.unwrap();

        assert!(
            powered.load(Ordering::Relaxed),
            "poweroff must have been asked for"
        );
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
        params.policy.walk = Some("/opt/robot/releases/7/alpha_walking.onnx".into());
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

    /// **The spasm.** One dropped bus read is ordinary — `robotd` logs it as such — and it
    /// must be invisible. It was not: the tick lost its sample, so the policy stopped, the
    /// loop commanded `hold`, and `hold` was the home pose left over from bring-up. On a
    /// board with a monitor attached that happened ~8 times a minute, each one a jerk toward
    /// standing mid-stride, followed by a controller reset on the way back.
    ///
    /// Three assertions, one per link in that chain.
    #[test]
    fn a_dropped_read_is_survived_rather_than_jerked_through() {
        let mut walking = duck_control::Sensors::default();
        walking.positions[2] = 0.9; // mid-stride, nowhere near the home pose
        let mut coast = Coast::new();

        // A fresh sample is what it says it is, and it is remembered.
        assert_eq!(coast.sample(Some(walking)), Some(walking));
        assert_eq!(coast.known_positions(DEFAULT_POSITION), walking.positions);

        // 1. A drop keeps the policy driving on the last sample rather than dropping it.
        for tick in 1..=COAST_TICKS {
            assert_eq!(
                coast.sample(None),
                Some(walking),
                "drop {tick} must coast, not go blind"
            );
        }

        // 2. Past the coast the robot really is blind, and says so.
        assert_eq!(
            coast.sample(None),
            None,
            "a bus that stays down must stop the policy"
        );

        // 3. And even then the fallback holds where the robot *was*, never the home pose:
        // this is the assertion whose absence was the bug.
        assert_eq!(
            coast.known_positions(DEFAULT_POSITION),
            walking.positions,
            "a blind tick must hold the last known pose, not snap to home"
        );
        assert_ne!(coast.known_positions(DEFAULT_POSITION), DEFAULT_POSITION);

        // A sample arriving clears the coast, so the next drop gets the full allowance.
        assert_eq!(coast.sample(Some(walking)), Some(walking));
        assert_eq!(coast.sample(None), Some(walking));
    }

    /// The other half of the same jerk: coming back from a one-tick interruption must not
    /// reset the controller, because that zeroes the action the policy observes and drops the
    /// low-pass anchor — the discontinuity the reset is meant to avoid.
    #[test]
    fn a_hiccup_does_not_reset_the_controller_but_a_pause_does() {
        let now = Instant::now();

        assert!(
            !reset_on_resume(Some(now - Duration::from_millis(20)), now),
            "one tick of a bad bus is not a pause"
        );
        assert!(
            !reset_on_resume(Some(now - (RESET_AFTER_PAUSE / 2)), now),
            "nor is anything inside the window"
        );
        assert!(
            reset_on_resume(Some(now - RESET_AFTER_PAUSE), now),
            "a real pause resets"
        );
        assert!(
            reset_on_resume(None, now),
            "the first time the policy ever drives is a reset"
        );
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
        params.policy.walk = Some(PathBuf::from("/nonexistent/definitely-not-a-policy.onnx"));
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
            noop_poweroff(),
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
            noop_poweroff(),
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
            noop_poweroff(),
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
        control_loop_probe_with(io, state, Arc::new(Intents::new()), period).await
    }

    /// As above, with the intents the caller wants to drive — for the tests that need to press
    /// Start.
    async fn control_loop_probe_with<T: RobotIo>(
        io: &mut T,
        state: Arc<RobotState>,
        intents: Arc<Intents>,
        period: Duration,
    ) {
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
            fn set_torque(&mut self, on: bool) -> duck_control::io::Result<()> {
                self.0.set_torque(on)
            }
            fn slow_sensors(&mut self) -> duck_control::io::Result<duck_control::SlowSensors> {
                self.0.slow_sensors()
            }
        }
        control_loop(
            Borrowed(io),
            state,
            intents,
            Params::default(),
            period,
            noop_poweroff(),
        )
        .await
    }

    /// **A restart must not move the robot**, and that is the invariant the old "never touch torque"
    /// rule existed to protect. It is unchanged: the loop reads, publishes and holds, and asks for no
    /// torque at all until someone enables the policy.
    ///
    /// `torque: None` is the assertion — not `Some(false)`. Nothing wrote to those registers, so an
    /// update that restarts `robotd` mid-stand leaves the servos exactly as they were.
    #[tokio::test]
    async fn a_restart_asks_for_no_torque() {
        let io = FakeIo::at(DEFAULT_POSITION).frozen();
        let s = Arc::new(RobotState::new(&Params::default(), false, false));
        let (tx, rx) = std::sync::mpsc::channel();
        let loop_state = Arc::clone(&s);
        let handle = tokio::spawn(async move {
            let mut io = io;
            control_loop_probe(&mut io, loop_state, Duration::from_millis(2)).await;
            tx.send((io.torque, io.torque_writes)).unwrap();
        });

        while s.ticks.load(Ordering::Relaxed) < 5 {
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        s.shutdown.store(true, Ordering::Relaxed);
        handle.await.unwrap();

        let (torque, writes) = rx.recv().unwrap();
        assert_eq!(torque, None, "the loop powered the joints on its own");
        assert_eq!(writes, 0, "torque was written {writes} times at startup");
        assert!(!s.homed.load(Ordering::Relaxed));
    }

    /// Enabling a policy that is not there must not power the joints either.
    ///
    /// `robot.enable` means "enable the policy", so bringing the robot up to run one that is disabled
    /// or would not load is work towards nothing — and on a release whose bundle is broken it would
    /// stand the robot up and then hold it there, which reads as working. A robot that should stand
    /// without a policy is what `robotd init` is for.
    #[tokio::test]
    async fn enabling_without_a_policy_leaves_the_joints_limp() {
        let io = FakeIo::at(DEFAULT_POSITION).frozen();
        let mut params = Params::default();
        params.policy.enabled = false;
        let s = Arc::new(RobotState::new(&params, false, false));
        let intents = Arc::new(Intents::new());
        intents.set_enabled(true);

        let (tx, rx) = std::sync::mpsc::channel();
        let loop_state = Arc::clone(&s);
        let loop_intents = Arc::clone(&intents);
        let handle = tokio::spawn(async move {
            let mut io = io;
            control_loop_probe_with(&mut io, loop_state, loop_intents, Duration::from_millis(2))
                .await;
            tx.send(io.torque).unwrap();
        });

        while s.ticks.load(Ordering::Relaxed) < 5 {
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        s.shutdown.store(true, Ordering::Relaxed);
        handle.await.unwrap();

        assert_eq!(rx.recv().unwrap(), None, "powered up with no policy to run");
        assert!(!s.homed.load(Ordering::Relaxed));
    }

    /// **`robot.init` powers the joints and ramps, with no policy anywhere.**
    ///
    /// This is the path CI could not reach before: `enable` requires a loaded policy, and there is no
    /// ONNX Runtime here. `init` deliberately does not, because standing up is a reasonable thing to
    /// ask of a robot with no walking network — which also makes the whole bring-up testable.
    #[tokio::test]
    async fn robot_init_powers_the_joints_and_ramps_home() {
        let mut resting = DEFAULT_POSITION;
        resting[0] = DEFAULT_POSITION[0] + 0.4;
        // `frozen`, so reported positions do not chase the targets: the ramp has to be driven by the
        // clock rather than by the robot appearing to arrive.
        let io = FakeIo::at(resting).frozen();

        let mut params = Params::default();
        params.policy.enabled = false;
        let s = Arc::new(RobotState::new(&params, false, false));
        let intents = Arc::new(Intents::new());
        intents.request_init();

        let (tx, rx) = std::sync::mpsc::channel();
        let loop_state = Arc::clone(&s);
        let loop_intents = Arc::clone(&intents);
        let handle = tokio::spawn(async move {
            let mut io = io;
            control_loop_probe_with(&mut io, loop_state, loop_intents, Duration::from_millis(2))
                .await;
            tx.send((io.torque, io.torque_writes, io.last_written))
                .unwrap();
        });

        while s.ticks.load(Ordering::Relaxed) < 5 {
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        s.shutdown.store(true, Ordering::Relaxed);
        handle.await.unwrap();

        let (torque, writes, written) = rx.recv().unwrap();
        assert_eq!(torque, Some(true), "init did not power the joints");
        assert_eq!(writes, 1, "torque written {writes} times, not once");

        // Mid-ramp: commanded somewhere between where it was and home, not either end. The ramp
        // being real is the point — a jump straight to home is the lurch it exists to avoid.
        let written = written.expect("the loop must command something").positions;
        let (from, to) = (resting[0], DEFAULT_POSITION[0]);
        assert!(
            written[0] < from && written[0] > to,
            "commanded {} outside the ramp {from}..{to}",
            written[0]
        );
    }

    /// **External drive reaches the bus.** Once the robot is powered and homed, a client
    /// streaming `robot.setJoints` supersedes the policy: the loop writes the supplied joint
    /// targets (through `Safety::apply_external`) rather than a policy or hold pose. The head_yaw
    /// target is chosen past one tick's step so the ramp is real; the other joints must stay at
    /// home, proving it is the external frame — not a policy — driving.
    #[tokio::test]
    async fn external_drive_streams_joint_targets_to_the_bus() {
        // Not frozen: reported positions follow the writes, so the external step-limiter advances
        // its baseline toward the target tick by tick, as it would on the real bus.
        let io = FakeIo::at(DEFAULT_POSITION);
        let mut params = Params::default();
        params.policy.enabled = false;
        let s = Arc::new(RobotState::new(&params, false, false));
        let intents = Arc::new(Intents::new());
        intents.request_init();

        let mut target = DEFAULT_POSITION;
        const HEAD_YAW: usize = 7; // wide range (±1.4), home 0.0
        target[HEAD_YAW] = 0.5; // past external_max_step (0.2), so it takes a few ticks

        let (tx, rx) = std::sync::mpsc::channel();
        let loop_state = Arc::clone(&s);
        let loop_intents = Arc::clone(&intents);
        let handle = tokio::spawn(async move {
            let mut io = io;
            control_loop_probe_with(&mut io, loop_state, loop_intents, Duration::from_millis(2))
                .await;
            tx.send(io.last_written).unwrap();
        });

        // Wait for the 2 s home ramp to complete — external drive requires a powered, homed robot.
        let homed_by = Instant::now() + Duration::from_secs(6);
        while !s.homed.load(Ordering::Relaxed) {
            assert!(Instant::now() < homed_by, "robot never homed");
            tokio::time::sleep(Duration::from_millis(5)).await;
        }

        // Stream the frame for well over the external deadman (150 ms) so the step-limited ramp
        // reaches the target — a real controller re-sends every tick.
        let stream_until = Instant::now() + Duration::from_millis(250);
        while Instant::now() < stream_until {
            intents.set_external(target, None);
            tokio::time::sleep(Duration::from_millis(3)).await;
        }
        s.shutdown.store(true, Ordering::Relaxed);
        handle.await.unwrap();

        let written = rx.recv().unwrap().expect("the loop must command something");
        assert!(
            (written.positions[HEAD_YAW] - target[HEAD_YAW]).abs() < 0.05,
            "external drive did not reach the head_yaw target: wrote {}, wanted {}",
            written.positions[HEAD_YAW],
            target[HEAD_YAW]
        );
        // The joints the client did not move stay home — it is the external frame driving, not a
        // policy that would have moved the legs.
        assert!(
            (written.positions[0] - DEFAULT_POSITION[0]).abs() < 0.05,
            "a non-commanded joint drifted: hip_yaw wrote {}",
            written.positions[0]
        );
    }

    /// **`robot.relax` cuts power and goes back to the start**, so the next bring-up ramps from
    /// wherever the robot ended up rather than assuming it is still standing at home.
    #[tokio::test]
    async fn robot_relax_cuts_power_and_returns_to_limp() {
        let io = FakeIo::at(DEFAULT_POSITION).frozen();
        let mut params = Params::default();
        params.policy.enabled = false;
        let s = Arc::new(RobotState::new(&params, false, false));
        let intents = Arc::new(Intents::new());
        intents.request_init();

        let (tx, rx) = std::sync::mpsc::channel();
        let loop_state = Arc::clone(&s);
        let loop_intents = Arc::clone(&intents);
        let handle = tokio::spawn(async move {
            let mut io = io;
            control_loop_probe_with(&mut io, loop_state, loop_intents, Duration::from_millis(2))
                .await;
            tx.send((io.torque, io.torque_writes)).unwrap();
        });

        // Let the init land, then let go.
        while s.ticks.load(Ordering::Relaxed) < 3 {
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        let ticks_at_relax = s.ticks.load(Ordering::Relaxed);
        intents.request_relax();
        while s.ticks.load(Ordering::Relaxed) < ticks_at_relax + 3 {
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        s.shutdown.store(true, Ordering::Relaxed);
        handle.await.unwrap();

        let (torque, writes) = rx.recv().unwrap();
        assert_eq!(torque, Some(false), "relax left the joints powered");
        assert_eq!(writes, 2, "expected one on and one off, got {writes}");
        assert!(
            !s.homed.load(Ordering::Relaxed),
            "still reporting homed after relax"
        );
    }

    /// Relaxing clears `enabled` too. Otherwise the very next tick sees a robot that was asked to
    /// drive, brings it back up, and the robot someone just let go of stands up again.
    #[test]
    fn relaxing_stops_the_policy_asking_to_drive() {
        let intents = Intents::new();
        intents.set_enabled(true);
        intents.request_relax();
        assert!(!intents.snapshot().enabled);
        assert_eq!(
            intents.take_power_request(),
            Some(intents::PowerRequest::Relax)
        );
        // Taken once: a bus transaction per joint is not something to repeat every tick.
        assert_eq!(intents.take_power_request(), None);
    }

    /// The last request wins. Asking to stand up and then to let go within one tick must not stand
    /// the robot up.
    #[test]
    fn the_later_power_request_replaces_the_earlier_one() {
        let intents = Intents::new();
        intents.request_init();
        intents.request_relax();
        assert_eq!(
            intents.take_power_request(),
            Some(intents::PowerRequest::Relax)
        );
    }

    /// The ramp itself, which is the part that decides whether a robot stands up or snaps.
    ///
    /// Tested directly because the loop cannot reach it in CI: bringing up requires a loaded policy,
    /// and there is no ONNX Runtime here. What runs on the board is this arithmetic plus one
    /// `set_torque`.
    #[test]
    fn the_home_ramp_starts_where_the_robot_is_and_ends_at_home() {
        let mut resting = DEFAULT_POSITION;
        resting[0] = DEFAULT_POSITION[0] + 0.5;
        let since = Instant::now();
        let bringup = Bringup::Homing {
            from: resting,
            since,
        };

        // At the start it commands where the robot already is: no step, no lurch.
        let first = bringup.homing_target(since).expect("ramping");
        assert_eq!(first, resting);

        // Halfway is halfway, per joint.
        let mid = bringup
            .homing_target(since + HOME_RAMP / 2)
            .expect("still ramping");
        assert!(
            (mid[0] - (resting[0] + DEFAULT_POSITION[0]) / 2.0).abs() < 1e-6,
            "{}",
            mid[0]
        );

        // And it ends — `None` is what promotes the state to `Ready`, so a ramp that never
        // finished would leave the policy permanently locked out.
        assert!(bringup.homing_target(since + HOME_RAMP).is_none());
        assert!(
            bringup
                .homing_target(since + HOME_RAMP + Duration::from_secs(1))
                .is_none()
        );

        // Neither other state ramps anything.
        assert!(Bringup::Limp.homing_target(since).is_none());
        assert!(Bringup::Ready.homing_target(since).is_none());
    }
}
