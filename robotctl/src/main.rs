//! `robotctl` — local CLI for the robot.
//!
//! A **thin client over `updaterd`'s unix socket**: parse argv, send one JSON-RPC
//! request, print the streamed notifications and result, map the outcome to an
//! exit code. It contains no update logic — that lives in the engine inside
//! `updaterd`. Same relationship `btd` has to the socket, different transport:
//!
//! ```text
//!   phone ──▶ btd ──────┐
//!                       ├──▶ /run/updaterd.sock ──▶ updaterd
//!   you / CI ─▶ robotctl┘
//! ```
//!
//! Scope: **only the `update` namespace is implemented.** The `robotctl` name is
//! kept for the eventual general-purpose robot CLI, and commands are namespaced
//! from the start so scripts written today keep working when other namespaces are
//! added.
//!
//! Two audiences, and the second dictates the design rules:
//!  - support and field recovery, when the app or BLE isn't an option;
//!  - CI and bench testing, where every operation must be scriptable
//!    (`docs/updater-design.md` §16.1).
//!
//! Therefore:
//!  - **No prompts, ever.** Nothing here may ask a question.
//!  - **Idempotent.** Re-running a command that already holds is success, so
//!    scripts needn't branch on current state.
//!  - **Exit codes are meaningful**, so tests assert on them without parsing text.
//!  - **Notifications to stderr, results to stdout**, so `--json` stays pipeable
//!    while progress stays visible.
//!  - Works when `robotd` is dead. It talks to `updaterd`, not to `robotd`.

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use clap::{Args, CommandFactory, Parser, Subcommand};
use duck_ipc_proto as proto;

/// Exit codes. Stable — CI asserts on these.
mod exit {
    pub const OK: u8 = 0;
    pub const FAILED: u8 = 1;
    /// Bad usage. Matches clap's own convention.
    pub const USAGE: u8 = 2;
    /// `updaterd` unreachable — a different problem from a rejected command.
    pub const UNREACHABLE: u8 = 3;
    /// Another update is in flight. Distinct so scripts retry rather than fail.
    pub const BUSY: u8 = 4;
    /// Refused: incompatible, or preflight failed. Distinct so a test can assert
    /// "correctly rejected" rather than "something broke" — needed for the
    /// bad-signature and wrong-hardware cases.
    pub const REFUSED: u8 = 5;
    /// Not permitted to change this robot. Distinct from REFUSED: the request was
    /// well-formed and applicable, the caller just isn't allowed — so the fix is
    /// "run as root / ask an administrator", not "try something else".
    pub const DENIED: u8 = 6;
}

#[derive(Parser, Debug)]
#[command(
    name = "robotctl",
    about = "Local robot control",
    version,
    subcommand_required = true,
    arg_required_else_help = true
)]
struct Cli {
    /// Path to the updaterd socket.
    #[arg(long, global = true, default_value = proto::DEFAULT_SOCKET)]
    socket: PathBuf,

    /// Path to the robotd socket. Used by `health` and `version`, the two commands that ask
    /// the robot about itself rather than telling the update engine to do something.
    #[arg(long, global = true, default_value = "/run/robotd.sock")]
    robot_socket: PathBuf,

    #[command(subcommand)]
    namespace: Namespace,
}

/// Only `update` exists today, plus `version`. The namespace layer is here so adding
/// `robotctl motors` later is additive rather than a restructure.
#[derive(Subcommand, Debug)]
enum Namespace {
    /// Update and release management.
    #[command(subcommand_required = true, arg_required_else_help = true)]
    Update {
        #[command(subcommand)]
        command: UpdateCommand,
    },

    /// Watch what the robot is doing, live.
    ///
    /// This is the one window into the control loop. It shows what a client asked for
    /// alongside what was actually applied and why they differ — safety clamps things
    /// constantly, and "the stick is forward and the robot is still" is unreadable without
    /// the reason next to it.
    Monitor {
        /// Frames per second. The robot decimates server-side, so asking for less genuinely
        /// costs it less.
        #[arg(long, default_value_t = 10)]
        hz: u32,

        /// One JSON object per line, for piping somewhere.
        #[arg(long)]
        json: bool,
    },

    /// The full state of this robot: hardware and software.
    ///
    /// Hardware from `robotd` — the verdict the update system's health gate turns on, the loop
    /// and bus numbers behind it, the IMU, the battery and the motor temperatures. Software
    /// from `updaterd` — what is running, what is installed, what is pinned, and how the last
    /// update went.
    ///
    /// One command because that is how the question arrives. "What is wrong with this robot"
    /// does not divide into hardware and software until after it is answered, and a robot that
    /// reverted a release an hour ago looks exactly like a robot with unpowered servos until
    /// both halves are on screen together.
    ///
    /// Exits non-zero when the robot is unhealthy or unreachable, so it can gate a script.
    /// Nothing else here affects the exit code: a flat pack, a hot motor and a pinned
    /// component are reported, not judged.
    Health {
        /// Machine-readable output, for scripts and support bundles.
        #[arg(long)]
        json: bool,
    },

    /// What is running on this robot, and what is installed. The first thing to ask for
    /// in a support report.
    ///
    /// Distinct from `--version`, which reports only this binary. This asks every daemon.
    Version {
        /// Machine-readable output, for support bundles and scripts.
        #[arg(long)]
        json: bool,
    },

    /// Print a shell completion script on stdout.
    ///
    /// Generated from this binary's own command tree, so the completions a robot offers
    /// are the commands that robot's release actually has. `install.sh` therefore drops a
    /// loader that sources this at shell start rather than a snapshot of it: the snapshot
    /// would go stale the first time an update adds a subcommand.
    ///
    ///   robotctl completions bash > /etc/bash_completion.d/robotctl
    Completions {
        /// bash, zsh, fish, elvish or powershell.
        shell: clap_complete::Shell,
    },
}

#[derive(Subcommand, Debug)]
enum UpdateCommand {
    /// Report whether an update is available. Changes nothing.
    Check {
        /// Component to check; omit for all.
        component: Option<String>,
    },

    /// Install the latest release, or an exact version.
    Apply {
        component: String,

        /// Exact version to install. Omit for whatever the source calls latest.
        /// This is the primitive that makes release testing scriptable.
        #[arg(long, conflicts_with = "git_ref")]
        version: Option<semver::Version>,

        /// Install what a branch last built, e.g. `--ref my-branch`.
        ///
        /// Resolves to the moving `daemon-dev-<ref>` tag CI publishes on every push, so the
        /// exact version — `0.2.0-dev.17.abc1234` — never has to be typed. Dev builds are
        /// signed with the team key, so a robot only accepts one if `allow_dev_keys` is on
        /// and that key is in its trusted set: a customer robot refuses them.
        ///
        /// `conflicts_with` version rather than a silent precedence: asking for both a ref
        /// and a version is a mistake worth reporting, not one to resolve by guessing.
        #[arg(long = "ref", value_name = "REF", conflicts_with = "version")]
        git_ref: Option<String>,

        /// Verify everything, then stop before the symlink swap.
        #[arg(long)]
        dry_run: bool,

        /// Proceed even if a telepresence session is active. Never bypasses
        /// signature, hash, or compatibility checks.
        #[arg(long)]
        interrupt_sessions: bool,
    },

    /// Return to the previously installed release.
    Rollback { component: String },

    /// Return to the never-pruned known-good release.
    ResetToGolden { component: String },

    /// Activate an already-installed release without downloading.
    ///
    /// For `model` this switches library bundles; for `daemon` it is a targeted
    /// revert.
    Select {
        component: String,
        version: semver::Version,
    },

    /// Refuse versions other than this one. Omit the version to unpin.
    Pin {
        component: String,
        version: Option<semver::Version>,
    },

    /// Per-component state.
    Status(StatusArgs),

    /// Recent update attempts and outcomes.
    Log {
        #[arg(short = 'n', long, default_value_t = 20)]
        limit: usize,
    },

    /// Follow progress until interrupted.
    Watch,
}

#[derive(Args, Debug)]
struct StatusArgs {
    /// Emit JSON instead of a table.
    #[arg(long)]
    json: bool,
}

/// A blocking JSON-RPC connection to `updaterd`.
///
/// Deliberately `std::os::unix::net`, not tokio: this is a short-lived CLI issuing
/// one request. An async runtime would add a dependency and a concept for nothing.
struct Client {
    reader: BufReader<UnixStream>,
    writer: UnixStream,
    next_id: u64,
}

impl Client {
    /// Connect to `updaterd`. Names the service in its error, which is why
    /// [`Self::connect_to`] exists for anything else.
    fn connect(path: &std::path::Path) -> Result<Self, Failure> {
        Self::connect_to("updaterd", path)
    }

    /// As [`Self::connect`], but for another daemon on another socket.
    ///
    /// The service name is a parameter because the failure message names it and suggests a
    /// `systemctl status` for it. Hardcoding "updaterd" told anyone diagnosing a stopped
    /// `robotd` to go check the wrong service — a diagnostic that points at the wrong
    /// place is worse than none.
    fn connect_to(service: &str, path: &std::path::Path) -> Result<Self, Failure> {
        let stream = UnixStream::connect(path).map_err(|e| {
            Failure::new(
                exit::UNREACHABLE,
                format!(
                    "cannot reach {service} at {}: {e}\n\
                     Is the service running?  systemctl status {service}",
                    path.display()
                ),
            )
        })?;
        let writer = stream
            .try_clone()
            .map_err(|e| Failure::new(exit::FAILED, format!("could not split the socket: {e}")))?;
        Ok(Self {
            reader: BufReader::new(stream),
            writer,
            next_id: 1,
        })
    }

    /// Write one request. Used by [`Self::call`] and by `watch`, which reads replies
    /// itself rather than waiting for a terminal response.
    fn send(&mut self, request: &proto::Request) -> Result<(), Failure> {
        let mut line = serde_json::to_vec(request)
            .map_err(|e| Failure::new(exit::FAILED, format!("could not encode request: {e}")))?;
        line.push(b'\n');
        self.writer
            .write_all(&line)
            .and_then(|()| self.writer.flush())
            .map_err(|e| Failure::new(exit::UNREACHABLE, format!("could not send request: {e}")))
    }

    /// Send a call and return its terminal response.
    ///
    /// Progress notifications arrive interleaved and carry no `id`; they go to stderr
    /// so stdout stays pipeable. Anything with a non-matching id is ignored rather
    /// than treated as an error.
    fn call(&mut self, call: &proto::Call) -> Result<proto::Response, Failure> {
        let id = proto::Id::Number(self.next_id);
        self.next_id += 1;
        self.send(&proto::Request::call(id.clone(), call))?;

        loop {
            let mut buf = String::new();
            let read = self
                .reader
                .read_line(&mut buf)
                .map_err(|e| Failure::new(exit::UNREACHABLE, format!("connection lost: {e}")))?;
            if read == 0 {
                return Err(Failure::new(
                    exit::UNREACHABLE,
                    "updaterd closed the connection".into(),
                ));
            }
            let trimmed = buf.trim();
            if trimmed.is_empty() {
                continue;
            }

            // Notifications first: no id, so they can't be confused with a response.
            if let Ok(note) = serde_json::from_str::<proto::Request>(trimmed)
                && note.is_notification()
            {
                if let Ok(progress) = note.as_progress() {
                    report_progress(&progress);
                }
                continue;
            }

            let response: proto::Response = serde_json::from_str(trimmed).map_err(|e| {
                Failure::new(exit::FAILED, format!("malformed response: {e}: {trimmed}"))
            })?;
            if response.id.as_ref() == Some(&id) {
                return Ok(response);
            }
            // Someone else's reply on a shared connection; ignore.
        }
    }

    /// Refuse a protocol mismatch loudly rather than sending requests the daemon
    /// might misread. A stale `robotctl` in someone's shell is normal.
    fn hello(&mut self) -> Result<(), Failure> {
        self.hello_result().map(|_| ())
    }

    /// As [`Self::hello`], but returns what the daemon said about itself.
    ///
    /// `version` needs the payload rather than a pass/fail, and needs it even when the
    /// daemon speaks a different API version — "these two are out of step" is the single
    /// most useful thing the command can report, so refusing to print it would defeat the
    /// purpose. The protocol check still fails for every *other* command.
    fn hello_result(&mut self) -> Result<proto::HelloResult, Failure> {
        let response = self.call(&proto::Call::Hello(proto::HelloParams {
            api_version: proto::API_VERSION,
        }))?;
        if let Some(error) = response.error {
            return Err(Failure::new(
                exit::FAILED,
                format!(
                    "{}\nrobotctl and updaterd are out of step; install matching versions.",
                    error.message
                ),
            ));
        }
        response
            .result
            .and_then(|r| serde_json::from_value(r).ok())
            .ok_or_else(|| {
                Failure::new(
                    exit::FAILED,
                    "daemon answered hello in an unexpected shape".to_owned(),
                )
            })
    }
}

// ── version reporting ────────────────────────────────────────────────────────

/// What one daemon reports about itself, or why it could not be asked.
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize)]
struct ServiceReport {
    name: &'static str,
    /// `None` when the daemon could not be asked — a normal state for `robotd`, and the
    /// most important thing to report for `updaterd`.
    version: Option<String>,
    revision: Option<String>,
    /// Why the daemon could not be asked: not running, or answering something we cannot
    /// read (an API-version disagreement, say).
    ///
    /// Not called `unreachable`, because a daemon that is running and speaking a protocol
    /// this `robotctl` does not understand is very much reachable — and reporting that as
    /// "unreachable" would send support looking for a stopped service.
    #[serde(skip_serializing_if = "Option::is_none")]
    error: Option<String>,
}

impl ServiceReport {
    fn failed(name: &'static str, why: String) -> Self {
        Self {
            name,
            version: None,
            revision: None,
            error: Some(why),
        }
    }
}

/// A component's installed release, as opposed to what is *running*.
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize)]
struct ComponentReport {
    name: String,
    installed: Option<String>,
    revision: Option<String>,
    /// Set when this component refuses anything but one version. Worth surfacing without
    /// being asked: a pinned component silently ignores every release published after it,
    /// and the symptom — "updates stopped arriving" — points nowhere near the cause.
    #[serde(skip_serializing_if = "Option::is_none")]
    pinned: Option<String>,
    /// The last update attempt, as one line. `None` on a robot that has never updated.
    #[serde(skip_serializing_if = "Option::is_none")]
    last_attempt: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize)]
struct VersionReport {
    robotctl: String,
    robotctl_revision: Option<String>,
    services: Vec<ServiceReport>,
    components: Vec<ComponentReport>,
    /// Human-readable warnings: running/installed disagreements, unreachable daemons.
    warnings: Vec<String>,
}

// ── health reporting ─────────────────────────────────────────────────────────

/// The whole state of one robot: what the hardware is doing, and what software is on it.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
struct HealthReport {
    /// `robotd`'s answer. `None` when it could not be asked.
    #[serde(skip_serializing_if = "Option::is_none")]
    robot: Option<proto::HealthResult>,
    /// Why `robotd` could not be asked. Reported rather than fatal: a stopped `robotd` is
    /// itself the most useful sentence this command can print, and the software half is still
    /// worth having — that is often what explains the stopped daemon.
    #[serde(skip_serializing_if = "Option::is_none")]
    robot_error: Option<String>,
    software: VersionReport,
}

impl HealthReport {
    /// Is the robot working? `None` when `robotd` could not be asked.
    fn healthy(&self) -> Option<bool> {
        self.robot.as_ref().map(|r| r.healthy)
    }
}

/// Report the full state of the robot: control loop, bus, IMU, battery, motor temperature,
/// and the software running and installed.
///
/// Stream `robot.state` until interrupted.
///
/// Reads notifications rather than waiting for a terminal response, so it never "finishes"
/// — Ctrl-C is the exit. A closed socket ends it too, which is what happens when `robotd`
/// restarts during an update, and is worth seeing rather than hanging through.
fn run_monitor(robot_socket: &Path, hz: u32, json: bool) -> Result<(), Failure> {
    let mut client = Client::connect_to("robotd", robot_socket)?;
    let call = proto::Call::RobotSubscribe(proto::SubscribeParams {
        hz: (hz > 0).then_some(hz),
    });
    client.send(&proto::Request::call(proto::Id::Number(1), &call))?;

    let mut line = String::new();
    loop {
        line.clear();
        let read = client
            .reader
            .read_line(&mut line)
            .map_err(|e| Failure::new(exit::UNREACHABLE, format!("stream ended: {e}")))?;
        if read == 0 {
            // robotd went away — a restart mid-update looks exactly like this, so say so
            // rather than exiting silently as though the user had asked to stop.
            return Err(Failure::new(
                exit::UNREACHABLE,
                "robotd closed the connection".to_owned(),
            ));
        }

        let Ok(request) = serde_json::from_str::<proto::Request>(&line) else {
            continue;
        };
        let Some(state) = request.as_state() else {
            // The subscribe acknowledgement, or anything else this client does not model.
            continue;
        };

        if json {
            println!("{}", line.trim_end());
            continue;
        }

        let limits = if state.movement.limited_by.is_empty() {
            String::new()
        } else {
            format!("  [{}]", state.movement.limited_by.join(","))
        };
        // Gravity and gain sit next to the fall verdict on purpose: `fallen` is derived from
        // the first and overrides the second, and reading the verdict without its input made
        // "the robot is down" indistinguishable from "the IMU frame is wrong".
        println!(
            "{:8.2}  {:>5}  {:5.1}Hz miss={:<4} {}  g[{:+.2} {:+.2} {:+.2}] kp={:<4} \
             req[{:+.2} {:+.2} {:+.2}] app[{:+.2} {:+.2} {:+.2}]{}",
            state.t,
            state.policy,
            state.control_loop.hz,
            state.control_loop.missed,
            if state.safety.fallen {
                "FALLEN"
            } else {
                "ok    "
            },
            state.safety.gravity[0],
            state.safety.gravity[1],
            state.safety.gravity[2],
            state
                .safety
                .gain
                .map_or_else(|| "-".to_owned(), |g| g.to_string()),
            state.movement.requested[0],
            state.movement.requested[1],
            state.movement.requested[2],
            state.movement.applied[0],
            state.movement.applied[1],
            state.movement.applied[2],
            limits,
        );
    }
}

/// Ask `robotd` whether it is healthy.
/// Deliberately does **not** use the ordinary `Client::connect(..)?` + `hello()?` path.
/// That exits non-zero when `updaterd` is unreachable, which is precisely the situation
/// where someone is running this command. Every failure here becomes a line in the report
/// instead.
///
/// Both halves, in one command, because the question "what is wrong with this robot" does not
/// divide along that line — a robot that reverted a release an hour ago and a robot whose
/// servos are unpowered look identical until you can see both at once. `robotctl version`
/// remains the software half on its own, for when that is all that is wanted.
///
/// Exits non-zero when the robot is unhealthy or unreachable, so a script can gate on it —
/// `robotctl health && do_the_thing`, which `install.sh` relies on. Nothing else here affects
/// the exit code: a flat pack, a hot motor and a pinned component are all *reported*, and a
/// command that failed because of a low battery would be a command nobody could script.
fn run_health(socket: &Path, robot_socket: &Path, json: bool) -> Result<(), Failure> {
    let mut report = HealthReport {
        robot: None,
        robot_error: None,
        software: collect_version_report(socket, robot_socket),
    };

    match Client::connect_to("robotd", robot_socket) {
        Err(failure) => report.robot_error = Some(failure.message),
        Ok(mut client) => match client.call(&proto::Call::RobotHealth) {
            Err(failure) => report.robot_error = Some(failure.message),
            Ok(response) => match response.result_as::<proto::HealthResult>() {
                Ok(health) => report.robot = Some(health),
                Err(e) => {
                    report.robot_error =
                        Some(format!("robotd answered robot.health unreadably: {e}"));
                }
            },
        },
    }

    if json {
        println!(
            "{}",
            serde_json::to_string_pretty(&report).unwrap_or_else(|_| "{}".to_owned())
        );
    } else {
        print!("{}", render_health(&report));
    }

    match report.healthy() {
        Some(true) => Ok(()),
        // REFUSED, not FAILED: the robot answered correctly and the answer was "no". That is a
        // verdict, not a malfunction, and a script should be able to tell them apart.
        Some(false) => Err(Failure::silent(exit::REFUSED)),
        // Nothing answered. Distinct again: there is no verdict to act on.
        None => Err(Failure::silent(exit::UNREACHABLE)),
    }
}

/// The report as a human reads it: the verdict first, then the evidence, then the software.
///
/// Pure, so the cases that matter are testable without a robot — and the cases that matter are
/// the missing ones, which a live test on a working robot never produces.
fn render_health(report: &HealthReport) -> String {
    use std::fmt::Write;
    let mut out = String::new();

    match (&report.robot, &report.robot_error) {
        (Some(health), _) => {
            let verdict = match (health.healthy, health.degraded) {
                (true, _) => "healthy".to_owned(),
                // "degraded" reads as what it is: this release is fine, this board cannot move.
                (false, true) => format!(
                    "degraded: {}",
                    health.reason.as_deref().unwrap_or("no reason given")
                ),
                (false, false) => format!(
                    "unhealthy: {}",
                    health.reason.as_deref().unwrap_or("no reason given")
                ),
            };
            let _ = writeln!(out, "robot     {verdict}");

            if let Some(l) = &health.control_loop {
                let _ = writeln!(
                    out,
                    "  {:<9} {} of {:.1} Hz · {} ticks · {} missed · last {} ms ago",
                    "loop",
                    // Unknown is not 0 Hz: for the first second there is no measurement, and
                    // printing one would describe a healthy robot as stopped.
                    match l.achieved_hz {
                        Some(hz) => format!("{hz:.1}"),
                        None => "not measured yet,".to_owned(),
                    },
                    l.target_hz,
                    l.ticks,
                    l.missed,
                    l.last_tick_age_ms
                );
            }

            let bus = match (health.bus.consecutive_errors, health.bus.startup_failures) {
                (0, 0) => "ok".to_owned(),
                (0, n) => format!("waiting for a robot to answer, {n} attempts"),
                (n, _) => format!("{n} consecutive read failures"),
            };
            let _ = writeln!(out, "  {:<9} {bus}", "bus");

            if let Some(imu) = &health.imu {
                let _ = writeln!(
                    out,
                    "  {:<9} {}{}",
                    "imu",
                    if imu.ready { "ready" } else { "not ready" },
                    match imu.stale_blocks {
                        0 => String::new(),
                        // Worth shouting about: the board answers, so nothing else reports a
                        // fault, while the orientation being fed to the policy is frozen.
                        n => format!(", {n} stale reads — orientation may be dead"),
                    }
                );
            }

            // Silent when unknown rather than printing a zero: for the first second of uptime,
            // and on a robot whose bus cannot answer, there is genuinely no reading, and
            // "0.00 V" would read as a dead pack.
            if let Some(b) = &health.battery {
                let _ = writeln!(
                    out,
                    "  {:<9} {:.2} V ({:.0}%)",
                    "battery", b.volts, b.percent
                );
            }
            if let Some(m) = &health.motors {
                let _ = writeln!(
                    out,
                    "  {:<9} {:.0} °C max ({}) · {:.0} °C mean",
                    "motors", m.max_c, m.hottest, m.mean_c
                );
            }
            // Its own line, next to the motors rather than merged with them: hot servos and a
            // hot board are different faults with different fixes, and a reader scanning for
            // "what is too hot here" needs to see which.
            if let Some(cpu) = health.cpu_temp_c {
                let _ = writeln!(out, "  {:<9} {cpu:.0} °C", "cpu");
            }
        }
        (None, Some(why)) => {
            // First line only: a multi-line message would break the column layout, and the
            // rest of it is the `systemctl status` hint the connect error already carries.
            let brief = why.lines().next().unwrap_or("unavailable");
            let _ = writeln!(out, "robot     unavailable — {brief}");
        }
        (None, None) => {
            let _ = writeln!(out, "robot     unavailable");
        }
    }

    let _ = writeln!(out, "\nsoftware");
    for service in &report.software.services {
        match &service.error {
            Some(why) => {
                let brief = why.lines().next().unwrap_or("unavailable");
                let _ = writeln!(out, "  {:<9} unavailable — {brief}", service.name);
            }
            None => {
                let _ = writeln!(
                    out,
                    "  {:<9} {} {}",
                    service.name,
                    service.version.as_deref().unwrap_or("unknown"),
                    match &service.revision {
                        Some(rev) => format!("(rev {})", short_revision(rev)),
                        None => "(rev unknown)".to_owned(),
                    }
                );
            }
        }
    }
    for component in &report.software.components {
        let _ = writeln!(
            out,
            "  {:<9} {} installed{}",
            component.name,
            component.installed.as_deref().unwrap_or("none"),
            match &component.pinned {
                Some(v) => format!(", pinned to {v}"),
                None => String::new(),
            }
        );
        if let Some(attempt) = &component.last_attempt {
            let _ = writeln!(out, "  {:<9} last update {attempt}", "");
        }
    }

    // Same shape `robotctl version` uses, blank line and all: these are often multi-line —
    // the `systemctl status` hint on an unreachable daemon is the useful half — and the two
    // commands should not disagree about how a warning looks.
    for warning in &report.software.warnings {
        let _ = writeln!(out, "\n! {warning}");
    }

    out
}

fn run_version(socket: &Path, robot_socket: &Path, json: bool) -> Result<(), Failure> {
    let report = collect_version_report(socket, robot_socket);

    if json {
        println!(
            "{}",
            serde_json::to_string_pretty(&report).unwrap_or_default()
        );
    } else {
        print!("{}", render_version(&report));
    }
    Ok(())
}

/// Ask both daemons what they are running and `updaterd` what is installed.
///
/// Shared by `version` and `health` so the software half of a support report is gathered one
/// way. Two commands assembling it separately is how they start disagreeing.
fn collect_version_report(socket: &Path, robot_socket: &Path) -> VersionReport {
    let build = proto::build_info!();
    let mut report = VersionReport {
        robotctl: build.version.to_owned(),
        robotctl_revision: build.revision.map(str::to_owned),
        services: Vec::new(),
        components: Vec::new(),
        warnings: Vec::new(),
    };

    // updaterd: running build, then what it says is installed.
    let mut updaterd_running: Option<semver::Version> = None;
    match Client::connect(socket) {
        Err(failure) => report
            .services
            .push(ServiceReport::failed("updaterd", failure.message)),
        Ok(mut client) => {
            let hello = client.hello_result();
            match hello {
                Ok(hello) => {
                    updaterd_running = hello.daemon_version.clone();
                    report.services.push(ServiceReport {
                        name: "updaterd",
                        version: hello.daemon_version.map(|v| v.to_string()),
                        revision: hello.revision,
                        error: None,
                    });
                }
                Err(failure) => report
                    .services
                    .push(ServiceReport::failed("updaterd", failure.message)),
            }
            report.components = installed_components(&mut client);
        }
    }

    // robotd, over its own socket. Unreachable is routine — it may be stopped, or this may
    // be a robot where it has not been installed yet — so it is reported, not an error.
    match Client::connect_to("robotd", robot_socket) {
        Err(failure) => report
            .services
            .push(ServiceReport::failed("robotd", failure.message)),
        Ok(mut client) => match client.hello_result() {
            Ok(hello) => report.services.push(ServiceReport {
                name: "robotd",
                version: hello.daemon_version.map(|v| v.to_string()),
                revision: hello.revision,
                error: None,
            }),
            Err(failure) => report
                .services
                .push(ServiceReport::failed("robotd", failure.message)),
        },
    }

    report.warnings = version_warnings(&report, updaterd_running.as_ref());
    report
}

/// Installed release per component, with the revision of the active one.
///
/// Two calls per component rather than one: `status` knows the active version, and
/// `listInstalled` knows the revision it was built from. Revision matters for support —
/// once branch installs land, several builds share a version — so it is worth the extra
/// round trip in a diagnostic command.
fn installed_components(client: &mut Client) -> Vec<ComponentReport> {
    let Ok(response) = client.call(&proto::Call::Status) else {
        return Vec::new();
    };
    let Ok(statuses) = response.result_as::<Vec<proto::ComponentStatus>>() else {
        return Vec::new();
    };

    statuses
        .into_iter()
        .map(|status| {
            let revision = client
                .call(&proto::Call::ListInstalled(proto::ComponentParams {
                    component: status.component.clone(),
                }))
                .ok()
                .and_then(|r| r.result_as::<Vec<proto::InstalledRelease>>().ok())
                .and_then(|releases| {
                    releases
                        .into_iter()
                        .find(|release| release.active)?
                        .source_revision
                });
            ComponentReport {
                name: status.component.to_string(),
                installed: status.installed.map(|v| v.to_string()),
                revision,
                pinned: status.pinned.map(|v| v.to_string()),
                last_attempt: status.last_attempt.as_ref().map(describe_attempt),
            }
        })
        .collect()
}

/// One update attempt in one line: what it tried, and how it went.
///
/// The outcome's *reason* is kept for the failures, not trimmed to "rolled back" — it is the
/// only place the cause of an automatic revert appears outside the journal, and a robot that
/// reverted a week ago is exactly the robot someone is asking about.
fn describe_attempt(entry: &proto::LogEntry) -> String {
    let target = match (&entry.from, &entry.to) {
        (Some(from), Some(to)) => format!("{from} → {to}"),
        (None, Some(to)) => to.to_string(),
        (Some(from), None) => format!("from {from}"),
        (None, None) => "unknown version".to_owned(),
    };
    match &entry.outcome {
        proto::Outcome::Success => format!("{target}: applied"),
        proto::Outcome::RolledBack { reason } => format!("{target}: ROLLED BACK — {reason}"),
        proto::Outcome::Aborted { reason } => format!("{target}: refused — {reason}"),
    }
}

/// Disagreements worth telling a human about.
///
/// Pure, so the interesting cases are unit-testable without daemons: the running/installed
/// mismatch is the one support will actually hit, and it must be explained rather than
/// merely flagged — it is *expected* right after an update and alarming only if it
/// survives a reboot.
/// Is a running process from a different build than the installed release?
///
/// **Revisions decide it when both are known**, and versions only stand in when one is not.
/// That is not a refinement, it is the difference between right and wrong on a dev build: a
/// binary reports `CARGO_PKG_VERSION`, while the release it was packaged into is versioned
/// `0.1.4-dev.91.7f685a0` — the prerelease suffix is minted by `xtask package` at package
/// time, from a run number and a SHA the compiler never saw. So a dev-channel `robotd` reports
/// `0.1.4` against an installed `0.1.4-dev.91.7f685a0` *while being exactly that build*, and
/// comparing versions accused every single dev install of having failed its restart — the
/// louder of the two warnings, and always false.
///
/// A prefix match counts as equal so a short SHA and a full one agree; `dev.yml` passes
/// `GITHUB_SHA` in full and `DUCK_REVISION` is likewise full, but a hand-built release with a
/// `--short` revision must not read as a mismatch. Seven characters minimum, because a prefix
/// rule with no floor would make an empty string match everything.
fn is_behind(
    running_version: &semver::Version,
    running_revision: Option<&str>,
    installed_version: &semver::Version,
    installed_revision: Option<&str>,
) -> bool {
    match (running_revision, installed_revision) {
        (Some(running), Some(installed)) => !same_revision(running, installed),
        _ => running_version != installed_version,
    }
}

/// A revision as a human wants to read it: seven characters, the abbreviation git itself uses
/// and the one `xtask` embeds in a dev version. `DUCK_REVISION` carries the full 40, which in a
/// column of output is noise around the seven characters anyone actually compares.
///
/// `get` rather than slicing: it returns `None` on a non-boundary rather than panicking, and a
/// revision from a config file is not guaranteed to be a hex string.
fn short_revision(revision: &str) -> &str {
    revision.get(..7).unwrap_or(revision)
}

/// Two git revisions naming the same commit, allowing one to be an abbreviation of the other.
fn same_revision(a: &str, b: &str) -> bool {
    const MIN_ABBREV: usize = 7;
    let shortest = a.len().min(b.len());
    shortest >= MIN_ABBREV && a[..shortest] == b[..shortest]
}

fn version_warnings(
    report: &VersionReport,
    updaterd_running: Option<&semver::Version>,
) -> Vec<String> {
    let mut warnings = Vec::new();

    let daemon = report.components.iter().find(|c| c.name == "daemon");
    let daemon_installed = daemon
        .and_then(|c| c.installed.as_deref())
        .and_then(|v| semver::Version::parse(v).ok());
    let daemon_revision = daemon.and_then(|c| c.revision.as_deref());

    let updaterd_revision = report
        .services
        .iter()
        .find(|s| s.name == "updaterd")
        .and_then(|s| s.revision.as_deref());

    // Name the revision alongside the version wherever it is known, because on the dev channel
    // the version alone cannot show a difference: both sides read `0.1.4` and the SHA is the
    // whole story.
    let identify = |version: &semver::Version, revision: Option<&str>| match revision {
        Some(rev) => format!("{version} (rev {})", short_revision(rev)),
        None => version.to_string(),
    };

    if let (Some(running), Some(installed)) = (updaterd_running, daemon_installed.as_ref())
        && is_behind(running, updaterd_revision, installed, daemon_revision)
    {
        warnings.push(format!(
            "updaterd is running {} but the installed daemon release is {}.\n  \
             Expected right after an update — updaterd never restarts itself, so it keeps\n  \
             running the old binary until the next reboot. If this survives a reboot, the\n  \
             new release is not being launched: check the `current` symlink and the unit's\n  \
             ExecStart path.",
            identify(running, updaterd_revision),
            identify(installed, daemon_revision)
        ));
    }

    // robotd is in `on_apply`'s restart set, so unlike updaterd it *should* already be on
    // the installed release. A mismatch here means the restart did not take effect, which
    // is a different and more serious situation than updaterd's expected lag.
    let robotd = report.services.iter().find(|s| s.name == "robotd");
    let robotd_running = robotd
        .and_then(|s| s.version.as_deref())
        .and_then(|v| semver::Version::parse(v).ok());
    let robotd_revision = robotd.and_then(|s| s.revision.as_deref());
    if let (Some(running), Some(installed)) = (robotd_running.as_ref(), daemon_installed.as_ref())
        && is_behind(running, robotd_revision, installed, daemon_revision)
    {
        warnings.push(format!(
            "robotd is running {} but the installed daemon release is {}.\n  \
             robotd is in on_apply's restart set, so it should already be on the installed\n  \
             release: either the restart did not happen, or it failed and systemd restarted\n  \
             the old binary. Check `systemctl status robotd` and the update log.",
            identify(running, robotd_revision),
            identify(installed, daemon_revision)
        ));
    }

    for service in &report.services {
        if let Some(why) = &service.error {
            warnings.push(format!(
                "{} could not be asked what it is running: {why}",
                service.name
            ));
        }
    }

    warnings
}

/// Human-readable report. Kept separate from gathering so it is testable.
fn render_version(report: &VersionReport) -> String {
    use std::fmt::Write;
    let mut out = String::new();

    let rev = |r: &Option<String>| match r {
        Some(rev) => format!("rev {}", short_revision(rev)),
        None => "rev unknown".to_owned(),
    };

    let _ = writeln!(
        out,
        "robotctl   {}  {}",
        report.robotctl,
        rev(&report.robotctl_revision)
    );

    let _ = writeln!(out, "\nrunning");
    for service in &report.services {
        match &service.error {
            Some(why) => {
                // First line only: the full text goes in the warnings block, and a
                // multi-line message here would break the column layout.
                let brief = why.lines().next().unwrap_or("unavailable");
                let _ = writeln!(out, "  {:<10} unavailable — {brief}", service.name);
            }
            None => {
                let _ = writeln!(
                    out,
                    "  {:<10} {:<8} {}",
                    service.name,
                    service.version.as_deref().unwrap_or("unknown"),
                    rev(&service.revision)
                );
            }
        }
    }

    if !report.components.is_empty() {
        let _ = writeln!(out, "\ninstalled");
        for component in &report.components {
            let _ = writeln!(
                out,
                "  {:<12} {:<8} {}",
                component.name,
                component.installed.as_deref().unwrap_or("none"),
                rev(&component.revision)
            );
        }
    }

    for warning in &report.warnings {
        let _ = writeln!(out, "\n! {warning}");
    }

    out
}

/// An error carrying the exit code it should produce.
struct Failure {
    code: u8,
    message: String,
}

impl Failure {
    fn new(code: u8, message: String) -> Self {
        Self { code, message }
    }

    /// An exit code with nothing to say.
    ///
    /// For a command that has already printed its own answer on stdout and only needs the
    /// status to be non-zero — `robotctl health` on an unhealthy robot has reported the
    /// reason already, and repeating it as `error: ...` on stderr would read as though
    /// something had gone wrong with the command rather than with the robot.
    fn silent(code: u8) -> Self {
        Self {
            code,
            message: String::new(),
        }
    }

    /// Map a daemon error code to a CLI exit code, preserving the distinctions that
    /// let scripts branch: retry on BUSY, "correctly rejected" on REFUSED.
    fn from_rpc(error: proto::Error) -> Self {
        use proto::code;
        let exit = match error.code {
            code::BUSY => exit::BUSY,
            code::INCOMPATIBLE
            | code::PREFLIGHT_FAILED
            | code::VERIFICATION_FAILED
            | code::WOULD_DOWNGRADE
            | code::NOT_INSTALLED
            | code::ARCHIVE_TOO_LARGE => exit::REFUSED,
            code::PERMISSION_DENIED => exit::DENIED,
            code::PROTOCOL_MISMATCH => exit::USAGE,
            _ => exit::FAILED,
        };
        Self::new(exit, error.message)
    }
}

/// Progress goes to stderr so `--json` output on stdout stays pipeable.
///
/// The engine emits progress once per network chunk — around 250 notifications for a 3.6 MB
/// artifact — and printing a line for each buried the phases that actually matter in a
/// screenful of `Downloading N%`. On a terminal this now rewrites a single line; when
/// redirected, where `\r` is useless, it prints one line per decile instead.
fn report_progress(progress: &proto::Progress) {
    use std::io::{IsTerminal, Write};

    // `Some` also means "a bare `\r` line is open and owes a newline".
    static LAST: std::sync::Mutex<Option<(proto::Phase, u8)>> = std::sync::Mutex::new(None);

    let mut last = LAST.lock().unwrap_or_else(|e| e.into_inner());
    let tty = std::io::stderr().is_terminal();

    let Some(percent) = progress.percent else {
        // A phase with no percentage. Close any open counter line first, or it gets
        // overwritten and the download appears to stop partway.
        if tty && last.is_some() {
            eprintln!();
        }
        *last = None;
        eprintln!("  {:?}", progress.phase);
        return;
    };

    if tty {
        eprint!("\r  {:?} {percent}%", progress.phase);
        if percent >= 100 {
            eprintln!();
            *last = None;
        } else {
            *last = Some((progress.phase, 0));
        }
        let _ = std::io::stderr().flush();
        return;
    }

    // 100 in its own bucket, so a finished download says so rather than stopping at 90.
    let decile = if percent >= 100 { 10 } else { percent / 10 };
    if *last != Some((progress.phase, decile)) {
        *last = Some((progress.phase, decile));
        eprintln!("  {:?} {percent}%", progress.phase);
    }
}

/// Restore default `SIGPIPE` handling.
///
/// Rust ignores `SIGPIPE` at startup, so writing to a closed stdout returns `EPIPE`
/// and `println!` **panics** — meaning `robotctl update log | head` dies with a
/// backtrace instead of exiting quietly like every other unix tool. Resetting it makes
/// the process terminate the way `ls | head` does.
///
/// Found by the board test, which pipes output through `head`.
fn restore_sigpipe() {
    // Safety: setting a signal disposition to the default is always valid, and this
    // runs before any threads exist.
    unsafe {
        libc::signal(libc::SIGPIPE, libc::SIG_DFL);
    }
}

fn main() -> ExitCode {
    restore_sigpipe();
    let cli = Cli::parse();
    match run(cli) {
        Ok(()) => ExitCode::from(exit::OK),
        Err(failure) => {
            if !failure.message.is_empty() {
                eprintln!("error: {}", failure.message);
            }
            ExitCode::from(failure.code)
        }
    }
}

fn run(cli: Cli) -> Result<(), Failure> {
    let command = match cli.namespace {
        Namespace::Health { json } => {
            return run_health(&cli.socket, &cli.robot_socket, json);
        }
        Namespace::Version { json } => {
            return run_version(&cli.socket, &cli.robot_socket, json);
        }
        Namespace::Monitor { hz, json } => {
            return run_monitor(&cli.robot_socket, hz, json);
        }
        // Pure codegen: no socket, no daemon, no root. It must keep working on a robot
        // where nothing is running, since that is where an operator most wants to type
        // less.
        Namespace::Completions { shell } => {
            clap_complete::generate(
                shell,
                &mut Cli::command(),
                "robotctl",
                &mut std::io::stdout(),
            );
            return Ok(());
        }
        Namespace::Update { command } => command,
    };

    let mut client = Client::connect(&cli.socket)?;
    client.hello()?;

    let component = |name: &str| proto::ComponentParams {
        component: proto::ComponentId::new(name),
    };
    let call = match &command {
        UpdateCommand::Check { component: name } => {
            proto::Call::Check(component(name.as_deref().unwrap_or("daemon")))
        }
        UpdateCommand::Apply {
            component: name,
            version,
            git_ref,
            dry_run,
            interrupt_sessions,
        } => proto::Call::Apply(proto::ApplyParams {
            component: proto::ComponentId::new(name),
            // clap enforces that version and git_ref are mutually exclusive, so the order
            // here cannot silently prefer one over the other.
            target: match (version, git_ref) {
                (Some(version), _) => proto::Target::Exact(version.clone()),
                (None, Some(git_ref)) => proto::Target::Ref(git_ref.clone()),
                (None, None) => proto::Target::Latest,
            },
            options: proto::ApplyOptions {
                dry_run: *dry_run,
                interrupt_sessions: *interrupt_sessions,
            },
        }),
        UpdateCommand::Rollback { component: name } => proto::Call::Rollback(component(name)),
        UpdateCommand::ResetToGolden { component: name } => {
            proto::Call::ResetToGolden(component(name))
        }
        UpdateCommand::Select {
            component: name,
            version,
        } => proto::Call::Select(proto::SelectParams {
            component: proto::ComponentId::new(name),
            version: version.clone(),
        }),
        UpdateCommand::Pin {
            component: name,
            version,
        } => proto::Call::Pin(proto::PinParams {
            component: proto::ComponentId::new(name),
            version: version.clone(),
        }),
        UpdateCommand::Status(_) => proto::Call::Status,
        UpdateCommand::Log { limit } => proto::Call::Log(proto::LogParams { limit: *limit }),
        // Streams until interrupted, so it never reaches the single-response path below.
        UpdateCommand::Watch => return watch(&mut client),
    };

    let response = client.call(&call)?;
    if let Some(error) = response.error {
        return Err(Failure::from_rpc(error));
    }
    print_result(&command, response.result.unwrap_or(serde_json::Value::Null));
    Ok(())
}

/// `watch` never returns normally: it streams until interrupted.
fn watch(client: &mut Client) -> Result<(), Failure> {
    let request = proto::Request::call(proto::Id::Number(999), &proto::Call::Subscribe);
    client.send(&request)?;

    loop {
        let mut buf = String::new();
        if client.reader.read_line(&mut buf).unwrap_or(0) == 0 {
            return Ok(());
        }
        if let Ok(note) = serde_json::from_str::<proto::Request>(buf.trim())
            && let Ok(progress) = note.as_progress()
        {
            println!(
                "{} {:?} {:?}",
                progress.component, progress.phase, progress.percent
            );
        }
    }
}

/// Human-readable rendering. `status --json` and anything unrecognised print raw
/// JSON, so scripts always have a machine-readable path.
fn print_result(command: &UpdateCommand, result: serde_json::Value) {
    let json = |value: &serde_json::Value| {
        println!(
            "{}",
            serde_json::to_string_pretty(value).unwrap_or_default()
        );
    };

    match command {
        UpdateCommand::Status(args) if args.json => json(&result),
        // Typed, so a renamed field is a compile error rather than a column of "?".
        // Anything that will not parse falls back to raw JSON: a diagnostic command must
        // print what it got rather than nothing.
        UpdateCommand::Status(_) => {
            match serde_json::from_value::<Vec<proto::ComponentStatus>>(result.clone()) {
                Err(_) => json(&result),
                Ok(statuses) => {
                    for status in statuses {
                        let installed = match &status.installed {
                            Some(version) => version.to_string(),
                            None => "none".to_owned(),
                        };
                        let healthy = match status.healthy {
                            Some(true) => "healthy",
                            Some(false) => "UNHEALTHY",
                            None => "no probe",
                        };
                        println!("{}: {installed} ({healthy})", status.component);
                        if let Some(pinned) = &status.pinned {
                            println!("  pinned to {pinned}");
                        }
                        if let Some(last) = &status.last_attempt {
                            println!("  last attempt: {}", compact(last));
                        }
                    }
                }
            }
        }
        UpdateCommand::Log { .. } => {
            match serde_json::from_value::<Vec<proto::LogEntry>>(result.clone()) {
                Err(_) => json(&result),
                Ok(entries) => {
                    for entry in entries {
                        println!("{}", compact(&entry));
                    }
                }
            }
        }
        _ => json(&result),
    }
}

fn compact(value: &impl serde::Serialize) -> String {
    serde_json::to_string(value).unwrap_or_default()
}
#[cfg(test)]
mod tests {
    use super::*;
    use clap::CommandFactory;

    /// clap's own invariant check — catches conflicting flags/arg definitions at
    /// test time rather than on first run.
    #[test]
    fn cli_definition_is_valid() {
        Cli::command().debug_assert();
    }

    /// `completions` must name a shell rather than defaulting to one: a script that
    /// redirects the output into a file for the wrong shell would produce a file that is
    /// silently never used.
    #[test]
    fn completions_requires_a_shell() {
        assert!(
            Cli::try_parse_from(["robotctl", "completions"]).is_err(),
            "a bare `completions` must be a usage error"
        );

        let cli = Cli::try_parse_from(["robotctl", "completions", "bash"])
            .expect("`completions bash` must parse");
        assert!(matches!(
            cli.namespace,
            Namespace::Completions {
                shell: clap_complete::Shell::Bash
            }
        ));
    }

    /// The completion script is generated from this parser, so the only way the two can
    /// drift is if generation stops covering a namespace. Asserting on the commands an
    /// operator types is what catches that — including the nested ones, since `update` is
    /// where all the useful completions are.
    #[test]
    fn bash_completions_cover_the_command_tree() {
        let mut out = Vec::new();
        clap_complete::generate(
            clap_complete::Shell::Bash,
            &mut Cli::command(),
            "robotctl",
            &mut out,
        );
        let script = String::from_utf8(out).expect("the completion script must be UTF-8");

        for command in [
            "update",
            "version",
            "health",
            "completions",
            "apply",
            "rollback",
            "reset-to-golden",
            "--interrupt-sessions",
        ] {
            assert!(
                script.contains(command),
                "the bash completions never mention `{command}`"
            );
        }
    }

    #[test]
    fn apply_parses_exact_version_and_dry_run() {
        let cli = Cli::try_parse_from([
            "robotctl",
            "update",
            "apply",
            "daemon",
            "--version",
            "1.4.2",
            "--dry-run",
        ])
        .unwrap();

        let Namespace::Update {
            command:
                UpdateCommand::Apply {
                    component,
                    version,
                    dry_run,
                    ..
                },
        } = cli.namespace
        else {
            panic!("expected update apply");
        };
        assert_eq!(component, "daemon");
        assert_eq!(version, Some(semver::Version::new(1, 4, 2)));
        assert!(dry_run);
    }

    /// A malformed version must be rejected by parsing, not sent to the daemon.
    #[test]
    fn apply_rejects_bad_version() {
        assert!(
            Cli::try_parse_from(["robotctl", "update", "apply", "daemon", "--version", "nope"])
                .is_err()
        );
    }

    /// Omitting the version means "latest", which must stay expressible.
    #[test]
    fn apply_without_version_is_latest() {
        let cli = Cli::try_parse_from(["robotctl", "update", "apply", "model"]).unwrap();
        let Namespace::Update {
            command: UpdateCommand::Apply { version, .. },
        } = cli.namespace
        else {
            panic!("expected update apply");
        };
        assert_eq!(version, None);
    }

    /// `--ref` must reach the daemon as `Target::Ref`, not as anything else.
    #[test]
    fn apply_ref_becomes_a_ref_target() {
        let cli = Cli::try_parse_from([
            "robotctl",
            "update",
            "apply",
            "daemon",
            "--ref",
            "my-branch",
        ])
        .expect("--ref must parse");
        let Namespace::Update {
            command: UpdateCommand::Apply {
                git_ref, version, ..
            },
        } = cli.namespace
        else {
            panic!("expected update apply");
        };
        assert_eq!(git_ref.as_deref(), Some("my-branch"));
        assert!(version.is_none());
    }

    /// A branch name with a slash must survive argument parsing untouched.
    #[test]
    fn apply_ref_accepts_a_slash() {
        let cli = Cli::try_parse_from([
            "robotctl",
            "update",
            "apply",
            "daemon",
            "--ref",
            "feature/foo",
        ])
        .expect("a slashed ref must parse");
        let Namespace::Update {
            command: UpdateCommand::Apply { git_ref, .. },
        } = cli.namespace
        else {
            panic!("expected update apply");
        };
        assert_eq!(git_ref.as_deref(), Some("feature/foo"));
    }

    /// Asking for a ref *and* a version is a mistake, and must be reported rather than
    /// resolved by preferring one — the caller would otherwise get a build they did not ask
    /// for and no indication of why.
    #[test]
    fn apply_refuses_both_ref_and_version() {
        let result = Cli::try_parse_from([
            "robotctl",
            "update",
            "apply",
            "daemon",
            "--ref",
            "b",
            "--version",
            "1.0.0",
        ]);
        assert!(result.is_err(), "--ref with --version must be rejected");
    }

    // ── health reporting ─────────────────────────────────────────────────────

    fn health_report(
        robot: Option<proto::HealthResult>,
        robot_error: Option<&str>,
    ) -> HealthReport {
        HealthReport {
            robot,
            robot_error: robot_error.map(str::to_owned),
            software: report(vec![service("robotd", "0.2.0")], Some("0.2.0")),
        }
    }

    /// A working robot, rendered whole: every section present, one line each.
    #[test]
    fn health_renders_hardware_and_software_together() {
        let out = render_health(&health_report(
            Some(proto::HealthResult {
                healthy: true,
                battery: Some(proto::Battery {
                    volts: 7.62,
                    percent: 63.75,
                }),
                motors: Some(proto::MotorThermal {
                    hottest: "left_knee".into(),
                    max_c: 48.0,
                    mean_c: 36.0,
                }),
                cpu_temp_c: Some(52.0),
                control_loop: Some(proto::LoopHealth {
                    target_hz: 50.0,
                    achieved_hz: Some(49.8),
                    ticks: 2490,
                    missed: 2,
                    last_tick_age_ms: 12,
                }),
                bus: proto::BusHealth::default(),
                imu: Some(proto::ImuHealth {
                    ready: true,
                    stale_blocks: 0,
                }),
                ..Default::default()
            }),
            None,
        ));

        assert!(out.contains("robot     healthy"), "{out}");
        assert!(out.contains("49.8 of 50.0 Hz"), "{out}");
        assert!(out.contains("2490 ticks"), "{out}");
        assert!(out.contains("7.62 V (64%)"), "{out}");
        assert!(out.contains("48 °C max (left_knee)"), "{out}");
        // Board and servos on separate lines: they fail differently.
        assert!(out.contains("cpu       52 °C"), "{out}");
        assert!(out.contains("bus       ok"), "{out}");
        assert!(out.contains("imu       ready"), "{out}");
        // And the software half, in the same answer — the whole point of one command.
        assert!(out.contains("software"), "{out}");
        assert!(out.contains("robotd    0.2.0"), "{out}");
        assert!(out.contains("daemon    0.2.0 installed"), "{out}");
    }

    /// A stopped `robotd` must still produce the software half.
    ///
    /// This is the shape of a real support case — "the robot does nothing" is very often a
    /// daemon that failed to start, and what is *installed* is then the interesting half. A
    /// command that bailed out on the first unreachable socket would withhold exactly the
    /// information that explains the unreachable socket.
    #[test]
    fn health_reports_software_when_robotd_is_down() {
        let out = render_health(&health_report(
            None,
            Some(
                "cannot reach robotd at /run/robotd.sock: No such file or directory\nIs the service running?",
            ),
        ));

        assert!(
            out.contains("robot     unavailable — cannot reach robotd"),
            "{out}"
        );
        // First line only: the hint belongs in the warnings block, not wrapped through a
        // column layout.
        assert!(!out.contains("robot     unavailable — cannot reach robotd at /run/robotd.sock: No such file or directory\nIs"), "{out}");
        assert!(out.contains("software"), "{out}");
        assert!(out.contains("daemon    0.2.0 installed"), "{out}");
    }

    /// Nothing measured yet must not render as zeros.
    ///
    /// This is every robot for its first second, and the state a live test never catches. "0.0
    /// Hz" and "0.00 V" describe a stopped loop on a dead battery — the opposite of a robot
    /// that has only just started.
    #[test]
    fn health_renders_unknowns_as_unknown() {
        let out = render_health(&health_report(
            Some(proto::HealthResult {
                reason: Some("control loop has not completed a cycle yet".into()),
                control_loop: Some(proto::LoopHealth {
                    target_hz: 50.0,
                    achieved_hz: None,
                    ticks: 0,
                    missed: 0,
                    last_tick_age_ms: 0,
                }),
                imu: Some(proto::ImuHealth {
                    ready: false,
                    stale_blocks: 0,
                }),
                ..Default::default()
            }),
            None,
        ));

        assert!(
            out.contains("unhealthy: control loop has not completed"),
            "{out}"
        );
        assert!(out.contains("not measured yet"), "{out}");
        assert!(out.contains("not ready"), "{out}");
        // No battery line and no motors line at all, rather than zeroed ones.
        assert!(!out.contains(" V ("), "{out}");
        assert!(!out.contains("°C"), "{out}");
    }

    /// The two findings that must not hide: a bus that has stopped answering, and an IMU that
    /// answers without refreshing.
    ///
    /// Stale IMU reads are the nastiest of the lot — the reads *succeed*, so nothing else
    /// reports a fault, while the orientation feeding the policy is frozen.
    #[test]
    fn health_renders_a_broken_bus_and_a_stale_imu() {
        let out = render_health(&health_report(
            Some(proto::HealthResult {
                healthy: false,
                reason: Some("7 consecutive bus read failures".into()),
                bus: proto::BusHealth {
                    consecutive_errors: 7,
                    startup_failures: 0,
                },
                imu: Some(proto::ImuHealth {
                    ready: true,
                    stale_blocks: 41,
                }),
                ..Default::default()
            }),
            None,
        ));

        assert!(out.contains("7 consecutive read failures"), "{out}");
        assert!(out.contains("41 stale reads"), "{out}");
        assert!(out.contains("orientation may be dead"), "{out}");
    }

    /// An unpowered bench board reads as *degraded*, and the attempt count is the actionable
    /// part: it is how you tell "still coming up" from "there is no robot on this bus".
    #[test]
    fn health_renders_a_degraded_board_waiting_for_its_bus() {
        let out = render_health(&health_report(
            Some(proto::HealthResult {
                degraded: true,
                reason: Some("no robot on the motor bus after 4 attempts".into()),
                bus: proto::BusHealth {
                    consecutive_errors: 0,
                    startup_failures: 4,
                },
                ..Default::default()
            }),
            None,
        ));

        assert!(out.contains("degraded: no robot on the motor bus"), "{out}");
        assert!(
            out.contains("waiting for a robot to answer, 4 attempts"),
            "{out}"
        );
    }

    /// A pinned component and a rollback are both things nobody thinks to ask about, and both
    /// explain "updates stopped working" — so they appear without being asked for.
    #[test]
    fn health_surfaces_a_pin_and_the_last_update() {
        let mut report = health_report(Some(proto::HealthResult::default()), None);
        report.software.components[0].pinned = Some("0.1.9".into());
        report.software.components[0].last_attempt =
            Some("0.1.9 → 0.2.0: ROLLED BACK — not healthy within 30s".into());

        let out = render_health(&report);
        assert!(out.contains("pinned to 0.1.9"), "{out}");
        assert!(out.contains("ROLLED BACK"), "{out}");
    }

    /// The summary line for one update attempt, including the reason a revert happened —
    /// which outside the journal exists nowhere else.
    #[test]
    fn an_attempt_is_summarised_with_its_reason() {
        let entry = |outcome| proto::LogEntry {
            at: 0,
            component: proto::ComponentId::new("daemon"),
            from: Some(semver::Version::new(0, 1, 9)),
            to: Some(semver::Version::new(0, 2, 0)),
            outcome,
        };

        assert_eq!(
            describe_attempt(&entry(proto::Outcome::Success)),
            "0.1.9 → 0.2.0: applied"
        );
        let rolled = describe_attempt(&entry(proto::Outcome::RolledBack {
            reason: "not healthy within 30s".into(),
        }));
        assert!(rolled.contains("ROLLED BACK"), "{rolled}");
        assert!(rolled.contains("not healthy within 30s"), "{rolled}");

        // A first install has no `from`, and must not render as "None → 1.0.0".
        let first = proto::LogEntry {
            from: None,
            ..entry(proto::Outcome::Success)
        };
        assert_eq!(describe_attempt(&first), "0.2.0: applied");
    }

    // ── version reporting ────────────────────────────────────────────────────

    fn report(services: Vec<ServiceReport>, daemon_installed: Option<&str>) -> VersionReport {
        VersionReport {
            robotctl: "0.2.0".into(),
            robotctl_revision: None,
            services,
            components: vec![ComponentReport {
                name: "daemon".into(),
                installed: daemon_installed.map(str::to_owned),
                revision: None,
                pinned: None,
                last_attempt: None,
            }],
            warnings: Vec::new(),
        }
    }

    fn service(name: &'static str, version: &str) -> ServiceReport {
        ServiceReport {
            name,
            version: Some(version.into()),
            revision: None,
            error: None,
        }
    }

    fn service_at(name: &'static str, version: &str, revision: &str) -> ServiceReport {
        ServiceReport {
            revision: Some(revision.into()),
            ..service(name, version)
        }
    }

    /// **From a real board.** A dev-channel install accused itself of a failed restart.
    ///
    /// `robotctl health` on the Radxa reported "robotd is running 0.1.4 but the installed
    /// daemon release is 0.1.4-dev.91.7f685a0 … either the restart did not happen, or it
    /// failed" — while `robotd`'s own revision was `7f685a0`, the very commit that release was
    /// built from. It *was* the new build.
    ///
    /// A binary reports `CARGO_PKG_VERSION`; the prerelease suffix is minted by `xtask
    /// package` from a run number and a SHA, long after the compiler has gone. So on the dev
    /// channel the versions differ by construction and can never agree, and the loudest
    /// warning this command has was firing on every single dev install — training its reader
    /// to ignore it, which is worse than not having it.
    #[test]
    fn a_dev_build_matching_by_revision_is_not_reported_as_behind() {
        let sha = "7f685a0c0a51ba928a3bba5b575b2b78ca8dd59b";
        let mut report = report(
            vec![
                service_at("updaterd", "0.1.4", sha),
                service_at("robotd", "0.1.4", sha),
            ],
            Some("0.1.4-dev.91.7f685a0"),
        );
        // What `listInstalled` reports for the active release: the same commit, in full.
        report.components[0].revision = Some(sha.to_owned());

        let warnings = version_warnings(&report, Some(&semver::Version::parse("0.1.4").unwrap()));
        assert!(
            warnings.is_empty(),
            "same commit on both sides must not warn: {warnings:?}"
        );
    }

    /// The other half of that fix: a genuinely stale `robotd` must still be caught, and on the
    /// dev channel the *revision* is the only thing that can catch it — both sides say `0.1.4`.
    #[test]
    fn a_dev_build_from_another_commit_is_still_reported_as_behind() {
        let mut report = report(
            vec![service_at(
                "robotd",
                "0.1.4",
                "28c8f3b636fd0ada2b30cd8b7c367ef375c27f29",
            )],
            Some("0.1.4-dev.91.7f685a0"),
        );
        report.components[0].revision = Some("7f685a0c0a51ba928a3bba5b575b2b78ca8dd59b".to_owned());

        let warnings = version_warnings(&report, None);
        assert_eq!(warnings.len(), 1, "{warnings:?}");
        assert!(warnings[0].contains("robotd is running"), "{warnings:?}");
        // Named by revision, since the versions are identical and would show no difference.
        assert!(warnings[0].contains("rev 28c8f3b"), "{warnings:?}");
        assert!(warnings[0].contains("rev 7f685a0"), "{warnings:?}");
        // Abbreviated, not forty characters of hex in the middle of a sentence.
        assert!(!warnings[0].contains("28c8f3b636"), "{warnings:?}");
    }

    /// A short revision and a full one name the same commit. `dev.yml` passes `GITHUB_SHA` in
    /// full, but nothing guarantees a hand-cut release does.
    #[test]
    fn an_abbreviated_revision_matches_its_full_form() {
        assert!(same_revision(
            "7f685a0",
            "7f685a0c0a51ba928a3bba5b575b2b78ca8dd59b"
        ));
        assert!(!same_revision(
            "28c8f3b",
            "7f685a0c0a51ba928a3bba5b575b2b78ca8dd59b"
        ));
        // Too short to mean anything: a prefix rule with no floor would match everything,
        // including an empty string, and silently stop reporting stale daemons.
        assert!(!same_revision(
            "",
            "7f685a0c0a51ba928a3bba5b575b2b78ca8dd59b"
        ));
        assert!(!same_revision(
            "7f68",
            "7f685a0c0a51ba928a3bba5b575b2b78ca8dd59b"
        ));
    }

    /// The whole point of the command: after an update, `updaterd` is still running the old
    /// binary. Support must be told, and told that it is expected — otherwise the obvious
    /// reading is "the update did not work" and someone starts undoing a working robot.
    #[test]
    fn a_running_updaterd_behind_the_installed_release_is_flagged_and_explained() {
        let r = report(vec![service("updaterd", "0.1.0")], Some("0.2.0"));
        let warnings = version_warnings(&r, Some(&semver::Version::new(0, 1, 0)));

        assert_eq!(warnings.len(), 1, "{warnings:?}");
        let warning = &warnings[0];
        assert!(warning.contains("running 0.1.0"), "{warning}");
        assert!(warning.contains("0.2.0"), "{warning}");
        assert!(
            warning.contains("never restarts itself"),
            "must explain why this is expected, not merely report it: {warning}"
        );
        assert!(
            warning.contains("reboot"),
            "must say what resolves it: {warning}"
        );
    }

    /// The matching case must stay silent. A diagnostic that always warns trains people to
    /// ignore it.
    #[test]
    fn matching_versions_produce_no_warning() {
        let r = report(
            vec![service("updaterd", "0.2.0"), service("robotd", "0.2.0")],
            Some("0.2.0"),
        );
        let warnings = version_warnings(&r, Some(&semver::Version::new(0, 2, 0)));
        assert!(warnings.is_empty(), "{warnings:?}");
    }

    /// robotd lagging is a *different* problem from updaterd lagging: it is in on_apply's
    /// restart set, so it should already have been restarted. The two must not share one
    /// message, or the more serious case gets read as the benign one.
    #[test]
    fn a_lagging_robotd_gets_its_own_diagnosis() {
        let r = report(
            vec![service("updaterd", "0.2.0"), service("robotd", "0.1.0")],
            Some("0.2.0"),
        );
        let warnings = version_warnings(&r, Some(&semver::Version::new(0, 2, 0)));

        assert_eq!(warnings.len(), 1, "{warnings:?}");
        assert!(
            warnings[0].starts_with("robotd is running 0.1.0"),
            "{:?}",
            warnings[0]
        );
        assert!(
            warnings[0].contains("restart set"),
            "must point at the restart, not at a reboot: {:?}",
            warnings[0]
        );
    }

    /// A daemon that cannot be asked must be reported, not silently omitted — and the
    /// report must still be produced, because that is when it is needed most.
    #[test]
    fn an_unavailable_daemon_is_reported_rather_than_dropped() {
        let r = report(
            vec![ServiceReport::failed(
                "updaterd",
                "connection refused".into(),
            )],
            None,
        );
        let warnings = version_warnings(&r, None);

        assert_eq!(warnings.len(), 1, "{warnings:?}");
        assert!(warnings[0].contains("updaterd"), "{:?}", warnings[0]);
        assert!(
            warnings[0].contains("connection refused"),
            "{:?}",
            warnings[0]
        );

        // And it must render without panicking or claiming a version it does not know.
        let rendered = render_version(&r);
        assert!(rendered.contains("unavailable"), "{rendered}");
        assert!(!rendered.contains("0.0.0"), "{rendered}");
    }

    /// The rendered report must actually contain both numbers side by side. This is the
    /// text someone pastes into a support thread.
    #[test]
    fn rendering_shows_running_and_installed_together() {
        let mut r = report(vec![service("updaterd", "0.1.0")], Some("0.2.0"));
        r.warnings = version_warnings(&r, Some(&semver::Version::new(0, 1, 0)));
        let rendered = render_version(&r);

        assert!(rendered.contains("running"), "{rendered}");
        assert!(rendered.contains("installed"), "{rendered}");
        assert!(rendered.contains("0.1.0"), "{rendered}");
        assert!(rendered.contains("0.2.0"), "{rendered}");
    }
}
