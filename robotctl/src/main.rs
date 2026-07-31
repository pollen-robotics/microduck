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

use clap::{Args, Parser, Subcommand};
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

    /// Path to the robotd socket. Only used by `version`, which asks each daemon what it
    /// is running.
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

    /// What is running on this robot, and what is installed. The first thing to ask for
    /// in a support report.
    ///
    /// Distinct from `--version`, which reports only this binary. This asks every daemon.
    Version {
        /// Machine-readable output, for support bundles and scripts.
        #[arg(long)]
        json: bool,
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

/// Ask every daemon what it is running, and compare against what is installed.
///
/// Deliberately does **not** use the ordinary `Client::connect(..)?` + `hello()?` path.
/// That exits non-zero when `updaterd` is unreachable, which is precisely the situation
/// where someone is running this command. Every failure here becomes a line in the report
/// instead.
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
        println!(
            "{:8.2}  {:>5}  {:5.1}Hz miss={:<4} {}  req[{:+.2} {:+.2} {:+.2}] \
             app[{:+.2} {:+.2} {:+.2}]{}",
            state.t,
            state.policy,
            state.control_loop.hz,
            state.control_loop.missed,
            if state.safety.fallen {
                "FALLEN"
            } else {
                "ok    "
            },
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

fn run_version(socket: &Path, robot_socket: &Path, json: bool) -> Result<(), Failure> {
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
            }
        })
        .collect()
}

/// Disagreements worth telling a human about.
///
/// Pure, so the interesting cases are unit-testable without daemons: the running/installed
/// mismatch is the one support will actually hit, and it must be explained rather than
/// merely flagged — it is *expected* right after an update and alarming only if it
/// survives a reboot.
fn version_warnings(
    report: &VersionReport,
    updaterd_running: Option<&semver::Version>,
) -> Vec<String> {
    let mut warnings = Vec::new();

    let daemon_installed = report
        .components
        .iter()
        .find(|c| c.name == "daemon")
        .and_then(|c| c.installed.as_deref())
        .and_then(|v| semver::Version::parse(v).ok());

    if let (Some(running), Some(installed)) = (updaterd_running, daemon_installed.as_ref())
        && running != installed
    {
        warnings.push(format!(
            "updaterd is running {running} but the installed daemon release is {installed}.\n  \
             Expected right after an update — updaterd never restarts itself, so it keeps\n  \
             running the old binary until the next reboot. If this survives a reboot, the\n  \
             new release is not being launched: check the `current` symlink and the unit's\n  \
             ExecStart path."
        ));
    }

    // robotd is in `on_apply`'s restart set, so unlike updaterd it *should* already be on
    // the installed release. A mismatch here means the restart did not take effect, which
    // is a different and more serious situation than updaterd's expected lag.
    let robotd_running = report
        .services
        .iter()
        .find(|s| s.name == "robotd")
        .and_then(|s| s.version.as_deref())
        .and_then(|v| semver::Version::parse(v).ok());
    if let (Some(running), Some(installed)) = (robotd_running, daemon_installed.as_ref())
        && &running != installed
    {
        warnings.push(format!(
            "robotd is running {running} but the installed daemon release is {installed}.\n  \
             robotd is in on_apply's restart set, so it should already be on the installed\n  \
             release: either the restart did not happen, or it failed and systemd restarted\n  \
             the old binary. Check `systemctl status robotd` and the update log."
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
        Some(rev) => format!("rev {rev}"),
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
fn report_progress(progress: &proto::Progress) {
    match progress.percent {
        Some(percent) => eprintln!("  {:?} {percent}%", progress.phase),
        None => eprintln!("  {:?}", progress.phase),
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
            eprintln!("error: {}", failure.message);
            ExitCode::from(failure.code)
        }
    }
}

fn run(cli: Cli) -> Result<(), Failure> {
    let command = match cli.namespace {
        Namespace::Version { json } => {
            return run_version(&cli.socket, &cli.robot_socket, json);
        }
        Namespace::Monitor { hz, json } => {
            return run_monitor(&cli.robot_socket, hz, json);
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
