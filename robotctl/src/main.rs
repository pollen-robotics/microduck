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
//!    (`docs/design/updater-design.md` §16.1).
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

mod monitor;

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

    /// Path to the configd socket — wifi and the robot's identity.
    #[arg(long, global = true, default_value = proto::socket::CONFIG)]
    config_socket: PathBuf,

    /// Path to `padd`'s raw input tap, which `monitor` reads the pad's own event stream from.
    ///
    /// Optional in the only sense that matters: nothing fails when it is absent. `monitor` says the
    /// tap is not there and carries on showing the robot.
    #[arg(long, global = true, default_value = proto::socket::PAD)]
    pad_socket: PathBuf,

    #[command(subcommand)]
    namespace: Namespace,
}

/// Only `update` exists today, plus `version`. The namespace layer is here so adding
/// `robotctl motors` later is additive rather than a restructure.
#[derive(Subcommand, Debug)]
enum Namespace {
    /// Wifi. Served by `configd`, which drives NetworkManager.
    #[command(subcommand_required = true, arg_required_else_help = true)]
    Net {
        #[command(subcommand)]
        command: NetCommand,
    },

    /// The robot's name, identity and power.
    #[command(subcommand_required = true, arg_required_else_help = true)]
    System {
        #[command(subcommand)]
        command: SystemCommand,
    },

    /// Power to the joints: stand the robot up, or let it go.
    ///
    /// Served by `robotd`, which owns the motor bus — so unlike `robotd init` these need no daemon
    /// stopped and cannot corrupt the bus by writing to it at the same time as the control loop.
    #[command(subcommand_required = true, arg_required_else_help = true)]
    Robot {
        #[command(subcommand)]
        command: RobotCommand,
    },

    /// The gamepad. Pair one, see what is paired, forget one.
    ///
    /// Driving is not a command here: `padd.service` runs on its own and picks up whatever pad is
    /// connected, so pairing is the only step. `pad status` is how you find out whether that is
    /// working, and it answers the two questions separately — is a pad connected, and is `padd`
    /// running — because a connected pad and a dead driver look identical from the outside.
    #[command(subcommand_required = true, arg_required_else_help = true)]
    Pad {
        #[command(subcommand)]
        command: PadCommand,
    },

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
    /// the reason next to it. Every joint is there too, measured against commanded, so a
    /// servo that is not keeping up is visible rather than inferred, and the policy that is
    /// loaded is named — `walk` is a mode two releases with different gaits both report.
    ///
    /// On a terminal it repaints one frame in place: `q` quits, `↑`/`↓` scroll the joints if
    /// the window is too short for all of them. Redirected or piped it prints one line per
    /// tick instead, so `> log` and `| grep` behave; the joint vectors are in `--json`, which
    /// carries the whole state.
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
enum NetCommand {
    /// SSID, signal and addresses. Changes nothing.
    Status {
        #[arg(long)]
        json: bool,
    },
    /// Networks in range, strongest first. Changes nothing.
    Scan {
        #[arg(long)]
        json: bool,
    },
    /// Join a network and store it, so the robot rejoins by itself afterwards.
    Connect {
        ssid: String,
        /// Passphrase. Omit for an open network.
        ///
        /// Prefer `--psk-stdin` on a shared machine: an argument is visible in `ps` for the
        /// lifetime of the command, and this one is a credential.
        #[arg(long, conflicts_with = "psk_stdin")]
        psk: Option<String>,
        /// Read the passphrase from stdin instead, so it never reaches the process list.
        #[arg(long)]
        psk_stdin: bool,
        #[arg(long)]
        json: bool,
    },
    /// Forget a stored network.
    Forget {
        ssid: String,
        #[arg(long)]
        json: bool,
    },
}

#[derive(Subcommand, Debug)]
enum SystemCommand {
    /// Name, serial and uptime. Changes nothing.
    Info {
        #[arg(long)]
        json: bool,
    },
    /// Rename the robot. This is the name a phone sees over Bluetooth.
    SetName {
        name: String,
        #[arg(long)]
        json: bool,
    },
    /// Show the Bluetooth pairing PIN.
    ///
    /// Deliberately reachable here and NOT over Bluetooth: a PIN readable by an unpaired peer
    /// would authorise nothing at all.
    Pin {
        #[arg(long)]
        json: bool,
    },
    /// Set the Bluetooth pairing PIN. Six digits.
    SetPin {
        pin: String,
        #[arg(long)]
        json: bool,
    },
    /// Reboot, cleanly, through systemd.
    Reboot {
        /// Reboot without asking.
        #[arg(long)]
        yes: bool,
        #[arg(long)]
        json: bool,
    },
}

#[derive(Subcommand, Debug)]
enum RobotCommand {
    /// Power the joints and ramp to the home pose, over about two seconds.
    ///
    /// **This moves every joint.** Have the robot on its stand, or hold it. Needs no policy — a
    /// robot with no walking network can still stand — and it is what the gamepad's Start does on
    /// its way to driving, so running this by hand is for the bench rather than the everyday path.
    ///
    /// Refused on a robot that has fallen: the fall gate holds a fallen robot limp on purpose. Stand
    /// it up by hand first.
    Init {
        #[arg(long)]
        json: bool,
    },

    /// Cut power to the joints.
    ///
    /// **The robot collapses** if nothing is holding it. This is what you want before picking it up
    /// or putting it away, and it is the only way back to limp short of cutting power.
    ///
    /// Not the same as stopping: `robot.stop` zeroes the velocity and keeps the robot standing, and
    /// pressing Start again disables the policy while still holding the pose.
    Relax {
        /// Let go without asking.
        #[arg(long)]
        yes: bool,
        #[arg(long)]
        json: bool,
    },
}

#[derive(Subcommand, Debug)]
enum PadCommand {
    /// Which pads this robot is paired to, and whether `padd` is driving. Changes nothing.
    Status {
        #[arg(long)]
        json: bool,
    },

    /// Pair the gamepad that is in pairing mode now.
    ///
    /// Put the pad in pairing mode first. On an Xbox controller: switch it on with a short press of
    /// the Xbox button, then press the small **Sync** button on the top edge, next to the USB-C
    /// port, until the Xbox light flashes quickly. Do NOT hold the Xbox button itself — that
    /// switches the controller off. On a DualSense: hold Create and PS together until the light bar
    /// flashes.
    ///
    /// Then run this. No MAC address needed: the robot looks for a gamepad in pairing mode and
    /// takes the one it finds.
    ///
    /// **A pad already paired does not get in the way.** The robot prefers one in pairing mode, so a
    /// second pad can be added without forgetting the first, and both stay bonded — `padd` drives
    /// whichever connects. With nothing new in pairing mode this reports the pad already bonded, and
    /// re-asserts its trust, after waiting out the search window; `--timeout` shortens that.
    ///
    /// Once paired the pad is also *trusted*, which is what makes it reconnect by itself after a
    /// reboot with nobody logged in. Nothing else is needed — `padd.service` is already running and
    /// starts driving when the pad connects.
    Pair {
        /// Which pad, when more than one is in pairing mode — or when it is hardware the robot does
        /// not recognise as a gamepad. `pad pair` prints the addresses it saw when it refuses.
        mac: Option<String>,

        /// How long to look, in seconds.
        #[arg(long, value_name = "SECONDS")]
        timeout: Option<u32>,

        #[arg(long)]
        json: bool,
    },

    /// Forget a pad, so it stops reconnecting to this robot.
    ///
    /// Removes **the robot's half** of the bond, which is all a robot can remove. The pad keeps its
    /// own half, so pairing it again needs it back in pairing mode — otherwise it arrives with a key
    /// this robot no longer has and the bond is refused.
    Forget {
        mac: String,
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

        /// Install the release candidate from the staging channel.
        ///
        /// A candidate is what `release.yml` published and nobody has promoted yet. It is
        /// signed with the release key like any release, and carries the version it will be
        /// promoted under — what makes it unreachable otherwise is that it is flagged as a
        /// prerelease, which a plain `apply` skips so that no robot drifts onto a build no
        /// one has validated.
        ///
        /// This is that filter's only opt-in, and it is per-command on purpose: nothing on
        /// the board is left switched on afterwards, so the next `apply` is back on stable.
        /// Pair it with `--version` to name one candidate rather than the newest.
        #[arg(long, conflicts_with = "git_ref")]
        staging: bool,

        /// Install from a directory on this robot instead of from the configured source.
        ///
        /// The laptop-to-board path: `scripts/dev-push.sh` builds, signs with the dev key,
        /// copies the directory over and ends here. The directory holds what a release is —
        /// `<version>.manifest.json`, its `.minisig`, the artifact and the artifact's
        /// `.minisig` — which is what `cargo xtask package` writes.
        ///
        /// This is an ordinary apply: preflight, signature, hash, compatibility, the health
        /// gate and auto-rollback all still run, and a build that does not come up is
        /// reverted. That is the whole difference from `updaterd install --from`, which
        /// forces the gate off and therefore refuses to touch a live release at all.
        ///
        /// Being local relaxes nothing: a build installs because the dev key is in this
        /// robot's trusted set and `allow_dev_keys` is on, so a customer robot refuses it
        /// exactly as it refuses `--ref`.
        ///
        /// **Not under /tmp or /var/tmp.** `updaterd.service` sets `PrivateTmp=yes`, so the
        /// daemon that reads this directory has its own pair of those and neither is the one
        /// a shell copied into. Preflight names that case rather than letting it surface as a
        /// missing manifest in a directory the caller can plainly see.
        ///
        /// `conflicts_with = "staging"` because a directory has no channels — the source
        /// layer says so too, and being told at parse time is better than after a connection.
        #[arg(long, value_name = "DIR", conflicts_with = "staging")]
        from: Option<PathBuf>,

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

/// What to tell someone whose socket connect failed, chosen by *why* it failed.
///
/// Every kind used to get `Is the service running?  systemctl status …`, which is the right
/// question for exactly one of them. On a freshly provisioned board the usual answer is
/// `EACCES`, and there the service is running fine — the caller is simply not in the `robot`
/// group yet, because `install.sh` adds them to it and a group only takes effect in a new
/// login session. Sending that person to `systemctl status` shows them an active daemon and no
/// explanation, which is the failure mode [`Client::connect_to`] already exists to avoid.
///
/// `EACCES` from `connect` means something was found and refused us, never that it was absent
/// — that is `ENOENT` — so the two cases can be told apart without stat'ing anything.
fn unreachable_hint(service: &str, path: &std::path::Path, e: &std::io::Error) -> String {
    use std::io::ErrorKind;

    let head = format!("cannot reach {service} at {}: {e}", path.display());
    match e.kind() {
        ErrorKind::PermissionDenied => format!(
            "{head}\n\
             The socket refused this user rather than being absent, so {service} is running\n\
             and this is group membership, not a crash. It is root:robot mode 0660, and a\n\
             process's groups are fixed when it starts — so a group added since this shell\n\
             began is not in it. Installing adds the operator to `robot`, so usually:\n\
             \x20 newgrp robot\n\
             Check with `id -nG`; a new login has it already. If `robot` is not there at all,\n\
             `sudo usermod -aG robot $USER` and log out. `sudo robotctl …` works regardless."
        ),
        // ENOENT: nothing at that path. Either the daemon never started, or it is listening
        // somewhere else — which is worth naming, because `--socket` and the `robot_socket`
        // setting both move it and neither leaves a trace at the default path.
        ErrorKind::NotFound => format!(
            "{head}\n\
             Nothing is listening at that path. Is the service running, and is this the \
             socket it was told to use?  systemctl status {service}"
        ),
        // A socket file with no listener: the usual cause is a daemon that died without
        // cleaning up, so the file outlives the process that made it.
        ErrorKind::ConnectionRefused => format!(
            "{head}\n\
             The socket file is there but nothing is accepting on it, which is what a daemon \
             that died leaves behind.  systemctl status {service}"
        ),
        _ => format!("{head}\nIs the service running?  systemctl status {service}"),
    }
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
        let stream = UnixStream::connect(path)
            .map_err(|e| Failure::new(exit::UNREACHABLE, unreachable_hint(service, path, &e)))?;
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
            // The daemon's message is passed through unadorned. It used to gain
            // "robotctl and updaterd are out of step; install matching versions" here, which was
            // true and not actionable — the daemon now says which of the two is behind and what
            // to do, which this side cannot know, and two remedies would disagree.
            return Err(Failure::new(exit::FAILED, error.message));
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
    /// What systemd says about every unit a release manages, and which release each is running
    /// from. Empty when `configd` could not be asked — including on a release older than the one
    /// that added `system.services`, which is a silence rather than a warning.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    units: Vec<proto::ServiceUnit>,
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
fn run_health(
    socket: &Path,
    robot_socket: &Path,
    config_socket: &Path,
    json: bool,
) -> Result<(), Failure> {
    let mut report = HealthReport {
        robot: None,
        robot_error: None,
        software: collect_version_report(socket, robot_socket, config_socket),
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
                // Ticks are what make a stale count mean anything: 9 of them is a healthy board
                // over 100k reads and a broken one over 20. Taken from the loop line above
                // rather than carried in `ImuHealth`, since it is the same number of reads.
                let ticks = health.control_loop.as_ref().map_or(0, |l| l.ticks);
                let stale = if imu.frozen() {
                    // The one case worth shouting about: the board answers, so the bus reports
                    // no error and `ready` stays true, while the orientation being fed to the
                    // policy has not changed in half a second.
                    format!(
                        ", orientation frozen — {} stale reads running",
                        imu.consecutive_stale_blocks
                    )
                } else {
                    match (imu.stale_blocks, ticks) {
                        (0, _) => String::new(),
                        // Reported as a rate, because the absolute count is the thing that read
                        // as an alarm on a robot that was fine. One in five figures is a board
                        // keeping its own clock; one in three reads is a board in trouble, and
                        // the ratio says which without needing a threshold here.
                        (n, ticks) if ticks > n => {
                            format!(", {n} stale reads — 1 in {}", ticks / n)
                        }
                        // No tick count to scale against, or more stale reads than ticks
                        // sampled. Say the number plainly rather than divide by it.
                        (n, _) => format!(", {n} stale reads"),
                    }
                };
                let _ = writeln!(
                    out,
                    "  {:<9} {}{stale}",
                    "imu",
                    if imu.ready { "ready" } else { "not ready" },
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

    // After the installed lines rather than between them and the daemons above, because it has a
    // heading and they do not: a block inserted there ends the `software` block early and adopts
    // `daemon 0.5.0 installed` as one more unit — which reads as a systemd unit named `daemon`.
    if !report.software.units.is_empty() {
        let _ = writeln!(out, "\nunits");
        out.push_str(&render_units(&report.software.units, 2));
    }

    // Same shape `robotctl version` uses, blank line and all: these are often multi-line —
    // the `systemctl status` hint on an unreachable daemon is the useful half — and the two
    // commands should not disagree about how a warning looks.
    for warning in &report.software.warnings {
        let _ = writeln!(out, "\n! {warning}");
    }

    out
}

fn run_version(
    socket: &Path,
    robot_socket: &Path,
    config_socket: &Path,
    json: bool,
) -> Result<(), Failure> {
    let report = collect_version_report(socket, robot_socket, config_socket);

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
fn collect_version_report(
    socket: &Path,
    robot_socket: &Path,
    config_socket: &Path,
) -> VersionReport {
    let build = proto::build_info!();
    let mut report = VersionReport {
        robotctl: build.version.to_owned(),
        robotctl_revision: build.revision.map(str::to_owned),
        services: Vec::new(),
        units: Vec::new(),
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

    // configd, over its own socket. Same treatment as robotd: unreachable is a line in the
    // report rather than an error, because this command has to work when a daemon is down.
    //
    // `btd` is deliberately absent. It serves no socket — it is a *client* of the other three —
    // so there is nothing to ask it, and every crate in the workspace shares one version line, so
    // its version is the release's. "Is btd running" is `systemctl status btd`, a different
    // question from the one this answers.
    match Client::connect_to("configd", config_socket) {
        Err(failure) => report
            .services
            .push(ServiceReport::failed("configd", failure.message)),
        Ok(mut client) => {
            match client.hello_result() {
                Ok(hello) => report.services.push(ServiceReport {
                    name: "configd",
                    version: hello.daemon_version.map(|v| v.to_string()),
                    revision: hello.revision,
                    error: None,
                }),
                Err(failure) => report
                    .services
                    .push(ServiceReport::failed("configd", failure.message)),
            }
            // The same connection: `configd` is the only thing on the robot that can answer this —
            // reading another user's `/proc/<pid>/exe` needs privilege this CLI does not have.
            //
            // A failure is left silent rather than warned about. The commonest cause is a `configd`
            // from a release older than the one that added `system.services`, and a support tool
            // that shouts about the old robot it was pointed at is a support tool people stop
            // reading.
            report.units = client
                .call(&proto::Call::SystemServices)
                .ok()
                .and_then(|response| response.result_as::<Vec<proto::ServiceUnit>>().ok())
                .unwrap_or_default();
        }
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
/// The unit lines: what systemd says, and which release each process is actually running.
///
/// A block of its own rather than merged into the service lines above, because it answers a
/// different question with different evidence. A service line is what a daemon *said about itself*
/// over its socket; a unit line is what systemd and `/proc` say about it from outside — which is the
/// only available answer for `btd` and `padd`, and the only one at all for a daemon that is not
/// running to be asked.
fn render_units(units: &[proto::ServiceUnit], indent: usize) -> String {
    use std::fmt::Write;
    let mut out = String::new();

    for unit in units {
        // `btd.service` is how systemd names it and `btd` is how everyone else does, including the
        // `systemctl` line someone is about to type.
        let name = unit.unit.strip_suffix(".service").unwrap_or(&unit.unit);
        let state = match unit.state {
            proto::UnitState::Active => "active",
            proto::UnitState::Inactive => "stopped",
            proto::UnitState::Absent => "not installed",
            proto::UnitState::Unknown => "unknown",
        };

        let detail = match &unit.identity {
            // The release, because that is what gets compared with what is installed — and the
            // revision beside it, because on the dev channel two releases share a crate version and
            // the SHA is the whole difference.
            Some(identity) => {
                let build = match identity.release() {
                    Some(release) => release.to_string(),
                    // Published, but not from a release directory: a hand-built binary on a dev
                    // board. Its own version is all it can honestly claim.
                    None => format!("{} (not a release)", identity.version),
                };
                match &identity.revision {
                    Some(rev) => format!(" · {build} (rev {})", short_revision(rev)),
                    None => format!(" · {build}"),
                }
            }
            // Nothing published. For a stopped unit that is expected — systemd removes the runtime
            // directory with the unit. For a running one it means a build too old to publish, and
            // saying so beats inferring a version from somewhere else.
            None if unit.state == proto::UnitState::Active => " · build unknown (old)".to_owned(),
            None => String::new(),
        };
        let _ = writeln!(out, "{:indent$}{name:<9} {state}{detail}", "");
    }

    out
}

/// Units running a different release than the one installed.
///
/// **Only the ones no socket covers.** `updaterd`, `robotd` and `configd` report their own build
/// when asked, and that answer is better than a path — it comes from the running process. Warning
/// from both sources would print one problem twice in two different wordings.
///
/// A stopped unit is deliberately *not* warned about here. The unit block already prints `stopped`
/// next to its name, and a robot whose owner has no gamepad and disabled `padd` should not be told
/// off about it on every health check. A version disagreement is different: nobody chooses that, and
/// it is invisible without being pointed at.
fn unit_warnings(
    units: &[proto::ServiceUnit],
    socket_reported: &[ServiceReport],
    installed: Option<&semver::Version>,
) -> Vec<String> {
    let Some(installed) = installed else {
        return Vec::new();
    };

    let mut warnings = Vec::new();
    for unit in units {
        let name = unit.unit.strip_suffix(".service").unwrap_or(&unit.unit);
        if socket_reported.iter().any(|service| service.name == name) {
            continue;
        }
        let Some(running) = unit.identity.as_ref().and_then(proto::Identity::release) else {
            continue;
        };
        if running == *installed {
            continue;
        }

        warnings.push(format!(
            "{name} is running {running} but the installed daemon release is {installed}.\n  \
             The restart did not take effect, so the old binary is still the one serving.\n  \
             An update schedules some of these a few seconds after it replies, so a report\n  \
             taken during one can show this briefly. Otherwise restart it:\n  \
             sudo systemctl restart {name}"
        ));
    }
    warnings
}

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

    // First, because "the new binary is not the one running" outranks everything else here: it
    // explains symptoms that otherwise look like the release itself being broken.
    warnings.extend(unit_warnings(
        &report.units,
        &report.services,
        daemon_installed.as_ref(),
    ));
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
             Expected for a few seconds after an update — updaterd cannot restart itself\n  \
             mid-update, so the engine schedules that restart 5s after it replies. If this\n  \
             is still true a minute later, the scheduled restart did not happen or the new\n  \
             binary would not start: check `systemctl status updaterd` and the journal.",
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

    // configd joined on_apply's restart set with robotd, so the same reasoning applies: a
    // mismatch means the restart did not take, not an expected lag.
    //
    // Compared by *revision* via `is_behind`, exactly as robotd is. Comparing versions instead
    // warns on every dev-channel install and is how this first shipped: a daemon reports its
    // crate version (`0.2.0`) while the release is named `0.2.0-dev.121.de58259`, so the two
    // strings differ while the binary is precisely the one installed. A diagnostic that cries
    // wolf on the ordinary path is worse than none.
    let configd = report.services.iter().find(|s| s.name == "configd");
    let configd_running = configd
        .and_then(|s| s.version.as_deref())
        .and_then(|v| semver::Version::parse(v).ok());
    let configd_revision = configd.and_then(|s| s.revision.as_deref());
    if let (Some(running), Some(installed)) = (configd_running.as_ref(), daemon_installed.as_ref())
        && is_behind(running, configd_revision, installed, daemon_revision)
    {
        warnings.push(format!(
            "configd is running {} but the installed daemon release is {}.\n  \
             configd is in on_apply's restart set, so it should already be on the installed\n  \
             release: either the restart did not happen, or it failed and systemd restarted\n  \
             the old binary. Check `systemctl status configd` and the update log.",
            identify(running, configd_revision),
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

    if !report.units.is_empty() {
        let _ = writeln!(out, "\nunits");
        out.push_str(&render_units(&report.units, 2));
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

// ── configd: net.* and system.* ──────────────────────────────────────────────

/// A response becomes its result, or the failure the daemon reported.
///
/// The `update` path inlines this because it also has to print progress; these calls have none,
/// so they share one line.
fn result_of(response: proto::Response) -> Result<serde_json::Value, Failure> {
    if let Some(error) = response.error {
        return Err(Failure::from_rpc(error));
    }
    Ok(response.result.unwrap_or(serde_json::Value::Null))
}

/// Ask `configd` one question and print the answer.
///
/// Every one of these is a single call with no progress stream, so they share one shape:
/// connect, handshake, call, render. The rendering is deliberately not `Debug` output — a
/// human running `robotctl net status` wants two lines, and `--json` is there for everything
/// else.
fn run_net(socket: &Path, command: NetCommand) -> Result<(), Failure> {
    let mut client = Client::connect_to("configd", socket)?;
    client.hello()?;

    let (call, json) = match &command {
        NetCommand::Status { json } => (proto::Call::NetStatus, *json),
        NetCommand::Scan { json } => (proto::Call::NetScan, *json),
        NetCommand::Forget { ssid, json } => (
            proto::Call::NetForget(proto::NetForgetParams { ssid: ssid.clone() }),
            *json,
        ),
        NetCommand::Connect {
            ssid,
            psk,
            psk_stdin,
            json,
        } => {
            let psk = if *psk_stdin {
                Some(read_secret()?)
            } else {
                psk.clone()
            };
            (
                proto::Call::NetConnect(proto::NetConnectParams {
                    ssid: ssid.clone(),
                    psk,
                }),
                *json,
            )
        }
    };

    let result = result_of(client.call(&call)?)?;
    if json {
        println!("{}", compact(&result));
        return Ok(());
    }

    match command {
        NetCommand::Status { .. } => println!("{}", render_net_status(&result)?),
        NetCommand::Scan { .. } => println!("{}", render_scan(&result)?),
        NetCommand::Connect { .. } => return report_connect(&result),
        NetCommand::Forget { ssid, .. } => {
            let forgotten: proto::ForgetResult = decode(&result)?;
            if forgotten.removed {
                println!("forgot {ssid}");
            } else {
                // Not an error: a client asking twice should not be told it failed.
                println!("{ssid} was not stored");
            }
        }
    }
    Ok(())
}

fn run_system(socket: &Path, command: SystemCommand) -> Result<(), Failure> {
    // Asked before connecting, so a robot is not disturbed by a command the operator then
    // aborts.
    if let SystemCommand::Reboot { yes: false, .. } = &command {
        return Err(Failure::new(
            exit::USAGE,
            "this reboots the robot. Re-run with --yes if that is what you want.".to_owned(),
        ));
    }

    let mut client = Client::connect_to("configd", socket)?;
    client.hello()?;

    let (call, json) = match &command {
        SystemCommand::Info { json } => (proto::Call::SystemInfo, *json),
        SystemCommand::SetName { name, json } => (
            proto::Call::SystemSetName(proto::SetNameParams { name: name.clone() }),
            *json,
        ),
        SystemCommand::Pin { json } => (proto::Call::SystemPairingPin, *json),
        SystemCommand::SetPin { pin, json } => (
            proto::Call::SystemSetPairingPin(proto::SetPairingPinParams { pin: pin.clone() }),
            *json,
        ),
        SystemCommand::Reboot { json, .. } => (proto::Call::SystemReboot, *json),
    };

    let result = result_of(client.call(&call)?)?;
    if json {
        println!("{}", compact(&result));
        return Ok(());
    }

    match command {
        SystemCommand::Info { .. } => {
            let info: proto::SystemInfoResult = decode(&result)?;
            println!("name    {}", info.name);
            println!(
                "serial  {}",
                // A board with no readable SoC serial, not a board nobody provisioned: the
                // identity is derived from the hardware rather than assigned.
                info.serial.as_deref().unwrap_or("unknown on this board")
            );
            println!("uptime  {}", format_uptime(info.uptime_seconds));
        }
        SystemCommand::SetName { .. } => {
            let renamed: proto::SetNameResult = decode(&result)?;
            // The stored name, not what was asked for: trimming and truncation mean they can
            // differ, and showing the request would disagree with the robot.
            println!("name    {}", renamed.name);
            // btd reconciles its advertisement against configd every few seconds, so this needs
            // no restart. Said out loud because it used to need one, and because a phone shows
            // the old name until it scans again — which reads as the rename not having worked.
            println!(
                "Bluetooth advertises it within a few seconds; a phone must re-scan to see it"
            );
        }
        SystemCommand::Pin { .. } | SystemCommand::SetPin { .. } => {
            let pin: proto::PairingPinResult = decode(&result)?;
            println!("pairing PIN  {}", pin.pin);
            if pin.is_default {
                println!(
                    "This is the factory default, so it authorises nothing — anyone in range \n\
                     knows it. Set a per-robot PIN:  sudo robotctl system set-pin <6 digits>"
                );
            }
        }
        SystemCommand::Reboot { .. } => {
            let reboot: proto::RebootResult = decode(&result)?;
            println!("rebooting in {}s", reboot.in_seconds);
        }
    }
    Ok(())
}

/// Power to the joints, through `robotd`.
fn run_robot(socket: &Path, command: RobotCommand) -> Result<(), Failure> {
    // Asked before connecting, so a robot is not dropped by a command the operator then aborts.
    // Same shape as `system reboot`, and for a more immediate reason: this one takes effect in
    // milliseconds and the robot is standing.
    if let RobotCommand::Relax { yes: false, .. } = &command {
        return Err(Failure::new(
            exit::USAGE,
            "this cuts power to the joints and the robot will collapse. Re-run with --yes if              that is what you want."
                .to_owned(),
        ));
    }

    let mut client = Client::connect_to("robotd", socket)?;
    client.hello()?;

    let (call, json) = match &command {
        RobotCommand::Init { json } => (proto::Call::RobotInit, *json),
        RobotCommand::Relax { json, .. } => (proto::Call::RobotRelax, *json),
    };

    let result = result_of(client.call(&call)?)?;
    if json {
        println!("{}", compact(&result));
        return Ok(());
    }

    // An intent is a successful call that may report a refusal, and the exit code has to tell them
    // apart: a fallen robot refusing to stand up is not the same as a robot that could not be asked.
    let outcome: proto::IntentResult = decode(&result)?;
    if !outcome.accepted {
        let reason = outcome
            .reason
            .unwrap_or_else(|| "the robot refused".to_owned());
        return Err(Failure::new(exit::REFUSED, reason));
    }
    match command {
        RobotCommand::Init { .. } => println!("standing up — about two seconds to the home pose"),
        RobotCommand::Relax { .. } => println!("torque off"),
    }
    Ok(())
}

/// The gamepad, through `configd`.
///
/// `pair` is the only command here that takes a while — discovery is held open while someone holds
/// the pad's sync button — and it stays a single blocking call rather than a progress stream: there
/// is exactly one thing to report, and it arrives at the end.
fn run_pad(socket: &Path, command: PadCommand) -> Result<(), Failure> {
    let mut client = Client::connect_to("configd", socket)?;
    client.hello()?;

    let (call, json) = match &command {
        PadCommand::Status { json } => (proto::Call::PadStatus, *json),
        PadCommand::Pair { mac, timeout, json } => (
            proto::Call::PadPair(proto::PadPairParams {
                mac: mac.clone(),
                timeout_seconds: *timeout,
            }),
            *json,
        ),
        PadCommand::Forget { mac, json } => (
            proto::Call::PadForget(proto::PadForgetParams { mac: mac.clone() }),
            *json,
        ),
    };

    if let PadCommand::Pair { json: false, .. } = &command {
        // Printed before the call, not after: the call blocks for the whole discovery window, and
        // someone who ran this needs to know *now* that they should be holding the button.
        eprintln!(
            "looking for a gamepad in pairing mode — on an Xbox pad, press the small Sync \
             button on the top edge (not the Xbox button, which switches it off)"
        );
    }

    let result = result_of(client.call(&call)?)?;
    if json {
        println!("{}", compact(&result));
        return Ok(());
    }

    match command {
        PadCommand::Status { .. } => println!("{}", render_pad_status(&result)?),
        PadCommand::Pair { .. } => return report_pair(&result),
        PadCommand::Forget { mac, .. } => {
            let forgotten: proto::PadForgetResult = decode(&result)?;
            if forgotten.removed {
                println!("forgot {mac}");
                // Said every time, because "forgot" sounds like a clean slate and is not: a bond has
                // two halves and this removes one. A pad still holding its half refuses to pair
                // again — reporting `AuthenticationFailed` — until it is put back into pairing mode.
                println!(
                    "The pad still has its half of the bond. Press Sync before pairing it again."
                );
            } else {
                // Not an error, same contract as `net forget`: asking twice must not look like a
                // failure.
                println!("{mac} was not paired");
            }
        }
    }
    Ok(())
}

fn render_pad_status(result: &serde_json::Value) -> Result<String, Failure> {
    use std::fmt::Write;
    let status: proto::PadStatusResult = decode(result)?;

    let mut out = String::new();
    if status.pads.is_empty() {
        let _ = writeln!(out, "pad     none paired — run:  sudo robotctl pad pair");
    }
    for pad in &status.pads {
        // Four states, because they fail differently — and *trusted* is reported even when the pad
        // is connected, which is the case that most looks like everything is fine: it drives now and
        // does not come back after a reboot, because approving a reconnection needs an agent and at
        // boot there is none.
        let state = match (pad.connected, pad.trusted) {
            (true, true) => "connected".to_owned(),
            (true, false) => "connected, but NOT trusted — it will not reconnect after a reboot; \
                              re-run:  sudo robotctl pad pair"
                .to_owned(),
            (false, true) => "paired, not connected — switch the pad on".to_owned(),
            (false, false) => "paired but NOT trusted — re-run:  sudo robotctl pad pair".to_owned(),
        };
        let _ = writeln!(out, "pad     {} {}  {}", pad.name, pad.mac, state);
    }

    let driver = match status.driver {
        proto::UnitState::Active => "active — driving whatever pad connects".to_owned(),
        proto::UnitState::Inactive => {
            "NOT running — start it:  sudo systemctl start padd".to_owned()
        }
        proto::UnitState::Absent => "not installed — this release predates padd.service".to_owned(),
        proto::UnitState::Unknown => "unknown — could not ask systemd".to_owned(),
    };
    let _ = write!(out, "padd    {driver}");
    Ok(out)
}

/// Pairing is a successful call that may report a failed outcome, like a wifi join: the exit status
/// has to distinguish "the robot refused" from "the robot could not be asked".
fn report_pair(result: &serde_json::Value) -> Result<(), Failure> {
    match decode::<proto::PadPairResult>(result)? {
        proto::PadPairResult::Paired { pad } => {
            println!("paired  {} {}", pad.name, pad.mac);
            if pad.connected {
                println!("padd is driving from it now.");
            } else {
                // Bonded but not connected yet, which is normal for a second or two. Saying so
                // beats printing nothing where "it works" should be.
                println!("bonded; it will connect on its own in a moment.");
            }
            Ok(())
        }
        proto::PadPairResult::Failed { reason, detail } => {
            let advice = match reason {
                proto::PadPairFailure::NotFound => {
                    "no gamepad in pairing mode. On an Xbox pad: switch it on with a short press \
                     of the Xbox button, then press the small Sync button on the top edge next to \
                     the USB-C port until the Xbox light flashes quickly — holding the Xbox button \
                     switches the controller off instead. Then try again"
                }
                proto::PadPairFailure::Ambiguous => {
                    "more than one pad is in pairing mode; name the one you want by its address"
                }
                proto::PadPairFailure::Timeout => {
                    "the pad was found but never finished pairing; try again"
                }
                proto::PadPairFailure::NoAdapter => {
                    "this robot has no Bluetooth adapter yet. Just after a boot that is normal — \
                     hci0 appears about 73s in"
                }
                proto::PadPairFailure::Rejected => {
                    "Bluetooth refused the pairing. If this fails every time, check \
                     /etc/bluetooth/main.conf: the `Privacy` setting decides whether a pad can \
                     bond at all, and it should read `Privacy = device`. Fix it with \
                     scripts/setup-board.sh and reboot, since it does not apply until then. \
                     Otherwise the pad had probably left pairing mode: press Sync again and \
                     re-run this while its light is flashing quickly"
                }
                proto::PadPairFailure::Other => "pairing failed",
            };
            let detail = detail.map(|d| format!("\n{d}")).unwrap_or_default();
            Err(Failure::new(exit::REFUSED, format!("{advice}{detail}")))
        }
    }
}

fn render_net_status(result: &serde_json::Value) -> Result<String, Failure> {
    use std::fmt::Write;
    let status: proto::NetStatusResult = decode(result)?;

    let mut out = String::new();
    let _ = writeln!(out, "state   {:?}", status.state);
    if let Some(ssid) = &status.ssid {
        let signal = status
            .signal
            .map(|s| format!("  ({s}%)"))
            .unwrap_or_default();
        let _ = writeln!(out, "ssid    {ssid}{signal}");
    }
    if let Some(ip4) = &status.ip4 {
        let _ = writeln!(out, "ipv4    {ip4}");
    }
    if let Some(ip6) = &status.ip6 {
        let _ = writeln!(out, "ipv6    {ip6}");
    }
    if let Some(iface) = &status.iface {
        let mac = status.mac.as_deref().unwrap_or("unknown");
        let _ = writeln!(out, "iface   {iface}  {mac}");
    }

    // The one state worth explaining, because it is a provisioning mistake rather than a
    // network problem and the fix is a different script entirely.
    if status.state == proto::NetState::Unavailable {
        let _ = write!(
            out,
            "\nNetworkManager manages no wifi device. If this board still runs netplan, \n\
             run scripts/migrate-network.sh first."
        );
    }
    Ok(out.trim_end().to_owned())
}

fn render_scan(result: &serde_json::Value) -> Result<String, Failure> {
    use std::fmt::Write;
    let scan: proto::NetScanResult = decode(result)?;

    if scan.networks.is_empty() {
        return Ok("no networks in range".to_owned());
    }
    let mut out = String::new();
    for network in &scan.networks {
        let saved = if network.saved { " (saved)" } else { "" };
        let _ = writeln!(
            out,
            "{:>3}%  {:<12} {}{}",
            network.signal,
            format!("{:?}", network.security),
            network.ssid,
            saved
        );
    }
    Ok(out.trim_end().to_owned())
}

/// A join is a successful call that may report a failed outcome, and the difference matters:
/// the exit status has to distinguish "the robot refused" from "the robot could not be asked",
/// and a wrong passphrase is the case a script most wants to detect.
fn report_connect(result: &serde_json::Value) -> Result<(), Failure> {
    match decode::<proto::ConnectResult>(result)? {
        proto::ConnectResult::Connected { ssid, ip4 } => {
            println!("connected to {ssid}");
            if let Some(ip4) = ip4 {
                println!("ipv4      {ip4}");
            } else {
                // Associated but no address yet: DHCP is still running, and saying so is better
                // than printing nothing where an address should be.
                println!("ipv4      pending (DHCP)");
            }
            Ok(())
        }
        proto::ConnectResult::Failed { reason, detail } => {
            let advice = match reason {
                proto::ConnectFailure::BadKey => "the passphrase was rejected",
                proto::ConnectFailure::NotFound => "that network is not in range",
                proto::ConnectFailure::Timeout => "it associated but never finished; try again",
                proto::ConnectFailure::Unsupported => "this network needs something we cannot do",
                proto::ConnectFailure::Other => "the join failed",
            };
            let detail = detail.map(|d| format!(" ({d})")).unwrap_or_default();
            Err(Failure::new(exit::REFUSED, format!("{advice}{detail}")))
        }
    }
}

/// Read a passphrase from stdin, so it never appears in the process list.
fn read_secret() -> Result<String, Failure> {
    use std::io::BufRead;
    let mut line = String::new();
    std::io::stdin()
        .lock()
        .read_line(&mut line)
        .map_err(|e| Failure::new(exit::USAGE, format!("cannot read the passphrase: {e}")))?;
    // Only the trailing newline is stripped: a passphrase may legitimately end in a space, and
    // trimming one would produce a wrong-password failure nobody could explain.
    Ok(line.trim_end_matches(['\n', '\r']).to_owned())
}

fn format_uptime(seconds: u64) -> String {
    let (days, hours, minutes) = (
        seconds / 86400,
        (seconds % 86400) / 3600,
        (seconds % 3600) / 60,
    );
    if days > 0 {
        format!("{days}d {hours}h {minutes}m")
    } else if hours > 0 {
        format!("{hours}h {minutes}m")
    } else {
        format!("{minutes}m")
    }
}

/// Decode a result, naming the disagreement rather than reporting a serde error.
///
/// A shape mismatch here means `robotctl` and the daemon were built from different revisions,
/// which the `hello` handshake is meant to catch — so if it happens, saying so is far more use
/// than "missing field `state`".
fn decode<T: for<'de> serde::Deserialize<'de>>(value: &serde_json::Value) -> Result<T, Failure> {
    serde_json::from_value(value.clone()).map_err(|e| {
        Failure::new(
            exit::FAILED,
            format!(
                "cannot read the daemon's answer ({e}). Are robotctl and configd the same build?"
            ),
        )
    })
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

/// Which build `apply` should move to, from the three flags that can name one.
///
/// Its own function so the tests exercise this decision rather than a copy of it. clap already
/// refuses `--ref` beside either of the others, so the only pair reaching here together is
/// `--staging --version`, which names one candidate rather than the newest.
fn apply_target(
    staging: bool,
    version: Option<&semver::Version>,
    git_ref: Option<&str>,
) -> proto::Target {
    match (staging, version, git_ref) {
        (true, Some(version), _) => proto::Target::StagingExact(version.clone()),
        (true, None, _) => proto::Target::Staging,
        (false, Some(version), _) => proto::Target::Exact(version.clone()),
        (false, None, Some(git_ref)) => proto::Target::Ref(git_ref.to_owned()),
        (false, None, None) => proto::Target::Latest,
    }
}

/// Absolutise `--from` here, in the client, before it goes on the wire.
///
/// Two reasons, and the first one bites silently. `updaterd` runs with `/` as its working
/// directory, so a relative path means something different at each end: `--from dist` would
/// arrive as `/dist` and be reported as missing on a robot where it is sitting right there.
/// The second is that a typo should fail against the operator's own shell, with the path they
/// typed, rather than as an I/O error from the source layer three steps into an apply.
fn resolve_from_dir(dir: &std::path::Path) -> Result<String, Failure> {
    let resolved = dir
        .canonicalize()
        .map_err(|e| Failure::new(exit::USAGE, format!("--from {}: {e}", dir.display())))?;
    if !resolved.is_dir() {
        return Err(Failure::new(
            exit::USAGE,
            format!("--from {} is not a directory", resolved.display()),
        ));
    }
    // Lossy is unreachable in practice and honest about the wire: JSON carries text, so a
    // path that is not UTF-8 could not be sent whatever type this returned.
    Ok(resolved.to_string_lossy().into_owned())
}

fn run(cli: Cli) -> Result<(), Failure> {
    let command = match cli.namespace {
        Namespace::Health { json } => {
            return run_health(&cli.socket, &cli.robot_socket, &cli.config_socket, json);
        }
        Namespace::Version { json } => {
            return run_version(&cli.socket, &cli.robot_socket, &cli.config_socket, json);
        }
        Namespace::Monitor { hz, json } => {
            return monitor::run(&cli.robot_socket, &cli.pad_socket, hz, json);
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
        Namespace::Net { command } => {
            return run_net(&cli.config_socket, command);
        }
        Namespace::System { command } => {
            return run_system(&cli.config_socket, command);
        }
        Namespace::Pad { command } => {
            return run_pad(&cli.config_socket, command);
        }
        Namespace::Robot { command } => {
            return run_robot(&cli.robot_socket, command);
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
            staging,
            from,
            dry_run,
            interrupt_sessions,
        } => proto::Call::Apply(proto::ApplyParams {
            component: proto::ComponentId::new(name),
            target: apply_target(*staging, version.as_ref(), git_ref.as_deref()),
            options: proto::ApplyOptions {
                dry_run: *dry_run,
                interrupt_sessions: *interrupt_sessions,
                from_dir: from.as_deref().map(resolve_from_dir).transpose()?,
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

    /// A refused socket must not be reported as a stopped daemon.
    ///
    /// This is the first thing a freshly provisioned board says to its operator, and it was
    /// saying the wrong thing: `robotctl health` on a working install returns `EACCES` until
    /// the installing user's new `robot` group reaches a new login session, and the advice was
    /// `systemctl status`, which shows an active daemon and explains nothing.
    #[test]
    fn permission_denied_names_the_group_not_the_service() {
        let out = unreachable_hint(
            "robotd",
            std::path::Path::new("/run/robotd.sock"),
            &std::io::Error::from(std::io::ErrorKind::PermissionDenied),
        );

        assert!(
            out.starts_with("cannot reach robotd at /run/robotd.sock: "),
            "{out}"
        );
        assert!(out.contains("robot"), "{out}");
        assert!(out.contains("log"), "{out}");
        // The whole point: do not send them to check a service that is already running.
        assert!(!out.contains("systemctl status"), "{out}");
    }

    /// The other kinds keep pointing at the service, which for them is the right question.
    #[test]
    fn absent_and_refused_sockets_still_point_at_the_service() {
        for kind in [
            std::io::ErrorKind::NotFound,
            std::io::ErrorKind::ConnectionRefused,
            std::io::ErrorKind::TimedOut,
        ] {
            let out = unreachable_hint(
                "updaterd",
                std::path::Path::new("/run/updaterd.sock"),
                &std::io::Error::from(kind),
            );
            assert!(
                out.contains("systemctl status updaterd"),
                "{kind:?} lost the service hint: {out}"
            );
        }
    }

    /// Every hint must keep the cause on the first line, because `health` renders that line
    /// into a fixed-width column and pushes the rest into its warnings block. A hint that put
    /// its detail first would be truncated into nonsense there.
    #[test]
    fn every_hint_leads_with_the_cause() {
        for kind in [
            std::io::ErrorKind::PermissionDenied,
            std::io::ErrorKind::NotFound,
            std::io::ErrorKind::ConnectionRefused,
            std::io::ErrorKind::TimedOut,
        ] {
            let out = unreachable_hint(
                "robotd",
                std::path::Path::new("/run/robotd.sock"),
                &std::io::Error::from(kind),
            );
            let first = out.lines().next().unwrap_or_default();
            assert!(
                first.starts_with("cannot reach robotd at /run/robotd.sock: "),
                "{kind:?}: {first}"
            );
            assert!(out.lines().count() > 1, "{kind:?} has no advice: {out}");
        }
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

    /// `--staging` alone means the newest candidate; with `--version`, that one candidate.
    ///
    /// The pair is the only combination clap allows through, so the match that turns these
    /// into a `Target` has one case that cannot be reached by argument parsing — asserted
    /// here rather than trusted, because getting it backwards would install a *stable*
    /// release while reporting that it installed a candidate.
    #[test]
    fn staging_selects_the_candidate_channel() {
        let target = |args: &[&str]| {
            let mut argv = vec!["robotctl", "update", "apply", "daemon"];
            argv.extend_from_slice(args);
            let cli = Cli::try_parse_from(argv).expect("must parse");
            let Namespace::Update {
                command:
                    UpdateCommand::Apply {
                        version,
                        git_ref,
                        staging,
                        ..
                    },
            } = cli.namespace
            else {
                panic!("expected update apply");
            };
            apply_target(staging, version.as_ref(), git_ref.as_deref())
        };

        assert_eq!(target(&["--staging"]), proto::Target::Staging);
        assert_eq!(
            target(&["--staging", "--version", "0.3.0"]),
            proto::Target::StagingExact(semver::Version::new(0, 3, 0))
        );
        assert_eq!(target(&[]), proto::Target::Latest);
        assert_eq!(
            target(&["--version", "0.3.0"]),
            proto::Target::Exact(semver::Version::new(0, 3, 0))
        );
    }

    /// `--from` parses, and pairs with `--version` and `--ref`.
    ///
    /// The pairing is the point: the directory says *where* to read, the target says *which*
    /// release in it, and `dev-push.sh` names a version because a directory it has just
    /// rsynced holds exactly one.
    #[test]
    fn apply_from_parses_with_a_target() {
        let parse = |args: &[&str]| {
            let mut argv = vec!["robotctl", "update", "apply", "daemon"];
            argv.extend_from_slice(args);
            let cli = Cli::try_parse_from(argv).expect("must parse");
            let Namespace::Update {
                command:
                    UpdateCommand::Apply {
                        from,
                        version,
                        git_ref,
                        ..
                    },
            } = cli.namespace
            else {
                panic!("expected update apply");
            };
            (from, version, git_ref)
        };

        let (from, version, _) = parse(&["--from", "/var/tmp/duck-sideload"]);
        assert_eq!(from.as_deref(), Some(Path::new("/var/tmp/duck-sideload")));
        assert!(version.is_none(), "no version means the newest in the dir");

        let (from, version, _) = parse(&["--from", "/var/tmp/duck-sideload", "--version", "0.3.0"]);
        assert!(from.is_some());
        assert_eq!(version, Some(semver::Version::new(0, 3, 0)));

        let (from, _, git_ref) = parse(&["--from", "/var/tmp/duck-sideload", "--ref", "my-branch"]);
        assert!(from.is_some());
        assert_eq!(
            git_ref.as_deref(),
            Some("my-branch"),
            "a local_dir resolves a ref too — it becomes a filename there"
        );
    }

    /// A directory has no channels, so `--from --staging` cannot mean anything.
    #[test]
    fn from_and_staging_are_refused_together() {
        assert!(
            Cli::try_parse_from([
                "robotctl",
                "update",
                "apply",
                "daemon",
                "--from",
                "/var/tmp/duck-sideload",
                "--staging",
            ])
            .is_err(),
            "--from with --staging must be refused"
        );
    }

    /// A path that is not there fails as bad usage, against the shell that typed it.
    ///
    /// `updaterd` would also refuse it, but three steps into an apply and with the path as it
    /// looked from `/` — which is the other half of what this function does. A relative
    /// `--from dist` must not reach the daemon as `/dist`.
    #[test]
    fn from_dir_is_resolved_and_checked_locally() {
        // No `tempfile`: this crate carries no dev-dependencies, for the same reason it
        // carries so few dependencies.
        let dir = std::env::temp_dir().join(format!("robotctl-from-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();

        let Ok(resolved) = resolve_from_dir(&dir) else {
            panic!("an existing directory must resolve");
        };
        assert!(Path::new(&resolved).is_absolute());

        let file = dir.join("manifest.json");
        std::fs::write(&file, b"{}").unwrap();
        let Err(err) = resolve_from_dir(&file) else {
            panic!("a file is not a directory");
        };
        assert_eq!(err.code, exit::USAGE);

        let Err(err) = resolve_from_dir(&dir.join("nope")) else {
            panic!("a missing path must fail");
        };
        assert_eq!(err.code, exit::USAGE);
        assert!(err.message.contains("nope"), "{}", err.message);

        std::fs::remove_dir_all(&dir).unwrap();
    }

    /// A candidate and a branch build are different streams under different keys, so asking
    /// for both is a mistake to report rather than one to resolve by precedence.
    #[test]
    fn staging_and_ref_are_refused_together() {
        assert!(
            Cli::try_parse_from([
                "robotctl",
                "update",
                "apply",
                "daemon",
                "--staging",
                "--ref",
                "my-branch",
            ])
            .is_err(),
            "--staging with --ref must be refused"
        );
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
                    consecutive_stale_blocks: 0,
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
                    consecutive_stale_blocks: 0,
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
    /// reports a fault, while the orientation feeding the policy is frozen. What earns the
    /// alarm is the *run*: 41 reads without a refresh is most of a second of a robot balancing
    /// on an orientation that is no longer being measured.
    #[test]
    fn health_renders_a_broken_bus_and_a_frozen_imu() {
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
                    consecutive_stale_blocks: 41,
                }),
                ..Default::default()
            }),
            None,
        ));

        assert!(out.contains("7 consecutive read failures"), "{out}");
        assert!(out.contains("orientation frozen"), "{out}");
        assert!(out.contains("41 stale reads running"), "{out}");
    }

    /// A handful of stale reads over a long run is a healthy board, and must not wear an alarm.
    ///
    /// This is the case the old rendering got wrong: any non-zero count said "orientation may be
    /// dead", so a robot that had hiccuped nine times in 40 minutes looked broken for its whole
    /// uptime — and a warning that shows up on healthy robots stops being read at all. The rate
    /// is what carries the meaning here, so the count has to appear scaled by the reads it is
    /// drawn from.
    #[test]
    fn health_renders_sporadic_stale_reads_as_a_rate_without_alarm() {
        let out = render_health(&health_report(
            Some(proto::HealthResult {
                healthy: true,
                control_loop: Some(proto::LoopHealth {
                    target_hz: 50.0,
                    achieved_hz: Some(50.1),
                    ticks: 118_631,
                    missed: 137,
                    last_tick_age_ms: 12,
                }),
                imu: Some(proto::ImuHealth {
                    ready: true,
                    stale_blocks: 9,
                    // The board refreshed on the most recent read, so nothing is frozen.
                    consecutive_stale_blocks: 0,
                }),
                ..Default::default()
            }),
            None,
        ));

        assert!(out.contains("robot     healthy"), "{out}");
        assert!(out.contains("9 stale reads — 1 in 13181"), "{out}");
        assert!(!out.contains("frozen"), "{out}");
        assert!(!out.contains("dead"), "{out}");
    }

    /// A run below the threshold is still not an alarm — one repeated block is ordinary, and the
    /// robot must not be described as frozen for being sampled mid-hiccup.
    #[test]
    fn health_does_not_call_a_single_repeat_frozen() {
        let out = render_health(&health_report(
            Some(proto::HealthResult {
                healthy: true,
                control_loop: Some(proto::LoopHealth {
                    target_hz: 50.0,
                    achieved_hz: Some(50.0),
                    ticks: 1000,
                    missed: 0,
                    last_tick_age_ms: 10,
                }),
                imu: Some(proto::ImuHealth {
                    ready: true,
                    stale_blocks: 2,
                    consecutive_stale_blocks: 1,
                }),
                ..Default::default()
            }),
            None,
        ));

        assert!(out.contains("2 stale reads — 1 in 500"), "{out}");
        assert!(!out.contains("frozen"), "{out}");
    }

    /// Before the loop has reported any ticks there is nothing to divide by, and a rate would be
    /// a division by zero. The count still has to appear — this is also the shape a fake or a
    /// future backend produces if it counts stale reads without a loop behind them.
    #[test]
    fn health_renders_stale_reads_with_no_ticks_to_scale_against() {
        let out = render_health(&health_report(
            Some(proto::HealthResult {
                healthy: true,
                imu: Some(proto::ImuHealth {
                    ready: true,
                    stale_blocks: 3,
                    consecutive_stale_blocks: 0,
                }),
                ..Default::default()
            }),
            None,
        ));

        assert!(out.contains("imu       ready, 3 stale reads"), "{out}");
        assert!(!out.contains("1 in"), "{out}");
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

    /// The installed release stays in the `software` block, below the daemons and above `units`.
    ///
    /// The component lines carry no heading of their own, so whatever block is printed before them
    /// takes them: with `units` in between, `daemon    0.2.0 installed` renders under it and reads
    /// as a sixth systemd unit called `daemon`. Ordering is the whole of the fix, which is exactly
    /// the kind of thing a later edit undoes without noticing.
    #[test]
    fn the_installed_release_is_not_rendered_as_a_unit() {
        let mut report = health_report(Some(proto::HealthResult::default()), None);
        report.software.units = vec![
            unit("robotd", proto::UnitState::Active, Some("0.2.0")),
            unit("padd", proto::UnitState::Inactive, None),
        ];

        let out = render_health(&report);
        let installed = out.find("daemon    0.2.0 installed").expect(&out);
        let units = out.find("\nunits\n").expect(&out);
        assert!(installed < units, "{out}");
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
            units: Vec::new(),
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

    /// A unit whose process published an identity, as one installed from a release would.
    fn unit(name: &str, state: proto::UnitState, release: Option<&str>) -> proto::ServiceUnit {
        proto::ServiceUnit {
            unit: format!("{name}.service"),
            state,
            identity: release.map(|v| proto::Identity {
                service: name.to_owned(),
                version: "0.4.0".to_owned(),
                revision: Some("7610e6e19f151949e685bdd56783e564a72991e6".to_owned()),
                built_at: None,
                exe: Some(format!("/opt/robot/daemon/releases/{v}/bin/{name}")),
                pid: 4242,
            }),
        }
    }

    /// **The question this whole path exists for.** After an update, `btd` serves no socket, so
    /// nothing could say whether the process running was the new binary or the old one — and the
    /// crate version cannot tell them apart on the dev channel, where both read `0.4.0`.
    ///
    /// The warning has to name the restart, because that is the fix and it is one command.
    #[test]
    fn a_unit_still_running_the_old_release_is_named() {
        let units = vec![unit(
            "btd",
            proto::UnitState::Active,
            Some("0.4.0-dev.271.7610e6e"),
        )];
        let installed = semver::Version::parse("0.4.0").unwrap();
        let warnings = unit_warnings(&units, &[], Some(&installed));

        assert_eq!(warnings.len(), 1, "{warnings:?}");
        assert!(
            warnings[0].contains("btd is running 0.4.0-dev.271.7610e6e"),
            "{warnings:?}"
        );
        assert!(
            warnings[0].contains("systemctl restart btd"),
            "{warnings:?}"
        );
    }

    /// A daemon that answers its own socket must not be warned about from `/proc` as well. It
    /// already has a warning worded for its own case — `updaterd`'s says the lag is expected,
    /// because it cannot restart itself — and two warnings about one problem read as two problems.
    #[test]
    fn a_daemon_with_a_socket_is_not_warned_about_twice() {
        let units = vec![unit("robotd", proto::UnitState::Active, Some("0.3.0"))];
        let installed = semver::Version::parse("0.4.0").unwrap();
        let reported = vec![service("robotd", "0.3.0")];

        assert!(unit_warnings(&units, &reported, Some(&installed)).is_empty());
        // …and with no socket answer for it, the same disagreement does warn.
        assert_eq!(unit_warnings(&units, &[], Some(&installed)).len(), 1);
    }

    /// A stopped unit is printed, never warned about: a robot whose owner has no gamepad and
    /// disabled `padd` should not be scolded on every health check. The line in the unit block is
    /// the report; a warning is for what nobody chose.
    #[test]
    fn a_stopped_unit_is_shown_but_not_warned_about() {
        let units = vec![unit("padd", proto::UnitState::Inactive, None)];
        let installed = semver::Version::parse("0.4.0").unwrap();

        assert!(unit_warnings(&units, &[], Some(&installed)).is_empty());
        assert!(
            render_units(&units, 2).contains("padd      stopped"),
            "{}",
            render_units(&units, 2)
        );
    }

    /// **A daemon too old to publish an identity, which is the case that made this design possible.**
    /// It is allowed to simply not answer: saying `unknown (old)` beats inferring a version from
    /// somewhere else and presenting the guess as fact — and it must not read as a disagreement,
    /// because there is no version to disagree with.
    #[test]
    fn a_daemon_that_published_nothing_says_so() {
        let silent = proto::ServiceUnit {
            unit: "btd.service".into(),
            state: proto::UnitState::Active,
            identity: None,
        };
        let installed = semver::Version::parse("0.4.0").unwrap();

        let rendered = render_units(std::slice::from_ref(&silent), 2);
        assert!(rendered.contains("build unknown (old)"), "{rendered}");
        assert!(unit_warnings(&[silent], &[], Some(&installed)).is_empty());
    }

    /// A revision is what distinguishes two builds of one version, so the line has to carry it —
    /// abbreviated, because forty characters of SHA in a column is noise around the seven anyone
    /// compares.
    #[test]
    fn the_revision_is_shown_beside_the_release() {
        let rendered = render_units(&[unit("btd", proto::UnitState::Active, Some("0.4.0"))], 2);
        assert!(rendered.contains("0.4.0 (rev 7610e6e)"), "{rendered}");
    }

    /// A hand-built binary is normal on a dev board and is not a release. It publishes an identity
    /// like anything else, and reporting its crate version as a release would invent a fact — so the
    /// line says the build is not a release, and it must not warn.
    #[test]
    fn a_binary_outside_the_release_layout_is_named_not_guessed() {
        let hand_built = proto::ServiceUnit {
            unit: "btd.service".into(),
            state: proto::UnitState::Active,
            identity: Some(proto::Identity {
                service: "btd".into(),
                version: "0.4.0".into(),
                revision: None,
                built_at: None,
                exe: Some("/home/pierre/duck/target/debug/btd".into()),
                pid: 4242,
            }),
        };
        let installed = semver::Version::parse("0.4.0").unwrap();

        let rendered = render_units(std::slice::from_ref(&hand_built), 2);
        assert!(rendered.contains("not a release"), "{rendered}");
        assert!(unit_warnings(&[hand_built], &[], Some(&installed)).is_empty());
    }

    /// Nothing to compare against is not a disagreement. A robot whose `updaterd` is down reports
    /// no installed release, and inventing a mismatch there would accuse a healthy install.
    #[test]
    fn no_installed_release_means_no_verdict() {
        let units = vec![unit("btd", proto::UnitState::Active, Some("0.3.0"))];
        assert!(unit_warnings(&units, &[], None).is_empty());
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

    /// The whole point of the command: for a few seconds after an update, `updaterd` is still
    /// running the old binary. Support must be told, and told that it is expected — otherwise the
    /// obvious reading is "the update did not work" and someone starts undoing a working robot.
    ///
    /// The wording is pinned because it went stale once already: it promised the old binary would
    /// last "until the next reboot", written before the engine began scheduling that restart
    /// itself. Advice that outlives the mechanism it describes sends someone to reboot a robot that
    /// had already fixed itself.
    #[test]
    fn a_running_updaterd_behind_the_installed_release_is_flagged_and_explained() {
        let r = report(vec![service("updaterd", "0.1.0")], Some("0.2.0"));
        let warnings = version_warnings(&r, Some(&semver::Version::new(0, 1, 0)));

        assert_eq!(warnings.len(), 1, "{warnings:?}");
        let warning = &warnings[0];
        assert!(warning.contains("running 0.1.0"), "{warning}");
        assert!(warning.contains("0.2.0"), "{warning}");
        assert!(
            warning.contains("cannot restart itself"),
            "must explain why this is expected, not merely report it: {warning}"
        );
        assert!(
            warning.contains("5s after it replies"),
            "must name the mechanism that resolves it, since it resolves itself: {warning}"
        );
        assert!(
            warning.contains("still true a minute later"),
            "must say when it stops being expected and becomes a fault: {warning}"
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

    /// The false positive this shipped with, found on a board within a minute of `configd`
    /// starting correctly.
    ///
    /// On the dev channel a daemon reports its crate version (`0.2.0`) while the installed
    /// release is named `0.2.0-dev.121.de58259`. Comparing those *strings* says "behind" about a
    /// binary that is precisely the one installed, so `robotctl version` warned every single
    /// dev-channel install. Comparing the revision — which `is_behind` does when both are known —
    /// is the answer, and it is what updaterd and robotd already did.
    #[test]
    fn a_dev_release_does_not_make_configd_look_behind() {
        let mut r = report(
            vec![
                service_at("updaterd", "0.2.0", "de58259"),
                service_at("robotd", "0.2.0", "de58259"),
                service_at("configd", "0.2.0", "de58259"),
            ],
            Some("0.2.0-dev.121.de58259"),
        );
        // The installed release carries the same revision the daemons report, which is the whole
        // point: same commit, differently spelled version.
        r.components[0].revision = Some("de58259".into());

        let warnings = version_warnings(
            &r,
            Some(&semver::Version::parse("0.2.0-dev.121.de58259").unwrap()),
        );
        assert!(
            warnings.is_empty(),
            "a dev release must not read as every daemon being behind: {warnings:?}"
        );
    }

    /// And the real case still warns: same version string, different commit, which is exactly
    /// what "the restart did not take effect" looks like on the dev channel.
    #[test]
    fn a_configd_on_another_revision_still_warns() {
        let mut r = report(
            vec![
                service_at("updaterd", "0.2.0", "de58259"),
                service_at("robotd", "0.2.0", "de58259"),
                service_at("configd", "0.2.0", "cfe436a"),
            ],
            Some("0.2.0-dev.121.de58259"),
        );
        r.components[0].revision = Some("de58259".into());

        let warnings = version_warnings(
            &r,
            Some(&semver::Version::parse("0.2.0-dev.121.de58259").unwrap()),
        );
        assert_eq!(warnings.len(), 1, "{warnings:?}");
        assert!(
            warnings[0].starts_with("configd is running"),
            "{:?}",
            warnings[0]
        );
        assert!(
            warnings[0].contains("restart set"),
            "must diagnose a failed restart, not an expected lag: {:?}",
            warnings[0]
        );
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
