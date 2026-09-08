//! `updaterd` — the update daemon.
//!
//! Deliberately thin: parse args, load config, recover from any interrupted run,
//! then serve. All logic lives in the library so it can be tested without a
//! socket or a robot.
//!
//! **Startup order matters.** `Engine::recover_on_start` runs *before* the socket
//! is served, so a robot that booted into a bad release has already begun
//! reverting by the time anything can ask it to do something else.
//!
//! This process must be resident and must exclude *itself* from the units it
//! restarts — see `docs/design/updater-design.md` §4.
//!
//! **Resident is about triggers, not about applying.** Applying an update is a library
//! call, and mutual exclusion is a file lock in `state_dir` rather than a property of
//! there being one process ([`updater::engine::Engine::apply`]). What needs a daemon is
//! everything *around* an update: a socket for the app to trigger through, progress to
//! stream back, a timer so a mandatory release can pull a robot forward with nobody
//! present, and a process at boot for the boot counter to recover through. None of that
//! applies to a robot's first install, which is what the `install` subcommand is for.

use std::path::PathBuf;
use std::process::ExitCode;

use clap::{Parser, Subcommand};

const DEFAULT_CONFIG: &str = "/etc/robot/updater.toml";

#[derive(Parser, Debug)]
#[command(name = "updaterd", about = "Robot update daemon", version)]
struct Args {
    /// Path to updater.toml.
    #[arg(long, global = true, default_value = DEFAULT_CONFIG)]
    config: PathBuf,

    /// Socket to listen on.
    #[arg(long, global = true, default_value = updater::proto::DEFAULT_SOCKET)]
    socket: PathBuf,

    /// `robotd`'s socket, used for the safe-to-restart and health probes.
    ///
    /// Absent or silent is fine — the engine treats an unreachable `robotd` as a
    /// normal state (`docs/design/architecture.md` §1.1).
    /// Overrides `robot_socket` in the config. For running two updaterd instances on one
    /// dev box, or pointing at a stub robotd; a real deployment sets it in the config.
    #[arg(long, global = true)]
    robot_socket: Option<PathBuf>,

    /// Run boot recovery, then exit without serving.
    ///
    /// Note this **performs** recovery rather than reporting it: `record_boot` advances every armed
    /// trial and reverts one that is exhausted. An operator tool, not a probe — see `--self-test`
    /// for the read-only one, and `updater-design.md` §4 for why the distinction cost an
    /// investigation.
    #[arg(long, global = true)]
    check_only: bool,

    /// Load the config, construct the engine, and exit. Touches no state.
    ///
    /// The probe an update runs against the release it just swapped in, before committing to it.
    /// `updaterd` never restarts itself during an update, so a replacement binary that cannot start
    /// is otherwise discovered at the *next boot* — after the commit, with nobody watching, and with
    /// recovery living inside the process that is failing to start.
    ///
    /// Deliberately stops short of `recover_on_start`: running that mid-update would have a second
    /// engine advancing the in-flight trial's boot count against the same store, and reverting the
    /// update the first one is still performing.
    ///
    /// What it does exercise is what actually breaks: the binary loads and is the right
    /// architecture, its libraries resolve, it does not panic on startup, and — the likely one — it
    /// accepts the board's existing `updater.toml`, which belongs to the operator and is preserved
    /// across installs while a release is free to change what it expects.
    #[arg(long, global = true)]
    self_test: bool,

    /// Enable a fault injection point (repeatable). Test/bench only — refused
    /// unless the config allows it, and never set on a client robot.
    ///
    /// See [`updater::faults::Faults`] for the available points.
    #[arg(long = "inject-fault", value_name = "FAULT", global = true)]
    faults: Vec<String>,

    /// Absent means "serve", which is how systemd starts it.
    #[command(subcommand)]
    command: Option<Command>,
}

#[derive(Subcommand, Debug)]
enum Command {
    /// Install a component's first release from a local directory, then exit.
    ///
    /// How a robot that has never been updated gets the release containing the very daemon
    /// that would otherwise have to be running already. Runs the ordinary
    /// [`updater::engine::Engine::apply`], so there is no bootstrap-only path to drift from
    /// the real one.
    ///
    /// Two settings are forced for the duration, because on a robot with no release installed
    /// they are facts rather than policy:
    ///
    ///   - `on_apply` → `none`. The units live *inside* the release being installed, so
    ///     there is nothing to restart yet and `systemctl restart` would fail, failing
    ///     the update.
    ///   - `health` → `none`. A `probe = "socket"` gate asks `robotd`, which cannot be
    ///     running before its binary exists; the gate would fail and revert to nothing.
    ///     `updater/tests/apply.rs` pins that behaviour against an absent robot.
    ///
    /// Which is why this refuses to run once a release *is* live: those overrides would
    /// then silently disable auto-rollback on a working robot. Use
    /// `robotctl update apply` there — it goes through the daemon, with the real gate.
    Install {
        /// Install from a local directory instead of the configured source.
        ///
        /// Omit it on a robot with a network: the configured `github_releases` source
        /// resolves `latest` itself, which is what the one-line installer relies on —
        /// otherwise it would have to parse a signed manifest in shell to learn the
        /// version and the artifact URL.
        ///
        /// Supply it for an offline or factory install, or to sideload a build. The
        /// directory holds `<version>.manifest.json`, its `.minisig`, the artifact and
        /// the artifact's `.minisig` — the layout `updater::source::local` expects.
        #[arg(long)]
        from: Option<PathBuf>,

        /// Component to install. The default is the one that carries the daemons.
        #[arg(long, default_value = "daemon")]
        component: String,

        /// Verify everything and stop before the swap. Checks a downloaded release is
        /// installable without committing to it.
        #[arg(long)]
        dry_run: bool,

        /// Install over a release that is already live, with `robotd` stopped.
        ///
        /// For a board whose installed `updaterd` is too old to accept the release you are
        /// trying to give it — it will roll the new release back and can never install the
        /// version that fixes that. `robotctl update apply` cannot help, because the old
        /// binary is the one running the gate.
        ///
        /// Refused unless `robotd` is silent, so the reason plain `install` refuses — no
        /// health gate on a *working* robot — cannot apply. Stop it first:
        /// `systemctl stop robotd`. Signatures, hashes and compatibility are still checked;
        /// what is given up is auto-rollback for this one install.
        #[arg(long)]
        force: bool,
    },
}

/// Is a `robotd` answering the socket?
///
/// Short timeout: this asks whether anything is *there*, not whether it is well. Anything
/// other than `Unreachable` means a robot is running and must not have the release swapped
/// out from under it without a gate.
async fn robot_is_answering(robot: &dyn updater::robot::RobotClient) -> bool {
    !matches!(
        robot.health(std::time::Duration::from_secs(2)).await,
        updater::robot::Health::Unreachable
    )
}

/// A user name to a uid, and a group name to a gid — with a line in the journal either way.
///
/// `SO_PEERCRED` reports numbers, so a name has to become one somewhere. Doing it here, once at
/// startup, is what lets `deploy/updater.toml` name `btd` and stay correct on a board where
/// `systemd-sysusers` allocated a different uid.
///
/// The lookup itself is [`updater::unix`], which `account.rs` also uses for the token file's
/// group. What is left here is the logging, and that is the part worth keeping local: these two
/// lines are how you find out from the journal that a board's `allow_users` entry resolved to
/// nothing, which looks exactly like a permissions bug from the client's end.
fn resolve_uid(name: &str) -> Option<u32> {
    match updater::unix::user_id(name) {
        None => {
            tracing::warn!(
                user = name,
                "no such user; it cannot change this robot's software"
            );
            None
        }
        Some(uid) => {
            tracing::info!(user = name, uid, "may change this robot's software");
            Some(uid)
        }
    }
}

fn resolve_gid(name: &str) -> Option<u32> {
    match updater::unix::group_id(name) {
        None => {
            tracing::warn!(
                group = name,
                "no such group; it cannot change this robot's software"
            );
            None
        }
        Some(gid) => {
            tracing::info!(group = name, gid, "may change this robot's software");
            Some(gid)
        }
    }
}

#[tokio::main]
async fn main() -> ExitCode {
    let args = Args::parse();

    // Log to stderr; systemd captures it into the journal. Level via RUST_LOG.
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_writer(std::io::stderr)
        .init();

    duck_ipc_proto::log_startup_identity!("updaterd");

    match args.command {
        Some(Command::Install {
            ref from,
            ref component,
            dry_run,
            force,
        }) => install(&args, from.clone(), component, dry_run, force).await,
        None => serve(args).await,
    }
}

/// What both entry points need, loaded identically.
///
/// Split out so `serve` and `install` cannot diverge on the two things that must fail
/// loudly — a bad config and an unusable keyring. A bootstrap path that was lenient
/// about either would be a hole in the trust chain reachable exactly once per robot,
/// which is the worst possible time for it to be there.
struct Loaded {
    config: updater::config::Config,
    keys: updater::verify::KeyRing,
    robot: Box<dyn updater::robot::RobotClient>,
    faults: updater::faults::Faults,
}

/// `None` means the failure was already logged with its context.
fn load(args: &Args) -> Option<Loaded> {
    // Config and keyring first: both must fail loudly. A bad config that silently
    // left the robot unable to update would be invisible until it mattered.
    let config = match updater::config::Config::load(&args.config) {
        Ok(config) => config,
        Err(e) => {
            tracing::error!(path = %args.config.display(), error = %e, "invalid config");
            return None;
        }
    };

    // Refused unless the config permits it, so a client robot cannot be told to
    // fail on purpose.
    let faults =
        match updater::faults::Faults::from_names(&args.faults, config.allow_fault_injection) {
            Ok(faults) => faults,
            Err(e) => {
                tracing::error!(error = %e, "fault injection refused");
                return None;
            }
        };
    if faults.any_enabled() {
        tracing::warn!(
            ?faults,
            "FAULT INJECTION ACTIVE — this build will fail on purpose"
        );
    }

    // An empty trusted-keys directory is fatal, never an empty allow-list: silently
    // trusting nothing looks identical to a misconfigured path.
    let keys = match updater::verify::KeyRing::load(&config.trusted_keys_dir, config.allow_dev_keys)
    {
        Ok(keys) => keys,
        Err(e) => {
            tracing::error!(error = %e, "could not load trusted keys");
            return None;
        }
    };

    // `robotd` may well not be running — that is a normal, expected state, and the
    // client reports Unreachable rather than failing.
    let robot_socket = args
        .robot_socket
        .clone()
        .unwrap_or_else(|| config.robot_socket.clone());
    tracing::info!(path = %robot_socket.display(), "robotd socket");
    let robot: Box<dyn updater::robot::RobotClient> =
        Box::new(updater::robot::SocketRobotClient::new(robot_socket));

    Some(Loaded {
        config,
        keys,
        robot,
        faults,
    })
}

/// Install the first release from a local directory, synchronously, and exit.
///
/// See [`Command::Install`] for why this exists and why it refuses to run on a robot
/// that already has a release live.
async fn install(
    args: &Args,
    from: Option<PathBuf>,
    component: &str,
    dry_run: bool,
    force: bool,
) -> ExitCode {
    use updater::config::{ApplyAction, HealthCheck, SourceConfig};

    let Some(mut loaded) = load(args) else {
        return ExitCode::FAILURE;
    };

    // Resolved before anything else so a typo'd path fails here, rather than as a
    // confusing "no releases found" from the source layer three steps later.
    let from = match from {
        None => None,
        Some(path) => match path.canonicalize() {
            Ok(resolved) if resolved.is_dir() => Some(resolved),
            Ok(resolved) => {
                tracing::error!(path = %resolved.display(), "--from is not a directory");
                return ExitCode::FAILURE;
            }
            Err(e) => {
                tracing::error!(path = %path.display(), error = %e, "cannot read --from");
                return ExitCode::FAILURE;
            }
        },
    };

    let Some(cfg) = loaded.config.components.get_mut(component) else {
        tracing::error!(
            component,
            known = ?loaded.config.components.keys().collect::<Vec<_>>(),
            "no such component in the config"
        );
        return ExitCode::FAILURE;
    };

    // The guard that keeps the overrides below honest. Doing this through the store
    // rather than the engine keeps it to the precise question — is a release *live* —
    // so a half-finished install that left a directory but no symlink is still
    // recoverable with this command.
    let store = updater::store::Store::new(cfg.install_dir.clone());
    match store.current() {
        Ok(None) => {}
        // `--force`, and only with a silent robot. The objection to installing over a live
        // release is about a *working robot* losing auto-rollback, so the honest guard is
        // "is a robot answering", not "is a release live" — a stopped robotd cannot be
        // misled about its own health, which is the same position a bare board is in.
        Ok(Some(live)) if force => {
            if robot_is_answering(loaded.robot.as_ref()).await {
                tracing::error!(
                    component,
                    live = %live,
                    "refusing --force: robotd is still answering, so this would swap the \
                     release under a running robot with no health gate behind it. Stop it \
                     first: systemctl stop robotd"
                );
                return ExitCode::FAILURE;
            }
            tracing::warn!(
                component,
                live = %live,
                "--force: installing over a live release with robotd stopped. No health \
                 gate, so this install cannot auto-roll-back — `robotctl rollback` is the \
                 recovery path if the new release misbehaves."
            );
        }
        Ok(Some(live)) => {
            tracing::error!(
                component,
                live = %live,
                "refusing: this component already has a release live. `install` forces \
                 on_apply and health off, which on a working robot would silently disable \
                 auto-rollback. Use `robotctl update apply` — it goes through the daemon, \
                 with the real health gate. If that rolls back because the installed \
                 updaterd is too old to accept the release, stop robotd and re-run with \
                 --force."
            );
            return ExitCode::FAILURE;
        }
        Err(e) => {
            tracing::error!(component, error = %e, "cannot read the release store");
            return ExitCode::FAILURE;
        }
    }

    // Logged, not silent. These overrides are the one way this command differs from an
    // ordinary apply, so an operator reading the journal afterwards must be able to see
    // that the health gate did not run, rather than infer it.
    tracing::warn!(
        component,
        "bootstrap install: forcing on_apply=none, health=none — nothing is installed \
         yet, so there is no unit to restart and no robotd to probe"
    );
    cfg.on_apply = ApplyAction::None;
    cfg.health = HealthCheck::None;

    match &from {
        Some(path) => {
            tracing::warn!(from = %path.display(), "installing from a local directory");
            cfg.source = SourceConfig::LocalDir { path: path.clone() };
        }
        // Left as configured, which on a client robot is the network. Stated in the log
        // because "which release did this robot start life on" is a support question, and
        // the answer depends on which source answered.
        None => tracing::info!(source = ?cfg.source, "installing from the configured source"),
    }

    let mut engine =
        match updater::engine::Engine::new(loaded.config, loaded.keys, loaded.robot, loaded.faults)
        {
            Ok(engine) => engine,
            Err(e) => {
                tracing::error!(error = %e, "could not start the engine");
                return ExitCode::FAILURE;
            }
        };

    // Progress is advisory and the channel unbounded, so this cannot slow the install
    // down — it just makes a long download visible in the journal.
    //
    // Logged in deciles, not per callback. The engine emits progress per network chunk, so
    // a single 3.6 MB download produced ~250 journal lines — 13 of them at "percent=0",
    // before the first whole percent had even accrued. That is not merely noise: under
    // journald's size cap it is what evicts the logs someone actually needs, and this runs
    // on a robot whose logs may be all anyone has.
    //
    // Deciles because the point of the line is "is it moving", which ten lines answer as
    // well as two hundred and fifty.
    let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel::<updater::proto::Progress>();
    tokio::spawn(async move {
        let mut last_logged: Option<(updater::proto::Phase, u8)> = None;
        while let Some(p) = rx.recv().await {
            match p.percent {
                Some(percent) => {
                    let decile = progress_decile(percent);
                    if last_logged == Some((p.phase, decile)) {
                        continue;
                    }
                    last_logged = Some((p.phase, decile));
                    tracing::info!(phase = ?p.phase, percent, "installing");
                }
                None => {
                    last_logged = None;
                    tracing::info!(phase = ?p.phase, "installing");
                }
            }
        }
    });

    let options = updater::engine::ApplyOptions {
        dry_run,
        // Nothing can be streaming from a robot with no release on it.
        interrupt_sessions: false,
        // `install --from` builds the source itself, below: this path forces `on_apply` and the
        // health gate off, which is the difference between it and `apply --from`, and it must
        // not be reachable by an option that leaves them on.
        from_dir: None,
        // No peer to name: this is a shell running as root, not a client on the socket. What ran
        // it is worth saying anyway, because a transcript that says nothing here reads as an
        // unattended update rather than as the sideload it was.
        requested_by: Some("updaterd install --from, on this board".into()),
    };

    let result = engine
        .apply(component, updater::proto::Target::Latest, options, tx)
        .await;

    use updater::proto::ApplyResult;
    match result {
        Ok(ApplyResult::Applied { to, .. }) => {
            tracing::warn!(component, version = %to, "installed");
            ExitCode::SUCCESS
        }
        Ok(ApplyResult::DryRunPassed { candidate }) => {
            tracing::warn!(component, candidate = %candidate, "dry run passed, nothing installed");
            ExitCode::SUCCESS
        }
        // Unreachable given the store guard above, but reporting it as success would be
        // wrong if that guard ever moves.
        // `stale` is ignored rather than reported: this is a bootstrap install, before anything is
        // serving, so there is no running daemon for it to disagree with.
        Ok(ApplyResult::AlreadyCurrent { version, .. }) => {
            tracing::warn!(component, version = %version, "already current");
            ExitCode::SUCCESS
        }
        // With health and on_apply off, a rollback means a hook inside the artifact
        // failed. Worth naming, because the artifact — not the robot — is at fault.
        Ok(ApplyResult::RolledBack {
            attempted, reason, ..
        }) => {
            tracing::error!(
                component,
                attempted = %attempted,
                reason,
                "install was reverted; the release itself refused to install"
            );
            ExitCode::FAILURE
        }
        Ok(ApplyResult::Stuck { version, reason }) => {
            tracing::error!(
                component,
                version = %version,
                reason,
                "install failed with nothing to revert to; this robot needs attention"
            );
            ExitCode::FAILURE
        }
        Err(updater::Error::Busy) => {
            tracing::error!(
                "another update holds the lock — updaterd is probably already running. \
                 Stop it, or use `robotctl update apply`."
            );
            ExitCode::FAILURE
        }
        Err(e) => {
            tracing::error!(component, error = %e, "install failed");
            ExitCode::FAILURE
        }
    }
}

/// Recover, then serve the socket until told to stop. How systemd runs it.
async fn serve(args: Args) -> ExitCode {
    let Some(loaded) = load(&args) else {
        return ExitCode::FAILURE;
    };
    let (config, keys, robot, faults) = (loaded.config, loaded.keys, loaded.robot, loaded.faults);

    // Read before `config` is moved into the engine.
    let config_check_interval = config.check_interval;
    let config_auto_apply = config.auto_apply;
    // Numeric ids from the config, plus whatever the names resolve to. Names are what a shipped
    // config should use; the numeric lists remain for a bench override.
    let mut allow_uids = config.allow_uids.clone();
    allow_uids.extend(
        config
            .allow_users
            .iter()
            .filter_map(|name| resolve_uid(name)),
    );
    let mut allow_gids = config.allow_gids.clone();
    allow_gids.extend(
        config
            .allow_groups
            .iter()
            .filter_map(|name| resolve_gid(name)),
    );

    let mut engine = match updater::engine::Engine::new(config, keys, robot, faults) {
        Ok(engine) => engine,
        Err(e) => {
            tracing::error!(error = %e, "could not start the engine");
            return ExitCode::FAILURE;
        }
    };

    // Before recovery, not after: this must not touch state. See the flag's documentation.
    if args.self_test {
        tracing::info!("--self-test: config loaded and engine constructed; not serving");
        return ExitCode::SUCCESS;
    }

    // BEFORE serving. A robot that booted into a bad release has already begun
    // reverting by the time anything can ask it to do something else.
    match engine.recover_on_start().await {
        Ok(outcomes) if outcomes.is_empty() => tracing::info!("nothing to recover"),
        Ok(outcomes) => {
            for outcome in &outcomes {
                tracing::warn!(?outcome, "startup recovery");
            }
        }
        Err(e) => {
            // Not fatal: refusing to serve would remove the only way to fix it.
            tracing::error!(error = %e, "startup recovery failed; serving anyway");
        }
    }

    // After recovery, because recovery can change which release is active — checking before it
    // would compare every unit against a version this robot is in the middle of abandoning.
    //
    // This is where a deferred restart that never happened gets caught. `updaterd` cannot observe
    // its own restart, so the successor does it; see `updater::reconcile`.
    let findings = engine.reconcile_running_units().await;
    let stale = findings
        .iter()
        .filter(|f| f.verdict != updater::reconcile::Verdict::Current)
        .count();
    if stale == 0 {
        tracing::info!(
            units = findings.len(),
            "every unit is on the active release"
        );
    }

    if args.check_only {
        tracing::info!("--check-only: recovery done, not serving");
        return ExitCode::SUCCESS;
    }

    let check_interval = config_check_interval;
    let auto_apply = config_auto_apply;

    let server = std::sync::Arc::new(updater::ipc::Server::with_policy(
        engine, allow_uids, allow_gids,
    ));
    let socket = args.socket.clone();

    // Unconditional, unlike the update scheduler below: a Hugging Face token lasts thirty days
    // and its refresh token rotates, so a robot that is on for a month loses remote access
    // unless something renews it. `docs/design/remote-access-design.md` §2.7.
    server.spawn_account_maintenance();

    // The scheduler is what makes `min_supported` effective: without it the floor is
    // inert, because a robot only learns of it when someone opens the app.
    match check_interval {
        Some(interval) => {
            // `auto_apply = all` restarts the robot on the updater's schedule rather than
            // its owner's, so which policy is live belongs in the journal at a level that
            // survives `RUST_LOG=warn` — it is the first thing to check when a robot
            // restarted and nobody asked it to.
            if auto_apply == updater::config::AutoApply::All {
                tracing::warn!(
                    interval_secs = interval.as_secs(),
                    "auto_apply = all: this robot installs every available release without \
                     waiting for a client. Intended for canary and bench robots."
                );
            } else {
                tracing::info!(
                    interval_secs = interval.as_secs(),
                    ?auto_apply,
                    "periodic update checks enabled"
                );
            }
            server.spawn_periodic_checks(interval, auto_apply);
        }
        // Not just a missed convenience: with no timer, `auto_apply` is inert whatever it
        // says, so a robot configured to update itself silently does not.
        None if auto_apply != updater::config::AutoApply::Off => tracing::warn!(
            ?auto_apply,
            "auto_apply is set but there is no check_interval, so nothing will ever apply \
             it unattended; a mandatory update will only be noticed when a client asks"
        ),
        None => tracing::warn!(
            "periodic update checks are disabled (no check_interval); a mandatory update \
             will only be noticed when a client asks"
        ),
    }

    tokio::select! {
        result = std::sync::Arc::clone(&server).serve(&socket) => {
            if let Err(e) = result {
                tracing::error!(error = %e, "IPC server stopped");
                return ExitCode::FAILURE;
            }
        }
        _ = shutdown() => tracing::info!("shutting down"),
    }

    // Leaving a stale socket behind would make the next start log a warning for no
    // reason.
    let _ = std::fs::remove_file(&socket);
    ExitCode::SUCCESS
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

/// Which decile a percentage falls in, for throttling progress logs.
///
/// 100 gets its own bucket so a finished download reports 100 rather than stopping at 90.
fn progress_decile(percent: u8) -> u8 {
    if percent >= 100 { 10 } else { percent / 10 }
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::CommandFactory;
    use std::path::Path;

    #[test]
    fn args_definition_is_valid() {
        Args::command().debug_assert();
    }

    #[test]
    fn faults_are_repeatable() {
        let args = Args::try_parse_from([
            "updaterd",
            "--inject-fault",
            "fail_health",
            "--inject-fault",
            "abort_after_swap",
        ])
        .unwrap();
        assert_eq!(args.faults, ["fail_health", "abort_after_swap"]);
    }

    /// Defaults must be usable with no arguments at all — that is how systemd
    /// starts it. No subcommand means serve.
    #[test]
    fn defaults_need_no_arguments() {
        let args = Args::try_parse_from(["updaterd"]).unwrap();
        assert_eq!(args.config, PathBuf::from(DEFAULT_CONFIG));
        assert!(args.faults.is_empty());
        assert!(!args.check_only);
        assert!(
            args.command.is_none(),
            "a bare invocation must still mean serve"
        );
    }

    #[test]
    fn install_takes_a_directory_and_defaults_to_the_daemon_component() {
        let args = Args::try_parse_from(["updaterd", "install", "--from", "/var/tmp/rel"]).unwrap();
        let Some(Command::Install {
            from,
            component,
            dry_run,
            force,
        }) = args.command
        else {
            panic!("expected install, got {:?}", args.command);
        };
        assert_eq!(from.as_deref(), Some(Path::new("/var/tmp/rel")));
        assert_eq!(component, "daemon");
        assert!(!dry_run);
        assert!(!force, "a plain install must never force");
    }

    /// The one-liner installer's path: no `--from`, so the configured source resolves
    /// `latest` itself and nothing has to parse a signed manifest in shell.
    #[test]
    fn install_without_from_uses_the_configured_source() {
        let args = Args::try_parse_from(["updaterd", "install"]).unwrap();
        let Some(Command::Install { from, .. }) = args.command else {
            panic!("expected install, got {:?}", args.command);
        };
        assert!(from.is_none());
    }

    /// `--config` is global, so the bootstrap installer can point `install` at the same
    /// config the daemon will use rather than at a copy of its values. A copy is what
    /// would let the two disagree about `trusted_keys_dir` or `state_dir`.
    #[test]
    fn install_accepts_the_global_config_flag() {
        let args = Args::try_parse_from([
            "updaterd",
            "install",
            "--from",
            "/var/tmp/rel",
            "--config",
            "/etc/robot/updater.toml",
        ])
        .unwrap();
        assert_eq!(args.config, PathBuf::from("/etc/robot/updater.toml"));
        assert!(matches!(args.command, Some(Command::Install { .. })));
    }

    /// `--force` must be opt-in. Defaulting it on would let any bootstrap install swap a
    /// release under a running robot with no health gate.
    #[test]
    fn install_does_not_force_by_default() {
        let args = Args::try_parse_from(["updaterd", "install"]).unwrap();
        match args.command {
            Some(Command::Install { force, .. }) => assert!(!force),
            other => panic!("expected Install, got {other:?}"),
        }
    }

    #[test]
    fn install_accepts_force() {
        let args = Args::try_parse_from(["updaterd", "install", "--force"]).unwrap();
        match args.command {
            Some(Command::Install { force, .. }) => assert!(force),
            other => panic!("expected Install, got {other:?}"),
        }
    }

    /// A robot answering in a shape we cannot read is still a robot that is *running*.
    ///
    /// `robot_is_answering` guards `install --force`, which swaps a release with no health gate
    /// behind it. Its own doc comment already states the rule — "anything other than
    /// `Unreachable` means a robot is running and must not have the release swapped out from
    /// under it" — but before `Health::Incompatible` existed the code could not keep it: an
    /// unparseable reply became `Unreachable`, so a live robot whose health shape had drifted
    /// read as absent and `--force` proceeded. Exactly the robot least worth guessing about.
    #[tokio::test]
    async fn a_robot_answering_unreadably_still_counts_as_answering() {
        struct Unreadable;

        #[async_trait::async_trait]
        impl updater::robot::RobotClient for Unreadable {
            async fn safe_to_restart(
                &self,
                _: std::time::Duration,
            ) -> updater::robot::SafeToRestart {
                updater::robot::SafeToRestart::Unreachable
            }
            async fn health(&self, _: std::time::Duration) -> updater::robot::Health {
                updater::robot::Health::Incompatible("missing field `imu`".into())
            }
            async fn model_api(&self, _: std::time::Duration) -> Option<u32> {
                None
            }
            async fn remote_session_active(&self, _: std::time::Duration) -> bool {
                false
            }
            async fn reload_policies(&self, _: std::time::Duration) -> bool {
                false
            }
        }

        assert!(
            robot_is_answering(&Unreadable).await,
            "an unreadable answer is still an answer; --force must refuse"
        );
    }

    /// The engine emits progress once per network chunk, so a single 3.6 MB download wrote
    /// ~250 journal lines — 13 of them before the first whole percent had even accrued.
    /// That is not just noise: under journald's size cap it evicts the logs someone needs,
    /// on a robot where the journal may be all anyone has to go on.
    #[test]
    fn a_full_download_logs_eleven_lines_not_hundreds() {
        // The shape a chunked download really reports: repeated zeros, then every percent
        // more than once.
        let mut reported = vec![0u8; 13];
        for percent in 0..=100u8 {
            reported.push(percent);
            reported.push(percent);
        }
        assert!(
            reported.len() > 200,
            "the flood this throttling exists to fix"
        );

        let mut logged = 0;
        let mut last = None;
        for percent in reported {
            let decile = progress_decile(percent);
            if last != Some(decile) {
                last = Some(decile);
                logged += 1;
            }
        }
        // 0, 10, ... 90, then completion.
        assert_eq!(logged, 11, "one line per decile plus completion");
    }

    /// 100 must not share a bucket with 90, or a finished download looks stalled at 90%.
    #[test]
    fn completion_is_its_own_bucket() {
        assert_eq!(progress_decile(90), 9);
        assert_eq!(progress_decile(99), 9);
        assert_eq!(progress_decile(100), 10);
    }

    /// The start of a phase is worth exactly one line, not thirteen.
    #[test]
    fn the_leading_zeros_collapse() {
        assert_eq!(progress_decile(0), 0);
        assert_eq!(progress_decile(9), 0);
        assert_ne!(progress_decile(10), progress_decile(9));
    }
}
