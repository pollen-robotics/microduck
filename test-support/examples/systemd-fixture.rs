//! Mint releases that carry systemd units and a real `updaterd`, for the one harness that needs
//! real systemd: `scripts/systemd-test.sh`.
//!
//! Its sibling `fake-release` deliberately mints releases with **no units** and a config whose
//! `on_apply` is `none`, because it exists to drive the engine's own logic where systemd is not
//! present. That is the right fixture for almost everything and the wrong one for exactly one
//! question: *does the restart machinery work?* Answering that needs units that really start, a
//! transient timer that really fires, and an `updaterd` that is really replaced by its successor —
//! so it needs its own fixture rather than a flag on that one.
//!
//! What each release carries:
//!
//! ```text
//!   bin/updaterd  bin/robotctl        the binaries under test, built for the container
//!   systemd/updaterd.service          so the update can restart the updater — the whole point
//!   systemd/fake-robotd.service       a stand-in for a daemon in the restart set
//!   systemd/btd.service               a stand-in for the one daemon held back from that restart
//!   bin/fake-btd                      what it runs: publishes an identity, then sleeps
//!   systemd/recovery-check.service    a oneshot with no `[Install]`, which nothing may restart
//!   systemd/broken.service            only in `:broken-unit`: ExecStart=/bin/false
//!   systemd/needs-a-user.service      only in `:sysusers` and `:missing-user`, with `User=`
//!   systemd/sysusers.d/duck-test.conf only in `:sysusers`, and what creates that user
//!   hooks/postinstall                 the real one, so it installs and enables units for real
//! ```
//!
//! `fake-robotd` rather than a real `robotd`: what is under test is whether the engine restarts what
//! a release ships, and a `sleep` proves that as well as motor control does while needing no
//! hardware, no policy and no socket. Its main PID changing is the observation.
//!
//! **The stand-in named `btd` is named that on purpose.** `NEVER_RESTART` is a list of unit names in
//! code, so only a unit actually called `btd` is excluded from the in-flight restart and picked up by
//! the deferred one — which makes it the only way to observe the case the startup reconciliation
//! exists for: a deferred restart that never happened. Unlike `fake-robotd` it publishes an identity,
//! because that is what the reconciliation reads.

use std::path::{Path, PathBuf};

use clap::Parser;
use test_support::Publisher;

#[derive(Parser)]
#[command(about = "Mint releases carrying systemd units, for scripts/systemd-test.sh")]
struct Cli {
    /// Directory to create. Refused if it already holds something.
    root: PathBuf,

    /// The `updaterd` to place in every release, built for wherever this will run.
    #[arg(long)]
    updaterd: PathBuf,

    /// The matching `robotctl`. Same release as `updaterd` or the handshake refuses it.
    #[arg(long)]
    robotctl: PathBuf,

    /// The real `hooks/postinstall`, so the harness observes the shipped one rather than a stub.
    #[arg(long)]
    postinstall: PathBuf,

    /// Where the tree will be *read* from, when that is not where it is minted — the harness
    /// mints on the host and mounts it into a container, where every path differs.
    #[arg(long)]
    prefix: Option<PathBuf>,

    /// `<version>[:broken-unit|:sysusers|:missing-user]`.
    ///
    /// `broken-unit` adds a unit that installs and cannot start; `sysusers` a unit whose `User=` the
    /// release also brings the account for; `missing-user` the same unit with nothing creating it.
    #[arg(required = true, value_name = "SPEC")]
    releases: Vec<String>,
}

/// A unit for the updater itself, pointing through `current` exactly as the shipped one does.
///
/// `RuntimeDirectory=` is load-bearing rather than tidy: it is where the daemon publishes what
/// release it is running, and that file is the only way to see that the deferred restart actually
/// replaced this process rather than merely being scheduled.
fn updaterd_unit(prefix: &Path) -> String {
    format!(
        "[Unit]\nDescription=Updater under test\n\n\
         [Service]\nType=exec\n\
         ExecStart={p}/opt/daemon/current/bin/updaterd --config {p}/updater.toml \
         --socket /run/updaterd.sock\n\
         Restart=on-failure\nRestartSec=1s\nRuntimeDirectory=updaterd\n\
         Environment=RUST_LOG=info\n\n\
         [Install]\nWantedBy=multi-user.target\n",
        p = prefix.display()
    )
}

/// A daemon in the restart set, whose only job is to be restartable and observable.
const FAKE_ROBOTD_UNIT: &str = "[Unit]\nDescription=Stand-in for a daemon an update restarts\n\n\
     [Service]\nType=exec\nExecStart=/bin/sleep infinity\nRestart=always\nRestartSec=1s\n\n\
     [Install]\nWantedBy=multi-user.target\n";

/// What the `btd` stand-in runs.
///
/// `readlink -f`, not `$0`: the unit points through `current`, and an identity naming a symlink says
/// nothing about which release is running. The real daemons read `/proc/self/exe`, which resolves for
/// the same reason.
const FAKE_BTD: &str = "#!/bin/sh\n\
     exe=\"$(readlink -f \"$0\")\"\n\
     mkdir -p /run/btd\n\
     printf '{\"service\":\"btd\",\"version\":\"0.0.0\",\"exe\":\"%s\",\"pid\":%s}\\n' \\\n\
         \"$exe\" \"$$\" > /run/btd/identity.json\n\
     exec sleep infinity\n";

/// The unit for it, named `btd` so `NEVER_RESTART` applies to it.
fn fake_btd_unit(prefix: &Path) -> String {
    format!(
        "[Unit]\nDescription=Stand-in for the daemon an update must not restart\n\n\
         [Service]\nType=exec\nExecStart={p}/opt/daemon/current/bin/fake-btd\n\
         Restart=always\nRestartSec=1s\nRuntimeDirectory=btd\n\n\
         [Install]\nWantedBy=multi-user.target\n",
        p = prefix.display()
    )
}

/// A unit whose `User=` is created by the release's own `sysusers.d` file, which is the ordering
/// `hooks/postinstall` calls load-bearing: accounts before units, or the unit fails to start and the
/// failure reads as a broken daemon rather than a missing account.
const NEEDS_A_USER_UNIT: &str = "[Unit]\nDescription=A unit that runs as a shipped user\n\n\
     [Service]\nType=exec\nExecStart=/bin/sleep infinity\nUser=duck-test\nRestart=always\n\
     RestartSec=1s\n\n[Install]\nWantedBy=multi-user.target\n";

const SYSUSERS_CONF: &str = "u duck-test - \"a user a release brings with it\" /nonexistent\n";

/// The same unit with nothing creating its user — `enable --now` cannot start it, and neither can the
/// restart that follows.
const GHOST_USER_UNIT: &str = "[Unit]\nDescription=A unit whose user does not exist\n\n\
     [Service]\nType=exec\nExecStart=/bin/sleep infinity\nUser=nosuchuser-duck\n\n\
     [Install]\nWantedBy=multi-user.target\n";

/// A unit shaped like the recovery net's oneshot: triggered by a timer, never by an update.
///
/// **No `[Install]` section, which is the whole point.** The real one asks whether the release that
/// booted came up and hands over to `robot-rescue` if not, so an update that restarts it points a
/// rollback check at daemons that are legitimately mid-restart. It records each run, so the harness
/// can assert it did not happen rather than assume.
const RECOVERY_CHECK_UNIT: &str = "[Unit]\nDescription=Shaped like the boot recovery check\n\n\
     [Service]\nType=oneshot\n\
     ExecStart=/bin/sh -c 'echo ran >> /run/recovery-check.ran'\n";

/// A unit that installs cleanly and cannot start. Bug 1 of `install-path-gap.md` in one file: the
/// update must fail and name it, rather than reverting with "not healthy: unreachable".
const BROKEN_UNIT: &str = "[Unit]\nDescription=A unit that will not start\n\n\
     [Service]\nType=oneshot\nExecStart=/bin/false\n\n\
     [Install]\nWantedBy=multi-user.target\n";

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();

    // Same refusal as `fake-release`, for the same reason: this mints a fresh key, and adding to an
    // existing tree would leave what is already there unverifiable against it.
    if cli.root.exists() && std::fs::read_dir(&cli.root)?.next().is_some() {
        return Err(format!("{} exists and is not empty", cli.root.display()).into());
    }

    for dir in ["keys", "published", "opt/daemon", "var", "bin"] {
        std::fs::create_dir_all(cli.root.join(dir))?;
    }

    let updaterd = std::fs::read(&cli.updaterd)?;
    let robotctl = std::fs::read(&cli.robotctl)?;
    let postinstall = std::fs::read_to_string(&cli.postinstall)?;
    let prefix = match &cli.prefix {
        Some(prefix) => prefix.clone(),
        None => cli.root.canonicalize()?,
    };

    // A copy outside any release, which is how a bare board bootstraps: the first install has to be
    // run by an `updaterd` that is not yet installed anywhere.
    let bootstrap = cli.root.join("bin/updaterd");
    std::fs::write(&bootstrap, &updaterd)?;
    make_executable(&bootstrap)?;

    let publisher = Publisher::new(cli.root.join("keys"), cli.root.join("published"));

    for spec in &cli.releases {
        let (version, kind) = spec.split_once(':').unwrap_or((spec.as_str(), ""));
        let dir = cli.root.join("r").join(version);

        let mut release = publisher
            .release(version)
            .dir(dir.clone())
            .file("bin/updaterd", &updaterd, 0o755)
            .file("bin/robotctl", &robotctl, 0o755)
            .file(
                "systemd/updaterd.service",
                updaterd_unit(&prefix).as_bytes(),
                0o644,
            )
            .file(
                "systemd/fake-robotd.service",
                FAKE_ROBOTD_UNIT.as_bytes(),
                0o644,
            )
            .file(
                "systemd/recovery-check.service",
                RECOVERY_CHECK_UNIT.as_bytes(),
                0o644,
            )
            .file("bin/fake-btd", FAKE_BTD.as_bytes(), 0o755)
            .file(
                "systemd/btd.service",
                fake_btd_unit(&prefix).as_bytes(),
                0o644,
            )
            .hook(&postinstall);

        match kind {
            "" => {}
            "broken-unit" => {
                release = release.file("systemd/broken.service", BROKEN_UNIT.as_bytes(), 0o644);
            }
            "sysusers" => {
                release = release
                    .file(
                        "systemd/sysusers.d/duck-test.conf",
                        SYSUSERS_CONF.as_bytes(),
                        0o644,
                    )
                    .file(
                        "systemd/needs-a-user.service",
                        NEEDS_A_USER_UNIT.as_bytes(),
                        0o644,
                    );
            }
            "missing-user" => {
                release = release.file(
                    "systemd/needs-a-user.service",
                    GHOST_USER_UNIT.as_bytes(),
                    0o644,
                );
            }
            other => return Err(format!("unknown spec `:{other}` in `{spec}`").into()),
        }
        release.write();
        println!(
            "r/{version}{}",
            if kind.is_empty() {
                String::new()
            } else {
                format!(" ({kind})")
            }
        );
    }

    // `restart` with an empty list, which is not the same as `none`: the set comes from the units a
    // release ships, so this says "restart what you carry" and nothing more.
    //
    // A command probe rather than a socket one, because there is no `robotd` here — and asking
    // systemd whether the daemon it just restarted is active is a real gate rather than a formality,
    // which `/bin/true` would have been.
    let config = format!(
        r#"# Generated by `cargo run -p test-support --example systemd-fixture`.
trusted_keys_dir = "{p}/keys"
hw_rev = 1
state_dir = "{p}/var"
# Auto-rollback on: the checks driving this fixture are about what a revert does.
robotd_config = "{p}/robotd.toml"
allow_fault_injection = true

[component.daemon]
install_dir = "{p}/opt/daemon"
source = {{ type = "local_dir", path = "{p}/published" }}
keep_previous = 2
on_apply = {{ action = "restart", units = [] }}
health = {{ probe = "command", program = "/usr/bin/systemctl", args = ["is-active", "--quiet", "fake-robotd"], timeout = "20s" }}
"#,
        p = prefix.display()
    );
    std::fs::write(cli.root.join("updater.toml"), config)?;
    std::fs::write(
        cli.root.join("robotd.toml"),
        "[update_gate]\nauto_rollback = true\n",
    )?;

    println!(
        "\nminted at {}, paths written for {}",
        cli.root.display(),
        prefix.display()
    );
    Ok(())
}

#[cfg(unix)]
fn make_executable(path: &Path) -> std::io::Result<()> {
    use std::os::unix::fs::PermissionsExt;
    std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o755))
}

#[cfg(not(unix))]
fn make_executable(_path: &Path) -> std::io::Result<()> {
    Ok(())
}
