//! Mint signed releases, so the real `updaterd` and `robotctl` can be driven by hand.
//!
//! Those two binaries already do everything a robot does with a release — install the
//! first one, apply the next, roll back a bad one, recover from a crash mid-swap. What
//! nothing in the tree hands you is a *signed release to feed them*: `xtask package`
//! wants real built binaries and a version matching `Cargo.toml`, and the fixtures the
//! tests use live inside `#[test]` functions. This is that missing piece and nothing
//! more — it publishes, and never installs.
//!
//! Two callers: `scripts/board-test.sh`, which mints a fixture on the host and mounts it
//! into the ARM64 container, and the walkthrough in `README.md`.
//!
//! ```text
//!   cargo run -p test-support --example fake-release -- /tmp/pg 1.0.0 1.1.0
//! ```
//!
//! Lays out one tree, ready to point a daemon at with `--config <root>/updater.toml`:
//!
//! ```text
//!   <root>/keys/prod.pub      trusted key (what the robot ships with)
//!   <root>/r/<version>/       each signed release, minted but not offered
//!   <root>/published/         the "remote": copy a release in here to offer it
//!   <root>/opt/daemon/        install tree, where `current` will point
//!   <root>/var/               engine state: journal, lock, boot counter
//!   <root>/updater.toml       config
//! ```
//!
//! `r/` and `published/` are separate so the caller decides what `latest` resolves to. The
//! `local_dir` source serves the newest version in the directory it is pointed at, so
//! walking forward through versions means copying them in one at a time — and a release
//! that should be *refused* can be staged without ever having been installable.
//!
//! One run mints one keypair and signs everything with it, so releases from two runs are
//! not interchangeable. Ask for every version you need up front rather than adding to a
//! tree later; the secret key is deliberately not kept.

use std::path::PathBuf;

use clap::Parser;
use test_support::Publisher;

#[derive(Parser)]
#[command(about = "Mint signed releases for driving updaterd and robotctl by hand")]
struct Cli {
    /// Directory to create. Refused if it already holds something.
    root: PathBuf,

    /// Paths to write into `updater.toml`, when the tree will be *used* from somewhere
    /// other than where it was minted. The board test mints on the host and runs the
    /// same tree from a container mount, where every path is different.
    #[arg(long)]
    prefix: Option<PathBuf>,

    /// `<version>[:tamper|:hook|:bad-hook]` — e.g. `1.0.0`, `1.2.0:tamper`.
    ///
    /// `tamper` corrupts the artifact after signing (must be refused), `hook` embeds a
    /// post-install hook that succeeds, `bad-hook` one that fails (must roll back).
    #[arg(required = true, value_name = "SPEC")]
    releases: Vec<String>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();

    // Refuse rather than add to an existing tree: this mints a fresh keypair, which would
    // leave every release already sitting there unverifiable against the key now on disk —
    // a signature failure that looks exactly like the bug the fixture exists to detect.
    if cli.root.exists() && std::fs::read_dir(&cli.root)?.next().is_some() {
        return Err(format!(
            "{} already exists and is not empty.\n\
             This generates a new signing key, which would leave releases already there \
             unverifiable.\n\
             Remove it, or pick another directory.",
            cli.root.display()
        )
        .into());
    }

    for dir in ["keys", "published", "opt/daemon", "var"] {
        std::fs::create_dir_all(cli.root.join(dir))?;
    }

    let publisher = Publisher::new(cli.root.join("keys"), cli.root.join("published"));

    for spec in &cli.releases {
        let (version, kind) = match spec.split_once(':') {
            None => (spec.as_str(), ""),
            Some((version, kind)) => (version, kind),
        };
        let dir = cli.root.join("r").join(version);

        let mut release = publisher.release(version).dir(dir.clone());
        match kind {
            "" | "tamper" => {}
            "hook" => {
                release = release.hook(
                    "#!/bin/sh\necho \"hook ran: $UPDATE_OLD_VERSION -> $UPDATE_NEW_VERSION\"\n",
                );
            }
            "bad-hook" => {
                release = release.hook("#!/bin/sh\necho 'migration failed' >&2\nexit 1\n");
            }
            // Fails closed on a typo: silently publishing a plain release where a tampered
            // one was asked for would make the caller's "must be refused" check pass for
            // the wrong reason.
            other => return Err(format!("unknown spec `:{other}` in `{spec}`").into()),
        }
        release.write();

        if kind == "tamper" {
            publisher.tamper_in(&dir, "daemon", version);
        }

        match kind {
            "" => println!("r/{version}"),
            _ => println!("r/{version} ({kind})"),
        }
    }

    let prefix = match &cli.prefix {
        Some(prefix) => prefix.clone(),
        None => cli.root.canonicalize()?,
    };
    let config = format!(
        r#"# Generated by `cargo run -p test-support --example fake-release`.
trusted_keys_dir = "{prefix}/keys"
hw_rev = 1
state_dir = "{prefix}/var"

# `--inject-fault` is refused without this, and it is never set on a client robot.
allow_fault_injection = true

[component.daemon]
install_dir = "{prefix}/opt/daemon"
source = {{ type = "local_dir", path = "{prefix}/published" }}
# `none` because there are no systemd units here. On a robot this restarts robotd/mediad —
# never updaterd or btd (docs/design/updater-design.md §4).
on_apply = {{ action = "none" }}
# `none` because no robotd is answering. A socket probe with nothing to ask does not pass:
# it polls until the timeout and then fails the gate, which would roll back every release.
# To exercise rollback, run the daemon with `--inject-fault fail_health` instead — that
# fails the gate on purpose, which is the point.
health = {{ probe = "none" }}
keep_previous = 2
"#,
        prefix = prefix.display(),
    );
    std::fs::write(cli.root.join("updater.toml"), config)?;

    println!("\nminted at {}", cli.root.display());

    // Only when the tree is usable where it sits. With `--prefix` the config names paths
    // that do not exist yet on this machine, so a copy-pasteable command would be a lie.
    match &cli.prefix {
        Some(prefix) => println!("updater.toml reads paths under {}", prefix.display()),
        None => println!(
            "\nnext: offer one release and install it\n  \
             cp {root}/r/{first}/* {root}/published/\n  \
             cargo run -p updater --bin updaterd -- --config {root}/updater.toml install \
             --from {root}/published",
            root = cli.root.display(),
            first = cli.releases[0]
                .split_once(':')
                .map_or(cli.releases[0].as_str(), |(version, _)| version),
        ),
    }
    Ok(())
}
