//! `updaterd install` — the bootstrap path, tested against the real binary.
//!
//! These run the actual `updaterd` process rather than calling [`updater::engine`],
//! because what needs proving is specifically the wiring the subcommand adds: that it
//! overrides the source, `on_apply` and `health`, and *only* those. Driving the engine
//! directly would test the engine, which `apply.rs` already does thoroughly, and would
//! silently skip the part that can be wrong here.
//!
//! It lives in `updater/tests/` because that is the package defining the binary, so
//! cargo sets `CARGO_BIN_EXE_updaterd` and **guarantees it is rebuilt** before these
//! run. Deriving the path any other way is how `robotd`'s gate test once ended up
//! asserting against a stale binary while appearing to pass (see the M1 notes in
//! `docs/project/roadmap.md`).

use std::path::{Path, PathBuf};
use std::process::Output;

use test_support::Publisher;

/// A robot that has never been updated: published releases, trusted keys, an empty
/// install tree, and a config carrying the **production** `on_apply` and `health`
/// settings.
///
/// Production settings on purpose. A bootstrap that only worked against an inert config
/// would prove nothing — the whole question is whether `install` can land a first
/// release on a robot whose config says "restart robotd and wait for it to report
/// healthy" when neither `robotd` nor its unit exists yet.
struct FreshRobot {
    _dir: tempfile::TempDir,
    root: PathBuf,
    published: PathBuf,
    install: PathBuf,
    publisher: Publisher,
}

impl FreshRobot {
    fn new() -> Self {
        let dir = tempfile::tempdir().unwrap();
        let root = dir.path().to_path_buf();
        let published = root.join("published");
        let install = root.join("opt/robot/daemon");
        // Deliberately NOT creating `install`: a fresh robot has no install tree, and the
        // engine creating it is part of what this tests.
        let publisher = Publisher::new(root.join("keys"), published.clone());

        let fresh = Self {
            _dir: dir,
            root,
            published,
            install,
            publisher,
        };
        fresh.write_config();
        fresh
    }

    /// The config a real robot ships with, paths localised.
    ///
    /// `source` points at the network and `health` at `robotd`'s socket, exactly as
    /// `/etc/robot/updater.toml` does. `install` must override both; nothing here helps
    /// it along.
    fn write_config(&self) {
        std::fs::write(
            self.config_path(),
            format!(
                r#"
trusted_keys_dir = "{keys}"
hw_rev = 1
state_dir = "{state}"
robot_socket = "{root}/run/robotd.sock"

[component.daemon]
install_dir   = "{install}"
keep_previous = 1

[component.daemon.source]
type       = "github_releases"
repo       = "ORG/robot-daemon"
tag_prefix = "daemon-v"

[component.daemon.on_apply]
action = "restart"
units  = ["robotd"]

[component.daemon.health]
probe   = "socket"
timeout = "2s"
"#,
                keys = self.root.join("keys").display(),
                state = self.root.join("var/lib/robot/updater").display(),
                root = self.root.display(),
                install = self.install.display(),
            ),
        )
        .unwrap();
    }

    fn config_path(&self) -> PathBuf {
        self.root.join("updater.toml")
    }

    /// Rewrite the config so its *configured* source is the published directory.
    ///
    /// Lets the no-`--from` path — the one the one-line installer uses, where the source
    /// in the config is what resolves `latest` — be tested without reaching the network.
    fn point_config_at_published(&self) {
        let text = std::fs::read_to_string(self.config_path()).unwrap();
        let rewritten = text.replace(
            r#"type       = "github_releases"
repo       = "ORG/robot-daemon"
tag_prefix = "daemon-v""#,
            &format!(
                r#"type = "local_dir"
path = "{}""#,
                self.published.display()
            ),
        );
        assert_ne!(
            rewritten, text,
            "the source block should have been replaced"
        );
        std::fs::write(self.config_path(), rewritten).unwrap();
    }

    /// Publish a release containing the binaries a real one ships.
    ///
    /// Unlike the other suites, these tests assert the *binaries* landed — a swap onto an
    /// empty directory would satisfy a symlink check and leave a robot with nothing to run.
    fn publish(&self, version: &str) {
        let mut release = self.publisher.release(version);
        for bin in ["bin/updaterd", "bin/robotd", "bin/robotctl"] {
            release = release.file(bin, b"#!/bin/true\n", 0o755);
        }
        release.write();
    }

    fn tamper(&self, version: &str) {
        self.publisher.tamper("daemon", version);
    }

    fn install(&self, extra: &[&str]) -> Output {
        self.run(
            &[
                "--from",
                &self.published.to_string_lossy(),
                "--config",
                &self.config_path().to_string_lossy(),
            ],
            extra,
        )
    }

    /// `install` with no `--from`, so the configured source is used.
    fn install_from_config(&self) -> Output {
        self.run(&["--config", &self.config_path().to_string_lossy()], &[])
    }

    fn run(&self, base: &[&str], extra: &[&str]) -> Output {
        let mut cmd = std::process::Command::new(env!("CARGO_BIN_EXE_updaterd"));
        cmd.arg("install")
            .args(base)
            .args(extra)
            // Warn keeps the output to the lines these tests assert on, and matches what
            // the installer script shows an operator.
            .env("RUST_LOG", "warn");
        cmd.output().unwrap()
    }

    fn live(&self) -> Option<String> {
        std::fs::read_link(self.install.join("current"))
            .ok()
            .map(|t| t.file_name().unwrap().to_string_lossy().into_owned())
    }

    fn current(&self) -> PathBuf {
        self.install.join("current")
    }

    fn update_log(&self) -> String {
        std::fs::read_to_string(self.root.join("var/lib/robot/updater/update-log.jsonl"))
            .unwrap_or_default()
    }
}

fn stderr(out: &Output) -> String {
    String::from_utf8_lossy(&out.stderr).into_owned()
}

// ── the bootstrap ────────────────────────────────────────────────────────────

/// The whole point: a first release lands on a robot with no daemon running, through a
/// config whose `on_apply` restarts a unit that does not exist and whose `health` probes
/// a socket nothing is listening on.
///
/// Without the overrides this fails two ways over — `systemctl restart robotd` errors,
/// and the socket probe times out — and the release reverts to nothing.
#[test]
fn lands_a_first_release_despite_a_production_config() {
    let fresh = FreshRobot::new();
    fresh.publish("1.0.0");

    let out = fresh.install(&[]);
    assert!(out.status.success(), "install failed:\n{}", stderr(&out));

    assert_eq!(fresh.live().as_deref(), Some("1.0.0"));
    // Content, not just the symlink: a swap onto an empty directory would satisfy the
    // link check and leave a robot with no binaries.
    for required in ["bin/updaterd", "bin/robotd", "bin/robotctl", "version.toml"] {
        assert!(
            fresh.current().join(required).exists(),
            "installed release is missing {required}"
        );
    }
}

/// The one-liner installer's path: no `--from`, so the source in `/etc/robot/updater.toml`
/// resolves `latest` itself.
///
/// That is what keeps the shell script from having to parse a signed manifest to learn
/// the version and the artifact URL — which it could not do without either `jq` or a
/// hand-rolled JSON regex, and which would put a second, weaker reader of a signed
/// document into the trust chain.
#[test]
fn installs_from_the_configured_source_when_from_is_omitted() {
    let fresh = FreshRobot::new();
    fresh.publish("1.0.0");
    fresh.point_config_at_published();

    let out = fresh.install_from_config();
    assert!(out.status.success(), "{}", stderr(&out));
    assert_eq!(fresh.live().as_deref(), Some("1.0.0"));
    assert!(fresh.current().join("bin/updaterd").exists());
}

/// The install must be recorded like any other, or a robot's history starts with a hole
/// where the release it is running came from.
#[test]
fn records_the_install_in_the_update_log() {
    let fresh = FreshRobot::new();
    fresh.publish("1.0.0");
    assert!(fresh.install(&[]).status.success());

    let log = fresh.update_log();
    assert!(
        log.contains("1.0.0") && log.contains("success"),
        "the update log should record the bootstrap install, got: {log:?}"
    );
}

/// Signature verification is not relaxed for the first install. This is the one place a
/// bootstrap shortcut would be invisible and fatal: every later update's trust derives
/// from the release landed here.
#[test]
fn refuses_a_tampered_artifact() {
    let fresh = FreshRobot::new();
    fresh.publish("1.0.0");
    fresh.tamper("1.0.0");

    let out = fresh.install(&[]);
    assert!(
        !out.status.success(),
        "a tampered artifact must not install:\n{}",
        stderr(&out)
    );
    assert_eq!(
        fresh.live(),
        None,
        "nothing may be live after a refused install"
    );
}

/// An unsigned-by-us release is refused for the same reason, via a key the robot does
/// not trust rather than a corrupted byte.
#[test]
fn refuses_a_release_signed_by_an_untrusted_key() {
    let fresh = FreshRobot::new();
    fresh.publish("1.0.0");

    // Re-sign everything with a key that was never installed.
    let attacker = minisign::KeyPair::generate_unencrypted_keypair().unwrap();
    for name in ["daemon-1.0.0.tar.zst", "1.0.0.manifest.json"] {
        let path = fresh.published.join(name);
        let bytes = std::fs::read(&path).unwrap();
        let sig = minisign::sign(None, &attacker.sk, bytes.as_slice(), None, None)
            .unwrap()
            .to_string();
        std::fs::write(format!("{}.minisig", path.display()), sig).unwrap();
    }

    let out = fresh.install(&[]);
    assert!(!out.status.success(), "{}", stderr(&out));
    assert_eq!(fresh.live(), None);
}

/// The guard that keeps the forced `health = none` from ever applying to a working
/// robot. Without it, `install` would be a way to apply an update with auto-rollback
/// silently disabled.
#[test]
fn refuses_once_a_release_is_live_and_says_what_to_use_instead() {
    let fresh = FreshRobot::new();
    fresh.publish("1.0.0");
    assert!(fresh.install(&[]).status.success());
    fresh.publish("1.1.0");

    let out = fresh.install(&[]);
    assert!(
        !out.status.success(),
        "install must refuse a robot that already has a release live"
    );
    let stderr = stderr(&out);
    assert!(
        stderr.contains("robotctl update apply"),
        "the refusal must name the right tool, got: {stderr}"
    );
    assert_eq!(
        fresh.live().as_deref(),
        Some("1.0.0"),
        "a refused install must not disturb the live release"
    );
    assert!(
        stderr.contains("--force"),
        "the refusal must name the escape hatch, got: {stderr}"
    );
}

/// The escape hatch, and the reason it exists: a board whose installed `updaterd` is too
/// old to accept the release that fixes it. `robotctl update apply` cannot help, because the
/// old binary is the one running the gate.
///
/// No `robotd` answers in this fixture, which is the condition `--force` requires — the
/// objection to installing over a live release is about a *working* robot losing
/// auto-rollback, and there is no robot here.
#[test]
fn force_installs_over_a_live_release_when_no_robot_answers() {
    let fresh = FreshRobot::new();
    fresh.publish("1.0.0");
    assert!(fresh.install(&[]).status.success());
    fresh.publish("1.1.0");

    let out = fresh.install(&["--force"]);
    assert!(
        out.status.success(),
        "--force must install over a live release: {}",
        stderr(&out)
    );
    assert_eq!(
        fresh.live().as_deref(),
        Some("1.1.0"),
        "the forced install must actually move the live release"
    );
    let stderr = stderr(&out);
    assert!(
        stderr.contains("cannot auto-roll-back"),
        "a forced install must say what it gave up, got: {stderr}"
    );
}

/// `--dry-run` proves a downloaded release is installable without committing to it,
/// which is what lets an installer validate before it touches the robot.
#[test]
fn dry_run_verifies_without_installing() {
    let fresh = FreshRobot::new();
    fresh.publish("1.0.0");

    let out = fresh.install(&["--dry-run"]);
    assert!(out.status.success(), "{}", stderr(&out));
    assert_eq!(
        fresh.live(),
        None,
        "a dry run must leave the robot untouched"
    );

    // And it must not have been a no-op that would pass on a broken release either.
    let fresh = FreshRobot::new();
    fresh.publish("1.0.0");
    fresh.tamper("1.0.0");
    assert!(!fresh.install(&["--dry-run"]).status.success());
}

/// A misspelled component must say so, rather than reporting "no releases found" from
/// the source layer or, worse, installing the wrong thing.
#[test]
fn unknown_component_is_named() {
    let fresh = FreshRobot::new();
    fresh.publish("1.0.0");

    let out = fresh.install(&["--component", "modle-walk"]);
    assert!(!out.status.success());
    assert!(
        stderr(&out).contains("no such component"),
        "got: {}",
        stderr(&out)
    );
}

/// A `--from` that does not exist is an operator typo, and must be reported as one at
/// the point of the mistake.
#[test]
fn missing_source_directory_is_reported_directly() {
    let fresh = FreshRobot::new();
    let missing = fresh.root.join("nowhere");

    let mut cmd = std::process::Command::new(env!("CARGO_BIN_EXE_updaterd"));
    let out = cmd
        .arg("install")
        .arg("--from")
        .arg(&missing)
        .arg("--config")
        .arg(fresh.config_path())
        .env("RUST_LOG", "warn")
        .output()
        .unwrap();

    assert!(!out.status.success());
    assert!(
        stderr(&out).contains("cannot read --from"),
        "got: {}",
        stderr(&out)
    );
}

/// The config on disk is the single source of truth for the trust anchor. An `install`
/// that fell back to some built-in default when `trusted_keys_dir` was wrong would be a
/// robot installing unverified code.
#[test]
fn a_config_with_no_trusted_keys_is_fatal() {
    let fresh = FreshRobot::new();
    fresh.publish("1.0.0");
    std::fs::remove_file(fresh.publisher.key_file()).unwrap();

    let out = fresh.install(&[]);
    assert!(!out.status.success(), "{}", stderr(&out));
    assert_eq!(fresh.live(), None);
}

/// Sanity on the harness itself: `CARGO_BIN_EXE_updaterd` must point at a binary that
/// actually has this subcommand. If someone removes `install`, these tests should fail
/// loudly rather than every case failing for the same uninformative reason.
#[test]
fn the_binary_under_test_has_an_install_subcommand() {
    let out = std::process::Command::new(env!("CARGO_BIN_EXE_updaterd"))
        .args(["install", "--help"])
        .output()
        .unwrap();
    assert!(out.status.success());
    let help = String::from_utf8_lossy(&out.stdout);
    assert!(help.contains("--from"), "got: {help}");
    assert!(
        Path::new(env!("CARGO_BIN_EXE_updaterd")).exists(),
        "cargo should have built the binary before running this test"
    );
}
