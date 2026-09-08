//! End-to-end tests for the update state machine, over the `LocalDir` source.
//!
//! These are the Tier-1 mechanism tests from `docs/design/updater-design.md` §16.2: they
//! drive the **real engine code path** with no network and no robot, so they cannot
//! drift from production behaviour. This is the suite that replaces manually
//! reverting a robot, applying an update, and eyeballing the result.
//!
//! Rollback is the thing most likely to be quietly broken, because it only runs
//! when something else already went wrong — so most of these tests deliberately
//! break something.

use std::path::{Path, PathBuf};

use test_support::Publisher;

use updater::config::Config;
use updater::engine::{ApplyOptions, Engine};
use updater::faults::Faults;
use updater::proto::{ApplyResult, CheckResult, Outcome, Target};
use updater::robot::{AbsentRobot, Health, RobotClient, SafeToRestart};
use updater::verify::KeyRing;

// ── fixture ──────────────────────────────────────────────────────────────────

/// A robot that answers exactly what a test wants. The reason [`RobotClient`] is a
/// trait: degraded paths must be testable without staging a real crash.
struct FakeRobot {
    healthy: bool,
    model_api: Option<u32>,
}

impl FakeRobot {
    fn healthy() -> Self {
        Self {
            healthy: true,
            model_api: Some(1),
        }
    }
    fn unhealthy() -> Self {
        Self {
            healthy: false,
            model_api: Some(1),
        }
    }
}

/// `robotd` is down *now* (so its model API is unknown and nothing is moving), but
/// comes up healthy once a good release is linked.
///
/// This is the realistic recovery scenario, and distinct from [`AbsentRobot`],
/// which models a robot that never comes up at all — where rolling back wouldn't
/// help either.
struct DeadThenHealthy;

#[async_trait::async_trait]
impl RobotClient for DeadThenHealthy {
    async fn safe_to_restart(&self, _t: std::time::Duration) -> SafeToRestart {
        SafeToRestart::Unreachable
    }
    async fn health(&self, _t: std::time::Duration) -> Health {
        Health::Healthy
    }
    async fn model_api(&self, _t: std::time::Duration) -> Option<u32> {
        None
    }
    async fn remote_session_active(&self, _t: std::time::Duration) -> bool {
        false
    }
    async fn reload_policies(&self, _timeout: std::time::Duration) -> bool {
        true
    }
}

/// A robot that came up fine but cannot see its servos — a bench board with the motor
/// supply off, which is exactly the configuration the update system gets tested on.
struct DegradedRobot;

#[async_trait::async_trait]
impl RobotClient for DegradedRobot {
    async fn safe_to_restart(&self, _t: std::time::Duration) -> SafeToRestart {
        SafeToRestart::Yes
    }
    async fn health(&self, _t: std::time::Duration) -> Health {
        Health::Degraded("no answer from the motor bus after 12 attempts".into())
    }
    async fn model_api(&self, _t: std::time::Duration) -> Option<u32> {
        None
    }
    async fn remote_session_active(&self, _t: std::time::Duration) -> bool {
        false
    }
    async fn reload_policies(&self, _timeout: std::time::Duration) -> bool {
        true
    }
}

#[async_trait::async_trait]
impl RobotClient for FakeRobot {
    async fn safe_to_restart(&self, _t: std::time::Duration) -> SafeToRestart {
        SafeToRestart::Yes
    }
    async fn health(&self, _t: std::time::Duration) -> Health {
        if self.healthy {
            Health::Healthy
        } else {
            Health::Unhealthy("motors not responding".into())
        }
    }
    async fn model_api(&self, _t: std::time::Duration) -> Option<u32> {
        self.model_api
    }
    async fn remote_session_active(&self, _t: std::time::Duration) -> bool {
        false
    }
    async fn reload_policies(&self, _timeout: std::time::Duration) -> bool {
        true
    }
}

struct Fixture {
    _dir: tempfile::TempDir,
    root: PathBuf,
    releases: PathBuf,
    install: PathBuf,
    publisher: Publisher,
}

impl Fixture {
    fn new() -> Self {
        let dir = tempfile::tempdir().unwrap();
        let root = dir.path().to_path_buf();
        let releases = root.join("published");
        let install = root.join("opt/robot/daemon");
        std::fs::create_dir_all(&install).unwrap();
        let publisher = Publisher::new(root.join("keys"), releases.clone());

        Self {
            _dir: dir,
            root,
            releases,
            install,
            publisher,
        }
    }

    fn publish(&self, version: &str, hook: Option<&str>) {
        let release = self.publisher.release(version);
        match hook {
            Some(script) => release.hook(script).write(),
            None => release.write(),
        }
    }

    /// As [`Self::publish`], but lets the caller mutate the manifest before it is signed —
    /// for compatibility and floor tests.
    fn publish_with(
        &self,
        version: &str,
        hook: Option<&str>,
        edit: impl FnOnce(&mut serde_json::Value),
    ) {
        let release = self.publisher.release(version).manifest(edit);
        match hook {
            Some(script) => release.hook(script).write(),
            None => release.write(),
        }
    }

    /// Where model releases are published. Separate from the daemon's remote: one shared
    /// directory would let the daemon's `latest` resolve to a model manifest.
    fn model_releases(&self) -> PathBuf {
        let dir = self.root.join("published-model");
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn publish_model(&self, version: &str) {
        self.publisher
            .release(version)
            .channel("model")
            .dir(self.model_releases())
            .file("walk.onnx", b"weights", 0o644)
            .write();
    }

    /// A directory a laptop would have rsynced onto the board, holding releases the
    /// configured source has never heard of. `scripts/dev-push.sh` writes this shape.
    fn sideload(&self) -> PathBuf {
        let dir = self.root.join("var/tmp/duck-sideload");
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// Publish `version` into the sideload directory *only*, signed with the same key: a
    /// local build is signed with a trusted dev key, which is the whole reason it installs.
    fn publish_sideload(&self, version: &str) {
        self.publisher.release(version).dir(self.sideload()).write();
    }

    fn point_ref_at(&self, git_ref: &str, version: &str) {
        self.publisher.point_ref_at(git_ref, version);
    }

    fn unpublish(&self, version: &str) {
        self.publisher.unpublish("daemon", version);
    }

    fn tamper(&self, version: &str) {
        self.publisher.tamper("daemon", version);
    }

    fn live_version(&self) -> Option<String> {
        test_support::live_version(&self.install)
    }

    fn live_marker(&self) -> Option<String> {
        test_support::live_marker(&self.install)
    }

    fn pending_file_exists(&self) -> bool {
        self.root
            .join("var/lib/robot/updater/pending.json")
            .exists()
    }

    /// Keys are written once at construction, never here — otherwise a test that
    /// swaps the trusted key would have it silently restored on the next
    /// `engine()` call.
    fn config(&self, extra: &str) -> Config {
        let keys_dir = self.root.join("keys");

        Config::from_toml(&format!(
            r#"
trusted_keys_dir = "{keys}"
hw_rev = 1
state_dir = "{state}"

[component.daemon]
install_dir = "{install}"
source = {{ type = "local_dir", path = "{published}" }}
on_apply = {{ action = "none" }}
health = {{ probe = "socket", timeout = "2s" }}
{extra}
"#,
            keys = keys_dir.display(),
            state = self.root.join("var/lib/robot/updater").display(),
            install = self.install.display(),
            published = self.releases.display(),
        ))
        .expect("fixture config must be valid")
    }

    fn engine(&self, robot: Box<dyn RobotClient>, faults: Faults, extra: &str) -> Engine {
        let config = self.config(extra);
        let keys = KeyRing::load(&config.trusted_keys_dir, config.allow_dev_keys).unwrap();
        // No deferred restarts: this file runs dozens of engines in parallel, and a `fork` in any one
        // of them hands copies of the *others*' update locks to a child, which surfaced as
        // `got Busy` in whichever test held a lock at that moment. See
        // `Engine::without_deferred_restarts`.
        Engine::new(config, keys, robot, faults)
            .unwrap()
            .without_deferred_restarts()
    }

    fn engine_healthy(&self) -> Engine {
        self.engine(Box::new(FakeRobot::healthy()), Faults::none(), "")
    }

    /// A stand-in for `/etc/systemd/system`, which a test can neither read nor write meaningfully:
    /// on a CI runner it holds whatever that machine has installed, and on a robot it holds the
    /// real units.
    fn unit_dir(&self) -> PathBuf {
        let dir = self.root.join("etc/systemd/system");
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// Install a unit that execs `bin/<name>` through this component's `current` symlink — what
    /// `hooks/postinstall` leaves on the board.
    fn install_unit(&self, name: &str) {
        std::fs::write(
            self.unit_dir().join(format!("{name}.service")),
            format!(
                "[Service]\nExecStart={}/current/bin/{name}\n",
                self.install.display()
            ),
        )
        .unwrap();
    }

    /// A release carrying binaries, so a unit installed on the board has something to exec.
    fn publish_with_binaries(&self, version: &str, binaries: &[&str]) {
        let mut release = self.publisher.release(version);
        for name in binaries {
            release = release.file(&format!("bin/{name}"), b"#!/bin/true\n", 0o755);
        }
        release.write();
    }

    fn engine_seeing_units(&self) -> Engine {
        self.engine_healthy().with_unit_dir(self.unit_dir())
    }

    fn release_exists(&self, version: &str) -> bool {
        self.install.join("releases").join(version).is_dir()
    }

    fn staging_leftovers(&self) -> usize {
        let dir = self.install.join("releases");
        std::fs::read_dir(&dir)
            .map(|entries| {
                entries
                    .filter_map(Result::ok)
                    .filter(|e| e.file_name().to_string_lossy().starts_with(".staging-"))
                    .count()
            })
            .unwrap_or(0)
    }
}

/// Drains progress so senders never block, and returns the phases seen.
fn progress_channel() -> (
    updater::engine::ProgressTx,
    tokio::sync::mpsc::UnboundedReceiver<updater::proto::Progress>,
) {
    tokio::sync::mpsc::unbounded_channel()
}

async fn apply_exact(engine: &mut Engine, version: &str) -> Result<ApplyResult, updater::Error> {
    let (tx, _rx) = progress_channel();
    engine
        .apply(
            "daemon",
            Target::Exact(semver::Version::parse(version).unwrap()),
            ApplyOptions::default(),
            tx,
        )
        .await
}

async fn apply_ref(engine: &mut Engine, git_ref: &str) -> Result<ApplyResult, updater::Error> {
    let (tx, _rx) = progress_channel();
    engine
        .apply(
            "daemon",
            Target::Ref(git_ref.to_owned()),
            ApplyOptions::default(),
            tx,
        )
        .await
}

async fn apply_latest(engine: &mut Engine) -> Result<ApplyResult, updater::Error> {
    let (tx, _rx) = progress_channel();
    engine
        .apply("daemon", Target::Latest, ApplyOptions::default(), tx)
        .await
}

// ── happy path ───────────────────────────────────────────────────────────────

#[tokio::test]
async fn installs_from_scratch_and_reports_progress() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();

    let (tx, mut rx) = progress_channel();
    let result = engine
        .apply("daemon", Target::Latest, ApplyOptions::default(), tx)
        .await
        .unwrap();

    assert!(matches!(result, ApplyResult::Applied { from: None, .. }));
    assert_eq!(fx.live_version().as_deref(), Some("1.0.0"));
    assert_eq!(fx.live_marker().as_deref(), Some("version=1.0.0\n"));

    // The app's progress bar depends on these arriving in order.
    let mut phases = Vec::new();
    while let Ok(p) = rx.try_recv() {
        phases.push(p.phase);
    }
    use updater::proto::Phase;
    for expected in [
        Phase::Checking,
        Phase::Preflight,
        Phase::Verifying,
        Phase::Swapping,
        Phase::HealthGate,
        Phase::Committing,
    ] {
        assert!(
            phases.contains(&expected),
            "missing {expected:?} in {phases:?}"
        );
    }
}

#[tokio::test]
async fn upgrades_and_keeps_previous_for_rollback() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    fx.publish("1.1.0", None);
    let result = apply_latest(&mut engine).await.unwrap();

    match result {
        ApplyResult::Applied { from, to } => {
            assert_eq!(from, Some(semver::Version::new(1, 0, 0)));
            assert_eq!(to, semver::Version::new(1, 1, 0));
        }
        other => panic!("expected Applied, got {other:?}"),
    }
    assert_eq!(fx.live_version().as_deref(), Some("1.1.0"));
    assert!(
        fx.release_exists("1.0.0"),
        "previous must be kept for rollback"
    );
}

#[tokio::test]
async fn applying_the_same_version_is_a_no_op() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    let result = apply_latest(&mut engine).await.unwrap();
    assert!(matches!(result, ApplyResult::AlreadyCurrent { .. }));
}

#[tokio::test]
async fn applies_an_exact_version_not_just_latest() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    fx.publish("2.0.0", None);
    let mut engine = fx.engine_healthy();

    let (tx, _rx) = progress_channel();
    engine
        .apply(
            "daemon",
            Target::Exact(semver::Version::new(1, 0, 0)),
            ApplyOptions::default(),
            tx,
        )
        .await
        .unwrap();

    // This is the primitive that makes release testing scriptable.
    assert_eq!(fx.live_version().as_deref(), Some("1.0.0"));
}

#[tokio::test]
async fn check_reports_availability_without_changing_anything() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let engine = fx.engine_healthy();

    let result = engine.check("daemon").await.unwrap();
    assert!(matches!(
        result,
        CheckResult::Available {
            installed: None,
            ..
        }
    ));
    assert_eq!(fx.live_version(), None, "check must not install anything");
}

#[tokio::test]
async fn dry_run_verifies_everything_but_does_not_swap() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();

    let (tx, _rx) = progress_channel();
    let result = engine
        .apply(
            "daemon",
            Target::Latest,
            ApplyOptions {
                dry_run: true,
                ..Default::default()
            },
            tx,
        )
        .await
        .unwrap();

    assert!(matches!(result, ApplyResult::DryRunPassed { .. }));
    assert_eq!(fx.live_version(), None, "dry run must not swap");
    assert_eq!(
        fx.staging_leftovers(),
        0,
        "dry run must clean up after itself"
    );
}

// ── refusals: nothing changes ────────────────────────────────────────────────

#[tokio::test]
async fn refuses_tampered_artifact() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    fx.tamper("1.0.0");

    let mut engine = fx.engine_healthy();
    let err = apply_latest(&mut engine).await.unwrap_err();

    assert!(
        matches!(err, updater::Error::Verification(_)),
        "got {err:?}"
    );
    assert_eq!(fx.live_version(), None, "nothing may be installed");
    assert_eq!(fx.staging_leftovers(), 0);
}

#[tokio::test]
async fn refuses_artifact_signed_by_an_untrusted_key() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);

    // Replace the trusted key with an unrelated one.
    let other = minisign::KeyPair::generate_unencrypted_keypair().unwrap();
    let other_pk = other
        .pk
        .to_box()
        .unwrap()
        .to_string()
        .lines()
        .next_back()
        .unwrap()
        .to_owned();
    std::fs::write(fx.root.join("keys/prod.pub"), other_pk).unwrap();

    let mut engine = fx.engine_healthy();
    let err = apply_latest(&mut engine).await.unwrap_err();
    assert!(
        matches!(err, updater::Error::Verification(_)),
        "got {err:?}"
    );
    assert_eq!(fx.live_version(), None);
}

#[tokio::test]
async fn refuses_release_requiring_newer_hardware() {
    let fx = Fixture::new();
    fx.publish_with("2.0.0", None, |m| {
        m["min_hw_rev"] = serde_json::json!(9);
    });

    let mut engine = fx.engine_healthy();
    let err = apply_latest(&mut engine).await.unwrap_err();
    assert!(
        matches!(err, updater::Error::Incompatible(_)),
        "got {err:?}"
    );
    assert_eq!(fx.live_version(), None);
}

#[tokio::test]
async fn refuses_manifest_from_the_wrong_channel() {
    let fx = Fixture::new();
    fx.publish_with("1.0.0", None, |m| {
        m["channel"] = serde_json::json!("model");
    });

    let mut engine = fx.engine_healthy();
    let err = apply_latest(&mut engine).await.unwrap_err();
    // A misconfigured URL must not be able to install a model as the daemon.
    assert!(
        matches!(err, updater::Error::Incompatible(_)),
        "got {err:?}"
    );
}

#[tokio::test]
async fn refuses_when_pinned_to_another_version() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    fx.publish("1.1.0", None);

    let mut engine = fx.engine(
        Box::new(FakeRobot::healthy()),
        Faults::none(),
        r#"pinned = "1.0.0""#,
    );
    let err = apply_latest(&mut engine).await.unwrap_err();
    assert!(
        matches!(err, updater::Error::Incompatible(_)),
        "got {err:?}"
    );
}

#[tokio::test]
async fn refuses_when_disk_is_full() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);

    let mut engine = fx.engine(
        Box::new(FakeRobot::healthy()),
        Faults {
            simulate_disk_full: true,
            ..Faults::none()
        },
        "",
    );
    let err = apply_latest(&mut engine).await.unwrap_err();
    assert!(matches!(err, updater::Error::Preflight(_)), "got {err:?}");
    assert_eq!(fx.live_version(), None, "must abort before downloading");
}

/// The case that motivated the check, as it happened: a board running a release with `configd`
/// resolves to one that predates it, `configd.service` stays behind, and its binary is gone.
///
/// What it used to do — install, fail the restart with `203/EXEC`, roll back, and report a systemd
/// error code — was the right outcome reached by the most expensive route, and named nothing.
#[tokio::test]
async fn refuses_a_release_that_would_orphan_an_installed_unit() {
    let fx = Fixture::new();
    fx.install_unit("robotd");
    fx.install_unit("configd");
    // The candidate predates `configd`: it still ships `robotd`, and that unit must not be
    // reported — otherwise this would pass while refusing every release for the wrong reason.
    fx.publish_with_binaries("1.0.0", &["robotd"]);

    let mut engine = fx.engine_seeing_units();
    let err = apply_latest(&mut engine).await.unwrap_err();

    let updater::Error::WouldOrphanUnit(message) = &err else {
        panic!("got {err:?}");
    };
    assert!(
        message.contains("configd.service execs bin/configd"),
        "{message}"
    );
    assert!(!message.contains("robotd.service"), "{message}");
    // The refusal is only useful if the way past it is in the text.
    assert!(
        message.contains("systemctl disable --now configd.service"),
        "{message}"
    );

    assert_eq!(fx.live_version(), None, "nothing may be installed");
    assert_eq!(fx.staging_leftovers(), 0);
}

/// The escape, pinned: removing the unit is the whole remedy, and there is no flag. An operator who
/// follows the refusal text must end up with the release installed, or this is a bricked update
/// path on a board where nothing else would get them off a bad release.
#[tokio::test]
async fn removing_the_orphaned_unit_lets_the_release_through() {
    let fx = Fixture::new();
    fx.install_unit("configd");
    fx.publish_with_binaries("1.0.0", &["robotd"]);

    let mut engine = fx.engine_seeing_units();
    assert!(apply_latest(&mut engine).await.is_err());

    std::fs::remove_file(fx.unit_dir().join("configd.service")).unwrap();

    let result = apply_latest(&mut engine).await.unwrap();
    assert!(matches!(result, ApplyResult::Applied { .. }), "{result:?}");
    assert_eq!(fx.live_version().as_deref(), Some("1.0.0"));
}

/// A dry run is how "will this downgrade work?" gets asked, so it has to answer — which is why the
/// check runs before the dry run returns rather than at the swap.
#[tokio::test]
async fn a_dry_run_reports_the_orphan_rather_than_passing() {
    let fx = Fixture::new();
    fx.install_unit("configd");
    fx.publish_with_binaries("1.0.0", &["robotd"]);

    let mut engine = fx.engine_seeing_units();
    let (tx, _rx) = progress_channel();
    let err = engine
        .apply(
            "daemon",
            Target::Latest,
            ApplyOptions {
                dry_run: true,
                ..Default::default()
            },
            tx,
        )
        .await
        .unwrap_err();

    assert!(
        matches!(err, updater::Error::WouldOrphanUnit(_)),
        "got {err:?}"
    );
}

// ── rollback ─────────────────────────────────────────────────────────────────

#[tokio::test]
async fn rolls_back_when_the_new_release_is_unhealthy() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);

    // Install 1.0.0 with a healthy robot.
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    // Now 1.1.0 comes up sick.
    fx.publish("1.1.0", None);
    let mut engine = fx.engine(Box::new(FakeRobot::unhealthy()), Faults::none(), "");
    let result = apply_latest(&mut engine).await.unwrap();

    match result {
        ApplyResult::RolledBack {
            attempted,
            reverted_to,
            ..
        } => {
            assert_eq!(attempted, semver::Version::new(1, 1, 0));
            assert_eq!(reverted_to, Some(semver::Version::new(1, 0, 0)));
        }
        other => panic!("expected RolledBack, got {other:?}"),
    }

    // The robot must be running the *old* release, content and all.
    assert_eq!(fx.live_version().as_deref(), Some("1.0.0"));
    assert_eq!(fx.live_marker().as_deref(), Some("version=1.0.0\n"));
}

/// The counterpart to `rolls_back_when_the_new_release_is_unhealthy`. Rollback answers
/// "did this release break the robot?", and a board with no servo power answers nothing
/// about the release — it said the same before the swap, and reverting cannot change it.
/// Rolling back there would revert every release shipped to a bench board and churn the
/// boot counter doing it.
#[tokio::test]
async fn commits_when_the_robot_is_degraded_rather_than_broken() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);

    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    fx.publish("1.1.0", None);
    let mut engine = fx.engine(Box::new(DegradedRobot), Faults::none(), "");
    let result = apply_latest(&mut engine).await.unwrap();

    assert!(
        matches!(result, ApplyResult::Applied { .. }),
        "a degraded board must still take the update, got {result:?}"
    );
    assert_eq!(fx.live_version().as_deref(), Some("1.1.0"));
}

#[tokio::test]
async fn rolls_back_when_the_post_install_hook_fails() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    // A hook that fails is exactly as fatal as a failed health probe.
    fx.publish(
        "1.1.0",
        Some("#!/bin/sh\necho migration failed >&2\nexit 1\n"),
    );
    let result = apply_latest(&mut engine).await.unwrap();

    assert!(
        matches!(result, ApplyResult::RolledBack { .. }),
        "{result:?}"
    );
    assert_eq!(fx.live_version().as_deref(), Some("1.0.0"));
}

#[tokio::test]
async fn a_passing_hook_does_not_block_the_update() {
    let fx = Fixture::new();
    fx.publish("1.0.0", Some("#!/bin/sh\nexit 0\n"));
    let mut engine = fx.engine_healthy();

    let result = apply_latest(&mut engine).await.unwrap();
    assert!(matches!(result, ApplyResult::Applied { .. }), "{result:?}");
}

#[tokio::test]
async fn rolls_back_when_the_health_probe_hangs() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    fx.publish("1.1.0", None);
    // Socket open, no reply — the case that stalls naive clients. A timeout must
    // count as failure: unproven is not healthy.
    let mut engine = fx.engine(
        Box::new(FakeRobot::healthy()),
        Faults {
            hang_health: true,
            ..Faults::none()
        },
        "",
    );
    let result = apply_latest(&mut engine).await.unwrap();
    assert!(
        matches!(result, ApplyResult::RolledBack { .. }),
        "{result:?}"
    );
    assert_eq!(fx.live_version().as_deref(), Some("1.0.0"));
}

/// The worst case: the update failed *and* recovery failed. It must surface
/// distinctly, not as an ordinary failure.
#[tokio::test]
async fn failed_rollback_is_reported_distinctly() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    fx.publish("1.1.0", None);
    let mut engine = fx.engine(
        Box::new(FakeRobot::unhealthy()),
        Faults {
            fail_rollback: true,
            ..Faults::none()
        },
        "",
    );
    let err = apply_latest(&mut engine).await.unwrap_err();

    assert!(
        matches!(err, updater::Error::RollbackFailed(_)),
        "got {err:?}"
    );
    assert_eq!(
        err.code(),
        updater::proto::code::ROLLBACK_FAILED,
        "support must be able to spot this immediately"
    );
}

/// The case the one above used to swallow: the swap succeeded and the *restart* did not.
///
/// Reachable without a fault, and `scripts/systemd-test.sh` reaches it — `hooks/postinstall`
/// overwrites unit files and by design does not put them back, so a release that fails *because* one
/// of its units cannot start leaves that file behind, and the revert's own restart of a unit with the
/// same name inherits the same failure.
///
/// Reporting that as `RollbackFailed` asserted the one thing that was false — the robot had been put
/// back — and buried the one thing that was true, which is that a daemon is down. Both facts, in the
/// outcome.
#[tokio::test]
async fn a_revert_whose_restart_fails_is_still_a_revert() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    fx.publish("1.1.0", None);
    let mut engine = fx.engine(
        Box::new(FakeRobot::unhealthy()),
        Faults {
            fail_rollback_apply: true,
            ..Faults::none()
        },
        "",
    );

    let outcome = apply_latest(&mut engine)
        .await
        .expect("a revert that happened is not an error");

    match outcome {
        updater::proto::ApplyResult::RolledBack {
            reverted_to,
            reason,
            ..
        } => {
            assert_eq!(
                reverted_to.as_ref().map(ToString::to_string).as_deref(),
                Some("1.0.0"),
                "the release must be reported as reverted, because it was"
            );
            // Why the update failed comes first — that is what someone is looking for — and the
            // unit that did not come back is a second thing to act on rather than a correction.
            assert!(reason.contains("not healthy"), "{reason}");
            assert!(reason.contains("the release was reverted"), "{reason}");

            // The claim it must not make. Every daemon runs `Restart=always`, so a refused restart
            // is not an outage — asserting one put "something on this robot is down" under a
            // `robot healthy` line on a bench board.
            assert!(
                !reason.contains("is down"),
                "an outage this never checked for: {reason}"
            );
            assert!(reason.contains("robotctl health"), "{reason}");
        }
        other => panic!("expected RolledBack, got {other:?}"),
    }

    assert_eq!(
        fx.live_version().as_deref(),
        Some("1.0.0"),
        "and the tree must actually be back, which is the claim the outcome makes"
    );
}

// ── crash recovery ───────────────────────────────────────────────────────────

/// Simulates `kill -9` right after the symlink swap: the new release is live and the boot counter is
/// still armed. The in-process health gate cannot help — it died with the process — so the boot
/// counter must catch it **when the robot is not working on the new release**.
///
/// The robot is unhealthy here on purpose. A crash after the swap is not by itself evidence against
/// a release, and what happens when the robot *is* fine is
/// [`an_unconfirmed_trial_on_a_healthy_robot_is_committed`].
#[tokio::test]
async fn crash_after_swap_is_reverted_when_the_robot_is_unhealthy() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    fx.publish("1.1.0", None);
    let mut crashing = fx.engine(
        Box::new(FakeRobot::healthy()),
        Faults {
            abort_after_swap: true,
            ..Faults::none()
        },
        "",
    );
    let _ = apply_latest(&mut crashing).await;

    // Post-crash state: 1.1.0 is live but never confirmed healthy.
    assert_eq!(fx.live_version().as_deref(), Some("1.1.0"));

    // Restarts: the first is still "on trial", the second exhausts the budget. The robot does not
    // work on 1.1.0, which is what makes reverting the right answer.
    let mut engine = fx.engine(Box::new(FakeRobot::unhealthy()), Faults::none(), "");
    assert!(engine.recover_on_start().await.unwrap().is_empty());
    let recovered = engine.recover_on_start().await.unwrap();

    assert_eq!(recovered.len(), 1, "should have reverted");
    assert_eq!(fx.live_version().as_deref(), Some("1.0.0"));
    assert_eq!(fx.live_marker().as_deref(), Some("version=1.0.0\n"));
}

/// **A release the robot is healthy on must not be reverted for want of a confirmation.**
///
/// The trial exists to catch a release that does not work. An apply killed before its gate ran —
/// which is what happens when the release's own `hooks/postinstall` restarts `updaterd` — arms a
/// trial nothing will ever confirm, and reverting on the budget alone then swaps working code out
/// from under whoever is using the robot, silently. Observed on a board that had installed a branch
/// build, paired a gamepad on it, and came back two boots later running the stable release.
#[tokio::test]
async fn an_unconfirmed_trial_on_a_healthy_robot_is_committed() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    fx.publish("1.1.0", None);
    let mut crashing = fx.engine(
        Box::new(FakeRobot::healthy()),
        Faults {
            abort_after_swap: true,
            ..Faults::none()
        },
        "",
    );
    let _ = apply_latest(&mut crashing).await;
    assert_eq!(fx.live_version().as_deref(), Some("1.1.0"));

    // Three restarts: more than the budget, so nothing here is the budget merely not having run out.
    let mut engine = fx.engine_healthy();
    for _ in 0..3 {
        assert!(
            engine.recover_on_start().await.unwrap().is_empty(),
            "a healthy robot on the new release is not a rollback"
        );
    }
    assert_eq!(fx.live_version().as_deref(), Some("1.1.0"));
}

/// And the case that prompted all of this: a bench board with no servo power.
///
/// `robotd` comes up and answers, and reports `degraded` because it can see no motor bus. That is a
/// hardware fault. Reverting hides it behind a software change — and reverts the next release too,
/// forever, on a board whose only problem is a switch nobody turned on.
#[tokio::test]
async fn an_unconfirmed_trial_on_a_degraded_robot_is_committed() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    fx.publish("1.1.0", None);
    let mut crashing = fx.engine(
        Box::new(FakeRobot::healthy()),
        Faults {
            abort_after_swap: true,
            ..Faults::none()
        },
        "",
    );
    let _ = apply_latest(&mut crashing).await;
    assert_eq!(fx.live_version().as_deref(), Some("1.1.0"));

    let mut engine = fx.engine(Box::new(DegradedRobot), Faults::none(), "");
    for _ in 0..3 {
        assert!(
            engine.recover_on_start().await.unwrap().is_empty(),
            "no motor bus is a hardware fault, not a failed release"
        );
    }
    assert_eq!(fx.live_version().as_deref(), Some("1.1.0"));
}

#[tokio::test]
async fn recovery_confirms_a_healthy_update_and_does_nothing() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    // A committed update leaves no pending trial, so restarts are no-ops.
    for _ in 0..3 {
        assert!(engine.recover_on_start().await.unwrap().is_empty());
    }
    assert_eq!(fx.live_version().as_deref(), Some("1.0.0"));
}

#[tokio::test]
async fn recovery_cleans_staging_leftovers() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    // Fake an interrupted extract.
    let leftover = fx.install.join("releases/.staging-9.9.9/root");
    std::fs::create_dir_all(&leftover).unwrap();
    assert_eq!(fx.staging_leftovers(), 1);

    engine.recover_on_start().await.unwrap();
    assert_eq!(
        fx.staging_leftovers(),
        0,
        "a kill -9 must not leak disk forever"
    );
}

// ── explicit transitions ─────────────────────────────────────────────────────

#[tokio::test]
async fn explicit_rollback_returns_to_the_previous_release() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    fx.publish("1.1.0", None);
    let mut engine = fx.engine_healthy();

    let (tx, _rx) = progress_channel();
    engine
        .apply(
            "daemon",
            Target::Exact(semver::Version::new(1, 0, 0)),
            ApplyOptions::default(),
            tx,
        )
        .await
        .unwrap();
    apply_latest(&mut engine).await.unwrap();
    assert_eq!(fx.live_version().as_deref(), Some("1.1.0"));

    let result = engine.rollback("daemon").await.unwrap();
    assert!(matches!(result, ApplyResult::Applied { .. }), "{result:?}");
    assert_eq!(fx.live_version().as_deref(), Some("1.0.0"));
}

#[tokio::test]
async fn reset_to_golden_works_when_robotd_is_dead() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    fx.publish("1.1.0", None);

    // Install both so golden is on disk.
    let mut engine = fx.engine(
        Box::new(FakeRobot::healthy()),
        Faults::none(),
        r#"golden = "1.0.0""#,
    );
    let (tx, _rx) = progress_channel();
    engine
        .apply(
            "daemon",
            Target::Exact(semver::Version::new(1, 0, 0)),
            ApplyOptions::default(),
            tx,
        )
        .await
        .unwrap();
    apply_latest(&mut engine).await.unwrap();
    assert_eq!(fx.live_version().as_deref(), Some("1.1.0"));

    // Now robotd is down. Recovery must still work — this is the whole point of the
    // updater not depending on the thing it repairs. It comes back up once golden
    // is linked, which is what the gate then observes.
    let mut engine = fx.engine(
        Box::new(DeadThenHealthy),
        Faults::none(),
        r#"golden = "1.0.0""#,
    );
    let result = engine.reset_to_golden("daemon").await.unwrap();

    assert!(matches!(result, ApplyResult::Applied { .. }), "{result:?}");
    assert_eq!(fx.live_version().as_deref(), Some("1.0.0"));
}

/// A robot that never comes up is a different case: the gate cannot pass, so the
/// engine must report a rollback rather than claim success. Recovery has to be
/// honest about not having worked.
#[tokio::test]
async fn robot_that_never_comes_up_is_reported_not_papered_over() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    fx.publish("1.1.0", None);

    let mut engine = fx.engine_healthy();
    let (tx, _rx) = progress_channel();
    engine
        .apply(
            "daemon",
            Target::Exact(semver::Version::new(1, 0, 0)),
            ApplyOptions::default(),
            tx,
        )
        .await
        .unwrap();
    apply_latest(&mut engine).await.unwrap();

    let mut engine = fx.engine(Box::new(AbsentRobot), Faults::none(), r#"golden = "1.0.0""#);
    let result = engine.reset_to_golden("daemon").await.unwrap();

    assert!(
        matches!(result, ApplyResult::RolledBack { .. }),
        "unreachable health must fail the gate, got {result:?}"
    );
}

#[tokio::test]
async fn select_activates_an_installed_release() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    fx.publish("1.1.0", None);
    let mut engine = fx.engine_healthy();

    let (tx, _rx) = progress_channel();
    engine
        .apply(
            "daemon",
            Target::Exact(semver::Version::new(1, 0, 0)),
            ApplyOptions::default(),
            tx,
        )
        .await
        .unwrap();
    apply_latest(&mut engine).await.unwrap();

    let result = engine
        .select("daemon", &semver::Version::new(1, 0, 0))
        .await
        .unwrap();
    assert!(matches!(result, ApplyResult::Applied { .. }), "{result:?}");
    assert_eq!(fx.live_version().as_deref(), Some("1.0.0"));
}

#[tokio::test]
async fn select_refuses_a_version_that_is_not_installed() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    assert!(
        engine
            .select("daemon", &semver::Version::new(9, 9, 9))
            .await
            .is_err()
    );
}

// ── bookkeeping ──────────────────────────────────────────────────────────────

#[tokio::test]
async fn prune_keeps_the_configured_number_of_previous_releases() {
    let fx = Fixture::new();
    let mut engine = fx.engine_healthy();

    for v in ["1.0.0", "1.1.0", "1.2.0", "1.3.0"] {
        fx.publish(v, None);
        apply_latest(&mut engine).await.unwrap();
    }

    // keep_previous defaults to 1, so only the live release plus one survive.
    let installed = engine.list_installed("daemon").unwrap();
    assert_eq!(installed.len(), 2, "{installed:?}");
    assert!(installed.iter().any(|r| r.active));
}

#[tokio::test]
async fn golden_is_never_pruned() {
    let fx = Fixture::new();
    let mut engine = fx.engine(
        Box::new(FakeRobot::healthy()),
        Faults::none(),
        r#"golden = "1.0.0""#,
    );

    for v in ["1.0.0", "1.1.0", "1.2.0", "1.3.0"] {
        fx.publish(v, None);
        apply_latest(&mut engine).await.unwrap();
    }

    assert!(
        fx.release_exists("1.0.0"),
        "the never-brick guarantee must not expire as versions accumulate"
    );
    let installed = engine.list_installed("daemon").unwrap();
    assert!(installed.iter().any(|r| r.golden));
}

#[tokio::test]
async fn log_records_success_rollback_and_refusal() {
    let fx = Fixture::new();

    // Success.
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    // Refusal (tampered).
    fx.publish("1.1.0", None);
    fx.tamper("1.1.0");
    let _ = apply_latest(&mut engine).await;

    // Rollback (unhealthy).
    fx.publish("1.2.0", None);
    let mut sick = fx.engine(Box::new(FakeRobot::unhealthy()), Faults::none(), "");
    let _ = apply_latest(&mut sick).await;

    let log = engine.log(10).unwrap();
    let kinds: Vec<_> = log
        .iter()
        .map(|e| match &e.outcome {
            Outcome::Success => "success",
            Outcome::RolledBack { .. } => "rolled_back",
            Outcome::Aborted { .. } => "aborted",
        })
        .collect();

    // Newest first. All three classes must be present — a log of only successes is
    // useless to support.
    assert!(kinds.contains(&"success"), "{kinds:?}");
    assert!(kinds.contains(&"aborted"), "{kinds:?}");
    assert!(kinds.contains(&"rolled_back"), "{kinds:?}");
}

#[tokio::test]
async fn concurrent_updates_report_busy_rather_than_colliding() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);

    let mut first = fx.engine_healthy();
    let mut second = fx.engine_healthy();

    // Hold the lock by starting an update that blocks in the health gate.
    let mut blocking = fx.engine(
        Box::new(FakeRobot::healthy()),
        Faults {
            hang_health: true,
            ..Faults::none()
        },
        "",
    );

    let (tx, _rx) = progress_channel();
    let handle = tokio::spawn(async move {
        blocking
            .apply("daemon", Target::Latest, ApplyOptions::default(), tx)
            .await
    });

    // Give it time to take the lock and reach the gate.
    tokio::time::sleep(std::time::Duration::from_millis(300)).await;

    let err = apply_latest(&mut second).await.unwrap_err();
    assert!(matches!(err, updater::Error::Busy), "got {err:?}");

    let _ = handle.await;
    // And once it's released, the lock is usable again.
    assert!(apply_latest(&mut first).await.is_ok());
}

#[tokio::test]
async fn unknown_component_is_rejected() {
    let fx = Fixture::new();
    let engine = fx.engine_healthy();
    assert!(matches!(
        engine.check("nope").await.unwrap_err(),
        updater::Error::UnknownComponent(_)
    ));
}

#[tokio::test]
async fn mandatory_flag_is_surfaced_by_check() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    // 1.2.0 declares that anything below it must upgrade.
    fx.publish_with("1.2.0", None, |m| {
        m["min_supported"] = serde_json::json!("1.2.0");
    });

    match engine.check("daemon").await.unwrap() {
        CheckResult::Available { mandatory, .. } => {
            assert!(mandatory, "floor must make this non-optional");
        }
        other => panic!("expected Available, got {other:?}"),
    }
}

/// The embedded manifest gives provenance without a network round-trip.
#[tokio::test]
async fn installed_release_reports_source_revision() {
    let fx = Fixture::new();
    fx.publish_with("1.0.0", None, |m| {
        m["source_revision"] = serde_json::json!("abc1234");
    });
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    let installed = engine.list_installed("daemon").unwrap();
    assert_eq!(
        installed[0].source_revision.as_deref(),
        Some("abc1234"),
        "needed to reproduce a client's exact build in the lab"
    );
}

fn _unused(_: &Path) {}

// ── regressions ──────────────────────────────────────────────────────────────
//
// One test per bug found in review. Each reproduces the original failure, so a
// regression shows up as a specific named failure rather than a vague one.

/// **#1** `rollback` walked *forward* onto the release that had just failed.
///
/// After an auto-rollback the bad release is still on disk — the failure path
/// deliberately doesn't prune — so "newest that isn't current" picked it. That made
/// the one command support reaches for after a bad update reinstall the bad update.
#[tokio::test]
async fn rollback_does_not_walk_forward_onto_the_failed_release() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    // 1.1.0 fails its gate and is auto-rolled-back, but stays on disk.
    fx.publish("1.1.0", None);
    let mut sick = fx.engine(Box::new(FakeRobot::unhealthy()), Faults::none(), "");
    let result = apply_latest(&mut sick).await.unwrap();
    assert!(
        matches!(result, ApplyResult::RolledBack { .. }),
        "{result:?}"
    );
    assert!(
        fx.release_exists("1.1.0"),
        "precondition: bad release still present"
    );
    assert_eq!(fx.live_version().as_deref(), Some("1.0.0"));

    // An explicit rollback must not land back on 1.1.0. With only 1.0.0 below it and
    // 1.0.0 already live, there is nowhere older to go.
    let err = engine.rollback("daemon").await.unwrap_err();
    assert_eq!(fx.live_version().as_deref(), Some("1.0.0"), "must not move");
    assert!(
        err.to_string().contains("no older, known-good release"),
        "got: {err}"
    );
}

/// **#1b** Rollback skips a release the journal recorded as rolled back, even when
/// it is the newest one below current.
#[tokio::test]
async fn rollback_skips_known_bad_releases() {
    let fx = Fixture::new();
    for v in ["1.0.0", "1.1.0", "1.2.0"] {
        fx.publish(v, None);
    }
    // keep_previous is raised so the older good release survives pruning — the
    // point of the test is which target rollback *chooses*, not what prune keeps.
    let keep = "keep_previous = 5";
    let mut engine = fx.engine(Box::new(FakeRobot::healthy()), Faults::none(), keep);

    // Install 1.0.0 then 1.2.0, so 1.1.0 can be installed-but-bad in between.
    apply_exact(&mut engine, "1.0.0").await.unwrap();

    // 1.1.0 fails: recorded as rolled back, left on disk.
    let mut sick = fx.engine(Box::new(FakeRobot::unhealthy()), Faults::none(), keep);
    let result = apply_exact(&mut sick, "1.1.0").await.unwrap();
    assert!(
        matches!(result, ApplyResult::RolledBack { .. }),
        "{result:?}"
    );

    apply_exact(&mut engine, "1.2.0").await.unwrap();
    assert_eq!(fx.live_version().as_deref(), Some("1.2.0"));

    // Newest below current is 1.1.0, but it is known-bad, so 1.0.0 wins.
    engine.rollback("daemon").await.unwrap();
    assert_eq!(fx.live_version().as_deref(), Some("1.0.0"));
}

/// **#3** An unrecoverable trial was re-reported on every boot, appending a bogus
/// rollback entry each time and never converging.
#[tokio::test]
async fn unrecoverable_first_install_reports_stuck_once() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);

    // First install, no previous release, no golden — and it crashes after the swap.
    let mut crashing = fx.engine(
        Box::new(FakeRobot::healthy()),
        Faults {
            abort_after_swap: true,
            ..Faults::none()
        },
        "",
    );
    let _ = apply_latest(&mut crashing).await;

    // Unhealthy, because the subject is what happens when a revert is warranted and there is
    // nowhere to revert *to*. On a robot that works, the trial is committed and there is nothing to
    // report — see `an_unconfirmed_trial_on_a_healthy_robot_is_committed`.
    let mut engine = fx.engine(Box::new(FakeRobot::unhealthy()), Faults::none(), "");
    assert!(
        engine.recover_on_start().await.unwrap().is_empty(),
        "on trial"
    );

    // Budget exhausted: reported as Stuck, because nothing was reverted. Calling it
    // a rollback would be a lie.
    let outcomes = engine.recover_on_start().await.unwrap();
    assert_eq!(outcomes.len(), 1);
    assert!(
        matches!(&outcomes[0], ApplyResult::Stuck { .. }),
        "got {:?}",
        outcomes[0]
    );

    // And it must not be re-reported forever.
    for _ in 0..3 {
        assert!(
            engine.recover_on_start().await.unwrap().is_empty(),
            "a stuck update must be reported once, not on every boot"
        );
    }
    assert!(
        !fx.pending_file_exists(),
        "the trial must be cleared so it stops repeating"
    );
}

/// **#4** The boot counter was a single global slot, so any other component's
/// transition destroyed a daemon update's trial — losing exactly the record the
/// never-brick guarantee rests on.
#[tokio::test]
async fn a_second_component_does_not_consume_the_daemon_trial() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    fx.publish_model("3.0.0");

    let extra = format!(
        r#"
[component.model]
install_dir = "{}"
source = {{ type = "local_dir", path = "{}" }}
on_apply = {{ action = "none" }}
health = {{ probe = "none" }}
"#,
        fx.root.join("opt/robot/model").display(),
        fx.model_releases().display(),
    );

    // Daemon: install 1.0.0, then crash after the swap of 1.1.0.
    let mut engine = fx.engine(Box::new(FakeRobot::healthy()), Faults::none(), &extra);
    apply_latest(&mut engine).await.unwrap();

    fx.publish("1.1.0", None);
    let mut crashing = fx.engine(
        Box::new(FakeRobot::healthy()),
        Faults {
            abort_after_swap: true,
            ..Faults::none()
        },
        &extra,
    );
    let _ = apply_latest(&mut crashing).await;
    assert_eq!(fx.live_version().as_deref(), Some("1.1.0"));

    // Now the model updates successfully. Under the old single-slot counter this
    // silently cleared the daemon's trial.
    let mut engine = fx.engine(Box::new(FakeRobot::healthy()), Faults::none(), &extra);
    let (tx, _rx) = progress_channel();
    engine
        .apply("model", Target::Latest, ApplyOptions::default(), tx)
        .await
        .unwrap();

    // The daemon trial must have survived, and still revert. A fresh engine with an *unhealthy*
    // robot, because what is being asserted here is that the trial still exists and is still acted
    // on — and a robot that works on the new release is committed rather than reverted, which would
    // pass this assertion for the wrong reason. See
    // `an_unconfirmed_trial_on_a_healthy_robot_is_committed`.
    let mut engine = fx.engine(Box::new(FakeRobot::unhealthy()), Faults::none(), &extra);
    assert!(
        engine.recover_on_start().await.unwrap().is_empty(),
        "on trial"
    );
    let outcomes = engine.recover_on_start().await.unwrap();
    assert_eq!(outcomes.len(), 1, "daemon trial was lost: {outcomes:?}");
    assert_eq!(fx.live_version().as_deref(), Some("1.0.0"));
}

/// **#5** `transition_to` armed the boot counter before validating the target, so a
/// failed swap left a trial for a version that was never live — self-healing later
/// only via a spurious rollback and a bogus log entry.
#[tokio::test]
async fn failed_transition_leaves_no_armed_trial() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    // Golden is configured but was never installed.
    let mut engine = fx.engine(
        Box::new(FakeRobot::healthy()),
        Faults::none(),
        r#"golden = "9.9.9""#,
    );
    let err = engine.reset_to_golden("daemon").await.unwrap_err();

    assert_eq!(
        err.code(),
        updater::proto::code::NOT_INSTALLED,
        "got {err:?}"
    );
    assert!(
        !fx.pending_file_exists(),
        "a refused transition must not leave a trial armed"
    );
    assert_eq!(fx.live_version().as_deref(), Some("1.0.0"));

    // And a later boot must not invent a recovery from it.
    assert!(engine.recover_on_start().await.unwrap().is_empty());
}

/// `scripts/robot-rescue` runs when `updaterd` does not, so it cannot ask this process which
/// release is golden and must not parse `updater.toml` to find out — a release whose `updaterd`
/// rejects that file is the likeliest thing it exists to rescue. Every start publishes the answer
/// as a symlink instead.
#[tokio::test]
async fn starting_up_publishes_golden_as_a_symlink() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    fx.publish("1.1.0", None);

    let mut engine = fx.engine(
        Box::new(FakeRobot::healthy()),
        Faults::none(),
        r#"golden = "1.0.0""#,
    );
    apply_exact(&mut engine, "1.0.0").await.unwrap();
    apply_latest(&mut engine).await.unwrap();
    assert_eq!(fx.live_version().as_deref(), Some("1.1.0"));

    engine.recover_on_start().await.unwrap();

    assert_eq!(
        std::fs::read_link(fx.install.join("golden")).unwrap(),
        std::path::Path::new("releases/1.0.0"),
        "golden must be published relative, like `current`, so the tree survives a moved mount"
    );
}

/// `robot-rescue` swaps `current` and reboots with no daemon running, so this start is the first
/// moment anything can write that down where it will be found — and the breadcrumb it left is also
/// the rescue's loop guard, which stays shut until this clears it.
#[tokio::test]
async fn starting_up_records_a_rescue_and_releases_its_guard() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    fx.publish("1.1.0", None);

    let mut engine = fx.engine(
        Box::new(FakeRobot::healthy()),
        Faults::none(),
        r#"golden = "1.0.0""#,
    );
    apply_exact(&mut engine, "1.0.0").await.unwrap();
    apply_latest(&mut engine).await.unwrap();

    // What the rescue leaves behind after putting the board back on golden.
    let breadcrumb = fx.root.join("var/lib/robot/updater/rescued");
    std::fs::write(
        &breadcrumb,
        format!(
            "at=1786453421\ninstall_dir={}\nfrom=1.1.0\nto=1.0.0\nbecause=boot check: \
             robotd.service (failed, 7 restarts)\n",
            fx.install.display()
        ),
    )
    .unwrap();

    let mut engine = fx.engine(
        Box::new(FakeRobot::healthy()),
        Faults::none(),
        r#"golden = "1.0.0""#,
    );
    engine.recover_on_start().await.unwrap();

    assert!(
        !breadcrumb.exists(),
        "the breadcrumb is the rescue's loop guard; not clearing it leaves the board unable to \
         rescue itself again"
    );

    let entry = engine
        .log(1)
        .unwrap()
        .into_iter()
        .next()
        .expect("an entry in the update log");
    assert_eq!(entry.component.0, "daemon", "matched by install_dir");
    assert_eq!(entry.from, Some(semver::Version::new(1, 1, 0)));
    assert_eq!(entry.to, Some(semver::Version::new(1, 0, 0)));
    match entry.outcome {
        updater::proto::Outcome::RolledBack { reason } => assert!(
            reason.contains("robotd.service"),
            "the reason the check gave has to survive into the log: {reason}"
        ),
        other => panic!("a rescue is a rollback, got {other:?}"),
    }
}

/// A rescue outranks an armed trial, and the order inside `recover_on_start` is what enforces it.
///
/// The trial would revert to `previous`; the rescue already went further, to golden. Left armed, the
/// boot counter would advance it and eventually move `current` *off* the release the rescue chose —
/// the recovery net and the never-brick guarantee undoing each other, one boot apart.
#[tokio::test]
async fn a_rescue_supersedes_the_trial_it_interrupted() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);

    let golden = r#"golden = "1.0.0""#;
    let mut engine = fx.engine(Box::new(FakeRobot::healthy()), Faults::none(), golden);
    apply_latest(&mut engine).await.unwrap();

    // 1.1.0 goes in and the process dies after the swap, which is what arms a trial.
    fx.publish("1.1.0", None);
    let mut crashing = fx.engine(
        Box::new(FakeRobot::healthy()),
        Faults {
            abort_after_swap: true,
            ..Faults::none()
        },
        golden,
    );
    let _ = apply_latest(&mut crashing).await;
    assert_eq!(fx.live_version().as_deref(), Some("1.1.0"));
    assert!(fx.pending_file_exists(), "a trial should be armed");

    // Now 1.1.0's daemons do not come up, and the rescue puts the board on golden without any of
    // this code running: it swaps the symlink and leaves a breadcrumb.
    let current = fx.install.join("current");
    std::fs::remove_file(&current).unwrap();
    std::os::unix::fs::symlink("releases/1.0.0", &current).unwrap();
    std::fs::write(
        fx.root.join("var/lib/robot/updater/rescued"),
        format!(
            "at=1786453421\ninstall_dir={}\nfrom=1.1.0\nto=1.0.0\nbecause=boot check\n",
            fx.install.display()
        ),
    )
    .unwrap();

    // Two starts, because `exhausted` is `boots >= 2`: the first would only count, the second would
    // act. Neither may move the release.
    let mut engine = fx.engine(Box::new(FakeRobot::healthy()), Faults::none(), golden);
    assert!(engine.recover_on_start().await.unwrap().is_empty());
    assert!(
        !fx.pending_file_exists(),
        "the rescue already decided; leaving the trial armed lets the boot counter overrule it"
    );
    assert!(engine.recover_on_start().await.unwrap().is_empty());
    assert_eq!(
        fx.live_version().as_deref(),
        Some("1.0.0"),
        "the board must still be on the golden release the rescue chose"
    );
}

/// A configured golden that was never installed is not a rollback target. A link pointing at it
/// would tell the rescue otherwise, and swapping onto it leaves a board that can exec nothing.
#[tokio::test]
async fn a_golden_that_is_not_installed_is_not_published() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);

    let mut engine = fx.engine(
        Box::new(FakeRobot::healthy()),
        Faults::none(),
        r#"golden = "9.9.9""#,
    );
    apply_latest(&mut engine).await.unwrap();

    engine.recover_on_start().await.unwrap();

    assert!(
        !fx.install.join("golden").exists(),
        "a dangling golden link is worse than none"
    );
}

/// **#7** `select` on a version that isn't installed reported UNKNOWN_COMPONENT,
/// telling clients the wrong thing — the component exists, the version doesn't.
#[tokio::test]
async fn select_missing_version_reports_not_installed() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    let err = engine
        .select("daemon", &semver::Version::new(9, 9, 9))
        .await
        .unwrap_err();
    assert_eq!(err.code(), updater::proto::code::NOT_INSTALLED);
    assert_ne!(err.code(), updater::proto::code::UNKNOWN_COMPONENT);

    // A genuinely unknown component still reports UNKNOWN_COMPONENT.
    let err = engine
        .select("nope", &semver::Version::new(1, 0, 0))
        .await
        .unwrap_err();
    assert_eq!(err.code(), updater::proto::code::UNKNOWN_COMPONENT);
}

/// **#8** Nothing refused a downgrade, so a stale or reverted mirror serving an old
/// but still-validly-signed manifest would walk the fleet backwards — the classic
/// rollback attack on a signed-artifact scheme.
#[tokio::test]
async fn latest_refuses_to_downgrade() {
    let fx = Fixture::new();
    fx.publish("2.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    // The mirror reverts to only offering 1.0.0 — properly signed, just old.
    fx.unpublish("2.0.0");
    fx.publish("1.0.0", None);

    let err = apply_latest(&mut engine).await.unwrap_err();
    assert_eq!(
        err.code(),
        updater::proto::code::WOULD_DOWNGRADE,
        "got {err:?}"
    );
    assert_eq!(fx.live_version().as_deref(), Some("2.0.0"));

    // `check` reports it rather than offering it as an available update.
    match engine.check("daemon").await.unwrap() {
        CheckResult::Incompatible { reason, .. } => {
            assert!(reason.contains("downgrade"), "got: {reason}");
        }
        other => panic!("expected Incompatible, got {other:?}"),
    }
}

/// **#8b** An *explicit* older version is still allowed: that is how a targeted
/// revert works, and it is a deliberate operator action rather than something a
/// mirror can induce.
#[tokio::test]
async fn exact_version_may_still_go_backwards() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    fx.publish("2.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();
    assert_eq!(fx.live_version().as_deref(), Some("2.0.0"));

    apply_exact(&mut engine, "1.0.0").await.unwrap();
    assert_eq!(fx.live_version().as_deref(), Some("1.0.0"));
}

// ── §16.2 Tier-1 gaps closed ────────────────────────────────────────────────

/// **§5.7** Robot-specific state must survive an update *and* a rollback.
///
/// Tier-1 in §16.2. This is the promise that
/// a client never has to recalibrate because of an update.
#[tokio::test]
async fn robot_specific_state_survives_update_and_rollback() {
    let fx = Fixture::new();

    // State that belongs to *this* robot, deliberately outside any release dir.
    let etc = fx.root.join("etc/robot");
    let var = fx.root.join("var/lib/robot");
    std::fs::create_dir_all(&etc).unwrap();
    std::fs::create_dir_all(&var).unwrap();
    std::fs::write(etc.join("imu_calibration.bin"), b"calibration").unwrap();
    std::fs::write(var.join("maploc_map.bin"), b"learned-map").unwrap();
    std::fs::write(etc.join("prefs.toml"), b"name = \"Ducky\"\n").unwrap();

    let intact = |stage: &str| {
        assert_eq!(
            std::fs::read(etc.join("imu_calibration.bin")).unwrap(),
            b"calibration",
            "calibration lost after {stage}"
        );
        assert_eq!(
            std::fs::read(var.join("maploc_map.bin")).unwrap(),
            b"learned-map",
            "learned state lost after {stage}"
        );
        assert_eq!(
            std::fs::read(etc.join("prefs.toml")).unwrap(),
            b"name = \"Ducky\"\n",
            "user prefs lost after {stage}"
        );
    };

    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();
    intact("first install");

    fx.publish("1.1.0", None);
    apply_latest(&mut engine).await.unwrap();
    intact("upgrade");

    engine.rollback("daemon").await.unwrap();
    assert_eq!(fx.live_version().as_deref(), Some("1.0.0"));
    intact("rollback");

    // And through a failed update, which is when a naive implementation would clean
    // up too aggressively.
    fx.publish("1.2.0", None);
    let mut sick = fx.engine(Box::new(FakeRobot::unhealthy()), Faults::none(), "");
    let result = apply_latest(&mut sick).await.unwrap();
    assert!(
        matches!(result, ApplyResult::RolledBack { .. }),
        "{result:?}"
    );
    intact("auto-rollback");
}

/// **§9** A `schema_version` bump runs the hook with both old and new versions, so a
/// migration can tell which transition it is in.
#[tokio::test]
async fn schema_version_bump_gives_the_hook_both_versions() {
    let fx = Fixture::new();
    let migrated = fx.root.join("etc/robot/migrated.txt");
    std::fs::create_dir_all(migrated.parent().unwrap()).unwrap();

    fx.publish_with("1.0.0", None, |m| {
        m["schema_version"] = serde_json::json!(1);
    });
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    // 1.1.0 bumps the schema and migrates, recording what it saw.
    let hook = format!(
        "#!/bin/sh\nprintf '%s->%s\\n' \"$UPDATE_OLD_SCHEMA_VERSION\" \
         \"$UPDATE_NEW_SCHEMA_VERSION\" > {}\n",
        migrated.display()
    );
    fx.publish_with("1.1.0", Some(&hook), |m| {
        m["schema_version"] = serde_json::json!(2);
    });

    apply_latest(&mut engine).await.unwrap();

    assert_eq!(
        std::fs::read_to_string(&migrated).unwrap().trim(),
        "1->2",
        "the hook must see both schema versions to know which migration to run"
    );
}

/// A schema bump must be **installable**, not refused.
///
/// Gating on `schema_version` looks prudent and is self-defeating: the engine judging
/// a manifest is always the previous release's engine, so refusing a higher schema
/// would make every bump undeliverable — including the release carrying the engine
/// that understands it. The hook does the migration; the engine only passes the
/// numbers through.
#[tokio::test]
async fn a_schema_bump_is_deliverable() {
    let fx = Fixture::new();
    fx.publish_with("2.0.0", None, |m| {
        m["schema_version"] = serde_json::json!(99);
    });

    let mut engine = fx.engine_healthy();
    let result = apply_latest(&mut engine).await.unwrap();
    assert!(matches!(result, ApplyResult::Applied { .. }), "{result:?}");
    assert_eq!(fx.live_version().as_deref(), Some("2.0.0"));
}

/// A pin survives being read back, and blocks an otherwise-valid update.
#[tokio::test]
async fn pin_persists_and_blocks_updates() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    engine
        .pin("daemon", Some(semver::Version::new(1, 0, 0)))
        .await
        .unwrap();

    fx.publish("1.1.0", None);
    let err = apply_latest(&mut engine).await.unwrap_err();
    assert!(err.to_string().contains("pinned"), "got {err}");

    // Visible in status, and it outlives this Engine instance.
    let fresh = fx.engine_healthy();
    let status = fresh.status().await.unwrap();
    assert_eq!(status[0].pinned, Some(semver::Version::new(1, 0, 0)));

    // Unpinning lets the update through.
    engine.pin("daemon", None).await.unwrap();
    apply_latest(&mut engine).await.unwrap();
    assert_eq!(fx.live_version().as_deref(), Some("1.1.0"));
}

/// A pin nothing can satisfy would be an invisible update freeze.
#[tokio::test]
async fn pin_refuses_an_unobtainable_version() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    let err = engine
        .pin("daemon", Some(semver::Version::new(7, 7, 7)))
        .await
        .unwrap_err();
    assert_eq!(
        err.code(),
        updater::proto::code::NOT_INSTALLED,
        "got {err:?}"
    );
}

/// Recovery must escalate to golden when the recorded `previous` is no longer on
/// disk, or a robot whose previous release has been pruned never reaches golden
/// (§8.2).
#[tokio::test]
async fn recovery_escalates_to_golden_when_previous_is_gone() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    fx.publish("1.1.0", None);
    let golden = "golden = \"1.0.0\"";

    let mut engine = fx.engine(Box::new(FakeRobot::healthy()), Faults::none(), golden);
    apply_exact(&mut engine, "1.0.0").await.unwrap();
    apply_exact(&mut engine, "1.1.0").await.unwrap();

    // Crash after swapping to a third release, so `previous` is recorded as 1.1.0.
    fx.publish("1.2.0", None);
    let mut crashing = fx.engine(
        Box::new(FakeRobot::healthy()),
        Faults {
            abort_after_swap: true,
            ..Faults::none()
        },
        golden,
    );
    let _ = apply_latest(&mut crashing).await;

    // Now 1.1.0 disappears (pruned, or a corrupted directory removed by hand).
    std::fs::remove_dir_all(fx.install.join("releases/1.1.0")).unwrap();

    // Unhealthy on purpose: the subject here is *where* a revert lands, so there has to be one.
    // A robot that works on the new release is committed instead — see
    // `an_unconfirmed_trial_on_a_healthy_robot_is_committed`.
    let mut engine = fx.engine(Box::new(FakeRobot::unhealthy()), Faults::none(), golden);
    engine.recover_on_start().await.unwrap();
    engine.recover_on_start().await.unwrap();

    assert_eq!(
        fx.live_version().as_deref(),
        Some("1.0.0"),
        "recovery should escalate to golden when previous is gone"
    );
}

/// **Journal `to` semantics.** Three sites wrote rollback entries and disagreed about
/// what `to` meant: `apply` recorded the version that *failed*, while `select` /
/// `rollback` / `reset-to-golden` / `recover_on_start` recorded the version landed
/// *on*. `known_bad` reads that field, so the second group blacklisted the
/// known-**good** release — and a later `rollback` then refused to land on it.
///
/// No test covered a *failing* select, which is why it survived.
#[tokio::test]
async fn a_failed_select_does_not_blacklist_the_release_it_reverted_to() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    fx.publish("1.1.0", None);
    let keep = "keep_previous = 5";

    let mut engine = fx.engine(Box::new(FakeRobot::healthy()), Faults::none(), keep);
    apply_exact(&mut engine, "1.0.0").await.unwrap();
    apply_exact(&mut engine, "1.1.0").await.unwrap();
    assert_eq!(fx.live_version().as_deref(), Some("1.1.0"));

    // Select back to 1.0.0, but the robot comes up unhealthy, so it reverts to 1.1.0.
    let mut sick = fx.engine(Box::new(FakeRobot::unhealthy()), Faults::none(), keep);
    let result = sick
        .select("daemon", &semver::Version::new(1, 0, 0))
        .await
        .unwrap();
    match &result {
        ApplyResult::RolledBack {
            attempted,
            reverted_to,
            ..
        } => {
            assert_eq!(*attempted, semver::Version::new(1, 0, 0));
            assert_eq!(*reverted_to, Some(semver::Version::new(1, 1, 0)));
        }
        other => panic!("expected RolledBack, got {other:?}"),
    }
    assert_eq!(fx.live_version().as_deref(), Some("1.1.0"));

    // 1.1.0 is the release we are *running* and it is healthy. It must not have been
    // recorded as bad.
    let bad = engine.known_bad("daemon");
    assert!(
        !bad.contains(&semver::Version::new(1, 1, 0)),
        "the release we reverted TO must not be blacklisted; known_bad = {bad:?}"
    );
    assert!(
        bad.contains(&semver::Version::new(1, 0, 0)),
        "the release that FAILED should be blacklisted; known_bad = {bad:?}"
    );
}

/// The same disagreement, seen through its user-visible consequence: `rollback` must
/// still be able to land on a release that a failed `select` merely reverted to.
#[tokio::test]
async fn rollback_still_works_after_a_failed_select() {
    let fx = Fixture::new();
    for v in ["1.0.0", "1.1.0", "1.2.0"] {
        fx.publish(v, None);
    }
    let keep = "keep_previous = 5";

    // All three must be *installed* for `select` to be able to reach 1.1.0 —
    // publishing alone would make select return NotInstalled instead.
    let mut engine = fx.engine(Box::new(FakeRobot::healthy()), Faults::none(), keep);
    apply_exact(&mut engine, "1.0.0").await.unwrap();
    apply_exact(&mut engine, "1.1.0").await.unwrap();
    apply_exact(&mut engine, "1.2.0").await.unwrap();

    // A failed select to 1.1.0 reverts to 1.2.0.
    let mut sick = fx.engine(Box::new(FakeRobot::unhealthy()), Faults::none(), keep);
    let _ = sick.select("daemon", &semver::Version::new(1, 1, 0)).await;
    assert_eq!(fx.live_version().as_deref(), Some("1.2.0"));

    // Rolling back must reach 1.0.0. Under the bug, 1.2.0 was blacklisted (harmless
    // here, it's current) *and* the entry claimed 1.2.0 rolled back, which is the
    // wrong story for support to read.
    engine.rollback("daemon").await.unwrap();
    assert_eq!(fx.live_version().as_deref(), Some("1.0.0"));

    // And the log must attribute the failure to 1.1.0, not to 1.2.0.
    let log = engine.log(10).unwrap();
    let rollback_entry = log
        .iter()
        .find(|e| matches!(e.outcome, Outcome::RolledBack { .. }))
        .expect("a rollback should be recorded");
    assert_eq!(
        rollback_entry.to,
        Some(semver::Version::new(1, 1, 0)),
        "a RolledBack entry's `to` must name the version that failed"
    );
}

// ── installing by ref (the dev channel, `docs/project/roadmap.md` M2) ────────────────

/// **A ref installs what that branch last built.**
///
/// The version is a semver prerelease, so it sorts *below* the release it precedes — which
/// is the whole point (a dev build can never look like an upgrade for the fleet) and also
/// why this needs the downgrade guard to stand aside. See the next test.
#[tokio::test]
async fn a_ref_installs_the_build_it_points_at() {
    let fx = Fixture::new();
    fx.publish("0.2.0-dev.5.abc1234", None);
    fx.point_ref_at("my-branch", "0.2.0-dev.5.abc1234");

    let mut engine = fx.engine_healthy();
    let result = apply_ref(&mut engine, "my-branch").await.unwrap();

    assert!(
        matches!(result, ApplyResult::Applied { .. }),
        "expected Applied, got {result:?}"
    );
    assert_eq!(fx.live_version().as_deref(), Some("0.2.0-dev.5.abc1234"));
}

/// **A ref must bypass the downgrade guard.**
///
/// A robot on 0.2.0 installing `0.2.0-dev.6.…` is moving *backwards* in semver terms, every
/// single time — a prerelease sorts below its release. If the guard applied here, installing
/// a teammate's branch onto any up-to-date board would be refused, and the dev channel would
/// be unusable on exactly the boards people develop on.
#[tokio::test]
async fn a_ref_may_move_backwards_where_latest_may_not() {
    let fx = Fixture::new();
    fx.publish("0.2.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();
    assert_eq!(fx.live_version().as_deref(), Some("0.2.0"));

    // The branch build precedes 0.2.0 in semver order.
    fx.publish("0.2.0-dev.6.def5678", None);
    fx.point_ref_at("my-branch", "0.2.0-dev.6.def5678");

    let result = apply_ref(&mut engine, "my-branch").await.unwrap();
    assert!(
        matches!(result, ApplyResult::Applied { .. }),
        "a ref must not be refused as a downgrade, got {result:?}"
    );
    assert_eq!(fx.live_version().as_deref(), Some("0.2.0-dev.6.def5678"));

    // And a plain `apply` gets the board back onto the release stream, because `latest`
    // resolves to the highest *stable* version and a prerelease sorts below its release.
    // Worth pinning: it means a board carrying someone's branch is one ordinary update away
    // from being a normal robot again, with no special "leave the dev channel" command — and
    // that a dev build never becomes what the fleet sees as latest.
    let back = apply_latest(&mut engine).await.unwrap();
    assert!(
        matches!(back, ApplyResult::Applied { .. }),
        "latest should move a dev board back onto the release, got {back:?}"
    );
    assert_eq!(fx.live_version().as_deref(), Some("0.2.0"));
}

/// A dev build still has to verify. Signing with the team key is a *different key*, never a
/// relaxed check, so a tampered dev artifact is refused exactly like a tampered release.
#[tokio::test]
async fn a_ref_is_verified_like_anything_else() {
    let fx = Fixture::new();
    fx.publish("0.2.0-dev.7.aaaaaaa", None);
    fx.point_ref_at("my-branch", "0.2.0-dev.7.aaaaaaa");
    fx.tamper("0.2.0-dev.7.aaaaaaa");

    let mut engine = fx.engine_healthy();
    let err = apply_ref(&mut engine, "my-branch").await.unwrap_err();

    assert!(
        matches!(err, updater::Error::Verification(_)),
        "expected a verification failure, got {err:?}"
    );
    assert!(fx.live_version().is_none(), "nothing may be installed");
}

/// An unknown ref must fail with something that names the ref, not with a generic error.
#[tokio::test]
async fn an_unknown_ref_says_which_ref() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();

    let err = apply_ref(&mut engine, "no-such-branch").await.unwrap_err();
    let message = err.to_string();
    assert!(
        message.contains("no-such-branch"),
        "the error must name the ref: {message}"
    );
}

/// A ref that would escape the directory is refused. `local_dir` turns a ref into a
/// filename, so this is a path-traversal guard, and it fails as a verification error rather
/// than quietly reading something else.
#[tokio::test]
async fn a_ref_cannot_escape_the_source_directory() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();

    for bad in ["../outside", "sub/dir", ".."] {
        let err = apply_ref(&mut engine, bad).await.unwrap_err();
        assert!(
            matches!(err, updater::Error::Verification(_)),
            "ref {bad:?} must be refused, got {err:?}"
        );
    }
}

// ── installing from a directory on the board (`apply --from`) ────────────────────────

/// Applies a directory the same way `robotctl update apply --from <dir>` does.
async fn apply_from(
    engine: &mut Engine,
    dir: &std::path::Path,
    target: Target,
) -> Result<ApplyResult, updater::Error> {
    let (tx, _rx) = progress_channel();
    engine
        .apply(
            "daemon",
            target,
            ApplyOptions {
                from_dir: Some(dir.to_path_buf()),
                ..Default::default()
            },
            tx,
        )
        .await
}

/// **`--from` reaches a release the configured source cannot see.**
///
/// The laptop path: the build never went to GitHub, so nothing but the directory knows it
/// exists. The second half of the test is the load-bearing one — the same version asked for
/// without `--from` must fail — because it proves the override is what made the build
/// reachable, rather than the fixture having quietly published it everywhere.
#[tokio::test]
async fn from_dir_installs_what_the_configured_source_does_not_have() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();
    assert_eq!(fx.live_version().as_deref(), Some("1.0.0"));

    fx.publish_sideload("2.0.0");

    // Not through the configured source, which has never heard of it.
    let err = apply_exact(&mut engine, "2.0.0").await.unwrap_err();
    assert!(
        matches!(err, updater::Error::Network(_) | updater::Error::Io { .. }),
        "the configured source must not resolve a sideloaded version, got {err:?}"
    );

    let result = apply_from(&mut engine, &fx.sideload(), Target::Latest)
        .await
        .unwrap();
    assert!(
        matches!(result, ApplyResult::Applied { .. }),
        "expected Applied, got {result:?}"
    );
    assert_eq!(fx.live_version().as_deref(), Some("2.0.0"));
}

/// **The health gate still runs, and a bad local build is rolled back.**
///
/// This is the entire reason `apply --from` exists rather than `updaterd install --from`,
/// which has to force the gate off and so can never be used on a live release. A dev board
/// running a build somebody made ten seconds ago is where auto-rollback earns its keep.
#[tokio::test]
async fn a_sideloaded_build_that_does_not_come_up_is_rolled_back() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    fx.publish_sideload("1.1.0");

    // The robot stops answering once the new release is live — a broken local build.
    let mut engine = fx.engine(
        Box::new(FakeRobot::unhealthy()),
        updater::faults::Faults::none(),
        "",
    );
    let result = apply_from(&mut engine, &fx.sideload(), Target::Latest)
        .await
        .unwrap();

    assert!(
        matches!(result, ApplyResult::RolledBack { .. }),
        "expected RolledBack, got {result:?}"
    );
    assert_eq!(
        fx.live_version().as_deref(),
        Some("1.0.0"),
        "the board must be back on what it was running"
    );
}

/// **A local directory is not a relaxed one.** Same signature check, same hash, same refusal.
///
/// Worth pinning separately from the `--ref` case: `--from` is the one path where the bytes
/// never crossed a network, and "it came off my own laptop" is exactly the argument that
/// would justify skipping a check. Nothing skips one.
#[tokio::test]
async fn from_dir_verifies_like_any_download() {
    let fx = Fixture::new();
    fx.publish_sideload("1.0.0");
    fx.publisher.tamper_in(&fx.sideload(), "daemon", "1.0.0");

    let mut engine = fx.engine_healthy();
    let err = apply_from(&mut engine, &fx.sideload(), Target::Latest)
        .await
        .unwrap_err();

    assert!(
        matches!(err, updater::Error::Verification(_)),
        "expected a verification failure, got {err:?}"
    );
    assert!(fx.live_version().is_none(), "nothing may be installed");
}

/// **`--from` bypasses the downgrade guard, and only `--from`.**
///
/// A laptop build is a prerelease of the version it precedes, so on a board that is up to
/// date it sorts *below* what is installed — every push would be refused as a downgrade. The
/// same manifest offered by the configured source is still refused, which is what keeps the
/// guard meaningful: what stands aside is an operator naming a directory, not a mirror that
/// has gone backwards.
#[tokio::test]
async fn from_dir_may_move_backwards_where_latest_may_not() {
    let fx = Fixture::new();
    fx.publish("0.3.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    // What `scripts/dev-push.sh` builds on a board already running the release.
    let local = "0.3.0-dev.local.1754800000.gabc1234";
    fx.publish_sideload(local);

    // The same build offered by the configured source, and nothing newer — a mirror that has
    // gone backwards, which is the case the guard is for.
    fx.unpublish("0.3.0");
    fx.publish(local, None);

    let err = apply_latest(&mut engine).await.unwrap_err();
    assert_eq!(
        err.code(),
        updater::proto::code::WOULD_DOWNGRADE,
        "the guard must still hold for the configured source, got {err:?}"
    );

    let result = apply_from(&mut engine, &fx.sideload(), Target::Latest)
        .await
        .unwrap();
    assert!(
        matches!(result, ApplyResult::Applied { .. }),
        "a named directory must not be refused as a downgrade, got {result:?}"
    );
    assert_eq!(fx.live_version().as_deref(), Some(local));
}

/// A missing directory must say so, and leave the robot alone.
#[tokio::test]
async fn from_dir_that_does_not_exist_fails_before_anything_changes() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    let missing = fx.root.join("var/tmp/not-there");
    let err = apply_from(&mut engine, &missing, Target::Latest)
        .await
        .unwrap_err();

    assert!(
        err.to_string().contains("not-there"),
        "the error must name the directory: {err}"
    );
    assert_eq!(fx.live_version().as_deref(), Some("1.0.0"));
    assert_eq!(fx.staging_leftovers(), 0);
}

// ── the run transcript ───────────────────────────────────────────────────────
//
// `docs/design/updater-design.md` §8.3. The update log says an attempt happened; these cover the
// record of what it *did*, and the link between the two.

/// Every event kind in a transcript, by name, for asserting on shape rather than on wording.
fn event_names(events: &[updater::proto::RunRecord]) -> Vec<String> {
    use updater::proto::RunEvent::*;
    events
        .iter()
        .map(|record| match &record.event {
            Began { .. } => "began".to_owned(),
            Phase { phase, .. } => format!("{phase:?}"),
            Manifest { .. } => "manifest".to_owned(),
            Hook { .. } => "hook".to_owned(),
            Unit { .. } => "unit".to_owned(),
            Health { .. } => "health".to_owned(),
            Note { .. } => "note".to_owned(),
            Ended { .. } => "ended".to_owned(),
            Truncated { .. } => "truncated".to_owned(),
            Unrecognised => "?".to_owned(),
        })
        .collect()
}

/// The link that makes the pair usable: a log entry names the run that explains it.
#[tokio::test]
async fn an_apply_writes_a_transcript_its_log_entry_points_at() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    let entry = engine.log(1).unwrap().pop().expect("an entry");
    let run = entry.run.expect("the entry must name its run");

    let transcript = engine.show(Some(run)).unwrap();
    assert_eq!(transcript.run, run);
    assert_eq!(transcript.component.as_str(), "daemon");

    // The backbone: opened, resolved a manifest, walked the phases, ended.
    let names = event_names(&transcript.events);
    for expected in ["began", "manifest", "Downloading", "Swapping", "ended"] {
        assert!(
            names.contains(&expected.to_owned()),
            "missing {expected} in {names:?}"
        );
    }

    // And the manifest facts are the ones a person is reading it for.
    let manifest = transcript
        .events
        .iter()
        .find_map(|record| match &record.event {
            updater::proto::RunEvent::Manifest {
                version, signed_by, ..
            } => Some((version.clone(), signed_by.clone())),
            _ => None,
        })
        .expect("the manifest must be recorded");
    assert_eq!(manifest.0, semver::Version::new(1, 0, 0));
    assert!(
        manifest.1.is_some(),
        "which key admitted it is the point of recording it"
    );
}

/// With no run named, the most recent — the one someone debugging has just caused.
#[tokio::test]
async fn show_defaults_to_the_most_recent_run() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();
    fx.publish("1.1.0", None);
    apply_latest(&mut engine).await.unwrap();

    let latest = engine.show(None).unwrap();
    assert_eq!(latest.run, 2);
    // And it says what else is there, newest first, so a client need not ask twice.
    assert_eq!(latest.available, vec![2, 1]);
}

/// The rollback case, which is the whole reason for the feature: the gate's verdict and the
/// reason for the revert are both in the run, not only in a journal that may be gone.
#[tokio::test]
async fn a_rolled_back_apply_records_the_gate_that_failed_it() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    fx.publish("1.1.0", None);
    let mut engine = fx.engine(Box::new(FakeRobot::unhealthy()), Faults::none(), "");
    let result = apply_latest(&mut engine).await.unwrap();
    assert!(
        matches!(result, ApplyResult::RolledBack { .. }),
        "{result:?}"
    );

    let transcript = engine.show(None).unwrap();
    let names = event_names(&transcript.events);
    assert!(names.contains(&"RollingBack".to_owned()), "{names:?}");

    let health = transcript
        .events
        .iter()
        .find_map(|record| match &record.event {
            updater::proto::RunEvent::Health { passed, detail } => Some((*passed, detail.clone())),
            _ => None,
        })
        .expect("the gate's verdict must be recorded");
    assert!(!health.0);
    assert!(health.1.is_some(), "a failed gate must say why");

    match &transcript.events.last().unwrap().event {
        updater::proto::RunEvent::Ended { outcome, summary } => {
            assert!(
                matches!(outcome, Some(Outcome::RolledBack { .. })),
                "{outcome:?}"
            );
            assert!(summary.contains("1.1.0"), "{summary}");
        }
        other => panic!("a finished run must end with `ended`, got {other:?}"),
    }
}

/// A refusal is one of the two runs anyone reads, and it must carry what it refused — not just
/// that something was refused.
#[tokio::test]
async fn a_refused_apply_still_records_what_it_refused() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    fx.publish("2.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    // Newest first is 2.0.0; asking for 1.0.0 by `latest` is the downgrade guard's case, so
    // provoke the refusal the way a stale mirror would: pin, then ask for something else.
    engine
        .pin("daemon", Some(semver::Version::new(2, 0, 0)))
        .await
        .unwrap();
    let refused = apply_exact(&mut engine, "1.0.0").await;
    assert!(refused.is_err(), "{refused:?}");

    let transcript = engine.show(None).unwrap();
    let names = event_names(&transcript.events);
    assert!(names.contains(&"manifest".to_owned()), "{names:?}");
    match &transcript.events.last().unwrap().event {
        updater::proto::RunEvent::Ended { summary, .. } => {
            assert!(summary.contains("refused"), "{summary}");
            assert!(summary.contains("pinned"), "{summary}");
        }
        other => panic!("expected an ending, got {other:?}"),
    }
}

/// The hook's output is the richest thing an update produces and used to survive only in the
/// journal, dropped by both call sites.
#[tokio::test]
async fn the_post_install_hook_output_is_kept() {
    let fx = Fixture::new();
    fx.publish(
        "1.0.0",
        Some("#!/bin/sh\necho 'onnxruntime 1.20.1 already present'\necho 'gstreamer: h264 ok'\n"),
    );
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    let transcript = engine.show(None).unwrap();
    let output = transcript
        .events
        .iter()
        .find_map(|record| match &record.event {
            updater::proto::RunEvent::Hook { output, .. } => Some(output.clone()),
            _ => None,
        })
        .expect("the hook that ran must be in the transcript");
    assert!(
        output.contains("onnxruntime 1.20.1 already present"),
        "{output}"
    );
    assert!(output.contains("gstreamer: h264 ok"), "{output}");
}

/// A rollback is an update event too, and the one someone asks about when a robot is mysteriously
/// on an old release.
#[tokio::test]
async fn an_explicit_rollback_gets_its_own_run() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();
    fx.publish("1.1.0", None);
    apply_latest(&mut engine).await.unwrap();

    engine.rollback("daemon").await.unwrap();

    let transcript = engine.show(None).unwrap();
    assert_eq!(transcript.run, 3);
    match &transcript.events.first().unwrap().event {
        updater::proto::RunEvent::Began { target, .. } => assert_eq!(target, "rollback"),
        other => panic!("expected the run to open, got {other:?}"),
    }
    assert!(event_names(&transcript.events).contains(&"ended".to_owned()));
}

/// Asking for a run that is not there says which ones are, because the number was a guess.
#[tokio::test]
async fn asking_for_a_run_that_is_not_there_says_which_are() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine_healthy();
    apply_latest(&mut engine).await.unwrap();

    let message = engine.show(Some(99)).unwrap_err().to_string();
    assert!(message.contains("run 99"), "{message}");
    assert!(message.contains("run 1 is the only one kept"), "{message}");
}

/// Passing the gate and being healthy are different facts, and the transcript must not round one
/// to the other.
///
/// `HealthCheck::Socket` commits a release onto a robot reporting *degraded* when the degradation
/// is something the release cannot have caused — servo power off is the everyday case. Recording
/// that as a plain pass produced a transcript saying "the robot reported healthy" while the same
/// board's journal, at the same second, said it was degraded. Found on a robot.
#[tokio::test]
async fn a_degraded_commit_says_so_rather_than_reporting_healthy() {
    let fx = Fixture::new();
    fx.publish("1.0.0", None);
    let mut engine = fx.engine(Box::new(DegradedRobot), Faults::none(), "");

    let result = apply_latest(&mut engine).await.unwrap();
    assert!(matches!(result, ApplyResult::Applied { .. }), "{result:?}");

    let transcript = engine.show(None).unwrap();
    let (passed, detail) = transcript
        .events
        .iter()
        .find_map(|record| match &record.event {
            updater::proto::RunEvent::Health { passed, detail } => Some((*passed, detail.clone())),
            _ => None,
        })
        .expect("the gate's verdict must be recorded");

    assert!(passed, "the release was committed, so the gate passed");
    let detail = detail.expect("a pass that was not a clean bill of health must say why");
    assert!(detail.contains("motor bus"), "{detail}");
    assert!(detail.contains("cannot have caused"), "{detail}");
}
