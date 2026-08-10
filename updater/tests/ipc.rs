//! IPC tests: a real `Server` on a real unix socket, driven by a hand-rolled
//! JSON-RPC client.
//!
//! Deliberately does not use `robotctl` — these test the *protocol*, and going
//! through the CLI would conflate wire behaviour with argument parsing and output
//! formatting.
//!
//! The properties under test come from `docs/design/architecture.md` §1.1 and
//! `docs/design/updater-design.md` §7: the socket is group-restricted, `status` stays
//! answerable while an update runs, a client disconnecting mid-update does not
//! cancel it, and error codes survive the round trip so clients can branch on them.

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use test_support::Publisher;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::UnixStream;
use updater::config::{AutoApply, Config};
use updater::engine::Engine;
use updater::faults::Faults;
use updater::ipc::Server;
use updater::proto::{self, method};
use updater::robot::{Health, RobotClient, SafeToRestart};
use updater::verify::KeyRing;

// ── fixture ──────────────────────────────────────────────────────────────────

/// Health is shared and mutable so a test can change it *between* updates — the realistic
/// shape of a bad release: the robot was fine on the version it had, and the new one comes
/// up sick. A fixed flag can only express "healthy throughout" or "broken throughout",
/// neither of which is the interesting case.
struct FakeRobot {
    healthy: Arc<AtomicBool>,
}

#[async_trait::async_trait]
impl RobotClient for FakeRobot {
    async fn safe_to_restart(&self, _t: Duration) -> SafeToRestart {
        SafeToRestart::Yes
    }
    async fn health(&self, _t: Duration) -> Health {
        if self.healthy.load(Ordering::Relaxed) {
            Health::Healthy
        } else {
            Health::Unhealthy("unhealthy".into())
        }
    }
    async fn model_api(&self, _t: Duration) -> Option<u32> {
        Some(1)
    }
    async fn remote_session_active(&self, _t: Duration) -> bool {
        false
    }
}

struct Harness {
    _dir: tempfile::TempDir,
    root: PathBuf,
    publisher: Publisher,
    socket: PathBuf,
}

impl Harness {
    fn new() -> Self {
        let dir = tempfile::tempdir().unwrap();
        let root = dir.path().to_path_buf();
        std::fs::create_dir_all(root.join("opt/robot/daemon")).unwrap();
        std::fs::create_dir_all(root.join("var/lib/robot/updater")).unwrap();
        let publisher = Publisher::new(root.join("keys"), root.join("published"));

        // Per-process socket path: several of these harnesses run concurrently, and a shared
        // path makes them fight over the same socket.
        let socket = root.join("updaterd.sock");

        Self {
            _dir: dir,
            root,
            publisher,
            socket,
        }
    }

    /// Publish a signed release, optionally corrupting the artifact afterwards so the
    /// signature no longer matches.
    fn publish(&self, version: &str, tamper: bool) {
        self.publish_with(version, tamper, |_| {});
    }

    fn publish_with(&self, version: &str, tamper: bool, edit: impl FnOnce(&mut serde_json::Value)) {
        self.publisher.release(version).manifest(edit).write();
        if tamper {
            self.publisher.tamper("daemon", version);
        }
    }

    fn engine(&self, healthy: bool, faults: Faults) -> Engine {
        self.engine_with(healthy, faults, "")
    }

    /// As [`Self::engine`], but returns the health switch so a test can flip it between
    /// updates.
    fn engine_toggleable(&self) -> (Engine, Arc<AtomicBool>) {
        let healthy = Arc::new(AtomicBool::new(true));
        let mut engine = self.engine_with(true, Faults::none(), "");
        engine.replace_robot_for_test(Box::new(FakeRobot {
            healthy: Arc::clone(&healthy),
        }));
        (engine, healthy)
    }

    fn engine_with(&self, healthy: bool, faults: Faults, extra: &str) -> Engine {
        let config = Config::from_toml(&format!(
            r#"
trusted_keys_dir = "{keys}"
hw_rev = 1
state_dir = "{state}"

{extra}

[component.daemon]
install_dir = "{install}"
source = {{ type = "local_dir", path = "{published}" }}
on_apply = {{ action = "none" }}
health = {{ probe = "socket", timeout = "2s" }}
"#,
            keys = self.root.join("keys").display(),
            state = self.root.join("var/lib/robot/updater").display(),
            install = self.root.join("opt/robot/daemon").display(),
            published = self.publisher.releases.display(),
            extra = extra,
        ))
        .unwrap();
        let keys = KeyRing::load(&config.trusted_keys_dir, false).unwrap();
        // `without_deferred_restarts` for the same reason as `apply.rs`: engines run in parallel here
        // and a fork in one holds another's update lock until it execs.
        Engine::new(
            config,
            keys,
            Box::new(FakeRobot {
                healthy: Arc::new(AtomicBool::new(healthy)),
            }),
            faults,
        )
        .unwrap()
        .without_deferred_restarts()
    }

    /// Serve in the background and return once the socket accepts connections.
    async fn serve(&self, engine: Engine) -> tokio::task::JoinHandle<()> {
        let server = Arc::new(Server::new(engine));
        let socket = self.socket.clone();
        let handle = tokio::spawn(async move {
            let _ = server.serve(&socket).await;
        });

        for _ in 0..100 {
            if UnixStream::connect(&self.socket).await.is_ok() {
                return handle;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        panic!("server did not start");
    }

    /// Serve an already-constructed server, for policy tests.
    async fn serve_with(&self, server: Arc<Server>) -> tokio::task::JoinHandle<()> {
        let socket = self.socket.clone();
        let handle = tokio::spawn(async move {
            let _ = server.serve(&socket).await;
        });
        for _ in 0..100 {
            if UnixStream::connect(&self.socket).await.is_ok() {
                return handle;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        panic!("server did not start");
    }

    /// Apply `latest` the way a client would, over the socket.
    ///
    /// The scheduler tests need a robot that already has a release live before they can ask
    /// what a *scheduled* check does; going through the RPC rather than the engine keeps
    /// that setup on the same path a real robot took to get there.
    async fn apply_via_client(&self) {
        let mut client = Client::connect(&self.socket).await;
        client.hello().await;
        let response = client
            .call(
                method::APPLY,
                serde_json::json!({ "component": "daemon", "target": "latest" }),
            )
            .await;
        assert!(response.error.is_none(), "{:?}", response.error);
    }

    fn live_version(&self) -> Option<String> {
        let target = std::fs::read_link(self.root.join("opt/robot/daemon/current")).ok()?;
        Some(target.file_name()?.to_str()?.to_owned())
    }

    /// Every journal entry, read off disk as JSON.
    ///
    /// Read from the file rather than via the `log` RPC so a test can count *attempts*,
    /// including ones the RPC's default limit might drop.
    fn journal_entries(&self) -> Vec<serde_json::Value> {
        let path = self.root.join("var/lib/robot/updater/update-log.jsonl");
        let Ok(text) = std::fs::read_to_string(path) else {
            return Vec::new();
        };
        text.lines()
            .filter(|l| !l.trim().is_empty())
            .map(|l| serde_json::from_str(l).expect("journal line must be JSON"))
            .collect()
    }
}

impl Drop for Harness {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.socket);
    }
}

/// Minimal JSON-RPC client over the socket.
struct Client {
    reader: BufReader<tokio::net::unix::OwnedReadHalf>,
    writer: tokio::net::unix::OwnedWriteHalf,
    next_id: u64,
}

impl Client {
    async fn connect(socket: &Path) -> Self {
        let stream = UnixStream::connect(socket).await.unwrap();
        let (read_half, writer) = stream.into_split();
        Self {
            reader: BufReader::new(read_half),
            writer,
            next_id: 1,
        }
    }

    /// Send a raw method name and params.
    ///
    /// Built by hand rather than through [`proto::Request::call`] on purpose: several tests
    /// below send shapes a typed client cannot express — a malformed `params`, an
    /// unsupported api_version — which is exactly what the server's error paths exist for.
    async fn send(&mut self, method: &str, params: serde_json::Value) -> proto::Id {
        let id = proto::Id::Number(self.next_id);
        self.next_id += 1;
        let request = proto::Request {
            jsonrpc: proto::JSONRPC_VERSION.to_owned(),
            id: Some(id.clone()),
            method: method.to_owned(),
            params: Some(params),
        };
        let mut line = serde_json::to_vec(&request).unwrap();
        line.push(b'\n');
        self.writer.write_all(&line).await.unwrap();
        self.writer.flush().await.unwrap();
        id
    }

    /// Read until the response matching `id`, collecting notification phases seen.
    async fn await_response(&mut self, id: &proto::Id) -> (proto::Response, Vec<proto::Phase>) {
        let mut phases = Vec::new();
        loop {
            let mut line = String::new();
            let read = self.reader.read_line(&mut line).await.unwrap();
            assert!(read > 0, "connection closed before a response");
            let trimmed = line.trim();

            if let Ok(note) = serde_json::from_str::<proto::Request>(trimmed)
                && note.is_notification()
            {
                if let Ok(progress) = note.as_progress() {
                    phases.push(progress.phase);
                }
                continue;
            }
            let response: proto::Response = serde_json::from_str(trimmed).unwrap();
            if response.id.as_ref() == Some(id) {
                return (response, phases);
            }
        }
    }

    async fn call(&mut self, method: &str, params: serde_json::Value) -> proto::Response {
        let id = self.send(method, params).await;
        self.await_response(&id).await.0
    }

    async fn hello(&mut self) -> proto::Response {
        self.call(
            method::HELLO,
            serde_json::json!({ "api_version": proto::API_VERSION }),
        )
        .await
    }
}

// ── tests ────────────────────────────────────────────────────────────────────

#[tokio::test]
async fn socket_is_group_restricted() {
    use std::os::unix::fs::PermissionsExt;

    let fx = Harness::new();
    let _server = fx.serve(fx.engine(true, Faults::none())).await;

    let mode = std::fs::metadata(&fx.socket).unwrap().permissions().mode() & 0o777;
    // Anyone who can write here can trigger an update or a rollback, so "others"
    // must have nothing.
    assert_eq!(mode, 0o660, "socket mode is {mode:o}, want 660");
}

#[tokio::test]
async fn hello_accepts_matching_version_and_refuses_a_mismatch() {
    let fx = Harness::new();
    let _server = fx.serve(fx.engine(true, Faults::none())).await;
    let mut client = Client::connect(&fx.socket).await;

    let response = client.hello().await;
    assert!(response.error.is_none(), "{:?}", response.error);
    let result: proto::HelloResult = response.result_as().unwrap();
    assert_eq!(result.api_version, proto::API_VERSION);

    // A stale robotctl in someone's shell must get a clear refusal, not a
    // misinterpreted request.
    let response = client
        .call(method::HELLO, serde_json::json!({ "api_version": 999 }))
        .await;
    assert_eq!(response.error.unwrap().code, proto::code::PROTOCOL_MISMATCH);
}

#[tokio::test]
async fn apply_streams_progress_then_a_terminal_result() {
    let fx = Harness::new();
    fx.publish("1.0.0", false);
    let _server = fx.serve(fx.engine(true, Faults::none())).await;
    let mut client = Client::connect(&fx.socket).await;
    client.hello().await;

    let id = client
        .send(
            method::APPLY,
            serde_json::json!({ "component": "daemon", "target": "latest" }),
        )
        .await;
    let (response, phases) = client.await_response(&id).await;

    assert!(response.error.is_none(), "{:?}", response.error);
    let result: proto::ApplyResult = response.result_as().unwrap();
    assert!(
        matches!(result, proto::ApplyResult::Applied { .. }),
        "{result:?}"
    );
    assert_eq!(fx.live_version().as_deref(), Some("1.0.0"));

    // The app's progress bar depends on these arriving as notifications.
    for expected in [
        proto::Phase::Verifying,
        proto::Phase::Swapping,
        proto::Phase::Committing,
    ] {
        assert!(
            phases.contains(&expected),
            "missing {expected:?} in {phases:?}"
        );
    }
}

/// Error codes must survive the round trip: clients (and `robotctl`'s exit codes)
/// branch on them.
#[tokio::test]
async fn refusals_carry_their_code_over_the_wire() {
    let fx = Harness::new();
    fx.publish("1.0.0", true); // tampered
    let _server = fx.serve(fx.engine(true, Faults::none())).await;
    let mut client = Client::connect(&fx.socket).await;
    client.hello().await;

    let response = client
        .call(
            method::APPLY,
            serde_json::json!({ "component": "daemon", "target": "latest" }),
        )
        .await;

    let error = response.error.expect("should be refused");
    assert_eq!(error.code, proto::code::VERIFICATION_FAILED);
    assert!(error.message.contains("sha256"), "{}", error.message);
    assert_eq!(fx.live_version(), None, "nothing may be installed");
}

#[tokio::test]
async fn unknown_method_and_bad_params_are_reported_distinctly() {
    let fx = Harness::new();
    let _server = fx.serve(fx.engine(true, Faults::none())).await;
    let mut client = Client::connect(&fx.socket).await;

    let response = client.call("update.nonsense", serde_json::json!({})).await;
    assert_eq!(response.error.unwrap().code, proto::code::METHOD_NOT_FOUND);

    let response = client
        .call(method::APPLY, serde_json::json!({ "wrong": "shape" }))
        .await;
    assert_eq!(response.error.unwrap().code, proto::code::INVALID_PARAMS);
}

#[tokio::test]
async fn malformed_json_gets_a_parse_error_and_the_connection_survives() {
    let fx = Harness::new();
    let _server = fx.serve(fx.engine(true, Faults::none())).await;
    let mut client = Client::connect(&fx.socket).await;

    client.writer.write_all(b"{ not json\n").await.unwrap();
    let mut line = String::new();
    client.reader.read_line(&mut line).await.unwrap();
    let response: proto::Response = serde_json::from_str(line.trim()).unwrap();
    assert_eq!(response.error.unwrap().code, proto::code::PARSE_ERROR);

    // The connection must still be usable — one bad line is not a fatal session error.
    assert!(client.hello().await.error.is_none());
}

/// `status` must answer *during* an update. Blocking would make the app go blank for
/// the whole duration — exactly when someone is most likely watching it.
#[tokio::test]
async fn status_answers_while_an_update_is_running() {
    let fx = Harness::new();
    fx.publish("1.0.0", false);
    // `hang_health` keeps the engine busy in the health gate for the full timeout.
    let engine = fx.engine(
        true,
        Faults {
            hang_health: true,
            ..Faults::none()
        },
    );
    let _server = fx.serve(engine).await;

    let mut applier = Client::connect(&fx.socket).await;
    applier.hello().await;
    let apply_id = applier
        .send(
            method::APPLY,
            serde_json::json!({ "component": "daemon", "target": "latest" }),
        )
        .await;

    // Let it get into the gate.
    tokio::time::sleep(Duration::from_millis(400)).await;

    let mut observer = Client::connect(&fx.socket).await;
    observer.hello().await;
    let response = tokio::time::timeout(
        Duration::from_secs(1),
        observer.call(method::STATUS, serde_json::json!({})),
    )
    .await
    .expect("status must not block on the in-flight update");

    assert!(response.error.is_none(), "{:?}", response.error);

    // And a second mutating request is refused as busy rather than queued.
    let response = observer
        .call(
            method::APPLY,
            serde_json::json!({ "component": "daemon", "target": "latest" }),
        )
        .await;
    assert_eq!(response.error.unwrap().code, proto::code::BUSY);

    let _ = applier.await_response(&apply_id).await;
}

/// The robot pulls, so a client vanishing mid-update is normal and must not cancel
/// it (`architecture.md` §1.1).
#[tokio::test]
async fn update_completes_after_the_client_disconnects() {
    let fx = Harness::new();
    fx.publish("1.0.0", false);
    let _server = fx.serve(fx.engine(true, Faults::none())).await;

    {
        let mut client = Client::connect(&fx.socket).await;
        client.hello().await;
        client
            .send(
                method::APPLY,
                serde_json::json!({ "component": "daemon", "target": "latest" }),
            )
            .await;
        // Drop without reading the response — the BLE-dropped case.
    }

    // The update must still land.
    for _ in 0..100 {
        if fx.live_version().as_deref() == Some("1.0.0") {
            return;
        }
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
    panic!("update did not complete after the client disconnected");
}

#[tokio::test]
async fn subscribe_receives_progress_from_another_connection() {
    let fx = Harness::new();
    fx.publish("1.0.0", false);
    let _server = fx.serve(fx.engine(true, Faults::none())).await;

    let mut watcher = Client::connect(&fx.socket).await;
    watcher.send(method::SUBSCRIBE, serde_json::json!({})).await;

    let mut applier = Client::connect(&fx.socket).await;
    applier.hello().await;
    let id = applier
        .send(
            method::APPLY,
            serde_json::json!({ "component": "daemon", "target": "latest" }),
        )
        .await;
    applier.await_response(&id).await;

    // At least one notification must reach the separate subscriber — this is the
    // path `btd` uses to feed the app.
    let mut line = String::new();
    tokio::time::timeout(Duration::from_secs(2), watcher.reader.read_line(&mut line))
        .await
        .expect("subscriber should receive progress")
        .unwrap();

    let note: proto::Request = serde_json::from_str(line.trim()).unwrap();
    assert!(note.is_notification());
    assert_eq!(note.method, method::PROGRESS);
}

#[tokio::test]
async fn read_only_methods_work_with_no_releases_published() {
    let fx = Harness::new();
    let _server = fx.serve(fx.engine(true, Faults::none())).await;
    let mut client = Client::connect(&fx.socket).await;
    client.hello().await;

    // A fresh robot must still be able to report on itself.
    let response = client.call(method::STATUS, serde_json::json!({})).await;
    assert!(response.error.is_none(), "{:?}", response.error);

    let response = client
        .call(method::LOG, serde_json::json!({ "limit": 10 }))
        .await;
    assert!(response.error.is_none(), "{:?}", response.error);

    let response = client
        .call(
            method::LIST_INSTALLED,
            serde_json::json!({ "component": "daemon" }),
        )
        .await;
    assert!(response.error.is_none(), "{:?}", response.error);
}

// ── scheduled checks (§8.1) ──────────────────────────────────────────────────

/// **`min_supported` must actually pull a robot forward.**
///
/// Previously the floor was inert: `check` reported `mandatory`, but nothing polled,
/// so a robot only learned of it when someone opened the app — useless as the
/// remediation path for "we shipped a bad release".
#[tokio::test]
async fn a_mandatory_update_is_applied_without_a_client() {
    let fx = Harness::new();
    fx.publish("1.0.0", false);

    // Install 1.0.0 the ordinary way.
    let engine = fx.engine(true, Faults::none());
    let server = Arc::new(Server::new(engine));
    {
        let socket = fx.socket.clone();
        let s = Arc::clone(&server);
        tokio::spawn(async move {
            let _ = s.serve(&socket).await;
        });
        for _ in 0..100 {
            if UnixStream::connect(&fx.socket).await.is_ok() {
                break;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        let mut client = Client::connect(&fx.socket).await;
        client.hello().await;
        let response = client
            .call(
                method::APPLY,
                serde_json::json!({ "component": "daemon", "target": "latest" }),
            )
            .await;
        assert!(response.error.is_none(), "{:?}", response.error);
    }
    assert_eq!(fx.live_version().as_deref(), Some("1.0.0"));

    // 1.1.0 declares that anything below it must not be used.
    fx.publish_with("1.1.0", false, |m| {
        m["min_supported"] = serde_json::json!("1.1.0");
    });

    // A scheduled check at the default policy must move the robot, with nobody watching.
    server.check_all_for_test(AutoApply::Mandatory).await;

    assert_eq!(
        fx.live_version().as_deref(),
        Some("1.1.0"),
        "a mandatory update must be applied unattended"
    );
}

/// **A bad mandatory release must not put the robot in an apply/rollback loop.**
///
/// The failure this prevents is a fleet-wide one. `min_supported` exists to force robots
/// forward without waiting for a client, so if the release carrying that floor is itself
/// broken, every robot: checks, sees mandatory, applies, fails the gate, rolls back, waits
/// `check_interval`, and does it all again — re-downloading the artifact, rewriting the
/// eMMC and restarting `robotd` each time, forever, on battery. Nothing in the cycle
/// converges, and no client is involved to notice.
///
/// The guard is `known_bad`, which is derived from the journal's latest outcome per
/// version, so it self-clears if the release ever does succeed.
#[tokio::test]
async fn a_mandatory_release_that_failed_is_not_reapplied_unattended() {
    let fx = Harness::new();
    fx.publish("1.0.0", false);

    let (engine, healthy) = fx.engine_toggleable();
    let server = Arc::new(Server::new(engine));
    {
        let socket = fx.socket.clone();
        let s = Arc::clone(&server);
        tokio::spawn(async move {
            let _ = s.serve(&socket).await;
        });
        for _ in 0..100 {
            if UnixStream::connect(&fx.socket).await.is_ok() {
                break;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        let mut client = Client::connect(&fx.socket).await;
        client.hello().await;
        let response = client
            .call(
                method::APPLY,
                serde_json::json!({ "component": "daemon", "target": "latest" }),
            )
            .await;
        assert!(response.error.is_none(), "{:?}", response.error);
    }
    assert_eq!(fx.live_version().as_deref(), Some("1.0.0"));

    // 1.1.0 is mandatory *and* broken: the robot goes sick the moment it is live.
    healthy.store(false, Ordering::Relaxed);
    fx.publish_with("1.1.0", false, |m| {
        m["min_supported"] = serde_json::json!("1.1.0");
    });

    // First scheduled check: it is right to try, and right to roll back.
    server.check_all_for_test(AutoApply::Mandatory).await;
    assert_eq!(
        fx.live_version().as_deref(),
        Some("1.0.0"),
        "a release that fails its gate must be reverted"
    );

    // Subsequent checks must refuse. Three of them, because a guard that only holds for
    // one round would still loop — just more slowly.
    for _ in 0..3 {
        server.check_all_for_test(AutoApply::Mandatory).await;
    }

    assert_eq!(
        fx.live_version().as_deref(),
        Some("1.0.0"),
        "the robot must stay on the release that works"
    );

    // The real assertion: exactly ONE attempt is recorded. Checking the live version alone
    // would pass even if every round re-applied and re-reverted — which is the actual bug,
    // and is invisible from the symlink.
    let attempts = fx
        .journal_entries()
        .into_iter()
        .filter(|e| {
            e["to"] == serde_json::json!("1.1.0")
                && e["outcome"]["kind"] == serde_json::json!("rolled_back")
        })
        .count();
    assert_eq!(
        attempts, 1,
        "1.1.0 must be attempted once and then refused, not retried on every check"
    );
}

/// Opting out must be respected — but loudly, because a silently-ignored mandatory
/// update is exactly the situation the floor exists to prevent.
#[tokio::test]
async fn a_mandatory_update_is_not_applied_when_auto_apply_is_off() {
    let fx = Harness::new();
    fx.publish("1.0.0", false);
    let engine = fx.engine(true, Faults::none());
    let server = Arc::new(Server::new(engine));
    let _handle = {
        let socket = fx.socket.clone();
        let s = Arc::clone(&server);
        tokio::spawn(async move {
            let _ = s.serve(&socket).await;
        })
    };
    for _ in 0..100 {
        if UnixStream::connect(&fx.socket).await.is_ok() {
            break;
        }
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    let mut client = Client::connect(&fx.socket).await;
    client.hello().await;
    client
        .call(
            method::APPLY,
            serde_json::json!({ "component": "daemon", "target": "latest" }),
        )
        .await;

    fx.publish_with("1.1.0", false, |m| {
        m["min_supported"] = serde_json::json!("1.1.0");
    });

    server.check_all_for_test(AutoApply::Off).await;

    assert_eq!(
        fx.live_version().as_deref(),
        Some("1.0.0"),
        "auto_apply = off must be respected"
    );
}

/// At the default policy an ordinary update must not be applied behind the owner's back.
/// `mandatory` is about the floor, not about updates in general.
#[tokio::test]
async fn an_ordinary_update_is_not_applied_at_the_mandatory_policy() {
    let fx = Harness::new();
    fx.publish("1.0.0", false);
    let engine = fx.engine(true, Faults::none());
    let server = Arc::new(Server::new(engine));
    let _handle = {
        let socket = fx.socket.clone();
        let s = Arc::clone(&server);
        tokio::spawn(async move {
            let _ = s.serve(&socket).await;
        })
    };
    for _ in 0..100 {
        if UnixStream::connect(&fx.socket).await.is_ok() {
            break;
        }
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    let mut client = Client::connect(&fx.socket).await;
    client.hello().await;
    client
        .call(
            method::APPLY,
            serde_json::json!({ "component": "daemon", "target": "latest" }),
        )
        .await;

    // No floor declared, so this is an ordinary update.
    fx.publish("1.1.0", false);
    server.check_all_for_test(AutoApply::Mandatory).await;

    assert_eq!(
        fx.live_version().as_deref(),
        Some("1.0.0"),
        "at the mandatory policy, only a mandatory update may be applied unattended"
    );
}

/// `auto_apply = all` is the canary and bench-robot setting: an ordinary release, with no
/// floor declared and nobody attached, installs itself.
#[tokio::test]
async fn auto_apply_all_installs_an_ordinary_update() {
    let fx = Harness::new();
    fx.publish("1.0.0", false);
    let engine = fx.engine(true, Faults::none());
    let server = Arc::new(Server::new(engine));
    let _handle = fx.serve_with(Arc::clone(&server)).await;
    fx.apply_via_client().await;
    assert_eq!(fx.live_version().as_deref(), Some("1.0.0"));

    // No `min_supported`, so this is an ordinary update — the case `mandatory` skips.
    fx.publish("1.1.0", false);
    server.check_all_for_test(AutoApply::All).await;

    assert_eq!(
        fx.live_version().as_deref(),
        Some("1.1.0"),
        "auto_apply = all must install an ordinary release with no client attached"
    );
}

/// The anti-loop guard has to cover the `all` policy too, and this is the test that says
///
/// Without it, `auto_apply = all` plus one bad release is an endless cycle: apply, fail the
/// gate, roll back, wait `check_interval`, re-download the artifact, rewrite the eMMC,
/// restart `robotd`, repeat. On a canary that is merely wasteful; the same code runs on a
/// robot in the field.
#[tokio::test]
async fn auto_apply_all_refuses_a_release_that_already_failed_its_gate() {
    let fx = Harness::new();
    fx.publish("1.0.0", false);
    let (engine, healthy) = fx.engine_toggleable();
    let server = Arc::new(Server::new(engine));
    let _handle = fx.serve_with(Arc::clone(&server)).await;
    fx.apply_via_client().await;
    assert_eq!(fx.live_version().as_deref(), Some("1.0.0"));

    // 1.1.0 is ordinary *and* broken: the robot goes sick the moment it is live.
    healthy.store(false, Ordering::Relaxed);
    fx.publish("1.1.0", false);

    // First pass: right to try, right to roll back.
    server.check_all_for_test(AutoApply::All).await;
    assert_eq!(
        fx.live_version().as_deref(),
        Some("1.0.0"),
        "a release that fails its gate must be reverted"
    );

    // Three more, because a guard that holds for one round would still loop, just slower.
    for _ in 0..3 {
        server.check_all_for_test(AutoApply::All).await;
    }
    assert_eq!(fx.live_version().as_deref(), Some("1.0.0"));

    // The real assertion: exactly ONE attempt. Checking the live version alone passes even
    // if every round re-applied and re-reverted, which is the actual bug and is invisible
    // from the symlink.
    let attempts = fx
        .journal_entries()
        .into_iter()
        .filter(|e| {
            e["to"] == serde_json::json!("1.1.0")
                && e["outcome"]["kind"] == serde_json::json!("rolled_back")
        })
        .count();
    assert_eq!(
        attempts, 1,
        "1.1.0 should have been attempted once and then refused, not retried every round"
    );
}

/// `off` means off, for ordinary releases as well as mandatory ones.
#[tokio::test]
async fn auto_apply_off_installs_nothing() {
    let fx = Harness::new();
    fx.publish("1.0.0", false);
    let engine = fx.engine(true, Faults::none());
    let server = Arc::new(Server::new(engine));
    let _handle = fx.serve_with(Arc::clone(&server)).await;
    fx.apply_via_client().await;

    fx.publish("1.1.0", false);
    server.check_all_for_test(AutoApply::Off).await;

    assert_eq!(
        fx.live_version().as_deref(),
        Some("1.0.0"),
        "auto_apply = off must not install anything unattended"
    );
}

// ── peer credential enforcement ──────────────────────────────────────────────

/// The uid `updaterd` runs as is always allowed. In tests that is the test process,
/// so the ordinary path must keep working — a policy that locked out the owner would
/// be indistinguishable from a broken daemon.
#[tokio::test]
async fn the_owning_uid_may_mutate() {
    let fx = Harness::new();
    fx.publish("1.0.0", false);
    let _server = fx.serve(fx.engine(true, Faults::none())).await;
    let mut client = Client::connect(&fx.socket).await;
    client.hello().await;

    let response = client
        .call(
            method::APPLY,
            serde_json::json!({ "component": "daemon", "target": "latest" }),
        )
        .await;
    assert!(response.error.is_none(), "{:?}", response.error);
}

/// With a policy that excludes the caller, mutating requests are refused — and
/// refused *distinctly*, so a client can say "ask an administrator" rather than
/// "something broke".
#[tokio::test]
async fn an_unlisted_peer_cannot_mutate() {
    let fx = Harness::new();
    fx.publish("1.0.0", false);

    // Owner uid set to something the test process is not, and no allowances.
    let server = Arc::new(Server::with_policy_for_test(
        fx.engine(true, Faults::none()),
        u32::MAX,
        Vec::new(),
        Vec::new(),
    ));
    let _handle = fx.serve_with(Arc::clone(&server)).await;

    let mut client = Client::connect(&fx.socket).await;
    client.hello().await;

    for (method, params) in [
        (
            method::APPLY,
            serde_json::json!({ "component": "daemon", "target": "latest" }),
        ),
        (
            method::ROLLBACK,
            serde_json::json!({ "component": "daemon" }),
        ),
        (
            method::RESET_TO_GOLDEN,
            serde_json::json!({ "component": "daemon" }),
        ),
        (
            method::PIN,
            serde_json::json!({ "component": "daemon", "version": "1.0.0" }),
        ),
    ] {
        let response = client.call(method, params).await;
        let error = response.error.expect("should be denied");
        assert_eq!(
            error.code,
            proto::code::PERMISSION_DENIED,
            "{method} should be denied, got {error:?}"
        );
        // The message must say what to do about it.
        assert!(error.message.contains("allow_uids"), "{}", error.message);
    }

    // And nothing was installed.
    assert_eq!(fx.live_version(), None);
}

/// Read-only requests are deliberately **not** gated: reaching the socket already
/// requires its group, and support must be able to inspect a robot it is not
/// authorised to change.
#[tokio::test]
async fn an_unlisted_peer_may_still_read() {
    let fx = Harness::new();
    let server = Arc::new(Server::with_policy_for_test(
        fx.engine(true, Faults::none()),
        u32::MAX,
        Vec::new(),
        Vec::new(),
    ));
    let _handle = fx.serve_with(Arc::clone(&server)).await;

    let mut client = Client::connect(&fx.socket).await;
    client.hello().await;

    for (method, params) in [
        (method::STATUS, serde_json::json!({})),
        (method::LOG, serde_json::json!({ "limit": 5 })),
        (
            method::LIST_INSTALLED,
            serde_json::json!({ "component": "daemon" }),
        ),
    ] {
        let response = client.call(method, params).await;
        assert!(
            response.error.is_none(),
            "{method} should be readable: {:?}",
            response.error
        );
    }
}

/// An explicit allowance lets a non-owner mutate — the mechanism `btd`'s user will
/// rely on once it exists.
#[tokio::test]
async fn an_allowed_uid_may_mutate() {
    let fx = Harness::new();
    fx.publish("1.0.0", false);

    let me = std::fs::metadata(fx.root.join("keys"))
        .map(|m| {
            use std::os::unix::fs::MetadataExt;
            m.uid()
        })
        .unwrap();

    let server = Arc::new(Server::with_policy_for_test(
        fx.engine(true, Faults::none()),
        u32::MAX,
        vec![me],
        Vec::new(),
    ));
    let _handle = fx.serve_with(Arc::clone(&server)).await;

    let mut client = Client::connect(&fx.socket).await;
    client.hello().await;
    let response = client
        .call(
            method::APPLY,
            serde_json::json!({ "component": "daemon", "target": "latest" }),
        )
        .await;
    assert!(response.error.is_none(), "{:?}", response.error);
    assert_eq!(fx.live_version().as_deref(), Some("1.0.0"));
}
