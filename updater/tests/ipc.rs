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
    async fn reload_policies(&self, _timeout: Duration) -> bool {
        true
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

/// `hello` serves a client built against another `API_VERSION`, in both directions.
///
/// It used to refuse on an exact `!=`, and `hello` precedes every `robotctl` command — so one
/// differing digit took away `update apply`, which is how a skew ends, and `version`, which is how
/// it gets diagnosed. A client newer than the daemon is the ordinary few seconds after an update;
/// a client older than it is a copy from somewhere other than `/usr/local/bin/robotctl`. Neither is
/// a reason to refuse a call this release can serve, and both learn the daemon's version from the
/// reply and can say so themselves.
///
/// What refuses is in `unknown_method_and_bad_params_are_reported_distinctly`: a route that is
/// genuinely missing, and a parameter this release does not know.
#[tokio::test]
async fn hello_serves_a_client_from_another_release() {
    let fx = Harness::new();
    let _server = fx.serve(fx.engine(true, Faults::none())).await;
    let mut client = Client::connect(&fx.socket).await;

    for theirs in [proto::API_VERSION, 999, 1] {
        let response = client
            .call(method::HELLO, serde_json::json!({ "api_version": theirs }))
            .await;
        assert!(response.error.is_none(), "v{theirs}: {:?}", response.error);
        let result: proto::HelloResult = response.result_as().unwrap();
        assert_eq!(result.api_version, proto::API_VERSION, "v{theirs}");
    }
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

/// `from_dir` has to survive the wire, because the wire is the whole path.
///
/// `robotctl update apply --from <dir>` is one JSON field: everything else about it — the
/// source override, the exempted downgrade guard, the health gate that still runs — is on the
/// daemon side of the socket, and reachable only if this field arrives. It is also the field a
/// daemon one API version older would parse and silently ignore, installing from its
/// configured source instead, which is why `API_VERSION` moved with it.
#[tokio::test]
async fn apply_from_a_directory_over_the_wire() {
    let fx = Harness::new();
    fx.publish("1.0.0", false);
    let _server = fx.serve(fx.engine(true, Faults::none())).await;
    let mut client = Client::connect(&fx.socket).await;
    client.hello().await;

    // A directory the configured source knows nothing about, as a laptop push would leave it.
    let sideload = fx.root.join("var/tmp/duck-sideload");
    std::fs::create_dir_all(&sideload).unwrap();
    fx.publisher.release("1.1.0").dir(sideload.clone()).write();

    let response = client
        .call(
            method::APPLY,
            serde_json::json!({
                "component": "daemon",
                "target": "latest",
                "options": { "from_dir": sideload },
            }),
        )
        .await;

    assert!(response.error.is_none(), "{:?}", response.error);
    let result: proto::ApplyResult = response.result_as().unwrap();
    assert!(
        matches!(result, proto::ApplyResult::Applied { .. }),
        "{result:?}"
    );
    assert_eq!(
        fx.live_version().as_deref(),
        Some("1.1.0"),
        "the release in the named directory is the one that must be live"
    );
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

/// The three ways a mismatched client actually fails, now that the handshake is not one of them.
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

    // A member from a later release, on a method that exists. This is the case the handshake gate
    // was standing in for: without it, serde would ignore the member and the apply would run
    // against the *configured* source while the caller believed it was sideloading a directory.
    let response = client
        .call(
            method::APPLY,
            serde_json::json!({
                "component": "daemon",
                "target": "latest",
                "from_a_later_release": true,
            }),
        )
        .await;
    let error = response.error.expect("an unknown member must be refused");
    assert_eq!(error.code, proto::code::INVALID_PARAMS);
    assert!(
        error.message.contains("from_a_later_release"),
        "{}",
        error.message
    );
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

/// `check` must come straight back during an update, and say why.
///
/// It used to take the engine lock and wait, so "is there an update available?" asked while one
/// was running answered whenever that update finished — minutes, for a daemon release. On a phone
/// that is indistinguishable from a robot that has stopped answering, and it is the wrong answer
/// besides: there is something to say, and it is that an update is in progress.
#[tokio::test]
async fn check_says_busy_rather_than_waiting_for_the_update_to_finish() {
    let fx = Harness::new();
    fx.publish("1.0.0", false);
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

    // Let it get into the gate, where it holds the engine.
    tokio::time::sleep(Duration::from_millis(400)).await;

    let mut observer = Client::connect(&fx.socket).await;
    observer.hello().await;
    let response = tokio::time::timeout(
        Duration::from_secs(1),
        observer.call(method::CHECK, serde_json::json!({ "component": "daemon" })),
    )
    .await
    .expect("check must not block on the in-flight update");

    assert_eq!(response.error.expect("busy").code, proto::code::BUSY);

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

// ── account.* ────────────────────────────────────────────────────────────────
//
// The login is a device-code flow, so the interesting property is not "the call works" but the
// *shape* it works in: `account.login` answers with a code and hands the waiting to the daemon,
// and a client that goes away — which is what a phone does when it opens a browser — comes back
// to `account.status` to find out what happened. These drive that over a real socket against a
// stand-in for huggingface.co, because every part of it that can be wrong is in the wiring
// between the three.

/// A stand-in for huggingface.co: a device code, then pending, then a token.
///
/// One state machine rather than a fixed script, so a test observes the same sequence a real
/// login does — including the `authorization_pending` the robot has to keep polling through.
struct FakeHub {
    base: String,
    polls: Arc<std::sync::atomic::AtomicUsize>,
    _task: tokio::task::JoinHandle<()>,
}

impl FakeHub {
    /// `approve_after` is how many `authorization_pending` answers to give before the token.
    async fn start(approve_after: usize) -> Self {
        use axum::extract::Form;
        use axum::routing::{get, post};

        let polls = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let counter = Arc::clone(&polls);

        let app = axum::Router::new()
            .route(
                "/oauth/device",
                // Hugging Face sends no `verification_uri_complete`, so this does not either —
                // what falls out of that is part of what these tests cover. It *does* send an
                // `interval` HF omits, and only to keep the suite quick: the robot sleeps one
                // interval before its first poll, so the real five seconds would make every
                // login test five seconds long. That HF's omission falls back to five is pinned
                // in `account::tests::a_device_code_response_is_normalised`, where it costs
                // nothing.
                post(|| async {
                    axum::Json(serde_json::json!({
                        "device_code": "device-abc",
                        "user_code": "A6MY-0314",
                        "verification_uri": "https://hf.co/oauth/device",
                        "expires_in": 300,
                        "interval": 1
                    }))
                }),
            )
            .route(
                "/oauth/token",
                post(
                    move |Form(form): Form<std::collections::HashMap<String, String>>| {
                        let counter = Arc::clone(&counter);
                        async move {
                            assert_eq!(
                                form.get("client_id").map(String::as_str),
                                Some(updater::account::CLIENT_ID),
                                "the device flow must identify itself as the public client"
                            );
                            let n = counter.fetch_add(1, Ordering::SeqCst);
                            if n < approve_after {
                                return axum::Json(serde_json::json!({
                                    "error": "authorization_pending"
                                }));
                            }
                            axum::Json(serde_json::json!({
                                "access_token": "an-access-token",
                                "refresh_token": "a-refresh-token",
                                "expires_in": 2_591_999u64,
                                "token_type": "bearer"
                            }))
                        }
                    },
                ),
            )
            .route(
                "/oauth/userinfo",
                get(|| async {
                    axum::Json(serde_json::json!({
                        "name": "Rouanet",
                        "preferred_username": "PierreRouanet"
                    }))
                }),
            );

        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let base = format!("http://{}", listener.local_addr().unwrap());
        let task = tokio::spawn(async move {
            let _ = axum::serve(listener, app).await;
        });
        Self {
            base,
            polls,
            _task: task,
        }
    }
}

/// Serve with the account pointed at a temp file and a fake hub.
async fn serve_with_account(fx: &Harness, hub: &FakeHub) -> (PathBuf, tokio::task::JoinHandle<()>) {
    serve_with_account_as(fx, hub, None).await
}

/// As [`serve_with_account`], with an owning uid the test process is not.
///
/// `None` means the ordinary case: the socket's owner is whoever ran the test, so mutating calls
/// are authorised. `Some(uid)` is how a test asks what an *unlisted* peer sees, which for
/// `account.*` is the whole point of one call being a read.
async fn serve_with_account_as(
    fx: &Harness,
    hub: &FakeHub,
    owner_uid: Option<u32>,
) -> (PathBuf, tokio::task::JoinHandle<()>) {
    let token_path = fx.root.join("etc/robot/hf-token");
    std::fs::create_dir_all(token_path.parent().unwrap()).unwrap();
    let engine = fx.engine(true, Faults::none());
    let server = match owner_uid {
        None => Server::new(engine),
        Some(uid) => Server::with_policy_for_test(engine, uid, Vec::new(), Vec::new()),
    };
    let server = Arc::new(server.with_account_for_test(token_path.clone(), hub.base.clone()));
    let handle = fx.serve_with(server).await;
    (token_path, handle)
}

/// The whole flow: a code comes back, the daemon polls, and the token lands on disk.
///
/// The three assertions that matter are ordered as a client experiences them — the code arrives
/// *before* anyone has approved anything, the account appears without the client asking Hugging
/// Face anything itself, and what is written is a credential that can be renewed.
#[tokio::test]
async fn a_device_code_login_completes_without_the_client_waiting() {
    let fx = Harness::new();
    let hub = FakeHub::start(2).await;
    let (token_path, _server) = serve_with_account(&fx, &hub).await;

    let mut client = Client::connect(&fx.socket).await;
    client.hello().await;

    // 1. `login` answers immediately, with something to show a person.
    let response = client
        .call(method::ACCOUNT_LOGIN, serde_json::json!({}))
        .await;
    assert!(response.error.is_none(), "{:?}", response.error);
    let result = response.result.unwrap();
    assert_eq!(result["user_code"], "A6MY-0314");
    assert_eq!(result["verification_uri"], "https://hf.co/oauth/device");
    assert_eq!(
        result["verification_uri_complete"], "https://hf.co/oauth/device",
        "the plain URI, because Hugging Face sends no complete one and its device page ignores a \
         `?user_code=` query — so no URL carries the code and a client has to show it"
    );
    assert_eq!(
        result["interval"], 1,
        "the server's interval, passed through for a client that wants to show a countdown"
    );
    assert!(
        !token_path.exists(),
        "nothing is stored until somebody approves it"
    );

    // A second client, because the first one is allowed to have gone away by now — this is the
    // property the whole shape exists for.
    let mut watcher = Client::connect(&fx.socket).await;
    watcher.hello().await;

    // While it is in flight, `status` carries the code so a client that reconnected can show it
    // again rather than starting a second login.
    let pending = watcher
        .call(method::ACCOUNT_STATUS, serde_json::json!({}))
        .await;
    let pending = pending.result.unwrap();
    assert_eq!(pending["login"]["user_code"], "A6MY-0314");
    assert!(pending["account"].is_null(), "not signed in yet");

    // 2. The daemon is doing the polling. `FakeHub` approves on the third ask and answers with a
    // one-second interval, so this is the one place the test has to wait for real time — a few
    // seconds of it. Bounded rather than a bare loop: a login that never lands should fail here
    // saying so, not hang until whatever is running the suite gives up on it.
    let mut signed_in = None;
    for _ in 0..100 {
        tokio::time::sleep(Duration::from_millis(200)).await;
        let status = watcher
            .call(method::ACCOUNT_STATUS, serde_json::json!({}))
            .await
            .result
            .unwrap();
        assert!(
            status["last_error"].is_null(),
            "the login failed: {status:?}"
        );
        if !status["account"].is_null() {
            signed_in = Some(status);
            break;
        }
    }
    let signed_in = signed_in.expect("the login never completed");

    assert_eq!(signed_in["account"]["username"], "PierreRouanet");
    assert!(
        signed_in["login"].is_null(),
        "the pending login is cleared once it resolves"
    );
    assert_eq!(
        signed_in["account"]["refreshable"], true,
        "a refresh token came back, so the robot can renew this itself"
    );
    assert!(
        hub.polls.load(Ordering::SeqCst) >= 3,
        "the daemon must have polled through the pending answers"
    );

    // 3. What is on disk is the pair, not just the access token: a 30-day token with no way to
    // renew it is a robot that silently stops being reachable next month.
    let stored: serde_json::Value =
        serde_json::from_slice(&std::fs::read(&token_path).unwrap()).unwrap();
    assert_eq!(stored["access_token"], "an-access-token");
    assert_eq!(stored["refresh_token"], "a-refresh-token");
    assert_eq!(stored["username"], "PierreRouanet");
    assert!(stored["expires_at"].as_i64().unwrap() > 0);

    // And signing out forgets it, naming who it was — which is what lets a robot change hands.
    let out = client
        .call(method::ACCOUNT_LOGOUT, serde_json::json!({}))
        .await
        .result
        .unwrap();
    assert_eq!(out["was"], "PierreRouanet");
    assert!(!token_path.exists(), "the credential is gone from disk");
}

/// A robot that already belongs to somebody refuses, by name, and does not start a login.
///
/// `INVALID_PARAMS` rather than a generic failure because the fix is a parameter, and a client
/// should be able to offer "replace it?" without parsing English.
#[tokio::test]
async fn a_second_login_needs_force() {
    let fx = Harness::new();
    let hub = FakeHub::start(0).await;
    let (token_path, _server) = serve_with_account(&fx, &hub).await;

    let mut client = Client::connect(&fx.socket).await;
    client.hello().await;
    client
        .call(method::ACCOUNT_LOGIN, serde_json::json!({}))
        .await;
    // Wait for the first login to land, since that is what the second one collides with.
    for _ in 0..100 {
        if token_path.exists() {
            break;
        }
        tokio::time::sleep(Duration::from_millis(200)).await;
    }
    assert!(token_path.exists(), "the first login did not complete");
    let polls_after_first = hub.polls.load(Ordering::SeqCst);

    let refused = client
        .call(method::ACCOUNT_LOGIN, serde_json::json!({}))
        .await;
    let error = refused.error.expect("must refuse");
    assert_eq!(error.code, proto::code::INVALID_PARAMS);
    assert!(
        error.message.contains("PierreRouanet") && error.message.contains("--force"),
        "the refusal must name the account and the way past it: {}",
        error.message
    );
    assert_eq!(
        hub.polls.load(Ordering::SeqCst),
        polls_after_first,
        "the refusal must come before anything reaches Hugging Face — a device code burned to \
         arrive at the same answer is a code somebody might have been typing"
    );

    // With `force`, it starts.
    let forced = client
        .call(method::ACCOUNT_LOGIN, serde_json::json!({ "force": true }))
        .await;
    assert!(forced.error.is_none(), "{:?}", forced.error);
    assert_eq!(forced.result.unwrap()["user_code"], "A6MY-0314");
}

/// `account.status` is a read, so it answers without the privilege the other two need.
///
/// The gate is `Call::is_mutating`, and this drives it from the outside against a server whose
/// owning uid the test process is not: `login` and `logout` are refused, and `status` still
/// answers. Both halves matter — a support engineer has to be able to ask which account a robot
/// thinks it belongs to, and nobody who merely reached the socket may rebind it.
#[tokio::test]
async fn status_is_not_a_privileged_call() {
    let fx = Harness::new();
    let hub = FakeHub::start(0).await;
    // Owner uid set to something the test process is not, and no allowances — the shape
    // `an_unlisted_peer_cannot_mutate` uses for the update calls.
    let (token_path, _server) = serve_with_account_as(&fx, &hub, Some(u32::MAX)).await;

    assert!(
        !proto::Call::AccountStatus.is_mutating(),
        "account.status must never require change authority"
    );
    assert!(
        proto::Call::AccountLogin(proto::AccountLoginParams { force: false }).is_mutating()
            && proto::Call::AccountLogout.is_mutating(),
        "binding a robot to an account, and unbinding it, must both be authorised"
    );

    let mut client = Client::connect(&fx.socket).await;
    client.hello().await;

    for (method, params) in [
        (method::ACCOUNT_LOGIN, serde_json::json!({})),
        (method::ACCOUNT_LOGOUT, serde_json::json!({})),
    ] {
        let error = client
            .call(method, params)
            .await
            .error
            .unwrap_or_else(|| panic!("{method} should be denied"));
        assert_eq!(
            error.code,
            proto::code::PERMISSION_DENIED,
            "{method}: {error:?}"
        );
    }
    assert!(
        !token_path.exists(),
        "a denied login must not have reached Hugging Face, let alone stored anything"
    );

    let status = client
        .call(method::ACCOUNT_STATUS, serde_json::json!({}))
        .await;
    assert!(status.error.is_none(), "{:?}", status.error);
    assert!(status.result.unwrap()["account"].is_null());
}
