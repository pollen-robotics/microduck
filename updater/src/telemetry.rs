//! Anonymous usage data, sent to PostHog EU.
//!
//! On by default; `telemetry.set` is the owner's switch, and `MICRODUCK_TELEMETRY=0` or
//! `DO_NOT_TRACK=1` in `updaterd`'s environment forces it off. Events carry a random robot id,
//! the board, versions and the CPU architecture — never an IP, a name, an account, a serial, logs
//! or media. `docs/robot/telemetry.md` owns that list; change both together.
//!
//! The same design as Reachy Mini's daemon: a queue in memory, flushed once a minute by a task,
//! and a state file holding the choice, the id and the last version seen.

use std::collections::VecDeque;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

use crate::{Error, proto};

/// Where the state lives when the server is not told otherwise. `updaterd` passes its
/// `state_dir`; this is that directory on a robot.
pub const DEFAULT_PATH: &str = "/var/lib/robot/updater/telemetry.json";

const POSTHOG_URL: &str = "https://eu.i.posthog.com/batch/";
/// Public, write-only project key of the microduck project.
const POSTHOG_KEY: &str = "phc_CHtC7Bhn9RUnx9y7qGBaFjXNvgqidxvd57W9Y3cFSaWk";

const FLUSH_INTERVAL: Duration = Duration::from_secs(60);
const HEARTBEAT_INTERVAL: Duration = Duration::from_secs(24 * 3600);
const MAX_QUEUE: usize = 500;
const VERSION: &str = env!("CARGO_PKG_VERSION");

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
struct State {
    enabled: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    robot_id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    last_version: Option<String>,
}

struct Inner {
    state: State,
    queue: VecDeque<Value>,
}

pub struct Telemetry {
    path: PathBuf,
    /// The properties every event carries.
    envelope: Value,
    inner: Mutex<Inner>,
}

impl Telemetry {
    pub fn new(path: PathBuf) -> Self {
        Self::with_envelope(path, envelope())
    }

    fn with_envelope(path: PathBuf, envelope: Value) -> Self {
        let state = load(&path);
        Self {
            path,
            envelope,
            inner: Mutex::new(Inner {
                state,
                queue: VecDeque::new(),
            }),
        }
    }

    pub fn status(&self) -> proto::TelemetryResult {
        result(&self.lock().state)
    }

    /// Persist the owner's choice and apply it at once. Off forgets the robot id and the last
    /// version, so turning it back on starts an identity nothing links to the old one.
    pub fn set(&self, enabled: bool) -> Result<proto::TelemetryResult, Error> {
        let mut inner = self.lock();
        let next = if enabled {
            State {
                enabled,
                robot_id: match inner.state.robot_id.clone() {
                    Some(id) => Some(id),
                    None => Some(new_id()?),
                },
                ..inner.state.clone()
            }
        } else {
            inner.queue.clear();
            State::default()
        };
        self.persist(&next)?;
        inner.state = next;
        Ok(result(&inner.state))
    }

    /// Report this start, then every minute note what the policy slots run and flush the queue,
    /// and send a heartbeat once a day.
    pub async fn run(self: Arc<Self>, robot_socket: PathBuf) {
        self.started();
        let client = match crate::source::http::client() {
            Ok(client) => client,
            Err(e) => {
                tracing::warn!(error = %e, "telemetry off for this run");
                return;
            }
        };
        let since = tokio::time::Instant::now();
        let mut flush = tokio::time::interval(FLUSH_INTERVAL);
        let mut heartbeat =
            tokio::time::interval_at(since + HEARTBEAT_INTERVAL, HEARTBEAT_INTERVAL);
        let mut policies = None;
        loop {
            tokio::select! {
                _ = flush.tick() => {
                    self.watch_policies(&robot_socket, &mut policies).await;
                    self.flush(&client).await;
                }
                _ = heartbeat.tick() => self.capture(
                    "daily_heartbeat",
                    json!({ "uptime_s": since.elapsed().as_secs() }),
                ),
            }
        }
    }

    /// `daemon_started`, and `update_result` when the version moved since the last start.
    fn started(&self) {
        let previous = {
            let mut inner = self.lock();
            if !active(&inner.state) {
                return;
            }
            let mut next = inner.state.clone();
            if next.robot_id.is_none() {
                match new_id() {
                    Ok(id) => next.robot_id = Some(id),
                    Err(e) => {
                        tracing::warn!(error = %e, "telemetry off for this run");
                        return;
                    }
                }
            }
            let previous = next.last_version.replace(VERSION.to_owned());
            if let Err(e) = self.persist(&next) {
                tracing::warn!(error = %e, "telemetry off for this run");
                return;
            }
            inner.state = next;
            previous
        };
        self.capture("daemon_started", json!({}));
        if let Some(from) = previous.filter(|v| v != VERSION) {
            self.capture(
                "update_result",
                // No `success`: a start cannot tell an update from a rollback or a `select`. A
                // downgrade in from/to is what a rollback looks like.
                json!({ "from_version": from, "to_version": VERSION }),
            );
        }
    }

    /// `policies` once per robot id, the first time `robotd` answers, naming what every slot runs;
    /// then `policy_loaded` for each slot whose policy changed since the last answer.
    async fn watch_policies(
        &self,
        robot_socket: &Path,
        last: &mut Option<(String, serde_json::Map<String, Value>)>,
    ) {
        let id = {
            let inner = self.lock();
            match inner
                .state
                .robot_id
                .clone()
                .filter(|_| active(&inner.state))
            {
                Some(id) => id,
                None => return,
            }
        };
        let Some(answer) = crate::robot::ask_socket(
            robot_socket,
            &proto::Call::RobotPolicies,
            Duration::from_secs(2),
        )
        .await
        else {
            return;
        };
        let Ok(result) = serde_json::from_value::<proto::PoliciesResult>(answer) else {
            return;
        };
        let now: serde_json::Map<String, Value> = result
            .slots
            .iter()
            .filter(|_| result.enabled)
            .filter_map(|s| Some((format!("policy_{}", s.slot), policy_id(s)?.into())))
            .collect();
        // A new id — off, then on again within a run — gets its own snapshot.
        match last.take().filter(|(was, _)| *was == id) {
            None => {
                let mut properties = now.clone();
                properties.insert("mode".into(), result.mode.clone().into());
                self.capture("policies", Value::Object(properties));
            }
            Some((_, before)) => {
                for key in changed(&before, &now) {
                    let slot = key.trim_start_matches("policy_");
                    let origin = result
                        .slots
                        .iter()
                        .find(|s| s.slot == slot)
                        .map(|s| &s.origin);
                    self.capture(
                        "policy_loaded",
                        json!({ "slot": slot, "mode": result.mode, "origin": origin, "policy": now[key] }),
                    );
                }
            }
        }
        *last = Some((id, now));
    }

    fn capture(&self, event: &str, properties: Value) {
        let mut inner = self.lock();
        let Some(id) = inner
            .state
            .robot_id
            .clone()
            .filter(|_| active(&inner.state))
        else {
            return;
        };
        let mut merged = self.envelope.clone();
        if let (Some(all), Value::Object(extra)) = (merged.as_object_mut(), properties) {
            all.extend(extra);
        }
        if inner.queue.len() >= MAX_QUEUE {
            inner.queue.pop_front();
        }
        // No `timestamp`: the RTC has no battery and resets on a hard power-off, so before NTP
        // the clock is wrong. PostHog stamps an event with the time it arrives instead; a queued
        // event is late by its wait.
        inner
            .queue
            .push_back(json!({ "event": event, "distinct_id": id, "properties": merged }));
    }

    async fn flush(&self, client: &reqwest::Client) {
        let batch: Vec<Value> = self.lock().queue.drain(..).collect();
        if batch.is_empty() {
            return;
        }
        let sent = client
            .post(POSTHOG_URL)
            .timeout(Duration::from_secs(10))
            .json(&json!({ "api_key": POSTHOG_KEY, "batch": batch }))
            .send()
            .await;
        let retry = match sent {
            Ok(response) if response.status().is_success() => {
                tracing::debug!(events = batch.len(), "telemetry sent");
                false
            }
            Ok(response) => {
                let status = response.status();
                let retry = status.as_u16() == 429 || status.is_server_error();
                if !retry {
                    tracing::warn!(%status, "telemetry rejected");
                }
                retry
            }
            // Offline, or before NTP, when TLS cannot validate anything.
            Err(e) => {
                tracing::debug!(error = %e, "telemetry not sent");
                true
            }
        };
        if retry {
            let mut inner = self.lock();
            // Dropped if the owner opted out, or got a new id, while it was in flight.
            if batch[0]["distinct_id"].as_str() == inner.state.robot_id.as_deref() {
                for event in batch.into_iter().rev() {
                    inner.queue.push_front(event);
                }
                inner.queue.truncate(MAX_QUEUE);
            }
        }
    }

    fn persist(&self, state: &State) -> Result<(), Error> {
        let bytes = serde_json::to_vec(state).expect("State serialises");
        crate::fsutil::write_atomic(&self.path, &bytes)
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Inner> {
        self.inner
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }
}

/// The stored choice. No file is the default, on; a file that cannot be read is off, so a
/// corrupted choice never turns into consent.
fn load(path: &Path) -> State {
    match std::fs::read(path) {
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => State {
            enabled: true,
            ..State::default()
        },
        Ok(bytes) => serde_json::from_slice(&bytes).unwrap_or_else(|e| {
            tracing::warn!(path = %path.display(), error = %e, "unreadable telemetry state; off");
            State::default()
        }),
        Err(e) => {
            tracing::warn!(path = %path.display(), error = %e, "unreadable telemetry state; off");
            State::default()
        }
    }
}

fn result(state: &State) -> proto::TelemetryResult {
    proto::TelemetryResult {
        enabled: state.enabled,
        active: active(state),
    }
}

fn active(state: &State) -> bool {
    state.enabled && !blocked_by_env()
}

/// The `policy_<slot>` keys whose policy is new or different in `now`. A slot that emptied is not
/// a load, so it is not among them.
fn changed<'a>(
    before: &serde_json::Map<String, Value>,
    now: &'a serde_json::Map<String, Value>,
) -> Vec<&'a str> {
    now.iter()
        .filter(|(key, policy)| before.get(*key) != Some(*policy))
        .map(|(key, _)| key.as_str())
        .collect()
}

/// What a slot runs, as telemetry may name it: `org/name@revision/file` for a policy from the
/// Hub — public, since both the official set and `policy.fetch` download without a token — and
/// only `local` for a file put on the robot by hand, whose path could say anything. `None` for an
/// empty slot.
fn policy_id(slot: &proto::PolicySlot) -> Option<String> {
    policy_id_at(
        Path::new(slot.path.as_deref()?),
        crate::policy::installed(Path::new(crate::policy::POLICY_ROOT)),
    )
}

fn policy_id_at(path: &Path, official: Option<crate::policy::Source>) -> Option<String> {
    let file = path.file_name()?.to_string_lossy();
    if path.starts_with(robotd_params::POLICY_DIR) {
        return Some(match official {
            Some(set) => format!("{}@{}/{file}", set.repo, set.version),
            None => format!("official/{file}"),
        });
    }
    // `<library>/<org>/<name>/<revision>/<file>`, laid out by `policy::fetch`.
    if let Ok(rest) = path.strip_prefix(crate::policy::LIBRARY_ROOT) {
        let parts: Vec<_> = rest.iter().map(|c| c.to_string_lossy()).collect();
        if let [org, name, revision, _] = parts.as_slice() {
            return Some(format!("{org}/{name}@{revision}/{file}"));
        }
    }
    Some("local".into())
}

fn blocked_by_env() -> bool {
    let var = |k| {
        std::env::var(k)
            .unwrap_or_default()
            .trim()
            .to_ascii_lowercase()
    };
    matches!(
        var("MICRODUCK_TELEMETRY").as_str(),
        "0" | "false" | "no" | "off"
    ) || matches!(var("DO_NOT_TRACK").as_str(), "1" | "true" | "yes" | "on")
}

/// 128 random bits, in hex. Random rather than derived from the SoC serial, so it identifies a
/// robot to nobody but this file.
fn new_id() -> Result<String, Error> {
    let path = Path::new("/dev/urandom");
    let mut bytes = [0u8; 16];
    std::fs::File::open(path)
        .and_then(|mut f| f.read_exact(&mut bytes))
        .map_err(|source| Error::Io {
            path: path.to_path_buf(),
            source,
        })?;
    Ok(bytes.iter().map(|b| format!("{b:02x}")).collect())
}

fn envelope() -> Value {
    json!({
        "daemon_version": VERSION,
        "os_version": format!("{} {}", os_release("ID"), os_release("VERSION_ID")).trim(),
        // The image build: `VERSION_ID` is Yocto's release, the same on every image built from it.
        "os_build": os_release("BUILD_ID"),
        "board": robotd_params::board::Board::detected().map_or("unknown", |b| b.label()),
        "arch": std::env::consts::ARCH,
        "$geoip_disable": true,
        "$process_person_profile": false,
    })
}

/// One field of `/etc/os-release`, or empty.
fn os_release(key: &str) -> String {
    std::fs::read_to_string("/etc/os-release")
        .unwrap_or_default()
        .lines()
        .find_map(|l| l.strip_prefix(key)?.strip_prefix('='))
        .map(|v| v.trim_matches('"').to_owned())
        .unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(dir: &tempfile::TempDir) -> Telemetry {
        Telemetry::with_envelope(dir.path().join("telemetry.json"), json!({}))
    }

    #[test]
    fn on_by_default_and_off_forgets_the_id() {
        let dir = tempfile::tempdir().unwrap();
        assert!(at(&dir).status().enabled, "no file means the default, on");

        let t = at(&dir);
        t.set(true).unwrap();
        let first = t.lock().state.robot_id.clone().unwrap();
        assert_eq!(first.len(), 32);

        t.set(false).unwrap();
        assert!(!at(&dir).status().enabled, "off survives a restart");
        assert_eq!(at(&dir).lock().state, State::default(), "and keeps nothing");

        let t = at(&dir);
        t.set(true).unwrap();
        assert_ne!(t.lock().state.robot_id.clone().unwrap(), first);
    }

    #[test]
    fn an_unreadable_file_is_off() {
        let dir = tempfile::tempdir().unwrap();
        std::fs::write(dir.path().join("telemetry.json"), b"{not json").unwrap();
        assert!(!at(&dir).status().enabled);
    }

    #[test]
    fn nothing_is_queued_while_off() {
        let dir = tempfile::tempdir().unwrap();
        let t = at(&dir);
        t.set(true).unwrap();
        t.capture("daemon_started", json!({}));
        assert_eq!(t.lock().queue.len(), 1);
        t.set(false).unwrap();
        t.capture("daemon_started", json!({}));
        assert!(
            t.lock().queue.is_empty(),
            "off drops the queue and queues nothing more"
        );
    }

    #[test]
    fn only_a_slot_whose_policy_changed_is_reported() {
        let slots = |pairs: &[(&str, &str)]| -> serde_json::Map<String, Value> {
            pairs
                .iter()
                .map(|(k, v)| (k.to_string(), json!(v)))
                .collect()
        };
        let before = slots(&[
            ("policy_walk", "a@v8/walk.onnx"),
            ("policy_stand", "a@v8/stand.onnx"),
        ]);

        assert!(
            changed(&before, &before).is_empty(),
            "the same answer is no event"
        );
        assert_eq!(
            changed(
                &before,
                &slots(&[
                    ("policy_walk", "b@main/walk.onnx"),
                    ("policy_stand", "a@v8/stand.onnx")
                ])
            ),
            vec!["policy_walk"],
        );
        assert_eq!(
            changed(
                &before,
                &slots(&[
                    ("policy_walk", "a@v8/walk.onnx"),
                    ("policy_stand", "a@v8/stand.onnx"),
                    ("policy_kick", "local")
                ])
            ),
            vec!["policy_kick"],
            "a slot filled for the first time is a load"
        );
        assert!(
            changed(&before, &slots(&[("policy_walk", "a@v8/walk.onnx")])).is_empty(),
            "a slot that emptied is not"
        );
    }

    #[test]
    fn a_policy_is_named_by_its_hub_repo_and_a_hand_placed_one_is_not() {
        let set = crate::policy::Source {
            repo: "pollen-robotics/microduck-policies".into(),
            version: "v8".into(),
        };
        let id = |p: &str| policy_id_at(Path::new(p), Some(set.clone())).unwrap();
        assert_eq!(
            id("/opt/robot/policies/current/walk.onnx"),
            "pollen-robotics/microduck-policies@v8/walk.onnx"
        );
        assert_eq!(
            id("/var/lib/robot/policies/RemiFabre/microduck-flamingo/main/policy.onnx"),
            "RemiFabre/microduck-flamingo@main/policy.onnx"
        );
        assert_eq!(id("/home/fabien/secret-project/walk.onnx"), "local");
    }
}
