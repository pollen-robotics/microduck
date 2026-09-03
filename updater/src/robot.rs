//! The engine's view of `robotd`.
//!
//! A trait rather than a concrete client for two reasons: the engine is built
//! before `robotd` exists (`docs/design/architecture.md` §9), and the degraded-mode
//! paths — `robotd` dead, crash-looping, or hung — must be testable without
//! staging a real crash (`docs/design/updater-design.md` §16.2).
//!
//! **Every method here is allowed to fail and must be timeout-bounded.** A dead
//! or silent `robotd` is a normal, expected answer. That is invariant 1 in
//! `docs/design/architecture.md` §1.1: `updaterd` is the recovery path, so it cannot
//! require the thing it is recovering.

use std::time::Duration;

/// Can the robot tolerate a restart of its control loop right now?
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SafeToRestart {
    Yes,
    /// Actively moving or otherwise mid-task. Carries a displayable reason.
    No(String),
    /// Answered, in a shape this `updaterd` cannot read. Does **not** permit a restart.
    ///
    /// The distinction from [`Self::Unreachable`] is the whole point, and getting it wrong is
    /// what this variant exists to fix: a reply that *arrived* is evidence of a `robotd` whose
    /// control loop is running, and reading its "no, I am mid-task" as "sure, go ahead" because
    /// a field was renamed is how an update restarts a walking robot. Silence is safe; an
    /// unreadable answer is not, and the two used to collapse into one variant that permitted
    /// the restart either way — against what that variant's own comment promised.
    ///
    /// Same split, for the same reason, as [`Health::Incompatible`] against
    /// [`Health::Unreachable`]. Carries serde's message, which names the field.
    ///
    /// The escape, when this blocks an update that needs to happen, is to make the robot
    /// genuinely silent — `systemctl stop robotd` — which is honest rather than a bypass: a
    /// stopped robot cannot be moving, and cannot be misread about it either. The refusal says
    /// so.
    Incompatible(String),
    /// `robotd` did not answer.
    ///
    /// Treated as **safe**: if the control loop isn't running, nothing is moving,
    /// and this is exactly the case where an update is the fix. Making this an
    /// error would block recovery on precisely the robots that need it.
    Unreachable,
}

impl SafeToRestart {
    pub fn permits_restart(&self) -> bool {
        !matches!(self, SafeToRestart::No(_) | SafeToRestart::Incompatible(_))
    }
}

/// Result of asking the new release whether it came up correctly.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Health {
    Healthy,
    /// Came up and reported a problem, but one belonging to the board rather than to the
    /// release — no servo power, no motor bus. Passes the gate: see
    /// [`crate::proto::HealthResult::degraded`].
    Degraded(String),
    /// Came up and reported a problem.
    Unhealthy(String),
    /// Answered, in a shape this `updaterd` cannot read. Fails the gate — an unreadable
    /// verdict is not a healthy one — but says so in different words, which is the point.
    ///
    /// Distinct from [`Self::Unreachable`] because the two ask for opposite things from
    /// whoever reads the outcome. "Unreachable" sends you to look at a daemon that is
    /// probably dead; this sends you to look at a *contract*, and the robot is likely fine.
    /// Reusing `Unreachable` for it cost an hour: the gate reported "not healthy within 30s:
    /// unreachable" about a `robotd` that was serving its socket and running its loop at
    /// 50 Hz, and had merely omitted one JSON field a newer parser required. See
    /// `docs/project/install-path-gap.md`.
    ///
    /// Carries serde's own message. It names the missing or unexpected field, which is the
    /// single most useful string available at that moment.
    Incompatible(String),
    /// Did not answer within the timeout — includes crash-looping and hung
    /// (socket open, no reply). Fails the gate: unproven is not healthy.
    Unreachable,
}

impl Health {
    pub fn is_healthy(&self) -> bool {
        matches!(self, Health::Healthy)
    }
}

/// Every method is timeout-bounded and allowed to fail. Wrap the underlying IO in
/// [`tokio::time::timeout`] and map elapsed-time to the `Unreachable` variants —
/// a hung peer (socket open, no reply) must look the same as a dead one.
#[async_trait::async_trait]
pub trait RobotClient: Send + Sync {
    /// Refuse to restart motor control mid-motion
    /// (`docs/design/updater-design.md` §7.2).
    async fn safe_to_restart(&self, timeout: Duration) -> SafeToRestart;

    /// The post-apply health gate. Must return within `timeout` even if the peer
    /// holds the socket open and never replies.
    async fn health(&self, timeout: Duration) -> Health;

    /// Model API version the running daemon implements, for model compatibility
    /// checks (`docs/design/updater-design.md` §5.5). `None` when unreachable.
    async fn model_api(&self, timeout: Duration) -> Option<u32>;

    /// Is a telepresence/WebRTC session live? Restarting mid-session is a bad
    /// surprise (`docs/design/architecture.md` §5).
    ///
    /// Defaults to `false` when unknown: this check is a courtesy, and must never
    /// be the reason a recovery update is refused.
    async fn remote_session_active(&self, timeout: Duration) -> bool;

    /// Tell `robotd` to re-read its policy slots from disk.
    ///
    /// Called after a policy set is swapped underneath it. `false` when the robot could not be
    /// reached or refused — the files are in place and it is still running the old ones, which is
    /// worth reporting rather than presenting as a success it has not acted on. A reboot fixes it
    /// either way, so this is never a reason to fail an install.
    async fn reload_policies(&self, timeout: Duration) -> bool;

    /// Every policy file the robot is currently pointed at — slots and skills both.
    ///
    /// `None` when it could not be asked, which is not the same as "none in use" and must never
    /// be read as one: it is what stops the library prune deleting the gait a silent robot is
    /// about to come back up on.
    ///
    /// Defaults to `None` for the same reason — a client that cannot answer this question causes
    /// nothing to be deleted, which is the failure this wants.
    async fn policy_paths(&self, _timeout: Duration) -> Option<Vec<String>> {
        None
    }
}

/// Talks to `robotd` over its unix socket.
pub struct SocketRobotClient {
    path: std::path::PathBuf,
}

impl SocketRobotClient {
    pub fn new(path: std::path::PathBuf) -> Self {
        Self { path }
    }
}

impl SocketRobotClient {
    /// One request/response exchange, entirely inside `timeout`.
    ///
    /// Every failure — connect refused, no reply, malformed reply — collapses to
    /// `None`, which callers map to their `Unreachable` variant. A wedged peer
    /// (socket open, silent) must be indistinguishable from a dead one, or the
    /// engine would hang on exactly the robot it is trying to repair.
    async fn ask(&self, call: &crate::proto::Call, timeout: Duration) -> Option<serde_json::Value> {
        let exchange = async {
            let stream = tokio::net::UnixStream::connect(&self.path).await.ok()?;
            let (read_half, mut write_half) = stream.into_split();

            let request = crate::proto::Request::call(crate::proto::Id::Number(1), call);
            let mut line = serde_json::to_vec(&request).ok()?;
            line.push(b'\n');

            use tokio::io::{AsyncBufReadExt, AsyncWriteExt};
            write_half.write_all(&line).await.ok()?;
            write_half.flush().await.ok()?;

            let mut reply = String::new();
            tokio::io::BufReader::new(read_half)
                .read_line(&mut reply)
                .await
                .ok()?;

            let response: crate::proto::Response = serde_json::from_str(reply.trim()).ok()?;
            response.result
        };

        match tokio::time::timeout(timeout, exchange).await {
            Ok(result) => result,
            Err(_elapsed) => {
                tracing::debug!(
                    method = call.method(),
                    "robotd did not answer within the timeout"
                );
                None
            }
        }
    }
}

#[async_trait::async_trait]
impl RobotClient for SocketRobotClient {
    async fn safe_to_restart(&self, timeout: Duration) -> SafeToRestart {
        let call = crate::proto::Call::RobotSafeToRestart;
        let Some(result) = self.ask(&call, timeout).await else {
            return SafeToRestart::Unreachable;
        };
        // An answer we cannot parse is not guessed at, and specifically is not read as safe:
        // guessing "safe" is what could restart a walking robot.
        match serde_json::from_value::<crate::proto::SafeToRestartResult>(result) {
            Ok(answer) if answer.safe => SafeToRestart::Yes,
            Ok(answer) => SafeToRestart::No(
                answer
                    .reason
                    .unwrap_or_else(|| "robot reports it is not safe to restart".into()),
            ),
            Err(e) => {
                tracing::warn!(error = %e, "robotd answered safeToRestart in an unexpected shape");
                SafeToRestart::Incompatible(e.to_string())
            }
        }
    }

    async fn health(&self, timeout: Duration) -> Health {
        let Some(result) = self.ask(&crate::proto::Call::RobotHealth, timeout).await else {
            return Health::Unreachable;
        };
        match serde_json::from_value::<crate::proto::HealthResult>(result) {
            Ok(answer) if answer.healthy => Health::Healthy,
            Ok(answer) if answer.degraded => Health::Degraded(
                answer
                    .reason
                    .unwrap_or_else(|| "robot reports degraded".into()),
            ),
            Ok(answer) => Health::Unhealthy(
                answer
                    .reason
                    .unwrap_or_else(|| "robot reports unhealthy".into()),
            ),
            Err(e) => {
                tracing::warn!(error = %e, "robotd answered health in an unexpected shape");
                Health::Incompatible(e.to_string())
            }
        }
    }

    async fn model_api(&self, timeout: Duration) -> Option<u32> {
        let result = self
            .ask(&crate::proto::Call::RobotModelApi, timeout)
            .await?;
        serde_json::from_value::<crate::proto::ModelApiResult>(result)
            .ok()
            .map(|answer| answer.model_api)
    }

    async fn reload_policies(&self, timeout: Duration) -> bool {
        self.ask(&crate::proto::Call::RobotReloadPolicies, timeout)
            .await
            .and_then(|v| serde_json::from_value::<crate::proto::IntentResult>(v).ok())
            .is_some_and(|result| result.accepted)
    }

    async fn policy_paths(&self, timeout: Duration) -> Option<Vec<String>> {
        let policies: crate::proto::PoliciesResult = serde_json::from_value(
            self.ask(&crate::proto::Call::RobotPolicies, timeout)
                .await?,
        )
        .ok()?;
        // The skills too: a fetched policy can fill a slot or answer to a name, and a prune that
        // only looked at slots would delete the bow somebody put on a button.
        let skills: Option<crate::proto::SkillsResult> =
            match self.ask(&crate::proto::Call::RobotSkills, timeout).await {
                Some(value) => Some(serde_json::from_value(value).ok()?),
                None => None,
            };
        Some(
            policies
                .slots
                .into_iter()
                .filter_map(|slot| slot.path)
                .chain(
                    skills
                        .into_iter()
                        .flat_map(|s| s.skills)
                        .filter_map(|skill| skill.path),
                )
                .collect(),
        )
    }

    async fn remote_session_active(&self, timeout: Duration) -> bool {
        // Defaults to false when unknown: this check is a courtesy and must never be
        // the reason a recovery update is refused.
        self.ask(&crate::proto::Call::RobotRemoteSessionActive, timeout)
            .await
            .and_then(|r| serde_json::from_value::<crate::proto::SessionActiveResult>(r).ok())
            .is_some_and(|answer| answer.active)
    }
}

/// A `robotd` that isn't there.
///
/// Not only a test double: it's the correct client for a component whose
/// `health` probe is `None`, and it documents the intended degraded behaviour.
pub struct AbsentRobot;

#[async_trait::async_trait]
impl RobotClient for AbsentRobot {
    async fn safe_to_restart(&self, _timeout: Duration) -> SafeToRestart {
        SafeToRestart::Unreachable
    }

    async fn health(&self, _timeout: Duration) -> Health {
        Health::Unreachable
    }

    async fn model_api(&self, _timeout: Duration) -> Option<u32> {
        None
    }

    async fn reload_policies(&self, _timeout: Duration) -> bool {
        false
    }

    async fn policy_paths(&self, _timeout: Duration) -> Option<Vec<String>> {
        None
    }

    async fn remote_session_active(&self, _timeout: Duration) -> bool {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn unreachable_robot_permits_restart() {
        // The recovery case: robotd is dead, so nothing is moving, so an update
        // must be allowed to proceed.
        let verdict = AbsentRobot.safe_to_restart(Duration::from_secs(1)).await;
        assert!(verdict.permits_restart());
    }

    #[tokio::test]
    async fn unreachable_robot_fails_health_gate() {
        // The other direction: absence must never be mistaken for success, or
        // auto-rollback would never trigger on a release that won't start.
        assert!(
            !AbsentRobot
                .health(Duration::from_secs(1))
                .await
                .is_healthy()
        );
    }

    /// Serve one canned `robot.health` result on a unix socket and ask about it.
    ///
    /// A real socket rather than a fake `RobotClient`, because the behaviour under test lives in
    /// `SocketRobotClient::health` — the deserialization step. A double implementing the trait
    /// would replace exactly the code that has the bug.
    async fn health_of(reply: serde_json::Value) -> Health {
        let dir = tempfile::tempdir().expect("tempdir");
        let path = dir.path().join("robot.sock");
        let listener = tokio::net::UnixListener::bind(&path).expect("bind");

        let server = tokio::spawn(async move {
            let (stream, _) = listener.accept().await.expect("accept");
            let (read_half, mut write_half) = stream.into_split();

            use tokio::io::{AsyncBufReadExt, AsyncWriteExt};
            let mut request = String::new();
            tokio::io::BufReader::new(read_half)
                .read_line(&mut request)
                .await
                .expect("read request");

            let response = crate::proto::Response::ok(Some(crate::proto::Id::Number(1)), &reply);
            let mut line = serde_json::to_vec(&response).expect("encode");
            line.push(b'\n');
            write_half.write_all(&line).await.expect("write");
            write_half.flush().await.expect("flush");
        });

        let health = SocketRobotClient::new(path)
            .health(Duration::from_secs(5))
            .await;
        server.await.expect("server");
        health
    }

    /// The reply an older `robotd` sends must still parse.
    ///
    /// This exact shape reverted a good release: `consecutive_stale_blocks` had been added to
    /// `ImuHealth` and released, and a branch that merged `main` before that sent an `imu`
    /// section without it. `robotd` was entirely healthy — socket served, both policies loaded,
    /// loop at 50 Hz — and the resident `updaterd` could not read `healthy: true`.
    ///
    /// Written as literal JSON, not as a struct with a field left out, because a struct cannot
    /// express "this field does not exist" — and that is the whole failure.
    #[tokio::test]
    async fn an_older_robotd_that_omits_a_health_field_is_still_healthy() {
        let health = health_of(serde_json::json!({
            "healthy": true,
            "bus": { "consecutive_errors": 0 },
            "imu": { "ready": true, "stale_blocks": 3 },
        }))
        .await;

        assert_eq!(health, Health::Healthy, "got {health:?}");
    }

    /// And an answer that genuinely cannot be read must not claim the robot is absent.
    ///
    /// `Unreachable` sends whoever reads the outcome to look at a daemon that is probably dead.
    /// When the daemon answered and the *contract* is what disagrees, that is an hour spent in
    /// the wrong place — which is what happened. The reason string carries serde's own message
    /// so the field is named.
    #[tokio::test]
    async fn an_unreadable_answer_is_incompatible_not_unreachable() {
        let health = health_of(serde_json::json!({ "healthy": "yes, very" })).await;

        match health {
            Health::Incompatible(reason) => assert!(!reason.is_empty(), "must name the problem"),
            other => panic!("expected Incompatible, got {other:?}"),
        }
    }

    /// Failing the gate is not negotiable: an unreadable verdict is not a healthy one.
    ///
    /// Split from the test above deliberately. The point of `Incompatible` is that it reads
    /// differently, not that it decides differently, and a future edit that softened it into a
    /// pass would be the worst possible reading of "the robot was probably fine".
    #[tokio::test]
    async fn incompatible_still_fails_the_gate() {
        assert!(!Health::Incompatible("missing field `imu`".into()).is_healthy());
    }
}
