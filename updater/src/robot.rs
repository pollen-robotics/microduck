//! The engine's view of `robotd`.
//!
//! A trait rather than a concrete client for two reasons: the engine is built
//! before `robotd` exists (`docs/architecture.md` §9), and the degraded-mode
//! paths — `robotd` dead, crash-looping, or hung — must be testable without
//! staging a real crash (`docs/updater-design.md` §16.2).
//!
//! **Every method here is allowed to fail and must be timeout-bounded.** A dead
//! or silent `robotd` is a normal, expected answer. That is invariant 1 in
//! `docs/architecture.md` §1.1: `updaterd` is the recovery path, so it cannot
//! require the thing it is recovering.

use std::time::Duration;

/// Can the robot tolerate a restart of its control loop right now?
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SafeToRestart {
    Yes,
    /// Actively moving or otherwise mid-task. Carries a displayable reason.
    No(String),
    /// `robotd` did not answer.
    ///
    /// Treated as **safe**: if the control loop isn't running, nothing is moving,
    /// and this is exactly the case where an update is the fix. Making this an
    /// error would block recovery on precisely the robots that need it.
    Unreachable,
}

impl SafeToRestart {
    pub fn permits_restart(&self) -> bool {
        !matches!(self, SafeToRestart::No(_))
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
    /// (`docs/updater-design.md` §7.2).
    async fn safe_to_restart(&self, timeout: Duration) -> SafeToRestart;

    /// The post-apply health gate. Must return within `timeout` even if the peer
    /// holds the socket open and never replies.
    async fn health(&self, timeout: Duration) -> Health;

    /// Model API version the running daemon implements, for model compatibility
    /// checks (`docs/updater-design.md` §5.5). `None` when unreachable.
    async fn model_api(&self, timeout: Duration) -> Option<u32>;

    /// Is a telepresence/WebRTC session live? Restarting mid-session is a bad
    /// surprise (`docs/architecture.md` §5).
    ///
    /// Defaults to `false` when unknown: this check is a courtesy, and must never
    /// be the reason a recovery update is refused.
    async fn remote_session_active(&self, timeout: Duration) -> bool;
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
        // An answer we cannot parse is treated as unreachable rather than guessed at:
        // guessing "safe" could restart a walking robot.
        match serde_json::from_value::<crate::proto::SafeToRestartResult>(result) {
            Ok(answer) if answer.safe => SafeToRestart::Yes,
            Ok(answer) => SafeToRestart::No(
                answer
                    .reason
                    .unwrap_or_else(|| "robot reports it is not safe to restart".into()),
            ),
            Err(e) => {
                tracing::warn!(error = %e, "robotd answered safeToRestart in an unexpected shape");
                SafeToRestart::Unreachable
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
                Health::Unreachable
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
}
