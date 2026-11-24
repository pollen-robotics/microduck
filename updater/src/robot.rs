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

/// A [`Health`] verdict in the three fields [`crate::proto::ComponentStatus`] carries it in.
///
/// Five verdicts into two booleans and a string, and the lossy part is deliberate: the booleans
/// answer the only question the gate asks — commit or revert, and whose fault — while the string
/// keeps the distinction a reader needs and the gate does not.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HealthReport {
    pub healthy: bool,
    pub degraded: bool,
    pub reason: Option<String>,
}

impl Health {
    pub fn is_healthy(&self) -> bool {
        matches!(self, Health::Healthy)
    }

    /// How this verdict is reported to whoever asked for status.
    ///
    /// Not `is_healthy`, which is the gate's question and answers `false` for four different
    /// situations: a board with no servo power (which the gate *commits* onto), a broken control
    /// loop, a `robotd` that said nothing, and one that said something unreadable. A status view
    /// built on `is_healthy` alone prints the same word for all four — see
    /// `docs/design/updater-design.md` §8 for the three-way question this is the reporting half
    /// of.
    ///
    /// `degraded` is set for exactly the verdict the gate commits on and blames the board for.
    /// The other three carry their reason in words instead: the two booleans genuinely cannot
    /// tell "did not answer" from "answered unreadably", and conflating *those* two is what
    /// `Health::Incompatible` exists to stop, so the string has to say it.
    pub fn report(&self) -> HealthReport {
        let (degraded, reason) = match self {
            Health::Healthy => (false, None),
            Health::Degraded(reason) => (true, Some(reason.clone())),
            Health::Unhealthy(reason) => (false, Some(reason.clone())),
            // The gate's own words for this, and for its reason: "unreachable" about a robotd
            // that is serving its socket sends the reader to the wrong half of the system.
            Health::Incompatible(detail) => (
                false,
                Some(format!(
                    "answered in a shape this updaterd cannot read ({detail}) — the robot may be \
                     fine and the contract is what disagrees"
                )),
            ),
            Health::Unreachable => (false, Some("unreachable: robotd did not answer".to_owned())),
        };
        HealthReport {
            healthy: self.is_healthy(),
            degraded,
            reason,
        }
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
    /// One request/response exchange with `robotd` — see [`ask_socket`].
    async fn ask(&self, call: &crate::proto::Call, timeout: Duration) -> Option<serde_json::Value> {
        ask_socket(&self.path, call, timeout).await
    }
}

/// One request/response exchange on a daemon's unix socket, entirely inside `timeout`.
///
/// Every failure — connect refused, no reply, malformed reply, an error answer — collapses to
/// `None`, which callers map to their `Unreachable` variant. A wedged peer (socket open, silent)
/// must be indistinguishable from a dead one, or the engine would hang on exactly the robot it is
/// trying to repair.
pub async fn ask_socket(
    path: &std::path::Path,
    call: &crate::proto::Call,
    timeout: Duration,
) -> Option<serde_json::Value> {
    let exchange = async {
        let stream = tokio::net::UnixStream::connect(path).await.ok()?;
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
                path = %path.display(),
                "no answer within the timeout"
            );
            None
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

    /// Degraded is the one verdict that fails `is_healthy` and blames the board, and it is the
    /// everyday state of a robot on a desk. Reporting it as a plain "not healthy" is what made a
    /// committed release look reverted.
    #[test]
    fn a_degraded_report_says_whose_fault_it_is() {
        let report = Health::Degraded("no answer from the motor bus".into()).report();

        assert!(!report.healthy);
        assert!(report.degraded);
        assert_eq!(
            report.reason.as_deref(),
            Some("no answer from the motor bus")
        );
    }

    /// The three that revert must not be reported as the one that commits, and each must say
    /// which of the three it was -- the booleans cannot, so the reason has to.
    #[test]
    fn the_verdicts_that_revert_are_reported_apart() {
        for verdict in [
            Health::Unhealthy("no policy".into()),
            Health::Incompatible("missing field `imu`".into()),
            Health::Unreachable,
        ] {
            let report = verdict.report();
            assert!(!report.healthy, "{verdict:?}");
            assert!(
                !report.degraded,
                "{verdict:?} must not read as a board fault"
            );
            assert!(
                report.reason.is_some(),
                "{verdict:?} must say what happened"
            );
        }

        // And the two that are easiest to confuse say so in their own words.
        let unreadable = Health::Incompatible("missing field `imu`".into()).report();
        assert!(
            unreadable.reason.unwrap().contains("cannot read"),
            "an unreadable answer must not read as an absent robot"
        );
        let absent = Health::Unreachable.report();
        assert!(absent.reason.unwrap().contains("did not answer"));
    }

    /// A healthy robot has nothing to explain, and inventing a reason would put a string on
    /// every status line that a reader then has to learn to ignore.
    #[test]
    fn a_healthy_report_carries_no_reason() {
        let report = Health::Healthy.report();

        assert!(report.healthy);
        assert!(!report.degraded);
        assert_eq!(report.reason, None);
    }
}
