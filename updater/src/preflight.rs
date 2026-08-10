//! Preconditions checked before anything is downloaded or changed.
//!
//! Every failure here aborts cleanly with **no side effects**. See
//! `docs/design/updater-design.md` §7.2.
//!
//! Single-flight is *not* one of these checks: it is enforced by the on-disk lock
//! [`crate::journal::UpdateLock`], taken before any of this runs, and surfaces as
//! [`crate::Error::Busy`]. Listing it here as well would imply a second, redundant
//! mechanism.
//!
//! Run twice per apply: once with no manifest (clock, robot stopped, no live
//! session) *before* any network access, then again for the disk-space check once
//! the manifest's `size` is known. Ordering matters — the manifest fetch is HTTPS,
//! and an unsynced clock breaks it with an opaque TLS error rather than the
//! diagnostic the clock check exists to give.

use std::time::Duration;

use crate::Error;
use crate::robot::{RobotClient, SafeToRestart};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Check {
    /// The clock is plausible.
    ///
    /// A board with no battery-backed RTC boots with a wrong clock, and HTTPS
    /// then fails cert-date validation before any download can start. minisign
    /// itself is time-independent, but TLS is not.
    Clock,
    /// Not mid-motion.
    RobotStopped,
    /// No live telepresence session.
    NoRemoteSession,
    /// Room for download + extract + retained releases.
    DiskSpace,
}

#[derive(Debug, Clone)]
pub struct CheckResult {
    pub check: Check,
    pub passed: bool,
    /// Why it failed, safe to display.
    pub detail: Option<String>,
}

#[derive(Debug, Clone)]
pub struct Report {
    pub results: Vec<CheckResult>,
}

impl Report {
    pub fn passed(&self) -> bool {
        self.results.iter().all(|r| r.passed)
    }

    pub fn first_failure(&self) -> Option<&CheckResult> {
        self.results.iter().find(|r| !r.passed)
    }
}

pub struct Preflight<'a> {
    pub robot: &'a dyn RobotClient,
    /// Bytes needed for download + extract, from the manifest, plus headroom.
    pub required_bytes: u64,
    pub available_bytes: u64,
    /// Skip only the remote-session check. Never affects verification.
    pub interrupt_sessions: bool,
    pub robot_query_timeout: Duration,
}

/// Clock floor: a system time before this cannot be right, and TLS would fail
/// cert-date validation. 2025-01-01T00:00:00Z.
///
/// A board with no battery-backed RTC boots at the epoch (or at its image's build
/// date), so this catches exactly the "never synced NTP yet" case without needing
/// to talk to `timedatectl`.
const CLOCK_FLOOR_UNIX: i64 = 1_735_689_600;

impl Preflight<'_> {
    /// Run every check and report all results.
    ///
    /// Deliberately does **not** short-circuit: telling the user "clock is wrong
    /// AND disk is full" in one round beats making them fix one, retry, and
    /// discover the next.
    pub async fn run(&self) -> Result<Report, Error> {
        let mut results = Vec::new();

        results.push(self.check_clock());
        results.push(self.check_disk());
        results.push(self.check_robot_stopped().await);
        results.push(self.check_no_remote_session().await);

        Ok(Report { results })
    }

    fn check_clock(&self) -> CheckResult {
        let now = crate::journal::now_unix();
        let ok = now >= CLOCK_FLOOR_UNIX;
        CheckResult {
            check: Check::Clock,
            passed: ok,
            detail: (!ok).then(|| {
                "system clock is implausibly early (NTP has not synced); HTTPS would fail \
                 certificate date validation"
                    .to_owned()
            }),
        }
    }

    fn check_disk(&self) -> CheckResult {
        let ok = self.available_bytes >= self.required_bytes;
        CheckResult {
            check: Check::DiskSpace,
            passed: ok,
            detail: (!ok).then(|| {
                format!(
                    "needs {} bytes free, only {} available",
                    self.required_bytes, self.available_bytes
                )
            }),
        }
    }

    async fn check_robot_stopped(&self) -> CheckResult {
        let verdict = self.robot.safe_to_restart(self.robot_query_timeout).await;
        // Unreachable counts as safe: if the control loop isn't running, nothing is
        // moving — and that is precisely the case where an update is the fix.
        let passed = verdict.permits_restart();
        CheckResult {
            check: Check::RobotStopped,
            passed,
            detail: match &verdict {
                SafeToRestart::No(reason) => Some(reason.clone()),
                _ => None,
            },
        }
    }

    async fn check_no_remote_session(&self) -> CheckResult {
        if self.interrupt_sessions {
            return CheckResult {
                check: Check::NoRemoteSession,
                passed: true,
                detail: Some("session check bypassed by request".into()),
            };
        }

        let active = self
            .robot
            .remote_session_active(self.robot_query_timeout)
            .await;
        CheckResult {
            check: Check::NoRemoteSession,
            passed: !active,
            detail: active.then(|| {
                "a remote/telepresence session is active; restarting would drop it".to_owned()
            }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robot::{AbsentRobot, Health, RobotClient};

    /// A robot that answers however the test wants. The whole reason
    /// [`RobotClient`] is a trait: degraded paths must be testable without staging
    /// a real crash.
    struct FakeRobot {
        safe: SafeToRestart,
        session: bool,
    }

    #[async_trait::async_trait]
    impl RobotClient for FakeRobot {
        async fn safe_to_restart(&self, _t: Duration) -> SafeToRestart {
            self.safe.clone()
        }
        async fn health(&self, _t: Duration) -> Health {
            Health::Healthy
        }
        async fn model_api(&self, _t: Duration) -> Option<u32> {
            Some(1)
        }
        async fn remote_session_active(&self, _t: Duration) -> bool {
            self.session
        }
    }

    fn preflight<'a>(robot: &'a dyn RobotClient, required: u64, available: u64) -> Preflight<'a> {
        Preflight {
            robot,
            required_bytes: required,
            available_bytes: available,
            interrupt_sessions: false,
            robot_query_timeout: Duration::from_millis(50),
        }
    }

    #[tokio::test]
    async fn passes_when_everything_is_fine() {
        let robot = FakeRobot {
            safe: SafeToRestart::Yes,
            session: false,
        };
        let report = preflight(&robot, 100, 1_000).run().await.unwrap();
        assert!(report.passed(), "{:?}", report.first_failure());
    }

    #[tokio::test]
    async fn fails_when_disk_is_short() {
        let robot = FakeRobot {
            safe: SafeToRestart::Yes,
            session: false,
        };
        let report = preflight(&robot, 5_000, 1_000).run().await.unwrap();
        assert!(!report.passed());
        assert_eq!(report.first_failure().unwrap().check, Check::DiskSpace);
    }

    #[tokio::test]
    async fn fails_while_robot_is_moving() {
        let robot = FakeRobot {
            safe: SafeToRestart::No("walking".into()),
            session: false,
        };
        let report = preflight(&robot, 0, 1_000).run().await.unwrap();
        assert!(!report.passed());
        let failure = report.first_failure().unwrap();
        assert_eq!(failure.check, Check::RobotStopped);
        assert_eq!(failure.detail.as_deref(), Some("walking"));
    }

    /// The recovery case: `robotd` is dead, so nothing is moving, so preflight must
    /// let the update through. Blocking here would strand exactly the robots that
    /// need fixing.
    #[tokio::test]
    async fn unreachable_robot_passes_preflight() {
        let report = preflight(&AbsentRobot, 0, 1_000).run().await.unwrap();
        assert!(report.passed(), "{:?}", report.first_failure());
    }

    #[tokio::test]
    async fn active_session_blocks_unless_bypassed() {
        let robot = FakeRobot {
            safe: SafeToRestart::Yes,
            session: true,
        };

        let report = preflight(&robot, 0, 1_000).run().await.unwrap();
        assert_eq!(
            report.first_failure().unwrap().check,
            Check::NoRemoteSession
        );

        let mut bypass = preflight(&robot, 0, 1_000);
        bypass.interrupt_sessions = true;
        assert!(bypass.run().await.unwrap().passed());
    }

    /// All failures are reported in one pass, so the user fixes everything at once.
    #[tokio::test]
    async fn reports_every_failure_not_just_the_first() {
        let robot = FakeRobot {
            safe: SafeToRestart::No("walking".into()),
            session: true,
        };
        let report = preflight(&robot, 5_000, 1_000).run().await.unwrap();
        let failures = report.results.iter().filter(|r| !r.passed).count();
        assert_eq!(failures, 3, "{:?}", report.results);
    }

    #[tokio::test]
    async fn clock_check_passes_with_a_real_clock() {
        // Guards against the floor being set past "now" by mistake.
        let robot = FakeRobot {
            safe: SafeToRestart::Yes,
            session: false,
        };
        let report = preflight(&robot, 0, 1_000).run().await.unwrap();
        let clock = report
            .results
            .iter()
            .find(|r| r.check == Check::Clock)
            .unwrap();
        assert!(clock.passed, "clock floor must be in the past");
    }
}
