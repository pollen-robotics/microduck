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
    /// The `--from` directory is one *this process* can read.
    ///
    /// `updaterd.service` sets `PrivateTmp=yes`, which gives the unit its own `/tmp` **and**
    /// its own `/var/tmp`. A release copied to either from a shell — the obvious place to put
    /// one, and where `scripts/dev-push.sh` used to put it — is therefore not the one this
    /// process sees, and every message downstream is a lie: "no manifest for version X in
    /// /var/tmp/duck-sideload", against a directory whose `ls` shows that exact manifest, its
    /// signature and the artifact. Nothing in that output points at the namespace, and the
    /// caller has done nothing wrong.
    ///
    /// So it is a named check rather than a better error message further down: it fails before
    /// any lookup, it says which mount namespace is responsible, and a board that does not
    /// privatise `/var/tmp` passes it without noticing it exists.
    SideloadDir,
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
    /// The directory `apply --from <dir>` was pointed at, if any. `None` for every other
    /// target, which reads its manifest from the configured source instead.
    pub from_dir: Option<&'a std::path::Path>,
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
        results.push(self.check_sideload_dir());
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

    /// Whether `--from <dir>` names a directory this process can see.
    ///
    /// Two failures, and they need different sentences. A path under `/tmp` or `/var/tmp` is
    /// almost certainly there for the caller and hidden from here by `PrivateTmp=yes`
    /// ([`Check::SideloadDir`]), so the message names the namespace and where to put the
    /// release instead. Anywhere else, a missing directory is a missing directory.
    ///
    /// `exists()` and not a permission check: this runs as root, which reads any path it can
    /// reach, so what remains is reachability.
    fn check_sideload_dir(&self) -> CheckResult {
        let Some(dir) = self.from_dir else {
            return CheckResult {
                check: Check::SideloadDir,
                passed: true,
                detail: None,
            };
        };
        if dir.exists() {
            return CheckResult {
                check: Check::SideloadDir,
                passed: true,
                detail: None,
            };
        }

        let private = dir.starts_with("/tmp") || dir.starts_with("/var/tmp");
        CheckResult {
            check: Check::SideloadDir,
            passed: false,
            detail: Some(if private {
                format!(
                    "{} is not there for updaterd, whichever shell created it: this unit runs \
                     with PrivateTmp=yes, so it has a /tmp and a /var/tmp of its own and neither \
                     is the one you copied into. The release is fine — put it anywhere outside \
                     those two (a home directory, or /var/lib) and install from there.",
                    dir.display()
                )
            } else {
                format!("{} does not exist", dir.display())
            }),
        }
    }

    async fn check_robot_stopped(&self) -> CheckResult {
        let verdict = self.robot.safe_to_restart(self.robot_query_timeout).await;
        // Unreachable counts as safe: if the control loop isn't running, nothing is
        // moving — and that is precisely the case where an update is the fix. An answer
        // that arrived and could not be read does not, because the loop *is* running.
        let passed = verdict.permits_restart();
        CheckResult {
            check: Check::RobotStopped,
            passed,
            detail: match &verdict {
                SafeToRestart::No(reason) => Some(reason.clone()),
                // Names the contract rather than the robot, and names the way out. Without the
                // second half this is a refusal with no stated remedy, on a board where the
                // remedy is not guessable: the robot looks fine, because it is.
                SafeToRestart::Incompatible(detail) => Some(format!(
                    "robotd answered in a shape this updaterd cannot read ({detail}), so whether \
                     it is moving is unknown and a restart is refused. The robot may be perfectly \
                     healthy and the contract is what disagrees — usually a release that added a \
                     field. To update anyway, make it genuinely stopped rather than unreadable: \
                     systemctl stop robotd"
                )),
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
    use std::path::Path;

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
        async fn reload_policies(&self, _timeout: Duration) -> bool {
            true
        }
    }

    fn preflight<'a>(robot: &'a dyn RobotClient, required: u64, available: u64) -> Preflight<'a> {
        Preflight {
            robot,
            required_bytes: required,
            available_bytes: available,
            interrupt_sessions: false,
            robot_query_timeout: Duration::from_millis(50),
            from_dir: None,
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

    /// The failure a `PrivateTmp=yes` unit gives for a directory the caller can see, and the
    /// reason this check exists at all: without it the first error is "no manifest for version X
    /// in /var/tmp/duck-sideload", which is unfalsifiable from a shell where `ls` lists the
    /// manifest. The namespace has to be named, or there is nothing to act on.
    #[tokio::test]
    async fn a_sideload_dir_under_var_tmp_names_the_namespace() {
        let robot = FakeRobot {
            safe: SafeToRestart::Yes,
            session: false,
        };
        let mut pf = preflight(&robot, 0, 1_000);
        let dir = Path::new("/var/tmp/duck-sideload-does-not-exist");
        pf.from_dir = Some(dir);

        let report = pf.run().await.unwrap();
        let failure = report.first_failure().expect("an invisible dir must fail");
        assert_eq!(failure.check, Check::SideloadDir);
        let detail = failure.detail.as_deref().unwrap_or_default();
        assert!(detail.contains("PrivateTmp=yes"), "{detail}");
        assert!(detail.contains("duck-sideload-does-not-exist"), "{detail}");
    }

    /// Outside `/tmp`, the same missing directory is just missing — telling someone about mount
    /// namespaces when they typo'd a path sends them after the wrong thing.
    #[tokio::test]
    async fn a_missing_dir_elsewhere_is_reported_plainly() {
        let robot = FakeRobot {
            safe: SafeToRestart::Yes,
            session: false,
        };
        let mut pf = preflight(&robot, 0, 1_000);
        pf.from_dir = Some(Path::new("/var/lib/robot/no-such-sideload"));

        let report = pf.run().await.unwrap();
        let failure = report.first_failure().expect("a missing dir must fail");
        assert_eq!(failure.check, Check::SideloadDir);
        let detail = failure.detail.as_deref().unwrap_or_default();
        assert!(!detail.contains("PrivateTmp"), "{detail}");
        assert!(detail.contains("does not exist"), "{detail}");
    }

    /// A directory that is there passes, including one under `/var/tmp`: this check is about
    /// what the process can read, not about a policy on where a release may live. `updaterd`
    /// running as a CLI (`updaterd install --from`) is outside the unit's namespace and sees
    /// `/var/tmp` normally — refusing it there would break the bootstrap path.
    #[tokio::test]
    async fn a_visible_dir_passes_wherever_it_is() {
        let robot = FakeRobot {
            safe: SafeToRestart::Yes,
            session: false,
        };
        let dir = tempfile::tempdir().unwrap();
        let mut pf = preflight(&robot, 0, 1_000);
        pf.from_dir = Some(dir.path());

        let report = pf.run().await.unwrap();
        assert!(report.passed(), "{:?}", report.first_failure());
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

    /// The pair above and below are one decision, and it used to be made the wrong way: an
    /// unreadable reply was mapped to `Unreachable`, which permits a restart, so a `robotd`
    /// answering "I am walking" in a shape one field newer was read as "go ahead".
    ///
    /// Silence means the control loop is not running. An answer means it is. Those must not
    /// share a verdict, whatever else changes here.
    #[tokio::test]
    async fn an_unreadable_answer_blocks_the_restart() {
        let robot = FakeRobot {
            safe: SafeToRestart::Incompatible("missing field `safe`".into()),
            session: false,
        };
        let report = preflight(&robot, 0, 1_000).run().await.unwrap();

        let failure = report.first_failure().expect("must not pass");
        assert_eq!(failure.check, Check::RobotStopped);
        let detail = failure.detail.as_deref().unwrap_or_default();
        // The reason serde gave, so the field that broke it is in the message.
        assert!(detail.contains("missing field"), "{detail}");
        // And the way out, which is not guessable from a robot that looks healthy.
        assert!(detail.contains("systemctl stop robotd"), "{detail}");
    }

    /// A refusal that cannot be escaped is a bricked update path, and the escape here is not a
    /// flag: stopping `robotd` turns an unreadable answer into an absent one, which is the
    /// honest version of "nothing is moving".
    #[tokio::test]
    async fn stopping_the_robot_is_the_way_past_it() {
        let unreadable = FakeRobot {
            safe: SafeToRestart::Incompatible("missing field `safe`".into()),
            session: false,
        };
        assert!(
            !preflight(&unreadable, 0, 1_000)
                .run()
                .await
                .unwrap()
                .passed()
        );
        assert!(
            preflight(&AbsentRobot, 0, 1_000)
                .run()
                .await
                .unwrap()
                .passed()
        );
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
