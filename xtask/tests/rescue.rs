//! What the boot recovery net decides: `scripts/robot-boot-check` and `scripts/robot-rescue`,
//! exercised against a temporary tree.
//!
//! This is the code that has to work when everything else does not, and the code least amenable to a
//! test: on a real board it runs only when a release cannot start. So both halves are written to be
//! askable without a board — the rescue's decision is a pure function of two symlinks and a
//! breadcrumb, and the check's is a pure function of what `systemctl show` answers. `ROBOT_*`
//! environment variables move the tree, the state dir, the rescue's path and the uptime source, and a
//! stub `systemctl` on `PATH` supplies unit states and records whether a reboot was asked for.
//!
//! What that leaves uncovered is real and worth naming: **systemd itself**. Whether the timer fires
//! at `OnBootSec=1`, whether the oneshot's `Conflicts=shutdown.target` keeps it off the way down,
//! and whether `NRestarts` reads the way this assumes on a crash-looping unit — none of that is here.
//! It needs real systemd — `systemd-nspawn`, or a privileged container with it as pid 1 — which
//! `docs/project/install-path-gap.md` makes the case for and `docs/design/boot-recovery-net.md` says
//! why this mechanism in particular wants it.
//!
//! In `xtask` because these are *repository scripts* rather than any crate's behaviour, which is what
//! the rest of this directory is for. `sh` and not `bash`: they run on a board where the interpreter
//! is whatever `/bin/sh` is, and the flag detection inside the rescue (`mv -T` on GNU, `mv -h` on
//! BSD) exists precisely because the same script has to work here and there.

use std::os::unix::fs::symlink;
use std::path::{Path, PathBuf};
use std::process::{Command, Output};

/// A board tree: `releases/<version>` directories, and whichever symlinks the case needs.
struct Board {
    dir: tempfile::TempDir,
}

impl Board {
    fn with_releases(versions: &[&str]) -> Self {
        let dir = tempfile::tempdir().expect("tempdir");
        for version in versions {
            std::fs::create_dir_all(dir.path().join("releases").join(version)).expect("release");
        }
        std::fs::create_dir_all(dir.path().join("state")).expect("state dir");
        Self { dir }
    }

    fn install(&self) -> PathBuf {
        self.dir.path().to_path_buf()
    }

    fn state(&self) -> PathBuf {
        self.dir.path().join("state")
    }

    /// Link `name` at a release the way the store does: a target relative to the install dir.
    fn link(&self, name: &str, version: &str) -> &Self {
        symlink(
            Path::new("releases").join(version),
            self.dir.path().join(name),
        )
        .expect("symlink");
        self
    }

    fn link_target(&self, name: &str) -> Option<String> {
        std::fs::read_link(self.dir.path().join(name))
            .ok()
            .map(|t| t.to_string_lossy().into_owned())
    }

    fn breadcrumb(&self) -> Option<String> {
        std::fs::read_to_string(self.state().join("rescued")).ok()
    }

    /// Leave a breadcrumb from an earlier rescue, in the format the script writes.
    fn with_breadcrumb(&self, from: &str, to: &str) -> &Self {
        std::fs::write(
            self.state().join("rescued"),
            format!(
                "at=1786453421\ninstall_dir={}\nfrom={from}\nto={to}\nbecause=boot check\n",
                self.install().display()
            ),
        )
        .expect("breadcrumb");
        self
    }

    /// Run `robot-rescue`, with a stub `systemctl` on `PATH` recording whatever it is asked to do.
    fn rescue(&self, args: &[&str]) -> Rescue {
        self.run("scripts/robot-rescue", args, &[])
    }

    /// Run `robot-boot-check` against stubbed unit states.
    ///
    /// `units` is what the stub `systemctl show` answers: `(unit, ActiveState, NRestarts)`, each
    /// having entered that state at boot. Anything not listed answers empty, which is what a real
    /// `systemctl` does for a unit that is not loaded — a release that predates the unit, on a
    /// board that must not be rolled back for it.
    fn boot_check(&self, args: &[&str], units: &[(&str, &str, u32)]) -> Rescue {
        let at_boot: Vec<_> = units.iter().map(|&(u, s, r)| (u, s, r, 0)).collect();
        self.boot_check_since(args, &at_boot)
    }

    /// The same, with the second after boot each unit entered its state: `(unit, ActiveState,
    /// NRestarts, since)`. What tells a daemon that has run steadily from one that just came back.
    ///
    /// Every case must reach a verdict on its first look — the uptime is a file that does not
    /// move, so a board that is neither settled nor past its deadline would be watched forever.
    fn boot_check_since(&self, args: &[&str], units: &[(&str, &str, u32, u32)]) -> Rescue {
        self.run("scripts/robot-boot-check", args, units)
    }

    /// Write `[update_gate] boot_deadline_s` into one of robotd.toml's two layers.
    fn with_deadline(&self, layer: &str, value: &str) -> &Self {
        std::fs::write(
            self.dir.path().join(layer),
            format!("[update_gate]\n# a comment\nboot_deadline_s = {value}  # seconds\n"),
        )
        .expect("params layer");
        self
    }

    fn run(&self, script: &str, args: &[&str], units: &[(&str, &str, u32, u32)]) -> Rescue {
        let repo = Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .expect("xtask/ has a parent");

        let stub_dir = self.dir.path().join("stub");
        std::fs::create_dir_all(&stub_dir).expect("stub dir");
        let log = stub_dir.join("systemctl.log");

        // `systemctl show -p <Property> --value <unit>`, so the property is `$3` and the unit is
        // `$5`. Anything that is not a `show` is recorded instead, which is how a test asserts on
        // `reboot` — and, more often, on the absence of one.
        let mut stub = String::from("#!/bin/sh\nif [ \"$1\" = show ]; then\n  case \"$5\" in\n");
        for (unit, state, restarts, since) in units {
            let since_us = u64::from(*since) * 1_000_000;
            stub.push_str(&format!(
                "    {unit}) case \"$3\" in ActiveState) echo {state} ;; NRestarts) echo {restarts} ;; \
                 ActiveEnterTimestampMonotonic) echo {since_us} ;; LoadState) echo loaded ;; esac ;;\n"
            ));
        }
        // An empty `case` body is not portable, and a unit the table does not mention must answer
        // nothing — which is what a real `systemctl` does for one that is not loaded.
        stub.push_str("    *) ;;\n  esac\n  exit 0\nfi\n");
        stub.push_str(&format!("echo \"$@\" >> {}\n", log.display()));

        std::fs::write(stub_dir.join("systemctl"), stub).expect("stub");
        std::fs::set_permissions(
            stub_dir.join("systemctl"),
            <std::fs::Permissions as std::os::unix::fs::PermissionsExt>::from_mode(0o755),
        )
        .expect("stub mode");

        let output = Command::new("sh")
            .arg(repo.join(script))
            .args(args)
            .env("ROBOT_INSTALL_DIR", self.install())
            .env("ROBOT_STATE_DIR", self.state())
            .env("ROBOT_RESCUE", repo.join("scripts/robot-rescue"))
            // A boot check that read the host's real uptime would decline on any machine that has
            // been up for more than ten minutes, which is every CI runner and every laptop.
            .env("ROBOT_UPTIME_FILE", self.uptime_file())
            // Never the host's robotd.toml. Absent unless a case writes one (`with_deadline`).
            .env("ROBOT_PARAMS_BASE", self.dir.path().join("base.toml"))
            .env("ROBOT_PARAMS_USER", self.dir.path().join("user.toml"))
            .env(
                "PATH",
                format!(
                    "{}:{}",
                    stub_dir.display(),
                    std::env::var("PATH").unwrap_or_default()
                ),
            )
            .output()
            .unwrap_or_else(|e| panic!("run {script}: {e}"));

        Rescue {
            output,
            systemctl: std::fs::read_to_string(&log).unwrap_or_default(),
        }
    }

    /// What `updaterd` does on its next start: copy the breadcrumb into the update log and remove
    /// it, which is what releases the rescue's loop guard.
    fn updaterd_started(&self) -> &Self {
        let _ = std::fs::remove_file(self.state().join("rescued"));
        self
    }

    /// Pretend the board booted `secs` ago.
    fn booted_secs_ago(&self, secs: u32) -> &Self {
        std::fs::write(self.uptime_file(), format!("{secs}.42 1234.00\n")).expect("uptime");
        self
    }

    fn uptime_file(&self) -> PathBuf {
        self.dir.path().join("uptime")
    }
}

struct Rescue {
    output: Output,
    systemctl: String,
}

impl Rescue {
    fn code(&self) -> i32 {
        self.output.status.code().expect("exited")
    }

    fn stderr(&self) -> String {
        String::from_utf8_lossy(&self.output.stderr).into_owned()
    }
}

/// No golden means no rollback target, which is every board until 1.0.0 exists. Declining is the
/// answer, and it has to name both reasons: unset in the config, or set but never published.
#[test]
fn declines_when_no_golden_is_published() {
    let board = Board::with_releases(&["1.2.0"]);
    board.link("current", "1.2.0");

    let run = board.rescue(&[]);

    assert_eq!(run.code(), 2, "stderr: {}", run.stderr());
    assert!(run.stderr().contains("no golden release published"));
    assert!(
        run.stderr().contains("updater.toml"),
        "an operator needs to be told where golden is set: {}",
        run.stderr()
    );
    assert_eq!(
        board.link_target("current").as_deref(),
        Some("releases/1.2.0")
    );
}

/// A configured golden that was pruned or never installed. The link resolves to nothing, and
/// swapping onto it would leave `current` pointing at an empty path — a board that cannot exec
/// anything at all, which is worse than the failure being rescued.
#[test]
fn declines_when_golden_is_not_installed() {
    let board = Board::with_releases(&["1.2.0"]);
    board.link("current", "1.2.0").link("golden", "1.0.0");

    let run = board.rescue(&[]);

    assert_eq!(run.code(), 2, "stderr: {}", run.stderr());
    assert!(run.stderr().contains("not installed"), "{}", run.stderr());
    assert_eq!(
        board.link_target("current").as_deref(),
        Some("releases/1.2.0")
    );
}

/// The check that keeps a hardware fault from becoming a reboot loop: if the daemons are down on
/// the release carrying the standing guarantee, this is not a release fault and a swap changes
/// nothing except adding a reboot.
#[test]
fn declines_when_current_is_already_golden() {
    let board = Board::with_releases(&["1.0.0"]);
    board.link("current", "1.0.0").link("golden", "1.0.0");

    let run = board.rescue(&["--reboot"]);

    assert_eq!(run.code(), 2, "stderr: {}", run.stderr());
    assert!(run.stderr().contains("already golden"), "{}", run.stderr());
    assert!(
        run.systemctl.is_empty(),
        "declining must not reboot, even when asked to: {:?}",
        run.systemctl
    );
}

#[test]
fn swaps_current_to_golden_and_records_it() {
    let board = Board::with_releases(&["1.0.0", "1.2.0"]);
    board.link("current", "1.2.0").link("golden", "1.0.0");

    let run = board.rescue(&[]);

    assert_eq!(run.code(), 0, "stderr: {}", run.stderr());
    assert_eq!(
        board.link_target("current").as_deref(),
        Some("releases/1.0.0"),
        "current must point at golden, and at a *relative* target like the store writes"
    );

    let breadcrumb = board.breadcrumb().expect("a breadcrumb in the state dir");
    assert!(
        breadcrumb.contains("1.2.0") && breadcrumb.contains("1.0.0"),
        "the breadcrumb has to say what was swapped for what: {breadcrumb:?}"
    );
}

/// A board with no live release at all — a swap interrupted before it linked, or a `current`
/// deleted by hand. There is nothing to lose and golden is exactly where it should be put.
#[test]
fn acts_when_there_is_no_current_at_all() {
    let board = Board::with_releases(&["1.0.0"]);
    board.link("golden", "1.0.0");

    let run = board.rescue(&[]);

    assert_eq!(run.code(), 0, "stderr: {}", run.stderr());
    assert_eq!(
        board.link_target("current").as_deref(),
        Some("releases/1.0.0")
    );
}

/// `--reboot` is opt-in because the robot may be standing: every unit execs through `current`, so
/// the swap does nothing until they restart, and whoever is holding the robot decides when that is.
#[test]
fn does_not_reboot_unless_asked() {
    let board = Board::with_releases(&["1.0.0", "1.2.0"]);
    board.link("current", "1.2.0").link("golden", "1.0.0");

    let quiet = board.rescue(&[]);
    assert_eq!(quiet.code(), 0, "stderr: {}", quiet.stderr());
    assert!(
        quiet.systemctl.is_empty(),
        "swapped and rebooted without being asked: {:?}",
        quiet.systemctl
    );
    assert!(
        quiet.stderr().contains("systemctl reboot"),
        "having declined to reboot, it must say how: {}",
        quiet.stderr()
    );

    // Back to a release that is not golden, so the second run has something to do — and past the
    // loop guard, which is what a successful `updaterd` start does for real.
    std::fs::remove_file(board.install().join("current")).expect("unlink current");
    board.link("current", "1.2.0").updaterd_started();

    let loud = board.rescue(&["--reboot"]);
    assert_eq!(loud.code(), 0, "stderr: {}", loud.stderr());
    assert_eq!(loud.systemctl.trim(), "reboot");
}

/// `--dry-run` is what an operator reaches for first, and on a board that is merely suspect it must
/// not be the thing that changes the release.
#[test]
fn dry_run_decides_but_changes_nothing() {
    let board = Board::with_releases(&["1.0.0", "1.2.0"]);
    board.link("current", "1.2.0").link("golden", "1.0.0");

    let run = board.rescue(&["--dry-run"]);

    assert_eq!(run.code(), 0, "stderr: {}", run.stderr());
    assert!(run.stderr().contains("would swap"), "{}", run.stderr());
    assert_eq!(
        board.link_target("current").as_deref(),
        Some("releases/1.2.0")
    );
    assert!(board.breadcrumb().is_none(), "a dry run left a breadcrumb");
    assert!(run.systemctl.is_empty());
}

/// An unknown flag must not be ignored. A timer that grows a typo in its `ExecStart` should fail
/// loudly rather than silently rescue on the wrong terms.
#[test]
fn refuses_an_argument_it_does_not_understand() {
    let board = Board::with_releases(&["1.0.0", "1.2.0"]);
    board.link("current", "1.2.0").link("golden", "1.0.0");

    let run = board.rescue(&["--yolo"]);

    assert_eq!(run.code(), 1, "stderr: {}", run.stderr());
    assert_eq!(
        board.link_target("current").as_deref(),
        Some("releases/1.2.0")
    );
}

/// The loop guard. `robot-boot-check` invokes the rescue without a human, so a swap that did not fix
/// the board must not become a reboot loop. A breadcrumb is still on record when `updaterd` has not
/// started since — which means the release the last rescue chose is not starting either.
#[test]
fn declines_while_a_previous_rescue_is_still_on_record() {
    let board = Board::with_releases(&["1.0.0", "1.2.0"]);
    board
        .link("current", "1.2.0")
        .link("golden", "1.0.0")
        .with_breadcrumb("1.1.0", "1.0.0");

    let run = board.rescue(&["--reboot"]);

    assert_eq!(run.code(), 2, "stderr: {}", run.stderr());
    assert!(
        run.stderr().contains("updaterd has not started since"),
        "the reason has to name what is stuck: {}",
        run.stderr()
    );
    assert_eq!(
        board.link_target("current").as_deref(),
        Some("releases/1.2.0")
    );
    assert!(
        run.systemctl.is_empty(),
        "a guarded run must not reboot: {:?}",
        run.systemctl
    );
}

/// And the way past it by hand, for an operator who has read the journal and decided.
#[test]
fn force_gets_past_the_breadcrumb() {
    let board = Board::with_releases(&["1.0.0", "1.2.0"]);
    board
        .link("current", "1.2.0")
        .link("golden", "1.0.0")
        .with_breadcrumb("1.1.0", "1.0.0");

    let run = board.rescue(&["--force"]);

    assert_eq!(run.code(), 0, "stderr: {}", run.stderr());
    assert_eq!(
        board.link_target("current").as_deref(),
        Some("releases/1.0.0")
    );
}

/// The breadcrumb is read by `updaterd` (`journal::Breadcrumb`) as well as by a person, so its shape
/// is a contract. Overwritten rather than appended: one attempt at a time, with the history that
/// must survive going into the update log instead.
#[test]
fn the_breadcrumb_carries_what_the_update_log_will_need() {
    let board = Board::with_releases(&["1.0.0", "1.2.0"]);
    board.link("current", "1.2.0").link("golden", "1.0.0");

    board.rescue(&[
        "--because",
        "boot check: robotd.service (failed, 7 restarts)",
    ]);

    let crumb = board.breadcrumb().expect("a breadcrumb");
    let fields: Vec<&str> = crumb.lines().collect();
    assert!(fields.iter().any(|l| l.starts_with("at=")), "{crumb:?}");
    assert!(
        fields
            .iter()
            .any(|l| *l == format!("install_dir={}", board.install().display())),
        "updaterd matches the component by install_dir: {crumb:?}"
    );
    assert!(fields.contains(&"from=1.2.0"), "{crumb:?}");
    assert!(fields.contains(&"to=1.0.0"), "{crumb:?}");
    assert!(
        fields
            .iter()
            .any(|l| l.starts_with("because=boot check: robotd.service")),
        "{crumb:?}"
    );
}

/// The ordinary boot: every daemon up, nothing to do. This is the case that runs on every robot
/// every time, so it has to be silent about everything except its own verdict.
#[test]
fn the_boot_check_does_nothing_when_every_daemon_came_up() {
    let board = Board::with_releases(&["1.0.0", "1.2.0"]);
    board.link("current", "1.2.0").link("golden", "1.0.0");
    board.booted_secs_ago(180);

    let run = board.boot_check(
        &[],
        &[
            ("updaterd.service", "active", 0),
            ("robotd.service", "active", 0),
            ("configd.service", "active", 0),
            ("btd.service", "active", 1),
        ],
    );

    assert_eq!(run.code(), 0, "stderr: {}", run.stderr());
    assert!(run.stderr().contains("every daemon came up"));
    assert_eq!(
        board.link_target("current").as_deref(),
        Some("releases/1.2.0"),
        "a healthy boot must not move the release"
    );
    assert!(run.systemctl.is_empty(), "{:?}", run.systemctl);
}

#[test]
fn the_boot_check_rescues_a_failed_daemon() {
    let board = Board::with_releases(&["1.0.0", "1.2.0"]);
    board.link("current", "1.2.0").link("golden", "1.0.0");
    board.booted_secs_ago(180);

    let run = board.boot_check(
        &[],
        &[
            ("updaterd.service", "active", 0),
            ("robotd.service", "failed", 5),
            ("configd.service", "active", 0),
            ("btd.service", "active", 0),
        ],
    );

    assert_eq!(run.code(), 0, "stderr: {}", run.stderr());
    assert_eq!(
        board.link_target("current").as_deref(),
        Some("releases/1.0.0"),
        "stderr: {}",
        run.stderr()
    );
    assert_eq!(run.systemctl.trim(), "reboot");

    let crumb = board.breadcrumb().expect("a breadcrumb");
    assert!(
        crumb.contains("robotd.service"),
        "the reason has to reach the update log: {crumb:?}"
    );
}

/// A daemon that is `active` at this instant but has died five times is not a daemon that came up.
/// With `Restart=always` and a 2-5s `RestartSec`, a crash-looper is `active` a good fraction of the
/// time — a check that only looked at the state would call this a healthy boot.
#[test]
fn the_boot_check_rescues_a_daemon_that_keeps_restarting() {
    let board = Board::with_releases(&["1.0.0", "1.2.0"]);
    board.link("current", "1.2.0").link("golden", "1.0.0");
    board.booted_secs_ago(200);

    let run = board.boot_check(
        &[],
        &[
            ("updaterd.service", "active", 0),
            ("robotd.service", "active", 0),
            ("configd.service", "active", 0),
            ("btd.service", "active", 5),
        ],
    );

    assert_eq!(run.code(), 0, "stderr: {}", run.stderr());
    assert_eq!(
        board.link_target("current").as_deref(),
        Some("releases/1.0.0")
    );
}

/// Someone stopped it. That is `inactive` with no restarts, and it must not cost the board its
/// release — the same decision the startup reconciliation makes when it leaves a stopped unit
/// stopped.
#[test]
fn the_boot_check_leaves_a_stopped_daemon_alone() {
    let board = Board::with_releases(&["1.0.0", "1.2.0"]);
    board.link("current", "1.2.0").link("golden", "1.0.0");
    board.booted_secs_ago(180);

    let run = board.boot_check(
        &[],
        &[
            ("updaterd.service", "active", 0),
            ("robotd.service", "active", 0),
            ("configd.service", "inactive", 0),
            ("btd.service", "active", 0),
        ],
    );

    assert_eq!(run.code(), 0, "stderr: {}", run.stderr());
    assert_eq!(
        board.link_target("current").as_deref(),
        Some("releases/1.2.0")
    );
}

/// A release older than one of the units answers nothing at all for it. Reading that as a failure
/// would roll back every board running a release that predates a daemon.
#[test]
fn the_boot_check_ignores_a_unit_the_release_does_not_carry() {
    let board = Board::with_releases(&["1.0.0", "1.2.0"]);
    board.link("current", "1.2.0").link("golden", "1.0.0");
    board.booted_secs_ago(180);

    let run = board.boot_check(
        &[],
        &[
            ("updaterd.service", "active", 0),
            ("robotd.service", "active", 0),
        ],
    );

    assert_eq!(run.code(), 0, "stderr: {}", run.stderr());
    assert_eq!(
        board.link_target("current").as_deref(),
        Some("releases/1.2.0")
    );
}

/// It is a *boot* check. Invoked long after boot, something other than the timer started it — most
/// likely an installer running `enable --now` over the units it just wrote, mid-update, with daemons
/// legitimately restarting. Rolling back there would be the worst thing this could do.
#[test]
fn the_boot_check_declines_long_after_boot() {
    let board = Board::with_releases(&["1.0.0", "1.2.0"]);
    board.link("current", "1.2.0").link("golden", "1.0.0");
    board.booted_secs_ago(4 * 60 * 60);

    let run = board.boot_check(&[], &[("robotd.service", "failed", 9)]);

    assert_eq!(run.code(), 2, "stderr: {}", run.stderr());
    assert!(
        run.stderr().contains("not a boot check"),
        "{}",
        run.stderr()
    );
    assert_eq!(
        board.link_target("current").as_deref(),
        Some("releases/1.2.0")
    );
    assert!(board.breadcrumb().is_none());
}

#[test]
fn the_boot_check_dry_run_hands_over_nothing() {
    let board = Board::with_releases(&["1.0.0", "1.2.0"]);
    board.link("current", "1.2.0").link("golden", "1.0.0");
    board.booted_secs_ago(180);

    let run = board.boot_check(&["--dry-run"], &[("btd.service", "failed", 4)]);

    // Not 0: the OS's own boot gate asks this question with `--dry-run`, and reads the answer here.
    assert_eq!(run.code(), 3, "stderr: {}", run.stderr());
    assert!(run.stderr().contains("would hand over"), "{}", run.stderr());
    assert_eq!(
        board.link_target("current").as_deref(),
        Some("releases/1.2.0")
    );
    assert!(board.breadcrumb().is_none());
    assert!(run.systemctl.is_empty());
}

/// The ordinary boot, decided as soon as it can be: every daemon `active` for 10 s, here the last
/// one since 15 s in, so 25 s after boot — not at the deadline, and not three minutes in.
#[test]
fn the_boot_check_passes_once_every_daemon_has_settled() {
    let board = Board::with_releases(&["1.0.0", "1.2.0"]);
    board.link("current", "1.2.0").link("golden", "1.0.0");
    board.booted_secs_ago(25);

    let run = board.boot_check_since(
        &[],
        &[
            ("updaterd.service", "active", 0, 15),
            ("robotd.service", "active", 0, 8),
            ("configd.service", "active", 0, 9),
            ("btd.service", "active", 0, 8),
        ],
    );

    assert_eq!(run.code(), 0, "stderr: {}", run.stderr());
    assert!(
        run.stderr().contains("settled 25s after boot"),
        "{}",
        run.stderr()
    );
    assert!(run.systemctl.is_empty(), "{:?}", run.systemctl);
}

/// Counts only grow, so a member that has already restarted three times is decided on the spot —
/// 20 s in, with the deadline 40 s away.
#[test]
fn the_boot_check_rescues_before_the_deadline() {
    let board = Board::with_releases(&["1.0.0", "1.2.0"]);
    board.link("current", "1.2.0").link("golden", "1.0.0");
    board.booted_secs_ago(20);

    let run = board.boot_check_since(
        &[],
        &[
            ("updaterd.service", "active", 0, 15),
            ("robotd.service", "active", 3, 19),
            ("configd.service", "active", 0, 9),
            ("btd.service", "active", 0, 8),
        ],
    );

    assert_eq!(run.code(), 0, "stderr: {}", run.stderr());
    assert_eq!(
        board.link_target("current").as_deref(),
        Some("releases/1.0.0")
    );
    assert_eq!(run.systemctl.trim(), "reboot");
}

/// Back up for five seconds after two crashes is not settled, and not failed either. At the deadline
/// it is judged by the same rule as before the check watched: two restarts is a transient, left alone.
#[test]
fn the_boot_check_leaves_a_daemon_that_just_came_back_alone_at_the_deadline() {
    let board = Board::with_releases(&["1.0.0", "1.2.0"]);
    board.link("current", "1.2.0").link("golden", "1.0.0");
    board.booted_secs_ago(60);

    let run = board.boot_check_since(
        &[],
        &[
            ("updaterd.service", "active", 0, 15),
            ("robotd.service", "active", 0, 8),
            ("configd.service", "active", 0, 9),
            ("btd.service", "active", 2, 55),
        ],
    );

    assert_eq!(run.code(), 0, "stderr: {}", run.stderr());
    assert!(
        run.stderr()
            .contains("60s deadline passed with btd.service"),
        "{}",
        run.stderr()
    );
    assert_eq!(
        board.link_target("current").as_deref(),
        Some("releases/1.2.0")
    );
}

/// The owner's layer wins over the image's, key by key, as `Params::load` merges them. A deadline
/// read wrongly would leave this case watching forever rather than deciding at 30 s.
#[test]
fn the_boot_check_reads_its_deadline_from_the_owners_layer() {
    let board = Board::with_releases(&["1.0.0", "1.2.0"]);
    board.link("current", "1.2.0").link("golden", "1.0.0");
    board
        .with_deadline("base.toml", "90")
        .with_deadline("user.toml", "30");
    board.booted_secs_ago(30);

    let run = board.boot_check_since(&[], &[("btd.service", "active", 1, 28)]);

    assert_eq!(run.code(), 0, "stderr: {}", run.stderr());
    assert!(
        run.stderr().contains("30s deadline passed"),
        "{}",
        run.stderr()
    );
}

/// Clamped, not refused: a robotd that refused the file would not start, and a robotd that does not
/// start is what this script rolls a release back for.
#[test]
fn the_boot_check_clamps_a_deadline_out_of_range() {
    let board = Board::with_releases(&["1.0.0", "1.2.0"]);
    board.link("current", "1.2.0").link("golden", "1.0.0");
    board.with_deadline("user.toml", "5");
    board.booted_secs_ago(20);

    let run = board.boot_check_since(&[], &[("btd.service", "active", 1, 18)]);

    assert_eq!(run.code(), 0, "stderr: {}", run.stderr());
    assert!(run.stderr().contains("using 20"), "{}", run.stderr());
    assert!(
        run.stderr().contains("20s deadline passed"),
        "{}",
        run.stderr()
    );
}

/// A value the script cannot read is named and dropped, and the default stands.
#[test]
fn the_boot_check_ignores_a_deadline_it_cannot_read() {
    let board = Board::with_releases(&["1.0.0", "1.2.0"]);
    board.link("current", "1.2.0").link("golden", "1.0.0");
    board.with_deadline("user.toml", "\"soon\"");
    board.booted_secs_ago(60);

    let run = board.boot_check_since(&[], &[("btd.service", "active", 1, 58)]);

    assert_eq!(run.code(), 0, "stderr: {}", run.stderr());
    assert!(
        run.stderr().contains("not a whole number"),
        "{}",
        run.stderr()
    );
    assert!(
        run.stderr().contains("60s deadline passed"),
        "{}",
        run.stderr()
    );
}

/// The script cannot link `robotd-params`, so it repeats the key's default and bounds. This is what
/// keeps `robotctl configure` and the board agreeing on what the number means.
#[test]
fn the_boot_check_and_robotd_params_agree_on_the_deadline() {
    let script = std::fs::read_to_string(
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("..")
            .join("scripts/robot-boot-check"),
    )
    .expect("the script");
    let value = |name: &str| -> u32 {
        script
            .lines()
            .find_map(|line| line.strip_prefix(&format!("{name}=")))
            .unwrap_or_else(|| panic!("no {name}= in the script"))
            .parse()
            .unwrap_or_else(|e| panic!("{name}: {e}"))
    };

    assert_eq!(
        value("DEADLINE_DEFAULT"),
        robotd_params::UpdateGate::default().boot_deadline_s
    );
    assert_eq!(value("DEADLINE_MIN"), robotd_params::BOOT_DEADLINE_MIN_S);
    assert_eq!(value("DEADLINE_MAX"), robotd_params::BOOT_DEADLINE_MAX_S);
}
