//! Pre/post-install hooks.
//!
//! Hooks ship *inside* the signed artifact, so no unsigned code ever runs. Same
//! idea as dpkg's `postinst`; see `docs/design/updater-design.md` §9.
//!
//! Ordering:
//! ```text
//! extract → [pre_install] → symlink swap → [post_install] → apply → health gate
//! ```
//!
//! A non-zero exit is a failed update and triggers rollback, identically to a
//! failed health probe. Hooks are part of the gate, not fire-and-forget.

use std::path::{Path, PathBuf};
use std::time::Duration;

use crate::Error;

/// Hook filenames looked for inside a release, relative to its root.
pub const PRE_INSTALL: &str = "hooks/preinstall";
pub const POST_INSTALL: &str = "hooks/postinstall";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HookKind {
    PreInstall,
    PostInstall,
}

impl HookKind {
    pub fn relative_path(self) -> &'static str {
        match self {
            HookKind::PreInstall => PRE_INSTALL,
            HookKind::PostInstall => POST_INSTALL,
        }
    }
}

/// Context passed to hooks as environment variables, so one hook can branch on
/// what kind of transition it's in (notably a `schema_version` bump needing a
/// config migration).
#[derive(Debug, Clone)]
pub struct HookContext {
    /// Component name, e.g. `daemon`. Exported as both `UPDATE_COMPONENT` and
    /// `UPDATE_CHANNEL`: they are the same thing here (a component's channel is its
    /// name), and §9 documents the latter.
    pub component: String,
    pub old_version: Option<semver::Version>,
    pub new_version: semver::Version,
    /// Component root, e.g. `/opt/robot/daemon`.
    pub install_dir: PathBuf,
    /// The release being installed, e.g. `/opt/robot/daemon/releases/1.4.2`. This is
    /// what a migration usually wants.
    pub release_dir: PathBuf,
    pub old_schema_version: Option<u32>,
    pub new_schema_version: u32,
}

impl HookContext {
    /// `UPDATE_*` environment pairs. Absent values are omitted rather than set
    /// empty, so a hook can distinguish "first install" from "unknown".
    pub fn env(&self) -> Vec<(String, String)> {
        let mut env = vec![
            ("UPDATE_COMPONENT".into(), self.component.clone()),
            // Documented in §9 as `UPDATE_CHANNEL`; same value.
            ("UPDATE_CHANNEL".into(), self.component.clone()),
            ("UPDATE_NEW_VERSION".into(), self.new_version.to_string()),
            // The component's *root* (e.g. /opt/robot/daemon), not the release
            // directory. A hook's own location is its cwd, and `$PWD` is the release
            // being installed — see §9.
            (
                "UPDATE_INSTALL_DIR".into(),
                self.install_dir.display().to_string(),
            ),
            (
                "UPDATE_RELEASE_DIR".into(),
                self.release_dir.display().to_string(),
            ),
            (
                "UPDATE_NEW_SCHEMA_VERSION".into(),
                self.new_schema_version.to_string(),
            ),
        ];
        if let Some(old) = &self.old_version {
            env.push(("UPDATE_OLD_VERSION".into(), old.to_string()));
        }
        if let Some(old) = self.old_schema_version {
            env.push(("UPDATE_OLD_SCHEMA_VERSION".into(), old.to_string()));
        }
        env
    }
}

#[derive(Debug, Clone)]
pub struct HookOutcome {
    pub ran: bool,
    pub exit_code: Option<i32>,
    /// Captured output, truncated. Logged by [`run`] whatever the outcome, and
    /// returned so a caller can put it in a reply too.
    pub output: String,
}

/// Captured output is truncated to this, so a chatty or runaway hook can't blow up
/// the update log.
const MAX_OUTPUT: usize = 8 * 1024;

/// Run a hook if present.
///
/// A missing hook is success (`ran: false`) — most releases won't have one. A
/// present-but-failing hook is an error, and so is a timeout: a hook that hangs
/// must not wedge the updater forever, and an unfinished migration is not a
/// successful one.
pub async fn run(
    release_dir: &Path,
    kind: HookKind,
    ctx: &HookContext,
    timeout: Duration,
) -> Result<HookOutcome, Error> {
    let hook = release_dir.join(kind.relative_path());
    if !hook.exists() {
        return Ok(HookOutcome {
            ran: false,
            exit_code: Some(0),
            output: String::new(),
        });
    }

    let hook_name = kind.relative_path().to_owned();

    let mut command = tokio::process::Command::new(&hook);
    command
        .current_dir(release_dir)
        .env_clear()
        // A minimal environment: hooks get what we pass plus PATH, so their
        // behaviour doesn't depend on however systemd happened to invoke us.
        .env(
            "PATH",
            "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
        )
        .kill_on_drop(true)
        .stdin(std::process::Stdio::null())
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped());

    for (key, value) in ctx.env() {
        command.env(key, value);
    }

    let child = crate::spawn::retrying_busy(&mut command)
        .await
        .map_err(|e| Error::Hook {
            hook: hook_name.clone(),
            detail: format!("could not execute {}: {e}", hook.display()),
        })?;

    // `kill_on_drop` plus this timeout is what guarantees a hanging hook is
    // reaped rather than left behind holding the update open.
    let finished = tokio::time::timeout(timeout, child.wait_with_output()).await;

    let output = match finished {
        Ok(Ok(output)) => output,
        Ok(Err(e)) => {
            return Err(Error::Hook {
                hook: hook_name,
                detail: format!("failed while running: {e}"),
            });
        }
        Err(_elapsed) => {
            return Err(Error::Hook {
                hook: hook_name,
                detail: format!("timed out after {}s", timeout.as_secs()),
            });
        }
    };

    let mut text = String::from_utf8_lossy(&output.stdout).into_owned();
    text.push_str(&String::from_utf8_lossy(&output.stderr));
    if text.len() > MAX_OUTPUT {
        // Cut on a char boundary: `from_utf8_lossy` can place a multi-byte character (often a
        // 3-byte U+FFFD for invalid input) straddling the cap, and a bare `truncate` panics on
        // that — inside `Engine::apply`, where it kills the unattended-update task for good.
        let mut end = MAX_OUTPUT;
        while !text.is_char_boundary(end) {
            end -= 1;
        }
        text.truncate(end);
    }

    if !output.status.success() {
        return Err(Error::Hook {
            hook: hook_name,
            detail: format!(
                "exited with {}: {}",
                output
                    .status
                    .code()
                    .map(|c| c.to_string())
                    .unwrap_or_else(|| "signal".into()),
                text.trim()
            ),
        });
    }

    // **Logged on success, not only on failure.** Both call sites take this outcome with `?` and
    // drop it, so for a hook that worked the output went nowhere — and this is the hook that
    // installs ONNX Runtime and the GStreamer stack, whose whole report is the answer to "can this
    // board encode H.264 in hardware, at this release". A grep of `journalctl -u updaterd` for it
    // came back empty on a board where the hook had plainly run, which is how this was found.
    //
    // Line by line rather than one entry: a multi-line message is one journal record, and what a
    // reader does with this is grep it.
    for line in text.lines() {
        if !line.trim().is_empty() {
            tracing::info!(hook = %hook_name, "{}", line.trim_end());
        }
    }

    Ok(HookOutcome {
        ran: true,
        exit_code: output.status.code(),
        output: text,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ctx() -> HookContext {
        HookContext {
            component: "daemon".into(),
            old_version: Some(semver::Version::new(1, 0, 0)),
            new_version: semver::Version::new(1, 1, 0),
            install_dir: PathBuf::from("/opt/robot/daemon"),
            release_dir: PathBuf::from("/opt/robot/daemon/releases/1.1.0"),
            old_schema_version: Some(1),
            new_schema_version: 2,
        }
    }

    #[test]
    fn env_omits_absent_old_version() {
        let ctx = HookContext {
            old_version: None,
            old_schema_version: None,
            ..ctx()
        };
        let env = ctx.env();
        assert!(!env.iter().any(|(k, _)| k == "UPDATE_OLD_VERSION"));
        assert!(!env.iter().any(|(k, _)| k == "UPDATE_OLD_SCHEMA_VERSION"));
        assert!(
            env.iter()
                .any(|(k, v)| k == "UPDATE_NEW_VERSION" && v == "1.1.0")
        );
    }

    /// Write an executable hook script into a fake release dir.
    fn write_hook(release: &Path, kind: HookKind, body: &str) {
        use std::os::unix::fs::PermissionsExt;

        let path = release.join(kind.relative_path());
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(&path, body).unwrap();
        std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o755)).unwrap();
    }

    /// Most releases have no hook; that must be success, not a failure.
    #[tokio::test]
    async fn missing_hook_is_success() {
        let dir = tempfile::tempdir().unwrap();
        let outcome = run(
            dir.path(),
            HookKind::PostInstall,
            &ctx(),
            Duration::from_secs(5),
        )
        .await
        .unwrap();
        assert!(!outcome.ran);
    }

    /// Collects whatever a `tracing` subscriber is handed, so a test can assert what reached the
    /// journal rather than what a function returned.
    #[derive(Clone, Default)]
    struct Collected(std::sync::Arc<std::sync::Mutex<Vec<u8>>>);

    /// The collector, installed once as the **global** default.
    ///
    /// A thread-local default (`set_default`) is not enough and the difference is not academic: it
    /// passed run on its own and collected nothing under `cargo test`, because these events are
    /// emitted wherever the runtime happens to poll the future rather than on the thread that
    /// installed the guard. Global, once, and the assertions below are `contains` — so another
    /// test's events landing in the same buffer are harmless.
    fn log_collector() -> Collected {
        static COLLECTED: std::sync::OnceLock<Collected> = std::sync::OnceLock::new();
        COLLECTED
            .get_or_init(|| {
                let collected = Collected::default();
                let subscriber = tracing_subscriber::fmt()
                    .with_writer(collected.clone())
                    .with_max_level(tracing::Level::INFO)
                    .finish();
                // Best effort: if something else in this binary got there first, the assertions
                // below will say so rather than this failing obscurely here.
                let _ = tracing::subscriber::set_global_default(subscriber);
                collected
            })
            .clone()
    }

    impl std::io::Write for Collected {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            self.0.lock().unwrap().extend_from_slice(buf);
            Ok(buf.len())
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    impl<'a> tracing_subscriber::fmt::MakeWriter<'a> for Collected {
        type Writer = Self;
        fn make_writer(&'a self) -> Self::Writer {
            self.clone()
        }
    }

    /// A hook that *worked* must still say what it said.
    ///
    /// Both call sites take the outcome with `?` and drop it, so for a successful hook the returned
    /// `output` reaches nobody — and the pre-install hook's report is the answer to "can this board
    /// encode H.264 in hardware, at this release". A grep of `journalctl -u updaterd` for it came
    /// back empty on a board where the hook had plainly run, which is what this test now holds.
    #[tokio::test]
    async fn a_successful_hook_reaches_the_log() {
        let dir = tempfile::tempdir().unwrap();
        write_hook(
            dir.path(),
            HookKind::PreInstall,
            "#!/bin/sh\necho 'plugins already at v3'\necho 'mpph264enc present' >&2\n",
        );

        let collected = log_collector();

        run(
            dir.path(),
            HookKind::PreInstall,
            &ctx(),
            Duration::from_secs(5),
        )
        .await
        .unwrap();

        let logged = String::from_utf8(collected.0.lock().unwrap().clone()).unwrap();
        assert!(
            logged.contains("plugins already at v3"),
            "stdout did not reach the log: {logged}"
        );
        assert!(
            logged.contains("mpph264enc present"),
            "stderr did not reach the log: {logged}"
        );
        assert!(
            logged.contains("preinstall"),
            "the log does not name which hook said it: {logged}"
        );
    }

    #[tokio::test]
    async fn successful_hook_captures_output() {
        let dir = tempfile::tempdir().unwrap();
        write_hook(
            dir.path(),
            HookKind::PostInstall,
            "#!/bin/sh\necho migrated\nexit 0\n",
        );

        let outcome = run(
            dir.path(),
            HookKind::PostInstall,
            &ctx(),
            Duration::from_secs(5),
        )
        .await
        .unwrap();

        assert!(outcome.ran);
        assert_eq!(outcome.exit_code, Some(0));
        assert!(outcome.output.contains("migrated"));
    }

    /// Hook output is capped at [`MAX_OUTPUT`], and the cap used to be a bare
    /// `String::truncate` — which panics when the cut lands mid-character, easily reachable
    /// because `from_utf8_lossy` pads invalid bytes into 3-byte U+FFFD runs. The panic
    /// unwound through `Engine::apply`, killing the unattended-update task with no journal
    /// entry. The cut must retreat to a char boundary instead.
    #[tokio::test]
    async fn oversized_output_is_cut_on_a_char_boundary() {
        let dir = tempfile::tempdir().unwrap();
        // 8191 ASCII bytes, then a two-byte `é` straddling byte 8192, then a failure so the
        // output travels inside the error — the path the panic used to abort.
        write_hook(
            dir.path(),
            HookKind::PostInstall,
            "#!/bin/sh\nhead -c 8191 /dev/zero | tr '\\0' 'a'\nprintf 'é'\nexit 1\n",
        );

        let err = run(
            dir.path(),
            HookKind::PostInstall,
            &ctx(),
            Duration::from_secs(5),
        )
        .await
        .expect_err("the hook failed; its output must come back as an error, not a panic");

        let Error::Hook { detail, .. } = err else {
            panic!("expected Error::Hook, got {err:?}");
        };
        let output = detail
            .strip_prefix("exited with 1: ")
            .expect("the exit status is reported with the output");
        assert!(
            output.trim().chars().all(|c| c == 'a'),
            "the é straddling the cap must be cut away whole, not split: {output:?}"
        );
    }

    /// A non-zero exit is a failed update — the caller turns this into a rollback.
    #[tokio::test]
    async fn failing_hook_is_an_error_with_output() {
        let dir = tempfile::tempdir().unwrap();
        write_hook(
            dir.path(),
            HookKind::PostInstall,
            "#!/bin/sh\necho 'schema migration failed' >&2\nexit 3\n",
        );

        let err = run(
            dir.path(),
            HookKind::PostInstall,
            &ctx(),
            Duration::from_secs(5),
        )
        .await
        .unwrap_err();

        match err {
            Error::Hook { hook, detail } => {
                assert_eq!(hook, POST_INSTALL);
                // The reason must reach the log, or a support ticket is unanswerable.
                assert!(detail.contains("schema migration failed"), "got: {detail}");
                assert!(
                    detail.contains('3'),
                    "exit code should be reported: {detail}"
                );
            }
            other => panic!("expected Hook error, got {other:?}"),
        }
    }

    /// The `ETXTBSY` retry, driven deterministically rather than hoped for.
    ///
    /// Linux refuses to exec a file that any process has open for writing, tracked on the
    /// inode, and it counts our own descriptors — so holding one reproduces exactly what an
    /// unrelated forked child does for a few microseconds during an update. Releasing it
    /// inside the retry budget must produce a hook that ran, not a failed update.
    ///
    /// Linux-only, and deliberately not made portable: macOS permits the exec, so on a Mac
    /// this would pass without ever provoking the condition — the most expensive kind of
    /// green test. Its partner below is what keeps it honest: if `ETXTBSY` ever stopped
    /// being produced, that one fails, so the two cannot both pass vacuously.
    #[cfg(target_os = "linux")]
    #[tokio::test]
    async fn a_hook_busy_for_exec_is_retried_rather_than_failed() {
        let dir = tempfile::tempdir().unwrap();
        write_hook(dir.path(), HookKind::PostInstall, "#!/bin/sh\nexit 0\n");
        let path = dir.path().join(HookKind::PostInstall.relative_path());

        let holder = std::fs::OpenOptions::new().write(true).open(&path).unwrap();
        // Well inside the ~100 ms budget, and long enough that the first attempts do fail.
        let releaser = tokio::spawn(async move {
            tokio::time::sleep(Duration::from_millis(25)).await;
            drop(holder);
        });

        let outcome = run(
            dir.path(),
            HookKind::PostInstall,
            &ctx(),
            Duration::from_secs(5),
        )
        .await
        .expect("a transiently busy hook must be retried, not reported as a failure");

        assert!(outcome.ran);
        assert_eq!(outcome.exit_code, Some(0));
        releaser.await.unwrap();
    }

    /// And the retry is bounded. A file that stays busy must eventually be an error rather
    /// than hanging the update — the budget exists to give up, not to wait forever.
    ///
    /// Also the control for the test above: this failing means the platform is not producing
    /// `ETXTBSY`, and therefore that the retry test proves nothing.
    #[cfg(target_os = "linux")]
    #[tokio::test]
    async fn a_hook_busy_forever_still_fails() {
        let dir = tempfile::tempdir().unwrap();
        write_hook(dir.path(), HookKind::PostInstall, "#!/bin/sh\nexit 0\n");
        let path = dir.path().join(HookKind::PostInstall.relative_path());

        // Held for the whole call.
        let _holder = std::fs::OpenOptions::new().write(true).open(&path).unwrap();

        let err = run(
            dir.path(),
            HookKind::PostInstall,
            &ctx(),
            Duration::from_secs(5),
        )
        .await
        .unwrap_err();

        match err {
            Error::Hook { hook, detail } => {
                assert_eq!(hook, POST_INSTALL);
                assert!(detail.contains("could not execute"), "got: {detail}");
            }
            other => panic!("expected Hook error, got {other:?}"),
        }
    }

    /// A hook that hangs must be killed, not left holding the update open forever.
    #[tokio::test]
    async fn hanging_hook_times_out() {
        let dir = tempfile::tempdir().unwrap();
        write_hook(dir.path(), HookKind::PreInstall, "#!/bin/sh\nsleep 30\n");

        let err = run(
            dir.path(),
            HookKind::PreInstall,
            &ctx(),
            Duration::from_millis(200),
        )
        .await
        .unwrap_err();

        assert!(
            matches!(&err, Error::Hook { detail, .. } if detail.contains("timed out")),
            "got {err:?}"
        );
    }

    /// The context env is what lets one hook handle several upgrade paths, so it
    /// must actually arrive in the process.
    #[tokio::test]
    async fn hook_receives_version_context() {
        let dir = tempfile::tempdir().unwrap();
        write_hook(
            dir.path(),
            HookKind::PostInstall,
            r#"#!/bin/sh
[ "$UPDATE_OLD_VERSION" = "1.0.0" ] || exit 10
[ "$UPDATE_NEW_VERSION" = "1.1.0" ] || exit 11
[ "$UPDATE_COMPONENT" = "daemon" ] || exit 12
[ "$UPDATE_OLD_SCHEMA_VERSION" = "1" ] || exit 13
[ "$UPDATE_NEW_SCHEMA_VERSION" = "2" ] || exit 14
[ "$UPDATE_CHANNEL" = "daemon" ] || exit 15
[ -n "$UPDATE_RELEASE_DIR" ] || exit 16
exit 0
"#,
        );

        run(
            dir.path(),
            HookKind::PostInstall,
            &ctx(),
            Duration::from_secs(5),
        )
        .await
        .expect("hook should see full context");
    }

    /// A hook that isn't executable is a packaging mistake; it must fail loudly
    /// rather than be silently skipped.
    #[tokio::test]
    async fn non_executable_hook_is_an_error() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join(POST_INSTALL);
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(&path, "#!/bin/sh\nexit 0\n").unwrap();

        let err = run(
            dir.path(),
            HookKind::PostInstall,
            &ctx(),
            Duration::from_secs(5),
        )
        .await
        .unwrap_err();
        assert!(matches!(err, Error::Hook { .. }), "got {err:?}");
    }
}
