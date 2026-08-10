//! Update log, single-flight lock, and boot counter — the engine's own persistent
//! state.
//!
//! All of it lives in `state_dir`, which must be **outside** every component's
//! `install_dir`: a symlink swap or rollback must not be able to destroy the
//! record of what happened (`docs/design/updater-design.md` §5.7). [`crate::config::Config::validate`]
//! enforces that.

use std::collections::BTreeMap;
use std::io::Write;
use std::path::{Path, PathBuf};

use crate::Error;
use crate::fsutil::write_atomic;
use crate::proto::{LogEntry, Outcome};

/// Unix seconds now. The engine stamps log entries; nothing else in the engine
/// depends on the clock except preflight's sanity check.
pub fn now_unix() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs() as i64)
        .unwrap_or(0)
}

/// Append-only record of update attempts.
///
/// The first thing support asks for when a client says "the update failed", so it
/// records refused and rolled-back attempts too — not just successes
/// (`docs/design/updater-design.md` §8.3).
pub struct Journal {
    path: PathBuf,
    max_entries: usize,
}

impl Journal {
    pub fn open(state_dir: &Path, max_entries: usize) -> Result<Self, Error> {
        std::fs::create_dir_all(state_dir).map_err(|e| Error::Io {
            path: state_dir.to_path_buf(),
            source: e,
        })?;
        Ok(Self {
            path: state_dir.join("update-log.jsonl"),
            max_entries,
        })
    }

    /// Append one entry.
    ///
    /// Durable enough to survive the power loss a failed update can itself
    /// provoke: newline-delimited JSON, then `sync_data`. A torn final line is
    /// tolerated on read rather than treated as corruption — losing the last entry
    /// is acceptable; refusing to read the log because of it is not.
    pub fn append(&self, entry: &LogEntry) -> Result<(), Error> {
        let mut line = serde_json::to_vec(entry)
            .map_err(|e| Error::Internal(format!("serialising log entry: {e}")))?;
        line.push(b'\n');

        {
            let mut file = std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(&self.path)
                .map_err(|e| Error::Io {
                    path: self.path.clone(),
                    source: e,
                })?;
            file.write_all(&line).map_err(|e| Error::Io {
                path: self.path.clone(),
                source: e,
            })?;
            file.sync_data().map_err(|e| Error::Io {
                path: self.path.clone(),
                source: e,
            })?;
        }

        self.trim()
    }

    /// Keep the log bounded. Runs after each append; cheap at these sizes.
    fn trim(&self) -> Result<(), Error> {
        let all = self.read_all()?;
        if all.len() <= self.max_entries {
            return Ok(());
        }

        let keep = &all[all.len() - self.max_entries..];
        let mut buf = Vec::new();
        for entry in keep {
            serde_json::to_writer(&mut buf, entry)
                .map_err(|e| Error::Internal(format!("serialising log entry: {e}")))?;
            buf.push(b'\n');
        }
        // Write-and-rename so a crash mid-trim can't leave a truncated log.
        write_atomic(&self.path, &buf)
    }

    /// Every entry, oldest first. Skips unparseable lines (a torn tail).
    fn read_all(&self) -> Result<Vec<LogEntry>, Error> {
        let text = match std::fs::read_to_string(&self.path) {
            Ok(t) => t,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(Vec::new()),
            Err(e) => {
                return Err(Error::Io {
                    path: self.path.clone(),
                    source: e,
                });
            }
        };
        Ok(text
            .lines()
            .filter(|l| !l.trim().is_empty())
            .filter_map(|l| serde_json::from_str(l).ok())
            .collect())
    }

    /// Most recent entries, newest first.
    pub fn recent(&self, limit: usize) -> Result<Vec<LogEntry>, Error> {
        let mut all = self.read_all()?;
        all.reverse();
        all.truncate(limit);
        Ok(all)
    }

    pub fn last_for(&self, component: &str) -> Result<Option<LogEntry>, Error> {
        Ok(self
            .read_all()?
            .into_iter()
            .rev()
            .find(|e| e.component.as_str() == component))
    }

    /// Versions whose **most recent** recorded outcome for this component was a
    /// rollback.
    ///
    /// Used to keep `rollback` from landing back on a release that already failed
    /// its gate. Latest-outcome rather than ever-failed, so a version that failed
    /// once and later succeeded is not blacklisted forever.
    pub fn known_bad(&self, component: &str) -> Result<Vec<semver::Version>, Error> {
        let mut seen: Vec<(semver::Version, bool)> = Vec::new();
        // Newest first: the first outcome seen for a version is its latest.
        for entry in self.read_all()?.into_iter().rev() {
            if entry.component.as_str() != component {
                continue;
            }
            let Some(version) = entry.to.clone() else {
                continue;
            };
            if seen.iter().any(|(v, _)| *v == version) {
                continue;
            }
            let bad = matches!(entry.outcome, Outcome::RolledBack { .. });
            seen.push((version, bad));
        }
        Ok(seen
            .into_iter()
            .filter_map(|(v, bad)| bad.then_some(v))
            .collect())
    }
}

/// Single-flight lock, using std's advisory file locking
/// (`File::try_lock`, stable since Rust 1.89 — no dependency needed).
///
/// An OS lock rather than a PID file so the kernel releases it if `updaterd` is
/// killed — a stale PID file would leave the robot permanently unable to update,
/// which is a worse failure than a race.
pub struct UpdateLock {
    /// Dropping the handle releases the lock.
    _file: std::fs::File,
}

impl UpdateLock {
    /// Acquire without blocking.
    ///
    /// `Ok(None)` when already held: "busy" is a normal answer to report to the
    /// app, not an error.
    pub fn try_acquire(state_dir: &Path) -> Result<Option<Self>, Error> {
        use std::fs::TryLockError;

        let path = state_dir.join("update.lock");
        let file = std::fs::OpenOptions::new()
            .create(true)
            .read(true)
            .write(true)
            .truncate(false)
            .open(&path)
            .map_err(|e| Error::Io {
                path: path.clone(),
                source: e,
            })?;

        match file.try_lock() {
            Ok(()) => Ok(Some(Self { _file: file })),
            // Held by another process: report "busy", don't fail.
            Err(TryLockError::WouldBlock) => Ok(None),
            Err(TryLockError::Error(source)) => Err(Error::Io { path, source }),
        }
    }
}

/// Runtime version pins, keyed by component.
///
/// Kept in `state_dir` rather than written back into `updater.toml`: a pin is
/// **device state**, so it belongs outside the shipped config (§5.7), and rewriting
/// a human-edited TOML file in place would lose comments and formatting. The config
/// may still carry a `pinned` value as a deployment-time default; this file
/// overrides it.
pub struct Pins {
    path: PathBuf,
}

impl Pins {
    pub fn open(state_dir: &Path) -> Self {
        Self {
            path: state_dir.join("pins.json"),
        }
    }

    fn read_all(&self) -> Result<BTreeMap<String, semver::Version>, Error> {
        match std::fs::read(&self.path) {
            Ok(bytes) => Ok(serde_json::from_slice(&bytes).unwrap_or_default()),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(BTreeMap::new()),
            Err(e) => Err(Error::Io {
                path: self.path.clone(),
                source: e,
            }),
        }
    }

    pub fn get(&self, component: &str) -> Result<Option<semver::Version>, Error> {
        Ok(self.read_all()?.remove(component))
    }

    /// Set or clear a pin. `None` unpins.
    pub fn set(&self, component: &str, version: Option<&semver::Version>) -> Result<(), Error> {
        let mut all = self.read_all()?;
        match version {
            Some(v) => {
                all.insert(component.to_owned(), v.clone());
            }
            None => {
                all.remove(component);
            }
        }
        if all.is_empty() {
            return match std::fs::remove_file(&self.path) {
                Ok(()) => Ok(()),
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(()),
                Err(e) => Err(Error::Io {
                    path: self.path.clone(),
                    source: e,
                }),
            };
        }
        let bytes = serde_json::to_vec(&all)
            .map_err(|e| Error::Internal(format!("serialising pins: {e}")))?;
        write_atomic(&self.path, &bytes)
    }
}

/// Counts boots since an update that has not yet proven itself healthy.
///
/// Catches the failure the in-process health gate cannot: a release that doesn't
/// come up at all, or wedges hard enough to take `updaterd` with it. If a pending
/// update hasn't been confirmed after `max_attempts` boots, revert unconditionally
/// (`docs/design/updater-design.md` §8).
pub struct BootCounter {
    path: PathBuf,
}

#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct PendingUpdate {
    pub component: String,
    pub version: semver::Version,
    pub previous: Option<semver::Version>,
    #[serde(default)]
    pub boots: u32,
}

impl BootCounter {
    pub fn open(state_dir: &Path) -> Self {
        Self {
            path: state_dir.join("pending.json"),
        }
    }

    /// Record that `version` is on trial for `pending.component`.
    ///
    /// Written **before** the swap, so a crash between swap and health check is
    /// still recoverable — the opposite order would leave an unrecorded bad release
    /// live.
    ///
    /// Trials are **keyed by component**: a model selection must not overwrite or
    /// clear a daemon update's trial, or a daemon release that crashed after its
    /// swap would silently lose the record the never-brick guarantee rests on.
    pub fn arm(&self, pending: &PendingUpdate) -> Result<(), Error> {
        let mut all = self.read_all()?;
        all.insert(pending.component.clone(), pending.clone());
        self.write_all(&all)
    }

    fn read_all(&self) -> Result<BTreeMap<String, PendingUpdate>, Error> {
        match std::fs::read(&self.path) {
            Ok(bytes) => Ok(serde_json::from_slice(&bytes).unwrap_or_default()),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(BTreeMap::new()),
            Err(e) => Err(Error::Io {
                path: self.path.clone(),
                source: e,
            }),
        }
    }

    fn write_all(&self, all: &BTreeMap<String, PendingUpdate>) -> Result<(), Error> {
        if all.is_empty() {
            // No trials outstanding: remove the file rather than leave `{}` behind,
            // so "is anything pending?" is answerable by existence.
            return match std::fs::remove_file(&self.path) {
                Ok(()) => Ok(()),
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(()),
                Err(e) => Err(Error::Io {
                    path: self.path.clone(),
                    source: e,
                }),
            };
        }
        let bytes = serde_json::to_vec(all)
            .map_err(|e| Error::Internal(format!("serialising pending updates: {e}")))?;
        write_atomic(&self.path, &bytes)
    }

    pub fn pending_for(&self, component: &str) -> Result<Option<PendingUpdate>, Error> {
        Ok(self.read_all()?.remove(component))
    }

    /// Every outstanding trial.
    pub fn all(&self) -> Result<Vec<PendingUpdate>, Error> {
        Ok(self.read_all()?.into_values().collect())
    }

    /// Clear one component's trial after a passed health gate.
    pub fn confirm(&self, component: &str) -> Result<(), Error> {
        let mut all = self.read_all()?;
        all.remove(component);
        self.write_all(&all)
    }

    /// Increment every outstanding trial and return them. Called once per boot,
    /// before serving.
    pub fn record_boot(&self) -> Result<Vec<PendingUpdate>, Error> {
        let mut all = self.read_all()?;
        if all.is_empty() {
            return Ok(Vec::new());
        }
        for pending in all.values_mut() {
            pending.boots = pending.boots.saturating_add(1);
        }
        self.write_all(&all)?;
        Ok(all.into_values().collect())
    }

    /// Should we give up and revert?
    pub fn exhausted(pending: &PendingUpdate, max_attempts: u32) -> bool {
        pending.boots >= max_attempts
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proto::{ComponentId, Outcome};

    fn entry(component: &str, to: &str, outcome: Outcome) -> LogEntry {
        LogEntry {
            at: 1_700_000_000,
            component: ComponentId::new(component),
            from: None,
            to: Some(semver::Version::parse(to).unwrap()),
            outcome,
        }
    }

    #[test]
    fn appends_and_reads_newest_first() {
        let dir = tempfile::tempdir().unwrap();
        let journal = Journal::open(dir.path(), 100).unwrap();

        journal
            .append(&entry("daemon", "1.0.0", Outcome::Success))
            .unwrap();
        journal
            .append(&entry(
                "daemon",
                "1.1.0",
                Outcome::RolledBack {
                    reason: "health".into(),
                },
            ))
            .unwrap();

        let recent = journal.recent(10).unwrap();
        assert_eq!(recent.len(), 2);
        assert_eq!(recent[0].to, Some(semver::Version::new(1, 1, 0)));
    }

    /// Failures and refusals must be recorded too — those are exactly the entries
    /// support needs.
    #[test]
    fn records_failures_not_just_successes() {
        let dir = tempfile::tempdir().unwrap();
        let journal = Journal::open(dir.path(), 100).unwrap();
        journal
            .append(&entry(
                "daemon",
                "1.2.0",
                Outcome::Aborted {
                    reason: "disk full".into(),
                },
            ))
            .unwrap();

        let last = journal.last_for("daemon").unwrap().unwrap();
        assert!(matches!(last.outcome, Outcome::Aborted { .. }));
    }

    #[test]
    fn reading_an_absent_log_is_empty_not_an_error() {
        let dir = tempfile::tempdir().unwrap();
        let journal = Journal::open(dir.path(), 10).unwrap();
        assert!(journal.recent(5).unwrap().is_empty());
        assert!(journal.last_for("daemon").unwrap().is_none());
    }

    #[test]
    fn trims_to_max_entries_keeping_newest() {
        let dir = tempfile::tempdir().unwrap();
        let journal = Journal::open(dir.path(), 3).unwrap();

        for minor in 0..6 {
            journal
                .append(&entry("daemon", &format!("1.{minor}.0"), Outcome::Success))
                .unwrap();
        }

        let recent = journal.recent(100).unwrap();
        assert_eq!(recent.len(), 3);
        assert_eq!(recent[0].to, Some(semver::Version::new(1, 5, 0)));
        assert_eq!(recent[2].to, Some(semver::Version::new(1, 3, 0)));
    }

    /// A half-written final line (power lost mid-append) must cost us that entry,
    /// not the whole log.
    #[test]
    fn tolerates_torn_final_line() {
        let dir = tempfile::tempdir().unwrap();
        let journal = Journal::open(dir.path(), 100).unwrap();
        journal
            .append(&entry("daemon", "1.0.0", Outcome::Success))
            .unwrap();

        let mut file = std::fs::OpenOptions::new()
            .append(true)
            .open(dir.path().join("update-log.jsonl"))
            .unwrap();
        file.write_all(br#"{"at":1,"component":"daemon","#).unwrap();

        let recent = journal.recent(10).unwrap();
        assert_eq!(recent.len(), 1, "intact entry must still be readable");
    }

    #[test]
    fn last_for_filters_by_component() {
        let dir = tempfile::tempdir().unwrap();
        let journal = Journal::open(dir.path(), 100).unwrap();
        journal
            .append(&entry("daemon", "1.0.0", Outcome::Success))
            .unwrap();
        journal
            .append(&entry("model", "3.0.0", Outcome::Success))
            .unwrap();

        assert_eq!(
            journal.last_for("daemon").unwrap().unwrap().to,
            Some(semver::Version::new(1, 0, 0))
        );
        assert_eq!(
            journal.last_for("model").unwrap().unwrap().to,
            Some(semver::Version::new(3, 0, 0))
        );
        assert!(journal.last_for("nope").unwrap().is_none());
    }

    /// Re-acquiring within the same process must report busy, not deadlock and not
    /// succeed twice.
    #[test]
    fn lock_is_exclusive_then_released_on_drop() {
        let dir = tempfile::tempdir().unwrap();

        let first = UpdateLock::try_acquire(dir.path()).unwrap();
        assert!(first.is_some(), "first acquire should succeed");

        assert!(
            UpdateLock::try_acquire(dir.path()).unwrap().is_none(),
            "second acquire should report busy"
        );

        drop(first);

        // Closing the descriptor releases the lock, so this must succeed immediately.
        // Distinguishing None from Err matters: "still held" and "could not open the file"
        // have different causes and the bare assertion could not tell them apart.
        let reacquired = match UpdateLock::try_acquire(dir.path()) {
            Ok(Some(lock)) => Some(lock),
            Ok(None) => panic!("lock still reported busy after the holder was dropped"),
            Err(e) => panic!("re-acquire failed: {e}"),
        };
        assert!(
            reacquired.is_some(),
            "lock should be free once the handle is dropped (dir={})",
            dir.path().display()
        );
    }

    fn pending(boots: u32) -> PendingUpdate {
        PendingUpdate {
            component: "daemon".into(),
            version: semver::Version::new(1, 1, 0),
            previous: Some(semver::Version::new(1, 0, 0)),
            boots,
        }
    }

    #[test]
    fn boot_counter_arms_increments_and_confirms() {
        let dir = tempfile::tempdir().unwrap();
        let counter = BootCounter::open(dir.path());

        assert!(counter.pending_for("daemon").unwrap().is_none());

        counter.arm(&pending(0)).unwrap();
        assert_eq!(counter.record_boot().unwrap()[0].boots, 1);
        assert_eq!(counter.record_boot().unwrap()[0].boots, 2);

        counter.confirm("daemon").unwrap();
        assert!(
            counter.pending_for("daemon").unwrap().is_none(),
            "confirmed update must leave no pending trial"
        );
        // record_boot on a clean state must not invent a pending update.
        assert!(counter.record_boot().unwrap().is_empty());
    }

    /// The bug this guards: with a single global slot, a model transition would
    /// overwrite or clear a daemon update's trial — losing exactly the record the
    /// never-brick guarantee depends on.
    #[test]
    fn trials_are_independent_per_component() {
        let dir = tempfile::tempdir().unwrap();
        let counter = BootCounter::open(dir.path());

        counter.arm(&pending(0)).unwrap();
        counter
            .arm(&PendingUpdate {
                component: "model".into(),
                version: semver::Version::new(3, 0, 0),
                previous: None,
                boots: 0,
            })
            .unwrap();

        // Confirming the model must not touch the daemon's trial.
        counter.confirm("model").unwrap();

        let daemon = counter.pending_for("daemon").unwrap();
        assert!(daemon.is_some(), "daemon trial must survive");
        assert_eq!(daemon.unwrap().version, semver::Version::new(1, 1, 0));
        assert!(counter.pending_for("model").unwrap().is_none());
    }

    #[test]
    fn record_boot_increments_every_component() {
        let dir = tempfile::tempdir().unwrap();
        let counter = BootCounter::open(dir.path());
        counter.arm(&pending(0)).unwrap();
        counter
            .arm(&PendingUpdate {
                component: "model".into(),
                version: semver::Version::new(3, 0, 0),
                previous: None,
                boots: 0,
            })
            .unwrap();

        let after = counter.record_boot().unwrap();
        assert_eq!(after.len(), 2);
        assert!(after.iter().all(|p| p.boots == 1));
    }

    /// Clearing the last trial removes the file, so "anything pending?" stays an
    /// existence check rather than parsing `{}`.
    #[test]
    fn clearing_the_last_trial_removes_the_file() {
        let dir = tempfile::tempdir().unwrap();
        let counter = BootCounter::open(dir.path());
        counter.arm(&pending(0)).unwrap();
        assert!(dir.path().join("pending.json").exists());

        counter.confirm("daemon").unwrap();
        assert!(!dir.path().join("pending.json").exists());
    }

    #[test]
    fn known_bad_tracks_latest_outcome_per_version() {
        let dir = tempfile::tempdir().unwrap();
        let journal = Journal::open(dir.path(), 100).unwrap();

        // 1.1.0 failed, then a later attempt succeeded: not blacklisted.
        journal
            .append(&entry(
                "daemon",
                "1.1.0",
                Outcome::RolledBack {
                    reason: "health".into(),
                },
            ))
            .unwrap();
        journal
            .append(&entry("daemon", "1.1.0", Outcome::Success))
            .unwrap();
        // 1.2.0 failed and never succeeded: blacklisted.
        journal
            .append(&entry(
                "daemon",
                "1.2.0",
                Outcome::RolledBack {
                    reason: "hook".into(),
                },
            ))
            .unwrap();
        // Another component's failure must not leak across.
        journal
            .append(&entry(
                "model",
                "9.0.0",
                Outcome::RolledBack {
                    reason: "health".into(),
                },
            ))
            .unwrap();

        let bad = journal.known_bad("daemon").unwrap();
        assert_eq!(bad, vec![semver::Version::new(1, 2, 0)], "{bad:?}");
    }

    #[test]
    fn exhaustion_is_inclusive() {
        assert!(BootCounter::exhausted(&pending(2), 2));
        assert!(!BootCounter::exhausted(&pending(1), 2));
    }

    /// Confirming when nothing is pending is a no-op, so recovery paths can call
    /// it unconditionally.
    #[test]
    fn confirm_is_idempotent() {
        let dir = tempfile::tempdir().unwrap();
        let counter = BootCounter::open(dir.path());
        counter.confirm("daemon").unwrap();
        counter.confirm("daemon").unwrap();
    }
}
