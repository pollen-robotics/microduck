//! On-disk release store: versioned directories plus a symlink consumers read
//! through.
//!
//! ```text
//! /opt/robot/daemon/
//! ├── releases/1.4.1/     ← previous, kept for rollback
//! ├── releases/1.4.2/     ← new
//! └── current → releases/1.4.2
//! ```
//!
//! Atomicity comes from a single `rename(2)` over the symlink, so no partially
//! written release is ever live. See `docs/design/updater-design.md` §7.1.
//!
//! Nothing in here may touch robot-specific state — calibration, learned state
//! and user config live outside `install_dir` precisely so that a swap or a
//! rollback cannot destroy them (§5.7).

use std::fs;
use std::path::{Path, PathBuf};

use crate::Error;

/// Marks an in-progress install. Chosen so it can never parse as a semver
/// version, which is what keeps [`Store::list`] from ever seeing a staging dir as
/// a real release.
const STAGING_PREFIX: &str = ".staging-";

/// Subdirectory holding installed releases.
const RELEASES_DIR: &str = "releases";

/// Symlink consumers read through.
const CURRENT_LINK: &str = "current";

pub struct Store {
    root: PathBuf,
}

impl Store {
    pub fn new(root: PathBuf) -> Self {
        Self { root }
    }

    pub fn releases_dir(&self) -> PathBuf {
        self.root.join(RELEASES_DIR)
    }

    /// Path of the symlink consumers read through.
    pub fn link_path(&self) -> PathBuf {
        self.root.join(CURRENT_LINK)
    }

    pub fn release_dir(&self, version: &semver::Version) -> PathBuf {
        self.releases_dir().join(version.to_string())
    }

    /// Staging directory for an in-progress install.
    ///
    /// A sibling of the final path so the later `rename` stays on one
    /// filesystem, and clearly marked so a crashed run leaves obvious garbage
    /// rather than something mistakable for a real release.
    pub fn staging_dir(&self, version: &semver::Version) -> PathBuf {
        self.releases_dir()
            .join(format!("{STAGING_PREFIX}{version}"))
    }

    /// Version the symlink currently points at.
    ///
    /// `Ok(None)` when the link is absent (a fresh robot) or dangling — both are
    /// recoverable states, not errors.
    pub fn current(&self) -> Result<Option<semver::Version>, Error> {
        let link = self.link_path();
        let target = match fs::read_link(&link) {
            Ok(t) => t,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
            Err(e) => {
                return Err(Error::Io {
                    path: link,
                    source: e,
                });
            }
        };
        let name = target
            .file_name()
            .and_then(|n| n.to_str())
            .ok_or_else(|| Error::Corrupt(format!("unreadable symlink target: {target:?}")))?;
        Ok(semver::Version::parse(name).ok())
    }

    /// Installed releases, newest first. Ignores staging dirs and any entry whose
    /// name isn't a version.
    pub fn list(&self) -> Result<Vec<semver::Version>, Error> {
        let dir = self.releases_dir();
        let entries = match fs::read_dir(&dir) {
            Ok(e) => e,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(Vec::new()),
            Err(e) => {
                return Err(Error::Io {
                    path: dir,
                    source: e,
                });
            }
        };

        let mut versions: Vec<_> = entries
            .filter_map(Result::ok)
            .filter_map(|e| {
                e.file_name()
                    .to_str()
                    .and_then(|n| semver::Version::parse(n).ok())
            })
            .collect();
        versions.sort_by(|a, b| b.cmp(a));
        Ok(versions)
    }

    /// Point the symlink at `version` atomically.
    ///
    /// Writes a temporary symlink beside the real one and `rename`s it over the
    /// top: `rename(2)` on the same directory is atomic, so a concurrent reader
    /// sees either the old target or the new one, never a missing link. Removing
    /// and recreating the symlink would open exactly that window.
    pub fn swap_to(&self, version: &semver::Version) -> Result<(), Error> {
        let release = self.release_dir(version);
        if !release.is_dir() {
            return Err(Error::Corrupt(format!(
                "refusing to link missing release: {}",
                release.display()
            )));
        }

        // Store a relative target so the tree stays valid if the mount point
        // moves (and so it reads sensibly in a shell).
        let target = Path::new(RELEASES_DIR).join(version.to_string());

        let link = self.link_path();
        let tmp = self.root.join(format!(".{CURRENT_LINK}.tmp"));

        // A leftover tmp link from a crashed run must not block the swap.
        let _ = fs::remove_file(&tmp);

        std::os::unix::fs::symlink(&target, &tmp).map_err(|e| Error::Io {
            path: tmp.clone(),
            source: e,
        })?;

        fs::rename(&tmp, &link).map_err(|e| {
            let _ = fs::remove_file(&tmp);
            Error::Io {
                path: link.clone(),
                source: e,
            }
        })?;

        // Durability, not just atomicity. The boot counter is armed before this
        // swap; if the swap reached disk and the pending record did not, a power cut
        // would leave a bad release live with no trial to revert it — the one state
        // §7 says cannot happen.
        crate::fsutil::fsync_parent(&link)
    }

    /// Delete old releases, keeping the active one, `keep_previous` most recent
    /// others, and `golden` unconditionally.
    ///
    /// Golden is exempt from the count so the never-brick guarantee doesn't
    /// silently expire as versions accumulate (`docs/design/updater-design.md` §8.2).
    pub fn prune(
        &self,
        keep_previous: usize,
        golden: Option<&semver::Version>,
    ) -> Result<Vec<semver::Version>, Error> {
        let active = self.current()?;
        let installed = self.list()?;

        // Candidates are everything that isn't the live release or golden. Golden
        // is exempt from the count, not merely prioritised by it, so the
        // never-brick guarantee can't silently expire as versions accumulate.
        let mut candidates: Vec<_> = installed
            .into_iter()
            .filter(|v| Some(v) != active.as_ref() && Some(v) != golden)
            .collect();

        // Newest first, so `keep_previous` retains the most recent — those are the
        // plausible rollback targets.
        candidates.sort_by(|a, b| b.cmp(a));

        let doomed: Vec<_> = candidates.into_iter().skip(keep_previous).collect();

        let mut removed = Vec::new();
        for version in doomed {
            let dir = self.release_dir(&version);
            std::fs::remove_dir_all(&dir).map_err(|e| Error::Io {
                path: dir,
                source: e,
            })?;
            removed.push(version);
        }
        Ok(removed)
    }

    /// Remove staging leftovers from an interrupted run.
    ///
    /// Called at startup: a `kill -9` mid-extract must not leak disk forever. Only
    /// touches the `.staging-` prefix, so a real release can never be caught by it.
    pub fn clean_staging(&self) -> Result<usize, Error> {
        let dir = self.releases_dir();
        let entries = match std::fs::read_dir(&dir) {
            Ok(e) => e,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(0),
            Err(e) => {
                return Err(Error::Io {
                    path: dir,
                    source: e,
                });
            }
        };

        let mut removed = 0;
        for entry in entries.filter_map(Result::ok) {
            let name = entry.file_name();
            let Some(name) = name.to_str() else { continue };
            if !name.starts_with(STAGING_PREFIX) {
                continue;
            }
            let path = entry.path();
            std::fs::remove_dir_all(&path).map_err(|e| Error::Io {
                path: path.clone(),
                source: e,
            })?;
            removed += 1;
        }
        Ok(removed)
    }

    /// Free bytes on the filesystem holding the store, for the preflight space
    /// check.
    pub fn available_space(&self) -> Result<u64, Error> {
        self.available_space_at(&self.releases_dir())
    }

    /// Free bytes on the filesystem holding `dir`.
    ///
    /// Takes an explicit path so a caller can measure the nearest *existing* ancestor:
    /// on a fresh robot `releases/` does not exist yet, and measuring a missing path
    /// fails.
    pub fn available_space_at(&self, dir: &Path) -> Result<u64, Error> {
        fs4::available_space(dir).map_err(|e| Error::Io {
            path: dir.to_path_buf(),
            source: e,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v(s: &str) -> semver::Version {
        semver::Version::parse(s).unwrap()
    }

    /// A store under a fresh temp dir. The `TempDir` is returned so the caller
    /// keeps it alive — dropping it removes the tree.
    fn scratch() -> (tempfile::TempDir, Store) {
        let dir = tempfile::tempdir().unwrap();
        fs::create_dir_all(dir.path().join("releases")).unwrap();
        let store = Store::new(dir.path().to_path_buf());
        (dir, store)
    }

    #[test]
    fn current_is_none_when_no_link() {
        let (_dir, store) = scratch();
        assert_eq!(store.current().unwrap(), None);
    }

    #[test]
    fn swap_is_atomic_and_repeatable() {
        let (_dir, store) = scratch();
        fs::create_dir_all(store.release_dir(&v("1.0.0"))).unwrap();
        fs::create_dir_all(store.release_dir(&v("1.1.0"))).unwrap();

        store.swap_to(&v("1.0.0")).unwrap();
        assert_eq!(store.current().unwrap(), Some(v("1.0.0")));

        // Swapping over an existing link must work — this is the upgrade path.
        store.swap_to(&v("1.1.0")).unwrap();
        assert_eq!(store.current().unwrap(), Some(v("1.1.0")));

        // And back, which is the rollback path.
        store.swap_to(&v("1.0.0")).unwrap();
        assert_eq!(store.current().unwrap(), Some(v("1.0.0")));
    }

    #[test]
    fn swap_refuses_missing_release() {
        let (_dir, store) = scratch();
        assert!(store.swap_to(&v("9.9.9")).is_err());
        // Failure must leave no link behind rather than a dangling one.
        assert_eq!(store.current().unwrap(), None);
    }

    #[test]
    fn list_is_newest_first_and_ignores_staging() {
        let (_dir, store) = scratch();
        for s in ["1.0.0", "1.2.0", "1.1.0"] {
            fs::create_dir_all(store.release_dir(&v(s))).unwrap();
        }
        fs::create_dir_all(store.staging_dir(&v("2.0.0"))).unwrap();

        assert_eq!(
            store.list().unwrap(),
            vec![v("1.2.0"), v("1.1.0"), v("1.0.0")]
        );
    }

    /// Lexical ordering would put 1.9.0 above 1.10.0 and pick the wrong rollback
    /// target.
    #[test]
    fn list_orders_by_semver_not_string() {
        let (_dir, store) = scratch();
        for s in ["1.9.0", "1.10.0"] {
            fs::create_dir_all(store.release_dir(&v(s))).unwrap();
        }
        assert_eq!(store.list().unwrap(), vec![v("1.10.0"), v("1.9.0")]);
    }

    #[test]
    fn prune_keeps_active_and_n_previous() {
        let (_dir, store) = scratch();
        for s in ["1.0.0", "1.1.0", "1.2.0", "1.3.0"] {
            fs::create_dir_all(store.release_dir(&v(s))).unwrap();
        }
        store.swap_to(&v("1.3.0")).unwrap();

        let removed = store.prune(1, None).unwrap();

        assert_eq!(removed, vec![v("1.1.0"), v("1.0.0")]);
        assert_eq!(store.list().unwrap(), vec![v("1.3.0"), v("1.2.0")]);
        // The live release must survive regardless of counts.
        assert_eq!(store.current().unwrap(), Some(v("1.3.0")));
    }

    /// Golden is exempt from the count, not merely prioritised by it — otherwise
    /// the never-brick guarantee expires silently as versions pile up.
    #[test]
    fn prune_never_removes_golden() {
        let (_dir, store) = scratch();
        for s in ["1.0.0", "1.1.0", "1.2.0", "1.3.0"] {
            fs::create_dir_all(store.release_dir(&v(s))).unwrap();
        }
        store.swap_to(&v("1.3.0")).unwrap();

        let golden = v("1.0.0");
        let removed = store.prune(1, Some(&golden)).unwrap();

        assert!(!removed.contains(&golden));
        assert!(store.release_dir(&golden).is_dir(), "golden must survive");
        assert_eq!(removed, vec![v("1.1.0")]);
    }

    #[test]
    fn prune_is_idempotent() {
        let (_dir, store) = scratch();
        for s in ["1.0.0", "1.1.0", "1.2.0"] {
            fs::create_dir_all(store.release_dir(&v(s))).unwrap();
        }
        store.swap_to(&v("1.2.0")).unwrap();

        store.prune(1, None).unwrap();
        let second = store.prune(1, None).unwrap();
        assert!(second.is_empty(), "second prune should find nothing");
    }

    #[test]
    fn clean_staging_removes_only_staging_dirs() {
        let (_dir, store) = scratch();
        fs::create_dir_all(store.release_dir(&v("1.0.0"))).unwrap();
        fs::create_dir_all(store.staging_dir(&v("1.1.0"))).unwrap();
        fs::create_dir_all(store.staging_dir(&v("1.2.0"))).unwrap();

        assert_eq!(store.clean_staging().unwrap(), 2);
        assert_eq!(store.list().unwrap(), vec![v("1.0.0")]);
        assert!(!store.staging_dir(&v("1.1.0")).exists());
    }

    #[test]
    fn clean_staging_tolerates_missing_dir() {
        let dir = tempfile::tempdir().unwrap();
        let store = Store::new(dir.path().to_path_buf());
        assert_eq!(store.clean_staging().unwrap(), 0);
    }
}
