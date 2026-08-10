//! Durable filesystem primitives.
//!
//! The design's crash guarantees rest on two renames being both **durable** and
//! **ordered**: the boot-counter record must survive a power cut that also made the
//! symlink swap visible (`docs/design/updater-design.md` §7). `rename(2)` is atomic but
//! not durable — the directory entry can still be in page cache — so every rename
//! we depend on is followed by an fsync of the containing directory.

use std::io::Write;
use std::path::Path;

use crate::Error;

/// fsync the directory containing `path`, making a rename into it durable.
///
/// Without this, a power cut can leave the swap visible and the pending record
/// gone — precisely the state §7 says cannot happen.
pub fn fsync_parent(path: &Path) -> Result<(), Error> {
    // `parent()` yields `Some("")` for a bare filename, which cannot be opened;
    // the containing directory in that case is the working directory.
    let dir = match path.parent() {
        None => return Ok(()),
        Some(p) if p.as_os_str().is_empty() => Path::new("."),
        Some(p) => p,
    };
    // Opening a directory read-only and fsyncing it is the portable way to flush
    // its entries on Linux and macOS.
    let handle = std::fs::File::open(dir).map_err(|e| Error::Io {
        path: dir.to_path_buf(),
        source: e,
    })?;
    handle.sync_all().map_err(|e| Error::Io {
        path: dir.to_path_buf(),
        source: e,
    })
}

/// Write via a temp file and `rename`, so readers never see a partial file, then
/// fsync both the file and its directory so the result survives a power cut.
pub fn write_atomic(path: &Path, bytes: &[u8]) -> Result<(), Error> {
    let tmp = path.with_extension("tmp");
    {
        let mut file = std::fs::File::create(&tmp).map_err(|e| Error::Io {
            path: tmp.clone(),
            source: e,
        })?;
        file.write_all(bytes).map_err(|e| Error::Io {
            path: tmp.clone(),
            source: e,
        })?;
        // Contents first: a durable rename to a file whose data is still in cache
        // would leave a correctly-named empty file.
        file.sync_all().map_err(|e| Error::Io {
            path: tmp.clone(),
            source: e,
        })?;
    }
    std::fs::rename(&tmp, path).map_err(|e| Error::Io {
        path: path.to_path_buf(),
        source: e,
    })?;
    fsync_parent(path)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn write_atomic_replaces_contents() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("f.json");

        write_atomic(&path, b"first").unwrap();
        assert_eq!(std::fs::read(&path).unwrap(), b"first");

        write_atomic(&path, b"second").unwrap();
        assert_eq!(std::fs::read(&path).unwrap(), b"second");

        // The temp file must not be left behind.
        assert!(!dir.path().join("f.tmp").exists());
    }

    #[test]
    fn fsync_parent_tolerates_a_bare_filename() {
        // `parent()` is `Some("")` for a bare name; must not error.
        assert!(fsync_parent(Path::new("only-a-name")).is_ok());
    }
}
