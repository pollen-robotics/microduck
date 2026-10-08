//! The map on disk.
//!
//! One JSON file, replaced atomically: written beside itself, flushed, renamed over the old one,
//! and the directory flushed, so a power cut leaves either the old map or the new one and never
//! half of either.
//!
//! **A file that cannot be read is set aside, never overwritten.** The first maploc started fresh
//! on any read error and its next autosave replaced the file — a format bump, or a branch build
//! with a different layout, silently destroyed hours of mapping. Here an unreadable map is renamed
//! to `<name>.bad-<seconds>` and said so in the journal, and the new map starts beside it; the old
//! one is still there for whoever wants to know what happened.

use std::io::Write;
use std::path::{Path, PathBuf};

use maploc::mapper::{SESSION_VERSION, Session};

/// What loading found.
#[derive(Debug)]
pub enum Loaded {
    /// A map, to resume.
    Map(Box<Session>),
    /// No file: a robot that has never mapped, or was wiped.
    Nothing,
    /// A file that could not be used, now moved to the path given.
    SetAside { reason: String, moved_to: PathBuf },
}

pub fn load(path: &Path) -> Loaded {
    let bytes = match std::fs::read(path) {
        Ok(b) => b,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Loaded::Nothing,
        Err(e) => return set_aside(path, format!("unreadable: {e}")),
    };
    // The version first, on its own: a map from a newer or older format must be named as such,
    // not as whichever field this build happens to find missing.
    #[derive(serde::Deserialize)]
    struct Version {
        version: u32,
    }
    match serde_json::from_slice::<Version>(&bytes) {
        Ok(v) if v.version != SESSION_VERSION => {
            return set_aside(
                path,
                format!(
                    "map format {} (this mapd reads {SESSION_VERSION})",
                    v.version
                ),
            );
        }
        Ok(_) => {}
        Err(e) => return set_aside(path, format!("not a map: {e}")),
    }
    match serde_json::from_slice::<Session>(&bytes) {
        Ok(s) => {
            if s.keyframes.len() != s.graph.nodes.len() || s.island.len() != s.keyframes.len() {
                return set_aside(
                    path,
                    "keyframes, nodes and islands disagree in number".into(),
                );
            }
            if s.graph
                .edges
                .iter()
                .any(|e| e.from >= s.graph.nodes.len() || e.to >= s.graph.nodes.len())
            {
                return set_aside(path, "an edge names a node that does not exist".into());
            }
            Loaded::Map(Box::new(s))
        }
        Err(e) => set_aside(path, format!("not a map: {e}")),
    }
}

fn set_aside(path: &Path, reason: String) -> Loaded {
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    let mut moved_to = path.as_os_str().to_owned();
    moved_to.push(format!(".bad-{secs}"));
    let moved_to = PathBuf::from(moved_to);
    if let Err(e) = std::fs::rename(path, &moved_to) {
        // Could not even move it: say so, and still do not overwrite it — the caller saves to
        // the same path, so refuse by reporting the original location.
        return Loaded::SetAside {
            reason: format!("{reason}; and moving it aside failed: {e}"),
            moved_to: path.to_owned(),
        };
    }
    Loaded::SetAside { reason, moved_to }
}

/// Replace the file with `session`, atomically.
pub fn save(path: &Path, session: &Session) -> std::io::Result<()> {
    let dir = path.parent().unwrap_or(Path::new("."));
    std::fs::create_dir_all(dir)?;
    let mut tmp = path.as_os_str().to_owned();
    tmp.push(".tmp");
    let tmp = PathBuf::from(tmp);
    {
        let mut f = std::fs::File::create(&tmp)?;
        serde_json::to_writer(&mut f, session).map_err(std::io::Error::other)?;
        f.flush()?;
        f.sync_all()?;
    }
    std::fs::rename(&tmp, path)?;
    std::fs::File::open(dir)?.sync_all()
}

/// Forget the map: the file goes, so a restart begins fresh too.
pub fn remove(path: &Path) -> std::io::Result<()> {
    match std::fs::remove_file(path) {
        Err(e) if e.kind() != std::io::ErrorKind::NotFound => Err(e),
        _ => Ok(()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use maploc::mapper::{Mapper, MapperConfig};

    #[test]
    fn a_map_survives_the_round_trip() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("map.json");
        let session = Mapper::new(MapperConfig::default()).session();
        save(&path, &session).unwrap();
        match load(&path) {
            Loaded::Map(s) => assert_eq!(*s, session),
            other => panic!("{other:?}"),
        }
    }

    /// The first maploc's worst habit: a map it could not read was overwritten by the next
    /// autosave. This one moves it aside and keeps it.
    #[test]
    fn an_unreadable_map_is_kept_aside_not_overwritten() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("map.json");
        std::fs::write(&path, br#"{"version": 99, "keyframes": []}"#).unwrap();
        let Loaded::SetAside { moved_to, reason } = load(&path) else {
            panic!("a future format was loaded");
        };
        assert!(reason.contains("format"), "{reason}");
        assert!(!path.exists());
        assert_eq!(
            std::fs::read(&moved_to).unwrap(),
            br#"{"version": 99, "keyframes": []}"#
        );
        // And a map saved now goes to the original path, beside the kept one.
        save(&path, &Mapper::new(MapperConfig::default()).session()).unwrap();
        assert!(moved_to.exists() && path.exists());
    }

    #[test]
    fn no_file_is_nothing_not_an_error() {
        let dir = tempfile::tempdir().unwrap();
        assert!(matches!(
            load(&dir.path().join("map.json")),
            Loaded::Nothing
        ));
    }
}
