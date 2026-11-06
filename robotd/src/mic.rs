//! The microphone, lent out: the petting worker's capture, on a socket, to whoever connects.
//!
//! **One capture, because the codec gives one.** The TLV320AIC3104's capture PCM opens once, so a
//! second process asking ALSA for the mic gets `EBUSY` while the petting worker holds it. The way
//! round that inside ALSA is `dsnoop`, and it does not survive these two units:
//! `robotd-design.md` §4.6 has why. So the mic stays `robotd`'s and this hands the bytes on — which
//! also means a listener hears exactly what the classifier hears.
//!
//! The wire is [`proto::mic_socket_path`]'s: raw S16LE mono at [`proto::MIC_RATE`], no header.
//! It is written from the worker's thread, so nothing here may block it:
//!
//! - **A listener that falls behind is dropped**, not waited for. The writes are non-blocking, and
//!   a short write would leave the next sample split across two writes — every sample after it a
//!   byte out of step, which is noise rather than speech. The kernel buffers a few seconds of this
//!   stream, so a reader dropped here had stopped reading, and reconnecting costs it nothing.
//! - **Accepting is a thread of its own**, parked in `accept`. The control loop never sees any of
//!   this; nor does the worker, except as one call per read.

use std::io::Write;
use std::os::unix::fs::PermissionsExt;
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::Path;
use std::sync::{Arc, Condvar, Mutex};
use std::time::Duration;

use duck_ipc_proto as proto;

/// Owner and the `robot` group: the socket takes this process's primary group, which the unit
/// sets to `robot` — the same audience as `robot.state`, and the group `mediad` runs in.
const SOCKET_MODE: u32 = 0o660;

/// The listeners, and the worker's way of hearing that the first one arrived.
pub struct Tap {
    listeners: Mutex<Vec<UnixStream>>,
    arrived: Condvar,
}

impl Tap {
    /// Bind `path` and start accepting. `Err` is a socket nobody can connect to, which costs the
    /// listeners and nothing else — the caller says so and carries on without one.
    pub fn serve(path: &Path) -> std::io::Result<Arc<Self>> {
        use std::os::unix::fs::FileTypeExt;
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        // `robotd` is one process per robot — it owns the servo bus — so a socket already here is
        // a previous one's, left by a crash. Only a socket is removed: anything else is not ours.
        if std::fs::symlink_metadata(path).is_ok_and(|m| m.file_type().is_socket()) {
            let _ = std::fs::remove_file(path);
        }
        let listener = UnixListener::bind(path)?;
        std::fs::set_permissions(path, std::fs::Permissions::from_mode(SOCKET_MODE))?;

        let tap = Arc::new(Self {
            listeners: Mutex::new(Vec::new()),
            arrived: Condvar::new(),
        });
        let accepting = Arc::clone(&tap);
        std::thread::Builder::new()
            .name("mic-tap".into())
            .spawn(move || accepting.accept(listener))?;
        tracing::info!(path = %path.display(), "lending the mic to whoever connects");
        Ok(tap)
    }

    fn accept(&self, listener: UnixListener) {
        for stream in listener.incoming() {
            let stream = match stream {
                Ok(stream) => stream,
                Err(error) => {
                    tracing::warn!(%error, "mic tap: accept failed");
                    // A persistent failure (out of descriptors) would otherwise spin a core.
                    std::thread::sleep(Duration::from_millis(500));
                    continue;
                }
            };
            if let Err(error) = stream.set_nonblocking(true) {
                tracing::warn!(%error, "mic tap: a listener that would block the worker; refused");
                continue;
            }
            let mut listeners = self.lock();
            listeners.push(stream);
            tracing::info!(listeners = listeners.len(), "mic tap: a listener connected");
            self.arrived.notify_all();
        }
    }

    // A panic while holding this lock leaves a list of sockets, which is as good as it was.
    fn lock(&self) -> std::sync::MutexGuard<'_, Vec<UnixStream>> {
        self.listeners
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }
}

impl pet_detect::worker::Tap for Tap {
    fn wanted(&self) -> bool {
        !self.lock().is_empty()
    }

    fn wait_wanted(&self, timeout: Duration) -> bool {
        let listeners = self.lock();
        if !listeners.is_empty() {
            return true;
        }
        let (listeners, _) = self
            .arrived
            .wait_timeout(listeners, timeout)
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        !listeners.is_empty()
    }

    fn send(&self, pcm: &[u8]) {
        let mut listeners = self.lock();
        let before = listeners.len();
        // Whole or not at all — see the module header for why a short write is a disconnect.
        listeners.retain_mut(|stream| matches!(stream.write(pcm), Ok(n) if n == pcm.len()));
        if listeners.len() < before {
            tracing::info!(
                listeners = listeners.len(),
                "mic tap: a listener left or fell behind"
            );
        }
    }
}

/// The tap at its shipped path, or `None` with the reason in the journal.
pub fn serve_default() -> Option<Arc<Tap>> {
    let path = proto::mic_socket_path();
    Tap::serve(&path)
        .inspect_err(|error| {
            tracing::warn!(
                %error,
                path = %path.display(),
                "the mic will not be lent out: WebRTC peers hear silence, petting is unaffected"
            );
        })
        .ok()
}

#[cfg(test)]
mod tests {
    use super::*;
    use pet_detect::worker::Tap as _;
    use std::io::Read;
    use std::time::Instant;

    fn served() -> (tempfile::TempDir, std::path::PathBuf, Arc<Tap>) {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("robotd").join("mic.sock");
        let tap = Tap::serve(&path).expect("binds");
        (dir, path, tap)
    }

    fn until(what: impl Fn() -> bool) -> bool {
        let deadline = Instant::now() + Duration::from_secs(5);
        while Instant::now() < deadline {
            if what() {
                return true;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        false
    }

    /// Nobody connected is nobody wanting, which is what keeps the mic closed on a robot with
    /// petting off. A regression is a duck recording its room for nobody.
    #[test]
    fn nobody_connected_is_nobody_wanting_and_a_listener_changes_that() {
        let (_dir, path, tap) = served();
        assert!(!tap.wanted());
        assert!(!tap.wait_wanted(Duration::from_millis(10)));

        let _listener = UnixStream::connect(&path).unwrap();
        assert!(
            tap.wait_wanted(Duration::from_secs(5)),
            "the worker parked in wait_wanted must wake for a listener"
        );
    }

    /// A listener hears the bytes as they were sent, in order.
    #[test]
    fn a_listener_hears_what_was_sent() {
        let (_dir, path, tap) = served();
        let mut listener = UnixStream::connect(&path).unwrap();
        assert!(until(|| tap.wanted()));

        tap.send(&[1, 2, 3, 4]);
        tap.send(&[5, 6]);
        let mut heard = [0u8; 6];
        listener.read_exact(&mut heard).unwrap();
        assert_eq!(heard, [1, 2, 3, 4, 5, 6]);
    }

    /// A listener that hung up is forgotten on the next send, so the capture it was holding open
    /// can close.
    #[test]
    fn a_listener_that_left_stops_being_wanted() {
        let (_dir, path, tap) = served();
        let listener = UnixStream::connect(&path).unwrap();
        assert!(until(|| tap.wanted()));
        drop(listener);

        // The first write after a hang-up can still land in the buffer; the one after fails.
        assert!(until(|| {
            tap.send(&[0; 320]);
            !tap.wanted()
        }));
    }

    /// **A listener that stops reading is dropped rather than waited for.** The writes happen on
    /// the petting worker's thread; a blocking write there would stall the classifier behind a
    /// stuck reader, and a short one would put every sample after it a byte out of step.
    #[test]
    fn a_listener_that_stops_reading_is_dropped_and_never_blocks_the_sender() {
        let (_dir, path, tap) = served();
        let _stuck = UnixStream::connect(&path).unwrap();
        assert!(until(|| tap.wanted()));

        let started = Instant::now();
        let chunk = [0u8; 4096];
        while tap.wanted() {
            tap.send(&chunk);
            assert!(
                started.elapsed() < Duration::from_secs(5),
                "a reader that never reads must be dropped"
            );
        }
    }

    /// A socket left by a crashed robotd is replaced, not an error: `robotd` restarts on its own.
    #[test]
    fn a_stale_socket_is_replaced() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("mic.sock");
        drop(UnixListener::bind(&path).unwrap());
        assert!(path.exists(), "the stale socket file stays behind");
        let tap = Tap::serve(&path).expect("binds over the stale socket");
        let _listener = UnixStream::connect(&path).unwrap();
        assert!(tap.wait_wanted(Duration::from_secs(5)));
    }
}
