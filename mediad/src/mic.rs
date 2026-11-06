//! The microphone track's source: `robotd`'s mic tap, paced into 20 ms chunks, silence when there
//! is nothing to hear.
//!
//! **`mediad` does not open the mic.** The codec's capture PCM opens once and `robotd`'s petting
//! worker is its reader, so this connects to the tap that worker feeds
//! ([`duck_ipc_proto::mic_socket_path`]) — `robotd-design.md` §4.6 has why that, and not ALSA's
//! `dsnoop`. What arrives is raw S16LE mono at [`duck_ipc_proto::MIC_RATE`].
//!
//! **The track never stops, whatever the tap does.** `webrtcsink` will not negotiate a session
//! until every one of its input streams has caps, so an audio branch that waits for a microphone
//! is a robot whose *video and control channel* wait for one too. So [`feed`] runs on its own
//! clock from the start, and every 20 ms hands on either the next chunk of the mic or a chunk of
//! zeros: `robotd` restarting, audio being off, a twin with no codec — each is a peer hearing
//! silence, never a session that will not start.
//!
//! **And it asks for the mic only while somebody is listening.** `wanted` is the count of WebRTC
//! peers; with none, the tap is closed, and `robotd` — unless petting detection keeps the capture
//! open for itself — stops recording. A duck nobody is watching records nothing.
//!
//! What is portable here is all of it but the push: [`pipeline`](crate::pipeline) turns chunks
//! into timestamped buffers on an `appsrc`.

use std::collections::VecDeque;
use std::io::Read;
use std::os::unix::net::UnixStream;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use duck_ipc_proto as proto;

/// One push: 20 ms, the frame size Opus is happiest at and the one a browser's jitter buffer
/// expects. Short enough that pacing it costs nothing to latency.
pub const CHUNK: Duration = Duration::from_millis(20);

/// Bytes in a [`CHUNK`]: 320 samples of S16 mono at 16 kHz.
pub const CHUNK_BYTES: usize = (proto::MIC_RATE as usize / 50) * 2;

/// Bytes per millisecond of the tap's stream.
const BYTES_PER_MS: usize = proto::MIC_RATE as usize * 2 / 1000;

/// How much audio has to be waiting before any is played, after a connect or a gap.
///
/// **The tap is bursty, and this is sized to its bursts.** `robotd` reads `arecord` a period at a
/// time — 125 ms with `arecord`'s defaults — and passes each read on whole, so audio arrives in
/// lumps eight chunks long. Playing from the first byte would drain a lump and click into silence
/// before the next; waiting for a lump and a half rides through them. It is also most of this
/// track's latency, which for a voice someone is listening to is the right trade.
const PRIME_BYTES: usize = 200 * BYTES_PER_MS;

/// The most that may wait. Past it the oldest audio is dropped back to [`PRIME_BYTES`]: a peer
/// hearing the room late by seconds is worse than one that lost a moment of it.
const CAP_BYTES: usize = 500 * BYTES_PER_MS;

/// How far behind its own clock [`feed`] may fall before it gives up catching up and starts again
/// from now — telling the push, so the timestamps jump with it rather than pretend.
const RESYNC_AFTER: Duration = Duration::from_millis(200);

/// How often a wanted tap that is not there is asked for again.
const RECONNECT_EVERY: Duration = Duration::from_secs(1);

/// The mic's bytes between the tap's bursts and the steady 20 ms the track wants.
#[derive(Debug, Default)]
pub struct Jitter {
    queued: VecDeque<u8>,
    primed: bool,
}

impl Jitter {
    /// Bytes as the socket gave them. A stream socket may split a sample across two reads; that
    /// is harmless here, because the queue keeps byte order and only ever hands out, or drops,
    /// whole samples from the front.
    pub fn feed(&mut self, bytes: &[u8]) {
        self.queued.extend(bytes);
        if self.queued.len() > CAP_BYTES {
            // Even, so what is left still starts on a sample.
            let excess = (self.queued.len() - PRIME_BYTES) & !1;
            self.queued.drain(..excess);
        }
    }

    /// The next chunk: the mic's, or silence while it is gathering or has nothing.
    pub fn take(&mut self) -> [u8; CHUNK_BYTES] {
        let mut chunk = [0u8; CHUNK_BYTES];
        if !self.primed && self.queued.len() >= PRIME_BYTES {
            self.primed = true;
        }
        if self.primed {
            if self.queued.len() >= CHUNK_BYTES {
                for (slot, byte) in chunk.iter_mut().zip(self.queued.drain(..CHUNK_BYTES)) {
                    *slot = byte;
                }
            } else {
                // Run dry: one gap, then gather again rather than click through every lump.
                self.primed = false;
            }
        }
        chunk
    }

    /// A new stream starts on a sample boundary of its own; what is left of the old one may not.
    pub fn clear(&mut self) {
        self.queued.clear();
        self.primed = false;
    }

    #[cfg(test)]
    fn queued(&self) -> usize {
        self.queued.len()
    }
}

/// What the push says back.
#[derive(Debug, PartialEq, Eq)]
pub enum Flow {
    Continue,
    /// The pipeline is gone; so is the reason for this thread.
    Stop,
}

/// Run the track's clock until `push` says stop.
///
/// Every [`CHUNK`] it hands `push` 640 bytes and whether this chunk does *not* follow on from the
/// last — the first, and any after the thread fell more than [`RESYNC_AFTER`] behind, as a
/// throttled CPU can make it. In between it keeps the tap connected while `wanted()` and
/// disconnected while not.
pub fn feed(
    socket: PathBuf,
    wanted: impl Fn() -> bool,
    mut push: impl FnMut(&[u8; CHUNK_BYTES], bool) -> Flow,
) {
    let mut jitter = Jitter::default();
    let mut tap: Option<UnixStream> = None;
    let mut last_attempt: Option<Instant> = None;
    // Said once per change rather than per attempt: a robot with audio off would otherwise log
    // a refused connect every second for as long as a peer stays.
    let mut said_missing = false;
    let mut buf = [0u8; 4096];

    let mut deadline = Instant::now();
    let mut discont = true;
    loop {
        let now = Instant::now();
        if now < deadline {
            std::thread::sleep(deadline - now);
        } else if now - deadline > RESYNC_AFTER {
            deadline = now;
            discont = true;
        }
        deadline += CHUNK;

        if wanted() {
            if tap.is_none() && last_attempt.is_none_or(|at| at.elapsed() >= RECONNECT_EVERY) {
                last_attempt = Some(Instant::now());
                match UnixStream::connect(&socket).and_then(|s| s.set_nonblocking(true).map(|_| s))
                {
                    Ok(stream) => {
                        tracing::info!(socket = %socket.display(), "mic: listening to robotd");
                        jitter.clear();
                        tap = Some(stream);
                        said_missing = false;
                    }
                    Err(error) => {
                        if !said_missing {
                            tracing::info!(
                                %error,
                                socket = %socket.display(),
                                "mic: robotd is not lending the mic ([audio] off, or robotd not \
                                 running); the peer hears silence until it does"
                            );
                            said_missing = true;
                        }
                    }
                }
            }
        } else if tap.take().is_some() {
            // Nobody left to hear it: let `robotd` stop recording.
            tracing::info!("mic: no peers; closing the tap");
            jitter.clear();
            last_attempt = None;
        }

        if let Some(stream) = tap.as_mut() {
            let lost = loop {
                match stream.read(&mut buf) {
                    Ok(0) => break Some("robotd closed the tap".to_owned()),
                    Ok(n) => jitter.feed(&buf[..n]),
                    Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break None,
                    Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
                    Err(e) => break Some(e.to_string()),
                }
            };
            if let Some(reason) = lost {
                // `robotd` restarting, or it dropped us for falling behind. Either way it is worth
                // asking again straight away rather than in a second.
                tracing::info!(%reason, "mic: lost the tap; silence until it is back");
                tap = None;
                last_attempt = None;
                jitter.clear();
            }
        }

        if push(&jitter.take(), std::mem::take(&mut discont)) == Flow::Stop {
            return;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use std::os::unix::net::UnixListener;
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, Ordering};

    #[test]
    fn a_chunk_is_twenty_milliseconds_of_the_tap() {
        assert_eq!(CHUNK_BYTES, 640);
        assert_eq!(CHUNK_BYTES, CHUNK.as_millis() as usize * BYTES_PER_MS);
    }

    /// Nothing plays until a burst and a half is waiting, so the gaps between `arecord`'s periods
    /// are ridden through rather than heard as a click every 125 ms.
    #[test]
    fn audio_waits_until_enough_has_gathered() {
        let mut jitter = Jitter::default();
        jitter.feed(&[7; PRIME_BYTES - 2]);
        assert!(jitter.take().iter().all(|b| *b == 0), "not primed yet");
        jitter.feed(&[7; 2]);
        assert!(jitter.take().iter().all(|b| *b == 7), "primed");
    }

    /// Running dry is one gap and then gathering again, not a chunk of mic for every chunk of
    /// silence while the next burst trickles in.
    #[test]
    fn running_dry_gathers_again() {
        let mut jitter = Jitter::default();
        jitter.feed(&[7; PRIME_BYTES]);
        let chunks = PRIME_BYTES / CHUNK_BYTES;
        for _ in 0..chunks {
            assert!(jitter.take().iter().all(|b| *b == 7));
        }
        // The next burst is late: this chunk is the gap.
        assert!(jitter.take().iter().all(|b| *b == 0));
        jitter.feed(&[7; CHUNK_BYTES]);
        assert!(
            jitter.take().iter().all(|b| *b == 0),
            "one chunk after running dry is not enough to play again"
        );
    }

    /// **A backlog is dropped from the front, by whole samples.** Odd here would put every sample
    /// after it a byte out of step — full-scale noise where a voice was.
    #[test]
    fn a_backlog_is_cut_back_by_whole_samples() {
        let mut jitter = Jitter::default();
        // Distinguishable samples: the low byte counts up, the high byte is a marker.
        let mut stream = Vec::new();
        for i in 0..(CAP_BYTES / 2 + 101) {
            stream.extend_from_slice(&[(i % 251) as u8, 0xAB]);
        }
        // Split at an odd offset, the way a stream socket may.
        jitter.feed(&stream[..1001]);
        jitter.feed(&stream[1001..]);
        assert!(jitter.queued() <= CAP_BYTES);
        assert!(jitter.queued() >= PRIME_BYTES);
        let chunk = jitter.take();
        assert!(
            chunk.chunks(2).all(|sample| sample[1] == 0xAB),
            "every sample must still start on an even byte"
        );
    }

    fn tap_at(dir: &tempfile::TempDir) -> (PathBuf, UnixListener) {
        let path = dir.path().join("mic.sock");
        let listener = UnixListener::bind(&path).unwrap();
        (path, listener)
    }

    /// The clock runs with no tap at all, which is what keeps a robot with audio off — or no
    /// `robotd` — from holding up `webrtcsink`'s negotiation.
    #[test]
    fn with_no_tap_the_track_is_silence_on_time() {
        let dir = tempfile::tempdir().unwrap();
        let mut chunks = 0;
        let started = Instant::now();
        feed(
            dir.path().join("absent.sock"),
            || true,
            |chunk, _| {
                assert!(chunk.iter().all(|b| *b == 0));
                chunks += 1;
                if chunks == 10 {
                    Flow::Stop
                } else {
                    Flow::Continue
                }
            },
        );
        let took = started.elapsed();
        assert!(
            took >= CHUNK * 9 && took < CHUNK * 50,
            "ten chunks are paced at 20 ms: {took:?}"
        );
    }

    /// The tap's audio comes out of the track, after the priming delay.
    #[test]
    fn the_tap_is_heard_once_primed() {
        let dir = tempfile::tempdir().unwrap();
        let (path, listener) = tap_at(&dir);
        let robotd = std::thread::spawn(move || {
            let (mut stream, _) = listener.accept().unwrap();
            // Bursts the size `arecord` hands over, faster than real time.
            for _ in 0..8 {
                if stream.write_all(&[0x22; 4000]).is_err() {
                    return;
                }
            }
            std::thread::sleep(Duration::from_secs(2));
        });
        let mut heard = false;
        let mut chunks = 0;
        feed(
            path,
            || true,
            |chunk, _| {
                chunks += 1;
                heard |= chunk.iter().all(|b| *b == 0x22);
                if heard || chunks > 100 {
                    Flow::Stop
                } else {
                    Flow::Continue
                }
            },
        );
        assert!(heard, "the tap's bytes must reach the track");
        drop(robotd);
    }

    /// **No peers, no tap.** The connection is what keeps `robotd` recording, so holding it with
    /// nobody to hear would be a duck recording its room for nobody.
    #[test]
    fn the_tap_is_closed_when_nobody_wants_it() {
        let dir = tempfile::tempdir().unwrap();
        let (path, listener) = tap_at(&dir);
        let wanted = Arc::new(AtomicBool::new(true));
        let closed = Arc::new(AtomicBool::new(false));
        {
            let closed = Arc::clone(&closed);
            std::thread::spawn(move || {
                let (mut stream, _) = listener.accept().unwrap();
                let mut byte = [0u8; 1];
                // The tap end sees EOF when mediad lets go.
                if matches!(stream.read(&mut byte), Ok(0)) {
                    closed.store(true, Ordering::Release);
                }
            });
        }
        let mut chunks = 0;
        let w = Arc::clone(&wanted);
        feed(
            path,
            move || w.load(Ordering::Acquire),
            |_, _| {
                chunks += 1;
                if chunks == 5 {
                    wanted.store(false, Ordering::Release);
                }
                if closed.load(Ordering::Acquire) || chunks > 100 {
                    Flow::Stop
                } else {
                    Flow::Continue
                }
            },
        );
        assert!(closed.load(Ordering::Acquire), "the tap must be let go of");
    }

    /// The first chunk says it starts a stream, and the ones after it do not — the timestamps are
    /// built from that, and a discontinuity on every chunk is a track that never plays.
    #[test]
    fn only_the_first_chunk_is_a_discontinuity_on_a_steady_clock() {
        let dir = tempfile::tempdir().unwrap();
        let mut flags = Vec::new();
        feed(
            dir.path().join("absent.sock"),
            || false,
            |_, discont| {
                flags.push(discont);
                if flags.len() == 5 {
                    Flow::Stop
                } else {
                    Flow::Continue
                }
            },
        );
        assert_eq!(flags, [true, false, false, false, false]);
    }
}
