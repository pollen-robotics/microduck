//! Turning a GATT characteristic into a line-oriented byte pipe.
//!
//! The control plane is NDJSON — one JSON object per line — over every transport
//! (`architecture.md` §2.2). BLE is the only one where a message does not fit in a single
//! write: usable payload is `ATT_MTU - 3`, which on a phone that never renegotiates is 20
//! bytes, and on a good link is a few hundred. So a line arrives in pieces and leaves in
//! pieces.
//!
//! **There is no framing header.** The newline that already separates messages is the frame
//! delimiter, in both directions. That is safe rather than lucky: `serde_json` escapes a
//! newline inside a string as `\n`, so a raw `0x0A` never appears inside a serialised JSON
//! object — which is the same property that makes NDJSON work on the unix socket.
//!
//! Adding a length prefix would mean a second framing dialect that only BLE speaks, and
//! every client would have to implement it. A phone app can instead do exactly what
//! `robotctl` does: write bytes, read until newline.

/// Longest line we will reassemble before giving up on a peer.
///
/// A BLE client that never sends a newline would otherwise grow this buffer without bound,
/// and it is reachable by anyone in radio range. Generous next to any real request — the
/// largest is an `update.apply` with a long ref — and far below `updaterd`'s own 1 MiB line
/// limit, because nothing that big has any business arriving over BLE.
///
/// **This bounds what a peer may send, not what the robot may answer.** The two are not the
/// same size and were one constant until a reply outgrew it: see [`MAX_REPLY_LINE`].
pub const MAX_LINE: usize = 8 * 1024;

/// Longest line a *client* will reassemble from the robot.
///
/// The other direction, and it needs its own number. [`MAX_LINE`] is a security bound — the
/// peer it limits is anyone in radio range, so it is deliberately tight — whereas this limits
/// the robot a client chose to connect to and already trusts. Reusing the tight one here made
/// every reply over 8 KiB unreadable, which quietly took `update.show` (kilobytes for an
/// ordinary run) and `system.logs` (a screenful of journal) out of reach of `duckctl` while
/// `btd` served both correctly.
///
/// 64 KiB, and the real limit on a reply is smaller still and lives elsewhere: `proto`'s
/// `MAX_LOG_BYTES` keeps the journal tail well inside this, because at a typical ATT MTU
/// 64 KiB is a minute on the air and no answer should take that long.
pub const MAX_REPLY_LINE: usize = 64 * 1024;

/// Reassembles inbound chunks into whole lines.
#[derive(Debug)]
pub struct Reassembler {
    buf: Vec<u8>,
    /// What "too long" means for this direction. See [`MAX_LINE`] and [`MAX_REPLY_LINE`].
    limit: usize,
}

/// The robot's side: [`MAX_LINE`], the tight bound, because the default must be the safe one.
impl Default for Reassembler {
    fn default() -> Self {
        Self {
            buf: Vec::new(),
            limit: MAX_LINE,
        }
    }
}

/// Why a peer's bytes were rejected. Both cases mean "drop the connection", but they are
/// logged differently: one is a client that cannot frame, the other may be an attack.
#[derive(Debug, PartialEq, Eq)]
pub enum FramingError {
    /// No newline within this reassembler's limit, which the variant carries because the two
    /// directions have different ones and the message must say which was hit.
    LineTooLong { limit: usize },
    /// Not valid UTF-8, so it cannot be JSON either.
    NotUtf8,
}

impl std::fmt::Display for FramingError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::LineTooLong { limit } => write!(f, "no newline within {limit} bytes"),
            Self::NotUtf8 => write!(f, "not valid UTF-8"),
        }
    }
}

// A real error, not just a Debug-able enum: this crate is a library, and a client using it — the
// `duckctl` example is the first — wants `?` to work.
impl std::error::Error for FramingError {}

impl Reassembler {
    pub fn new() -> Self {
        Self::default()
    }

    /// A reassembler for the answers coming *back*, bounded by [`MAX_REPLY_LINE`].
    ///
    /// Named for the direction rather than taking a number, so that a client reading replies
    /// cannot pick the wrong bound and `btd` cannot accidentally be given the loose one.
    pub fn for_replies() -> Self {
        Self {
            buf: Vec::new(),
            limit: MAX_REPLY_LINE,
        }
    }

    /// Feed one chunk; get back every complete line it completed.
    ///
    /// Returns a `Vec` because one write can legitimately carry several short lines — a
    /// client that batches `hello` and `update.status` into one 40-byte write is being
    /// efficient, not wrong.
    pub fn push(&mut self, chunk: &[u8]) -> Result<Vec<String>, FramingError> {
        if self.buf.len() + chunk.len() > self.limit {
            // Clear rather than keep the partial line: whatever follows is unparseable
            // anyway, and holding it would let a peer pin the memory.
            self.buf.clear();
            return Err(FramingError::LineTooLong { limit: self.limit });
        }
        self.buf.extend_from_slice(chunk);

        let mut lines = Vec::new();
        while let Some(at) = self.buf.iter().position(|&b| b == b'\n') {
            let line = self.buf.drain(..=at).collect::<Vec<u8>>();
            // Drop the newline, and tolerate CRLF from a client that helpfully added one.
            let line = &line[..line.len() - 1];
            let line = line.strip_suffix(b"\r").unwrap_or(line);

            if line.is_empty() {
                continue;
            }
            match std::str::from_utf8(line) {
                Ok(text) => lines.push(text.to_owned()),
                Err(_) => {
                    self.buf.clear();
                    return Err(FramingError::NotUtf8);
                }
            }
        }
        Ok(lines)
    }

    /// Bytes held pending a newline. For logging a peer that has gone quiet mid-line.
    pub fn pending(&self) -> usize {
        self.buf.len()
    }
}

/// Split one outbound line into notification-sized chunks, newline included.
///
/// The newline is part of the payload rather than a separate final chunk: a client that
/// reassembles until newline then needs no notion of "end of message" beyond the one it
/// already has.
pub fn chunks(line: &str, mtu: usize) -> Vec<Vec<u8>> {
    // A zero or absurd MTU would divide by zero or emit one chunk per byte. 20 is the
    // floor BLE guarantees before any negotiation.
    let mtu = mtu.max(20);

    let mut bytes = line.as_bytes().to_vec();
    if !bytes.ends_with(b"\n") {
        bytes.push(b'\n');
    }
    bytes.chunks(mtu).map(<[u8]>::to_vec).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_line_split_across_chunks_reassembles() {
        let mut r = Reassembler::new();
        assert!(r.push(b"{\"jsonrpc\":\"2.0\"").unwrap().is_empty());
        assert!(r.push(b",\"id\":1,\"method\":").unwrap().is_empty());
        let lines = r.push(b"\"hello\"}\n").unwrap();
        assert_eq!(lines, vec![r#"{"jsonrpc":"2.0","id":1,"method":"hello"}"#]);
        assert_eq!(r.pending(), 0);
    }

    /// One write may carry several lines, and all of them must come back.
    #[test]
    fn several_lines_in_one_chunk_all_come_back() {
        let mut r = Reassembler::new();
        assert_eq!(r.push(b"{\"a\":1}\n{\"b\":2}\n").unwrap().len(), 2);
    }

    /// A trailing partial line must be kept, not discarded: it is the next message's start.
    #[test]
    fn a_partial_trailing_line_is_retained() {
        let mut r = Reassembler::new();
        let lines = r.push(b"{\"a\":1}\n{\"b\":").unwrap();
        assert_eq!(lines, vec![r#"{"a":1}"#]);
        assert_eq!(r.push(b"2}\n").unwrap(), vec![r#"{"b":2}"#]);
    }

    /// Blank lines are ignored rather than forwarded as an empty request, matching what
    /// `robotd` and `updaterd` already do on their sockets.
    #[test]
    fn blank_lines_are_skipped() {
        let mut r = Reassembler::new();
        assert!(r.push(b"\n\n\r\n").unwrap().is_empty());
    }

    #[test]
    fn crlf_is_tolerated() {
        let mut r = Reassembler::new();
        assert_eq!(r.push(b"{\"a\":1}\r\n").unwrap(), vec![r#"{"a":1}"#]);
    }

    /// An unbounded peer must be cut off rather than allowed to grow the buffer. This is the
    /// one framing failure reachable by anybody in radio range.
    #[test]
    fn a_line_without_a_newline_is_refused_at_the_cap() {
        let mut r = Reassembler::new();
        let big = vec![b'x'; MAX_LINE + 1];
        assert_eq!(
            r.push(&big),
            Err(FramingError::LineTooLong { limit: MAX_LINE })
        );
        // And the buffer was released, so the peer cannot pin memory by retrying.
        assert_eq!(r.pending(), 0);
    }

    /// A reply larger than the request cap must reassemble on the client side.
    ///
    /// The regression this pins is one constant serving both directions: with a single 8 KiB
    /// cap, a `system.logs` answer arrived intact from the robot and died in `duckctl` as
    /// "no newline within 8192 bytes", which reads like a broken robot and is not one.
    #[test]
    fn a_reply_bigger_than_the_request_cap_is_accepted_by_a_client() {
        let reply = format!("{{\"result\":\"{}\"}}\n", "x".repeat(32 * 1024));

        let mut robot_side = Reassembler::new();
        assert!(matches!(
            robot_side.push(reply.as_bytes()),
            Err(FramingError::LineTooLong { limit: MAX_LINE })
        ));

        let mut client_side = Reassembler::for_replies();
        let lines = client_side.push(reply.as_bytes()).expect("under the cap");
        assert_eq!(lines.len(), 1);
        assert_eq!(lines[0].len(), reply.len() - 1);
    }

    #[test]
    fn invalid_utf8_is_refused() {
        let mut r = Reassembler::new();
        assert_eq!(r.push(&[0xff, 0xfe, b'\n']), Err(FramingError::NotUtf8));
    }

    /// Chunking and reassembly are each other's inverse — the property the whole transport
    /// rests on. Checked across MTUs from the BLE floor to bigger than the message.
    #[test]
    fn chunking_round_trips_at_every_mtu() {
        let line = format!(
            r#"{{"jsonrpc":"2.0","id":7,"result":{{"pad":"{}"}}}}"#,
            "y".repeat(300)
        );

        for mtu in [20, 23, 100, 185, 512, 4096] {
            let mut r = Reassembler::new();
            let mut got = Vec::new();
            for chunk in chunks(&line, mtu) {
                assert!(chunk.len() <= mtu.max(20), "chunk exceeded mtu {mtu}");
                got.extend(r.push(&chunk).unwrap());
            }
            assert_eq!(got, vec![line.clone()], "mtu {mtu}");
        }
    }

    /// A newline is appended if absent and never doubled if present — otherwise a client
    /// sees an empty line between every message.
    #[test]
    fn chunks_terminate_with_exactly_one_newline() {
        for line in ["{\"a\":1}", "{\"a\":1}\n"] {
            let flat: Vec<u8> = chunks(line, 512).concat();
            assert_eq!(flat.iter().filter(|&&b| b == b'\n').count(), 1, "{line:?}");
            assert_eq!(flat.last(), Some(&b'\n'), "{line:?}");
        }
    }

    /// A degenerate MTU must not panic or emit one chunk per byte.
    #[test]
    fn an_absurd_mtu_falls_back_to_the_ble_floor() {
        assert_eq!(chunks(&"z".repeat(40), 0).len(), 3); // 41 bytes over 20-byte chunks
    }
}
