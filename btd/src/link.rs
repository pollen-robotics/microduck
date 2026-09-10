//! The seam between the radio and everything worth testing.
//!
//! Deliberately **not a trait**. A `GattLink` trait would need an async `recv` and an async
//! `send`, and the session loop has to wait on both at once — which means either splitting the
//! link into halves with associated types, or fighting the borrow checker inside a `select!`.
//! Two channels and a plain struct express the same thing with none of that, and the test
//! constructs one by hand rather than implementing anything.
//!
//! So a backend's whole job is: accept a connection, feed inbound chunks into `inbound`, write
//! whatever appears on `outbound` back to the central, and drop the channels when the central
//! goes away. What happens between those two channels is [`crate::session`], and it never
//! learns whether a radio is involved.

use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};

use tokio::sync::mpsc;

/// How many chunks may queue in either direction.
///
/// Sized so that a **maximal inbound line never has to block**, which is a correctness
/// requirement rather than a tuning choice. The radio backend must hand chunks over without
/// awaiting: BlueZ dispatches each write as its own task, so any yield point between receiving a
/// chunk and enqueueing it is a chance for two chunks to swap places — and a reordered chunk
/// silently corrupts a request. Chunk 2 of 3 arriving last once produced
/// `{"id":1,"jsonrpc":"2.info","params":{}}`, which is valid JSON missing a field, and a parse
/// error blaming the client.
///
/// So the queue has to be deep enough that a synchronous `try_send` cannot fail on legitimate
/// traffic: `QUEUE * 20 >= framing::MAX_LINE`, where 20 is the smallest payload BLE guarantees.
/// A test asserts that relationship. Beyond it, a flood gets a clean ATT error rather than a
/// dropped chunk, because failing a write is recoverable and corrupting one is not.
pub const QUEUE: usize = 512;

/// One connected central.
pub struct Link {
    /// Chunks written by the central to the `request` characteristic, in arrival order.
    /// Closed when it disconnects.
    pub inbound: mpsc::Receiver<Vec<u8>>,
    /// Chunks to notify on the `response` characteristic.
    pub outbound: mpsc::Sender<Vec<u8>>,
    /// Usable notification payload — `ATT_MTU - 3` — as negotiated for this connection.
    ///
    /// **A shared cell rather than a number, because the two halves of the link learn it at
    /// different times.** BlueZ reports the negotiated MTU on every inbound write and offers the
    /// notify side no way to ask, so a session begins knowing only the 20-byte floor and learns
    /// the real value from the first write a client makes — which is always `system.authenticate`,
    /// before any reply worth chunking exists. Sizing replies for the floor for the whole session
    /// was a tenfold cost in notifications, and above about 5 KiB it stopped being merely slow:
    /// see the pacing in `bluez`.
    ///
    /// Read once per *line*, never per chunk, which is the invariant that matters — a line chunked
    /// two different ways cannot be reassembled.
    mtu: Arc<AtomicUsize>,
    /// The central's address, for the log line. Never used for authorization — a BLE address
    /// is trivially spoofed, and pairing is what authorizes (`architecture.md` §4.2).
    pub peer: String,
}

impl Link {
    /// A link wired to channels the caller drives, with a payload size that never changes.
    /// Used by tests and by `--fake`.
    pub fn pair(
        mtu: usize,
        peer: impl Into<String>,
    ) -> (Self, mpsc::Sender<Vec<u8>>, mpsc::Receiver<Vec<u8>>) {
        Self::pair_sharing_mtu(Arc::new(AtomicUsize::new(mtu)), peer)
    }

    /// A link whose payload size is written by somebody else — the radio backend, from what
    /// BlueZ reports on each inbound write. See [`Link::mtu`].
    pub fn pair_sharing_mtu(
        mtu: Arc<AtomicUsize>,
        peer: impl Into<String>,
    ) -> (Self, mpsc::Sender<Vec<u8>>, mpsc::Receiver<Vec<u8>>) {
        let (to_robot, inbound) = mpsc::channel(QUEUE);
        let (outbound, from_robot) = mpsc::channel(QUEUE);
        (
            Self {
                inbound,
                outbound,
                mtu,
                peer: peer.into(),
            },
            to_robot,
            from_robot,
        )
    }

    /// The payload to size the next outbound line for.
    pub fn mtu(&self) -> usize {
        self.mtu.load(Ordering::Relaxed)
    }
}

/// The queue must be deep enough that a maximal line never needs a blocking send.
///
/// A compile-time check rather than a test, because it is a relationship between two constants
/// and nothing about it can be true at runtime and false at build time. 20 bytes is the payload
/// every BLE link is required to support, and therefore the smallest chunk a client may use.
///
/// This is the invariant behind the radio backend using a synchronous `try_send`: it may not
/// await, because a yield point between receiving a chunk and enqueueing it lets two chunks swap
/// places, and a reordered chunk corrupts a request rather than failing it.
const _: () = assert!(
    QUEUE * 20 >= crate::framing::MAX_LINE,
    "QUEUE * 20 must be at least framing::MAX_LINE, or a full-length request can fill the \
     inbound queue and be refused"
);
