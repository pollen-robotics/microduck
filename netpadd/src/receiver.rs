//! What arrives on the socket, turned into "send this frame now" — with no I/O and no clock of
//! its own, so every rule below is a unit test rather than a timing experiment.
//!
//! Nothing is queued. The newest accepted state replaces the last one; the edges between them are
//! accumulated per datagram so none is lost to merging; a send happens on arrival when the last
//! one is at least `min_interval` old, else at that slot. The spec's §3 is the argument.

use std::net::SocketAddr;
use std::time::{Duration, Instant};

use pad_map::wire::{Packet, WireError};
use pad_map::{Buttons, PadFrame};

/// Assumes `min_interval <= fallback`, which `[netpad] max_hz >= 10` guarantees in robotd-params.
#[derive(Debug, Clone, Copy)]
pub struct Timing {
    /// `1 / [netpad] max_hz`.
    pub min_interval: Duration,
    /// `[netpad] timeout_ms`.
    pub timeout: Duration,
    /// How often the mapping runs with no new datagram — `pad_map::HEARTBEAT`.
    pub fallback: Duration,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Accept {
    /// Accepted, and it is the first since the client was last gone.
    Connected,
    Taken,
    /// Not newer than what is held.
    Stale,
    /// Another sender holds the lock.
    OtherPeer,
    Malformed(WireError),
}

#[derive(Debug, PartialEq)]
pub enum Step {
    /// Run the mapping on this.
    Tick(PadFrame),
    /// The client just went quiet: send nothing from now, and reset the holds.
    Gone,
    Idle,
}

#[derive(Debug)]
pub struct Receiver {
    timing: Timing,
    /// The client, while it is alive. `None` means nobody is driving.
    peer: Option<SocketAddr>,
    latest: Packet,
    last_rx: Instant,
    last_tick: Option<Instant>,
    /// Something was accepted since the last tick.
    fresh: bool,
    pressed: Buttons,
    released: Buttons,
    reserved_seen: u16,
}

impl Receiver {
    pub fn new(timing: Timing) -> Self {
        debug_assert!(timing.min_interval <= timing.fallback);
        Self {
            timing,
            peer: None,
            latest: Packet::default(),
            last_rx: Instant::now(),
            last_tick: None,
            fresh: false,
            pressed: Buttons::NONE,
            released: Buttons::NONE,
            reserved_seen: 0,
        }
    }

    pub fn on_datagram(&mut self, now: Instant, from: SocketAddr, bytes: &[u8]) -> Accept {
        let packet = match Packet::decode(bytes) {
            Ok(p) => p,
            Err(e) => return Accept::Malformed(e),
        };
        self.reserved_seen |= packet.reserved();
        let connected = match self.peer {
            Some(peer) if peer != from => return Accept::OtherPeer,
            Some(_) => {
                // Newer, in wrapping order: within half the sequence space ahead.
                if (packet.seq.wrapping_sub(self.latest.seq) as i32) <= 0 {
                    return Accept::Stale;
                }
                let (now_held, before) = (packet.held(), self.latest.held());
                self.pressed |= now_held.newly_set(before);
                self.released |= before.newly_set(now_held);
                false
            }
            None => {
                // A new client, or the old one back: what it holds was pressed before we saw it,
                // and firing it now would run a skill nobody just asked for.
                self.peer = Some(from);
                self.pressed = Buttons::NONE;
                self.released = Buttons::NONE;
                self.last_tick = None;
                true
            }
        };
        self.latest = packet;
        self.last_rx = now;
        self.fresh = true;
        if connected {
            Accept::Connected
        } else {
            Accept::Taken
        }
    }

    pub fn poll(&mut self, now: Instant) -> Step {
        if self.peer.is_none() {
            return Step::Idle;
        }
        if now.duration_since(self.last_rx) >= self.timing.timeout {
            self.peer = None;
            self.fresh = false;
            return Step::Gone;
        }
        let due = match self.last_tick {
            None => true,
            Some(at) => {
                let since = now.duration_since(at);
                (self.fresh && since >= self.timing.min_interval) || since >= self.timing.fallback
            }
        };
        if !due {
            return Step::Idle;
        }
        let frame = self.latest.frame(self.pressed, self.released);
        self.pressed = Buttons::NONE;
        self.released = Buttons::NONE;
        self.fresh = false;
        self.last_tick = Some(now);
        Step::Tick(frame)
    }

    /// When `poll` next has something to say, or `None` when nobody is driving.
    pub fn deadline(&self) -> Option<Instant> {
        self.peer?;
        let timeout = self.last_rx + self.timing.timeout;
        let next = match self.last_tick {
            None => return Some(self.last_rx),
            Some(at) if self.fresh => at + self.timing.min_interval,
            Some(at) => at + self.timing.fallback,
        };
        Some(next.min(timeout))
    }

    /// Button bits this build does not know, OR-ed over everything accepted — for one log line.
    pub fn reserved_bits_seen(&self) -> u16 {
        self.reserved_seen
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pad_map::wire::Packet;

    const A: &str = "10.0.0.2:5000";
    const B: &str = "10.0.0.3:5000";

    fn rx() -> Receiver {
        Receiver::new(Timing {
            min_interval: Duration::from_millis(33),
            timeout: Duration::from_millis(250),
            fallback: Duration::from_millis(100),
        })
    }
    fn addr(s: &str) -> SocketAddr {
        s.parse().unwrap()
    }
    fn pkt(seq: u32, buttons: Buttons, lx: f64) -> [u8; 28] {
        Packet::from_axes(seq, 0, [lx, 0.0, 0.0, 0.0], [0.0, 0.0], buttons).encode()
    }
    fn ms(t0: Instant, n: u64) -> Instant {
        t0 + Duration::from_millis(n)
    }
    fn tick(step: Step) -> PadFrame {
        match step {
            Step::Tick(f) => f,
            other => panic!("expected a tick, got {other:?}"),
        }
    }

    #[test]
    fn a_first_datagram_is_sent_at_once() {
        let (mut r, t) = (rx(), Instant::now());
        assert_eq!(
            r.on_datagram(t, addr(A), &pkt(1, Buttons::NONE, 0.5)),
            Accept::Connected
        );
        assert!((tick(r.poll(t)).left_x - 0.5).abs() < 1e-4);
    }

    /// The cap: a second change inside one interval waits for the slot, and the slot sends the
    /// newest state, not the one in between.
    #[test]
    fn changes_inside_an_interval_merge_into_the_newest_at_the_slot() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(1, Buttons::NONE, 0.1));
        tick(r.poll(t));
        r.on_datagram(ms(t, 5), addr(A), &pkt(2, Buttons::NONE, 0.2));
        r.on_datagram(ms(t, 10), addr(A), &pkt(3, Buttons::NONE, 0.3));
        assert_eq!(r.poll(ms(t, 10)), Step::Idle, "inside the interval");
        assert_eq!(r.deadline(), Some(ms(t, 33)));
        assert!(
            (tick(r.poll(ms(t, 33))).left_x - 0.3).abs() < 1e-4,
            "the newest"
        );
    }

    /// Once the last send is an interval old, a change goes out on arrival — no waiting for a
    /// periodic slot. This is the latency the design exists for.
    #[test]
    fn a_change_after_a_quiet_interval_is_sent_on_arrival() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(1, Buttons::NONE, 0.0));
        tick(r.poll(t));
        r.on_datagram(ms(t, 60), addr(A), &pkt(2, Buttons::NONE, 0.9));
        assert!((tick(r.poll(ms(t, 60))).left_x - 0.9).abs() < 1e-4);
    }

    #[test]
    fn a_late_datagram_is_dropped() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(5, Buttons::NONE, 0.5));
        assert_eq!(
            r.on_datagram(t, addr(A), &pkt(4, Buttons::NONE, -0.5)),
            Accept::Stale
        );
        assert_eq!(
            r.on_datagram(t, addr(A), &pkt(5, Buttons::NONE, -0.5)),
            Accept::Stale
        );
        assert!((tick(r.poll(t)).left_x - 0.5).abs() < 1e-4);
    }

    #[test]
    fn seq_wraps() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(u32::MAX, Buttons::NONE, 0.0));
        assert_eq!(
            r.on_datagram(t, addr(A), &pkt(0, Buttons::NONE, 0.0)),
            Accept::Taken
        );
        assert_eq!(
            r.on_datagram(t, addr(A), &pkt(u32::MAX, Buttons::NONE, 0.0)),
            Accept::Stale
        );
    }

    /// A press and a release that both land between two sends still make a tap.
    #[test]
    fn a_press_and_release_inside_one_interval_are_both_seen() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(1, Buttons::NONE, 0.0));
        tick(r.poll(t));
        r.on_datagram(ms(t, 5), addr(A), &pkt(2, Buttons::START, 0.0));
        r.on_datagram(ms(t, 10), addr(A), &pkt(3, Buttons::NONE, 0.0));
        let f = tick(r.poll(ms(t, 33)));
        assert!(f.pressed.contains(Buttons::START) && f.released.contains(Buttons::START));
        assert!(!f.held.contains(Buttons::START));
        let f = tick(r.poll(ms(t, 140)));
        assert_eq!(
            (f.pressed, f.released),
            (Buttons::NONE, Buttons::NONE),
            "edges are consumed"
        );
    }

    #[test]
    fn the_first_datagram_makes_no_edges() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(1, Buttons::A | Buttons::UP, 0.0));
        let f = tick(r.poll(t));
        assert_eq!(f.pressed, Buttons::NONE);
        assert!(f.held.contains(Buttons::A));
    }

    #[test]
    fn another_peer_is_ignored_while_the_first_is_alive() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(1, Buttons::NONE, 0.5));
        assert_eq!(
            r.on_datagram(ms(t, 10), addr(B), &pkt(99, Buttons::NONE, -1.0)),
            Accept::OtherPeer
        );
        assert!((tick(r.poll(ms(t, 10))).left_x - 0.5).abs() < 1e-4);
    }

    #[test]
    fn going_away_reports_gone_once_and_frees_the_lock() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(1, Buttons::START, 0.5));
        tick(r.poll(t));
        tick(r.poll(ms(t, 100)));
        tick(r.poll(ms(t, 200)));
        assert_eq!(r.poll(ms(t, 250)), Step::Gone);
        assert_eq!(r.poll(ms(t, 300)), Step::Idle);
        assert_eq!(r.deadline(), None, "nothing to wake for");
        assert_eq!(
            r.on_datagram(ms(t, 400), addr(B), &pkt(1, Buttons::NONE, 0.0)),
            Accept::Connected
        );
    }

    #[test]
    fn a_restarted_client_is_taken_after_the_timeout() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(1000, Buttons::NONE, 0.0));
        tick(r.poll(t));
        assert_eq!(
            r.on_datagram(ms(t, 50), addr(A), &pkt(3, Buttons::NONE, 0.0)),
            Accept::Stale
        );
        assert_eq!(r.poll(ms(t, 250)), Step::Gone);
        assert_eq!(
            r.on_datagram(ms(t, 260), addr(A), &pkt(4, Buttons::NONE, 0.0)),
            Accept::Connected
        );
    }

    /// Lost datagrams do not stop the mapping: hold timing and the heartbeat keep their clock.
    #[test]
    fn the_fallback_ticks_with_no_new_datagram() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(1, Buttons::NONE, 0.0));
        tick(r.poll(t));
        assert_eq!(r.poll(ms(t, 99)), Step::Idle);
        assert_eq!(r.deadline(), Some(ms(t, 100)));
        tick(r.poll(ms(t, 100)));
    }

    #[test]
    fn malformed_is_reported_and_changes_nothing() {
        let (mut r, t) = (rx(), Instant::now());
        assert!(matches!(
            r.on_datagram(t, addr(A), b"hello"),
            Accept::Malformed(_)
        ));
        assert_eq!(r.poll(t), Step::Idle);
        assert_eq!(
            r.on_datagram(t, addr(B), &pkt(1, Buttons::NONE, 0.0)),
            Accept::Connected,
            "no lock taken"
        );
    }
}
