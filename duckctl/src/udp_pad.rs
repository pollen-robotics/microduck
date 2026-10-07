//! `duckctl udp-pad` — the pad plugged into this machine, sent to a duck's `netpadd`.
//!
//! The reference sender for the format `docs/robot/udp-pad.md` owns. It sends the pad's whole
//! state the moment anything changes, at most `hz` times a second, and a keepalive every 1/hz
//! when nothing does. No pad, nothing sent — the robot times the client out and its deadman holds
//! it, exactly as when a Bluetooth pad goes away.

use std::hash::{BuildHasher, Hasher};
use std::net::UdpSocket;
use std::time::{Duration, Instant};

use gilrs::{Axis, Button, Gilrs};
use pad_map::Buttons;
use pad_map::wire::{DEFAULT_PORT, Packet};

/// When to send: a change and a keepalive are both capped at the interval. What a change buys
/// is that the loop wakes for it at once rather than at the next keepalive (see `run`).
pub struct Pacer {
    interval: Duration,
    last: Option<Instant>,
}

impl Pacer {
    pub fn new(interval: Duration) -> Self {
        Self {
            interval,
            last: None,
        }
    }

    /// Whether to send now; records it when yes. A change and a keepalive are both capped at
    /// the interval — the difference is that the loop wakes for a change at once (see `run`).
    pub fn due(&mut self, now: Instant) -> bool {
        let due = self
            .last
            .is_none_or(|at| now.duration_since(at) >= self.interval);
        if due {
            self.last = Some(now);
        }
        due
    }

    /// When the next send may happen.
    pub fn next(&self) -> Option<Instant> {
        self.last.map(|at| at + self.interval)
    }
}

fn target(host: &str) -> String {
    if host
        .rsplit_once(':')
        .is_some_and(|(_, p)| p.parse::<u16>().is_ok())
    {
        host.to_owned()
    } else {
        format!("{host}:{DEFAULT_PORT}")
    }
}

const GILRS: [(Button, Buttons); 12] = [
    (Button::South, Buttons::A),
    (Button::East, Buttons::B),
    (Button::West, Buttons::X),
    (Button::North, Buttons::Y),
    (Button::LeftTrigger, Buttons::LB),
    (Button::RightTrigger, Buttons::RB),
    (Button::Start, Buttons::START),
    (Button::Select, Buttons::SELECT),
    (Button::DPadUp, Buttons::UP),
    (Button::DPadDown, Buttons::DOWN),
    (Button::DPadLeft, Buttons::LEFT),
    (Button::DPadRight, Buttons::RIGHT),
];

/// How often the pad is read. Faster than any `hz` worth sending, so a change is seen within a
/// millisecond and the cap, not the polling, is what decides when it goes.
const POLL: Duration = Duration::from_millis(1);

pub fn run(host: &str, hz: u32) -> Result<(), Box<dyn std::error::Error>> {
    let mut gilrs = Gilrs::new().map_err(|e| format!("no gamepad subsystem: {e}"))?;
    let to = target(host);
    let socket = UdpSocket::bind("0.0.0.0:0")?;
    socket.connect(&to)?;
    let mut pacer = Pacer::new(Duration::from_secs_f64(1.0 / f64::from(hz)));
    // A random start, so a restarted sender is never mistaken for a stale one for long.
    let mut seq = std::collections::hash_map::RandomState::new()
        .build_hasher()
        .finish() as u32;
    let started = Instant::now();
    let (mut last, mut sent) = (None::<Packet>, 0u64);
    eprintln!("sending the first pad on this machine to {to}, at most {hz} Hz — Ctrl-C to stop");

    loop {
        while gilrs.next_event().is_some() {}
        let Some((_, pad)) = gilrs.gamepads().next() else {
            last = None;
            std::thread::sleep(Duration::from_millis(200));
            continue;
        };
        let held = GILRS
            .iter()
            .filter(|(b, _)| pad.is_pressed(*b))
            .fold(Buttons::NONE, |a, (_, bit)| a | *bit);
        let trigger = |b: Button| f64::from(pad.button_data(b).map_or(0.0, |d| d.value()));
        let now = Instant::now();
        let mut packet = Packet::from_axes(
            0,
            now.duration_since(started).as_millis() as u32,
            [
                f64::from(pad.value(Axis::LeftStickX)),
                f64::from(pad.value(Axis::LeftStickY)),
                f64::from(pad.value(Axis::RightStickX)),
                f64::from(pad.value(Axis::RightStickY)),
            ],
            [
                trigger(Button::LeftTrigger2),
                trigger(Button::RightTrigger2),
            ],
            held,
        );
        let changed = last.is_none_or(|l| l.axes != packet.axes || l.buttons != packet.buttons);
        if pacer.due(now) {
            seq = seq.wrapping_add(1);
            packet.seq = seq;
            // A refused send (no route yet, the duck rebooting) is not fatal: the next one tries again.
            if socket.send(&packet.encode()).is_ok() {
                sent += 1;
            }
            last = Some(packet);
            if sent % 300 == 1 {
                eprintln!("{sent} sent");
            }
        }
        let wake = if changed {
            now + POLL
        } else {
            pacer.next().unwrap_or(now + POLL).min(now + POLL * 5)
        };
        if let Some(d) = wake.checked_duration_since(Instant::now()) {
            std::thread::sleep(d);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

    #[test]
    fn sends_are_capped_at_the_interval_whether_or_not_anything_changed() {
        let t = Instant::now();
        let mut p = Pacer::new(Duration::from_millis(33));
        assert!(p.due(t), "first");
        assert!(!p.due(t + Duration::from_millis(10)), "capped");
        assert!(p.due(t + Duration::from_millis(33)), "change at the slot");
        assert!(!p.due(t + Duration::from_millis(50)), "steady, not yet");
        assert!(p.due(t + Duration::from_millis(66)), "keepalive");
    }

    #[test]
    fn host_without_port_gets_the_default() {
        assert_eq!(target("10.0.0.5"), "10.0.0.5:4210");
        assert_eq!(target("10.0.0.5:9000"), "10.0.0.5:9000");
        assert_eq!(target("duck.local"), "duck.local:4210");
    }
}
