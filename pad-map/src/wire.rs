//! The pad on the wire: one UDP datagram per state, 28 bytes, little-endian.
//! `docs/robot/udp-pad.md` owns the format; this is its only implementation.
//!
//! Every datagram is the pad's *whole* state, never a delta, so a lost one costs nothing — the
//! next replaces it — and nothing ever has to be retransmitted or reordered.

use crate::{Buttons, PadFrame};

pub const MAGIC: [u8; 4] = *b"DKPD";
pub const VERSION: u8 = 1;
pub const LEN: usize = 28;
pub const DEFAULT_PORT: u16 = 4210;

const FULL: f64 = 32767.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Packet {
    /// +1 per datagram, wrapping. The receiver drops anything not newer than what it has.
    pub seq: u32,
    /// The sender's own clock, ms. For jitter in logs only; never compared with the robot's.
    pub t_ms: u32,
    /// lx ly rx ry (±32767 is ±1, up positive), lt rt (0..32767 is 0..1).
    pub axes: [i16; 6],
    pub buttons: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WireError {
    Short(usize),
    Magic,
    Version(u8),
}

impl std::fmt::Display for WireError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Short(n) => write!(f, "{n} bytes, a pad datagram is {LEN}"),
            Self::Magic => write!(f, "not a pad datagram (magic)"),
            Self::Version(v) => write!(f, "pad datagram version {v}, this build reads {VERSION}"),
        }
    }
}

impl Packet {
    pub fn encode(&self) -> [u8; LEN] {
        let mut b = [0u8; LEN];
        b[..4].copy_from_slice(&MAGIC);
        b[4] = VERSION;
        b[6..10].copy_from_slice(&self.seq.to_le_bytes());
        b[10..14].copy_from_slice(&self.t_ms.to_le_bytes());
        for (i, a) in self.axes.iter().enumerate() {
            b[14 + 2 * i..16 + 2 * i].copy_from_slice(&a.to_le_bytes());
        }
        b[26..28].copy_from_slice(&self.buttons.to_le_bytes());
        b
    }

    /// Longer than [`LEN`] is fine — appended fields this build does not know are ignored.
    pub fn decode(b: &[u8]) -> Result<Self, WireError> {
        if b.len() < LEN {
            return Err(WireError::Short(b.len()));
        }
        if b[..4] != MAGIC {
            return Err(WireError::Magic);
        }
        if b[4] != VERSION {
            return Err(WireError::Version(b[4]));
        }
        let u32_at = |i: usize| u32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]);
        let mut axes = [0i16; 6];
        for (i, a) in axes.iter_mut().enumerate() {
            *a = i16::from_le_bytes([b[14 + 2 * i], b[15 + 2 * i]]);
        }
        Ok(Self {
            seq: u32_at(6),
            t_ms: u32_at(10),
            axes,
            buttons: u16::from_le_bytes([b[26], b[27]]),
        })
    }

    pub fn held(&self) -> Buttons {
        Buttons(self.buttons & Buttons::KNOWN)
    }

    pub fn reserved(&self) -> u16 {
        self.buttons & !Buttons::KNOWN
    }

    pub fn frame(&self, pressed: Buttons, released: Buttons) -> PadFrame {
        let stick = |v: i16| (f64::from(v) / FULL).clamp(-1.0, 1.0);
        let trigger = |v: i16| (f64::from(v) / FULL).clamp(0.0, 1.0);
        let a = self.axes;
        PadFrame {
            left_x: stick(a[0]),
            left_y: stick(a[1]),
            right_x: stick(a[2]),
            right_y: stick(a[3]),
            lt: trigger(a[4]),
            rt: trigger(a[5]),
            held: self.held(),
            pressed,
            released,
            attitude: None,
            has_imu: false,
        }
    }

    /// For a sender: sticks `[lx, ly, rx, ry]` in -1..1, triggers `[lt, rt]` in 0..1.
    pub fn from_axes(
        seq: u32,
        t_ms: u32,
        sticks: [f64; 4],
        triggers: [f64; 2],
        held: Buttons,
    ) -> Self {
        let q = |v: f64, lo: f64| (v.clamp(lo, 1.0) * FULL).round() as i16;
        Self {
            seq,
            t_ms,
            axes: [
                q(sticks[0], -1.0),
                q(sticks[1], -1.0),
                q(sticks[2], -1.0),
                q(sticks[3], -1.0),
                q(triggers[0], 0.0),
                q(triggers[1], 0.0),
            ],
            buttons: held.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> Packet {
        Packet {
            seq: 0xDEAD_BEEF,
            t_ms: 42,
            axes: [32767, -32767, 0, 100, 32767, 0],
            buttons: 0b1_0100_0001,
        }
    }

    #[test]
    fn a_packet_round_trips_at_28_bytes() {
        let bytes = sample().encode();
        assert_eq!(bytes.len(), 28);
        assert_eq!(&bytes[..4], b"DKPD");
        assert_eq!(bytes[4], 1);
        assert_eq!(Packet::decode(&bytes), Ok(sample()));
    }

    #[test]
    fn the_layout_is_little_endian_at_the_documented_offsets() {
        let b = sample().encode();
        assert_eq!(
            u32::from_le_bytes(b[6..10].try_into().unwrap()),
            0xDEAD_BEEF
        );
        assert_eq!(u32::from_le_bytes(b[10..14].try_into().unwrap()), 42);
        assert_eq!(i16::from_le_bytes([b[14], b[15]]), 32767);
        assert_eq!(u16::from_le_bytes([b[26], b[27]]), 0b1_0100_0001);
    }

    #[test]
    fn garbage_is_refused_not_panicked_on() {
        assert_eq!(Packet::decode(&[]), Err(WireError::Short(0)));
        assert_eq!(Packet::decode(&[0u8; 27]), Err(WireError::Short(27)));
        let mut b = sample().encode();
        b[0] = b'X';
        assert_eq!(Packet::decode(&b), Err(WireError::Magic));
        let mut b = sample().encode();
        b[4] = 2;
        assert_eq!(Packet::decode(&b), Err(WireError::Version(2)));
    }

    /// Fields can be appended without a version bump: a v1 reader takes the 28 it knows.
    #[test]
    fn a_longer_v1_packet_is_read_and_its_tail_ignored() {
        let mut long = sample().encode().to_vec();
        long.extend_from_slice(&[9, 9, 9, 9]);
        assert_eq!(Packet::decode(&long), Ok(sample()));
    }

    #[test]
    fn i16_min_clamps_to_minus_one() {
        let p = Packet {
            axes: [i16::MIN, 0, 0, 0, i16::MIN, 0],
            ..Default::default()
        };
        let f = p.frame(Buttons::NONE, Buttons::NONE);
        assert_eq!(f.left_x, -1.0);
        assert_eq!(f.lt, 0.0, "a trigger is never negative");
    }

    #[test]
    fn reserved_bits_are_split_from_buttons() {
        let p = Packet {
            buttons: 0xF001,
            ..Default::default()
        };
        assert_eq!(p.held(), Buttons::A | Buttons::HOME, "bit 12 is Home");
        assert_eq!(p.reserved(), 0xE000);
    }

    #[test]
    fn from_axes_inverts_frame() {
        let p = Packet::from_axes(7, 9, [0.5, -1.0, 0.0, 1.0], [1.0, 0.25], Buttons::START);
        let f = p.frame(Buttons::NONE, Buttons::NONE);
        assert!((f.left_x - 0.5).abs() < 1e-4 && f.left_y == -1.0 && f.right_y == 1.0);
        assert!((f.lt - 1.0).abs() < 1e-4 && (f.rt - 0.25).abs() < 1e-4);
        assert_eq!(f.held, Buttons::START);
        assert!(f.attitude.is_none() && !f.has_imu);
    }
}
