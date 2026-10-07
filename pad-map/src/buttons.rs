//! The pad's buttons as a bitset. The bit order **is** the wire's (`wire.rs`), so a datagram's
//! `buttons` field is a `Buttons` without translation.

use std::ops::{BitOr, BitOrAssign};

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Buttons(pub u16);

impl Buttons {
    pub const NONE: Self = Self(0);
    pub const A: Self = Self(1 << 0);
    pub const B: Self = Self(1 << 1);
    pub const X: Self = Self(1 << 2);
    pub const Y: Self = Self(1 << 3);
    pub const LB: Self = Self(1 << 4);
    pub const RB: Self = Self(1 << 5);
    pub const START: Self = Self(1 << 6);
    pub const SELECT: Self = Self(1 << 7);
    pub const UP: Self = Self(1 << 8);
    pub const DOWN: Self = Self(1 << 9);
    pub const LEFT: Self = Self(1 << 10);
    pub const RIGHT: Self = Self(1 << 11);
    /// The middle button — Home, Xbox, PS. The flashlight, on a board with one.
    pub const HOME: Self = Self(1 << 12);
    /// Every bit that means something. The rest are reserved.
    pub const KNOWN: u16 = 0x1FFF;

    /// The six bindable buttons, by their `[pad]` config name, in `[pad]`'s order.
    pub const BINDABLE: [(Self, &'static str); 6] = [
        (Self::A, "a"),
        (Self::B, "b"),
        (Self::X, "x"),
        (Self::Y, "y"),
        (Self::LB, "lb"),
        (Self::RB, "rb"),
    ];

    pub fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0 && other.0 != 0
    }

    /// Bits set in `self` that were not set in `before` — the presses between two states.
    pub fn newly_set(self, before: Self) -> Self {
        Self(self.0 & !before.0)
    }
}

impl BitOr for Buttons {
    type Output = Self;
    fn bitor(self, rhs: Self) -> Self {
        Self(self.0 | rhs.0)
    }
}

impl BitOrAssign for Buttons {
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn edges_are_the_difference_in_each_direction() {
        let before = Buttons::A | Buttons::START;
        let now = Buttons::A | Buttons::B;
        assert_eq!(now.newly_set(before), Buttons::B, "pressed");
        assert_eq!(before.newly_set(now), Buttons::START, "released");
        assert!(
            !Buttons::NONE.contains(Buttons::NONE),
            "nothing is not a button"
        );
    }
}
