//! Whether the pad is driving, while the duck on its own is switched on (`robot.mode`'s
//! `autonomous`, which `robotd` starts from `[autonomous] enabled` and an NFC tag can change).
//!
//! Nothing in `robotd` arbitrates between drivers, so the handoff is the pad daemon keeping quiet.
//! A pad that is merely connected sends nothing: its heartbeat would hold `move.source` at the
//! pad forever, and the brain yields to that. Touching it engages it, and it then drives exactly as
//! it always has. `pad_idle_s` after the last touch it lets go — one zero twist, so the robot stops
//! on the pad's word rather than on the deadman — and goes quiet again.
//! `docs/design/autonomous.md` §3 owns the handoff.
//!
//! With autonomy off this always says [`Engage::Drive`], so nothing changes for a duck without a
//! brain.

use std::time::{Duration, Instant};

use crate::{Buttons, PadFrame};

#[derive(Debug, Default, Clone, Copy)]
pub struct Engagement {
    last_touch: Option<Instant>,
    engaged: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Engage {
    /// Send this tick's intents.
    Drive,
    /// The idle time just ran out: stop the robot once, then stay silent.
    LetGo,
    /// Send nothing.
    Silent,
}

/// Anything a person does with the pad: a button down, up or held, a stick or trigger past the
/// deadzone. A pad lying on the table with its sticks centred is not touched.
pub fn touched(frame: &PadFrame, deadzone: f64) -> bool {
    frame.held != Buttons::NONE
        || frame.pressed != Buttons::NONE
        || frame.released != Buttons::NONE
        || [
            frame.left_x,
            frame.left_y,
            frame.right_x,
            frame.right_y,
            frame.lt,
            frame.rt,
        ]
        .into_iter()
        .any(|v| v.abs() > deadzone)
}

impl Engagement {
    /// `on` is whether the duck on its own is switched on; `autonomy` gives the idle time.
    pub fn tick(
        &mut self,
        touched: bool,
        now: Instant,
        on: bool,
        autonomy: &robotd_params::AutonomousParams,
    ) -> Engage {
        if !on {
            self.engaged = true;
            return Engage::Drive;
        }
        if touched {
            if !self.engaged {
                tracing::warn!("pad touched — driving; the duck's own brain steps aside");
            }
            self.last_touch = Some(now);
            self.engaged = true;
            return Engage::Drive;
        }
        let idle = Duration::from_secs_f64(autonomy.pad_idle_s);
        let fresh = self
            .last_touch
            .is_some_and(|at| now.duration_since(at) < idle);
        match (self.engaged, fresh) {
            (true, true) => Engage::Drive,
            (true, false) => {
                self.engaged = false;
                tracing::warn!(
                    idle_s = autonomy.pad_idle_s,
                    "pad untouched — letting go; the duck takes back over"
                );
                Engage::LetGo
            }
            (false, _) => Engage::Silent,
        }
    }

    /// Whether the pad is the one driving. A pad that goes away while it is not driving was not
    /// holding the robot, so losing it is no reason to sit the robot down.
    pub fn engaged(&self) -> bool {
        self.engaged
    }

    /// The pad went away: whatever it was doing, it is not driving now, and the next pad starts
    /// silent until touched.
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn autonomy(enabled: bool) -> robotd_params::AutonomousParams {
        robotd_params::AutonomousParams {
            enabled,
            pad_idle_s: 30.0,
        }
    }

    /// Without a brain, the pad is the robot's only driver and must never go quiet — a pad that
    /// stopped heartbeating because nobody touched it would put `deadman` in every frame.
    #[test]
    fn without_autonomy_the_pad_always_drives() {
        let mut e = Engagement::default();
        let t0 = Instant::now();
        assert_eq!(e.tick(false, t0, false, &autonomy(false)), Engage::Drive);
        assert_eq!(
            e.tick(
                false,
                t0 + Duration::from_secs(3600),
                false,
                &autonomy(false)
            ),
            Engage::Drive
        );
    }

    /// With a brain: silent until touched, driving while touched, one let-go after the idle time,
    /// then silent — and a touch after that engages again at once.
    #[test]
    fn with_autonomy_the_pad_drives_only_while_touched() {
        let on = autonomy(true);
        let mut e = Engagement::default();
        let t0 = Instant::now();
        let at = |s: u64| t0 + Duration::from_secs(s);
        assert_eq!(
            e.tick(false, t0, true, &on),
            Engage::Silent,
            "connected, untouched"
        );
        assert_eq!(e.tick(true, at(1), true, &on), Engage::Drive);
        assert_eq!(
            e.tick(false, at(20), true, &on),
            Engage::Drive,
            "inside the idle time"
        );
        assert_eq!(
            e.tick(false, at(31), true, &on),
            Engage::LetGo,
            "exactly once"
        );
        assert_eq!(e.tick(false, at(32), true, &on), Engage::Silent);
        assert_eq!(e.tick(true, at(40), true, &on), Engage::Drive);
    }

    /// A pad that drops and comes back must not resume driving on the strength of a touch made
    /// to the pad before it left.
    #[test]
    fn a_pad_that_comes_back_starts_silent() {
        let on = autonomy(true);
        let mut e = Engagement::default();
        let t0 = Instant::now();
        assert_eq!(e.tick(true, t0, true, &on), Engage::Drive);
        e.reset();
        assert!(!e.engaged());
        assert_eq!(
            e.tick(false, t0 + Duration::from_secs(1), true, &on),
            Engage::Silent
        );
    }

    /// Centred sticks and released triggers are not a touch; a stick past the deadzone is.
    #[test]
    fn a_centred_pad_is_not_touched() {
        let mut frame = PadFrame::default();
        assert!(!touched(&frame, 0.1));
        frame.left_x = 0.05;
        assert!(!touched(&frame, 0.1), "inside the deadzone");
        frame.rt = 0.5;
        assert!(touched(&frame, 0.1));
    }
}
