//! Start and Select: buttons whose meaning is how long they stay down.

use std::time::{Duration, Instant};

/// Start held this long brings the robot to its home pose with the motors stiff and the policy
/// off: the "put everything back" button. Long enough that a press meant for the policy toggle
/// never reaches it, short enough to be the obvious thing to do when the robot is somewhere odd.
pub const HOME_HOLD: Duration = Duration::from_millis(1500);

/// Select let go after this long, and before [`SHUTDOWN_HOLD`], puts the robot down for a rest:
/// `robot.rest` — it sits if it is driving, eases into the rest pose, then torque goes off and
/// every servo reboots. Decided on the release, because until then the hold may still become a
/// power-off, and both start the same way.
pub const REST_HOLD: Duration = Duration::from_secs(2);

/// Select held this long powers the robot off: `robot.shutdown`, the same sit and rest pose ending
/// in a power-off. Sent the moment the hold gets here; the release after it does nothing.
pub const SHUTDOWN_HOLD: Duration = Duration::from_secs(4);

/// A button that does different things depending on how long it is held.
///
/// Each threshold fires once, on the tick the hold crosses it, so a longer hold walks through
/// every shorter one on the way — Select cuts torque at two seconds and then powers off at four.
/// A release that crossed no threshold is a [`HoldAction::Tap`]; the release after a hold that
/// did fire says nothing, because the thumb coming off is not a second request.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct HoldButton {
    /// When the current hold began. `None` between presses.
    held_since: Option<Instant>,
    /// How many thresholds this hold has crossed.
    fired: usize,
    /// The pad went away during a hold that had already fired. Whatever is still held when it
    /// comes back is the tail of that hold, and does nothing until the release.
    spent: bool,
}

/// What a [`HoldButton`] asks for this tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HoldAction {
    Nothing,
    /// Pressed and let go before the first threshold.
    Tap,
    /// The hold just crossed this threshold, by index.
    Reached(usize),
    /// Let go after a hold whose furthest threshold was this one, by index. For a button whose
    /// action depends on where the hold stopped, which is only known at the release.
    ReleasedAfter(usize),
}

impl HoldButton {
    /// One tick: is the button down now, and did it come up since the last tick.
    ///
    /// `released` is the edge from the event queue, because a press and release inside one tick
    /// leaves `pressed` false on both sides and would otherwise be a tap nobody saw.
    pub fn tick(
        &mut self,
        pressed: bool,
        released: bool,
        now: Instant,
        thresholds: &[Duration],
    ) -> HoldAction {
        if pressed {
            let since = *self.held_since.get_or_insert(now);
            if !self.spent
                && let Some(&next) = thresholds.get(self.fired)
                && now.duration_since(since) >= next
            {
                self.fired += 1;
                return HoldAction::Reached(self.fired - 1);
            }
            return HoldAction::Nothing;
        }
        let was_held = released || self.held_since.is_some();
        let (fired, spent) = (self.fired, self.spent);
        *self = Self::default();
        match (fired, spent) {
            // The tail of a hold cut by a dropout says nothing — see `reset`.
            (_, true) => HoldAction::Nothing,
            (0, false) if released => HoldAction::Tap,
            (0, false) => HoldAction::Nothing,
            (n, false) if was_held => HoldAction::ReleasedAfter(n - 1),
            (_, false) => HoldAction::Nothing,
        }
    }

    /// Forget a hold in flight. Called when the pad goes away: the hold's start was measured
    /// against *that* pad's button, and carrying it onto the next pad would turn a button still
    /// held across a long dropout into its longest action on the first tick back.
    ///
    /// A hold that already fired is marked spent rather than forgotten, because what it did is a
    /// fact about the robot rather than about the pad: the release that follows must stay silent,
    /// and the rest of the hold must not reach the next threshold from a fresh start.
    pub fn reset(&mut self) {
        self.held_since = None;
        if self.fired > 0 {
            self.spent = true;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SELECT: [Duration; 2] = [REST_HOLD, SHUTDOWN_HOLD];
    const START: [Duration; 1] = [HOME_HOLD];

    /// Select: a tap does nothing to the robot. Let go between two and four seconds, a rest —
    /// decided at the release, since the hold could still have become a power-off. Held to four,
    /// the power-off goes out at four, and its release adds nothing.
    #[test]
    fn select_rests_on_a_release_between_two_and_four_and_powers_off_at_four() {
        let t0 = Instant::now();
        let at = |ms: u64| t0 + Duration::from_millis(ms);
        let mut select = HoldButton::default();
        let mut tick = |pressed, released, ms| select.tick(pressed, released, at(ms), &SELECT);

        // A tap: reported, so the log can say what a hold would do.
        assert_eq!(tick(true, false, 0), HoldAction::Nothing);
        assert_eq!(tick(false, true, 300), HoldAction::Tap);

        // Let go at 1.9 s: still a tap.
        assert_eq!(tick(true, false, 1_000), HoldAction::Nothing);
        assert_eq!(tick(false, true, 2_900), HoldAction::Tap);

        // Let go at 3 s: two seconds reached, four not — a rest, at the release only.
        assert_eq!(tick(true, false, 10_000), HoldAction::Nothing);
        assert_eq!(tick(true, false, 12_000), HoldAction::Reached(0));
        assert_eq!(tick(true, false, 12_500), HoldAction::Nothing);
        assert_eq!(tick(false, true, 13_000), HoldAction::ReleasedAfter(0));
        assert_eq!(tick(false, false, 13_020), HoldAction::Nothing);

        // Held to four: the power-off at four, once, and a release that is not a rest.
        assert_eq!(tick(true, false, 20_000), HoldAction::Nothing);
        assert_eq!(tick(true, false, 22_000), HoldAction::Reached(0));
        assert_eq!(tick(true, false, 24_000), HoldAction::Reached(1));
        assert_eq!(tick(true, false, 24_020), HoldAction::Nothing);
        assert_eq!(tick(false, true, 25_000), HoldAction::ReleasedAfter(1));
    }

    /// Start: a tap is the stand-up / policy toggle, a 1.5 s hold is the way home — and a hold
    /// is never also a tap, or going home would toggle the policy straight back on.
    #[test]
    fn start_taps_toggle_and_a_long_hold_goes_home_only() {
        let t0 = Instant::now();
        let at = |ms: u64| t0 + Duration::from_millis(ms);
        let mut start = HoldButton::default();

        assert_eq!(start.tick(true, false, at(0), &START), HoldAction::Nothing);
        assert_eq!(start.tick(false, true, at(200), &START), HoldAction::Tap);

        // Press and release inside one tick: the state never read as down, the edge did.
        assert_eq!(start.tick(false, true, at(1_000), &START), HoldAction::Tap);

        assert_eq!(
            start.tick(true, false, at(2_000), &START),
            HoldAction::Nothing
        );
        assert_eq!(
            start.tick(true, false, at(3_490), &START),
            HoldAction::Nothing
        );
        assert_eq!(
            start.tick(true, false, at(3_500), &START),
            HoldAction::Reached(0)
        );
        assert_eq!(
            start.tick(true, false, at(9_000), &START),
            HoldAction::Nothing
        );
        assert_eq!(
            start.tick(false, true, at(9_020), &START),
            HoldAction::ReleasedAfter(0),
            "reported, and Start acts on nothing at its release"
        );
    }

    /// Select held when the pad drops, back three seconds later with Select still down: that
    /// is a reconnection, not a long hold — the hold was measured against the pad that left.
    /// Pinned both ways, because the `tick` arithmetic alone would call it a torque cut.
    #[test]
    fn a_hold_does_not_survive_the_pad_going_away() {
        let t0 = Instant::now();
        let mut select = HoldButton::default();
        assert_eq!(select.tick(true, false, t0, &SELECT), HoldAction::Nothing);

        select.reset();
        assert_eq!(
            select.tick(
                true,
                false,
                t0 + REST_HOLD + Duration::from_secs(1),
                &SELECT
            ),
            HoldAction::Nothing,
            "a hold older than the pad's absence does nothing"
        );

        // Without the reset that same tick is the torque cut — the arithmetic is why the
        // reset exists.
        let mut stale = HoldButton::default();
        assert_eq!(stale.tick(true, false, t0, &SELECT), HoldAction::Nothing);
        assert_eq!(
            stale.tick(
                true,
                false,
                t0 + REST_HOLD + Duration::from_secs(1),
                &SELECT
            ),
            HoldAction::Reached(0)
        );
    }

    /// A pad dropping out after the torque cut does not let the rest of that hold power the
    /// robot off from a fresh start, and its release is not a tap either.
    #[test]
    fn a_pad_dropout_after_a_threshold_spends_the_rest_of_the_hold() {
        let t0 = Instant::now();
        let mut select = HoldButton::default();
        assert_eq!(select.tick(true, false, t0, &SELECT), HoldAction::Nothing);
        assert_eq!(
            select.tick(true, false, t0 + REST_HOLD, &SELECT),
            HoldAction::Reached(0)
        );

        // Pad gone, pad back with Select still down for longer than the whole sequence.
        select.reset();
        let back = t0 + REST_HOLD + Duration::from_secs(3);
        assert_eq!(select.tick(true, false, back, &SELECT), HoldAction::Nothing);
        assert_eq!(
            select.tick(true, false, back + SHUTDOWN_HOLD, &SELECT),
            HoldAction::Nothing,
            "the tail of a spent hold powers nothing off"
        );
        assert_eq!(
            select.tick(false, true, back + SHUTDOWN_HOLD, &SELECT),
            HoldAction::Nothing,
            "and its release is not a tap"
        );

        // Once that release has been seen, Select is an ordinary hold again.
        let t1 = back + Duration::from_secs(10);
        assert_eq!(select.tick(true, false, t1, &SELECT), HoldAction::Nothing);
        assert_eq!(
            select.tick(true, false, t1 + REST_HOLD, &SELECT),
            HoldAction::Reached(0)
        );
    }
}
