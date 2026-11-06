//! The get-up: from the moment the policy is back on a robot on the floor, is it up, still
//! trying, out of time, or did it never go down.
//!
//! Pure: it is fed the clock and `safety.fallen`, and says what happened. The driver does the
//! cutting and the enabling; this decides when.
//!
//! **Up means up for a while.** `fallen` clears on one upright sample, and a robot rocking on its
//! way up can read upright and go down again, so up is `fallen` false held for [`Timing::hold`].
//! The get-up time is to the start of that hold. **No frame is not up**: a `robotd` that stopped
//! answering mid-get-up has not got the robot up.

/// How long things are allowed to take, seconds.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Timing {
    /// Down this long after the policy came back: cut it and give it back once, then give up.
    pub timeout: f64,
    /// How long the cut between the two tries lasts. The driver's to wait; here for the record.
    pub reset: f64,
    /// Upright this long without a relapse is up.
    pub hold: f64,
    /// A provoked fall not seen within this long of the policy coming back did not happen.
    pub no_fall: f64,
}

impl Default for Timing {
    fn default() -> Self {
        Self {
            timeout: 7.0,
            reset: 2.0,
            hold: 1.0,
            no_fall: 2.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Verdict {
    Waiting,
    /// The cut did not put the robot down.
    NoFall,
    /// Up, `seconds` after the first enable — so a get-up that needed the reset reads as long as
    /// it was.
    GotUp {
        seconds: f64,
        after_reset: bool,
    },
    /// Down at the timeout: cut the policy for [`Timing::reset`], give it back, call
    /// [`Getup::resumed`]. Said once.
    Reset,
    /// Down at the timeout after the reset. Said once; the driver stops for good.
    GaveUp,
}

#[derive(Debug, Clone)]
pub struct Getup {
    timing: Timing,
    /// Whether the robot has been seen down. A provoked fall starts unseen: the robot is still
    /// toppling when the policy comes back, and up before it has been down is not a get-up.
    seen_down: bool,
    /// When the current upright stretch began.
    up_since: Option<f64>,
    /// When the current try's clock started: the first enable, then the one after the reset.
    try_start: f64,
    after_reset: bool,
    /// Between `Reset` and `resumed`: the driver has the robot.
    resetting: bool,
    done: bool,
}

impl Getup {
    /// From the enable after a provoked cut, at time 0.
    pub fn provoked(timing: Timing) -> Self {
        Self {
            timing,
            seen_down: false,
            up_since: None,
            try_start: 0.0,
            after_reset: false,
            resetting: false,
            done: false,
        }
    }

    /// From the first frame of a fall nobody asked for, at time 0. It is down already, so it
    /// cannot be a no-fall.
    pub fn unprovoked(timing: Timing) -> Self {
        Self {
            seen_down: true,
            ..Self::provoked(timing)
        }
    }

    /// The policy is back after the reset, at `now`: a second try, with its own timeout.
    pub fn resumed(&mut self, now: f64) {
        self.try_start = now;
        self.after_reset = true;
        self.resetting = false;
        self.up_since = None;
    }

    /// `now` is seconds since construction; `fallen` is the latest frame's, `None` for no frame.
    pub fn tick(&mut self, now: f64, fallen: Option<bool>) -> Verdict {
        if self.done || self.resetting {
            return Verdict::Waiting;
        }
        if fallen.unwrap_or(true) {
            self.seen_down = true;
            self.up_since = None;
        } else if !self.seen_down {
            if now >= self.timing.no_fall - 1e-9 {
                self.done = true;
                return Verdict::NoFall;
            }
            return Verdict::Waiting;
        } else {
            let since = *self.up_since.get_or_insert(now);
            if now - since >= self.timing.hold - 1e-9 {
                self.done = true;
                return Verdict::GotUp {
                    seconds: since,
                    after_reset: self.after_reset,
                };
            }
        }
        // Upright when the time runs out is given its hold; down is out of time.
        if self.up_since.is_none() && now - self.try_start >= self.timing.timeout - 1e-9 {
            if self.after_reset {
                self.done = true;
                return Verdict::GaveUp;
            }
            self.resetting = true;
            return Verdict::Reset;
        }
        Verdict::Waiting
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Tick `g` every 50 ms over `[from, to)` with `fallen(t)`, returning the first verdict that
    /// is not `Waiting`, and when.
    fn run(
        g: &mut Getup,
        from: f64,
        to: f64,
        fallen: impl Fn(f64) -> Option<bool>,
    ) -> (f64, Verdict) {
        let mut t = from;
        while t < to - 1e-9 {
            let v = g.tick(t, fallen(t));
            if v != Verdict::Waiting {
                return (t, v);
            }
            t += 0.05;
        }
        (to, Verdict::Waiting)
    }

    fn got_up(v: Verdict) -> (f64, bool) {
        match v {
            Verdict::GotUp {
                seconds,
                after_reset,
            } => (seconds, after_reset),
            other => panic!("not up: {other:?}"),
        }
    }

    #[test]
    fn up_within_the_timeout_reports_the_time_to_the_start_of_the_hold() {
        let mut g = Getup::provoked(Timing::default());
        let (at, v) = run(&mut g, 0.0, 10.0, |t| Some(t > 0.1 && t < 3.0));
        let (seconds, after_reset) = got_up(v);
        assert!((seconds - 3.0).abs() < 0.06, "{seconds}");
        assert!((at - 4.0).abs() < 0.06, "{at}");
        assert!(!after_reset);
    }

    #[test]
    fn a_relapse_inside_the_hold_restarts_it() {
        let mut g = Getup::provoked(Timing::default());
        let fallen = |t: f64| Some(t > 0.1 && (t < 2.0 || (2.5..3.0).contains(&t)));
        let (at, v) = run(&mut g, 0.0, 10.0, fallen);
        let (seconds, _) = got_up(v);
        assert!((seconds - 3.0).abs() < 0.06, "{seconds}");
        assert!((at - 4.0).abs() < 0.06, "{at}");
    }

    #[test]
    fn still_down_at_seven_seconds_asks_for_a_reset_then_gives_up() {
        let mut g = Getup::provoked(Timing::default());
        let (at, v) = run(&mut g, 0.0, 20.0, |_| Some(true));
        assert_eq!(v, Verdict::Reset);
        assert!((at - 7.0).abs() < 0.06, "{at}");
        g.resumed(9.0);
        assert_eq!(g.tick(15.9, Some(true)), Verdict::Waiting);
        assert_eq!(g.tick(16.0, Some(true)), Verdict::GaveUp);
    }

    #[test]
    fn up_after_the_reset_is_marked_as_such() {
        let mut g = Getup::provoked(Timing::default());
        assert_eq!(run(&mut g, 0.0, 20.0, |_| Some(true)).1, Verdict::Reset);
        g.resumed(9.0);
        let (_, v) = run(&mut g, 9.0, 20.0, |t| Some(t < 11.0));
        let (seconds, after_reset) = got_up(v);
        assert!(after_reset);
        // From the first enable, so a reset get-up reads as the long one it was.
        assert!((seconds - 11.0).abs() < 0.06, "{seconds}");
    }

    #[test]
    fn never_down_within_two_seconds_is_no_fall() {
        let mut g = Getup::provoked(Timing::default());
        assert_eq!(g.tick(1.9, Some(false)), Verdict::Waiting);
        assert_eq!(g.tick(2.0, Some(false)), Verdict::NoFall);
    }

    #[test]
    fn an_unprovoked_fall_uses_the_same_rules() {
        let mut g = Getup::unprovoked(Timing::default());
        let (at, v) = run(&mut g, 0.0, 10.0, |_| Some(false));
        let (seconds, _) = got_up(v);
        assert!(seconds.abs() < 1e-9, "{seconds}");
        assert!((at - 1.0).abs() < 0.06, "{at}");

        let mut g = Getup::unprovoked(Timing::default());
        assert_eq!(run(&mut g, 0.0, 20.0, |_| Some(true)).1, Verdict::Reset);
    }

    #[test]
    fn no_frame_counts_as_down() {
        let mut g = Getup::provoked(Timing::default());
        let fallen = |t: f64| if t < 1.0 { Some(t > 0.5) } else { None };
        let (at, v) = run(&mut g, 0.0, 20.0, fallen);
        assert_eq!(v, Verdict::Reset);
        assert!((at - 7.0).abs() < 0.06, "{at}");
    }
}
