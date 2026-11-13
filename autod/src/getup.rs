//! Getting back up after a fall: is the robot up, still trying, or out of time.
//!
//! `robotd` catches a fall (limp, land, pose back) and hands the robot to the standing policy,
//! which works its way upright. Usually. When it does not, a robot left to it flails on the floor
//! for as long as nobody comes. `robotctl stress` (the `stress-test` branch, `stress/getup.rs`)
//! measured the answer that works: cut the policy so the robot settles into its init pose, give it
//! back, and let it try again from there. This is that watcher, for the duck on its own —
//! `docs/design/autonomous.md` §2.
//!
//! Pure: it is fed the clock and `safety.fallen`, and says what to do. `main.rs` does the cutting
//! and the enabling.
//!
//! **Up means up for a while.** `fallen` clears on one upright sample, and a robot rocking on its
//! way up can read upright and go down again, so up is `fallen` false held for [`Timing::hold`].

/// How long things are allowed to take, seconds.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Timing {
    /// Down this long after the policy took over: cut it, give it back once, then give up.
    pub timeout: f64,
    /// How long the cut lasts: the robot settling into its init pose.
    pub reset: f64,
    /// Upright this long without a relapse is up.
    pub hold: f64,
}

impl Default for Timing {
    /// The stress test's, with the timeout shortened from 7 s: a duck on its own has nobody
    /// timing it, and a quicker reset reads as the duck trying again rather than struggling.
    fn default() -> Self {
        Self {
            timeout: 5.0,
            reset: 2.0,
            hold: 1.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Verdict {
    Waiting,
    /// Up, `seconds` after the watch started.
    GotUp {
        seconds: f64,
        after_reset: bool,
    },
    /// Down at the timeout: cut the policy for [`Timing::reset`], give it back, call
    /// [`Getup::resumed`]. Said once.
    Reset,
    /// Down at the timeout after the reset. Said once; the caller stops trying.
    GaveUp,
}

#[derive(Debug, Clone)]
pub struct Getup {
    pub timing: Timing,
    /// When the current upright stretch began.
    up_since: Option<f64>,
    /// When the current try's clock started: the watch, then the policy's return after the reset.
    try_start: f64,
    after_reset: bool,
    /// Between `Reset` and `resumed`: the caller has the robot.
    resetting: bool,
    done: bool,
}

impl Getup {
    /// From a robot seen down with the policy driving, at time 0.
    pub fn new(timing: Timing) -> Self {
        Self {
            timing,
            up_since: None,
            try_start: 0.0,
            after_reset: false,
            resetting: false,
            done: false,
        }
    }

    /// The policy is back after the reset, at `now`: a second try, with its own timeout.
    pub fn resumed(&mut self, now: f64) {
        self.try_start = now;
        self.after_reset = true;
        self.resetting = false;
        self.up_since = None;
    }

    /// `now` is seconds since the watch started; `fallen` is this tick's.
    pub fn tick(&mut self, now: f64, fallen: bool) -> Verdict {
        if self.done || self.resetting {
            return Verdict::Waiting;
        }
        if fallen {
            self.up_since = None;
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
    fn run(g: &mut Getup, from: f64, to: f64, fallen: impl Fn(f64) -> bool) -> (f64, Verdict) {
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

    #[test]
    fn up_within_the_timeout_is_up_after_its_hold() {
        let mut g = Getup::new(Timing::default());
        let (at, v) = run(&mut g, 0.0, 10.0, |t| t < 3.0);
        assert!(
            matches!(
                v,
                Verdict::GotUp {
                    after_reset: false,
                    ..
                }
            ),
            "{v:?}"
        );
        assert!((at - 4.0).abs() < 0.06, "{at}");
    }

    #[test]
    fn a_relapse_inside_the_hold_restarts_it() {
        let mut g = Getup::new(Timing::default());
        let (at, _) = run(&mut g, 0.0, 10.0, |t| t < 2.0 || (2.5..3.0).contains(&t));
        assert!((at - 4.0).abs() < 0.06, "{at}");
    }

    #[test]
    fn still_down_at_five_seconds_asks_for_a_reset_then_gives_up() {
        let mut g = Getup::new(Timing::default());
        let (at, v) = run(&mut g, 0.0, 20.0, |_| true);
        assert_eq!(v, Verdict::Reset);
        assert!((at - 5.0).abs() < 0.06, "{at}");
        assert_eq!(
            g.tick(6.0, true),
            Verdict::Waiting,
            "quiet while the caller resets"
        );
        g.resumed(7.0);
        assert_eq!(g.tick(11.9, true), Verdict::Waiting);
        assert_eq!(g.tick(12.0, true), Verdict::GaveUp);
        assert_eq!(g.tick(13.0, false), Verdict::Waiting, "said once");
    }

    #[test]
    fn up_after_the_reset_is_marked_as_such() {
        let mut g = Getup::new(Timing::default());
        assert_eq!(run(&mut g, 0.0, 20.0, |_| true).1, Verdict::Reset);
        g.resumed(7.0);
        let (_, v) = run(&mut g, 7.0, 20.0, |t| t < 9.0);
        assert!(
            matches!(
                v,
                Verdict::GotUp {
                    after_reset: true,
                    ..
                }
            ),
            "{v:?}"
        );
    }
}
