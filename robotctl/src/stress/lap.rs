//! The lap: what the robot is told to do, in order, the same on every robot.
//!
//! Two runs compare lap for lap only if they ran the same lap, so the lap is data with a
//! version, and the version goes in `meta.json`. Change a length or a step and bump it.
//!
//! It is symmetric — as far back as forward, as much left as right — so the robot stays near
//! where it started without being steered there.

/// The lap this build runs. `scripts/endurance-plot.py` refuses to overlay runs that differ.
pub const VERSION: &str = "stress-v1";

/// What one step asks of the robot.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Action {
    /// `robot.move` at this twist for the step's length.
    Walk { vx: f64, vyaw: f64 },
    /// Trip (or cut) the robot while walking at `vx`, and wait for the get-up. The walk
    /// leading into it is the step before; this one lasts as long as the get-up does.
    Fall { vx: f64 },
    /// `sit_toggle` into the seat, and stay there.
    Sit,
    /// `sit_toggle` out of the seat, and stand still.
    Stand,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Step {
    /// `"A1.fwd"`: block, repeat and what — what the events and the `step` column carry.
    pub name: &'static str,
    pub action: Action,
    /// How long the step lasts, from its start. Zero for a fall: it ends with its get-up.
    pub seconds: f64,
}

const A: [[&str; 4]; 3] = [
    ["A1.fwd", "A1.fall_fwd", "A1.back", "A1.fall_back"],
    ["A2.fwd", "A2.fall_fwd", "A2.back", "A2.fall_back"],
    ["A3.fwd", "A3.fall_fwd", "A3.back", "A3.fall_back"],
];
const C: [[&str; 2]; 3] = [
    ["C1.sit", "C1.stand"],
    ["C2.sit", "C2.stand"],
    ["C3.sit", "C3.stand"],
];

/// The lap, expanded: A ×3 (2 s forward, fall; 2 s back, fall), B (3 s forward, 3 s back, 2 s
/// left, 2 s right), C ×3 (3 s seated, 2 s standing).
pub fn lap(speed: f64, vyaw: f64) -> Vec<Step> {
    let step = |name, action, seconds| Step {
        name,
        action,
        seconds,
    };
    let walk = |vx, vyaw| Action::Walk { vx, vyaw };
    let mut steps = Vec::with_capacity(22);
    for [fwd, fall_fwd, back, fall_back] in A {
        steps.push(step(fwd, walk(speed, 0.0), 2.0));
        steps.push(step(fall_fwd, Action::Fall { vx: speed }, 0.0));
        steps.push(step(back, walk(-speed, 0.0), 2.0));
        steps.push(step(fall_back, Action::Fall { vx: -speed }, 0.0));
    }
    // Back before turning: a turn between the two would send the walk back further out.
    steps.push(step("B.fwd", walk(speed, 0.0), 3.0));
    steps.push(step("B.back", walk(-speed, 0.0), 3.0));
    steps.push(step("B.left", walk(0.0, vyaw), 2.0));
    steps.push(step("B.right", walk(0.0, -vyaw), 2.0));
    for [sit, stand] in C {
        steps.push(step(sit, Action::Sit, 3.0));
        steps.push(step(stand, Action::Stand, 2.0));
    }
    steps
}

/// Late by less than this is jitter, and the next step keeps the old anchor; later is an
/// overrun, and the schedule starts again from now.
const SLACK: f64 = 0.2;

/// When each timed step ends. Each one starts where the last one was due to end, not when the
/// driver got round to it, so the lap's timeline is the same on every robot; only a get-up
/// (which has no length of its own) or an overrun moves it on.
#[derive(Debug, Default, Clone)]
pub struct Schedule {
    anchor: Option<f64>,
}

impl Schedule {
    /// The deadline of a step of `seconds` beginning now.
    pub fn deadline(&mut self, seconds: f64, now: f64) -> f64 {
        let start = match self.anchor {
            Some(anchor) if anchor >= now - SLACK => anchor,
            _ => now,
        };
        let deadline = start + seconds;
        self.anchor = Some(deadline);
        deadline
    }

    /// A get-up just ended: the next step starts when it begins.
    pub fn unanchor(&mut self) {
        self.anchor = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn walked(steps: &[Step]) -> (f64, f64) {
        steps.iter().fold((0.0, 0.0), |(x, yaw), s| match s.action {
            Action::Walk { vx, vyaw } => (x + vx * s.seconds, yaw + vyaw * s.seconds),
            _ => (x, yaw),
        })
    }

    /// Where the lap takes the robot on the floor, steps integrated in the world frame: the end
    /// point and the furthest it gets from the start.
    fn on_the_floor(steps: &[Step]) -> ((f64, f64), f64) {
        let (mut x, mut y, mut yaw, mut far) = (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);
        for s in steps {
            if let Action::Walk { vx, vyaw } = s.action {
                // Turns and translations never share a step, so each integrates exactly.
                x += vx * s.seconds * yaw.cos();
                y += vx * s.seconds * yaw.sin();
                yaw += vyaw * s.seconds;
                far = far.max(x.hypot(y));
            }
        }
        ((x, y), far)
    }

    #[test]
    fn a_lap_ends_where_it_started_on_the_floor() {
        let ((x, y), far) = on_the_floor(&lap(0.3, 1.5));
        assert!(x.hypot(y) < 1e-9, "ends {x:.2}, {y:.2} from the start");
        assert!((far - 0.9).abs() < 1e-9, "goes {far:.2} m out");
    }

    #[test]
    fn a_lap_has_six_falls_three_forward_three_back() {
        let steps = lap(0.3, 1.5);
        let falls: Vec<f64> = steps
            .iter()
            .filter_map(|s| match s.action {
                Action::Fall { vx } => Some(vx),
                _ => None,
            })
            .collect();
        assert_eq!(falls, [0.3, -0.3, 0.3, -0.3, 0.3, -0.3]);
    }

    #[test]
    fn a_lap_walks_as_far_back_as_forward_and_turns_cancel() {
        let (x, yaw) = walked(&lap(0.3, 1.5));
        assert!(x.abs() < 1e-9, "{x}");
        assert!(yaw.abs() < 1e-9, "{yaw}");
    }

    #[test]
    fn the_lap_is_the_spec_s_table() {
        let steps = lap(0.3, 1.5);
        let row = |i: usize| (steps[i].name, steps[i].action, steps[i].seconds);
        assert_eq!(steps.len(), 22);
        assert_eq!(row(0), ("A1.fwd", Action::Walk { vx: 0.3, vyaw: 0.0 }, 2.0));
        assert_eq!(row(1), ("A1.fall_fwd", Action::Fall { vx: 0.3 }, 0.0));
        assert_eq!(
            row(2),
            (
                "A1.back",
                Action::Walk {
                    vx: -0.3,
                    vyaw: 0.0
                },
                2.0
            )
        );
        assert_eq!(row(3), ("A1.fall_back", Action::Fall { vx: -0.3 }, 0.0));
        assert_eq!(row(12), ("B.fwd", Action::Walk { vx: 0.3, vyaw: 0.0 }, 3.0));
        assert_eq!(
            row(13),
            (
                "B.back",
                Action::Walk {
                    vx: -0.3,
                    vyaw: 0.0
                },
                3.0
            )
        );
        assert_eq!(
            row(14),
            ("B.left", Action::Walk { vx: 0.0, vyaw: 1.5 }, 2.0)
        );
        assert_eq!(
            row(15),
            (
                "B.right",
                Action::Walk {
                    vx: 0.0,
                    vyaw: -1.5
                },
                2.0
            )
        );
        assert_eq!(row(16), ("C1.sit", Action::Sit, 3.0));
        assert_eq!(row(17), ("C1.stand", Action::Stand, 2.0));
        assert_eq!(steps[21].name, "C3.stand");
    }

    #[test]
    fn the_schedule_does_not_slip_when_a_step_is_cut() {
        // Every step comes back 30 ms late — a tick of jitter, or a guard that ended its step
        // early and waited out the slot. The lap still ends when its lengths say.
        let mut s = Schedule::default();
        let mut now = 0.0;
        let mut deadline: f64 = 0.0;
        for seconds in [2.0, 2.0, 3.0, 3.0, 2.0, 2.0] {
            deadline = s.deadline(seconds, now);
            now = deadline + 0.03;
        }
        assert!((deadline - 14.0).abs() < 1e-9, "{deadline}");
    }

    #[test]
    fn a_get_up_moves_the_schedule_on() {
        let mut s = Schedule::default();
        assert_eq!(s.deadline(2.0, 0.0), 2.0);
        // The fall after it took 4.5 s to get up from: the next step starts then.
        s.unanchor();
        assert_eq!(s.deadline(2.0, 6.5), 8.5);
        // A step that overran its slot (a fall nobody asked for, mid-walk) does too.
        assert_eq!(s.deadline(2.0, 12.0), 14.0);
    }
}
