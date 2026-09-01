//! Waypoint follower: turn-then-go controller on top of a planned path.
//!
//! Inputs are the **estimated** pose (from the localizer) — that's all
//! the duck has on hardware. The caller applies the returned body-frame
//! velocities using the *true* body heading: motors act in the actual
//! body frame, not the estimated one. If the localizer is wrong, the
//! duck physically drifts off the planned path — that's the honest
//! behaviour we want.
//!
//! The command is a **velocity** (`vx` m/s, `wz` rad/s), not a per-tick
//! displacement. The old displacement API baked the *caller's* dt into
//! the command; the runtime then divided by a fixed nominal dt to get a
//! velocity back, so any tick-length jitter in the maploc worker (map
//! renders, MCL spikes: 80–100 ms against a 20 ms nominal) multiplied
//! straight into commanded speed — up to ~5× lurches while path
//! following. A velocity contract has no dt to disagree about.

#[derive(Debug, Clone, Default)]
pub struct FollowerState {
    waypoints: Vec<(f32, f32)>,
    idx: usize,
    /// Turn-then-go hysteresis: true while translation is enabled.
    /// Enter "go" below `GO_ENTER_RAD` of yaw error, leave above
    /// `GO_EXIT_RAD`. A bipedal gait oscillates yaw every step; a single
    /// threshold made forward motion stutter on/off around it.
    going: bool,
}

/// Yaw-error hysteresis band for the turn-then-go gate (radians).
const GO_ENTER_RAD: f32 = 0.25;
const GO_EXIT_RAD: f32 = 0.45;
/// Proportional yaw gain (1/s): wz = clamp(K · yaw_err, ±yaw_speed).
/// Saturates at full turn rate beyond ~0.4 rad of error.
const YAW_GAIN: f32 = 3.0;

impl FollowerState {
    pub fn new(waypoints: Vec<(f32, f32)>) -> Self {
        Self {
            waypoints,
            idx: 0,
            going: false,
        }
    }
    pub fn empty() -> Self {
        Self::default()
    }
    pub fn done(&self) -> bool {
        self.idx >= self.waypoints.len()
    }
    pub fn current(&self) -> Option<(f32, f32)> {
        self.waypoints.get(self.idx).copied()
    }
    pub fn waypoints(&self) -> &[(f32, f32)] {
        &self.waypoints
    }
}

/// Body-frame velocity command produced by one follower tick.
#[derive(Debug, Clone, Copy, Default)]
pub struct FollowCommand {
    /// Forward velocity (m/s, ≥ 0). Apply along the **true** body yaw
    /// on the actuator side.
    pub vx: f32,
    /// Yaw rate (rad/s, signed).
    pub wz: f32,
}

/// Compute the velocity command toward the current waypoint.
/// `arrive_radius` determines when a waypoint is "reached"; reached
/// waypoints are consumed immediately (no dead tick) so the command
/// always steers at the first waypoint still ahead.
pub fn follow_step(
    state: &mut FollowerState,
    est_pos: (f32, f32),
    est_yaw: f32,
    lin_speed: f32,
    yaw_speed: f32,
    arrive_radius: f32,
) -> FollowCommand {
    // Consume every waypoint already inside the arrive radius, then
    // steer at the next one — the old code returned a zero command for
    // one full tick per waypoint reached.
    let (tx, ty, dist) = loop {
        let Some((tx, ty)) = state.current() else {
            return FollowCommand::default();
        };
        let dx = tx - est_pos.0;
        let dy = ty - est_pos.1;
        let dist = (dx * dx + dy * dy).sqrt();
        if dist < arrive_radius {
            state.idx += 1;
            continue;
        }
        break (tx, ty, dist);
    };
    let _ = dist;

    let target_yaw = (ty - est_pos.1).atan2(tx - est_pos.0);
    let mut yaw_err = target_yaw - est_yaw;
    while yaw_err > std::f32::consts::PI {
        yaw_err -= 2.0 * std::f32::consts::PI;
    }
    while yaw_err < -std::f32::consts::PI {
        yaw_err += 2.0 * std::f32::consts::PI;
    }

    let wz = (YAW_GAIN * yaw_err).clamp(-yaw_speed, yaw_speed);

    // Turn-then-go with hysteresis.
    if state.going {
        if yaw_err.abs() > GO_EXIT_RAD {
            state.going = false;
        }
    } else if yaw_err.abs() < GO_ENTER_RAD {
        state.going = true;
    }
    let vx = if state.going {
        // Scale down smoothly with misalignment instead of a hard gate.
        lin_speed * yaw_err.cos().max(0.0)
    } else {
        0.0
    };
    FollowCommand { vx, wz }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn turns_first_then_drives() {
        // Waypoint due north of (0, 0); duck initially facing east (yaw=0).
        let mut s = FollowerState::new(vec![(0.0, 1.0)]);
        let cmd = follow_step(&mut s, (0.0, 0.0), 0.0, 0.5, 1.0, 0.05);
        // Large yaw error → only turn.
        assert!(
            cmd.vx.abs() < 1e-6,
            "should not translate yet, got {}",
            cmd.vx
        );
        assert!(cmd.wz > 0.0, "should turn CCW toward +y, got {}", cmd.wz);
        assert!(
            cmd.wz <= 1.0 + 1e-6,
            "wz must respect yaw_speed, got {}",
            cmd.wz
        );
    }

    #[test]
    fn translates_when_aimed() {
        let mut s = FollowerState::new(vec![(1.0, 0.0)]);
        let cmd = follow_step(&mut s, (0.0, 0.0), 0.0, 0.5, 1.0, 0.05);
        assert!(cmd.vx > 0.0, "should translate forward; got {}", cmd.vx);
        assert!(
            cmd.vx <= 0.5 + 1e-6,
            "vx must respect lin_speed, got {}",
            cmd.vx
        );
    }

    #[test]
    fn arrives_and_steers_at_next_without_dead_tick() {
        let mut s = FollowerState::new(vec![(0.01, 0.0), (1.0, 0.0)]);
        let cmd = follow_step(&mut s, (0.0, 0.0), 0.0, 0.5, 1.0, 0.05);
        // Inside arrive_radius (0.05) on the first waypoint: consume it
        // AND steer at the second in the same call.
        assert_eq!(s.idx, 1);
        assert!(!s.done());
        assert!(
            cmd.vx > 0.0,
            "should already drive at waypoint 2, got {}",
            cmd.vx
        );
    }

    #[test]
    fn hysteresis_keeps_going_through_small_wobble() {
        let mut s = FollowerState::new(vec![(1.0, 0.0)]);
        // Aligned: enters "go".
        let c1 = follow_step(&mut s, (0.0, 0.0), 0.0, 0.5, 1.0, 0.05);
        assert!(c1.vx > 0.0);
        // Gait wobble past the old single 0.30 threshold but inside the
        // exit band: keep driving.
        let c2 = follow_step(&mut s, (0.0, 0.0), 0.35, 0.5, 1.0, 0.05);
        assert!(c2.vx > 0.0, "0.35 rad wobble must not stop translation");
        // Beyond the exit band: stop and turn.
        let c3 = follow_step(&mut s, (0.0, 0.0), 0.60, 0.5, 1.0, 0.05);
        assert!(c3.vx.abs() < 1e-6, "0.60 rad error must gate translation");
    }

    #[test]
    fn empty_path_commands_zero() {
        let mut s = FollowerState::empty();
        let cmd = follow_step(&mut s, (0.0, 0.0), 0.0, 0.5, 1.0, 0.05);
        assert!(cmd.vx == 0.0 && cmd.wz == 0.0);
        assert!(s.done());
    }
}
