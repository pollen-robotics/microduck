//! Map a [`BeatState`] onto the stand policy's pose box.
//!
//! The waveform is the one the idea pins: most negative at phase 0 (a crouch on the
//! onset), zero at energy 0 (nominal stance). A zero-centered sine is 0 at the onset, so
//! it cannot be "crouch on the beat." Cap the **command that would reach the policy**, not
//! only the dance term — leftover BodyPose must not be summed with a bob.
//!
//! In-process overlay and the attended external client share [`overlay_applies`] so a pad,
//! a leftover `pose.active`, a fall, or a skill cannot disagree about who wins.

use std::f64::consts::PI;

use crate::beat::BeatState;

/// `padd`'s BodyPose box, documented on `PoseParams`. Out of this the stand policy is OOD.
pub const BODY_MAX_Z_UP: f64 = 0.010;
pub const BODY_MAX_Z_DOWN: f64 = 0.025;
pub const BODY_MAX_ANGLE: f64 = 0.2618;

/// Crouch amplitude at energy 1 — the crouch side of the box, not the rise.
pub const AZ_MAX: f64 = BODY_MAX_Z_DOWN;

/// What the stand network should see this tick, and the extra head/mouth the mouth servo
/// (not in any policy) can take.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Overlay {
    pub z: f64,
    pub roll: f64,
    pub pitch: f64,
    pub head: [f64; 4],
    pub mouth: f64,
}

/// Beat encoding the dance gait was trained on. Not for the alpha stand net.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct GaitCommand {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

impl Default for Overlay {
    fn default() -> Self {
        Self {
            z: 0.0,
            roll: 0.0,
            pitch: 0.0,
            head: [0.0; 4],
            mouth: 0.0,
        }
    }
}

/// Crouch at beat onset; energy 0 is z = 0. Roll/pitch stay 0 until a later pass adds sway.
pub fn map_body(phase: f32, energy: f32) -> Overlay {
    let energy = (energy as f64).clamp(0.0, 1.0);
    let phase = phase as f64;
    let az = AZ_MAX * energy;
    let z = -az * 0.5 * (1.0 + (2.0 * PI * phase).cos());
    Overlay {
        z: z.clamp(-BODY_MAX_Z_DOWN, BODY_MAX_Z_UP),
        roll: 0.0,
        pitch: 0.0,
        head: [0.0; 4],
        mouth: 0.0,
    }
}

pub fn map_beat(beat: &BeatState) -> Overlay {
    map_body(beat.phase, beat.energy)
}

/// `body_x=sin(2πφ)`, `body_y=cos(2πφ)`, `body_yaw=energy`. Energy 0 is all zeros,
/// matching the training idle. Do not feed this to alpha stand weights.
pub fn map_gait_command(phase: f32, energy: f32) -> GaitCommand {
    let energy = (energy as f64).clamp(0.0, 1.0);
    if energy == 0.0 {
        return GaitCommand::default();
    }
    let two_pi = 2.0 * PI * phase as f64;
    GaitCommand {
        x: two_pi.sin(),
        y: two_pi.cos(),
        yaw: energy,
    }
}

pub fn map_beat_gait(beat: &BeatState) -> GaitCommand {
    map_gait_command(beat.phase, beat.energy)
}

/// Cap a body command (dance term, or dance plus anything else that reached this tick)
/// so it cannot leave the trained box.
pub fn clamp_body(z: f64, roll: f64, pitch: f64) -> (f64, f64, f64) {
    (
        z.clamp(-BODY_MAX_Z_DOWN, BODY_MAX_Z_UP),
        roll.clamp(-BODY_MAX_ANGLE, BODY_MAX_ANGLE),
        pitch.clamp(-BODY_MAX_ANGLE, BODY_MAX_ANGLE),
    )
}

/// Everything that can refuse the overlay. In-process: omit, leave shared slots alone.
/// External (phases 0–1): send `active: false` and nominal head/mouth.
#[derive(Debug, Clone, Copy, Default)]
pub struct OverlayGate {
    pub opt_in: bool,
    pub walk_mode: bool,
    pub enabled: bool,
    pub has_standing: bool,
    /// Locomoting hop-dance net is loaded. Overlay can apply with this *or*
    /// [`Self::has_standing`] — crouch needs standing; travel-hop needs dance.
    pub has_dance: bool,
    pub fallen: bool,
    pub limp_fall: bool,
    pub skill_busy: bool,
    pub self_audio: bool,
    pub tracker_quarantined: bool,
    pub locked: bool,
    pub t_fresh: bool,
    /// Twist age inside the deadman window — any writer, including zeros.
    pub twist_fresh: bool,
    pub pose_active: bool,
}

impl OverlayGate {
    pub fn overlay_applies(self) -> bool {
        self.opt_in
            && self.walk_mode
            && self.enabled
            && (self.has_standing || self.has_dance)
            && self.locked
            && self.t_fresh
            && !self.fallen
            && !self.limp_fall
            && !self.skill_busy
            && !self.self_audio
            && !self.tracker_quarantined
            && !self.twist_fresh
            && !self.pose_active
    }

    /// Attended client: clear the shared slots. Broader than overlay_applies — a leftover
    /// crouch after a kill must still be released.
    pub fn external_must_release(self) -> bool {
        !self.locked
            || !self.t_fresh
            || self.tracker_quarantined
            || self.fallen
            || self.limp_fall
            || self.self_audio
            || self.skill_busy
    }
}

/// How old a BeatState `t` may be before the mapper treats it as a dead capture.
pub fn stale_after(bpm: f32) -> std::time::Duration {
    let beat_s = if bpm >= 60.0 {
        60.0 / bpm as f64
    } else {
        1.0
    };
    std::time::Duration::from_secs_f64(beat_s.max(0.4))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn z_is_most_negative_at_phase_zero_and_zero_at_energy_zero() {
        let at0 = map_body(0.0, 1.0);
        let at_half = map_body(0.5, 1.0);
        assert!(at0.z < at_half.z, "onset must be the crouch, got {} vs {}", at0.z, at_half.z);
        assert!(at0.z < 0.0);
        assert!(at0.z >= -BODY_MAX_Z_DOWN - 1e-9);
        // Rise side of this waveform is 0, not +1 cm.
        assert!(at_half.z.abs() < 1e-9, "phase 0.5 should be z=0, got {}", at_half.z);

        let silent = map_body(0.0, 0.0);
        assert_eq!(silent.z, 0.0);
        assert_eq!(silent.roll, 0.0);
        assert_eq!(silent.pitch, 0.0);
    }

    #[test]
    fn gait_command_matches_training_slots() {
        let silent = map_gait_command(0.0, 0.0);
        assert_eq!(silent, GaitCommand::default());

        let onset = map_gait_command(0.0, 1.0);
        assert!(onset.x.abs() < 1e-9, "sin(0) = 0, got {}", onset.x);
        assert!((onset.y - 1.0).abs() < 1e-9, "cos(0) = 1, got {}", onset.y);
        assert_eq!(onset.yaw, 1.0);

        let quarter = map_gait_command(0.25, 0.5);
        assert!((quarter.x - 1.0).abs() < 1e-9, "sin(π/2) = 1, got {}", quarter.x);
        assert!(quarter.y.abs() < 1e-9, "cos(π/2) = 0, got {}", quarter.y);
        assert_eq!(quarter.yaw, 0.5);
    }

    #[test]
    fn mapper_output_never_leaves_the_pose_box() {
        for energy in [0.0f32, 0.5, 1.0, 2.0] {
            for k in 0..32 {
                let phase = k as f32 / 32.0;
                let o = map_body(phase, energy);
                let (z, roll, pitch) = clamp_body(o.z, o.roll, o.pitch);
                assert!(z >= -BODY_MAX_Z_DOWN - 1e-9 && z <= BODY_MAX_Z_UP + 1e-9, "z={z}");
                assert!(roll.abs() <= BODY_MAX_ANGLE + 1e-9);
                assert!(pitch.abs() <= BODY_MAX_ANGLE + 1e-9);
                assert!(o.z <= 0.0 + 1e-9, "rise side must not go above nominal, z={}", o.z);
            }
        }
    }

    fn live() -> OverlayGate {
        OverlayGate {
            opt_in: true,
            walk_mode: true,
            enabled: true,
            has_standing: true,
            locked: true,
            t_fresh: true,
            ..OverlayGate::default()
        }
    }

    #[test]
    fn in_process_omits_on_fresh_twist_or_pose_active() {
        assert!(live().overlay_applies());
        let mut g = live();
        g.twist_fresh = true;
        assert!(!g.overlay_applies(), "a publishing pad must win");
        g = live();
        g.pose_active = true;
        assert!(!g.overlay_applies(), "leftover BodyPose must win");
        // The required regression: stale twist + pose.active — overlay off, leftover
        // crouch not summed with the dance term.
        g.twist_fresh = false;
        g.pose_active = true;
        assert!(!g.overlay_applies());
    }

    #[test]
    fn in_process_omits_on_fall_skill_self_audio_stale_lock() {
        for mut g in [
            OverlayGate { fallen: true, ..live() },
            OverlayGate { limp_fall: true, ..live() },
            OverlayGate { skill_busy: true, ..live() },
            OverlayGate { self_audio: true, ..live() },
            OverlayGate { tracker_quarantined: true, ..live() },
            OverlayGate { locked: false, ..live() },
            OverlayGate { t_fresh: false, ..live() },
            OverlayGate { opt_in: false, ..live() },
            OverlayGate { walk_mode: false, ..live() },
            OverlayGate { has_standing: false, ..live() },
            OverlayGate { enabled: false, ..live() },
        ] {
            assert!(!g.overlay_applies(), "{g:?}");
            g.pose_active = false;
            assert!(!g.overlay_applies());
        }
    }

    #[test]
    fn overlay_applies_with_dance_net_even_without_standing() {
        let mut g = live();
        g.has_standing = false;
        g.has_dance = true;
        assert!(g.overlay_applies());
        g.has_dance = false;
        assert!(!g.overlay_applies());
    }

    #[test]
    fn external_releases_on_lost_lock_stale_quarantine_fallen() {
        assert!(!live().external_must_release());
        assert!(OverlayGate { locked: false, ..live() }.external_must_release());
        assert!(OverlayGate { t_fresh: false, ..live() }.external_must_release());
        assert!(OverlayGate { tracker_quarantined: true, ..live() }.external_must_release());
        assert!(OverlayGate { fallen: true, ..live() }.external_must_release());
        assert!(OverlayGate { self_audio: true, ..live() }.external_must_release());
    }
}
