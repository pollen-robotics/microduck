//! SubmapManager — owns the list of frozen submaps + the active one,
//! and decides when to switch.
//!
//! Switching policy (initial cut, tunable):
//!   * close + open new submap when the active one has been live for
//!     ≥ `max_age_s`,
//!   * OR when the tracked pose has moved ≥ `max_travel_m` from the
//!     submap's anchor.
//!
//! On switch the new submap's anchor pose is the *current tracked
//! pose at switch time*, so there's no positional discontinuity in
//! the rendered global map (frozen submaps and the new one share the
//! same world frame).

use crate::grid::GridConfig;
use crate::submap::{Pose2, Submap};

/// What one manager tick did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TickOutcome {
    /// Nothing changed.
    Idle,
    /// A new submap opened (the previous one froze, unless this is the
    /// bootstrap open).
    Opened,
    /// The current submap was EMPTY when it hit a switch condition, so it
    /// was re-anchored at the current pose instead of frozen: an empty
    /// submap in the frozen list is a dead node the pose graph drags
    /// around, and its stale anchor would misplace whatever ink eventually
    /// arrives. A long stand (or a scanless walk) re-anchors; it does not
    /// pile up husks.
    Reanchored,
}

#[derive(Debug, Clone, Copy)]
pub struct SubmapManagerConfig {
    /// Submap dimensions (centred on the anchor; defines the local grid).
    pub grid: GridConfig,
    /// Max wall-time before forcing a switch.
    pub max_age_s: f32,
    /// Max in-submap travel before forcing a switch.
    pub max_travel_m: f32,
    /// The age rule only fires once the robot has moved at least this far
    /// from the anchor: the rule exists to bound intra-submap odometry
    /// drift, and a robot standing still accrues none — age-freezing it
    /// just mints identical same-viewpoint submaps (a fresh one every
    /// 8 s on the first field test's standing robot).
    pub min_travel_for_age_m: f32,
}

impl Default for SubmapManagerConfig {
    fn default() -> Self {
        // 4 m × 4 m local grid at 5 cm cells, centred on the anchor.
        let grid = GridConfig {
            x_range: (-2.0, 2.0),
            y_range: (-2.0, 2.0),
            cell: 0.05,
        };
        Self {
            grid,
            max_age_s: 20.0,
            max_travel_m: 2.0,
            min_travel_for_age_m: 0.15,
        }
    }
}

pub struct SubmapManager {
    cfg: SubmapManagerConfig,
    frozen: Vec<Submap>,
    current: Option<Submap>,
    /// Wall-time (session-relative seconds) at which the current
    /// submap was created. `None` while there is no current submap.
    current_started_s: Option<f32>,
}

impl SubmapManager {
    pub fn new(cfg: SubmapManagerConfig) -> Self {
        Self {
            cfg,
            frozen: Vec::new(),
            current: None,
            current_started_s: None,
        }
    }

    /// Restore a manager from a previously-saved state. `current_started_s`
    /// starts `None` and is re-armed on the first `tick`, so the restored
    /// submap gets a full `max_age_s` from resume time instead of either
    /// switching immediately or (the old bug) never aging at all.
    pub fn from_parts(
        cfg: SubmapManagerConfig,
        frozen: Vec<Submap>,
        current: Option<Submap>,
    ) -> Self {
        Self {
            cfg,
            frozen,
            current,
            current_started_s: None,
        }
    }

    pub fn config(&self) -> SubmapManagerConfig {
        self.cfg
    }

    /// Number of *frozen* submaps (excluding the active one).
    pub fn n_frozen(&self) -> usize {
        self.frozen.len()
    }

    /// Total submaps, including the active one if any.
    pub fn n_total(&self) -> usize {
        self.frozen.len() + if self.current.is_some() { 1 } else { 0 }
    }

    pub fn frozen(&self) -> &[Submap] {
        &self.frozen
    }
    pub fn frozen_mut(&mut self) -> &mut [Submap] {
        &mut self.frozen
    }
    pub fn current(&self) -> Option<&Submap> {
        self.current.as_ref()
    }
    pub fn current_mut(&mut self) -> Option<&mut Submap> {
        self.current.as_mut()
    }

    /// Iterate frozen + current as `&Submap` (cheap to call repeatedly).
    pub fn all(&self) -> impl Iterator<Item = &Submap> {
        self.frozen.iter().chain(self.current.iter())
    }

    /// Update the manager. Call on every tick; `tracked_pose` is the
    /// robot's current world pose (from odom-driven tracking), and
    /// `now_s` is the session-relative wall-time.
    pub fn tick(&mut self, now_s: f32, tracked_pose: Pose2) -> TickOutcome {
        // Bootstrap the first submap on the very first tick.
        if self.current.is_none() {
            self.start_new(tracked_pose, now_s);
            return TickOutcome::Opened;
        }
        // Session restore leaves `current_started_s = None`; arm the age
        // clock now. Without this, `should_switch` computes age = 0
        // forever and a restored submap can only close via the travel
        // rule — a loitering duck would grow it unboundedly.
        if self.current_started_s.is_none() {
            self.current_started_s = Some(now_s);
        }
        if self.should_switch(now_s, tracked_pose) {
            if !self.current.as_ref().is_some_and(Submap::has_content) {
                // Empty: nothing worth freezing — see [`TickOutcome::Reanchored`].
                self.start_new(tracked_pose, now_s);
                return TickOutcome::Reanchored;
            }
            // Move current → frozen, start fresh at the new anchor.
            let old = self.current.take().unwrap();
            self.frozen.push(old);
            self.start_new(tracked_pose, now_s);
            return TickOutcome::Opened;
        }
        TickOutcome::Idle
    }

    fn should_switch(&self, now_s: f32, tracked_pose: Pose2) -> bool {
        let cur = match &self.current {
            Some(c) => c,
            None => return false,
        };
        let (ax, ay, _) = cur.anchor_pose();
        let dx = tracked_pose.0 - ax;
        let dy = tracked_pose.1 - ay;
        let travel = (dx * dx + dy * dy).sqrt();
        let age = now_s - self.current_started_s.unwrap_or(now_s);
        if age >= self.cfg.max_age_s && travel >= self.cfg.min_travel_for_age_m {
            return true;
        }
        travel >= self.cfg.max_travel_m
    }

    fn start_new(&mut self, anchor_pose: Pose2, now_s: f32) {
        self.current = Some(Submap::new_at(anchor_pose, self.cfg.grid));
        self.current_started_s = Some(now_s);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sc() -> crate::submap::Scan {
        crate::submap::Scan::from_polar(&[0.0], &[1.0], (0.0, 0.0), 0.0)
    }

    /// A submap with nothing in it never freezes: hitting a switch
    /// condition re-anchors it at the current pose instead, so long stands
    /// and scanless walks cannot pile husk submaps into the pose graph.
    #[test]
    fn empty_submaps_reanchor_instead_of_freezing() {
        let cfg = SubmapManagerConfig {
            max_age_s: 5.0,
            max_travel_m: 1000.0,
            ..SubmapManagerConfig::default()
        };
        let mut mgr = SubmapManager::new(cfg);
        assert_eq!(mgr.tick(0.0, (0.0, 0.0, 0.0)), TickOutcome::Opened);
        assert_eq!(mgr.tick(6.0, (1.0, 0.0, 0.0)), TickOutcome::Reanchored);
        assert_eq!(mgr.n_frozen(), 0, "an empty submap must not freeze");
        assert_eq!(mgr.current().unwrap().anchor_pose().0, 1.0);
        // With content (and a step away from the anchor, arming the age
        // rule), the same condition freezes.
        mgr.current_mut()
            .unwrap()
            .integrate_scan((1.0, 0.0, 0.0), &sc());
        assert_eq!(mgr.tick(12.0, (1.2, 0.0, 0.0)), TickOutcome::Opened);
        assert_eq!(mgr.n_frozen(), 1);
    }

    /// M2 regression: after `from_parts` (session restore) the age
    /// clock must re-arm on the first tick — the old code left it
    /// `None` forever, so a restored submap could never age out.
    #[test]
    fn restored_submap_ages_out() {
        let cfg = SubmapManagerConfig {
            max_age_s: 10.0,
            max_travel_m: 1000.0, // age rule only
            ..SubmapManagerConfig::default()
        };
        let current = Some(Submap::new_at((0.0, 0.0, 0.0), cfg.grid));
        let mut mgr = SubmapManager::from_parts(cfg, Vec::new(), current);
        // Content, so aging freezes rather than re-anchors.
        mgr.current_mut()
            .unwrap()
            .integrate_scan((0.0, 0.0, 0.0), &sc());
        // First tick (resume at t=100): arms the clock, no switch. Poses sit
        // slightly off the anchor so the age rule is armed.
        assert_eq!(mgr.tick(100.0, (0.2, 0.0, 0.0)), TickOutcome::Idle);
        assert_eq!(mgr.n_frozen(), 0);
        // Just under the age limit: still no switch.
        assert_eq!(mgr.tick(109.0, (0.2, 0.0, 0.0)), TickOutcome::Idle);
        // Past the age limit: the restored submap must freeze.
        assert_eq!(
            mgr.tick(110.5, (0.2, 0.0, 0.0)),
            TickOutcome::Opened,
            "restored submap never aged out"
        );
        assert_eq!(mgr.n_frozen(), 1);
    }

    /// A robot standing at its anchor never age-freezes: there is no drift
    /// to bound, and freezing would mint identical submaps every few
    /// seconds for as long as it stands.
    #[test]
    fn standing_still_does_not_age_freeze() {
        let cfg = SubmapManagerConfig {
            max_age_s: 5.0,
            max_travel_m: 1000.0,
            ..SubmapManagerConfig::default()
        };
        let mut mgr = SubmapManager::new(cfg);
        mgr.tick(0.0, (0.0, 0.0, 0.0));
        mgr.current_mut()
            .unwrap()
            .integrate_scan((0.0, 0.0, 0.0), &sc());
        for t in 1..60 {
            assert_eq!(
                mgr.tick(t as f32, (0.0, 0.0, 0.0)),
                TickOutcome::Idle,
                "a standing robot minted a submap at t={t}"
            );
        }
        assert_eq!(mgr.n_total(), 1);
    }

    #[test]
    fn first_tick_creates_a_submap() {
        let mut mgr = SubmapManager::new(SubmapManagerConfig::default());
        assert_eq!(mgr.n_total(), 0);
        let opened = mgr.tick(0.0, (0.0, 0.0, 0.0));
        assert_eq!(opened, TickOutcome::Opened);
        assert_eq!(mgr.n_total(), 1);
        assert_eq!(mgr.n_frozen(), 0);
    }

    #[test]
    fn travel_triggers_switch() {
        let cfg = SubmapManagerConfig {
            max_travel_m: 1.0,
            max_age_s: 1000.0,
            ..SubmapManagerConfig::default()
        };
        let mut mgr = SubmapManager::new(cfg);
        mgr.tick(0.0, (0.0, 0.0, 0.0));
        mgr.current_mut()
            .unwrap()
            .integrate_scan((0.0, 0.0, 0.0), &sc());
        assert_eq!(mgr.n_frozen(), 0);
        mgr.tick(1.0, (0.5, 0.0, 0.0));
        assert_eq!(mgr.n_frozen(), 0, "0.5 m < threshold should not trigger");
        mgr.tick(2.0, (1.5, 0.0, 0.0));
        assert_eq!(mgr.n_frozen(), 1, "1.5 m ≥ threshold should switch");
        assert_eq!(mgr.n_total(), 2);
    }

    #[test]
    fn age_triggers_switch() {
        let cfg = SubmapManagerConfig {
            max_age_s: 5.0,
            max_travel_m: 1000.0,
            ..SubmapManagerConfig::default()
        };
        let mut mgr = SubmapManager::new(cfg);
        mgr.tick(0.0, (0.0, 0.0, 0.0));
        mgr.current_mut()
            .unwrap()
            .integrate_scan((0.0, 0.0, 0.0), &sc());
        // Slightly away from the anchor, so the age rule is armed.
        mgr.tick(3.0, (0.2, 0.0, 0.0));
        assert_eq!(mgr.n_frozen(), 0);
        mgr.tick(6.0, (0.2, 0.0, 0.0));
        assert_eq!(mgr.n_frozen(), 1);
    }

    #[test]
    fn new_submap_anchor_equals_tracked_pose_at_switch() {
        let cfg = SubmapManagerConfig {
            max_travel_m: 0.5,
            max_age_s: 1000.0,
            ..SubmapManagerConfig::default()
        };
        let mut mgr = SubmapManager::new(cfg);
        mgr.tick(0.0, (0.0, 0.0, 0.0));
        mgr.current_mut()
            .unwrap()
            .integrate_scan((0.0, 0.0, 0.0), &sc());
        let switch_pose = (1.0, 0.5, 0.7);
        mgr.tick(1.0, switch_pose);
        let cur = mgr.current().unwrap();
        let a = cur.anchor_pose();
        assert!((a.0 - switch_pose.0).abs() < 1e-6);
        assert!((a.1 - switch_pose.1).abs() < 1e-6);
        assert!((a.2 - switch_pose.2).abs() < 1e-6);
    }
}
