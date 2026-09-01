//! The assembled SLAM pipeline: what a host drives, one call per event.
//!
//! Everything here existed as wiring in the prototype runtime's `maploc.rs`
//! and again in this crate's replay bench; a third copy inside robotd would
//! be the one that drifts. The pipeline owns the submap manager, the pose
//! graph, loop closure and the optimizer, and exposes exactly the operations
//! a host performs: advance the tracked pose from odometry, integrate a
//! scan, let submaps age, render, persist.
//!
//! The tracked pose lives in the MAP frame. It equals the odometry frame
//! until a loop closure (which re-anchors it along with the submaps) or a
//! relocalization (`set_tracked`) says otherwise — odometry only ever
//! contributes *deltas*.

use std::path::Path;

use crate::global_render::{GlobalRenderConfig, render_global};
use crate::grid::OccupancyGrid;
use crate::loop_closer::{LoopCloserConfig, detect_loops};
use crate::optimizer::{OptimizerConfig, optimize};
use crate::pose_graph::{PoseEdge, PoseGraph, between, compose, information_from_sigmas, wrap_pi};
use crate::session::{SessionState, save_session};
use crate::submap::{Pose2, Scan};
use crate::submap_manager::{SubmapManager, SubmapManagerConfig, TickOutcome};

#[derive(Debug, Clone)]
pub struct SlamConfig {
    pub submap: SubmapManagerConfig,
    pub loops: LoopCloserConfig,
    pub optimizer: OptimizerConfig,
    /// Confidence of the odometry edges between consecutive submap anchors.
    pub odom_sigma_xy: f32,
    pub odom_sigma_yaw: f32,
    pub render: GlobalRenderConfig,
}

impl Default for SlamConfig {
    fn default() -> Self {
        Self {
            // Small submaps on purpose: with stop-and-scan the atomic unit
            // is the still window, and odometry drifts between stops — a
            // submap spanning several stops is not rigid, and loop-closure
            // witnesses captured a stop apart contradict each other by
            // exactly that internal drift (measured on recorded sessions).
            submap: SubmapManagerConfig {
                max_age_s: 8.0,
                max_travel_m: 0.8,
                ..SubmapManagerConfig::default()
            },
            loops: LoopCloserConfig {
                // Residual intra-submap drift is real; demanding 6 cm
                // consensus from witnesses a stop apart rejects true closures.
                verify_max_spread_m: 0.12,
                verify_max_spread_rad: 0.09,
                ..LoopCloserConfig::default()
            },
            optimizer: OptimizerConfig::default(),
            odom_sigma_xy: 0.10,
            odom_sigma_yaw: 0.05,
            render: GlobalRenderConfig::default(),
        }
    }
}

pub struct Slam {
    cfg: SlamConfig,
    mgr: SubmapManager,
    graph: PoseGraph,
    node_for_submap: Vec<usize>,
    n_loops: usize,
    tracked: Pose2,
    last_odom: Option<Pose2>,
    /// Index of the odometry edge feeding the CURRENT submap's node, so a
    /// re-anchor can update its measurement in place.
    edge_into_current: Option<usize>,
    /// Set by anything that changes what a render would show.
    dirty: bool,
}

impl Slam {
    pub fn new(cfg: SlamConfig) -> Self {
        Self {
            mgr: SubmapManager::new(cfg.submap),
            graph: PoseGraph::new(),
            node_for_submap: Vec::new(),
            n_loops: 0,
            tracked: (0.0, 0.0, 0.0),
            last_odom: None,
            edge_into_current: None,
            dirty: false,
            cfg,
        }
    }

    /// Resume from a saved session: the map is trusted; the tracked pose is
    /// wherever the robot was when it saved — accurate iff it actually
    /// starts where it left off.
    pub fn from_session(cfg: SlamConfig, s: SessionState) -> Self {
        let mgr = SubmapManager::from_parts(cfg.submap, s.frozen, s.current);
        let n_loops = s
            .graph
            .edges()
            .len()
            .saturating_sub(mgr.n_frozen().saturating_sub(1));
        Self {
            mgr,
            graph: s.graph,
            node_for_submap: s.node_for_submap,
            n_loops,
            tracked: s.tracked,
            last_odom: None,
            edge_into_current: None,
            dirty: true,
            cfg,
        }
    }

    pub fn tracked(&self) -> Pose2 {
        self.tracked
    }

    /// Overwrite the tracked pose (a relocalization result). Clears the
    /// odometry anchor so the next delta composes from fresh readings —
    /// the previous raw odometry is in a different frame now.
    pub fn set_tracked(&mut self, pose: Pose2) {
        self.tracked = pose;
        self.last_odom = None;
    }

    pub fn n_submaps(&self) -> usize {
        self.mgr.n_total()
    }

    pub fn n_loops(&self) -> usize {
        self.n_loops
    }

    /// Anchor pose of submap `idx` (frozen first, then current) — how a
    /// caller re-expresses a pose recorded relative to a submap after
    /// closures have moved it.
    pub fn anchor(&self, idx: usize) -> Option<Pose2> {
        if idx < self.mgr.n_frozen() {
            Some(self.mgr.frozen()[idx].anchor_pose())
        } else if idx < self.mgr.n_total() {
            self.mgr.current().map(|c| c.anchor_pose())
        } else {
            None
        }
    }

    /// Compose the newest odometry reading onto the tracked pose. Raw
    /// odometry poses live in their own frame; only body-frame deltas
    /// between consecutive readings carry over.
    pub fn observe_odom(&mut self, odom: Pose2) {
        if let Some((px, py, pyaw)) = self.last_odom {
            let (dxw, dyw) = (odom.0 - px, odom.1 - py);
            let (cp, sp) = (pyaw.cos(), pyaw.sin());
            let (dxb, dyb) = (cp * dxw + sp * dyw, -sp * dxw + cp * dyw);
            let dyaw = wrap_pi(odom.2 - pyaw);
            let (cy, sy) = (self.tracked.2.cos(), self.tracked.2.sin());
            self.tracked.0 += cy * dxb - sy * dyb;
            self.tracked.1 += sy * dxb + cy * dyb;
            self.tracked.2 = wrap_pi(self.tracked.2 + dyaw);
        }
        self.last_odom = Some(odom);
    }

    /// Let the submap manager age/travel-freeze; on a freeze, run loop
    /// closure and (if any closed) the graph optimizer, re-anchoring every
    /// submap and the tracked pose. Returns true when a submap froze.
    pub fn tick(&mut self, now_s: f32) -> bool {
        let prev_frozen = self.mgr.n_frozen();
        match self.mgr.tick(now_s, self.tracked) {
            TickOutcome::Idle => return false,
            TickOutcome::Reanchored => {
                // The empty current submap moved under its node: keep the
                // node (and the edge into it) telling the same story, or the
                // graph would preserve an anchor the map no longer has.
                let anchor = self.mgr.current().expect("re-anchored").anchor_pose();
                if let Some(&node) = self.node_for_submap.last() {
                    self.graph.nodes_mut()[node].pose = anchor;
                    if let Some(edge_idx) = self.edge_into_current {
                        let from = self.graph.edges()[edge_idx].from;
                        let from_pose = self.graph.nodes()[from].pose;
                        self.graph.edges_mut()[edge_idx].measurement = between(from_pose, anchor);
                    }
                }
                return false;
            }
            TickOutcome::Opened => {}
        }
        let anchor = self.mgr.current().expect("just opened").anchor_pose();
        let node = self.graph.add_node(anchor, self.mgr.n_total() - 1);
        // Chain the new node to its predecessor IMMEDIATELY — the current
        // submap's node included. An unconnected node is invisible to the
        // optimizer, and the tracked-pose correction after a loop closure
        // rides the current node: leave it floating and the closure moves
        // every frozen anchor while tracking sails on uncorrected (the
        // prototype's wiring had exactly this hole).
        self.edge_into_current = None;
        if let Some(&prev_node) = self.node_for_submap.last() {
            let prev_pose = self.graph.nodes()[prev_node].pose;
            self.graph.add_edge(PoseEdge {
                from: prev_node,
                to: node,
                measurement: between(prev_pose, anchor),
                information: information_from_sigmas(
                    self.cfg.odom_sigma_xy,
                    self.cfg.odom_sigma_yaw,
                ),
            });
            self.edge_into_current = Some(self.graph.edges().len() - 1);
        }
        self.node_for_submap.push(node);

        if self.mgr.n_frozen() > prev_frozen && self.mgr.n_frozen() >= 2 {
            let idx = self.mgr.n_frozen() - 1;
            let loops = detect_loops(self.mgr.frozen_mut(), idx, &self.cfg.loops);
            for lc in &loops {
                self.n_loops += 1;
                // Edge confidence scaled by match quality: a 9 cm-residual
                // match is not known to 5 cm.
                let sigma = self.cfg.loops.edge_sigma_xy.max(lc.residual_m);
                let factor = sigma / self.cfg.loops.edge_sigma_xy;
                self.graph.add_edge(PoseEdge {
                    from: self.node_for_submap[lc.from_idx],
                    to: self.node_for_submap[lc.to_idx],
                    measurement: lc.measurement,
                    information: information_from_sigmas(
                        sigma,
                        self.cfg.loops.edge_sigma_yaw * factor,
                    ),
                });
            }
            if !loops.is_empty() {
                // The optimizer moves anchors; `tracked` must move WITH the
                // current submap's anchor or every scan after the closure is
                // painted at the old, uncorrected pose.
                let cur_node = *self.node_for_submap.last().expect("nodes exist");
                let old_anchor = self.graph.nodes()[cur_node].pose;
                let _ = optimize(&mut self.graph, &self.cfg.optimizer);
                for (sm, &node) in self.node_for_submap.iter().enumerate() {
                    let pose = self.graph.nodes()[node].pose;
                    if sm < self.mgr.n_frozen() {
                        self.mgr.frozen_mut()[sm].set_anchor_pose(pose);
                    } else if let Some(cur) = self.mgr.current_mut() {
                        cur.set_anchor_pose(pose);
                    }
                }
                let new_anchor = self.graph.nodes()[cur_node].pose;
                self.tracked = compose(new_anchor, between(old_anchor, self.tracked));
            }
        }
        self.dirty = true;
        true
    }

    /// Ink one scan into the current submap at the given body pose (usually
    /// [`Slam::tracked`], but a still-window composite carries its own).
    pub fn integrate(&mut self, pose: Pose2, scan: &Scan) {
        self.integrate_weighted(pose, scan, 1);
    }

    /// Like [`Slam::integrate`], applying the log-odds `passes` times while
    /// storing the scan once — see `Submap::integrate_scan_weighted`.
    pub fn integrate_weighted(&mut self, pose: Pose2, scan: &Scan, passes: usize) {
        if let Some(cur) = self.mgr.current_mut() {
            cur.integrate_scan_weighted(pose, scan, passes);
            self.dirty = true;
        }
    }

    /// Composite of every submap, or `None` before the first one opens.
    pub fn render(&self) -> Option<OccupancyGrid> {
        render_global(self.mgr.all(), &self.cfg.render)
    }

    /// Whether anything changed since the flag was last taken — what an
    /// autosave or a map publisher polls instead of re-rendering blindly.
    pub fn take_dirty(&mut self) -> bool {
        std::mem::take(&mut self.dirty)
    }

    pub fn save<P: AsRef<Path>>(&self, path: P) -> std::io::Result<()> {
        save_session(
            path,
            self.mgr.frozen(),
            self.mgr.current(),
            &self.graph,
            &self.node_for_submap,
            self.tracked,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The core promise: drive a square with drifting odometry, scans of a
    /// fixed room; the pipeline freezes submaps, closes the loop and the
    /// tracked pose comes back better than raw odometry left it.
    #[test]
    fn a_walked_loop_closes_and_corrects_tracking() {
        // The "real" world: an asymmetric room grid to raycast against.
        let mut world = OccupancyGrid::new(crate::grid::GridConfig {
            x_range: (-2.5, 2.5),
            y_range: (-2.5, 2.5),
            cell: 0.05,
        });
        // Walls at ±1.6 m: comfortably inside every submap's ±2 m local
        // grid, wherever along the path the submap anchors.
        let n = 200;
        for _ in 0..6 {
            for i in 0..n {
                let t = -1.6 + 3.2 * (i as f32 / (n - 1) as f32);
                world.integrate_ray(0.0, 0.0, t, 1.6, true);
                world.integrate_ray(0.0, 0.0, t, -1.6, true);
                world.integrate_ray(0.0, 0.0, 1.6, t, true);
                world.integrate_ray(0.0, 0.0, -1.6, t, true);
            }
            for i in 0..n / 2 {
                let t = 1.6 * (i as f32 / ((n / 2) as f32));
                world.integrate_ray(0.0, 0.0, t, 0.5, true);
            }
        }
        let scan_at = |world: &mut OccupancyGrid, pose: Pose2| -> Scan {
            let mut a = Vec::new();
            let mut r = Vec::new();
            for k in 0..180 {
                let aa = -std::f32::consts::PI + (k as f32) * (2.0 * std::f32::consts::PI / 180.0);
                let rr = world.cast_ray(pose.0, pose.1, pose.2 + aa, 4.0);
                if rr < 3.9 {
                    a.push(aa);
                    r.push(rr);
                }
            }
            Scan::from_polar(&a, &r, (0.0, 0.0), 1e-3)
        };

        let mut slam = Slam::new(SlamConfig {
            loops: LoopCloserConfig {
                min_witness_beams: 60,
                ..SlamConfig::default().loops
            },
            ..SlamConfig::default()
        });

        // A square path, revisiting the start. Odometry drifts linearly:
        // +5 mm of x per step, ~0.27 m by the end — enough that the closure
        // is a correction worth an edge, not noise the floor should drop.
        let waypoints = [
            (0.0f32, 0.0f32),
            (0.9, 0.0),
            (0.9, -1.0),
            (-0.4, -1.0),
            (-0.4, 0.0),
            (0.0, 0.0),
        ];
        let mut truth_path: Vec<Pose2> = Vec::new();
        for pair in waypoints.windows(2) {
            let (ax, ay) = pair[0];
            let (bx, by) = pair[1];
            let steps = (((bx - ax).hypot(by - ay)) / 0.1).ceil() as usize;
            for s in 0..steps {
                let t = s as f32 / steps as f32;
                truth_path.push((ax + t * (bx - ax), ay + t * (by - ay), 0.0));
            }
        }
        let mut now = 0.0f32;
        for (i, truth) in truth_path.iter().enumerate() {
            let drift = 0.005 * i as f32;
            let odom = (truth.0 + drift, truth.1, truth.2);
            slam.observe_odom(odom);
            slam.tick(now);
            let scan = scan_at(&mut world, *truth);
            let pose = slam.tracked();
            // A real stop integrates a whole window of frames — wall cells
            // get hammered well past the matcher's confidence threshold.
            // One thin pass would leave walls the loop closer cannot see.
            for _ in 0..4 {
                slam.integrate(pose, &scan);
            }
            now += 1.0; // one "stop" per step: ages submaps quickly
        }

        // Stand at the end long enough for the last submap — the one that
        // revisits the start — to age out and freeze: closures run at
        // freeze time, and a walk that stops mid-submap has not yet had its
        // final chance to correct.
        let end_odom = {
            let t = truth_path.last().expect("path");
            (t.0 + 0.005 * (truth_path.len() - 1) as f32, t.1, t.2)
        };
        for _ in 0..12 {
            slam.observe_odom(end_odom);
            slam.tick(now);
            now += 1.0;
        }

        assert!(slam.n_submaps() > 3, "the walk must span several submaps");
        assert!(slam.n_loops() > 0, "revisiting the start must close a loop");

        let end_truth = *truth_path.last().expect("path");
        let raw_drift = 0.005 * (truth_path.len() - 1) as f32;
        let tracked = slam.tracked();
        let err = (tracked.0 - end_truth.0).hypot(tracked.1 - end_truth.1);
        assert!(
            err < raw_drift * 0.75,
            "loop closure must beat raw odometry: err {err:.3} vs drift {raw_drift:.3}"
        );

        let grid = slam.render().expect("submaps exist");
        assert!(grid.width() > 20 && grid.height() > 20);
    }

    /// Sessions round-trip through the pipeline: save, resume, keep mapping.
    #[test]
    fn a_session_survives_a_restart() {
        let mut slam = Slam::new(SlamConfig::default());
        slam.observe_odom((0.0, 0.0, 0.0));
        slam.tick(0.0);
        slam.integrate(
            (0.0, 0.0, 0.0),
            &Scan::from_polar(&[0.0], &[1.0], (0.0, 0.0), 1e-3),
        );

        let path = std::env::temp_dir().join(format!(
            "maploc_pipeline_test_{}.session",
            std::process::id()
        ));
        slam.save(&path).expect("save");
        let restored = SessionState::load(&path).expect("load").expect("present");
        std::fs::remove_file(&path).ok();

        let resumed = Slam::from_session(SlamConfig::default(), restored);
        assert_eq!(resumed.n_submaps(), 1);
        assert_eq!(resumed.tracked(), slam.tracked());
    }
}
