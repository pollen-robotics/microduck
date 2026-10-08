//! The whole loop: state and depth in, keyframes, a pose graph and a pose in the map out.
//!
//! The service drives it from its sockets; the replay bench and the tests drive it from files and
//! a synthetic world. Nothing here does IO, so all three run the same decisions.
//!
//! # The cycle
//!
//! The robot walks; odometry carries the pose. It stops; after a moment of stillness a keyframe
//! opens and collects the frames of the stop (the head sweeping, ideally). It moves again; the
//! keyframe closes and is placed:
//!
//! 1. **Predicted** where odometry says, relative to the last keyframe.
//! 2. **Matched** against the map around that prediction, inside a window as wide as odometry's
//!    uncertainty since the last match. The match becomes an edge with its own covariance —
//!    only in the directions the scan actually pins (see [`crate::matcher`]).
//! 3. **Loop-checked** against parts of the map the robot left long ago, with a wide search.
//!    Accepted only when two consecutive stops agree on the same correction: every single-window
//!    acceptance path in the first maploc eventually false-positived through the keyhole.
//! 4. **Optimized**, and the map-from-odometry transform updated from the keyframe's new pose.
//!
//! # Losing and finding itself
//!
//! Three things break odometry's chain: being picked up and carried (odometry's position is
//! meaningless while the feet are in the air; the gyro's heading still holds), a fall, and a
//! power cycle (a fresh odometry frame with an arbitrary heading). After each the mapper is
//! **lost**, with whatever it still knows as a hint — a heading after a carry, a rough pose after
//! a fall, the pose it was switched off at after a reboot. Keyframes made while lost go into an
//! **island**: a piece of graph chained by odometry but not attached to the map. Each new one is
//! searched for in the map — with the last few stops of the island as one wider scan — and when
//! two consecutive stops agree, the island is joined to the map and tracking resumes.
//!
//! Tracking can also be lost without any of those: two consecutive stops that land on seen-clear
//! floor or nowhere near the walls the map predicts. Those stops are never added to the map; they
//! start the island instead.

use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

use crate::field::{Field, FieldConfig};
use crate::graph::{Edge, EdgeKind, Graph, GraphConfig, OdometryNoise, info_in_frame};
use crate::grid::OccupancyGrid;
use crate::input::{Coverage, DepthFrame, History, StateSample};
use crate::keyframe::{Keyframe, KeyframeBuilder, KeyframeConfig};
use crate::matcher::{MatchConfig, MatchResult, Window, match_local, match_wide};
use crate::project::{ProjectConfig, Projector};
use crate::se2::{Pose2, wrap};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct MapperConfig {
    pub project: ProjectConfig,
    pub keyframe: KeyframeConfig,
    pub field: FieldConfig,
    pub matcher: MatchConfig,
    pub graph: GraphConfig,
    pub odometry: OdometryNoise,
    /// How long before `t_ns` a depth frame's light was collected: the stamp is taken when the
    /// frame is read, after the integration and the poll.
    pub tof_latency_ms: f64,
    /// Stillness required before a stop opens a keyframe — the trunk settling after the last
    /// step.
    pub settle_s: f64,
    /// Odometry speeds below which the robot counts as standing.
    pub still_speed_mps: f64,
    pub still_turn_rps: f64,
    /// Keyframes within this of a prediction make up the map it is matched against.
    pub local_radius_m: f64,
    pub local_max_keyframes: usize,
    /// Tracking match window, bounds.
    pub track_min_xy_m: f64,
    pub track_max_xy_m: f64,
    pub track_min_yaw_rad: f64,
    pub track_max_yaw_rad: f64,
    /// A tracking match is believed when at least this share of its obstacles land within one σ
    /// of a mapped wall and at most `max_conflict` on seen-clear floor.
    pub min_hit_frac: f64,
    pub min_known_frac: f64,
    pub max_conflict: f64,
    pub min_match_points: usize,
    /// A stop contradicts the map when the map knows this much of where its obstacles land and
    /// they still miss.
    pub contradiction_known_frac: f64,
    pub contradiction_hit_frac: f64,
    /// A loop candidate must be at least this far back along the path walked.
    pub loop_min_path_m: f64,
    /// Loop search radius: grows with the path walked since the last correction.
    pub loop_base_m: f64,
    pub loop_per_m: f64,
    pub loop_max_m: f64,
    pub loop_half_yaw_rad: f64,
    /// Wide-search acceptance: the best pose must be this much more likely (log, tempered) than
    /// the best elsewhere, pin all three directions, and the scan must span this much of the
    /// circle.
    pub wide_min_margin: f64,
    pub wide_min_hit_frac: f64,
    pub wide_min_span_rad: f64,
    /// Two consecutive wide matches agree when their implied corrections differ by less than
    /// this.
    pub agree_m: f64,
    pub agree_rad: f64,
    /// After a carry, how far the gyro's heading is trusted.
    pub carry_yaw_tol_rad: f64,
    /// After a fall, how far the pose is trusted.
    pub fall_xy_tol_m: f64,
    pub fall_yaw_tol_rad: f64,
    /// After a reboot, how far the robot may have been moved while off before the search
    /// widens to the whole map.
    pub boot_xy_tol_m: f64,
    pub boot_yaw_tol_rad: f64,
    /// Stops chained into one scan for a wide search.
    pub composite_keyframes: usize,
}

impl Default for MapperConfig {
    fn default() -> Self {
        Self {
            project: ProjectConfig::default(),
            keyframe: KeyframeConfig::default(),
            field: FieldConfig::default(),
            matcher: MatchConfig::default(),
            graph: GraphConfig::default(),
            odometry: OdometryNoise::default(),
            tof_latency_ms: 10.0,
            settle_s: 0.4,
            still_speed_mps: 0.03,
            still_turn_rps: 0.08,
            local_radius_m: 2.5,
            local_max_keyframes: 12,
            track_min_xy_m: 0.15,
            track_max_xy_m: 0.40,
            track_min_yaw_rad: 6f64.to_radians(),
            track_max_yaw_rad: 12f64.to_radians(),
            min_hit_frac: 0.5,
            min_known_frac: 0.3,
            max_conflict: 0.15,
            min_match_points: 15,
            contradiction_known_frac: 0.5,
            contradiction_hit_frac: 0.25,
            loop_min_path_m: 3.0,
            loop_base_m: 0.3,
            loop_per_m: 0.1,
            loop_max_m: 2.0,
            loop_half_yaw_rad: 8f64.to_radians(),
            wide_min_margin: 3.0,
            wide_min_hit_frac: 0.6,
            wide_min_span_rad: 60f64.to_radians(),
            agree_m: 0.15,
            agree_rad: 4f64.to_radians(),
            carry_yaw_tol_rad: 15f64.to_radians(),
            fall_xy_tol_m: 0.5,
            fall_yaw_tol_rad: 20f64.to_radians(),
            boot_xy_tol_m: 0.3,
            boot_yaw_tol_rad: 15f64.to_radians(),
            composite_keyframes: 3,
        }
    }
}

/// Why the mapper does not know where it is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LostCause {
    Boot,
    Carried,
    Fell,
    /// Stops stopped agreeing with the map.
    Contradiction,
}

/// What is still known while lost.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Hint {
    /// Nothing: search the whole map at every heading.
    None,
    /// The heading, as an offset from the odometry frame to the map: `map yaw = odom yaw + this`.
    Heading { odom_to_map_yaw: f64, tol: f64 },
    /// A pose in the map where the robot probably is, given in the odometry frame of the moment
    /// via `map_from_odom` (so a robot that walked a little while lost is still predicted).
    Pose {
        map_from_odom: Pose2,
        xy_tol: f64,
        yaw_tol: f64,
    },
    /// After a reboot: the map pose it was switched off at. The new odometry frame's relation to
    /// the map is unknown, so this applies to the first keyframe of the island only.
    Parked {
        pose: Pose2,
        xy_tol: f64,
        yaw_tol: f64,
    },
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Loc {
    Tracking { map_from_odom: Pose2 },
    Lost { cause: LostCause, hint: Hint },
}

/// What happened, for logs and the bench.
#[derive(Debug, Clone, PartialEq)]
pub enum Event {
    Keyframe {
        id: u32,
        frames: u32,
        occ: usize,
        free: usize,
        /// The tracking match, when there was a map to match against.
        matched: Option<MatchSummary>,
    },
    /// A stop that was too short to keep.
    Discarded {
        frames: u32,
    },
    LoopClosed {
        from: usize,
        to: usize,
        correction: Pose2,
    },
    Lost {
        cause: LostCause,
    },
    Relocalized {
        node: usize,
        pose: Pose2,
        after_keyframes: usize,
    },
    /// An edge the optimized graph refused, removed.
    EdgeRejected {
        kind: EdgeKind,
        from: usize,
        to: usize,
    },
}

/// How a tracking match went, believed or not.
#[derive(Debug, Clone, PartialEq)]
pub struct MatchSummary {
    pub accepted: bool,
    pub rank: usize,
    /// The matched pose relative to odometry's prediction.
    pub correction: Pose2,
    pub hit_frac: f64,
    pub known_frac: f64,
    pub conflict_frac: f64,
    pub n_points: usize,
    /// Spread of the match, per direction: metres, metres, degrees.
    pub sigma: [f64; 3],
}

/// A wide match waiting for the next stop to agree with it.
#[derive(Debug, Clone)]
struct Pending {
    node: usize,
    m: MatchResult,
    /// The map-from-odometry transform the match implies.
    implied: Pose2,
    ref_node: usize,
}

/// What the mapper saves: everything needed to resume after a reboot.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Session {
    pub version: u32,
    pub keyframes: Vec<Keyframe>,
    pub graph: Graph,
    pub island: Vec<u32>,
    /// The map pose at the last moment it was known, for the boot hint.
    pub last_pose: Option<Pose2>,
}

pub const SESSION_VERSION: u32 = 1;

pub struct Mapper {
    cfg: MapperConfig,
    projector: Projector,
    history: History,
    pending_frames: VecDeque<DepthFrame>,
    builder: Option<KeyframeBuilder>,
    builder_path: f64,
    builder_turn: f64,
    still_since: Option<u64>,

    keyframes: Vec<Keyframe>,
    graph: Graph,
    /// Per node: which piece of graph it belongs to. The map is the island of node 0.
    island: Vec<u32>,
    next_island: u32,
    /// Cumulative odometry path and turn at each node — loop candidates must be far back along
    /// the walk, and odometry edges are weighted by what was walked.
    path_at: Vec<f64>,
    turn_at: Vec<f64>,

    loc: Loc,
    /// The node the next keyframe chains to by odometry, if the chain is unbroken.
    chain: Option<usize>,
    path: f64,
    turn: f64,
    last_odom: Option<StateSample>,
    /// Path walked since a match last pinned the pose, for the tracking window.
    path_since_match: f64,
    /// Turn and time since a match last pinned the heading, for the tracking window's yaw.
    turn_since_yaw: f64,
    t_yaw_ns: u64,
    /// Path walked since a loop closure or relocalization, for the loop window.
    path_since_anchor: f64,
    contradictions: u32,
    /// A stop that contradicted the map, held out until the next one decides.
    suspect: Option<Keyframe>,
    pending_loop: Option<Pending>,
    pending_reloc: Option<Pending>,
    lost_keyframes: usize,
    last_pose: Option<Pose2>,
    flags: (bool, bool),
    events: Vec<Event>,
}

impl Mapper {
    pub fn new(cfg: MapperConfig) -> Self {
        Self {
            projector: Projector::new(cfg.project.clone()),
            cfg,
            history: History::default(),
            pending_frames: VecDeque::new(),
            builder: None,
            builder_path: 0.0,
            builder_turn: 0.0,
            still_since: None,
            keyframes: Vec::new(),
            graph: Graph::default(),
            island: Vec::new(),
            next_island: 1,
            path_at: Vec::new(),
            turn_at: Vec::new(),
            loc: Loc::Tracking {
                map_from_odom: Pose2::IDENTITY,
            },
            chain: None,
            path: 0.0,
            turn: 0.0,
            last_odom: None,
            path_since_match: 0.0,
            turn_since_yaw: 0.0,
            t_yaw_ns: 0,
            path_since_anchor: 0.0,
            contradictions: 0,
            suspect: None,
            pending_loop: None,
            pending_reloc: None,
            lost_keyframes: 0,
            last_pose: None,
            flags: (false, false),
            events: Vec::new(),
        }
    }

    /// Resume a saved map. The robot was off, so it does not know where it is until a stop or
    /// two confirm it — first where it was switched off, then anywhere.
    pub fn resume(cfg: MapperConfig, s: Session) -> Self {
        let mut m = Self::new(cfg);
        m.next_island = s.island.iter().copied().max().unwrap_or(0) + 1;
        m.keyframes = s.keyframes;
        m.graph = s.graph;
        m.island = s.island;
        // Saved nodes are as far back along the walk as anything can be: every one of them is a
        // loop candidate for this session's stops.
        m.path_at = vec![f64::NEG_INFINITY; m.keyframes.len()];
        m.turn_at = vec![0.0; m.keyframes.len()];
        m.last_pose = s.last_pose;
        if !m.keyframes.is_empty() {
            let hint = match s.last_pose {
                Some(pose) => Hint::Parked {
                    pose,
                    xy_tol: m.cfg.boot_xy_tol_m,
                    yaw_tol: m.cfg.boot_yaw_tol_rad,
                },
                None => Hint::None,
            };
            m.loc = Loc::Lost {
                cause: LostCause::Boot,
                hint,
            };
        }
        m
    }

    pub fn session(&self) -> Session {
        Session {
            version: SESSION_VERSION,
            keyframes: self.keyframes.clone(),
            graph: self.graph.clone(),
            island: self.island.clone(),
            last_pose: self.pose().or(self.last_pose),
        }
    }

    pub fn take_events(&mut self) -> Vec<Event> {
        std::mem::take(&mut self.events)
    }

    pub fn keyframes(&self) -> &[Keyframe] {
        &self.keyframes
    }

    pub fn graph(&self) -> &Graph {
        &self.graph
    }

    /// Keyframes of the map proper (not of an island), with their poses.
    pub fn map_keyframes(&self) -> impl Iterator<Item = (Pose2, &Keyframe)> + '_ {
        let main = self.island.first().copied();
        self.keyframes
            .iter()
            .enumerate()
            .filter(move |(i, _)| Some(self.island[*i]) == main)
            .map(|(i, k)| (self.graph.nodes[i], k))
    }

    /// The map, rendered at `res` metres per cell.
    pub fn grid(&self, res: f64) -> Option<OccupancyGrid> {
        let kfs: Vec<_> = self.map_keyframes().collect();
        let main = self.main_island();
        let path: Vec<[Pose2; 2]> = self
            .graph
            .edges
            .iter()
            .filter(|e| {
                e.kind == EdgeKind::Odometry
                    && self.island[e.from] == main
                    && self.island[e.to] == main
            })
            .map(|e| [self.graph.nodes[e.from], self.graph.nodes[e.to]])
            .collect();
        OccupancyGrid::render(&kfs, &path, res, self.cfg.keyframe.footprint_m)
    }

    /// The robot's pose in the map now, when it knows it.
    pub fn pose(&self) -> Option<Pose2> {
        match self.loc {
            Loc::Tracking { map_from_odom } => self
                .history
                .latest()
                .map(|s| map_from_odom.compose(s.odom_pose())),
            Loc::Lost { .. } => None,
        }
    }

    pub fn lost(&self) -> Option<LostCause> {
        match self.loc {
            Loc::Lost { cause, .. } => Some(cause),
            Loc::Tracking { .. } => None,
        }
    }

    /// Whether a stop is being collected right now — when a head sweep is worth doing.
    pub fn collecting(&self) -> bool {
        self.builder.is_some()
    }

    pub fn on_state(&mut self, s: StateSample) {
        if let Some(prev) = self.last_odom
            && s.t_ns > prev.t_ns
        {
            let d = (s.trunk_pos[0] - prev.trunk_pos[0]).hypot(s.trunk_pos[1] - prev.trunk_pos[1]);
            let a = wrap(s.odom_yaw - prev.odom_yaw).abs();
            if !s.activity.picked_up {
                self.path += d;
                self.turn += a;
                self.path_since_match += d;
                self.turn_since_yaw += a;
                self.path_since_anchor += d;
            }
        }
        self.last_odom = Some(s);
        self.history.push(s);
        self.drain_frames();

        let act = s.activity;
        // Carries and falls end the chain the moment they begin.
        if act.picked_up && !self.flags.0 {
            self.break_chain(LostCause::Carried);
        }
        if act.fallen && !self.flags.1 {
            self.break_chain(LostCause::Fell);
        }
        self.flags = (act.picked_up, act.fallen);

        let calm = !(act.commanded || act.sitting || act.fallen || act.picked_up || act.unpowered);
        let slow = self
            .history
            .recent_speed(500_000_000)
            .is_some_and(|(v, w)| v < self.cfg.still_speed_mps && w < self.cfg.still_turn_rps);
        if calm && slow {
            let since = *self.still_since.get_or_insert(s.t_ns);
            if self.builder.is_none() && (s.t_ns - since) as f64 * 1e-9 >= self.cfg.settle_s {
                self.open(s);
            }
        } else {
            self.still_since = None;
            if self.builder.is_some() {
                self.close();
            }
        }
        if self.builder.as_ref().is_some_and(KeyframeBuilder::is_full) {
            self.close();
            self.open(s);
        }
    }

    pub fn on_depth(&mut self, f: DepthFrame) {
        self.pending_frames.push_back(f);
        // Bounded: a depth stream with no state stream must not grow without end.
        while self.pending_frames.len() > 64 {
            self.pending_frames.pop_front();
        }
        self.drain_frames();
    }

    fn drain_frames(&mut self) {
        let lat = (self.cfg.tof_latency_ms * 1e6) as u64;
        while let Some(f) = self.pending_frames.front() {
            let t = f.t_ns.saturating_sub(lat);
            match self.history.covers(t) {
                Coverage::Later => return,
                Coverage::Never => {
                    self.pending_frames.pop_front();
                }
                Coverage::Now => {
                    let f = self.pending_frames.pop_front().expect("front exists");
                    let (Some(state), Some(b)) = (self.history.at(t), self.builder.as_mut()) else {
                        continue;
                    };
                    if t < b.t_start_ns() {
                        continue;
                    }
                    let mut p = self.projector.project(&f, &state);
                    p.t_ns = t;
                    b.add(&p);
                }
            }
        }
    }

    fn open(&mut self, s: StateSample) {
        self.builder = Some(KeyframeBuilder::new(
            self.cfg.keyframe.clone(),
            s.odom_pose(),
            s.t_ns,
        ));
        self.builder_path = self.path;
        self.builder_turn = self.turn;
    }

    fn close(&mut self) {
        let Some(b) = self.builder.take() else {
            return;
        };
        let frames = b.n_frames();
        match b.finish(self.keyframes.len() as u32) {
            Some(kf) => self.place(kf),
            None => self.events.push(Event::Discarded { frames }),
        }
    }

    fn break_chain(&mut self, cause: LostCause) {
        self.builder = None;
        self.still_since = None;
        self.chain = None;
        self.suspect = None;
        self.pending_loop = None;
        self.pending_reloc = None;
        if let Loc::Tracking { map_from_odom } = self.loc {
            self.last_pose = self.pose().or(self.last_pose);
            let hint = match cause {
                LostCause::Carried => Hint::Heading {
                    odom_to_map_yaw: map_from_odom.yaw,
                    tol: self.cfg.carry_yaw_tol_rad,
                },
                LostCause::Fell => Hint::Pose {
                    map_from_odom,
                    xy_tol: self.cfg.fall_xy_tol_m,
                    yaw_tol: self.cfg.fall_yaw_tol_rad,
                },
                LostCause::Boot | LostCause::Contradiction => Hint::None,
            };
            if self.keyframes.is_empty() {
                // Nothing mapped yet: there is nothing to be lost from. Start over wherever it
                // is put down.
                return;
            }
            self.loc = Loc::Lost { cause, hint };
            self.lost_keyframes = 0;
            self.events.push(Event::Lost { cause });
        }
    }

    fn add_node(&mut self, kf: Keyframe, pose: Pose2, island: u32, fixed: bool) -> usize {
        let id = self.graph.add_node(pose, fixed);
        debug_assert_eq!(id, self.keyframes.len());
        let mut kf = kf;
        kf.id = id as u32;
        self.keyframes.push(kf);
        self.island.push(island);
        self.path_at.push(self.builder_path);
        self.turn_at.push(self.builder_turn);
        id
    }

    fn odometry_edge(&mut self, from: usize, to: usize) {
        let a = self.keyframes[from].odom;
        let b = self.keyframes[to].odom;
        let z = a.between(b);
        let path = (self.path_at[to] - self.path_at[from]).max(z.x.hypot(z.y));
        let turn = self.turn_at[to] - self.turn_at[from];
        let dt = (self.keyframes[to]
            .t_start_ns
            .saturating_sub(self.keyframes[from].t_end_ns)) as f64
            * 1e-9;
        let info = self.cfg.odometry.info(z, path, turn.max(z.yaw.abs()), dt);
        self.graph.add_edge(Edge {
            from,
            to,
            z,
            info,
            kind: EdgeKind::Odometry,
        });
    }

    /// Nodes of `island` near `at`, nearest first, excluding `skip`.
    fn nearby(
        &self,
        island: u32,
        at: Pose2,
        radius: f64,
        max: usize,
        skip: &dyn Fn(usize) -> bool,
    ) -> Vec<usize> {
        let mut v: Vec<(f64, usize)> = (0..self.keyframes.len())
            .filter(|&i| self.island[i] == island && !skip(i) && !self.keyframes[i].occ.is_empty())
            .map(|i| (self.graph.nodes[i].dist(at), i))
            .filter(|&(d, _)| d <= radius)
            .collect();
        v.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        v.truncate(max);
        v.into_iter().map(|(_, i)| i).collect()
    }

    fn field_of(&self, nodes: &[usize]) -> Option<Field> {
        let set: Vec<(Pose2, &Keyframe)> = nodes
            .iter()
            .map(|&i| (self.graph.nodes[i], &self.keyframes[i]))
            .collect();
        Field::build(&self.cfg.field, &set, 0.5)
    }

    fn main_island(&self) -> u32 {
        self.island.first().copied().unwrap_or(0)
    }

    /// Place a finished keyframe.
    fn place(&mut self, kf: Keyframe) {
        let (frames, occ, free) = (kf.n_frames, kf.occ.len(), kf.free.len());
        let id = self.keyframes.len() as u32;
        match self.loc {
            Loc::Tracking { map_from_odom } => {
                if self.keyframes.is_empty() {
                    // The first stop ever: the map's origin.
                    let pose = map_from_odom.compose(kf.odom);
                    let n = self.add_node(kf, pose, 0, true);
                    self.chain = Some(n);
                    self.events.push(Event::Keyframe {
                        id,
                        frames,
                        occ,
                        free,
                        matched: None,
                    });
                    return;
                }
                self.track(kf, map_from_odom);
            }
            Loc::Lost { cause, hint } => self.search(kf, cause, hint),
        }
    }

    fn track(&mut self, kf: Keyframe, map_from_odom: Pose2) {
        let (frames, occ, free) = (kf.n_frames, kf.occ.len(), kf.free.len());
        let predicted = map_from_odom.compose(kf.odom);
        let main = self.main_island();

        // The map around the prediction, and where in it this stop fits.
        let local = self.nearby(
            main,
            predicted,
            self.cfg.local_radius_m,
            self.cfg.local_max_keyframes,
            &|_| false,
        );
        // A window three odometry-σ wide since the last match that pinned each direction —
        // never narrower than a few times the precision a match can reach, or a flat direction
        // and a pinned one spread alike and the matcher cannot tell them apart.
        let noise = &self.cfg.odometry;
        let sxy =
            noise.base_xy_m + noise.along_per_m.max(noise.across_per_m) * self.path_since_match;
        let dt_yaw = if self.t_yaw_ns == 0 {
            0.0
        } else {
            kf.t_start_ns.saturating_sub(self.t_yaw_ns) as f64 * 1e-9
        };
        let syaw =
            noise.base_yaw_rad + noise.yaw_per_rad * self.turn_since_yaw + noise.yaw_per_s * dt_yaw;
        let win = Window {
            center: predicted,
            half_xy: (3.0 * sxy).clamp(self.cfg.track_min_xy_m, self.cfg.track_max_xy_m),
            half_yaw: (3.0 * syaw).clamp(self.cfg.track_min_yaw_rad, self.cfg.track_max_yaw_rad),
            step_yaw: 1f64.to_radians(),
        };
        let m = self
            .field_of(&local)
            .and_then(|f| match_local(&f, &kf, win, &self.cfg.matcher).map(|m| (m, f)));

        let good = m.as_ref().is_some_and(|(m, _)| {
            m.n_points >= self.cfg.min_match_points
                && m.known_frac >= self.cfg.min_known_frac
                && m.hit_frac >= self.cfg.min_hit_frac
                && m.conflict_frac <= self.cfg.max_conflict
                && m.rank > 0
        });
        let contradicts = m.as_ref().is_some_and(|(m, _)| {
            m.n_points >= self.cfg.min_match_points
                && m.known_frac >= self.cfg.contradiction_known_frac
                && (m.hit_frac < self.cfg.contradiction_hit_frac
                    || m.conflict_frac > 2.0 * self.cfg.max_conflict)
        });

        if contradicts {
            self.contradictions += 1;
            if self.contradictions >= 2 {
                // Two stops in a row disagree with the map: the pose is wrong, not the map.
                self.contradictions = 0;
                self.last_pose = Some(predicted);
                self.loc = Loc::Lost {
                    cause: LostCause::Contradiction,
                    hint: Hint::Pose {
                        map_from_odom,
                        xy_tol: 1.0,
                        yaw_tol: 0.3,
                    },
                };
                self.lost_keyframes = 0;
                self.chain = None;
                self.pending_loop = None;
                self.events.push(Event::Lost {
                    cause: LostCause::Contradiction,
                });
                // Both suspects start the island; neither touches the map.
                if let Some(prev) = self.suspect.take() {
                    self.search(prev, LostCause::Contradiction, self.hint());
                }
                self.search(kf, LostCause::Contradiction, self.hint());
                return;
            }
            self.suspect = Some(kf);
            return;
        }
        self.contradictions = 0;
        if let Some(prev) = self.suspect.take() {
            // One bad stop, then a good one: the first was a person in front of the sensor or a
            // moved chair. Keep it, chained, unmatched.
            let p = map_from_odom.compose(prev.odom);
            let n = self.add_node(prev, p, main, false);
            if let Some(c) = self.chain {
                self.odometry_edge(c, n);
            }
            self.chain = Some(n);
        }

        let node = self.add_node(kf, predicted, main, false);
        if let Some(c) = self.chain {
            self.odometry_edge(c, node);
        }
        self.chain = Some(node);

        let matched = m.as_ref().map(|(m, _)| MatchSummary {
            accepted: good,
            rank: m.rank,
            correction: predicted.between(m.pose),
            hit_frac: m.hit_frac,
            known_frac: m.known_frac,
            conflict_frac: m.conflict_frac,
            n_points: m.n_points,
            sigma: [
                m.cov[0][0].sqrt(),
                m.cov[1][1].sqrt(),
                m.cov[2][2].sqrt().to_degrees(),
            ],
        });
        if let (true, Some((m, _))) = (good, m.as_ref()) {
            let r = local[0];
            let z = self.graph.nodes[r].between(m.pose);
            self.graph.add_edge(Edge {
                from: r,
                to: node,
                z,
                info: info_in_frame(&m.info, m.pose.yaw),
                kind: EdgeKind::Match,
            });
            // Each direction's uncertainty restarts when a match pins it: both translation
            // directions for the position, the heading for the yaw.
            let (xy_vals, _) = crate::matcher::eigen_sym3([
                [m.info[0][0], m.info[0][1], 0.0],
                [m.info[1][0], m.info[1][1], 0.0],
                [0.0, 0.0, 1.0],
            ]);
            let pinned_xy = 1.0 / (2.0 * self.cfg.track_min_xy_m / 3.0).powi(2);
            if xy_vals[0].min(xy_vals[1]) > pinned_xy {
                self.path_since_match = 0.0;
            }
            if m.info[2][2] > 1.0 / (self.cfg.track_min_yaw_rad / 3.0).powi(2) {
                self.turn_since_yaw = 0.0;
                self.t_yaw_ns = self.keyframes[node].t_end_ns;
            }
        }
        self.events.push(Event::Keyframe {
            id: node as u32,
            frames,
            occ,
            free,
            matched,
        });

        self.try_loop(node, &local);
        self.optimize();
        if let Loc::Tracking { .. } = self.loc {
            self.loc = Loc::Tracking {
                map_from_odom: self.graph.nodes[node].compose(self.keyframes[node].odom.inverse()),
            };
        }
    }

    fn hint(&self) -> Hint {
        match self.loc {
            Loc::Lost { hint, .. } => hint,
            Loc::Tracking { .. } => Hint::None,
        }
    }

    /// The stop and the few before it in the same island, as one scan in its frame.
    fn composite(&self, node: usize) -> Keyframe {
        let isl = self.island[node];
        let base = self.graph.nodes[node];
        let others: Vec<(Pose2, &Keyframe)> = (0..node)
            .rev()
            .filter(|&i| self.island[i] == isl)
            .take(self.cfg.composite_keyframes.saturating_sub(1))
            .map(|i| (base.between(self.graph.nodes[i]), &self.keyframes[i]))
            .collect();
        self.keyframes[node].composite(&others)
    }

    fn wide_ok(&self, m: &MatchResult, margin: f64, span: f64) -> bool {
        m.rank == 3
            && margin >= self.cfg.wide_min_margin
            && m.hit_frac >= self.cfg.wide_min_hit_frac
            && m.known_frac >= self.cfg.min_known_frac
            && m.conflict_frac <= self.cfg.max_conflict
            && m.n_points >= self.cfg.min_match_points
            && span >= self.cfg.wide_min_span_rad
    }

    fn agrees(&self, a: Pose2, b: Pose2) -> bool {
        a.dist(b) <= self.cfg.agree_m && wrap(a.yaw - b.yaw).abs() <= self.cfg.agree_rad
    }

    fn try_loop(&mut self, node: usize, local: &[usize]) {
        let main = self.main_island();
        let here = self.path_at[node];
        let min_gap = self.cfg.loop_min_path_m;
        let radius = (self.cfg.loop_base_m + self.cfg.loop_per_m * self.path_since_anchor)
            .min(self.cfg.loop_max_m);
        let predicted = self.graph.nodes[node];
        let far_back =
            |i: usize| here - self.path_at[i] < min_gap || i == node || local.contains(&i);
        let cands = self.nearby(main, predicted, radius + 1.5, 24, &far_back);
        if cands.is_empty() {
            self.pending_loop = None;
            return;
        }
        let Some(field) = self.field_of(&cands) else {
            return;
        };
        let scan = self.composite(node);
        let win = Window {
            center: predicted,
            half_xy: radius,
            half_yaw: self.cfg.loop_half_yaw_rad,
            step_yaw: 2f64.to_radians(),
        };
        let Some(w) = match_wide(&field, &scan, win, &self.cfg.matcher) else {
            return;
        };
        if !self.wide_ok(&w.best, w.margin, scan.span_rad) {
            self.pending_loop = None;
            return;
        }
        let implied = w.best.pose.compose(self.keyframes[node].odom.inverse());
        let ref_node = cands[0];
        let this = Pending {
            node,
            m: w.best,
            implied,
            ref_node,
        };
        match self.pending_loop.take() {
            Some(prev) if self.agrees(prev.implied, implied) => {
                for p in [&prev, &this] {
                    let z = self.graph.nodes[p.ref_node].between(p.m.pose);
                    self.graph.add_edge(Edge {
                        from: p.ref_node,
                        to: p.node,
                        z,
                        info: info_in_frame(&p.m.info, p.m.pose.yaw),
                        kind: EdgeKind::Loop,
                    });
                }
                self.path_since_anchor = 0.0;
                self.events.push(Event::LoopClosed {
                    from: ref_node,
                    to: node,
                    correction: self.graph.nodes[node].between(this.m.pose),
                });
            }
            _ => self.pending_loop = Some(this),
        }
    }

    fn optimize(&mut self) {
        let Some(r) = self.graph.optimize(&self.cfg.graph) else {
            return;
        };
        if r.outliers.is_empty() {
            return;
        }
        for &i in r.outliers.iter().rev() {
            let e = self.graph.edges.remove(i);
            self.events.push(Event::EdgeRejected {
                kind: e.kind,
                from: e.from,
                to: e.to,
            });
        }
        self.repair_components();
        self.graph.optimize(&self.cfg.graph);
    }

    /// After edges are removed, make every connected piece of the graph an island with its own
    /// fixed node again. A piece with no fixed node has no position — the solver's system is
    /// singular there — and a piece no longer connected to node 0 is no longer the map. If the
    /// robot's own chain fell off the map, it is lost.
    fn repair_components(&mut self) {
        let n = self.keyframes.len();
        let mut parent: Vec<usize> = (0..n).collect();
        fn root(p: &mut [usize], mut i: usize) -> usize {
            while p[i] != i {
                p[i] = p[p[i]];
                i = p[i];
            }
            i
        }
        for e in &self.graph.edges {
            let (a, b) = (root(&mut parent, e.from), root(&mut parent, e.to));
            if a != b {
                parent[a.max(b)] = a.min(b);
            }
        }
        let main_root = root(&mut parent, 0);
        let main = self.main_island();
        let mut seen: Vec<Option<u32>> = vec![None; n];
        for i in 0..n {
            let r = root(&mut parent, i);
            if r == main_root {
                if self.island[i] != main {
                    self.island[i] = main;
                }
                continue;
            }
            let isl = match seen[r] {
                Some(isl) => isl,
                None => {
                    let isl = if self.island[r] == main {
                        let isl = self.next_island;
                        self.next_island += 1;
                        isl
                    } else {
                        self.island[r]
                    };
                    seen[r] = Some(isl);
                    // The piece's oldest node holds it still.
                    self.graph.fixed[r] = true;
                    isl
                }
            };
            self.island[i] = isl;
        }
        if let (Some(c), Loc::Tracking { .. }) = (self.chain, self.loc)
            && self.island[c] != main
        {
            self.loc = Loc::Lost {
                cause: LostCause::Contradiction,
                hint: Hint::None,
            };
            self.lost_keyframes = 0;
            self.events.push(Event::Lost {
                cause: LostCause::Contradiction,
            });
        }
    }

    /// A stop while lost: add it to the island, then look for the island in the map.
    fn search(&mut self, kf: Keyframe, cause: LostCause, hint: Hint) {
        let (frames, occ, free) = (kf.n_frames, kf.occ.len(), kf.free.len());
        let isl = match self.chain {
            Some(c) => self.island[c],
            None => {
                let i = self.next_island;
                self.next_island += 1;
                i
            }
        };
        // Island poses live in the island's own frame, which is the odometry frame.
        let pose = kf.odom;
        let first_of_island = self.chain.is_none();
        let node = self.add_node(kf, pose, isl, first_of_island);
        if let Some(c) = self.chain {
            self.odometry_edge(c, node);
        }
        self.chain = Some(node);
        self.lost_keyframes += 1;
        self.events.push(Event::Keyframe {
            id: node as u32,
            frames,
            occ,
            free,
            matched: None,
        });

        let main = self.main_island();
        let scan = self.composite(node);
        let odom = self.keyframes[node].odom;
        // Where to look, from the hint.
        let all: Vec<usize> = (0..self.keyframes.len())
            .filter(|&i| self.island[i] == main && !self.keyframes[i].occ.is_empty())
            .collect();
        if all.is_empty() {
            return;
        }
        let (center, half_xy, half_yaw, nodes) = match hint {
            Hint::Pose {
                map_from_odom,
                xy_tol,
                yaw_tol,
            } => {
                let c = map_from_odom.compose(odom);
                (
                    c,
                    xy_tol,
                    yaw_tol,
                    self.nearby(main, c, xy_tol + self.cfg.local_radius_m, 32, &|_| false),
                )
            }
            Hint::Parked {
                pose,
                xy_tol,
                yaw_tol,
            } if self.lost_keyframes == 1 => (
                pose,
                xy_tol,
                yaw_tol,
                self.nearby(main, pose, xy_tol + self.cfg.local_radius_m, 32, &|_| false),
            ),
            Hint::Heading {
                odom_to_map_yaw,
                tol,
            } => {
                let (c, half) = self.map_extent(&all);
                (
                    Pose2::new(c[0], c[1], wrap(odom.yaw + odom_to_map_yaw)),
                    half,
                    tol,
                    all.clone(),
                )
            }
            Hint::Parked { .. } | Hint::None => {
                let (c, half) = self.map_extent(&all);
                (
                    Pose2::new(c[0], c[1], 0.0),
                    half,
                    std::f64::consts::PI,
                    all.clone(),
                )
            }
        };
        let Some(field) = self.field_of(&nodes) else {
            return;
        };
        let win = Window {
            center,
            half_xy,
            half_yaw,
            step_yaw: 2f64.to_radians(),
        };
        let Some(w) = match_wide(&field, &scan, win, &self.cfg.matcher) else {
            return;
        };
        if !self.wide_ok(&w.best, w.margin, scan.span_rad) {
            self.pending_reloc = None;
            // A parked hint only holds for the first stop after boot; fall back to the whole map.
            if let (Hint::Parked { .. }, LostCause::Boot) = (hint, cause) {
                self.loc = Loc::Lost {
                    cause,
                    hint: Hint::None,
                };
            }
            return;
        }
        // The island → map transform this match implies (island frame = odometry frame).
        let implied = w.best.pose.compose(odom.inverse());
        let ref_node = self.nearby(main, w.best.pose, f64::INFINITY, 1, &|_| false)[0];
        let this = Pending {
            node,
            m: w.best,
            implied,
            ref_node,
        };
        match self.pending_reloc.take() {
            Some(prev) if self.island[prev.node] == isl && self.agrees(prev.implied, implied) => {
                self.join(isl, implied, &[prev, this]);
            }
            _ => self.pending_reloc = Some(this),
        }
    }

    fn map_extent(&self, nodes: &[usize]) -> ([f64; 2], f64) {
        let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
        for &i in nodes {
            let p = self.graph.nodes[i];
            lo = [lo[0].min(p.x), lo[1].min(p.y)];
            hi = [hi[0].max(p.x), hi[1].max(p.y)];
        }
        let half = 0.5 * (hi[0] - lo[0]).max(hi[1] - lo[1]) + 1.0;
        ([0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1])], half)
    }

    /// Attach an island to the map: move its nodes into the map frame, link the matched stops,
    /// and resume tracking from its newest stop.
    fn join(&mut self, isl: u32, map_from_island: Pose2, matches: &[Pending]) {
        let main = self.main_island();
        for i in 0..self.keyframes.len() {
            if self.island[i] == isl {
                self.island[i] = main;
                self.graph.nodes[i] = map_from_island.compose(self.graph.nodes[i]);
                self.graph.fixed[i] = false;
            }
        }
        for p in matches {
            let z = self.graph.nodes[p.ref_node].between(p.m.pose);
            self.graph.add_edge(Edge {
                from: p.ref_node,
                to: p.node,
                z,
                info: info_in_frame(&p.m.info, p.m.pose.yaw),
                kind: EdgeKind::Relocalize,
            });
        }
        let node = matches.last().expect("two matches").node;
        self.optimize();
        if self.island[node] != main {
            // The optimized map refused the match edges: not relocalized after all.
            return;
        }
        let pose = self.graph.nodes[node];
        self.loc = Loc::Tracking {
            map_from_odom: pose.compose(self.keyframes[node].odom.inverse()),
        };
        self.path_since_match = 0.0;
        self.turn_since_yaw = 0.0;
        self.t_yaw_ns = self.keyframes[node].t_end_ns;
        self.path_since_anchor = 0.0;
        self.events.push(Event::Relocalized {
            node,
            pose,
            after_keyframes: self.lost_keyframes,
        });
        self.lost_keyframes = 0;
    }
}
