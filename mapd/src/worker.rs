//! The thread that owns the mapper.
//!
//! Everything the mapper does is CPU and none of it is cancellation-safe: a stop is placed in a
//! few milliseconds, but a relocalization search over a whole house is a second or two on an A55.
//! So it gets a plain thread, fed through a bounded channel, and publishes what the sockets serve
//! into a [`Shared`] snapshot that is only ever locked for a copy. A search in progress delays the
//! map; it never delays an answer to `map.status`.

use std::path::PathBuf;
use std::sync::mpsc::{Receiver, RecvTimeoutError, SyncSender};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use duck_ipc_proto as proto;
use maploc::grid::Cell;
use maploc::input::{DepthFrame, StateSample};
use maploc::mapper::{Event, LostCause, Mapper, MapperConfig};
use maploc::se2::Pose2;

use crate::store;

/// What the worker is sent.
pub enum Input {
    State(Box<StateSample>),
    Depth(DepthFrame),
    /// Render the grid at this resolution.
    Grid(f64, SyncSender<Option<proto::MapGridResult>>),
    /// Forget the map; answered with how many stops were forgotten.
    Wipe(SyncSender<u32>),
    /// Save and stop.
    Shutdown(SyncSender<()>),
}

/// What the sockets read.
#[derive(Debug, Default, Clone)]
pub struct Shared {
    pub status: proto::MapStatusResult,
    /// Why each input is not flowing, when it is not.
    pub robot_down: Option<String>,
    pub tof_down: Option<String>,
}

impl Shared {
    /// The status, with the inputs' health folded into `unavailable`.
    pub fn status(&self) -> proto::MapStatusResult {
        let mut s = self.status.clone();
        if s.unavailable.is_none() {
            s.unavailable = match (&self.robot_down, &self.tof_down) {
                (Some(r), Some(t)) => Some(format!("{r}; {t}")),
                (Some(r), None) => Some(r.clone()),
                (None, Some(t)) => Some(t.clone()),
                (None, None) => None,
            };
        }
        s
    }
}

pub struct Worker {
    mapper: Mapper,
    cfg: MapperConfig,
    path: PathBuf,
    autosave: Duration,
    shared: Arc<Mutex<Shared>>,
    dirty: bool,
    last_save: Instant,
    saved_pose: Option<Pose2>,
    loops: u32,
    relocalizations: u32,
}

/// The map moves on its own as the robot walks; a saved pose further than this from the current
/// one makes the file stale enough to rewrite, so the boot hint after a power cut is close.
const POSE_STALE_M: f64 = 0.3;

pub fn lost_name(c: LostCause) -> &'static str {
    match c {
        LostCause::Boot => "boot",
        LostCause::Carried => "carried",
        LostCause::Fell => "fell",
        LostCause::Contradiction => "contradiction",
    }
}

fn to_pose(p: Pose2) -> proto::MapPose {
    proto::MapPose {
        x: p.x,
        y: p.y,
        yaw: p.yaw,
    }
}

impl Worker {
    pub fn new(
        cfg: MapperConfig,
        path: PathBuf,
        autosave: Duration,
        shared: Arc<Mutex<Shared>>,
    ) -> Self {
        let mapper = match store::load(&path) {
            store::Loaded::Map(s) => {
                tracing::info!(path = %path.display(), keyframes = s.keyframes.len(), "resuming the saved map; lost until a stop recognises it");
                Mapper::resume(cfg.clone(), *s)
            }
            store::Loaded::Nothing => {
                tracing::info!(path = %path.display(), "no saved map; starting a new one at the first stop");
                Mapper::new(cfg.clone())
            }
            store::Loaded::SetAside { reason, moved_to } => {
                tracing::error!(%reason, kept = %moved_to.display(), "the saved map could not be used; it is kept aside and a new map starts");
                Mapper::new(cfg.clone())
            }
        };
        let mut w = Self {
            mapper,
            cfg,
            path,
            autosave,
            shared,
            dirty: false,
            last_save: Instant::now(),
            saved_pose: None,
            loops: 0,
            relocalizations: 0,
        };
        w.publish();
        w
    }

    pub fn run(mut self, rx: Receiver<Input>) {
        loop {
            match rx.recv_timeout(Duration::from_secs(1)) {
                Ok(Input::State(s)) => {
                    let t_ns = s.t_ns;
                    self.mapper.on_state(*s);
                    self.after_input(t_ns);
                }
                Ok(Input::Depth(f)) => {
                    let t_ns = f.t_ns;
                    self.mapper.on_depth(f);
                    self.after_input(t_ns);
                }
                Ok(Input::Grid(res, reply)) => {
                    let _ = reply.send(self.grid(res));
                }
                Ok(Input::Wipe(reply)) => {
                    let (n, _) = self.mapper.counts();
                    self.mapper = Mapper::new(self.cfg.clone());
                    if let Err(e) = store::remove(&self.path) {
                        tracing::error!(error = %e, path = %self.path.display(), "could not delete the map file");
                    }
                    self.dirty = false;
                    self.saved_pose = None;
                    tracing::warn!(
                        keyframes = n,
                        "map wiped; a new one starts at the next stop"
                    );
                    self.publish();
                    let _ = reply.send(n as u32);
                }
                Ok(Input::Shutdown(reply)) => {
                    self.save("shutdown");
                    let _ = reply.send(());
                    return;
                }
                Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => {
                    self.save("inputs closed");
                    return;
                }
            }
            if self.dirty && self.last_save.elapsed() >= self.autosave {
                self.save("autosave");
            }
        }
    }

    fn after_input(&mut self, t_ns: u64) {
        let events = self.mapper.take_events();
        let changed = !events.is_empty();
        for e in events {
            log_event(&e);
            match e {
                Event::LoopClosed { .. } => self.loops += 1,
                Event::Relocalized { .. } => self.relocalizations += 1,
                _ => {}
            }
            self.dirty = true;
        }
        if let (Some(p), Some(s)) = (self.mapper.pose(), self.saved_pose) {
            if p.dist(s) > POSE_STALE_M {
                self.dirty = true;
            }
        } else if self.mapper.pose().is_some() && self.saved_pose.is_none() {
            self.dirty = true;
        }
        self.publish_state(t_ns, changed);
    }

    fn publish_state(&mut self, t_ns: u64, full: bool) {
        if full {
            self.publish();
        }
        let state = self.state(t_ns);
        if let Ok(mut s) = self.shared.lock() {
            s.status.state = state;
        }
    }

    fn state(&self, t_ns: u64) -> proto::MapState {
        let (in_map, _) = self.mapper.counts();
        proto::MapState {
            t_ns,
            pose: self.mapper.pose().map(to_pose),
            lost: self.mapper.lost().map(|c| lost_name(c).to_owned()),
            collecting: self.mapper.collecting(),
            keyframes: in_map as u32,
        }
    }

    fn publish(&mut self) {
        let (_, islands) = self.mapper.counts();
        let t = self.shared.lock().map(|s| s.status.state.t_ns).unwrap_or(0);
        let state = self.state(t);
        let path = self.path.display().to_string();
        if let Ok(mut s) = self.shared.lock() {
            s.status.state = state;
            s.status.loops = self.loops;
            s.status.relocalizations = self.relocalizations;
            s.status.island_keyframes = islands as u32;
            s.status.path = path;
        }
    }

    fn save(&mut self, why: &str) {
        if !self.dirty {
            return;
        }
        let session = self.mapper.session();
        match store::save(&self.path, &session) {
            Ok(()) => {
                self.dirty = false;
                self.last_save = Instant::now();
                self.saved_pose = self.mapper.pose();
                let secs = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map_or(0, |d| d.as_secs());
                if let Ok(mut s) = self.shared.lock() {
                    s.status.saved_at = Some(secs);
                }
                tracing::info!(why, keyframes = session.keyframes.len(), path = %self.path.display(), "map saved");
            }
            Err(e) => {
                // Keep it dirty: the next autosave tries again.
                self.last_save = Instant::now();
                tracing::error!(error = %e, path = %self.path.display(), "could not save the map");
            }
        }
    }

    fn grid(&self, res: f64) -> Option<proto::MapGridResult> {
        let g = self.mapper.grid(res)?;
        let cells = g
            .cells
            .iter()
            .map(|c| match c {
                Cell::Unknown => '?',
                Cell::Free => '.',
                Cell::Occupied => '#',
            })
            .collect();
        Some(proto::MapGridResult {
            res_m: g.res,
            origin: g.origin,
            width: g.w as u32,
            height: g.h as u32,
            cells,
            keyframes: self
                .mapper
                .map_keyframes()
                .map(|(p, _)| to_pose(p))
                .collect(),
            pose: self.mapper.pose().map(to_pose),
        })
    }
}

fn log_event(e: &Event) {
    match e {
        Event::Keyframe {
            id,
            frames,
            occ,
            free,
            matched,
        } => match matched {
            Some(m) => tracing::info!(
                id,
                frames,
                occ,
                free,
                accepted = m.accepted,
                rank = m.rank,
                dx = format!("{:.3}", m.correction.x),
                dy = format!("{:.3}", m.correction.y),
                dyaw_deg = format!("{:.1}", m.correction.yaw.to_degrees()),
                hit = format!("{:.2}", m.hit_frac),
                "stop placed"
            ),
            None => tracing::info!(
                id,
                frames,
                occ,
                free,
                "stop placed, nothing to match against"
            ),
        },
        Event::Discarded { frames } => tracing::debug!(frames, "stop too short to keep"),
        Event::LoopClosed {
            from,
            to,
            correction,
        } => tracing::info!(
            from,
            to,
            dx = format!("{:.3}", correction.x),
            dy = format!("{:.3}", correction.y),
            "loop closed"
        ),
        Event::Lost { cause } => tracing::warn!(cause = lost_name(*cause), "lost"),
        Event::Relocalized {
            node,
            pose,
            after_keyframes,
        } => tracing::warn!(
            node,
            after_keyframes,
            x = format!("{:.2}", pose.x),
            y = format!("{:.2}", pose.y),
            yaw_deg = format!("{:.0}", pose.yaw.to_degrees()),
            "relocalized"
        ),
        Event::EdgeRejected { kind, from, to } => {
            tracing::info!(?kind, from, to, "edge rejected by the optimized map")
        }
    }
}
