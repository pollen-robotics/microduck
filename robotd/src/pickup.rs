//! `[pickup]`: pause the policy while somebody holds the robot.
//!
//! The classifier, its feature row, the pause pose and the hysteresis are
//! `duck_control::pickup` — the contract with the training code. This is the loop's side: what
//! gets fed in each tick, when the watch is allowed to act at all, and what a paused robot is
//! commanded. Built only when `[pickup] enabled = true`; otherwise the loop holds `None` and
//! none of this runs.

use std::time::Duration;

pub use duck_control::pickup::Edge;
use duck_control::pickup::{Detector, Latch, Thresholds, features, pause_target};
use duck_control::{NUM_JOINTS, Sensors};

/// The detector and the state around it.
#[derive(Debug)]
pub struct Watch<D = Detector> {
    detector: D,
    latch: Latch,
    /// What the loop commanded on the previous tick, and whether that was a pause target —
    /// the feature row pairs each reading with the command that produced it.
    previous: Option<([f64; NUM_JOINTS], bool)>,
    /// Where the current pause ramps from: the last target the policy commanded.
    pause_from: [f64; NUM_JOINTS],
    /// The latest probability, for the log lines on an edge.
    p_held: Option<f32>,
}

/// The scoring half, so the bookkeeping is testable without a model or ONNX Runtime.
pub trait Score {
    /// Add a row; `Ok(None)` while the window is still filling.
    fn observe(
        &mut self,
        row: [f32; duck_control::pickup::FEAT_LEN],
    ) -> Result<Option<f32>, String>;
    fn reset(&mut self);
}

impl Score for Detector {
    fn observe(
        &mut self,
        row: [f32; duck_control::pickup::FEAT_LEN],
    ) -> Result<Option<f32>, String> {
        Detector::observe(self, row).map_err(|e| e.to_string())
    }
    fn reset(&mut self) {
        Detector::reset(self);
    }
}

impl Watch {
    /// The watch `[pickup]` asks for, or `None` — in which case nothing is loaded and the loop
    /// never scores, pauses or records anything. A model that will not load is a warning, not
    /// unhealthy: the classifier is a convenience, and the robot walks without it.
    pub fn build(params: &crate::params::PickupParams) -> Option<Self> {
        if !params.enabled {
            return None;
        }
        let model = params.model_resolved()?;
        match Detector::load(&model) {
            Ok(detector) => {
                tracing::warn!(model = %model.display(), "pickup detection watching");
                Some(Self::new(
                    detector,
                    Thresholds {
                        pause_above: params.pause_threshold,
                        resume_below: params.resume_threshold,
                        ..Default::default()
                    },
                ))
            }
            Err(e) => {
                tracing::warn!(error = %e, "pickup detection unavailable");
                None
            }
        }
    }
}

impl<D: Score> Watch<D> {
    pub fn new(detector: D, thresholds: Thresholds) -> Self {
        Self {
            detector,
            latch: Latch::new(thresholds),
            previous: None,
            pause_from: [0.0; NUM_JOINTS],
            p_held: None,
        }
    }

    /// Paused because the robot is held.
    pub fn paused(&self) -> bool {
        self.latch.paused()
    }

    pub fn p_held(&self) -> Option<f32> {
        self.p_held
    }

    /// One tick, before the loop decides whether the policy drives.
    ///
    /// `watching` is whether the walking/standing networks have the robot (or would, but for
    /// this pause): anything else — disabled, homing, a skill, a sit, a limp-fall, the shutdown
    /// sit — owns the robot outright, and then the watch lets go of any pause and forgets its
    /// window. The window restarts empty because a second of history recorded under some other
    /// controller is not what the model was trained to read.
    ///
    /// `fresh` is this tick's sample, or `None` on a dropped read: a coasted repeat is not a
    /// new row, and the latch only moves on rows.
    pub fn tick(
        &mut self,
        watching: bool,
        fresh: Option<&Sensors>,
        period: Duration,
    ) -> Option<Edge> {
        if !watching {
            let was_paused = self.latch.paused();
            self.forget();
            return was_paused.then_some(Edge::Resume);
        }
        let (sensors, (previous_target, previous_paused)) = match (fresh, self.previous) {
            (Some(sensors), Some(previous)) => (sensors, previous),
            _ => return None,
        };
        let row = features(
            &sensors.imu,
            &sensors.positions,
            &sensors.velocities,
            &sensors.currents_ma,
            &previous_target,
            previous_paused,
        );
        match self.detector.observe(row) {
            Ok(Some(p)) => {
                self.p_held = Some(p);
                let edge = self.latch.update(p, period);
                if edge == Some(Edge::Pause) {
                    self.pause_from = previous_target;
                }
                edge
            }
            Ok(None) => None,
            Err(e) => {
                // A detector that cannot score must not hold the robot paused: let go and
                // start over, which costs a second of warm-up and nothing else.
                tracing::warn!(error = %e, "pickup detector failed; resuming and restarting it");
                let was_paused = self.latch.paused();
                self.forget();
                was_paused.then_some(Edge::Resume)
            }
        }
    }

    /// What a paused robot is commanded this tick.
    pub fn target(&self) -> [f64; NUM_JOINTS] {
        pause_target(&self.pause_from, self.latch.paused_for())
    }

    /// Whether the pause ramp is still travelling.
    pub fn ramping(&self) -> bool {
        self.latch.paused_for() < duck_control::pickup::PAUSE_RAMP
    }

    /// Record what the loop actually commanded this tick — the next row's "previous target".
    pub fn commanded(&mut self, targets: &[f64; NUM_JOINTS]) {
        self.previous = Some((*targets, self.latch.paused()));
    }

    fn forget(&mut self) {
        self.latch.reset();
        self.detector.reset();
        self.previous = None;
        self.p_held = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use duck_control::DEFAULT_POSITION;
    use duck_control::pickup::{FEAT_LEN, PAUSE_RAMP};

    const TICK: Duration = Duration::from_millis(20);

    /// Scores whatever it is told to, after a two-row warm-up, and keeps the rows.
    #[derive(Default)]
    struct Scripted {
        p: f32,
        rows: Vec<[f32; FEAT_LEN]>,
        fail: bool,
    }
    impl Score for Scripted {
        fn observe(&mut self, row: [f32; FEAT_LEN]) -> Result<Option<f32>, String> {
            if self.fail {
                return Err("boom".into());
            }
            self.rows.push(row);
            Ok((self.rows.len() >= 2).then_some(self.p))
        }
        fn reset(&mut self) {
            self.rows.clear();
        }
    }

    fn watch(p: f32) -> Watch<Scripted> {
        Watch::new(
            Scripted {
                p,
                ..Default::default()
            },
            Thresholds::default(),
        )
    }

    fn step(w: &mut Watch<Scripted>, watching: bool, targets: [f64; NUM_JOINTS]) -> Option<Edge> {
        let edge = w.tick(watching, Some(&Sensors::default()), TICK);
        let commanded = if w.paused() { w.target() } else { targets };
        w.commanded(&commanded);
        edge
    }

    /// The row pairs this tick's reading with LAST tick's command and pause flag — the way the
    /// training rows were recorded. Off by one tick here and every row lies about cause.
    #[test]
    fn rows_carry_the_previous_ticks_command() {
        let mut w = watch(0.0);
        let mut a = DEFAULT_POSITION;
        a[0] += 0.1;
        let mut b = DEFAULT_POSITION;
        b[0] += 0.2;
        step(&mut w, true, a);
        step(&mut w, true, b);
        step(&mut w, true, b);
        let rows = &w.detector.rows;
        assert_eq!(rows.len(), 2, "no row before there is a previous command");
        assert!((rows[0][34] - 0.1).abs() < 1e-6);
        assert!((rows[1][34] - 0.2).abs() < 1e-6);
        assert_eq!(rows[0][62], 0.0);
    }

    #[test]
    fn a_held_robot_pauses_and_ramps_from_the_last_policy_target() {
        let mut w = watch(0.99);
        let mut last = DEFAULT_POSITION;
        last[2] += 0.3;
        let mut edges = vec![];
        for _ in 0..20 {
            edges.extend(step(&mut w, true, last));
        }
        assert_eq!(edges, vec![Edge::Pause]);
        assert!(w.paused());
        // The pause began within the last 20 ticks, so the ramp starts at the policy's target.
        assert!(w.target()[2] <= last[2] + 1e-12);
        // And once a paused robot's rows are recorded, they say so.
        assert_eq!(w.detector.rows.last().unwrap()[62], 1.0);
    }

    /// Anything else taking the robot — a skill, a disable, a limp-fall — ends the pause at
    /// once, and the next decision waits for a fresh window.
    #[test]
    fn losing_the_robot_ends_the_pause_and_forgets_the_window() {
        let mut w = watch(0.99);
        for _ in 0..20 {
            step(&mut w, true, DEFAULT_POSITION);
        }
        assert!(w.paused());
        assert_eq!(step(&mut w, false, DEFAULT_POSITION), Some(Edge::Resume));
        assert!(!w.paused());
        assert!(w.detector.rows.is_empty());
        assert_eq!(
            step(&mut w, false, DEFAULT_POSITION),
            None,
            "one edge, not one per tick"
        );
    }

    /// A detector that errors must never strand the robot paused.
    #[test]
    fn a_failing_detector_lets_go() {
        let mut w = watch(0.99);
        for _ in 0..20 {
            step(&mut w, true, DEFAULT_POSITION);
        }
        w.detector.fail = true;
        assert_eq!(step(&mut w, true, DEFAULT_POSITION), Some(Edge::Resume));
        assert!(!w.paused());
    }

    /// A dropped read is not a row: the latch must not advance on a repeated sample.
    #[test]
    fn a_dropped_read_adds_no_row() {
        let mut w = watch(0.0);
        step(&mut w, true, DEFAULT_POSITION);
        step(&mut w, true, DEFAULT_POSITION);
        let before = w.detector.rows.len();
        assert_eq!(w.tick(true, None, TICK), None);
        assert_eq!(w.detector.rows.len(), before);
    }

    /// Off means off: not even the model file is looked at — this returns before any load,
    /// so a robot that never opted in pays nothing, not even ONNX Runtime's start-up.
    #[test]
    fn switched_off_builds_nothing() {
        let off = crate::params::PickupParams {
            enabled: false,
            model: Some("/nonexistent/pickup.onnx".into()),
            ..Default::default()
        };
        assert!(Watch::build(&off).is_none());
        let no_model = crate::params::PickupParams {
            enabled: true,
            model: Some("none".into()),
            ..Default::default()
        };
        assert!(Watch::build(&no_model).is_none());
        assert!(crate::params::PickupParams::default().enabled, "ships on");
    }

    #[test]
    fn the_ramp_ends_after_its_duration() {
        let mut w = watch(0.99);
        let mut ticks = 0;
        while !w.paused() {
            step(&mut w, true, DEFAULT_POSITION);
            ticks += 1;
            assert!(ticks < 100);
        }
        assert!(w.ramping());
        for _ in 0..(PAUSE_RAMP.as_millis() / TICK.as_millis()) {
            step(&mut w, true, DEFAULT_POSITION);
        }
        assert!(!w.ramping());
    }
}
