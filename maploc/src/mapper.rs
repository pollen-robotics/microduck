//! The mapping host loop — stillness, windows, the tracking watchdog and
//! kidnap recovery — shared by robotd's worker and the offline bench.
//!
//! [`crate::pipeline::Slam`] exists so the graph wiring is written once;
//! this module exists for the same reason one level up. The still
//! detector, the window flush rules, the quality gates and the
//! lost/relocalize state machine used to live in robotd's worker, with the
//! replay bench keeping a hand-mirrored copy — the exact arrangement whose
//! drift the pipeline module was created to prevent. A ground-truth
//! recording is only worth its disk space if the bench replays it through
//! the *same* decisions the robot made, so the decisions moved here and
//! both hosts drive this.
//!
//! The state machine, in one paragraph: scans integrate only through
//! vetted still windows (see [`crate::accumulator`]); before a window
//! inks, it is scored against the map at the tracked pose
//! ([`crate::relocalize::score_pose`]) and a window the map can judge but
//! flatly contradicts flips the mapper to *lost* — a kidnapped robot's
//! scans land in territory the map knows and disagree everywhere, while a
//! robot exploring a new room lands in territory the map cannot judge and
//! keeps mapping. While lost, nothing inks and every window becomes a
//! brute-force relocalize attempt ([`crate::relocalize`]); an accepted
//! pose snaps tracking there and mapping resumes. The same watchdog heals
//! a resumed session whose robot moved while the daemon was down.

use crate::accumulator::{AccumulatorConfig, WindowAccumulator};
use crate::grid::OccupancyGrid;
use crate::pipeline::Slam;
use crate::pose_graph::{between, compose, wrap_pi};
use crate::relocalize::{RelocalizeConfig, relocalize_against_grid, score_pose};
use crate::submap::{Pose2, Scan};

/// Stillness from odometry itself, not just the host's moving flag: a
/// robot pushed by hand is moving whatever the control loop asked for.
#[derive(Debug, Clone, Copy)]
pub struct StillConfig {
    /// Displacement window length.
    pub window_s: f32,
    /// Max translation across the window to count as still.
    pub max_dxy_m: f32,
    /// Max |yaw| across the window to count as still.
    pub max_dyaw_rad: f32,
}

impl Default for StillConfig {
    fn default() -> Self {
        Self {
            window_s: 0.5,
            max_dxy_m: 0.01,
            max_dyaw_rad: 0.05,
        }
    }
}

/// The tracking watchdog's thresholds. All three must hold to declare
/// tracking lost — the bar is deliberately high, because a false "lost"
/// stops the map cold until a relocalize succeeds.
#[derive(Debug, Clone, Copy)]
pub struct WatchdogConfig {
    /// The map must be able to judge at least this many beams.
    pub min_observed_beams: u32,
    /// ... and at least this fraction of the window's beams. A floor, not
    /// a majority: a kidnapped robot mostly paints new territory (12 % of
    /// beams judged, measured), and the verdict lives in the judged beams
    /// — an explorer's judged beams agree with the map, a kidnapped
    /// robot's contradict it.
    pub min_observed_fraction: f32,
    /// Mean residual over the judged beams above which the window is a
    /// contradiction, not noise. Map noise floor is ~0.05–0.09 m; honest
    /// inter-stop drift stays well under 0.2 m.
    pub max_mean_residual_m: f32,
    /// Per-beam residual clamp for the score.
    pub clamp_m: f32,
    /// A cell is a wall for the distance field past this. 150 matches the
    /// wire frame's wall definition: one double-inked window (2 × 85)
    /// qualifies, so a thinly-mapped revisit is not scored against a
    /// field that pretends its own walls are not there.
    pub wall_threshold_fp: i16,
    /// A cell is *observed* (judgeable) past this |log-odds|.
    pub observed_fp: i16,
    /// Consecutive contradicting windows before tracking is declared
    /// lost. The first contradiction is quarantined (not inked) — one
    /// window can be a lean, a passer-by, or fresh phantom ink; a kidnap
    /// contradicts on every window.
    pub lost_after_windows: u32,
}

impl Default for WatchdogConfig {
    fn default() -> Self {
        Self {
            min_observed_beams: 100,
            min_observed_fraction: 0.05,
            max_mean_residual_m: 0.25,
            clamp_m: 0.5,
            wall_threshold_fp: 150,
            observed_fp: 50,
            lost_after_windows: 2,
        }
    }
}

#[derive(Debug, Clone)]
pub struct MapperConfig {
    /// `false` = stop-and-scan (windows, votes, gates); `true` = ink every
    /// frame directly (more coverage, blurrier walls, no watchdog).
    pub continuous: bool,
    pub accumulator: AccumulatorConfig,
    pub still: StillConfig,
    /// A still window flushes after this long even if the stand continues,
    /// so the map builds while you watch it.
    pub window_flush_after_s: f32,
    /// A vetted window with fewer beams is discarded, not inked — a seated
    /// robot's floor-clutter windows measured 2–27 beams; a real stop
    /// measures in the hundreds.
    pub min_window_beams: usize,
    /// How many times a vetted window inks. One pass writes log-odds 85
    /// per wall cell and a wall starts at 150, so a lap that stops once
    /// per spot would paint itself invisibly; a window has survived
    /// per-cell frame voting and is worth more than one raw frame.
    pub window_ink_passes: usize,
    pub watchdog: WatchdogConfig,
    pub relocalize: RelocalizeConfig,
    /// A relocalize probe is the composite decimated to at most this many
    /// beams: a window composite carries thousands, and the brute-force
    /// search is O(cells × yaws × beams) — full composites would cost
    /// seconds per attempt on the robot for no accuracy the search needs.
    pub relocalize_max_beams: usize,
    /// A relocalize candidate never snaps tracking by itself: the NEXT
    /// window, moved to the candidate-implied pose via odometry, must
    /// also agree with the map this well. A wrong basin in a young map
    /// scored 0.022 on the search's own probe (measured, field test
    /// four) and poisoned everything after; a second, independent window
    /// from a slightly different moment is what a coincidence fails.
    pub relocalize_confirm_max_residual_m: f32,
    /// ... and the map must be able to judge at least this fraction of
    /// the confirming window. Through the ToF's keyhole, a wall wedge
    /// aliases onto any other wall at the same range — the measured
    /// kidnap landed 204 of 1680 beams on old walls at residual 0.005
    /// while 0.3 m off the truth. A window that sees the scene it claims
    /// to stand in gets judged on half its beams, not an eighth.
    pub relocalize_confirm_min_fraction: f32,
    /// When suspicion came from a sit, a fall or a session resume (soft —
    /// nothing has CONTRADICTED the pose) and this many windows could not
    /// be judged either way (unmapped view), give up and resume at the
    /// odometry-carried pose. Without an escape, a robot that sits facing
    /// an unmapped corner stays "searching" forever; with evidence of
    /// displacement the escape never applies.
    pub suspect_give_up_windows: u32,
}

impl Default for MapperConfig {
    fn default() -> Self {
        Self {
            continuous: false,
            accumulator: AccumulatorConfig::default(),
            still: StillConfig::default(),
            window_flush_after_s: 3.0,
            min_window_beams: 60,
            window_ink_passes: 2,
            watchdog: WatchdogConfig::default(),
            relocalize: RelocalizeConfig {
                // Align the search's idea of a wall with the watchdog's
                // (and the 2×-ink reality) — the stock 200 was tuned on
                // prototype captures inked far more than twice.
                wall_threshold_fp: 150,
                // A composite worth relocalizing on carries ≥ 60 beams
                // (min_window_beams); demanding that many *in the map*
                // kills the measured failure mode where a 44-beam wedge
                // "accepted" a pose across the room.
                min_beams_used: 60,
                ..RelocalizeConfig::default()
            },
            relocalize_max_beams: 256,
            relocalize_confirm_max_residual_m: 0.10,
            relocalize_confirm_min_fraction: 0.3,
            suspect_give_up_windows: 10,
        }
    }
}

/// One control-loop tick's worth of the robot, as the mapper needs it.
/// (Gravity, trunk height and head joints feed the *reprojection*, which
/// stays in the host — this crate never links the kinematics.)
#[derive(Debug, Clone, Copy)]
pub struct MapperSample {
    pub odom: Pose2,
    /// The host's "the robot is doing something" verdict.
    pub moving: bool,
    /// Seated: never map from sitting height — the ToF sees knees and
    /// floor clutter, and the ground-truth protocol uses the sit as its
    /// kidnap marker.
    pub sitting: bool,
    /// Fallen over: a fall can displace and rotate the robot, and the
    /// scans from the floor are garbage anyway.
    pub fallen: bool,
}

/// What one call did — the host turns these into log lines; the bench
/// turns them into metrics. Data, not strings, so both can.
#[derive(Debug, Clone, Copy)]
pub enum Note {
    WindowIntegrated {
        beams: usize,
        windows: u32,
        /// The watchdog's agreement score for this window (residual over
        /// the beams the map could judge, that count, and the window's
        /// total) — diagnostics the bench plots to tune the thresholds.
        mean_residual_m: f32,
        n_observed: u32,
        n_beams: u32,
    },
    WindowDiscarded {
        beams: usize,
    },
    /// A first contradicting window: not inked, not yet lost.
    WindowQuarantined {
        mean_residual_m: f32,
        n_observed: u32,
    },
    /// The search proposed a pose; the next window must confirm it.
    RelocalizeCandidate {
        pose: Pose2,
        mean_residual_m: f32,
    },
    /// The robot sat: it may have been carried, and neither odometry nor
    /// a keyhole ToF view can prove it was not (a kidnapped wall wedge
    /// aliases onto any wall, measured). The pose is suspect until a
    /// window confirms it — the current pose is pre-seeded as the
    /// relocalize candidate, so an unmoved robot confirms in one window.
    SuspectAfterSit,
    /// The robot fell: same treatment as a sit — a fall can drag and spin.
    SuspectAfterFall,
    /// Soft suspicion (sit/fall/resume) expired: nothing could judge the
    /// pose either way for `suspect_give_up_windows` windows, so tracking
    /// resumed at the odometry-carried pose, unverified.
    ResumedUnverified {
        pose: Pose2,
    },
    /// The map could judge this window and flatly contradicts it.
    LostTracking {
        mean_residual_m: f32,
        n_observed: u32,
    },
    Relocalized {
        pose: Pose2,
        mean_residual_m: f32,
    },
    RelocalizeRejected {
        best_pose: Pose2,
        mean_residual_m: f32,
    },
    LoopClosed {
        n_loops: usize,
        dx: f32,
        dy: f32,
        dyaw: f32,
    },
}

/// One confirmation attempt's outcome: a candidate pose checked against a
/// fresh window. Confirmed = the window agrees at the implied pose;
/// refuted = the map judged it and said no (real evidence of
/// displacement); ambiguous = the map judged some beams and the verdict
/// fell between agreement and contradiction — keep searching, but this is
/// NOT a window the map had no opinion about; unjudgeable = the view
/// lands where the map truly has no opinion, the only kind of window the
/// give-up escape may consume.
enum Verdict {
    Confirmed(Pose2, f32),
    Refuted,
    Ambiguous,
    Unjudgeable,
}

pub struct Mapper {
    cfg: MapperConfig,
    slam: Slam,
    acc: WindowAccumulator,
    /// (t_s, x, y, yaw) over the last `still.window_s`.
    odom_window: Vec<(f32, f32, f32, f32)>,
    was_still: bool,
    window_opened: Option<f32>,
    windows: u32,
    lost: bool,
    /// Consecutive contradicting windows so far (reset by any agreeing one).
    suspect: u32,
    /// A relocalize candidate awaiting confirmation: (candidate pose, the
    /// tracked pose when it was proposed — odometry deltas since then move
    /// the candidate along with the robot).
    pending_reloc: Option<(Pose2, Pose2)>,
    /// The "I was not moved" hypothesis, when suspicion is soft (sit,
    /// fall, session resume — no scan has contradicted the pose). Same
    /// (pose, tracked-then) carrying as `pending_reloc`. Cleared when a
    /// judgeable window REFUTES it: that is evidence of displacement, and
    /// the give-up escape must never fire after evidence.
    soft_seed: Option<(Pose2, Pose2)>,
    /// Windows since suspicion that could not be judged either way.
    unjudged: u32,
    /// Consecutive windows that AGREED with the soft seed. Two are needed
    /// before tracking resumes on it — see the seed-confirmation comment.
    seed_agreed: u32,
    /// The map as it stood when the current stand began — what the
    /// watchdog judges the stand's windows against. Judging against the
    /// LIVE map lets a kidnapped stand vouch for itself: its first window
    /// paints the kidnapper's room, and every following window then
    /// "agrees with the map" it just painted (measured: vs-map 0.005
    /// while vs-truth 0.3–0.5). Ink earned during a stand never testifies
    /// for that stand.
    stand_grid: Option<OccupancyGrid>,
    /// The last window handed to `absorb_window`, whatever became of it —
    /// a bench inspects it to score composites against ground truth. One
    /// composite clone per window; noise next to the integration itself.
    last_window: Option<(Pose2, Scan)>,
}

impl Mapper {
    pub fn new(cfg: MapperConfig, slam: Slam) -> Self {
        let mut mapper = Self {
            acc: WindowAccumulator::new(cfg.accumulator),
            cfg,
            slam,
            odom_window: Vec::new(),
            was_still: false,
            window_opened: None,
            windows: 0,
            lost: false,
            suspect: 0,
            pending_reloc: None,
            soft_seed: None,
            unjudged: 0,
            seed_agreed: 0,
            stand_grid: None,
            last_window: None,
        };
        // A resumed session cannot vouch for its pose: the robot may have
        // been moved, or even booted in another room, while the daemon was
        // down. Suspect until a window confirms — an unmoved robot
        // confirms in one. (A fresh mapper has nothing to confirm against
        // and starts trusting, as it must.)
        if mapper.slam.n_submaps() > 0 {
            mapper.arm_suspicion();
        }
        mapper
    }

    /// Soft suspicion: keep tracking odometry, ink nothing, and let the
    /// windows either confirm the carried pose, refute it (→ search), or
    /// exhaust the give-up budget.
    fn arm_suspicion(&mut self) {
        self.lost = true;
        self.suspect = 0;
        self.unjudged = 0;
        self.seed_agreed = 0;
        let here = self.slam.tracked();
        self.soft_seed = Some((here, here));
        self.pending_reloc = None;
    }

    pub fn slam(&self) -> &Slam {
        &self.slam
    }
    pub fn slam_mut(&mut self) -> &mut Slam {
        &mut self.slam
    }
    pub fn windows(&self) -> u32 {
        self.windows
    }
    pub fn still(&self) -> bool {
        self.was_still
    }
    /// False while lost (kidnapped, or a resumed session the scans refute).
    pub fn tracking(&self) -> bool {
        !self.lost
    }
    /// Frames sitting in the open still window.
    pub fn window_frames(&self) -> usize {
        self.acc.len()
    }
    /// The pose and composite of the last closed window (see field doc).
    pub fn last_window(&self) -> Option<&(Pose2, Scan)> {
        self.last_window.as_ref()
    }

    /// One control-loop tick. `t_s` is seconds on any monotonic timebase —
    /// the host's uptime, a recording's timestamps — as long as one mapper
    /// sees only one. Notes are appended, not replaced.
    pub fn observe(&mut self, t_s: f32, sample: MapperSample, notes: &mut Vec<Note>) {
        self.slam.observe_odom(sample.odom);
        self.odom_window
            .push((t_s, sample.odom.0, sample.odom.1, sample.odom.2));
        let horizon = self.cfg.still.window_s;
        self.odom_window.retain(|&(at, ..)| t_s - at <= horizon);

        let still = !sample.moving
            && !sample.sitting
            && !sample.fallen
            && self.odom_window.first().is_some_and(|&(_, fx, fy, fyaw)| {
                let dx = sample.odom.0 - fx;
                let dy = sample.odom.1 - fy;
                let dyaw = wrap_pi(sample.odom.2 - fyaw);
                (dx * dx + dy * dy).sqrt() < self.cfg.still.max_dxy_m
                    && dyaw.abs() < self.cfg.still.max_dyaw_rad
            });

        // A window flushes when the stand ends — or after
        // `window_flush_after_s` while it continues, so the map builds
        // while you watch instead of waiting for the next step. The window
        // closes here whatever comes of it: leaving it armed after a
        // fruitless finish would flush every subsequent frame alone.
        let stand_ended = self.was_still && !still;
        let ripe = self
            .window_opened
            .is_some_and(|t0| t_s - t0 >= self.cfg.window_flush_after_s);
        if (stand_ended || ripe) && !self.acc.is_empty() {
            self.window_opened = None;
            if let Some((pose, composite)) = self.acc.finish() {
                self.absorb_window(pose, &composite, t_s, notes);
            }
        }
        if stand_ended {
            self.window_opened = None;
        }
        if still && !self.was_still {
            // A stand begins: freeze the map the watchdog will judge this
            // stand's windows against.
            self.stand_grid = self.slam.render();
        }
        self.was_still = still;

        // A sit or a fall invalidates the pose: the robot cannot feel a
        // carry, and a fall can drag and spin it. Arm the lost machinery
        // with "I was not moved" as the seed — cheap to confirm when
        // true, refused when false. AFTER the window flush above, on
        // purpose: the window that closes at the sit describes the world
        // BEFORE the carry, and letting it count as the seed's first
        // agreement handed a real kidnap half its confirmation for free
        // (measured — field test five's second carry).
        if (sample.sitting || sample.fallen) && !self.lost {
            self.arm_suspicion();
            notes.push(if sample.fallen {
                Note::SuspectAfterFall
            } else {
                Note::SuspectAfterSit
            });
        }

        // While lost the tracked pose is a guess; freezing submaps or
        // running closures on it would launder the guess into the graph.
        if !self.lost {
            let loops_before = self.slam.n_loops();
            let before = self.slam.tracked();
            self.slam.tick(t_s);
            if self.slam.n_loops() > loops_before {
                let after = self.slam.tracked();
                notes.push(Note::LoopClosed {
                    n_loops: self.slam.n_loops(),
                    dx: after.0 - before.0,
                    dy: after.1 - before.1,
                    dyaw: wrap_pi(after.2 - before.2),
                });
                // The closure moved every anchor; a snapshot in the old
                // frame would mis-judge the stand's remaining windows by
                // exactly the correction — and the frames already pushed
                // carry PRE-correction poses: a composite mixing both
                // frames would ink smeared and displaced. Drop them; the
                // stand refills the window in a couple of seconds.
                if self.was_still {
                    self.stand_grid = self.slam.render();
                }
                if !self.acc.is_empty() {
                    self.acc = WindowAccumulator::new(self.cfg.accumulator);
                    self.window_opened = None;
                }
            }
        }
    }

    /// One reprojected depth frame, already in the body frame. Returns
    /// true when the frame was kept (accumulated or inked).
    pub fn frame(&mut self, t_s: f32, scan: Scan) -> bool {
        if self.cfg.continuous && !self.lost {
            self.slam.integrate(self.slam.tracked(), &scan);
            return true;
        }
        // Continuous mode falls through here while LOST: recovery is the
        // still-window machinery in both modes, or a continuous mapper
        // that sat once would sweep its head forever with no path back.
        if !self.was_still {
            return false;
        }
        if self.acc.is_empty() {
            self.window_opened = Some(t_s);
        }
        self.acc.push(self.slam.tracked(), scan);
        true
    }

    fn absorb_window(&mut self, pose: Pose2, composite: &Scan, t_s: f32, notes: &mut Vec<Note>) {
        self.last_window = Some((pose, composite.clone()));
        let beams = composite.n_valid();
        if beams < self.cfg.min_window_beams {
            notes.push(Note::WindowDiscarded { beams });
            return;
        }

        if self.lost {
            let Some(mut grid) = self.slam.render() else {
                return;
            };
            let now = self.slam.tracked();

            // The soft seed first: "I was not moved" outranks any search
            // candidate while it stands unrefuted. It must agree with TWO
            // windows before tracking resumes on it: a single static wedge
            // falsely confirmed a real kidnap in the field (residual 0.010
            // at the old pose — the new spot's wall matched the old spot's
            // wall), and the head sweep only decorrelates the second
            // window from the first if we wait for it.
            if let Some((cand, then)) = self.soft_seed.take() {
                match self.check_candidate(&mut grid, composite, cand, then, now) {
                    (implied, Verdict::Confirmed(pose, resid)) => {
                        self.seed_agreed += 1;
                        if self.seed_agreed >= 2 {
                            self.resume_at(pose, composite, t_s);
                            notes.push(Note::Relocalized {
                                pose,
                                mean_residual_m: resid,
                            });
                            return;
                        }
                        self.soft_seed = Some((implied, now));
                        notes.push(Note::RelocalizeCandidate {
                            pose,
                            mean_residual_m: resid,
                        });
                        return;
                    }
                    (_, Verdict::Refuted) => {
                        // Evidence of displacement: suspicion hardens, the
                        // give-up escape is off the table.
                        self.seed_agreed = 0;
                    }
                    (implied, Verdict::Ambiguous) => {
                        // Keep the hypothesis alive and keep looking, but
                        // spend none of the give-up budget on a window the
                        // map DID judge.
                        self.seed_agreed = 0;
                        self.soft_seed = Some((implied, now));
                    }
                    (implied, Verdict::Unjudgeable) => {
                        self.seed_agreed = 0;
                        self.unjudged += 1;
                        if self.unjudged >= self.cfg.suspect_give_up_windows {
                            self.resume_at(implied, composite, t_s);
                            notes.push(Note::ResumedUnverified { pose: implied });
                            return;
                        }
                        self.soft_seed = Some((implied, now));
                    }
                }
            }

            // Then the search's last candidate, if one is pending. (Already
            // two independent windows: one nominated it, this one judges.)
            if let Some((cand, then)) = self.pending_reloc.take()
                && let (_, Verdict::Confirmed(pose, resid)) =
                    self.check_candidate(&mut grid, composite, cand, then, now)
            {
                self.resume_at(pose, composite, t_s);
                notes.push(Note::Relocalized {
                    pose,
                    mean_residual_m: resid,
                });
                return;
            }

            // No confirmation: search this window for a fresh candidate.
            let probe = composite.decimated(self.cfg.relocalize_max_beams);
            match relocalize_against_grid(&mut grid, &probe, &self.cfg.relocalize) {
                Some(r) if r.accepted => {
                    self.pending_reloc = Some((r.pose, now));
                    notes.push(Note::RelocalizeCandidate {
                        pose: r.pose,
                        mean_residual_m: r.mean_residual_m,
                    });
                }
                Some(r) => notes.push(Note::RelocalizeRejected {
                    best_pose: r.pose,
                    mean_residual_m: r.mean_residual_m,
                }),
                None => {}
            }
            return;
        }

        // The watchdog: score the window against the map before believing
        // it. A window the map can judge but contradicts must not ink — it
        // would paint the kidnapper's room over the real one.
        let wd = self.cfg.watchdog;
        let mut agreement = (0.0_f32, 0u32, beams as u32);
        if let Some(grid) = self.stand_grid.as_mut() {
            let a = score_pose(
                grid,
                composite,
                pose,
                wd.clamp_m,
                wd.wall_threshold_fp,
                wd.observed_fp,
            );
            agreement = (a.mean_residual_m, a.n_observed, a.n_beams);
            if a.n_observed >= wd.min_observed_beams
                && a.n_observed as f32 >= wd.min_observed_fraction * a.n_beams as f32
                && a.mean_residual_m > wd.max_mean_residual_m
            {
                // Contradicting window: never ink it. One is a suspect
                // (a lean, a passer-by, fresh phantom ink); a run of them
                // is a kidnap.
                self.suspect += 1;
                if self.suspect >= wd.lost_after_windows {
                    self.lost = true;
                    self.suspect = 0;
                    self.pending_reloc = None;
                    notes.push(Note::LostTracking {
                        mean_residual_m: a.mean_residual_m,
                        n_observed: a.n_observed,
                    });
                } else {
                    notes.push(Note::WindowQuarantined {
                        mean_residual_m: a.mean_residual_m,
                        n_observed: a.n_observed,
                    });
                }
                return;
            }
            self.suspect = 0;
        }
        self.ink(pose, composite);
        notes.push(Note::WindowIntegrated {
            beams,
            windows: self.windows,
            mean_residual_m: agreement.0,
            n_observed: agreement.1,
            n_beams: agreement.2,
        });
    }

    /// Tracking resumes at `pose`; the window that earned it inks there.
    fn resume_at(&mut self, pose: Pose2, composite: &Scan, t_s: f32) {
        self.slam.set_tracked(pose);
        // Let the submap manager see the jump BEFORE inking: after a
        // cross-room carry the current submap's grid is still anchored at
        // the pre-carry pose, and a composite integrated there is silently
        // clipped to nothing — the travel rule opens (or re-anchors to) a
        // submap that actually covers where the robot now stands.
        self.slam.tick(t_s);
        self.ink(pose, composite);
        self.lost = false;
        self.suspect = 0;
        self.unjudged = 0;
        self.seed_agreed = 0;
        self.soft_seed = None;
        self.pending_reloc = None;
    }

    /// Check one candidate pose against a fresh window, carried to the
    /// candidate-implied pose by the odometry accumulated since it was
    /// proposed. (A plain method, not a closure — a four-parameter closure
    /// is exactly the construct two rustfmt versions disagree about.)
    fn check_candidate(
        &self,
        grid: &mut OccupancyGrid,
        composite: &Scan,
        cand: Pose2,
        tracked_then: Pose2,
        tracked_now: Pose2,
    ) -> (Pose2, Verdict) {
        let wd = self.cfg.watchdog;
        let implied = compose(cand, between(tracked_then, tracked_now));
        let a = score_pose(
            grid,
            composite,
            implied,
            wd.clamp_m,
            wd.wall_threshold_fp,
            wd.observed_fp,
        );
        // Two coverage floors on purpose. CONFIRMING needs the strict one
        // (relocalize_confirm_min_fraction): through the ToF keyhole a thin
        // wedge aliases, so agreement on an eighth of the beams is a
        // coincidence. REFUTING only needs the watchdog's floor: the
        // measured kidnap judged 12 % of its beams and contradicted on all
        // of them — a verdict of "cannot judge" there let the give-up
        // escape resume tracking at the kidnapped pose.
        let strong = a.n_observed >= wd.min_observed_beams
            && a.n_observed as f32 >= self.cfg.relocalize_confirm_min_fraction * a.n_beams as f32;
        let weak = a.n_observed >= wd.min_observed_beams
            && a.n_observed as f32 >= wd.min_observed_fraction * a.n_beams as f32;
        let verdict = if strong && a.mean_residual_m <= self.cfg.relocalize_confirm_max_residual_m {
            Verdict::Confirmed(implied, a.mean_residual_m)
        } else if strong || (weak && a.mean_residual_m > wd.max_mean_residual_m) {
            Verdict::Refuted
        } else if weak {
            // Judged, and the judgement fell between agreement and
            // contradiction: evidence exists but is ambiguous. Keep the
            // candidate and keep searching — but this window must not
            // count toward the give-up escape, whose premise is that the
            // map could not judge the pose AT ALL.
            Verdict::Ambiguous
        } else {
            Verdict::Unjudgeable
        };
        (implied, verdict)
    }

    fn ink(&mut self, pose: Pose2, composite: &Scan) {
        self.slam
            .integrate_weighted(pose, composite, self.cfg.window_ink_passes.max(1));
        self.windows += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::{Slam, SlamConfig};

    /// What the sensor sees standing at `pose` (the robot's TRUE pose) in
    /// a rectangular room (walls at x = ±1.5, y = ±1.1): body-frame
    /// angles, true ranges. Rectangular on purpose — a square room is
    /// 90°-symmetric and a relocalizer handed one is *entitled* to pick a
    /// rotated pose.
    /// Where the mapper paints them is its own business — that gap is what
    /// the kidnap test exercises.
    fn room_scan(pose: Pose2) -> Scan {
        let mut angles = Vec::new();
        let mut ranges = Vec::new();
        for k in 0..240 {
            let a = -std::f32::consts::PI + k as f32 * (2.0 * std::f32::consts::PI / 240.0);
            let (dx, dy) = ((pose.2 + a).cos(), (pose.2 + a).sin());
            let tx = if dx > 1e-6 {
                (1.5 - pose.0) / dx
            } else if dx < -1e-6 {
                (-1.5 - pose.0) / dx
            } else {
                f32::INFINITY
            };
            let ty = if dy > 1e-6 {
                (1.1 - pose.1) / dy
            } else if dy < -1e-6 {
                (-1.1 - pose.1) / dy
            } else {
                f32::INFINITY
            };
            let mut r = tx.min(ty);
            // A half-divider at x = 0.3, y ∈ [-1.1, 0] breaks the
            // rectangle's remaining 180° symmetry.
            if dx.abs() > 1e-6 {
                let td = (0.3 - pose.0) / dx;
                if td > 0.0 {
                    let y_hit = pose.1 + td * dy;
                    if (-1.1..=0.0).contains(&y_hit) {
                        r = r.min(td);
                    }
                }
            }
            if r.is_finite() && r < 1.9 {
                angles.push(a);
                ranges.push(r);
            }
        }
        Scan::from_polar(&angles, &ranges, (0.0, 0.0), 1e-3)
    }

    fn drive(
        mapper: &mut Mapper,
        t0: f32,
        pose: Pose2,
        seconds: f32,
        notes: &mut Vec<Note>,
    ) -> f32 {
        // 50 Hz odometry, 15 Hz frames, robot standing at `pose`.
        let mut t = t0;
        let end = t0 + seconds;
        let mut next_frame = t0;
        while t < end {
            mapper.observe(
                t,
                MapperSample {
                    odom: pose,
                    moving: false,
                    sitting: false,
                    fallen: false,
                },
                notes,
            );
            if t >= next_frame {
                mapper.frame(t, room_scan(pose));
                next_frame += 1.0 / 15.0;
            }
            t += 0.02;
        }
        t
    }

    /// The whole point of the machine: a kidnap (scans that contradict the
    /// map) flips tracking to lost, nothing inks, and a good window
    /// relocalizes back to the true pose.
    #[test]
    fn a_kidnapped_mapper_stops_relocates_and_resumes() {
        let mut mapper = Mapper::new(MapperConfig::default(), Slam::new(SlamConfig::default()));
        let mut notes = Vec::new();

        // Build a map from two stands — a second viewpoint fills the
        // divider's occlusion shadow, exactly like a real mapping lap
        // does; a single-viewpoint map penalizes the true post-kidnap
        // pose for beams into the territory only the kidnapper's side of
        // the room can see.
        let mut t = drive(&mut mapper, 0.0, (0.0, 0.0, 0.0), 8.0, &mut notes);
        t = drive(&mut mapper, t, (0.9, -0.6, -1.2), 8.0, &mut notes);
        assert!(mapper.windows() >= 2, "the stands must have inked windows");
        assert!(mapper.tracking());

        // The carry: the robot is SAT, carried, and stood back up — the
        // sit arms pose suspicion, which is the kidnap signal geometry
        // cannot fake through the ToF keyhole.
        for _ in 0..50 {
            mapper.observe(
                t,
                MapperSample {
                    odom: (0.0, 0.0, 0.0),
                    moving: false,
                    sitting: true,
                    fallen: false,
                },
                &mut notes,
            );
            t += 0.02;
        }
        assert!(!mapper.tracking(), "a sit must make the pose suspect");

        // Kidnap: odometry still reads the origin, but the robot now really
        // stands at (0.8, 0.5, 0.9) — its scans are the room seen from
        // there, expressed in the body frame odometry believes in.
        let truth = (0.8, 0.5, 0.9);
        let mut next_frame = t;
        let end = t + 20.0;
        let mut relocalized = None;
        while t < end {
            mapper.observe(
                t,
                MapperSample {
                    odom: (0.0, 0.0, 0.0),
                    moving: false,
                    sitting: false,
                    fallen: false,
                },
                &mut notes,
            );
            if t >= next_frame {
                mapper.frame(t, room_scan(truth));
                next_frame += 1.0 / 15.0;
            }
            for note in notes.drain(..) {
                if let Note::Relocalized { pose, .. } = note {
                    relocalized = Some(pose);
                }
            }
            if relocalized.is_some() {
                break;
            }
            t += 0.02;
        }

        let pose = relocalized.expect("the mapper must relocalize after a kidnap");
        let err = (pose.0 - truth.0).hypot(pose.1 - truth.1);
        assert!(err < 0.25, "relocalized {pose:?}, truth {truth:?}");
        assert!(wrap_pi(pose.2 - truth.2).abs() < 0.3);
        assert!(mapper.tracking());
    }

    /// A robot that sits and stands WITHOUT being moved confirms its own
    /// pose from the first window and resumes mapping.
    #[test]
    fn a_sit_in_place_recovers_in_one_window() {
        let mut mapper = Mapper::new(MapperConfig::default(), Slam::new(SlamConfig::default()));
        let mut notes = Vec::new();
        let mut t = drive(&mut mapper, 0.0, (0.0, 0.0, 0.0), 8.0, &mut notes);
        for _ in 0..100 {
            mapper.observe(
                t,
                MapperSample {
                    odom: (0.0, 0.0, 0.0),
                    moving: false,
                    sitting: true,
                    fallen: false,
                },
                &mut notes,
            );
            t += 0.02;
        }
        assert!(!mapper.tracking());
        let before = mapper.slam().tracked();
        drive(&mut mapper, t, (0.0, 0.0, 0.0), 8.0, &mut notes);
        assert!(
            mapper.tracking(),
            "an unmoved robot must confirm its pose and resume"
        );
        let confirmed = notes.iter().rev().find_map(|n| match n {
            Note::Relocalized { pose, .. } => Some(*pose),
            _ => None,
        });
        let pose = confirmed.expect("confirmation shows up as a relocalization");
        assert!((pose.0 - before.0).hypot(pose.1 - before.1) < 0.05);
    }

    /// A fall arms the same suspicion as a sit, and an unmoved robot
    /// confirms its pose and resumes.
    #[test]
    fn a_fall_makes_the_pose_suspect_and_recovers() {
        let mut mapper = Mapper::new(MapperConfig::default(), Slam::new(SlamConfig::default()));
        let mut notes = Vec::new();
        let mut t = drive(&mut mapper, 0.0, (0.0, 0.0, 0.0), 8.0, &mut notes);
        for _ in 0..50 {
            mapper.observe(
                t,
                MapperSample {
                    odom: (0.0, 0.0, 0.0),
                    moving: false,
                    sitting: false,
                    fallen: true,
                },
                &mut notes,
            );
            t += 0.02;
        }
        assert!(!mapper.tracking(), "a fall must make the pose suspect");
        drive(&mut mapper, t, (0.0, 0.0, 0.0), 8.0, &mut notes);
        assert!(
            mapper.tracking(),
            "an unmoved robot must confirm and resume"
        );
    }

    /// A resumed session boots with a suspect pose (the robot may have
    /// been moved while the daemon was off) and confirms it from the
    /// first window when it was not.
    #[test]
    fn a_resumed_session_confirms_before_inking() {
        let mut mapper = Mapper::new(MapperConfig::default(), Slam::new(SlamConfig::default()));
        let mut notes = Vec::new();
        drive(&mut mapper, 0.0, (0.0, 0.0, 0.0), 8.0, &mut notes);
        let path = std::env::temp_dir().join(format!(
            "maploc_mapper_resume_{}.session",
            std::process::id()
        ));
        mapper.slam().save(&path).expect("save");
        let restored = crate::session::SessionState::load(&path)
            .expect("load")
            .expect("present");
        std::fs::remove_file(&path).ok();

        let mut resumed = Mapper::new(
            MapperConfig::default(),
            Slam::from_session(SlamConfig::default(), restored),
        );
        assert!(
            !resumed.tracking(),
            "a resumed map must not vouch for its pose"
        );
        drive(&mut resumed, 100.0, (0.0, 0.0, 0.0), 8.0, &mut notes);
        assert!(
            resumed.tracking(),
            "booting where it saved must confirm from the first window"
        );
    }

    /// Soft suspicion facing territory the map cannot judge gives up
    /// after its budget and resumes at the odometry-carried pose —
    /// without the escape, a robot that sits facing an unmapped corner
    /// would say "searching" forever.
    #[test]
    fn soft_suspicion_gives_up_when_nothing_can_judge() {
        let mut mapper = Mapper::new(MapperConfig::default(), Slam::new(SlamConfig::default()));
        let mut notes = Vec::new();
        let mut t = drive(&mut mapper, 0.0, (0.0, 0.0, 0.0), 8.0, &mut notes);
        // Sit (suspicion), then wake up somewhere the map has never seen:
        // odometry says (5, 5) — far outside the mapped room — and the
        // scans are a fixed ring nothing can compare against.
        for _ in 0..50 {
            mapper.observe(
                t,
                MapperSample {
                    odom: (0.0, 0.0, 0.0),
                    moving: false,
                    sitting: true,
                    fallen: false,
                },
                &mut notes,
            );
            t += 0.02;
        }
        assert!(!mapper.tracking());
        let ring: Vec<f32> = (0..240)
            .map(|k| -std::f32::consts::PI + k as f32 * (2.0 * std::f32::consts::PI / 240.0))
            .collect();
        let ranges = vec![1.0f32; 240];
        let scan = || Scan::from_polar(&ring, &ranges, (0.0, 0.0), 1e-3);
        let mut next_frame = t;
        let mut resumed = None;
        let end = t + 60.0;
        while t < end {
            mapper.observe(
                t,
                MapperSample {
                    odom: (5.0, 5.0, 0.0),
                    moving: false,
                    sitting: false,
                    fallen: false,
                },
                &mut notes,
            );
            if t >= next_frame {
                mapper.frame(t, scan());
                next_frame += 1.0 / 15.0;
            }
            for n in notes.drain(..) {
                if let Note::ResumedUnverified { pose } = n {
                    resumed = Some(pose);
                }
            }
            if resumed.is_some() {
                break;
            }
            t += 0.02;
        }
        let pose = resumed.expect("soft suspicion must eventually give up");
        assert!(mapper.tracking());
        // The odometry carried the pose to (5, 5) relative to the seed.
        assert!((pose.0 - 5.0).abs() < 0.1 && (pose.1 - 5.0).abs() < 0.1);
    }

    /// Continuous mode must recover from suspicion the same way
    /// stop-and-scan does: a continuous mapper that sat once used to sweep
    /// its head forever — `lost` was armed on the sit and nothing on the
    /// continuous path could ever clear it.
    #[test]
    fn continuous_mode_recovers_from_suspicion() {
        let mut mapper = Mapper::new(
            MapperConfig {
                continuous: true,
                ..MapperConfig::default()
            },
            Slam::new(SlamConfig::default()),
        );
        let mut notes = Vec::new();
        let mut t = drive(&mut mapper, 0.0, (0.0, 0.0, 0.0), 8.0, &mut notes);
        assert!(mapper.tracking());
        for _ in 0..50 {
            mapper.observe(
                t,
                MapperSample {
                    odom: (0.0, 0.0, 0.0),
                    moving: false,
                    sitting: true,
                    fallen: false,
                },
                &mut notes,
            );
            t += 0.02;
        }
        assert!(!mapper.tracking(), "a sit must suspend continuous mapping");
        drive(&mut mapper, t, (0.0, 0.0, 0.0), 10.0, &mut notes);
        assert!(
            mapper.tracking(),
            "an unmoved continuous mapper must confirm its pose and resume"
        );
    }

    /// A displaced robot whose windows the map can judge — even on a
    /// minority of beams — must never exhaust soft suspicion into
    /// ResumedUnverified: refutation lives in the judged beams, and the
    /// give-up escape is only for views the map cannot judge at all.
    #[test]
    fn a_contradicted_seed_never_resumes_unverified() {
        let mut mapper = Mapper::new(MapperConfig::default(), Slam::new(SlamConfig::default()));
        let mut notes = Vec::new();
        let mut t = drive(&mut mapper, 0.0, (0.0, 0.0, 0.0), 8.0, &mut notes);
        for _ in 0..50 {
            mapper.observe(
                t,
                MapperSample {
                    odom: (0.0, 0.0, 0.0),
                    moving: false,
                    sitting: true,
                    fallen: false,
                },
                &mut notes,
            );
            t += 0.02;
        }
        assert!(!mapper.tracking());

        // The kidnapped view: ~85 % of beams land beyond the mapped walls
        // (unknown cells — unjudgeable), ~15 % land mid-room in carved
        // free space, far from every wall — judgeable, and contradicting.
        let mut angles = Vec::new();
        let mut ranges = Vec::new();
        for k in 0..1000 {
            let a = -std::f32::consts::PI + k as f32 * (2.0 * std::f32::consts::PI / 1000.0);
            angles.push(a);
            ranges.push(if k % 7 == 0 { 0.9 } else { 1.9 });
        }
        let scan = || Scan::from_polar(&angles, &ranges, (0.0, 0.0), 1e-3);

        let mut next_frame = t;
        let mut debug_once = true;
        let end = t + 90.0; // far past 10 windows' worth of give-up budget
        while t < end {
            mapper.observe(
                t,
                MapperSample {
                    odom: (0.0, 0.0, 0.0),
                    moving: false,
                    sitting: false,
                    fallen: false,
                },
                &mut notes,
            );
            if t >= next_frame {
                mapper.frame(t, scan());
                next_frame += 1.0 / 15.0;
            }
            if let (Some(mut g), Some((p, sc))) =
                (mapper.slam().render(), mapper.last_window().cloned())
            {
                let wd = mapper.cfg.watchdog;
                let a = crate::relocalize::score_pose(
                    &mut g,
                    &sc,
                    p,
                    wd.clamp_m,
                    wd.wall_threshold_fp,
                    wd.observed_fp,
                );
                if debug_once {
                    println!(
                        "window agreement: mean {:.3} over {}/{} ({:.0}%)",
                        a.mean_residual_m,
                        a.n_observed,
                        a.n_beams,
                        100.0 * a.n_observed as f32 / a.n_beams as f32
                    );
                    debug_once = false;
                }
            }
            for n in notes.drain(..) {
                assert!(
                    !matches!(n, Note::ResumedUnverified { .. }),
                    "a judged-and-contradicted pose must never resume unverified"
                );
                if let Note::Relocalized { pose, .. } = n {
                    panic!("nothing should confirm here, got {pose:?}");
                }
            }
            t += 0.02;
        }
        assert!(!mapper.tracking(), "the mapper must still be searching");
    }

    /// Beams into unexplored territory must never read as "lost".
    #[test]
    fn exploring_new_territory_is_not_a_kidnap() {
        let mut mapper = Mapper::new(MapperConfig::default(), Slam::new(SlamConfig::default()));
        let mut notes = Vec::new();
        let t = drive(&mut mapper, 0.0, (0.0, 0.0, 0.0), 8.0, &mut notes);
        // Face the other way from a spot the map has never judged: the
        // scans land in unknown cells.
        drive(&mut mapper, t, (0.4, -0.3, 2.5), 8.0, &mut notes);
        assert!(
            mapper.tracking(),
            "new territory must be mapped, not declared a kidnap"
        );
    }
}
