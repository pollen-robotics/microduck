//! The behaviour: a randomised state machine on an energy model.
//!
//! Ported from the runtime's `autonomous.rs` (`docs/design/autonomous.md` §4), minus BallPlay,
//! Held, Startle and Preen, and with Wander walking to goal points in the odometry frame rather
//! than holding random headings. **Pure decision-making**: it never touches a socket. Every tick it
//! takes a [`World`] and returns a [`Step`] of targets and one-shot events, and `main.rs` turns
//! those into intents. That is what lets the soak test below run ten minutes of duck in a
//! millisecond.
//!
//! The goal is believability. It should spend most of its time *not* walking, and never move in
//! straight mechanical lines.
//!
//! **Head pitch is "positive = look up"** here, the runtime's v1.5 convention. The alpha's
//! `+head_pitch` looks down, and `main.rs` flips it on the way out.

use crate::obstacle::{FRESH_S, Obstacle, REACT_M, STALE_S, STOP_M};

/// What the microphone heard this tick, already cleared of the duck's own voice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Heard {
    /// A sharp transient: a clap, a bang.
    Noise,
    /// Someone talking to the duck.
    Voice,
}

/// A voice-bank tag, as `robot.sound` names it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sound {
    Chirp,
    Coo,
}

/// Everything the brain reads, once a tick.
#[derive(Debug, Clone, Copy)]
pub struct World {
    /// Something else owns the robot right now: a skill or a sit transition is running, or the
    /// robot is down. The brain holds still and its clock stops.
    pub busy: bool,
    pub petting: bool,
    pub heard: Option<Heard>,
    /// A ground-pick policy is loaded.
    pub can_ground_pick: bool,
    /// A sit/stand policy is loaded, so naps are possible.
    pub can_sit_stand: bool,
    /// The robot is seated.
    pub sitting: bool,
    /// What the head ToF says is ahead. `None` when there is no ToF at all, and the duck then
    /// never walks: freshness can never be met.
    pub obstacle: Option<Obstacle>,
    /// Odometry `[x, y, yaw]`, for the novelty grid and wander headings.
    pub pose: [f64; 3],
}

/// One tick's output. Targets are smoothed by `robotd` (and by `head_alpha`); events are edges.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Step {
    /// `[vx, vy, vyaw]`, m/s and rad/s.
    pub cmd: [f64; 3],
    /// `[neck_pitch, head_pitch, head_yaw, head_roll]`, radians, head pitch positive up.
    pub head: [f64; 4],
    /// The per-tick EMA rate this head target wants (see `robot.head`'s `alpha`).
    pub head_alpha: f64,
    /// Body pose `[z, pitch, roll]`, meaningful while `body_active`.
    pub body: [f64; 3],
    pub body_active: bool,
    /// Mouth opening, 0..1.
    pub mouth: f64,
    pub sound: Option<Sound>,
    pub ground_pick: bool,
    /// `robot.do sit_toggle`: sit down when standing, stand up when seated.
    pub sit_toggle: bool,
    /// The wheee ride plays while this is held (zoomies).
    pub wheee: bool,
}

/// What the obstacle gate did to a walking command, one tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Gate {
    /// Fresh data, nothing close: as asked.
    Clear,
    /// The path was seen a while ago: half speed.
    Aging,
    /// The path is unseen: no advancing at all.
    Unseen,
    /// Something within `REACT_M`: slowed and steering away.
    Reacting,
    /// Something within `STOP_M`: stopped, turning away.
    Stopped,
}

/// One wander bout, for the line logged when it ends. Its job is to say *why* a duck did not get
/// far: a duck that sat at its start because the ToF never saw the path reads very differently
/// here from one that kept turning back.
#[derive(Debug, Clone, Copy, Default)]
struct Bout {
    start: [f64; 3],
    goal: [f64; 2],
    reached: bool,
    /// Metres walked by odometry — against the net displacement, how much the duck strayed off
    /// the straight line — and metres the commands asked for, which against the walked metres is
    /// how well the gait tracks the twist.
    path_m: f64,
    asked_m: f64,
    /// The closest it has been to the goal, and when (bout time) it last got meaningfully closer.
    best_left_m: f64,
    progress_at: f64,
    /// Seconds the duck wanted to walk and the gate said: as asked / half speed / not at all /
    /// slowed for something close.
    clear_s: f64,
    aging_s: f64,
    unseen_s: f64,
    reacting_s: f64,
    stopped: bool,
}

impl Bout {
    fn starting(pose: [f64; 3], goal: [f64; 2]) -> Self {
        Self {
            start: pose,
            goal,
            best_left_m: (goal[0] - pose[0]).hypot(goal[1] - pose[1]),
            ..Self::default()
        }
    }

    fn count(&mut self, gate: Gate, dt: f64) {
        match gate {
            Gate::Clear => self.clear_s += dt,
            Gate::Aging => self.aging_s += dt,
            Gate::Unseen => self.unseen_s += dt,
            Gate::Reacting => self.reacting_s += dt,
            Gate::Stopped => self.stopped = true,
        }
    }

    fn log(&self, pose: [f64; 3]) {
        let goal_m = (self.goal[0] - self.start[0]).hypot(self.goal[1] - self.start[1]);
        tracing::info!(
            covered_m = format!(
                "{:.2}",
                (pose[0] - self.start[0]).hypot(pose[1] - self.start[1])
            ),
            goal_m = format!("{goal_m:.2}"),
            left_m = format!(
                "{:.2}",
                (self.goal[0] - pose[0]).hypot(self.goal[1] - pose[1])
            ),
            reached = self.reached,
            path_m = format!("{:.2}", self.path_m),
            asked_m = format!("{:.2}", self.asked_m),
            clear_s = format!("{:.1}", self.clear_s),
            aging_s = format!("{:.1}", self.aging_s),
            unseen_s = format!("{:.1}", self.unseen_s),
            reacting_s = format!("{:.1}", self.reacting_s),
            stopped = self.stopped,
            "wander bout over"
        );
    }
}

/// Default (calm) head smoothing. Soft, so stepwise glance targets round off into animal motion.
const CALM_HEAD_ALPHA: f64 = 0.15;

/// Mouth-open animation length for one quack.
const QUACK_ANIM_S: f64 = 0.45;

/// Wander goals: how far away they are picked, metres. Far enough that a bout crosses a room
/// rather than shuffling, near enough that odometry — believed perfect until there is SLAM — has
/// not drifted much by the time the duck gets there.
const GOAL_DISTANCES: [f64; 3] = [1.0, 2.0, 3.0];
/// Directions tried per goal pick.
const GOAL_DIRECTIONS: usize = 16;
/// A goal this close is reached.
const GOAL_REACHED_M: f64 = 0.25;
/// A bout gives up on its goal after this long without getting `GOAL_PROGRESS_M` closer — a timer
/// from the distance and the commanded speed ended bouts short, because the gait does not walk
/// as fast as it is asked to, and by a margin that differs between the twin and a robot.
const GOAL_PATIENCE_S: f64 = 8.0;
const GOAL_PROGRESS_M: f64 = 0.1;
/// Even making progress, a bout ends after this: a goal it is still crawling toward is a duck
/// that should stop and look around anyway.
const BOUT_MAX_S: f64 = 90.0;
/// Inside this, the duck slows toward the goal instead of overshooting it.
const GOAL_SLOW_M: f64 = 0.4;
/// Where an obstacle stopped the duck is remembered this long (s, behaviour clock), and a goal
/// whose path passes within `BLOCKED_M` of it is not picked again — or the next goal is the same
/// wall from the same side.
const BLOCKED_S: f64 = 300.0;
const BLOCKED_M: f64 = 0.4;

/// The slowest forward command the gait actually walks at, m/s. Below it the walking policy
/// stands still with its label still saying `walk` — measured on the twin: legs at rest under
/// ~0.12 m/s, and a gait that would not *start* from a standstill at 0.14. Everything that scales
/// the walking speed down (aging ToF data, closing on an obstacle, arriving at a goal) used to land
/// there, which is a duck that stops for no visible reason and gives up on its goal.
const MIN_WALK_VX: f64 = 0.15;
/// A walk starts with this long at full speed, which is what gets the gait going.
const START_KICK_S: f64 = 0.6;

/// Novelty grid cell size (metres). Coarse on purpose: "have I hung around this half-metre
/// patch" is the right granularity for boredom.
const CELL_M: f64 = 0.5;

/// How long a ground pick may go unacknowledged before the brain decides it was refused.
/// `robotd` takes it up on the next tick, and the state stream says so a tick later, so a pick
/// that was going to run is visibly busy well inside this.
const PICK_ACK_S: f64 = 1.5;

/// Wrap an angle to (-π, π].
pub fn wrap_angle(a: f64) -> f64 {
    (a + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

/// xorshift64. Good enough for behaviour randomness, no crate needed.
struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Self(seed | 1)
    }
    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }
    /// Uniform in [0, 1).
    fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }
    fn chance(&mut self, p: f64) -> bool {
        self.unit() < p
    }
    /// Uniform in [-a, a].
    fn sym(&mut self, a: f64) -> f64 {
        self.range(-a, a)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum State {
    /// Stand still, head mostly neutral with tiny slow drifts.
    Chill,
    /// Stand still, actively glance in random directions.
    LookAround,
    /// Walk to a goal point in the odometry frame, picked toward space not yet visited.
    Wander,
    /// Rotate in place, usually to pick a new direction.
    TurnInPlace,
    /// A short burst of full-speed scampering with jerky heading changes. Only entered with
    /// fresh, clear ToF data ahead.
    Zoomies,
    /// Crane the head up, lean back, yawn, coo, then a head shake.
    Stretch,
    /// A quick full-body shiver, shaking the feathers back into place.
    Ruffle,
    /// A body-pose groove on the standing gait, the head grooving along.
    Dance,
    /// Inhale, hitch, a violent CHOO with a chirp, ring-down, dazed look around.
    Sneeze,
    /// Run the ground-pick policy and wait for it to finish.
    GroundPick,
    /// Sit, and either doze (head sinking, snapping back awake, dream twitches) or relax seated.
    /// Ends by standing up, usually into a Stretch.
    Nap,
    /// Being petted: melt into the hand, an excited wiggle, or a sleepy melt that can end in a
    /// nap. Exits a moment after the hand leaves.
    Petted,
}

pub struct Brain {
    rng: Rng,
    state: State,
    /// Time in the current state (s).
    t_state: f64,
    /// The current state ends when `t_state` reaches this.
    state_dur: f64,
    /// Countdown to the next intra-state retarget (a head glance, a heading change).
    retarget_in: f64,
    /// Persistent targets, re-rolled on retargets so motion is piecewise smooth.
    cmd: [f64; 3],
    head: [f64; 4],
    /// TurnInPlace direction and rate (rad/s, signed).
    turn_rate: f64,
    /// Countdown to the next spontaneous quack (s).
    next_quack_in: f64,
    /// Time since the current quack started; past `QUACK_ANIM_S` means idle.
    quack_t: f64,
    /// Quacks left in the current burst, and the countdown to the next one.
    quacks_left: u32,
    burst_gap_in: f64,
    /// GroundPick: the request went out; and whether the robot has been seen busy with it since.
    pick_pending: bool,
    pick_seen_busy: bool,
    /// Walking speed cap (m/s) and turning cap (rad/s): the pad's.
    max_speed: f64,
    max_turn: f64,
    /// Wander: base forward speed and current yaw command.
    wander_vx: f64,
    /// How long the current stretch of walking forward has lasted, for the start kick.
    walking_s: f64,
    wander_wz: f64,
    /// Where this bout is going, in the odometry frame. Odometry is believed perfect: there is no
    /// SLAM yet, and a goal a few metres off is reached before drift matters.
    goal: Option<[f64; 2]>,
    /// Where obstacles stopped the duck, `(point, clock)`, so goals avoid them.
    blocked: Vec<([f64; 2], f64)>,
    /// What this bout did, for the line logged when it ends.
    bout: Bout,
    /// Wander glances: true while a brief side glance is held, and the countdown to toggling.
    glancing: bool,
    glance_in: f64,
    /// Nap: doze vs seated-relax flavour, the sit/stand requests sent, the doze droop phase
    /// (below zero is an awake pause between sinks).
    nap_doze: bool,
    nap_sit_sent: bool,
    nap_stand_sent: bool,
    droop: f64,
    /// Dream twitches, the sneeze's CHOO latch.
    twitch_in: f64,
    twitch_t: f64,
    sneeze_fired: bool,
    /// Energy, 0..1. Zoomies and walking burn it, chilling and naps restore it, and it shapes the
    /// transition weights, the walking speed and how chatty the duck is — so behaviours arrive in
    /// believable rhythms (a playful burst, a tired lull, a nap, refreshed) rather than uniform
    /// dice rolls.
    energy: f64,
    /// Frustration: each avoidance hard stop adds one, decaying over ~12 s. Three while hot means
    /// boxed in, and the duck breaks out.
    stop_heat: f64,
    /// Answering a voice: the pending reply countdown (≤ 0 is none), its cooldown, and the
    /// listening pose's deadline on the behaviour clock.
    reply_in: f64,
    reply_cool: f64,
    perk_until: f64,
    /// Petted: flavour (0 melt, 1 wiggle, 2 sleepy), when the hand was last felt, next coo.
    pet_flavor: u8,
    pet_last_felt: f64,
    pet_coo_in: f64,
    /// Monotonic behaviour clock: the sum of non-busy dt.
    clock: f64,
    /// Novelty grid: dwell seconds per `CELL_M` cell (odometry frame), decayed over ~15 min.
    visits: std::collections::HashMap<(i32, i32), f32>,
    last_pose: [f64; 3],
    decay_tick: u64,
}

impl Brain {
    pub fn new(max_speed: f64, max_turn: f64, seed: u64) -> Self {
        let mut rng = Rng::new(seed);
        let next_quack_in = rng.range(4.0, 12.0);
        Self {
            rng,
            state: State::Chill,
            t_state: 0.0,
            state_dur: 3.0,
            retarget_in: 0.0,
            cmd: [0.0; 3],
            head: [0.0; 4],
            turn_rate: 0.0,
            next_quack_in,
            quack_t: QUACK_ANIM_S + 1.0,
            quacks_left: 0,
            burst_gap_in: 0.0,
            pick_pending: false,
            pick_seen_busy: false,
            max_speed: max_speed.max(0.0),
            max_turn: max_turn.max(0.0),
            wander_vx: 0.0,
            walking_s: 0.0,
            wander_wz: 0.0,
            goal: None,
            blocked: Vec::new(),
            bout: Bout::default(),
            glancing: false,
            glance_in: 0.0,
            nap_doze: false,
            nap_sit_sent: false,
            nap_stand_sent: false,
            droop: 0.0,
            twitch_in: 8.0,
            twitch_t: 1.0,
            sneeze_fired: false,
            energy: 0.8,
            stop_heat: 0.0,
            reply_in: -1.0,
            reply_cool: 0.0,
            perk_until: -1.0,
            pet_flavor: 0,
            pet_last_felt: -100.0,
            pet_coo_in: 0.0,
            clock: 0.0,
            visits: std::collections::HashMap::new(),
            last_pose: [0.0; 3],
            decay_tick: 0,
        }
    }

    #[cfg(test)]
    pub fn state(&self) -> State {
        self.state
    }

    /// Back to a fresh Chill with every latch cleared, after the pad has been driving. The
    /// novelty grid and the energy survive: those are memories, not session state.
    pub fn reset(&mut self) {
        self.enter(State::Chill);
        self.pick_pending = false;
        self.pick_seen_busy = false;
        self.nap_sit_sent = false;
        self.nap_stand_sent = false;
        self.droop = 0.0;
        self.reply_in = -1.0;
        self.perk_until = -1.0;
    }

    /// Advance by `dt` seconds.
    pub fn step(&mut self, dt: f64, world: &World) -> Step {
        let mut sound: Option<Sound> = None;
        let mut ground_pick = false;
        let mut sit_toggle = false;
        let mut head_alpha = CALM_HEAD_ALPHA;
        // Extra mouth opening beyond the quack envelope (a yawn, the sneeze's inhale),
        // merged with the quack in the output.
        let mut mouth_extra = 0.0f64;
        let mut body = [0.0f64; 3];
        let mut body_active = false;
        let two_pi = std::f64::consts::TAU;

        // The quack's animation clock always runs: a quack may straddle a state change.
        self.quack_t += dt;

        if world.busy {
            // Somebody else is driving — our own ground pick included. Freeze, neutral.
            if self.pick_pending {
                self.pick_seen_busy = true;
            }
            return Step {
                cmd: [0.0; 3],
                head: [0.0; 4],
                head_alpha: CALM_HEAD_ALPHA,
                body: [0.0; 3],
                body_active: false,
                mouth: self.mouth_now(),
                sound: None,
                ground_pick: false,
                sit_toggle: false,
                wheee: false,
            };
        }

        // Seated without having asked to be: someone sat the robot down from the pad and walked
        // away. Adopt it as a nap, so the brain stands it back up eventually instead of animating
        // a seated duck forever as if it were standing.
        if world.sitting && self.state != State::Nap {
            self.enter(State::Nap);
            self.nap_sit_sent = true;
        }

        self.t_state += dt;
        self.retarget_in -= dt;
        self.clock += dt;
        self.stop_heat = (self.stop_heat - dt / 12.0).max(0.0);

        // Energy: motion burns, rest restores. Continuous wandering drains a fresh duck in about
        // a minute and a half; a nap refills most of it.
        let de = match self.state {
            State::Zoomies => -dt / 25.0,
            State::Wander => -dt / 110.0,
            State::TurnInPlace => -dt / 70.0,
            State::Nap => dt / 40.0,
            State::Petted => dt / 60.0,
            State::Dance => -dt / 45.0,
            State::Chill => dt / 120.0,
            _ => dt / 240.0,
        };
        self.energy = (self.energy + de).clamp(0.05, 1.0);

        // Novelty grid: dwell time in the current cell, capped so old haunts are not infinitely
        // repulsive, and decayed slowly so places become fresh again.
        if self.state == State::Wander {
            self.bout.path_m +=
                (world.pose[0] - self.last_pose[0]).hypot(world.pose[1] - self.last_pose[1]);
            self.bout.asked_m += self.cmd[0] * dt;
        }
        self.last_pose = world.pose;
        let cell = Self::cell_of(world.pose[0], world.pose[1]);
        let w = self.visits.entry(cell).or_insert(0.0);
        *w = (*w + dt as f32).min(60.0);
        self.decay_tick += 1;
        if self.decay_tick.is_multiple_of(1500) {
            self.visits.retain(|_, w| {
                *w *= 0.97;
                *w > 0.05
            });
        }

        // Spontaneous quacks, whatever it is doing — ducks do not wait politely. Each is a burst:
        // usually one, sometimes two to four in quick succession.
        self.next_quack_in -= dt;
        if self.next_quack_in <= 0.0 {
            // Energetic ducks are chattier; tired ones pipe down.
            self.next_quack_in = self.rng.range(6.0, 20.0) * (1.8 - self.energy);
            self.start_quack_burst();
            sound = Some(Sound::Chirp);
        } else if self.quacks_left > 0 {
            self.burst_gap_in -= dt;
            if self.burst_gap_in <= 0.0 {
                self.quacks_left -= 1;
                self.quack_t = 0.0;
                self.burst_gap_in = self.rng.range(0.30, 0.45);
                sound = Some(Sound::Chirp);
            }
        }

        // Sounds. A voice gets an attentive perk and a quacked answer after a beat. A noise only
        // stirs a dozing duck — the runtime's noise startle went with the rest of Startle.
        self.reply_cool -= dt;
        let dozing = self.state == State::Nap && self.nap_doze;
        match world.heard {
            Some(Heard::Noise) if dozing => {
                // Stirred, not woken: the head pops up for a beat, then the doze resumes.
                self.droop = -0.6;
                self.twitch_t = 0.0;
            }
            Some(Heard::Voice)
                if self.reply_cool <= 0.0 && self.state != State::Petted && !dozing =>
            {
                // Listen first, answer after a polite beat.
                self.reply_in = self.rng.range(0.35, 0.8);
                self.reply_cool = 4.0;
                self.perk_until = self.clock + 1.4;
            }
            _ => {}
        }
        if self.reply_in > 0.0 {
            self.reply_in -= dt;
            if self.reply_in <= 0.0 {
                self.reply_in = -1.0;
                tracing::info!("answering");
                self.start_quack_burst();
                sound = Some(Sound::Chirp);
            }
        }

        // Petting, the best interrupt there is. A nap handles its own (a hand must not end a
        // seated nap).
        if world.petting && !matches!(self.state, State::Petted | State::Nap) {
            // Flavour by mood: tired → sleepy melt; fresh → often an excited wiggle.
            self.pet_flavor = if self.energy < 0.4 {
                2
            } else if self.energy > 0.65 && self.rng.chance(0.5) {
                1
            } else {
                0
            };
            tracing::info!(flavor = self.pet_flavor, "being petted");
            // `robotd` coos by itself when petting starts, so the first coo here waits a while
            // rather than doubling it.
            self.pet_coo_in = 3.0;
            self.enter(State::Petted);
        }

        match self.state {
            State::Chill => {
                self.cmd = [0.0; 3];
                // A barely-open breathing mouth reads as alive at rest.
                mouth_extra = (0.03 + 0.03 * (two_pi * 0.45 * self.t_state).sin()).max(0.0);
                if self.retarget_in <= 0.0 {
                    self.retarget_in = self.rng.range(1.2, 3.0);
                    // Even at rest the head keeps living: lazy but visible drifts, with the
                    // occasional head cock.
                    self.head = [
                        self.rng.sym(0.10),
                        self.rng.sym(0.20),
                        self.rng.sym(0.40),
                        if self.rng.chance(0.30) {
                            self.rng.sym(0.25)
                        } else {
                            0.0
                        },
                    ];
                }
            }
            State::LookAround => {
                self.cmd = [0.0; 3];
                if self.retarget_in <= 0.0 {
                    self.retarget_in = self.rng.range(0.7, 1.8);
                    // Big cartoonish glances: whip the head around, dip or crane it, and cock it
                    // sideways more often than not.
                    self.head = [
                        self.rng.sym(0.15),
                        self.rng.range(-0.55, 0.30),
                        self.rng.sym(1.1),
                        if self.rng.chance(0.60) {
                            self.rng.sym(0.50)
                        } else {
                            0.0
                        },
                    ];
                }
            }
            State::Wander => {
                if self.retarget_in <= 0.0 {
                    self.retarget_in = self.rng.range(1.2, 3.0);
                    // Tired ducks amble; fresh ones trot.
                    let pep = 0.65 + 0.35 * self.energy;
                    self.wander_vx = self.rng.range(0.6 * self.max_speed, self.max_speed) * pep;
                }
                let goal = match self.goal {
                    Some(goal) => goal,
                    None => {
                        let goal = self.pick_goal();
                        self.goal = Some(goal);
                        goal
                    }
                };
                let (dx, dy) = (goal[0] - self.last_pose[0], goal[1] - self.last_pose[1]);
                let distance = dx.hypot(dy);
                if distance < self.bout.best_left_m - GOAL_PROGRESS_M {
                    self.bout.best_left_m = distance;
                    self.bout.progress_at = self.t_state;
                }
                if distance < GOAL_REACHED_M {
                    self.bout.reached = true;
                    // The bout is over; the transition below picks what comes next.
                    self.t_state = self.state_dur;
                } else if self.t_state - self.bout.progress_at > GOAL_PATIENCE_S {
                    tracing::info!(left_m = distance, "no closer to the goal — giving up on it");
                    self.t_state = self.state_dur;
                }
                let heading_delta = wrap_angle(dy.atan2(dx) - self.last_pose[2]);
                let pivoting = heading_delta.abs() > 0.7;
                self.wander_wz = if pivoting {
                    // Far off the goal's bearing: pivot first, walk after. Crawling the turn at
                    // the walking yaw clamp wasted seconds at the start of every bout.
                    0.9 * self.max_turn * heading_delta.signum()
                } else {
                    (1.2 * heading_delta).clamp(-0.6, 0.6)
                };
                // Rubbernecking, but brief: short side glances that snap back to looking into
                // the turn, so the head-mounted ToF keeps re-observing the walking corridor.
                self.glance_in -= dt;
                if self.glance_in <= 0.0 {
                    self.glancing = !self.glancing;
                    if self.glancing {
                        self.glance_in = self.rng.range(0.4, 0.8);
                        self.head = [
                            0.0,
                            self.rng.range(-0.25, 0.10),
                            self.rng.sym(0.9),
                            if self.rng.chance(0.3) {
                                self.rng.sym(0.30)
                            } else {
                                0.0
                            },
                        ];
                    } else {
                        self.glance_in = self.rng.range(1.5, 3.5);
                        self.head = [
                            0.0,
                            self.rng.range(-0.15, 0.05),
                            (0.9 * self.wander_wz).clamp(-0.5, 0.5),
                            0.0,
                        ];
                    }
                }
                // Slow into the goal rather than overshoot it and circle back.
                let approach = (distance / GOAL_SLOW_M).clamp(0.3, 1.0);
                let base_vx = if pivoting {
                    0.0
                } else {
                    self.wander_vx * approach
                };
                let (vx, wz, gate) =
                    self.apply_obstacle_gate(base_vx, self.wander_wz, &world.obstacle);
                if base_vx > 0.0 {
                    self.bout.count(gate, dt);
                }
                let vx = self.walkable(vx, dt);
                self.cmd = [vx, 0.0, wz];
            }
            State::TurnInPlace => {
                self.cmd = [0.0, 0.0, self.turn_rate];
                // Lead the turn with the whole head, as if it cannot wait to see what is there.
                let side = self.turn_rate.signum();
                self.head = [0.0, 0.10, 0.7 * side, 0.15 * side];
            }
            State::Zoomies => {
                // Excited scamper: full speed, quick jerky heading changes, head low and locked
                // forward as if chasing something.
                if self.retarget_in <= 0.0 {
                    self.retarget_in = self.rng.range(0.5, 1.1);
                    let target = self.pick_heading(1.0);
                    let delta = wrap_angle(target - self.last_pose[2]);
                    self.wander_wz = (delta + self.rng.sym(0.3)).clamp(-0.7, 0.7);
                    self.head = [
                        0.0,
                        self.rng.range(-0.10, 0.10),
                        (0.5 * self.wander_wz).clamp(-0.35, 0.35),
                        self.rng.sym(0.15),
                    ];
                }
                let (vx, wz, _) =
                    self.apply_obstacle_gate(self.max_speed, self.wander_wz, &world.obstacle);
                let vx = self.walkable(vx, dt);
                self.cmd = [vx, 0.0, wz];
            }
            State::Stretch => {
                // Rise → climax (full extension, yawning, one contented coo) → release → a quick
                // head shake to gather itself before moving on.
                self.cmd = [0.0; 3];
                let t = self.t_state;
                let rise_end = 1.4;
                let release_start = self.state_dur - 2.2;
                let shake_start = self.state_dur - 1.2;
                let env = if t < release_start {
                    (t / rise_end).min(1.0)
                } else {
                    (1.0 - (t - release_start)).clamp(0.0, 1.0)
                };
                let climax = rise_end + 0.2;
                if t - dt < climax && t >= climax {
                    sound = Some(Sound::Coo);
                }
                // z barely registers physically, so the stretch is sold with the angles: a hard
                // lean back plus a slow roll sway.
                body = [
                    env * 0.010,
                    env * -0.20,
                    env * 0.10 * (two_pi * 0.35 * t).sin(),
                ];
                body_active = t < self.state_dur;
                if t < shake_start {
                    self.head = [
                        env * 0.30,
                        env * 0.70,
                        env * 0.12 * (two_pi * 0.3 * t).sin(),
                        env * 0.15 * (two_pi * 0.25 * t + 1.0).sin(),
                    ];
                } else {
                    // Gather-up shake, cat-after-a-yawn style: fast decaying whips with the head
                    // dipped. The rate is cranked so the 3.5 Hz whip arrives at full amplitude.
                    head_alpha = 0.8;
                    let ts = t - shake_start;
                    let decay = (1.0 - ts / 1.2).clamp(0.0, 1.0);
                    self.head = [
                        -0.10 * decay,
                        -0.30 * decay,
                        0.55 * decay * (two_pi * 3.5 * ts).sin(),
                        0.20 * decay * (two_pi * 3.5 * ts + 1.2).sin(),
                    ];
                }
                mouth_extra = ((t - 0.8) / 0.6).clamp(0.0, 1.0) * env * 0.5;
            }
            State::Ruffle => {
                // A ~4.5 Hz whole-body shiver: body roll plus a counter-phased head wag, head
                // dipped. Silent — it is grooming, not talking. Near-direct rate: the point is a
                // fast blur of a shake.
                self.cmd = [0.0; 3];
                head_alpha = 0.85;
                let t = self.t_state;
                let env = (t / 0.15).min(1.0) * ((self.state_dur - t) / 0.4).clamp(0.0, 1.0);
                let w = two_pi * 4.5 * t;
                body = [-0.005 * env, 0.0, 0.13 * env * w.sin()];
                body_active = t < self.state_dur;
                self.head = [
                    -0.08 * env,
                    -0.15 * env,
                    0.45 * env * (w + 0.9).sin(),
                    0.25 * env * (w + 2.2).sin(),
                ];
            }
            State::Dance => {
                self.cmd = [0.0; 3];
                let t = self.t_state;
                // Fade in over 0.5 s and out over the last second, so the body pose is back at
                // neutral before the mode ends.
                let env = (t / 0.5).min(1.0) * ((self.state_dur - t) / 1.0).clamp(0.0, 1.0);
                body = [
                    env * (-0.006 + 0.010 * (two_pi * 1.1 * t).sin()),
                    env * 0.07 * (two_pi * 0.55 * t).sin(),
                    env * 0.12 * (two_pi * 1.1 * t + 1.0).sin(),
                ];
                body_active = t < self.state_dur;
                // The head grooves along, hard — half the dance is the head.
                head_alpha = 0.30;
                self.head = [
                    0.0,
                    env * 0.18 * (two_pi * 0.55 * t).sin(),
                    env * 0.35 * (two_pi * 0.55 * t + 0.7).sin(),
                    env * 0.45 * (two_pi * 1.1 * t + 1.0).sin(),
                ];
            }
            State::Sneeze => {
                self.cmd = [0.0; 3];
                let t = self.t_state;
                if t < 0.75 {
                    // The inhale: head cranes up and back, mouth opening.
                    let r = (t / 0.75).powf(1.4);
                    self.head = [0.15 * r, 0.55 * r, 0.0, 0.0];
                    mouth_extra = 0.4 * r;
                } else if t < 0.85 {
                    // The hitch, frozen at the top.
                    head_alpha = 0.1;
                    mouth_extra = 0.45;
                } else {
                    if !self.sneeze_fired {
                        self.sneeze_fired = true;
                        self.quack_t = 0.0;
                        sound = Some(Sound::Chirp);
                    }
                    let ts = t - 0.85;
                    if ts < 0.5 {
                        // CHOO: a violent head-down whip with a ring-down wobble.
                        head_alpha = 0.9;
                        let ring = (two_pi * 5.0 * ts).sin() * (1.0 - ts / 0.5);
                        self.head = [-0.10, -0.65 * (1.0 - ts / 0.5), 0.25 * ring, 0.20 * ring];
                        mouth_extra = (0.5 - ts).max(0.0);
                    } else {
                        // Dazed: a slow "...what was that" look around.
                        head_alpha = 0.12;
                        let td = ts - 0.5;
                        self.head = [0.0, 0.05, 0.5 * (two_pi * 0.35 * td).sin(), 0.0];
                    }
                }
            }
            State::Nap => {
                self.cmd = [0.0; 3];
                let t = self.t_state;
                if !self.nap_sit_sent {
                    self.nap_sit_sent = true;
                    sit_toggle = true;
                } else if t > 1.5 && !world.sitting && !self.nap_stand_sent {
                    // The sit never engaged (refused robot-side, or the stand-up already
                    // happened under us): give up on the nap.
                    self.enter(State::Chill);
                } else if t < self.state_dur {
                    if t < 2.0 {
                        // Settling into the sit.
                        self.head = [0.0; 4];
                    } else if world.petting {
                        // A hand! Rise into it and coo softly, without ending the nap.
                        head_alpha = 0.2;
                        self.head = [
                            0.05,
                            0.35 + 0.06 * (two_pi * 0.5 * t).sin(),
                            0.0,
                            0.15 * (two_pi * 0.3 * t).sin(),
                        ];
                        mouth_extra = (0.08 + 0.05 * (two_pi * 0.8 * t).sin()).max(0.0);
                        self.pet_coo_in -= dt;
                        if self.pet_coo_in <= 0.0 {
                            self.pet_coo_in = self.rng.range(4.0, 7.0);
                            sound = Some(Sound::Coo);
                        }
                    } else if self.nap_doze {
                        // The head sinks with accelerating droop, snaps back up ("I'm awake!"),
                        // holds a beat, sinks again.
                        self.droop += dt / 5.0;
                        if self.droop >= 1.0 {
                            self.droop = self.rng.range(-0.5, -0.1);
                        }
                        let sink = self.droop.max(0.0).powf(1.6);
                        self.head = [
                            -0.10 * sink,
                            -0.55 * sink,
                            0.06 * (two_pi * 0.11 * t).sin(),
                            0.20 * sink,
                        ];
                        // Dream twitches, occasionally with a muffled half-quack. People watching
                        // a sleeping duck wait for exactly this.
                        self.twitch_in -= dt;
                        if self.twitch_in <= 0.0 {
                            self.twitch_in = self.rng.range(6.0, 14.0);
                            self.twitch_t = 0.0;
                            if self.rng.chance(0.3) {
                                self.quack_t = 0.0;
                                sound = Some(Sound::Chirp);
                            }
                        }
                        self.twitch_t += dt;
                        if self.twitch_t < 0.35 {
                            let tw =
                                (two_pi * 9.0 * self.twitch_t).sin() * (1.0 - self.twitch_t / 0.35);
                            self.head[2] += 0.12 * tw;
                            self.head[3] += 0.10 * tw;
                            head_alpha = 0.5;
                        }
                    } else {
                        // Seated relax: a soft slow look-around, the spontaneous quacks still
                        // running.
                        if self.retarget_in <= 0.0 {
                            self.retarget_in = self.rng.range(1.6, 3.2);
                            self.head = [
                                self.rng.sym(0.10),
                                self.rng.range(-0.15, 0.35),
                                self.rng.sym(0.8),
                                if self.rng.chance(0.4) {
                                    self.rng.sym(0.35)
                                } else {
                                    0.0
                                },
                            ];
                        }
                    }
                } else if !self.nap_stand_sent {
                    self.nap_stand_sent = true;
                    self.head = [0.0; 4];
                    sit_toggle = true;
                } else if !world.sitting {
                    // Back on two feet: usually a big wake-up stretch.
                    let next = if self.rng.chance(0.65) {
                        State::Stretch
                    } else {
                        State::LookAround
                    };
                    self.enter(next);
                } else if t > self.state_dur + 5.0 {
                    // Still seated long after asking to stand: the request was refused. Ask
                    // again rather than sit here forever.
                    self.nap_stand_sent = false;
                    self.t_state = self.state_dur;
                }
            }
            State::Petted => {
                // Body pose carries the sway, the head does the nuzzling, coos punctuate.
                self.cmd = [0.0; 3];
                let t = self.t_state;
                if world.petting {
                    self.pet_last_felt = self.clock;
                }
                let linger = self.clock - self.pet_last_felt;
                if linger > 1.2 {
                    // The hand is gone for good: exit befitting the mood.
                    let r = self.rng.unit();
                    if self.pet_flavor == 2 && world.can_sit_stand && !world.sitting && r < 0.6 {
                        tracing::info!("petted to sleep");
                        self.enter(State::Nap);
                        self.nap_doze = true;
                    } else if r < 0.40 {
                        self.enter(State::Ruffle)
                    } else if r < 0.55 {
                        self.enter(State::Stretch)
                    } else if r < 0.85 {
                        self.enter(State::LookAround)
                    } else {
                        self.enter(State::Chill)
                    }
                } else if !world.petting {
                    // Linger: head up, searching. "Where did the hand go?"
                    head_alpha = 0.3;
                    self.head = [0.10, 0.55, 0.25 * (two_pi * 0.8 * t).sin(), 0.0];
                    body_active = true;
                } else {
                    body_active = true;
                    let env = (t / 0.6).min(1.0);
                    self.pet_coo_in -= dt;
                    match self.pet_flavor {
                        1 => {
                            // Excited wiggle, the tail-wag equivalent: fast body-roll wag, bouncy
                            // head, happy chirps.
                            head_alpha = 0.35;
                            let w = (two_pi * 1.6 * t).sin();
                            self.head = [
                                0.05,
                                env * (0.35 + 0.06 * w),
                                env * 0.20 * (two_pi * 0.8 * t).sin(),
                                env * 0.35 * w,
                            ];
                            body = [-0.004 * env, 0.0, 0.12 * env * w];
                            mouth_extra = (0.18 + 0.08 * (two_pi * 1.2 * t).sin()).max(0.0);
                            if self.pet_coo_in <= 0.0 {
                                self.pet_coo_in = self.rng.range(3.0, 5.0);
                                self.start_quack_burst();
                                sound = Some(Sound::Chirp);
                            }
                        }
                        2 => {
                            // Sleepy melt: leans into the hand, then comfort slowly wins and the
                            // head drifts down over ~25 s.
                            head_alpha = 0.12;
                            let sink = (t / 25.0).min(1.0);
                            self.head = [
                                0.08 * env,
                                env * (0.45 * (1.0 - sink) - 0.35 * sink)
                                    + 0.05 * (two_pi * 0.35 * t).sin(),
                                0.10 * (two_pi * 0.20 * t).sin(),
                                env * (0.15 + 0.20 * sink),
                            ];
                            body = [-0.010 * env, 0.0, 0.06 * env * (two_pi * 0.30 * t).sin()];
                            mouth_extra = (0.08 + 0.05 * (two_pi * 0.8 * t).sin()).max(0.0);
                            if self.pet_coo_in <= 0.0 {
                                self.pet_coo_in = self.rng.range(6.0, 9.0);
                                sound = Some(Sound::Coo);
                            }
                        }
                        _ => {
                            // Melt-in: a slow lean into the hand, nuzzling, contented breathing.
                            head_alpha = 0.2;
                            let lean = 0.45 + 0.08 * (two_pi * 0.5 * t).sin();
                            self.head = [
                                env * 0.12 * (two_pi * 0.4 * t).sin(),
                                env * lean,
                                env * 0.15 * (two_pi * 0.22 * t).sin(),
                                env * 0.30 * (two_pi * 0.30 * t).sin(),
                            ];
                            body = [
                                env * (-0.006 + 0.005 * (two_pi * 0.35 * t).sin()),
                                0.0,
                                env * 0.10 * (two_pi * 0.40 * t).sin(),
                            ];
                            mouth_extra = (0.12 + 0.07 * (two_pi * 1.0 * t).sin()).max(0.0);
                            if self.pet_coo_in <= 0.0 {
                                self.pet_coo_in = self.rng.range(3.5, 6.0);
                                sound = Some(Sound::Coo);
                            }
                        }
                    }
                }
            }
            State::GroundPick => {
                self.cmd = [0.0; 3];
                self.head = [0.0; 4];
                if !self.pick_pending {
                    if world.can_ground_pick {
                        self.pick_pending = true;
                        self.pick_seen_busy = false;
                        ground_pick = true;
                    } else {
                        self.enter(State::Chill);
                    }
                } else if self.pick_seen_busy {
                    // The pick ran and the robot is ours again: a happy quack and move on.
                    self.pick_pending = false;
                    self.start_quack_burst();
                    sound = Some(Sound::Chirp);
                    self.enter(State::Chill);
                } else if self.t_state > PICK_ACK_S {
                    // Never went busy: the robot refused it. Move on quietly.
                    self.pick_pending = false;
                    self.enter(State::Chill);
                }
            }
        }

        // Listening: while a reply is brewing (or just delivered), hold an attentive head-up pose.
        if matches!(self.state, State::Chill | State::LookAround) && self.clock < self.perk_until {
            self.head = [0.05, 0.35, 0.0, 0.0];
            head_alpha = 0.3;
        }

        // The current state's time is up (GroundPick and Nap manage their own exits).
        if self.t_state >= self.state_dur && !matches!(self.state, State::GroundPick | State::Nap) {
            let can_zoom = world
                .obstacle
                .is_some_and(|o| o.age_s <= FRESH_S && o.ahead_m > 1.2);
            let can_nap = world.can_sit_stand && !world.sitting;
            let next = self.pick_next_state(world.can_ground_pick, can_zoom, can_nap);
            self.enter(next);
        }

        Step {
            cmd: self.cmd,
            head: self.head_out(),
            head_alpha,
            body,
            body_active,
            mouth: mouth_extra.max(self.mouth_now()).clamp(0.0, 1.0),
            sound,
            ground_pick,
            sit_toggle,
            wheee: self.state == State::Zoomies,
        }
    }

    /// The head target with its animation overlays: each quack throws the head up with the open
    /// mouth, and walking gets a duck-like rhythmic bob.
    fn head_out(&self) -> [f64; 4] {
        let mut h = self.head;
        h[1] += 0.35 * self.mouth_now();
        if matches!(self.state, State::Wander | State::Zoomies) && self.cmd[0] > 0.02 {
            h[1] += 0.12 * (std::f64::consts::TAU * 2.2 * self.t_state).sin();
        }
        h
    }

    fn cell_of(x: f64, y: f64) -> (i32, i32) {
        ((x / CELL_M).floor() as i32, (y / CELL_M).floor() as i32)
    }

    fn visit_at(&self, x: f64, y: f64) -> f32 {
        self.visits
            .get(&Self::cell_of(x, y))
            .copied()
            .unwrap_or(0.0)
    }

    /// A world heading biased toward the least-visited space: candidate headings within
    /// ±`spread` of the current yaw, scored by dwell time along a short ray (near cells weigh
    /// more), with jitter so ties break randomly. Fresh space scores zero and wins.
    fn pick_heading(&mut self, spread: f64) -> f64 {
        let [x, y, yaw] = self.last_pose;
        let mut best = (f64::MAX, yaw);
        for k in 0..9 {
            let h = yaw + spread * (k as f64 - 4.0) / 4.0;
            let mut score = 0.0f64;
            for d in [0.7, 1.5, 2.5] {
                let w = self.visit_at(x + d * h.cos(), y + d * h.sin());
                score += w as f64 * (1.7 - 0.4 * d);
            }
            score += self.rng.range(0.0, 0.5);
            if score < best.0 {
                best = (score, h);
            }
        }
        best.1
    }

    /// A goal for the next wander bout, in the odometry frame: the candidate point (in
    /// `GOAL_DIRECTIONS` directions at `GOAL_DISTANCES`) whose straight path crosses the least
    /// visited ground, keeps clear of where obstacles stopped the duck, and is far rather than
    /// near — with a small cost for turning, and jitter so ties break randomly.
    fn pick_goal(&mut self) -> [f64; 2] {
        let [x, y, yaw] = self.last_pose;
        let now = self.clock;
        self.blocked.retain(|(_, at)| now - at <= BLOCKED_S);
        let offset = self.rng.range(0.0, std::f64::consts::TAU);
        let mut best = (f64::MAX, [x + yaw.cos(), y + yaw.sin()]);
        for k in 0..GOAL_DIRECTIONS {
            let heading = offset + std::f64::consts::TAU * k as f64 / GOAL_DIRECTIONS as f64;
            let (s, c) = heading.sin_cos();
            for d in GOAL_DISTANCES {
                // Dwell along the path, sampled every quarter metre: the duck walks the whole of
                // it, so the whole of it is what is new or not.
                let steps = (d / 0.25) as usize;
                let mut dwell = 0.0f64;
                let mut shut = false;
                for i in 1..=steps {
                    let r = d * i as f64 / steps as f64;
                    let (px, py) = (x + r * c, y + r * s);
                    dwell += f64::from(self.visit_at(px, py));
                    shut |= self
                        .blocked
                        .iter()
                        .any(|(b, _)| (b[0] - px).hypot(b[1] - py) < BLOCKED_M);
                }
                let turn = wrap_angle(heading - yaw).abs() / std::f64::consts::PI;
                let score = dwell / steps as f64 + if shut { 100.0 } else { 0.0 } - 0.5 * d
                    + 0.6 * turn
                    + self.rng.range(0.0, 0.5);
                if score < best.0 {
                    best = (score, [x + d * c, y + d * s]);
                }
            }
        }
        best.1
    }

    /// A forward command the gait will act on: zero, or at least `MIN_WALK_VX`, and full speed for
    /// the first `START_KICK_S` of a walk.
    fn walkable(&mut self, vx: f64, dt: f64) -> f64 {
        // `self.cmd` is still last tick's here. Whatever stopped the walk — a pivot, a gate, or a
        // whole other state — the next forward command is a start, and gets the kick.
        if vx <= 0.01 || self.cmd[0] <= 0.01 {
            self.walking_s = 0.0;
        }
        if vx <= 0.01 {
            return 0.0;
        }
        self.walking_s += dt;
        if self.walking_s < START_KICK_S {
            self.max_speed
        } else {
            vx.max(MIN_WALK_VX.min(self.max_speed))
        }
    }

    /// The freshness/steer/stop gate on a walking command, and what it did. On a hard stop it
    /// also switches to TurnInPlace, away from the obstacle, so callers use the returned command
    /// as is.
    fn apply_obstacle_gate(
        &mut self,
        mut vx: f64,
        wz: f64,
        obstacle: &Option<Obstacle>,
    ) -> (f64, f64, Gate) {
        match obstacle.filter(|o| o.age_s <= STALE_S) {
            None => {
                // The walking direction is unobserved (never seen, too old, or we turned since):
                // do not advance, but KEEP TURNING. Turning in place is collision-safe, and
                // zeroing the turn too froze the duck mid-pivot until the next forward frame.
                (0.0, wz, Gate::Unseen)
            }
            Some(o) => {
                let aging = o.age_s > FRESH_S;
                if aging {
                    vx *= 0.5;
                }
                let away = o.away;
                if o.ahead_m < STOP_M {
                    tracing::info!(ahead_m = o.ahead_m, "obstacle ahead — turning away");
                    if self.state == State::Wander {
                        // Remember where the way was shut, so the next goal is not behind the same
                        // wall. Just past the beak, along the heading the duck was walking.
                        let [x, y, yaw] = self.last_pose;
                        let reach = crate::obstacle::BEAK_M + o.ahead_m + 0.1;
                        self.blocked
                            .push(([x + reach * yaw.cos(), y + reach * yaw.sin()], self.clock));
                    }
                    self.enter(State::TurnInPlace);
                    self.turn_rate = self.turn_rate.abs() * away;
                    // Three hard stops while the heat is up means boxed in: an annoyed quack
                    // burst and a decisive spin-out instead of more polite nudging.
                    self.stop_heat += 1.0;
                    if self.stop_heat >= 3.0 {
                        self.stop_heat = 0.0;
                        tracing::info!("boxed in — breaking out");
                        self.start_quack_burst();
                        self.burst_gap_in = 0.0;
                        self.quacks_left = self.quacks_left.max(2);
                        self.turn_rate = self.max_turn * away;
                        self.state_dur = self.state_dur.max(1.8);
                    }
                    (0.0, self.turn_rate, Gate::Stopped)
                } else if o.ahead_m < REACT_M {
                    let closeness = ((REACT_M - o.ahead_m) / (REACT_M - STOP_M)).clamp(0.0, 1.0);
                    (
                        vx * (1.0 - 0.8 * closeness),
                        wz + away * 0.6 * closeness,
                        Gate::Reacting,
                    )
                } else if aging {
                    (vx, wz, Gate::Aging)
                } else {
                    (vx, wz, Gate::Clear)
                }
            }
        }
    }

    /// Start a quack burst; the first quack is the caller's to emit. One quack 45% of the time,
    /// two 30%, three 18%, four 7%.
    fn start_quack_burst(&mut self) {
        let r = self.rng.unit();
        let count: u32 = if r < 0.45 {
            1
        } else if r < 0.75 {
            2
        } else if r < 0.93 {
            3
        } else {
            4
        };
        self.quack_t = 0.0;
        self.quacks_left = count - 1;
        self.burst_gap_in = self.rng.range(0.30, 0.45);
    }

    /// The quack's mouth: a half-sine over `QUACK_ANIM_S`.
    fn mouth_now(&self) -> f64 {
        if self.quack_t < QUACK_ANIM_S {
            (std::f64::consts::PI * self.quack_t / QUACK_ANIM_S).sin()
        } else {
            0.0
        }
    }

    fn enter(&mut self, next: State) {
        if self.state == State::Wander {
            self.bout.log(self.last_pose);
        }
        self.state = next;
        self.t_state = 0.0;
        self.retarget_in = 0.0;
        if next != State::GroundPick {
            self.pick_pending = false;
        }
        self.state_dur = match next {
            State::Chill => self.rng.range(3.0, 7.0),
            State::LookAround => self.rng.range(3.5, 7.0),
            State::Wander => {
                // Start "glancing" with an expired timer, so the first tick flips to the
                // forward-looking pose at once.
                self.glancing = true;
                self.glance_in = 0.0;
                let goal = self.pick_goal();
                self.goal = Some(goal);
                self.bout = Bout::starting(self.last_pose, goal);
                // Ends on arrival, or when it stops getting closer (see `GOAL_PATIENCE_S`).
                BOUT_MAX_S
            }
            State::TurnInPlace => {
                // Near the policy's top yaw rate: snappy turns read better than slow ones.
                // Usually aimed at the least-visited direction; sometimes a spin on a whim.
                let rate = self.rng.range(0.8, 1.0) * self.max_turn;
                self.turn_rate = rate * if self.rng.chance(0.5) { 1.0 } else { -1.0 };
                let mut dur = self.rng.range(0.8, 2.0);
                if self.rng.chance(0.7) {
                    let target = self.pick_heading(std::f64::consts::PI);
                    let delta = wrap_angle(target - self.last_pose[2]);
                    if delta.abs() > 0.3 {
                        self.turn_rate = rate * delta.signum();
                        dur = (delta.abs() / rate.max(1e-3)).clamp(0.5, 2.2);
                    }
                }
                dur
            }
            State::Zoomies => {
                self.energy = (self.energy - 0.08).max(0.05);
                let dur = self.rng.range(2.5, 5.0);
                // The wheee ride is the zoomies' soundtrack, and a quack would cut it off (one
                // sound at a time), so quacks wait.
                self.next_quack_in = self.next_quack_in.max(dur + 1.0);
                dur
            }
            State::Stretch => self.rng.range(5.5, 6.5),
            State::Ruffle => self.rng.range(1.4, 2.0),
            State::Dance => {
                // It kicks off with an excited chirp.
                self.next_quack_in = self.next_quack_in.min(0.5);
                self.rng.range(5.0, 8.0)
            }
            State::Sneeze => {
                self.sneeze_fired = false;
                self.rng.range(2.6, 3.2)
            }
            // Petted exits when the hand leaves; the cap is a safety net for a stuck flag.
            State::Petted => 90.0,
            State::Nap => {
                self.nap_doze = self.rng.chance(0.55);
                self.nap_sit_sent = false;
                self.nap_stand_sent = false;
                self.droop = -0.3;
                let dur = if self.nap_doze {
                    self.rng.range(20.0, 40.0)
                } else {
                    self.rng.range(12.0, 25.0)
                };
                // Dozing ducks do not quack; the relaxed flavour keeps them.
                if self.nap_doze {
                    self.next_quack_in = self.next_quack_in.max(dur + 2.0);
                }
                dur
            }
            // Effectively "until the pick completes".
            State::GroundPick => 60.0,
        };
        tracing::info!(state = ?next, dur_s = self.state_dur, energy = self.energy, "→");
    }

    /// The next state, conditioned on where we are coming from. Wander chains into turns, more
    /// wandering or zoomies so exploration reads as purposeful; picks and stretches only start
    /// from standing, never mid-stride.
    fn pick_next_state(&mut self, can_ground_pick: bool, can_zoom: bool, can_nap: bool) -> State {
        let r = self.rng.unit();
        match self.state {
            State::Wander => {
                // Keep exploring more often than settling; a walk often ends with a shake.
                if r < 0.20 {
                    State::TurnInPlace
                } else if r < 0.30 && can_zoom {
                    State::Zoomies
                } else if r < 0.50 {
                    State::Wander
                } else if r < 0.58 {
                    State::Ruffle
                } else if r < 0.80 {
                    State::LookAround
                } else {
                    State::Chill
                }
            }
            State::TurnInPlace => {
                // A turn is usually a prelude to going somewhere.
                if r < 0.55 {
                    State::Wander
                } else if r < 0.80 {
                    State::LookAround
                } else {
                    State::Chill
                }
            }
            // Catch its breath after zoomies.
            State::Zoomies => {
                if r < 0.5 {
                    State::Chill
                } else {
                    State::LookAround
                }
            }
            // Feathers back in place: look around, settle, or go somewhere.
            State::Ruffle => {
                if r < 0.40 {
                    State::LookAround
                } else if r < 0.70 {
                    State::Chill
                } else {
                    State::Wander
                }
            }
            // Shake it off after a sneeze, usually.
            State::Sneeze => {
                if r < 0.35 {
                    State::Ruffle
                } else if r < 0.70 {
                    State::LookAround
                } else {
                    State::Chill
                }
            }
            // Standing states: anything goes, but ENERGY shapes the pool. A fresh duck wants to
            // go; a tired one parks, grooms, naps.
            _ => {
                let e = self.energy;
                let weights: [(State, f64); 9] = [
                    (State::Wander, 0.16 + 0.28 * e),
                    (State::TurnInPlace, 0.06),
                    (State::Zoomies, if can_zoom && e > 0.5 { 0.07 } else { 0.0 }),
                    (
                        State::Nap,
                        if can_nap {
                            0.03 + 0.11 * (1.0 - e)
                        } else {
                            0.0
                        },
                    ),
                    (State::Stretch, 0.05),
                    (State::GroundPick, if can_ground_pick { 0.05 } else { 0.0 }),
                    (State::Ruffle, 0.06),
                    (State::Dance, if e > 0.4 { 0.05 } else { 0.0 }),
                    (State::Sneeze, 0.03),
                ];
                let look = (State::LookAround, 0.12 + 0.06 * (1.0 - e));
                let total: f64 = weights.iter().map(|(_, w)| w).sum::<f64>() + look.1 + 0.10;
                let mut x = r * total;
                for (state, w) in weights.into_iter().chain([look]) {
                    if x < w {
                        return state;
                    }
                    x -= w;
                }
                State::Chill
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MAX_SPEED: f64 = 0.3;
    const MAX_TURN: f64 = 1.5;

    fn clear_ahead() -> Option<Obstacle> {
        Some(Obstacle {
            ahead_m: f64::INFINITY,
            away: 1.0,
            age_s: 0.0,
        })
    }

    fn world(pose: [f64; 3]) -> World {
        World {
            busy: false,
            petting: false,
            heard: None,
            can_ground_pick: true,
            can_sit_stand: true,
            sitting: false,
            obstacle: clear_ahead(),
            pose,
        }
    }

    /// A fake robot around the brain: integrates the twist into a pose (the wander pivot needs
    /// yaw feedback to complete, the novelty grid needs real displacement), runs a 4 s ground
    /// pick and a 2 s sit transition, both busy, as `robotd` does.
    struct Bench {
        brain: Brain,
        pose: [f64; 3],
        busy_ticks: u32,
        sitting: bool,
        sit_flip_pending: bool,
    }

    impl Bench {
        fn new(seed: u64) -> Self {
            Self {
                brain: Brain::new(MAX_SPEED, MAX_TURN, seed),
                pose: [0.0; 3],
                busy_ticks: 0,
                sitting: false,
                sit_flip_pending: false,
            }
        }

        fn tick(&mut self, world_fn: impl Fn(&mut World)) -> Step {
            let dt = 0.02;
            let busy = self.busy_ticks > 0;
            if self.busy_ticks > 0 {
                self.busy_ticks -= 1;
                if self.busy_ticks == 0 && self.sit_flip_pending {
                    self.sitting = !self.sitting;
                    self.sit_flip_pending = false;
                }
            }
            let mut w = world(self.pose);
            w.busy = busy;
            w.sitting = self.sitting;
            world_fn(&mut w);
            let out = self.brain.step(dt, &w);
            self.pose[2] = wrap_angle(self.pose[2] + out.cmd[2] * dt);
            self.pose[0] += out.cmd[0] * self.pose[2].cos() * dt;
            self.pose[1] += out.cmd[0] * self.pose[2].sin() * dt;
            if out.ground_pick && !busy {
                self.busy_ticks = 200;
            }
            if out.sit_toggle && !busy {
                self.busy_ticks = 100;
                self.sit_flip_pending = true;
            }
            out
        }
    }

    /// Ten minutes at 50 Hz: everything stays in range, and the duck actually does things —
    /// walks, stands still more than it walks, picks, naps and stands back up.
    #[test]
    fn ten_minutes_of_duck_stay_in_bounds_and_do_things() {
        let mut bench = Bench::new(0x5eed);
        let (mut walk, mut still, mut picks, mut naps, mut quacks) = (0, 0, 0, 0, 0);
        let mut seen = std::collections::HashSet::new();
        let mut farthest = 0.0f64;
        for _ in 0..(10 * 60 * 50) {
            let out = bench.tick(|_| {});
            seen.insert(bench.brain.state());
            farthest = farthest.max(bench.pose[0].hypot(bench.pose[1]));
            assert!(
                (0.0..=MAX_SPEED + 1e-9).contains(&out.cmd[0]),
                "vx out of range: {}",
                out.cmd[0]
            );
            assert!(out.cmd[1] == 0.0, "the brain never strafes");
            assert!(
                out.cmd[2].abs() <= MAX_TURN + 0.6 + 1e-9,
                "wz: {}",
                out.cmd[2]
            );
            assert!((0.0..=1.0).contains(&out.mouth), "mouth: {}", out.mouth);
            assert!((0.0..=1.0).contains(&out.head_alpha));
            for h in out.head {
                assert!(h.abs() <= 1.5, "head target out of range: {h}");
            }
            assert!(out.body[0].abs() <= 0.025 && out.body[1].abs() <= 0.2618);
            assert!(out.body[2].abs() <= 0.2618);
            if out.cmd[0] > 0.01 {
                walk += 1;
            } else {
                still += 1;
            }
            picks += usize::from(out.ground_pick);
            naps += usize::from(out.sit_toggle && !bench.sitting);
            quacks += usize::from(out.sound == Some(Sound::Chirp));
        }
        // The runtime's bounds: wandering is deliberately dominant, so ask only that the duck walks
        // a good deal and still spends real time standing around being a duck.
        assert!(walk > 60 * 50, "barely walked: {walk} ticks");
        assert!(
            still > 60 * 50,
            "basically never stood still: {still} ticks"
        );
        // On a robot the random-heading wander stayed within half a metre of where it started.
        // Walking to goal points is what makes it range.
        assert!(
            farthest > 2.0,
            "never got far from the start: {farthest:.2} m"
        );
        assert!(picks > 0, "never pecked");
        assert!(naps > 0, "never napped");
        assert!(quacks >= 20, "too quiet: {quacks}");
        assert!(
            !bench.sitting || bench.brain.state() == State::Nap,
            "stuck seated"
        );
        for state in [
            State::Wander,
            State::TurnInPlace,
            State::LookAround,
            State::Chill,
        ] {
            assert!(seen.contains(&state), "never entered {state:?}");
        }
    }

    /// With odometry believed, a wander bout walks to its goal and stops there — not short of it
    /// on the timer, not past it.
    #[test]
    fn a_wander_bout_walks_to_its_goal() {
        let mut bench = Bench::new(21);
        bench.brain.next_quack_in = 1e9;
        bench.brain.enter(State::Wander);
        let goal = bench.brain.goal.expect("a goal on entry");
        let start = goal[0].hypot(goal[1]);
        assert!(start >= 1.0, "a goal worth walking to: {start:.2} m");
        let mut closest = f64::MAX;
        while bench.brain.state() == State::Wander {
            bench.tick(|_| {});
            closest = closest.min((goal[0] - bench.pose[0]).hypot(goal[1] - bench.pose[1]));
        }
        assert!(closest < GOAL_REACHED_M, "ended {closest:.2} m short");
        assert!(bench.brain.bout.reached);
    }

    /// Where an obstacle stopped the duck, the next goal does not lead back.
    #[test]
    fn a_goal_avoids_where_the_way_was_shut() {
        let mut brain = Brain::new(MAX_SPEED, MAX_TURN, 9);
        // A wall a little ahead in every direction but one: +y.
        for k in 0..16 {
            let a = std::f64::consts::TAU * k as f64 / 16.0;
            if (a - std::f64::consts::FRAC_PI_2).abs() > 1.0 {
                brain.blocked.push(([0.5 * a.cos(), 0.5 * a.sin()], 0.0));
            }
        }
        for _ in 0..20 {
            let goal = brain.pick_goal();
            assert!(goal[1] > 0.5, "goal {goal:?} goes through the wall");
        }
    }

    /// Every start from a standstill gets the kick, whatever stopped the duck before — the twin's
    /// gait does not get going below 0.25 m/s, and a bout that began after Chill used to inherit
    /// the last walk's timer and start at 0.2, standing still with `walk` on its label.
    #[test]
    fn every_walk_starts_with_a_kick() {
        let mut brain = Brain::new(MAX_SPEED, MAX_TURN, 1);
        brain.cmd = [0.2, 0.0, 0.0];
        brain.walking_s = 10.0;
        assert_eq!(brain.walkable(0.2, 0.02), 0.2, "mid-walk: as asked");
        brain.cmd = [0.0; 3];
        assert_eq!(brain.walkable(0.2, 0.02), MAX_SPEED, "a start: the kick");
        brain.cmd = [0.3, 0.0, 0.0];
        for _ in 0..40 {
            brain.walkable(0.2, 0.02);
        }
        assert_eq!(
            brain.walkable(0.05, 0.02),
            MIN_WALK_VX,
            "never below what the gait walks at"
        );
    }

    /// No ToF at all must mean no walking — turning is fine, advancing blind is not.
    #[test]
    fn without_a_tof_the_duck_never_advances() {
        let mut bench = Bench::new(7);
        for _ in 0..(5 * 60 * 50) {
            let out = bench.tick(|w| w.obstacle = None);
            assert_eq!(
                out.cmd[0],
                0.0,
                "advanced blind in {:?}",
                bench.brain.state()
            );
        }
    }

    /// Something at beak distance ahead stops the walk and turns the duck away from it, toward
    /// the side the obstacle model said was clearer.
    #[test]
    fn an_obstacle_at_the_beak_turns_the_duck_away() {
        let mut brain = Brain::new(MAX_SPEED, MAX_TURN, 3);
        brain.enter(State::Wander);
        let mut w = world([0.0; 3]);
        w.obstacle = Some(Obstacle {
            ahead_m: 0.10,
            away: -1.0,
            age_s: 0.0,
        });
        let out = brain.step(0.02, &w);
        assert_eq!(out.cmd[0], 0.0);
        assert!(out.cmd[2] < 0.0, "should turn right, got {}", out.cmd[2]);
        assert_eq!(brain.state(), State::TurnInPlace);
    }

    /// The robot answers the next tick; the state stream says so the tick after. A pick that has
    /// not gone busy yet has not finished — the runtime read "idle" there as "done" in-process,
    /// where there was no round trip to wait for.
    #[test]
    fn a_ground_pick_is_finished_only_after_it_ran() {
        let mut brain = Brain::new(MAX_SPEED, MAX_TURN, 11);
        brain.enter(State::GroundPick);
        let w = world([0.0; 3]);
        assert!(brain.step(0.02, &w).ground_pick, "asks once");
        let out = brain.step(0.02, &w);
        assert!(!out.ground_pick);
        assert_eq!(
            brain.state(),
            State::GroundPick,
            "not running yet is not done"
        );
        let mut busy = w;
        busy.busy = true;
        brain.step(0.02, &busy);
        brain.step(0.02, &w);
        assert_eq!(brain.state(), State::Chill, "ran, and is over");
    }

    /// Seated by someone else (the pad's D-pad down, then the pad let go): the brain adopts it as
    /// a nap and stands the robot back up in the end, rather than never asking.
    #[test]
    fn a_robot_left_seated_is_eventually_stood_up() {
        let mut bench = Bench::new(5);
        bench.sitting = true;
        let mut stood = false;
        for _ in 0..(60 * 50) {
            let out = bench.tick(|_| {});
            if out.sit_toggle && bench.sitting {
                stood = true;
                break;
            }
        }
        assert!(stood, "never asked to stand up");
    }

    /// A voice gets an answer, after a beat.
    #[test]
    fn a_voice_is_answered() {
        let mut brain = Brain::new(MAX_SPEED, MAX_TURN, 13);
        // Silence the spontaneous quacks so the only chirp is the answer.
        brain.next_quack_in = 1e9;
        let mut w = world([0.0; 3]);
        w.heard = Some(Heard::Voice);
        brain.step(0.02, &w);
        w.heard = None;
        let answered = (0..100).any(|_| brain.step(0.02, &w).sound == Some(Sound::Chirp));
        assert!(answered);
    }
}
