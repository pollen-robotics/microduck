//! The mapping itself: one frame of pad in, the intents it means out. `padd`'s crate doc
//! describes what each control does; this is where it is done.

use std::time::{Duration, Instant};

/// Select held this long powers the robot off; let go sooner and the motors go limp at once.
/// Rémi's own setting (2026-10-31), kept off main, where a tap does nothing, a release between
/// [`crate::hold::REST_HOLD`] and [`crate::hold::SHUTDOWN_HOLD`] rests and the latter powers off.
const SELECT_POWER_OFF: Duration = Duration::from_secs(3);

use duck_ipc_proto as proto;
use robotd_params::pad_axes::{AxisBinding, Sticks};

use crate::buttons::Buttons;
use crate::hold::{HOME_HOLD, HoldAction, HoldButton};

/// What the sticks drive, picked on the D-pad. Modal because two sticks cannot express nine
/// degrees of freedom.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    /// D-pad left. Left stick walks and strafes, right stick turns.
    Drive,
    /// D-pad up. All four axes pose the head; the body holds still.
    Head,
    /// D-pad right. Left stick walks and turns, right stick looks around — or, with
    /// `[pad_imu_head_control]` on a pad that has an IMU, the pad's tilt poses the head and the
    /// sticks keep the whole [`Mode::Drive`] mapping.
    HeadDrive,
    /// D-pad down. Body + head: the left stick crouches and leans the standing robot sideways,
    /// the right stick looks around. The body does not walk.
    BodyPose,
    /// D-pad left pressed again while driving (and again to leave). A stick flick is one
    /// precision nudge — a trained one-shot skill that moves the robot 1.5 cm or turns it 7.5° and
    /// stops still: the left stick forward, back, left or right, snapped to its dominant axis so a
    /// slightly diagonal finger is still a pure request; the right stick left or right turns. The
    /// walk is fed a zero twist meanwhile. See [`precision_nudge`].
    Precision,
}

impl Mode {
    /// Whether this mode sends head poses, so leaving it has a head to put back.
    fn poses_head(self) -> bool {
        matches!(self, Self::Head | Self::HeadDrive | Self::BodyPose)
    }
}

/// How far a stick must go for a flick to be a nudge, and how close to the centre both sticks must
/// come back before the next one counts. Far apart on purpose: a stick drifting back through the
/// middle of its travel must not read as a second flick.
pub const NUDGE_FLICK: f64 = 0.6;
pub const NUDGE_CENTRE: f64 = 0.3;

/// The nudge skills a flick asks for, by name (`robotctl policy add nudge_fwd …`). A robot without
/// them answers the request with the skills it does have, which `report` logs.
const NUDGE_FWD: &str = "nudge_fwd";
const NUDGE_BACK: &str = "nudge_back";
const NUDGE_LEFT: &str = "nudge_left";
const NUDGE_RIGHT: &str = "nudge_right";
const NUDGE_TURN_LEFT: &str = "nudge_turn_left";
const NUDGE_TURN_RIGHT: &str = "nudge_turn_right";

/// The nudge a precision-mode frame asks for, if any: the left stick first, snapped to whichever of
/// its axes is further out (forward/back or left/right, never between), else the right stick's
/// left/right as a turn. Signs are gilrs's — up and right read positive — so right is a nudge to
/// the robot's right and a clockwise turn.
pub fn precision_nudge(left_x: f64, left_y: f64, right_x: f64) -> Option<&'static str> {
    if left_x.abs().max(left_y.abs()) >= NUDGE_FLICK {
        Some(if left_y.abs() >= left_x.abs() {
            if left_y > 0.0 { NUDGE_FWD } else { NUDGE_BACK }
        } else if left_x > 0.0 {
            NUDGE_RIGHT
        } else {
            NUDGE_LEFT
        })
    } else if right_x.abs() >= NUDGE_FLICK {
        Some(if right_x > 0.0 {
            NUDGE_TURN_RIGHT
        } else {
            NUDGE_TURN_LEFT
        })
    } else {
        None
    }
}

/// What has to be sent when the sticks stop meaning `from` and start meaning `to`.
///
/// Leaving a mode puts back what it moved, because nothing else will: a body left leaning or a
/// head left turned stays that way, and the next mode does not send the joints the last one did.
/// Moving between modes that all pose the head keeps the head, since the next one goes on posing
/// it.
fn mode_exit_calls(from: Mode, to: Mode) -> Vec<proto::Call> {
    let mut calls = Vec::new();
    if from == Mode::BodyPose && to != Mode::BodyPose {
        calls.push(proto::Call::RobotPose(proto::PoseParams {
            active: false,
            ..Default::default()
        }));
    }
    if from.poses_head() && !to.poses_head() {
        calls.push(proto::Call::RobotHead(proto::HeadParams::default()));
    }
    calls
}

/// The head pose for a pad attitude relative to its reference.
///
/// `relative` is body → world of the pad now, in the frame of the reference — [`pad_imu::relative`].
/// Its pitch, roll and yaw become the head's, scaled by `gain` and clamped to each joint's
/// `[pad_head]` limit — the same travel the sticks get. The signs
/// are the ones that made the head copy the pad on the robot (2026-09-09): pad nose-up is a
/// positive `head_pitch`, pad yaw to the left a positive `head_yaw`, and the pad rolling right
/// (left side up) a negative `head_roll`. Pitch and roll came out opposite to the stick mapping's
/// guess, which is worth knowing: the sticks' signs describe "stick up looks up", not the joint
/// axes, and the pad frame is the joints'. The neck stays at zero: one pitch joint is enough to
/// follow a wrist.
fn head_from_pad(
    relative: [f32; 4],
    gain: f64,
    limits: &robotd_params::PadHeadParams,
) -> proto::HeadParams {
    let [pitch, roll, yaw] = pad_imu::euler_deg(relative);
    let angle = |degrees: f32, max: f64| (f64::from(degrees).to_radians() * gain).clamp(-max, max);
    proto::HeadParams {
        neck_pitch: 0.0,
        head_pitch: angle(pitch, limits.head_pitch_max),
        head_yaw: angle(yaw, limits.head_yaw_max),
        head_roll: -angle(roll, limits.head_roll_max),
    }
}

/// One instant of a pad, from whichever daemon read it. Sticks are raw — the deadzone is
/// applied here, so both sources get the same one — and `pressed`/`released` are the edges since
/// the previous frame, which the source is responsible for not losing.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct PadFrame {
    pub left_x: f64,
    pub left_y: f64,
    pub right_x: f64,
    pub right_y: f64,
    pub lt: f64,
    pub rt: f64,
    pub held: Buttons,
    pub pressed: Buttons,
    pub released: Buttons,
    /// The pad's IMU attitude, when `[pad_imu_head_control]` is on and the pad has said
    /// something believable. Always `None` from `netpadd`.
    pub attitude: Option<[f32; 4]>,
    /// Whether the pad has an IMU at all, for the "no attitude yet" warning.
    pub has_imu: bool,
}

/// How far a trigger goes down before it sounds: RT quacks on crossing it, LT rides the wheee
/// while past it. The prototype's threshold, and `duckctl udp-pad`'s, which plays the same cues
/// on the laptop it reads the pad from.
pub const SOUND_THRESHOLD: f64 = 0.3;

/// Everything the mapping reads that is not the pad: the controller sections of `robotd.toml`,
/// read by [`Config::reload`], and the robot's drive mode, which the daemon asks for.
#[derive(Debug, Clone)]
pub struct Config {
    pub bindings: robotd_params::PadParams,
    pub imu_head: robotd_params::PadImuHeadControlParams,
    /// `[pad_drive]`: full deflection while walking.
    pub drive: robotd_params::PadDriveParams,
    /// `[pad_axes]`: the deadzone, and which stick drives which command in each mode.
    pub axes: robotd_params::PadAxesParams,
    /// `[pad_head]`: full-deflection head travel per joint.
    pub head: robotd_params::PadHeadParams,
    /// `[pad_body]`: body + head mode's height and tilt ranges.
    pub body: robotd_params::PadBodyParams,
    /// `[pad_roller]`: full deflection on wheels, in place of `drive`.
    pub roller_limits: robotd_params::PadRollerParams,
    pub roller: bool,
}

/// What to send, in order. A `Request` is answered — "refused, and here is why" is a real
/// outcome — and the daemon hands the answer to [`report`]; a `Notify` is not.
#[derive(Debug, Clone, PartialEq)]
pub enum Out {
    Notify(proto::Call),
    Request(proto::Call),
}

/// The mapping's memory between frames.
#[derive(Debug)]
pub struct Mapper {
    mode: Mode,
    /// The pad attitude that reads as "head centred" while head + move follows the pad's IMU.
    /// `None` whenever it is not following — another mode, the feature off, no IMU, or the pad
    /// gone: a reference taken against one pad means nothing to the next.
    imu_reference: Option<[f32; 4]>,
    start: HoldButton,
    select: HoldButton,
    /// Trigger levels last tick, for the sound edges: RT quacks on its rising edge, LT
    /// starts the wheee ride. The prototype's threshold.
    prev_rt: f64,
    prev_lt: f64,
    /// Does this pad think the robot is up (torque on, at the home pose)? When not, Start sends
    /// `robot.init` (stand up and hold) instead of enabling the policy; the next Start enables it.
    /// Starts false: padd starts with the robot, and a robotd restart restarts padd too.
    up: bool,
    /// Precision mode: both sticks have been back near the centre since the last nudge, so the
    /// next flick counts. False on entering the mode, so a stick already held does not fire.
    nudge_armed: bool,
}

impl Default for Mapper {
    fn default() -> Self {
        Self::new()
    }
}

impl Mapper {
    pub fn new() -> Self {
        Self {
            mode: Mode::Drive,
            imu_reference: None,
            start: HoldButton::default(),
            select: HoldButton::default(),
            prev_rt: 0.0,
            prev_lt: 0.0,
            up: false,
            nudge_armed: false,
        }
    }

    /// The pad went away. A hold in flight was measured against it — see [`HoldButton::reset`] —
    /// and an IMU reference means nothing to the next pad.
    pub fn pad_gone(&mut self) {
        self.start.reset();
        self.select.reset();
        self.imu_reference = None;
    }

    /// One frame: the discrete intents into `out`, in the order they must be sent, and the
    /// continuous ones into `frame` for [`crate::Continuous`]. Both are cleared first.
    pub fn tick(
        &mut self,
        pad: &PadFrame,
        cfg: &Config,
        now: Instant,
        out: &mut Vec<Out>,
        frame: &mut Vec<proto::Call>,
    ) {
        out.clear();
        frame.clear();

        // The pad's attitude this tick, when the feature is on and the pad has an IMU that has
        // said something believable. `None` is every other case, and head + move then poses the
        // head from the right stick.
        let attitude = pad.attitude;
        if attitude.is_none() && self.imu_reference.is_some() {
            // The feature went off, or the IMU went away under us. Not silent: a head that stops
            // following mid-turn wants a line in the journal saying why.
            tracing::info!("IMU head control off — the right stick poses the head");
            self.imu_reference = None;
        }

        // The D-pad picks what the sticks mean. Two arrows pressed inside one frame resolve in
        // this fixed order, the last one winning — which is what gilrs's event order did in
        // `padd` for the only case that matters, one arrow at a time.
        let mut wanted_mode = None;
        for (button, mode) in [
            (Buttons::UP, Mode::Head),
            (Buttons::RIGHT, Mode::HeadDrive),
            (Buttons::LEFT, Mode::Drive),
            (Buttons::DOWN, Mode::BodyPose),
        ] {
            if pad.pressed.contains(button) {
                wanted_mode = Some(mode);
            }
        }
        // D-pad left pressed while already driving enters precision mode; pressed in precision
        // mode it goes back to driving. Every other arrow leaves precision for its own mode.
        if wanted_mode == Some(Mode::Drive) && pad.pressed.contains(Buttons::LEFT) {
            wanted_mode = Some(if self.mode == Mode::Drive {
                Mode::Precision
            } else {
                Mode::Drive
            });
        }
        // Which bindable buttons went down this frame, by their config name. A list rather than
        // a flag apiece, because what each one runs is config now and this mapping does not know.
        let pressed = Buttons::BINDABLE
            .iter()
            .filter(|(b, _)| pad.pressed.contains(*b))
            .map(|(_, n)| *n);

        // Start: a tap stands the robot up, then toggles the policy; held, it is the way home.
        let mut go_home = false;
        match self.start.tick(
            pad.held.contains(Buttons::START),
            pad.released.contains(Buttons::START),
            now,
            &[HOME_HOLD],
        ) {
            // Start's one threshold acts when it is reached; its release says nothing more.
            HoldAction::Nothing | HoldAction::ReleasedAfter(_) => {}
            HoldAction::Reached(_) => go_home = true,
            HoldAction::Tap if !self.up => {
                tracing::warn!("Start — robot.init: standing up. Press Start again to drive");
                out.push(Out::Request(proto::Call::RobotInit));
                // `padd` set this on the answer and exited on a failed send, so the next frame
                // sees the same state either way.
                self.up = true;
            }
            HoldAction::Tap => {
                // The robot owns the toggle. A local on/off belief here drifts from the
                // robot's the moment anything else moves it — robot.relax, the shutdown
                // sequence, either side restarting — and a stale belief turns Start into a
                // button that does nothing every other press. `toggle` flips the robot's own
                // state; turning OFF returns it to the home pose (the prototype's "returning
                // to default pose"), so turning on always starts the policy from home. The
                // answer is logged by [`report`].
                out.push(Out::Request(proto::Call::RobotEnable(
                    proto::EnableParams {
                        on: false,
                        toggle: true,
                    },
                )));
            }
        }

        if go_home {
            // Everything back: the sticks to plain driving, the policy off, and the robot ramped
            // to its home pose with torque on — from limp, from mid-walk or from a crouch alike.
            // The disable comes first so the policy cannot pick the robot up again the moment the
            // ramp ends, which is what `robot.init` alone does on a robot that is driving.
            tracing::warn!("Start held — home pose, motors stiff, policy off");
            for call in mode_exit_calls(self.mode, Mode::Drive) {
                out.push(Out::Notify(call));
            }
            self.mode = Mode::Drive;
            self.imu_reference = None;
            let disable = proto::Call::RobotEnable(proto::EnableParams {
                on: false,
                toggle: false,
            });
            for call in [disable, proto::Call::RobotInit] {
                out.push(Out::Request(call));
            }
            self.up = true;
        }

        if let Some(next) = wanted_mode {
            if next != self.mode {
                for call in mode_exit_calls(self.mode, next) {
                    out.push(Out::Notify(call));
                }
                // The mode is otherwise invisible on the robot: a chirp going in, a coo coming out.
                if next == Mode::Precision || self.mode == Mode::Precision {
                    let tag = if next == Mode::Precision {
                        proto::SoundTag::Chirp
                    } else {
                        proto::SoundTag::Coo
                    };
                    out.push(Out::Notify(proto::Call::RobotSound(proto::SoundParams {
                        tag,
                        hold: None,
                    })));
                }
                if next == Mode::Precision {
                    self.nudge_armed = false;
                    tracing::info!(
                        "precision mode: a stick flick is one nudge; D-pad left to leave"
                    );
                }
                self.mode = next;
                tracing::info!(mode = ?self.mode, "mode");
            }
            // Head + move follows the pad's IMU when it can, from wherever the pad is at the
            // press — and pressing it again re-centres, which is how a person beats the gyro's
            // yaw drift without a magnetometer.
            self.imu_reference = if self.mode == Mode::HeadDrive {
                attitude
            } else {
                None
            };
            if self.mode == Mode::HeadDrive {
                if self.imu_reference.is_some() {
                    tracing::info!("IMU head control: following the pad from here");
                } else if cfg.imu_head.enabled && pad.has_imu {
                    // The IMU is there and has not spoken yet — a second after connecting,
                    // typically. Saying so beats silently doing the other thing.
                    tracing::warn!(
                        "the pad's IMU has no attitude yet; the right stick has the head this once"
                    );
                }
            }
        }

        // One-shot skills. Answered, because "refused, and here is why" is a real outcome — a
        // skill this robot does not have, one mid-flight, or the policy not driving.
        //
        // The name comes from config and is sent as it was written. `padd` does not check it
        // against anything: which skills exist is the robot's to know, and it answers an unknown
        // one with the list it does have, which is a better error than this side could give.
        for button in pressed {
            // An empty binding is a button switched off on purpose, not a fault.
            let skill = cfg.bindings.skill(button).unwrap_or_default();
            if skill.is_empty() {
                tracing::debug!(button, "no skill bound");
                continue;
            }
            out.push(Out::Request(proto::Call::RobotDo(proto::DoParams {
                skill: skill.to_owned(),
            })));
        }

        // Precision mode: one nudge per flick. Answered, so a refusal says why in the journal — a
        // nudge already running (robotd refuses a skill mid-flight; the flick is dropped, not
        // queued), or a robot without the nudge skills (it lists the ones it has).
        if self.mode == Mode::Precision {
            let centred = pad.left_x.abs().max(pad.left_y.abs()) < NUDGE_CENTRE
                && pad.right_x.abs() < NUDGE_CENTRE;
            if centred {
                self.nudge_armed = true;
            } else if self.nudge_armed
                && let Some(skill) = precision_nudge(pad.left_x, pad.left_y, pad.right_x)
            {
                tracing::info!(skill, "precision nudge");
                out.push(Out::Request(proto::Call::RobotDo(proto::DoParams {
                    skill: skill.to_owned(),
                })));
                self.nudge_armed = false;
            }
        }

        // Home: the flashlight. Not bindable — it is not a skill, and the robot owns the toggle.
        // A request, so a board without a flashlight says so in the journal rather than not at
        // all.
        if pad.pressed.contains(Buttons::HOME) {
            out.push(Out::Request(proto::Call::RobotFlashlight(
                proto::FlashlightParams {
                    toggle: true,
                    ..Default::default()
                },
            )));
        }

        // X held: keep a chaining skill going. The robot starts another when a request lands
        // near the end of the current one, so "held" is spelled "resent every tick" — as a
        // notification, because fifty answered requests a second would spend their time waiting
        // on replies, and the press above already got the real answer.
        //
        // Whatever X is bound to, nothing by default: a skill that does not chain simply refuses
        // the resend, which costs a notification nobody reads. Only X, because it is the button
        // the prototype held the roulade on.
        let held = cfg.bindings.skill("x").unwrap_or_default();
        if pad.held.contains(Buttons::X) && !pad.pressed.contains(Buttons::X) && !held.is_empty() {
            out.push(Out::Notify(proto::Call::RobotDo(proto::DoParams {
                skill: held.to_owned(),
            })));
        }

        // Select: let go before three seconds and the motors go limp now — torque off and every
        // servo rebooted where the robot is, no sit first. Held to three, the gentle power-off:
        // sit, rest pose, then power off. The limp waits for the release because until then the
        // same press may still become the power-off, which needs a robot still up to sit with.
        match self.select.tick(
            pad.held.contains(Buttons::SELECT),
            pad.released.contains(Buttons::SELECT),
            now,
            &[SELECT_POWER_OFF],
        ) {
            HoldAction::Tap => {
                tracing::warn!(
                    "Select — robot.rebootMotors: limp now, torque off and servo reboot"
                );
                // The robot ends limp, so the next Start stands it up again rather than toggling
                // the policy on a robot that is lying on the floor.
                self.up = false;
                out.push(Out::Request(proto::Call::RobotRebootMotors(
                    proto::RebootMotorsParams::default(),
                )));
            }
            HoldAction::Reached(_) => {
                self.up = false;
                tracing::warn!("Select held 3 s — asking the robot to power off");
                out.push(Out::Request(proto::Call::RobotShutdown));
            }
            HoldAction::Nothing | HoldAction::ReleasedAfter(_) => {}
        }

        // Either trigger opens the mouth; the max wins, as in the prototype — where RT
        // also chirps and LT rides the wheee, which they now do here too.
        let rt = pad.rt;
        let lt = pad.lt;
        let mouth = rt.max(lt);
        out.push(Out::Notify(proto::Call::RobotMouth(proto::MouthParams {
            open: mouth,
        })));

        // Chirp on the right trigger's rising edge; the robot cuts off a still-playing
        // sound, so rapid pulses quack rapidly. The wheee rides the left trigger: start on
        // press, then a hold notification per tick — the robot treats the hold as a level
        // that decays, so a padd that dies mid-ride leaves a ride that lands. Release cuts
        // it instantly, as the prototype does.
        if self.prev_rt < SOUND_THRESHOLD && rt >= SOUND_THRESHOLD {
            out.push(Out::Notify(proto::Call::RobotSound(proto::SoundParams {
                tag: proto::SoundTag::Chirp,
                hold: None,
            })));
        }
        if lt >= SOUND_THRESHOLD {
            out.push(Out::Notify(proto::Call::RobotSound(proto::SoundParams {
                tag: proto::SoundTag::Wheee,
                hold: Some(true),
            })));
        } else if self.prev_lt >= SOUND_THRESHOLD {
            out.push(Out::Notify(proto::Call::RobotSound(proto::SoundParams {
                tag: proto::SoundTag::Wheee,
                hold: Some(false),
            })));
        }
        self.prev_rt = rt;
        self.prev_lt = lt;

        let sticks = Sticks {
            left_x: pad.left_x,
            left_y: pad.left_y,
            right_x: pad.right_x,
            right_y: pad.right_y,
        };
        let imu = match (self.mode, self.imu_reference, attitude) {
            (Mode::HeadDrive, Some(reference), Some(now)) => Some((reference, now)),
            _ => None,
        };
        // This tick's continuous intents, as one frame. Reused rather than built fresh:
        // a `Vec` per tick is an allocation fifty times a second to say what the sticks
        // were doing, which is the shape of thing this loop is meant not to do.
        stick_frame(self.mode, sticks, imu, cfg, frame);
    }
}

/// What the sticks ask for in `mode`, appended to `frame`. Which stick drives what, which way
/// round and how far is all `[pad_axes]` and its limits; what each mode sends is fixed here.
fn stick_frame(
    mode: Mode,
    sticks: Sticks,
    imu: Option<([f32; 4], [f32; 4])>,
    cfg: &Config,
    frame: &mut Vec<proto::Call>,
) {
    let value = |binding: &AxisBinding| binding.evaluate(sticks, cfg.axes.deadzone);
    let scale = robotd_params::PadDriveParams::scale;
    let roller_limits = cfg.roller_limits.limits();
    let limits = if cfg.roller {
        &roller_limits
    } else {
        &cfg.drive
    };
    let walk = |vx: &AxisBinding, vy: &AxisBinding, vyaw: &AxisBinding| {
        proto::Call::RobotMove(proto::MoveParams {
            vx: scale(value(vx), limits.vx_min, limits.vx_max),
            vy: scale(value(vy), limits.vy_min, limits.vy_max),
            vyaw: scale(value(vyaw), limits.vyaw_min, limits.vyaw_max),
        })
    };
    let head = &cfg.head;
    let look = |neck: &AxisBinding, pitch: &AxisBinding, yaw: &AxisBinding, roll: &AxisBinding| {
        proto::Call::RobotHead(proto::HeadParams {
            neck_pitch: value(neck) * head.neck_pitch_max,
            head_pitch: value(pitch) * head.head_pitch_max,
            head_yaw: value(yaw) * head.head_yaw_max,
            head_roll: value(roll) * head.head_roll_max,
        })
    };
    match mode {
        Mode::Drive => {
            let a = &cfg.axes.drive;
            frame.push(walk(&a.vx, &a.vy, &a.vyaw));
        }
        Mode::HeadDrive => match imu {
            // The pad's tilt has the head, so the sticks keep the whole drive mapping. In the
            // same frame: the pad's tilt and the sticks describe one instant.
            Some((reference, now)) => {
                let a = &cfg.axes.drive;
                frame.push(walk(&a.vx, &a.vy, &a.vyaw));
                frame.push(proto::Call::RobotHead(head_from_pad(
                    pad_imu::relative(reference, now),
                    cfg.imu_head.gain,
                    head,
                )));
            }
            None => {
                let a = &cfg.axes.head_drive;
                frame.push(walk(&a.vx, &a.vy, &a.vyaw));
                frame.push(look(
                    &a.neck_pitch,
                    &a.head_pitch,
                    &a.head_yaw,
                    &a.head_roll,
                ));
            }
        },
        Mode::Head => {
            // The body must not keep its last velocity while the sticks are posing the
            // head. The deadman would catch it eventually; a robot that keeps walking
            // because you started moving its head is a bad enough surprise to be
            // explicit about.
            //
            // In the same frame as the head rather than a notification of its own: the
            // two describe one instant, and sending them separately was two `write_all`
            // and two `flush` syscalls a tick to say so.
            frame.push(proto::Call::RobotMove(proto::MoveParams::default()));
            let a = &cfg.axes.head;
            frame.push(look(
                &a.neck_pitch,
                &a.head_pitch,
                &a.head_yaw,
                &a.head_roll,
            ));
        }
        // A fresh zero twist every tick rather than none: the twist's age feeds robotd's deadman,
        // and a nudge is a skill that does not read it anyway.
        Mode::Precision => frame.push(proto::Call::RobotMove(proto::MoveParams::default())),
        Mode::BodyPose => {
            frame.push(proto::Call::RobotMove(proto::MoveParams::default()));
            let (a, body) = (&cfg.axes.body_pose, &cfg.body);
            frame.push(proto::Call::RobotPose(proto::PoseParams {
                z: scale(value(&a.z), body.z_min, body.z_max),
                pitch: value(&a.pitch) * body.pitch_max,
                roll: value(&a.roll) * body.roll_max,
                active: true,
            }));
            frame.push(look(
                &a.neck_pitch,
                &a.head_pitch,
                &a.head_yaw,
                &a.head_roll,
            ));
        }
    }
}

/// Log what the robot said to a [`Out::Request`], the way `padd` always has: a refusal or a
/// not-accepted at `warn`, and the policy toggle's outcome, because the robot owns that state and
/// its answer is the only account of it.
///
/// The policy line is written for every answer to the toggle, a refusal and an unreadable answer
/// included — it then says "toggled", as it always did.
pub fn report(call: &proto::Call, response: Option<&proto::Response>) {
    let result = response.and_then(|r| r.result_as::<proto::IntentResult>().ok());
    if let Some(reason) = refusal(response) {
        tracing::warn!(call = call.method(), %reason, "refused");
    }
    if let proto::Call::RobotEnable(proto::EnableParams { toggle: true, .. }) = call {
        // The robot names the state it ended in; that is the log, since padd no longer has a
        // belief of its own to report.
        let outcome = result
            .and_then(|r| r.reason)
            .unwrap_or_else(|| "toggled".to_owned());
        tracing::warn!(%outcome, "policy");
    }
}

/// Why the robot did not carry out a request, when it did not: a JSON-RPC error, or an intent
/// answered `accepted: false`. `None` for a request it took.
pub fn refusal(response: Option<&proto::Response>) -> Option<String> {
    let response = response?;
    if let Some(error) = &response.error {
        return Some(error.message.clone());
    }
    let result = response.result_as::<proto::IntentResult>().ok()?;
    (!result.accepted).then(|| {
        result
            .reason
            .unwrap_or_else(|| "refused, no reason given".to_owned())
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> Config {
        Config::defaults(false, &Default::default())
    }

    /// The whole path a frame takes: a D-pad edge changes mode once, a held one does not change
    /// it again, a stick inside the deadzone walks nothing, and A's edge runs A's binding.
    /// Home toggles the flashlight, on its edge: once per press, held or not, and the robot owns
    /// the toggle.
    #[test]
    fn home_toggles_the_flashlight_once_per_press() {
        let (mut m, cfg, t) = (Mapper::new(), cfg(), Instant::now());
        let (mut out, mut frame) = (Vec::new(), Vec::new());
        let flashlight = |out: &[Out]| {
            out.iter()
                .filter(|o| {
                    matches!(
                        o,
                        Out::Request(proto::Call::RobotFlashlight(proto::FlashlightParams {
                            toggle: true,
                            ..
                        }))
                    )
                })
                .count()
        };

        let pad = PadFrame {
            pressed: Buttons::HOME,
            held: Buttons::HOME,
            ..Default::default()
        };
        m.tick(&pad, &cfg, t, &mut out, &mut frame);
        assert_eq!(flashlight(&out), 1, "{out:?}");

        let pad = PadFrame {
            held: Buttons::HOME,
            ..Default::default()
        };
        m.tick(&pad, &cfg, t, &mut out, &mut frame);
        assert_eq!(flashlight(&out), 0, "held is not a second press");
    }

    #[test]
    fn a_frame_becomes_the_intents_padd_always_sent() {
        let (mut m, cfg, t) = (Mapper::new(), cfg(), Instant::now());
        let (mut out, mut frame) = (Vec::new(), Vec::new());

        let pad = PadFrame {
            left_y: 1.0,
            ..Default::default()
        };
        m.tick(&pad, &cfg, t, &mut out, &mut frame);
        assert_eq!(
            frame,
            vec![proto::Call::RobotMove(proto::MoveParams {
                vx: 0.4,
                vy: 0.0,
                vyaw: 0.0
            })]
        );

        let pad = PadFrame {
            left_y: 0.05,
            pressed: Buttons::A,
            held: Buttons::A,
            ..Default::default()
        };
        m.tick(&pad, &cfg, t, &mut out, &mut frame);
        assert_eq!(
            frame,
            vec![proto::Call::RobotMove(proto::MoveParams::default())],
            "deadzone"
        );
        let skill = cfg.bindings.skill("a").unwrap_or_default().to_owned();
        assert!(
            skill.is_empty()
                || out.contains(&Out::Request(proto::Call::RobotDo(proto::DoParams {
                    skill
                }))),
            "{out:?}"
        );

        let pad = PadFrame {
            pressed: Buttons::UP,
            held: Buttons::UP,
            ..Default::default()
        };
        m.tick(&pad, &cfg, t, &mut out, &mut frame);
        assert!(
            frame.iter().any(|c| matches!(c, proto::Call::RobotHead(_))),
            "head mode"
        );
        let pad = PadFrame {
            held: Buttons::UP,
            ..Default::default()
        };
        m.tick(&pad, &cfg, t, &mut out, &mut frame);
        assert!(
            frame.iter().any(|c| matches!(c, proto::Call::RobotHead(_))),
            "still head mode"
        );
    }

    fn nudges(out: &[Out]) -> Vec<String> {
        out.iter()
            .filter_map(|o| match o {
                Out::Request(proto::Call::RobotDo(proto::DoParams { skill }))
                    if skill.starts_with("nudge_") =>
                {
                    Some(skill.clone())
                }
                _ => None,
            })
            .collect()
    }

    fn sounds(out: &[Out]) -> Vec<proto::SoundTag> {
        out.iter()
            .filter_map(|o| match o {
                Out::Notify(proto::Call::RobotSound(proto::SoundParams { tag, .. })) => Some(*tag),
                _ => None,
            })
            .collect()
    }

    fn press(button: Buttons) -> PadFrame {
        PadFrame {
            pressed: button,
            held: button,
            ..Default::default()
        }
    }

    /// Select let go before three seconds reboots every servo, limp at once; held to three it
    /// powers off instead, and that release adds nothing.
    #[test]
    fn select_goes_limp_on_a_short_press_and_powers_off_held_three_seconds() {
        let (mut m, cfg, t) = (Mapper::new(), cfg(), Instant::now());
        let (mut out, mut frame) = (Vec::new(), Vec::new());
        let limp = |out: &Vec<Out>| {
            out.iter()
                .any(|o| matches!(o, Out::Request(proto::Call::RobotRebootMotors(p)) if p.ids.is_empty()))
        };
        let off = |out: &Vec<Out>| {
            out.iter()
                .any(|o| matches!(o, Out::Request(proto::Call::RobotShutdown)))
        };
        let up = PadFrame {
            released: Buttons::SELECT,
            ..Default::default()
        };
        let held = PadFrame {
            held: Buttons::SELECT,
            ..Default::default()
        };

        m.tick(&press(Buttons::SELECT), &cfg, t, &mut out, &mut frame);
        assert!(
            !limp(&out) && !off(&out),
            "nothing while it is down: {out:?}"
        );
        m.tick(
            &up,
            &cfg,
            t + Duration::from_millis(150),
            &mut out,
            &mut frame,
        );
        assert!(limp(&out) && !off(&out), "{out:?}");

        let t = t + Duration::from_secs(10);
        m.tick(&press(Buttons::SELECT), &cfg, t, &mut out, &mut frame);
        m.tick(
            &held,
            &cfg,
            t + Duration::from_millis(2900),
            &mut out,
            &mut frame,
        );
        assert!(!limp(&out) && !off(&out), "{out:?}");
        m.tick(&held, &cfg, t + SELECT_POWER_OFF, &mut out, &mut frame);
        assert!(off(&out) && !limp(&out), "{out:?}");
        m.tick(&up, &cfg, t + Duration::from_secs(4), &mut out, &mut frame);
        assert!(
            !limp(&out) && !off(&out),
            "the release after a power-off: {out:?}"
        );
    }

    /// D-pad left pressed again while driving enters precision mode, with a chirp; pressed again
    /// it leaves, with a coo. The first left only (re)selects driving, as it always did.
    #[test]
    fn d_pad_left_twice_enters_precision_and_again_leaves() {
        let (mut m, cfg, t) = (Mapper::new(), cfg(), Instant::now());
        let (mut out, mut frame) = (Vec::new(), Vec::new());
        m.tick(&press(Buttons::UP), &cfg, t, &mut out, &mut frame);
        m.tick(&press(Buttons::LEFT), &cfg, t, &mut out, &mut frame);
        assert_eq!(m.mode, Mode::Drive, "from head mode, left is plain driving");
        assert!(sounds(&out).is_empty());
        m.tick(&press(Buttons::LEFT), &cfg, t, &mut out, &mut frame);
        assert_eq!(m.mode, Mode::Precision);
        assert_eq!(sounds(&out), vec![proto::SoundTag::Chirp]);
        m.tick(&press(Buttons::LEFT), &cfg, t, &mut out, &mut frame);
        assert_eq!(m.mode, Mode::Drive);
        assert_eq!(sounds(&out), vec![proto::SoundTag::Coo]);
    }

    /// A flick is one nudge, snapped to the stick's dominant axis; holding it does not repeat, and
    /// the stick must come back near the centre before the next one counts. Meanwhile the walk
    /// gets a zero twist every tick.
    #[test]
    fn a_flick_is_one_pure_nudge_and_holding_does_not_repeat() {
        let (mut m, cfg, t) = (Mapper::new(), cfg(), Instant::now());
        let (mut out, mut frame) = (Vec::new(), Vec::new());
        // padd starts driving, so one left press is "left again while driving"
        m.tick(&press(Buttons::LEFT), &cfg, t, &mut out, &mut frame);
        assert_eq!(m.mode, Mode::Precision);
        let stick = |lx: f64, ly: f64, rx: f64| PadFrame {
            left_x: lx,
            left_y: ly,
            right_x: rx,
            ..Default::default()
        };
        // centred: arms; then a slightly diagonal flick forward is a pure forward nudge
        m.tick(&stick(0.0, 0.0, 0.0), &cfg, t, &mut out, &mut frame);
        assert_eq!(
            frame,
            vec![proto::Call::RobotMove(proto::MoveParams::default())]
        );
        m.tick(&stick(0.35, 0.9, 0.0), &cfg, t, &mut out, &mut frame);
        assert_eq!(nudges(&out), vec!["nudge_fwd"]);
        assert_eq!(
            frame,
            vec![proto::Call::RobotMove(proto::MoveParams::default())],
            "no twist"
        );
        // held, and wandering through the middle of its travel: nothing more
        for ly in [0.9, 0.7, 0.45, 0.8] {
            m.tick(&stick(0.0, ly, 0.0), &cfg, t, &mut out, &mut frame);
            assert!(nudges(&out).is_empty(), "ly {ly}: {out:?}");
        }
        // back to the centre, then a flick left (mostly sideways) and one with the right stick
        m.tick(&stick(0.1, 0.1, 0.0), &cfg, t, &mut out, &mut frame);
        m.tick(&stick(-0.8, 0.4, 0.0), &cfg, t, &mut out, &mut frame);
        assert_eq!(nudges(&out), vec!["nudge_left"]);
        m.tick(&stick(0.0, 0.0, 0.0), &cfg, t, &mut out, &mut frame);
        m.tick(&stick(0.0, 0.0, 0.9), &cfg, t, &mut out, &mut frame);
        assert_eq!(nudges(&out), vec!["nudge_turn_right"]);
    }

    #[test]
    fn every_direction_snaps_to_its_nudge() {
        assert_eq!(precision_nudge(0.0, 1.0, 0.0), Some("nudge_fwd"));
        assert_eq!(precision_nudge(0.2, -0.7, 0.0), Some("nudge_back"));
        assert_eq!(precision_nudge(-0.9, 0.5, 0.0), Some("nudge_left"));
        assert_eq!(precision_nudge(0.7, 0.69, 0.0), Some("nudge_right"));
        assert_eq!(precision_nudge(0.0, 0.0, -0.8), Some("nudge_turn_left"));
        assert_eq!(precision_nudge(0.0, 0.0, 0.8), Some("nudge_turn_right"));
        assert_eq!(precision_nudge(0.5, 0.5, 0.5), None, "short of a flick");
        // the right stick's up/down is not a nudge
        assert_eq!(precision_nudge(0.0, 0.0, 0.1), None);
    }

    /// A stick already deflected when the mode starts is not a flick; any other arrow leaves the
    /// mode, and holding Start (home) resets it to driving.
    #[test]
    fn entering_with_a_held_stick_does_not_fire_and_other_ways_out() {
        let (mut m, cfg, t) = (Mapper::new(), cfg(), Instant::now());
        let (mut out, mut frame) = (Vec::new(), Vec::new());
        let mut held_fwd = press(Buttons::LEFT);
        held_fwd.left_y = 1.0;
        m.tick(&held_fwd, &cfg, t, &mut out, &mut frame);
        assert_eq!(m.mode, Mode::Precision);
        let still_held = PadFrame {
            left_y: 1.0,
            ..Default::default()
        };
        m.tick(&still_held, &cfg, t, &mut out, &mut frame);
        assert!(nudges(&out).is_empty(), "{out:?}");
        m.tick(&press(Buttons::DOWN), &cfg, t, &mut out, &mut frame);
        assert_eq!(m.mode, Mode::BodyPose);
        assert_eq!(sounds(&out), vec![proto::SoundTag::Coo]);
        m.tick(&press(Buttons::LEFT), &cfg, t, &mut out, &mut frame);
        m.tick(&press(Buttons::LEFT), &cfg, t, &mut out, &mut frame);
        assert_eq!(
            m.mode,
            Mode::Precision,
            "from body pose: left drives, left again is precision"
        );
        let start = PadFrame {
            held: Buttons::START,
            ..Default::default()
        };
        m.tick(&start, &cfg, t, &mut out, &mut frame);
        m.tick(
            &start,
            &cfg,
            t + HOME_HOLD + std::time::Duration::from_millis(50),
            &mut out,
            &mut frame,
        );
        assert_eq!(m.mode, Mode::Drive, "Start held goes home in driving mode");
    }

    /// Head + move with an IMU pad re-centres on every press: the reference is the pad's attitude
    /// at the press, so a re-press after a minute of drift starts the head at centre again.
    #[test]
    fn the_pad_attitude_at_the_press_reads_as_centre() {
        // Yawed 30° by the time of the press — drift, or a turn; the pad cannot tell.
        let yawed = [
            (15.0f32.to_radians()).cos(),
            0.0,
            0.0,
            (15.0f32.to_radians()).sin(),
        ];
        let head = head_from_pad(pad_imu::relative(yawed, yawed), 1.0, &cfg().head);
        assert!(
            head.head_yaw.abs() < 1e-4 && head.head_pitch.abs() < 1e-4,
            "{head:?}"
        );
    }

    /// The pad's tilt becomes the head's pose with the signs verified on the robot: nose up is a
    /// positive head_pitch, yaw left a positive head_yaw, rolled right a negative head_roll. Gain
    /// scales, the travel limit clamps, the neck stays put.
    #[test]
    fn the_head_follows_the_pad_with_the_sticks_signs_gain_and_limit() {
        let half = 15.0f32.to_radians();
        // 30° nose up: a rotation about +Y.
        let nose_up = [half.cos(), 0.0, half.sin(), 0.0];
        let head = head_from_pad(nose_up, 1.0, &cfg().head);
        assert!(
            (head.head_pitch - 30.0f64.to_radians()).abs() < 0.01,
            "{head:?}"
        );
        assert!(
            head.head_yaw.abs() < 0.01 && head.head_roll.abs() < 0.01,
            "{head:?}"
        );
        assert_eq!(head.neck_pitch, 0.0);

        // 30° rolled right (left side up): about +X. The head rolls the other sign.
        let rolled = [half.cos(), half.sin(), 0.0, 0.0];
        let head = head_from_pad(rolled, 1.0, &cfg().head);
        assert!(
            (head.head_roll - (-30.0f64.to_radians())).abs() < 0.01,
            "{head:?}"
        );

        // 30° yaw left: about +Z.
        let left = [half.cos(), 0.0, 0.0, half.sin()];
        let head = head_from_pad(left, 1.0, &cfg().head);
        assert!(
            (head.head_yaw - 30.0f64.to_radians()).abs() < 0.01,
            "{head:?}"
        );

        // Gain 2 doubles it; a yaw limit of 0.5 rad clamps it.
        let head = head_from_pad(left, 2.0, &cfg().head);
        assert!(
            (head.head_yaw - 60.0f64.to_radians()).abs() < 0.01,
            "{head:?}"
        );
        let tight = robotd_params::PadHeadParams {
            head_yaw_max: 0.5,
            ..Default::default()
        };
        let head = head_from_pad(left, 2.0, &tight);
        assert!((head.head_yaw - 0.5).abs() < 1e-6, "{head:?}");
    }

    /// Leaving a mode puts back what it moved, and only that: body + head releases the body,
    /// leaving every head-posing mode for plain driving re-centres the head, and moving between
    /// head-posing modes keeps it.
    #[test]
    fn leaving_a_mode_puts_back_what_it_moved() {
        let is_pose_off = |c: &proto::Call| matches!(c, proto::Call::RobotPose(p) if !p.active);
        let centre = proto::HeadParams::default();
        let is_head_centre =
            |c: &proto::Call| matches!(c, proto::Call::RobotHead(h) if *h == centre);

        let calls = mode_exit_calls(Mode::BodyPose, Mode::Drive);
        assert!(
            calls.len() == 2 && is_pose_off(&calls[0]) && is_head_centre(&calls[1]),
            "{calls:?}"
        );
        for to in [Mode::Head, Mode::HeadDrive] {
            let calls = mode_exit_calls(Mode::BodyPose, to);
            assert!(
                calls.len() == 1 && is_pose_off(&calls[0]),
                "→ {to:?}: {calls:?}"
            );
        }

        for head in [Mode::Head, Mode::HeadDrive] {
            let calls = mode_exit_calls(head, Mode::Drive);
            assert!(
                calls.len() == 1 && is_head_centre(&calls[0]),
                "{head:?}: {calls:?}"
            );
            assert!(mode_exit_calls(head, Mode::BodyPose).is_empty());
        }

        assert!(mode_exit_calls(Mode::Head, Mode::HeadDrive).is_empty());
        assert!(mode_exit_calls(Mode::HeadDrive, Mode::Head).is_empty());
        assert!(mode_exit_calls(Mode::Drive, Mode::Head).is_empty());
        assert!(mode_exit_calls(Mode::Drive, Mode::BodyPose).is_empty());
        assert!(mode_exit_calls(Mode::BodyPose, Mode::BodyPose).is_empty());
    }

    fn sticks(left_x: f64, left_y: f64, right_x: f64, right_y: f64) -> Sticks {
        Sticks {
            left_x,
            left_y,
            right_x,
            right_y,
        }
    }

    fn calls(mode: Mode, cfg: &Config, sticks: Sticks) -> Vec<proto::Call> {
        let mut frame = Vec::new();
        stick_frame(mode, sticks, None, cfg, &mut frame);
        frame
    }

    /// Every command a frame carries, flattened: vx, vy, vyaw, neck_pitch, head_pitch, head_yaw,
    /// head_roll, body z, body pitch, body roll.
    fn flat(calls: &[proto::Call]) -> [f64; 10] {
        let mut out = [0.0; 10];
        for call in calls {
            match call {
                proto::Call::RobotMove(m) => out[..3].copy_from_slice(&[m.vx, m.vy, m.vyaw]),
                proto::Call::RobotHead(h) => out[3..7].copy_from_slice(&[
                    h.neck_pitch,
                    h.head_pitch,
                    h.head_yaw,
                    h.head_roll,
                ]),
                proto::Call::RobotPose(b) => {
                    assert!(b.active);
                    out[7..].copy_from_slice(&[b.z, b.pitch, b.roll]);
                }
                other => panic!("unexpected {other:?}"),
            }
        }
        out
    }

    /// The shipped mapping, one axis at a time and both ways: with nothing in `[pad_axes]`, every
    /// stick in every mode drives what it always has, with the sign it always had. Expected values
    /// are written out, not computed with the evaluator.
    #[test]
    fn every_default_stick_axis_drives_what_it_always_has() {
        let cfg = cfg();
        let (vx, vy, vyaw, head) = (0.4, 0.3, 1.0, 2.5);
        #[rustfmt::skip]
        let cases = [
            (Mode::Drive, sticks(1.0, 0.0, 0.0, 0.0), [0.0, -vy, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            (Mode::Drive, sticks(0.0, 1.0, 0.0, 0.0), [vx, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            (Mode::Drive, sticks(0.0, 0.0, 1.0, 0.0), [0.0, 0.0, -vyaw, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            (Mode::Drive, sticks(0.0, 0.0, 0.0, 1.0), [0.0; 10]),
            (Mode::Head, sticks(1.0, 0.0, 0.0, 0.0), [0.0, 0.0, 0.0, 0.0, 0.0, -head, 0.0, 0.0, 0.0, 0.0]),
            (Mode::Head, sticks(0.0, 1.0, 0.0, 0.0), [0.0, 0.0, 0.0, 0.0, -head, 0.0, 0.0, 0.0, 0.0, 0.0]),
            (Mode::Head, sticks(0.0, 0.0, 1.0, 0.0), [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, head, 0.0, 0.0, 0.0]),
            (Mode::Head, sticks(0.0, 0.0, 0.0, 1.0), [0.0, 0.0, 0.0, head, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            (Mode::HeadDrive, sticks(1.0, 0.0, 0.0, 0.0), [0.0, 0.0, -vyaw, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            (Mode::HeadDrive, sticks(0.0, 1.0, 0.0, 0.0), [vx, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            (Mode::HeadDrive, sticks(0.0, 0.0, 1.0, 0.0), [0.0, 0.0, 0.0, 0.0, 0.0, -head, 0.0, 0.0, 0.0, 0.0]),
            (Mode::HeadDrive, sticks(0.0, 0.0, 0.0, 1.0), [0.0, 0.0, 0.0, 0.0, -head, 0.0, 0.0, 0.0, 0.0, 0.0]),
            (Mode::BodyPose, sticks(1.0, 0.0, 0.0, 0.0), [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2618]),
            (Mode::BodyPose, sticks(0.0, 1.0, 0.0, 0.0), [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.010, 0.0, 0.0]),
            (Mode::BodyPose, sticks(0.0, 0.0, 1.0, 0.0), [0.0, 0.0, 0.0, 0.0, 0.0, -head, 0.0, 0.0, 0.0, 0.0]),
            (Mode::BodyPose, sticks(0.0, 0.0, 0.0, 1.0), [0.0, 0.0, 0.0, 0.0, -head, 0.0, 0.0, 0.0, 0.0, 0.0]),
        ];
        for (mode, positive, expected) in cases {
            for direction in [1.0, -1.0] {
                let s = sticks(
                    positive.left_x * direction,
                    positive.left_y * direction,
                    positive.right_x * direction,
                    positive.right_y * direction,
                );
                let frame = calls(mode, &cfg, s);
                let mut expected = expected.map(|v| v * direction);
                // The crouch goes further down than the rise goes up.
                if mode == Mode::BodyPose && direction < 0.0 && positive.left_y != 0.0 {
                    expected[7] = -0.025;
                }
                assert_eq!(flat(&frame), expected, "{mode:?}, {s:?}");
                let sent = match mode {
                    Mode::Drive => 1,
                    Mode::BodyPose => 3,
                    _ => 2,
                };
                assert_eq!(frame.len(), sent, "{mode:?}: {frame:?}");
            }
        }
    }

    /// Head + move drives with the left stick alone — forward and turn, no strafe — and on wheels
    /// every walking mode takes `[pad_roller]`: push harder than brake, no strafe, slow turns.
    #[test]
    fn on_wheels_the_roller_limits_shape_every_walking_mode() {
        let mut cfg = cfg();
        cfg.drive.vx_min = -0.2;
        let twist = |cfg: &Config, mode, s| flat(&calls(mode, cfg, s));
        let t = twist(&cfg, Mode::HeadDrive, sticks(-1.0, 1.0, 0.0, 0.0));
        assert_eq!(
            &t[..3],
            &[0.4, 0.0, 1.0],
            "stick left turns left (positive yaw)"
        );
        assert_eq!(
            twist(&cfg, Mode::Drive, sticks(0.0, -1.0, 0.0, 0.0))[0],
            -0.2
        );

        cfg.roller = true;
        for (mode, s) in [
            (Mode::Drive, sticks(1.0, -1.0, -1.0, 0.0)),
            (Mode::HeadDrive, sticks(-1.0, -1.0, 0.0, 0.0)),
        ] {
            assert_eq!(&twist(&cfg, mode, s)[..3], &[-0.5, 0.0, 0.3], "{mode:?}");
        }
        assert_eq!(twist(&cfg, Mode::Drive, sticks(0.0, 1.0, 0.0, 0.0))[0], 0.6);
    }

    /// A binding moved, reversed, made gentler or switched off reaches the command, and so does
    /// each limit section.
    #[test]
    fn custom_bindings_and_limits_reach_the_commands() {
        use robotd_params::pad_axes::AxisSource;
        let mut cfg = cfg();
        cfg.axes.drive.vx = AxisBinding {
            source: AxisSource::RightY,
            invert: true,
            gain: 4.0,
        };
        cfg.axes.drive.vyaw.source = AxisSource::None;
        cfg.drive.vx_min = -0.2;
        let t = flat(&calls(Mode::Drive, &cfg, sticks(0.0, 0.0, 1.0, 0.5)));
        assert_eq!(
            (t[0], t[2]),
            (-0.2, 0.0),
            "gain 4 reaches the reverse cap; turn is off"
        );

        cfg.head.head_yaw_max = 0.7;
        cfg.axes.head.head_yaw.source = AxisSource::RightY;
        cfg.axes.head.head_yaw.gain = 0.5;
        let t = flat(&calls(Mode::Head, &cfg, sticks(0.0, 0.0, 0.0, 1.0)));
        assert_eq!(t[5], -0.35);

        cfg.body.pitch_max = 0.2;
        cfg.axes.body_pose.pitch.source = AxisSource::RightX;
        let t = flat(&calls(Mode::BodyPose, &cfg, sticks(0.0, 0.0, 1.0, 0.0)));
        assert_eq!(t[8], 0.2);

        cfg.roller = true;
        cfg.roller_limits.vyaw_max = 0.1;
        let t = flat(&calls(Mode::HeadDrive, &cfg, sticks(-1.0, 0.0, 0.0, 0.0)));
        assert_eq!(t[2], 0.1);
    }

    /// With the pad's IMU on the head, head + move takes the *drive* bindings for the sticks —
    /// not head + move's own — and the head stays inside its per-joint `[pad_head]` limits.
    #[test]
    fn the_imu_head_uses_the_drive_bindings_and_the_head_limits() {
        use robotd_params::pad_axes::AxisSource;
        let mut cfg = cfg();
        cfg.axes.drive.vx.source = AxisSource::RightY;
        cfg.axes.head_drive.vx.source = AxisSource::None;
        cfg.head.head_pitch_max = 0.1;
        let half = 0.25_f32;
        let nose_up = [half.cos(), 0.0, half.sin(), 0.0];
        let mut frame = Vec::new();
        stick_frame(
            Mode::HeadDrive,
            sticks(0.0, 0.0, 0.0, 1.0),
            Some(([1.0, 0.0, 0.0, 0.0], nose_up)),
            &cfg,
            &mut frame,
        );
        let t = flat(&frame);
        assert_eq!(t[0], 0.4);
        assert!((t[4] - 0.1).abs() < 1e-6, "{t:?}");
    }
}
