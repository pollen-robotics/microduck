//! The mapping itself: one frame of pad in, the intents it means out. `padd`'s crate doc
//! describes what each control does; this is where it is done.

use std::time::Instant;

use duck_ipc_proto as proto;

use crate::buttons::Buttons;
use crate::hold::{HOME_HOLD, HoldAction, HoldButton, REST_HOLD, SHUTDOWN_HOLD};

/// Body-pose stick ranges, from the training env via the prototype: z is asymmetric
/// (little headroom up at the standing height, more crouch down), angles capped at ~15°.
const BODY_MAX_Z_UP: f64 = 0.010;
const BODY_MAX_Z_DOWN: f64 = 0.025;
const BODY_MAX_ANGLE: f64 = 0.2618;

/// The prototype's roller-mode stick shaping: push and brake are asymmetric, there is no
/// strafe, and heading is capped at 0.3 rad/s regardless of the walking limits — the
/// roller launch line's `--max-angular-vel 0.3`, unchanged across both of its eras.
const ROLLER_PUSH: f64 = 0.6;
const ROLLER_BRAKE: f64 = 0.5;
const ROLLER_YAW: f64 = 0.3;

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
}

impl Mode {
    /// Whether this mode sends head poses, so leaving it has a head to put back.
    fn poses_head(self) -> bool {
        matches!(self, Self::Head | Self::HeadDrive | Self::BodyPose)
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
/// Its pitch, roll and yaw become the head's, scaled by `gain` and clamped to `max_head`. The signs
/// are the ones that made the head copy the pad on the robot (2026-09-09): pad nose-up is a
/// positive `head_pitch`, pad yaw to the left a positive `head_yaw`, and the pad rolling right
/// (left side up) a negative `head_roll`. Pitch and roll came out opposite to the stick mapping's
/// guess, which is worth knowing: the sticks' signs describe "stick up looks up", not the joint
/// axes, and the pad frame is the joints'. The neck stays at zero: one pitch joint is enough to
/// follow a wrist.
fn head_from_pad(relative: [f32; 4], gain: f64, max_head: f64) -> proto::HeadParams {
    let [pitch, roll, yaw] = pad_imu::euler_deg(relative);
    let angle = |degrees: f32| (f64::from(degrees).to_radians() * gain).clamp(-max_head, max_head);
    proto::HeadParams {
        neck_pitch: 0.0,
        head_pitch: angle(pitch),
        head_yaw: angle(yaw),
        head_roll: -angle(roll),
    }
}

/// How full deflection maps to velocity, for the modes that walk.
#[derive(Debug, Clone, Copy)]
pub(crate) struct DriveLimits<'a> {
    roller: bool,
    /// `[pad_drive]`: each direction of each axis onto its own signed bound.
    drive: &'a robotd_params::PadDriveParams,
}

impl DriveLimits<'_> {
    /// A velocity from three stick axes: forward/back, strafe and turn. Which physical axis is
    /// which depends on the mode; the shaping does not.
    fn walk(&self, forward: f64, strafe: f64, turn: f64) -> proto::MoveParams {
        if self.roller {
            // The prototype's roller shaping: push harder than you can brake, no strafe,
            // heading capped independently of the walking limits.
            return proto::MoveParams {
                vx: forward
                    * if forward >= 0.0 {
                        ROLLER_PUSH
                    } else {
                        ROLLER_BRAKE
                    },
                vy: 0.0,
                vyaw: -turn * ROLLER_YAW,
            };
        }
        let scale = robotd_params::PadDriveParams::scale;
        let d = self.drive;
        proto::MoveParams {
            vx: scale(forward, d.vx_min, d.vx_max),
            // `vy` is positive to the left; stick-left reads negative on every pad gilrs
            // normalises.
            vy: scale(-strafe, d.vy_min, d.vy_max),
            vyaw: scale(-turn, d.vyaw_min, d.vyaw_max),
        }
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

/// Everything the mapping reads that is not the pad.
#[derive(Debug, Clone)]
pub struct Config {
    pub bindings: robotd_params::PadParams,
    pub imu_head: robotd_params::PadImuHeadControlParams,
    pub drive: robotd_params::PadDriveParams,
    pub deadzone: f64,
    pub max_head: f64,
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
        let dz = |v: f64| if v.abs() < cfg.deadzone { 0.0 } else { v };

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

        // Select: nothing on a tap. Let go between two and four seconds, a rest; held to four, a
        // power-off. Both sit and ease into the rest pose before torque goes, which is why the
        // rest waits for the release: until then the same hold may still become a power-off.
        match self.select.tick(
            pad.held.contains(Buttons::SELECT),
            pad.released.contains(Buttons::SELECT),
            now,
            &[REST_HOLD, SHUTDOWN_HOLD],
        ) {
            HoldAction::ReleasedAfter(0) => {
                tracing::warn!(
                    "Select released — robot.rest: sit, rest pose, then torque off and servo reboot"
                );
                // A reboot at the end rather than a bare relax: a servo that tripped its overload
                // comes back with it, so this is also the way out of a tripped servo without
                // pulling the battery. The robot ends limp, so the next Start stands it up again
                // rather than toggling the policy on a robot that is lying on the floor.
                self.up = false;
                out.push(Out::Request(proto::Call::RobotRest));
            }
            HoldAction::Nothing | HoldAction::ReleasedAfter(_) => {}
            HoldAction::Tap => {
                tracing::info!("Select tapped — hold it 2 s to rest, 4 s to power off")
            }
            HoldAction::Reached(0) => {
                tracing::warn!("Select held 2 s — let go to rest, keep holding to power off")
            }
            HoldAction::Reached(_) => {
                self.up = false;
                tracing::warn!("Select held on — asking the robot to power off");
                out.push(Out::Request(proto::Call::RobotShutdown));
            }
        }

        let left_x = dz(pad.left_x);
        let left_y = dz(pad.left_y);
        let right_x = dz(pad.right_x);
        let right_y = dz(pad.right_y);

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
        const SOUND_THRESHOLD: f64 = 0.3;
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

        let limits = DriveLimits {
            roller: cfg.roller,
            drive: &cfg.drive,
        };

        // This tick's continuous intents, as one frame. Reused rather than built fresh:
        // a `Vec` per tick is an allocation fifty times a second to say what the sticks
        // were doing, which is the shape of thing this loop is meant not to do.
        match self.mode {
            Mode::Drive => frame.push(proto::Call::RobotMove(limits.walk(left_y, left_x, right_x))),
            Mode::HeadDrive => match (self.imu_reference, attitude) {
                // The pad's tilt has the head, so the sticks keep the whole drive mapping. In the
                // same frame: the pad's tilt and the sticks describe one instant.
                (Some(reference), Some(now)) => {
                    frame.push(proto::Call::RobotMove(limits.walk(left_y, left_x, right_x)));
                    frame.push(proto::Call::RobotHead(head_from_pad(
                        pad_imu::relative(reference, now),
                        cfg.imu_head.gain,
                        cfg.max_head,
                    )));
                }
                // Left stick walks and turns — no strafe, the right stick is busy — and the right
                // stick looks around, with the signs head mode uses for its left stick.
                _ => {
                    frame.push(proto::Call::RobotMove(limits.walk(left_y, 0.0, left_x)));
                    frame.push(proto::Call::RobotHead(proto::HeadParams {
                        neck_pitch: 0.0,
                        head_pitch: -right_y * cfg.max_head,
                        head_yaw: -right_x * cfg.max_head,
                        head_roll: 0.0,
                    }));
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
                // The prototype's alpha mapping, signs included (its head_pitch/head_yaw
                // joint axes are inverted relative to stick direction — verified on
                // hardware there, kept verbatim here).
                frame.push(proto::Call::RobotHead(proto::HeadParams {
                    neck_pitch: right_y * cfg.max_head,
                    head_pitch: -left_y * cfg.max_head,
                    head_yaw: -left_x * cfg.max_head,
                    head_roll: right_x * cfg.max_head,
                }));
            }
            Mode::BodyPose => {
                frame.push(proto::Call::RobotMove(proto::MoveParams::default()));
                frame.push(proto::Call::RobotPose(proto::PoseParams {
                    z: left_y
                        * if left_y >= 0.0 {
                            BODY_MAX_Z_UP
                        } else {
                            BODY_MAX_Z_DOWN
                        },
                    // No forward/back tilt here: the right stick has the head. The side lean
                    // keeps the old body mode's sign, moved from the right stick to the left.
                    pitch: 0.0,
                    roll: left_x * BODY_MAX_ANGLE,
                    active: true,
                }));
                // The same look-around as head + move, so the right stick means one thing
                // wherever it poses the head.
                frame.push(proto::Call::RobotHead(proto::HeadParams {
                    neck_pitch: 0.0,
                    head_pitch: -right_y * cfg.max_head,
                    head_yaw: -right_x * cfg.max_head,
                    head_roll: 0.0,
                }));
            }
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
    if let Some(response) = response {
        if let Some(error) = &response.error {
            tracing::warn!(code = error.code, message = %error.message, "refused");
        } else if let Some(result) = &result
            && !result.accepted
        {
            tracing::warn!(reason = ?result.reason, "not accepted");
        }
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

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> Config {
        Config {
            bindings: robotd_params::PadParams::default(),
            imu_head: robotd_params::PadImuHeadControlParams::default(),
            drive: robotd_params::PadDriveParams::default(),
            deadzone: 0.1,
            max_head: 2.5,
            roller: false,
        }
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
                vx: 0.3,
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
        let head = head_from_pad(pad_imu::relative(yawed, yawed), 1.0, 2.5);
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
        let head = head_from_pad(nose_up, 1.0, 2.5);
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
        let head = head_from_pad(rolled, 1.0, 2.5);
        assert!(
            (head.head_roll - (-30.0f64.to_radians())).abs() < 0.01,
            "{head:?}"
        );

        // 30° yaw left: about +Z.
        let left = [half.cos(), 0.0, 0.0, half.sin()];
        let head = head_from_pad(left, 1.0, 2.5);
        assert!(
            (head.head_yaw - 30.0f64.to_radians()).abs() < 0.01,
            "{head:?}"
        );

        // Gain 2 doubles it; a limit of 0.5 rad clamps it.
        let head = head_from_pad(left, 2.0, 2.5);
        assert!(
            (head.head_yaw - 60.0f64.to_radians()).abs() < 0.01,
            "{head:?}"
        );
        let head = head_from_pad(left, 2.0, 0.5);
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

    /// Head + move drives with the left stick alone — forward and turn, no strafe — and on wheels
    /// it takes the roller shaping like every other walking mode.
    #[test]
    fn head_and_move_walks_and_turns_from_the_left_stick() {
        let drive = robotd_params::PadDriveParams {
            vx_min: -0.2,
            ..Default::default()
        };
        let walking = DriveLimits {
            roller: false,
            drive: &drive,
        };
        // Left stick up and to the left: forward, turning left.
        let twist = walking.walk(1.0, 0.0, -1.0);
        assert_eq!(twist.vx, 0.3);
        assert_eq!(twist.vy, 0.0);
        assert_eq!(twist.vyaw, 1.5, "stick left turns left (positive yaw)");
        assert_eq!(
            walking.walk(-1.0, 0.0, 0.0).vx,
            -0.2,
            "reverse has its own cap"
        );

        let rolling = DriveLimits {
            roller: true,
            ..walking
        };
        let twist = rolling.walk(-1.0, 1.0, -1.0);
        assert_eq!(twist.vx, -ROLLER_BRAKE);
        assert_eq!(twist.vy, 0.0, "no strafe on wheels");
        assert_eq!(twist.vyaw, ROLLER_YAW);
    }
}
