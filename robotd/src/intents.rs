//! What clients are asking the robot to do.
//!
//! Written by IPC tasks, read once per tick by the control loop. The loop must never wait
//! on a client, so each slot is an [`ArcSwap`]: the reader does one atomic load and the
//! writer one atomic store, and neither can hold up the other.
//!
//! **Twist and head are separate slots on purpose.** A single combined slot would need
//! read-modify-write to update one field, and two clients — a gamepad driving the body and
//! something else driving the head — would silently lose each other's updates. Separate
//! slots make each one single-writer in practice, so last-writer-wins means what it says.
//!
//! Every slot is stamped, because the loop's real question is never "what is the value" but
//! "how old is it". That is what the deadman reads.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};

/// "No mode switch pending" in [`Intents::mode_switch`].
///
/// 255 rather than 0, because 0 is a real mode code and a sentinel that collides with a value is
/// how a "switch to walking" request becomes a no-op.
pub const MODE_NONE: u8 = u8::MAX;

/// Encodings for [`Intents::power`]. An `AtomicU8` rather than two bools, so "init" and "relax"
/// cannot both be pending — they are alternatives, and the last one asked for wins.
const POWER_NONE: u8 = 0;

/// Encodings for [`Intents::theremin`]. An edge rather than a level, and for a concrete
/// reason: the loop puts the instrument down by itself when the robot starts walking, and a
/// level slot still reading "up" would pick it straight back up with a background of
/// somewhere the duck no longer is.
const THEREMIN_NONE: u8 = 0;
const THEREMIN_UP: u8 = 1;
const THEREMIN_DOWN: u8 = 2;
const POWER_INIT: u8 = 1;
const POWER_RELAX: u8 = 2;
use std::time::{Duration, Instant};

use arc_swap::ArcSwap;
use duck_control::NUM_JOINTS;
use duck_control::obs::{BodyPose, Command};

/// A value and when it arrived.
#[derive(Debug, Clone, Copy)]
struct Stamped<T> {
    value: T,
    /// Microseconds since the [`Intents`] epoch.
    at_us: u64,
}

/// The body-pose intent, as the loop consumes it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PoseIntent {
    /// z, roll, pitch — the order the observation's body block uses.
    pub body: [f64; 3],
    /// While true the loop glides toward `body`; false snaps it back to nominal at once,
    /// which is the prototype's B-button exit.
    pub active: bool,
}

impl Default for PoseIntent {
    fn default() -> Self {
        Self {
            body: [0.0; 3],
            active: false,
        }
    }
}

/// External per-joint targets, as they sit in [`Intents`] — the raw payload of
/// `robot.setJoints`, kept on its own [`Stamped`] clock (see [`Intents::external`]).
#[derive(Debug, Clone, Copy, PartialEq)]
struct ExternalTargets {
    /// Absolute joint angles, radians, in `JOINT_NAMES` order.
    targets: [f64; NUM_JOINTS],
    /// The hold gain the client asked for, or `None` to use the mode's running gain.
    gain: Option<u16>,
}

/// The External drive intent, as the loop consumes it: the streamed joint targets, how old
/// the newest frame is (on the external clock, *not* the twist deadman's), and the gain to
/// hold them at. Present in a [`Snapshot`] only while External mode is engaged.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ExternalDrive {
    pub targets: [f64; NUM_JOINTS],
    /// Age of the newest external frame, for [`duck_control::safety::Safety::apply_external`]'s
    /// dedicated external deadman. Measured on the external slot's own stamp, so streaming
    /// joints never refreshes — nor is refreshed by — the twist's deadman clock.
    pub age: Duration,
    /// The per-frame hold gain, or `None` for the mode default.
    pub gain: Option<u16>,
}

/// Pending one-shot skill requests, taken once per tick.
///
/// Booleans rather than a queue: within one 20 ms tick a second press of the same button
/// means nothing extra, and two *different* requests both deserve to be seen — which a
/// single last-writer slot would lose.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SkillRequests {
    pub ground_pick: bool,
    pub kick_left: bool,
    pub kick_right: bool,
    pub sit_toggle: bool,
    /// Start a roll — or, arriving while one runs, chain another. Clients hold a button
    /// down by sending this every tick, so unlike the others it is a *level* in practice.
    pub roulade: bool,
}

impl SkillRequests {
    pub fn any(&self) -> bool {
        self.ground_pick || self.kick_left || self.kick_right || self.sit_toggle || self.roulade
    }
}

// Bit positions for the skill-request mask.
const SKILL_GROUND_PICK: u32 = 1 << 0;
const SKILL_KICK_LEFT: u32 = 1 << 1;
const SKILL_KICK_RIGHT: u32 = 1 << 2;
const SKILL_SIT_TOGGLE: u32 = 1 << 3;
const SKILL_ROULADE: u32 = 1 << 4;

/// How fresh a wheee hold must be to still count as held. `padd` re-notifies every tick
/// (20 ms) while the trigger is down, so anything much older means the client stopped
/// holding — or stopped existing. The stale hold lands the ride instead of looping forever.
pub const WHEEE_HOLD_FRESH: Duration = Duration::from_millis(300);

/// What the wheee level says, as the loop consumes it. Three states rather than a bool
/// because the two ways of *not* being held are different sounds: a client that spells out
/// `hold: false` wants the ride cut where it stands (the prototype's release), while a hold
/// that simply stopped arriving wants the ride played out through its end segment — nobody
/// asked for a cut there, the client just went away. Collapsing them loses the end segment
/// entirely, since a cut ride's writer only ever sees a broken pipe.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WheeeHold {
    /// A `hold: true` fresh enough to trust.
    Held,
    /// A client that is still there and said to stop.
    Released,
    /// A hold that went stale — the client stopped re-notifying, or stopped existing.
    Decayed,
}

pub struct Intents {
    /// Epoch for every stamp. `Instant` so the clock cannot run backwards under us.
    epoch: Instant,
    twist: ArcSwap<Stamped<[f64; 3]>>,
    head: ArcSwap<Stamped<[f64; 4]>>,
    /// Standing body pose. Unstamped: `active: false` is its own "nobody is posing".
    pose: ArcSwap<PoseIntent>,
    /// Mouth opening, 0..1, as `f64::to_bits`. Continuous like the twist, unstamped like
    /// the pose — a mouth left open by a dead client is exactly what the prototype does.
    mouth: std::sync::atomic::AtomicU64,
    /// Whether the policy should drive. Discrete, so a plain flag rather than a slot.
    enabled: AtomicBool,
    /// A pending `robot.init` / `robot.relax`, as [`PowerRequest`].
    ///
    /// A request rather than a flag, and taken rather than read: powering the joints is an *edge*,
    /// not a state the loop should keep re-applying. One `set_torque` is a bus transaction per
    /// joint, so a level here would put sixteen writes into every tick for as long as it stayed set.
    ///
    /// It lives with the intents because this is where the loop reads what clients asked for, and
    /// because the loop is the only thing that may touch the bus — the IPC task cannot do it itself.
    power: AtomicU8,
    /// A pending pick-up or put-down for the ToF theremin. See [`THEREMIN_NONE`].
    theremin: AtomicU8,
    /// The same, for the chorale.
    chorale: AtomicU8,
    /// The piece pinned by the pending chorale request, 0 for none. Written before the
    /// request flag, read after — the flag's swap is the synchronisation point.
    chorale_piece: AtomicU8,
    /// Beacons `btd` has heard, waiting for the loop.
    ///
    /// A queue behind a mutex rather than an `ArcSwap` slot, unlike every other intent here, and
    /// for a specific reason: the other intents are *levels*, where a value missed is a value
    /// superseded. A beacon is an *event* carrying an arrival time, and a beat dropped by
    /// last-writer-wins is a beat the phase lock never gets to average.
    chorale_heard: std::sync::Mutex<Vec<duck_ipc_proto::ChoraleHeard>>,
    /// Pending skill requests, a bitmask taken (swapped to zero) once per tick. A mask
    /// rather than one slot so two different buttons in the same tick both arrive.
    skills: std::sync::atomic::AtomicU32,
    /// A shutdown was requested. A level, not an edge: once asked, the sequence runs.
    shutdown: AtomicBool,
    /// A drive-mode switch was requested, and which mode to switch to.
    ///
    /// An `AtomicU8` holding [`MODE_NONE`] or a mode's code, for the same reason `shutdown` is a
    /// flag: the IPC thread writes and the control loop takes, with nothing to lock. The last
    /// request wins — two arriving in the same tick means somebody pressed twice, and the second
    /// answer is the one they are waiting for.
    mode_switch: AtomicU8,
    /// Pending one-shot sound tags, a bitmask taken once per tick like the skills.
    sounds: std::sync::atomic::AtomicU32,
    /// The wheee hold, as a stamped level: `padd` re-notifies while the trigger is down,
    /// and the loop reads value + age so a dead client's ride decays instead of looping.
    wheee: ArcSwap<Stamped<bool>>,
    /// The newest external joint targets (`robot.setJoints`), on their own stamp — see
    /// [`ExternalDrive::age`] for why the clock is separate from the twist's.
    external: ArcSwap<Stamped<ExternalTargets>>,
    /// Whether the External drive mode is engaged. Set by `robot.setJoints` (which enters the
    /// mode implicitly) and by `robot.setMode "external"`; cleared when the policy is handed
    /// back control (`robot.setMode walk|roller`, `robot.enable on`, `robot.relax`).
    external_active: AtomicBool,
}

/// What a client asked for, once.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PowerRequest {
    /// Torque on, ramp to the home pose.
    Init,
    /// Torque off. The robot collapses if nothing holds it.
    Relax,
}

/// What the loop reads at the top of a tick.
#[derive(Debug, Clone, Copy)]
pub struct Snapshot {
    pub command: Command,
    /// Age of the most recent *twist*, which is what the deadman guards. A stale head pose
    /// is harmless; a stale velocity walks the robot into a wall.
    pub twist_age: Duration,
    pub enabled: bool,
    /// The body-pose intent. The loop smooths `body` into `command.body` itself, because
    /// smoothing is per-tick state the intent slots must not own.
    pub pose: PoseIntent,
    /// Mouth opening, 0..1.
    pub mouth: f64,
    /// The External drive intent, present only while External mode is engaged. When `Some`,
    /// the control loop drives these joint targets in place of the policy, through
    /// [`duck_control::safety::Safety::apply_external`].
    pub external: Option<ExternalDrive>,
}

impl Default for Intents {
    fn default() -> Self {
        Self::new()
    }
}

impl Intents {
    pub fn new() -> Self {
        let epoch = Instant::now();
        Self {
            epoch,
            // Stamped at zero, so before any client connects the twist already reads as
            // maximally stale and the deadman holds the robot still. Starting "fresh" would
            // mean a robot that briefly believes it has a live driver.
            twist: ArcSwap::from_pointee(Stamped {
                value: [0.0; 3],
                at_us: 0,
            }),
            head: ArcSwap::from_pointee(Stamped {
                value: [0.0; 4],
                at_us: 0,
            }),
            pose: ArcSwap::from_pointee(PoseIntent::default()),
            mouth: std::sync::atomic::AtomicU64::new(0.0f64.to_bits()),
            enabled: AtomicBool::new(false),
            power: AtomicU8::new(POWER_NONE),
            theremin: AtomicU8::new(THEREMIN_NONE),
            chorale: AtomicU8::new(THEREMIN_NONE),
            chorale_piece: AtomicU8::new(0),
            chorale_heard: std::sync::Mutex::new(Vec::new()),
            skills: std::sync::atomic::AtomicU32::new(0),
            shutdown: AtomicBool::new(false),
            mode_switch: AtomicU8::new(MODE_NONE),
            sounds: std::sync::atomic::AtomicU32::new(0),
            wheee: ArcSwap::from_pointee(Stamped {
                value: false,
                at_us: 0,
            }),
            // Stamped at zero so, before any frame arrives, an engaged External mode reads as
            // maximally stale and the external deadman holds the robot rather than driving it
            // to a zero pose.
            external: ArcSwap::from_pointee(Stamped {
                value: ExternalTargets {
                    targets: [0.0; NUM_JOINTS],
                    gain: None,
                },
                at_us: 0,
            }),
            external_active: AtomicBool::new(false),
        }
    }

    fn now_us(&self) -> u64 {
        self.epoch.elapsed().as_micros() as u64
    }

    pub fn set_twist(&self, twist: [f64; 3]) {
        self.twist.store(Arc::new(Stamped {
            value: twist,
            at_us: self.now_us(),
        }));
    }

    pub fn set_head(&self, head: [f64; 4]) {
        self.head.store(Arc::new(Stamped {
            value: head,
            at_us: self.now_us(),
        }));
    }

    /// Zero the velocity now. Distinct from the deadman only in that it is deliberate.
    pub fn stop(&self) {
        self.set_twist([0.0; 3]);
    }

    pub fn set_pose(&self, pose: PoseIntent) {
        self.pose.store(Arc::new(pose));
    }

    pub fn set_mouth(&self, open: f64) {
        self.mouth
            .store(open.to_bits(), std::sync::atomic::Ordering::Relaxed);
    }

    /// Queue a one-shot skill for the loop's next tick.
    pub fn request_skill(&self, skill: duck_ipc_proto::Skill) {
        let bit = match skill {
            duck_ipc_proto::Skill::GroundPick => SKILL_GROUND_PICK,
            duck_ipc_proto::Skill::KickLeft => SKILL_KICK_LEFT,
            duck_ipc_proto::Skill::KickRight => SKILL_KICK_RIGHT,
            duck_ipc_proto::Skill::SitToggle => SKILL_SIT_TOGGLE,
            duck_ipc_proto::Skill::Roulade => SKILL_ROULADE,
        };
        self.skills
            .fetch_or(bit, std::sync::atomic::Ordering::Relaxed);
    }

    /// Take the pending skill requests, leaving none. Once per tick, like the power request.
    pub fn take_skills(&self) -> SkillRequests {
        let bits = self.skills.swap(0, std::sync::atomic::Ordering::Relaxed);
        SkillRequests {
            ground_pick: bits & SKILL_GROUND_PICK != 0,
            kick_left: bits & SKILL_KICK_LEFT != 0,
            kick_right: bits & SKILL_KICK_RIGHT != 0,
            sit_toggle: bits & SKILL_SIT_TOGGLE != 0,
            roulade: bits & SKILL_ROULADE != 0,
        }
    }

    /// Queue a sound for the loop's next tick. The wheee is the exception — it is a level,
    /// not an event, so it lands in its stamped slot instead of the mask (a bare
    /// `tag: wheee` with no `hold` is a hold that immediately starts decaying: one short
    /// ride).
    pub fn request_sound(&self, params: duck_ipc_proto::SoundParams) {
        use duck_ipc_proto::SoundTag;
        if params.tag == SoundTag::Wheee {
            self.wheee.store(Arc::new(Stamped {
                value: params.hold.unwrap_or(true),
                at_us: self.now_us(),
            }));
            return;
        }
        let bit = 1u32 << (params.tag as u32);
        self.sounds
            .fetch_or(bit, std::sync::atomic::Ordering::Relaxed);
    }

    /// Take the pending one-shot sounds, leaving none. Once per tick.
    pub fn take_sounds(&self) -> Vec<duck_ipc_proto::SoundTag> {
        use duck_ipc_proto::SoundTag;
        let bits = self.sounds.swap(0, std::sync::atomic::Ordering::Relaxed);
        [
            SoundTag::Alarm,
            SoundTag::Greet,
            SoundTag::Inquire,
            SoundTag::Peck,
            SoundTag::Chirp,
            SoundTag::Coo,
        ]
        .into_iter()
        .filter(|tag| bits & (1u32 << (*tag as u32)) != 0)
        .collect()
    }

    /// The wheee hold as the loop consumes it — see [`WheeeHold`] for why "not held" is
    /// two answers and not one.
    pub fn wheee_hold(&self) -> WheeeHold {
        let stamp = self.wheee.load();
        if !stamp.value {
            return WheeeHold::Released;
        }
        if Duration::from_micros(self.now_us().saturating_sub(stamp.at_us)) < WHEEE_HOLD_FRESH {
            WheeeHold::Held
        } else {
            WheeeHold::Decayed
        }
    }

    /// Ask for a drive-mode switch, by the code the caller and the loop agree on.
    ///
    /// The code rather than [`Mode`] itself, so this module does not have to know what modes
    /// exist — `main.rs` owns that mapping and the loop reads it back.
    pub fn request_mode_switch(&self, code: u8) {
        self.mode_switch.store(code, Ordering::Relaxed);
    }

    /// Store the newest external joint targets and enter External drive.
    ///
    /// `robot.setJoints`' sink. Stamps on the external clock — never the twist's — and sets
    /// the mode active, so a single `setJoints` both supplies a frame and takes the joints
    /// from the policy (the RFC's "setJoints implicitly enables External"). `gain` is the
    /// per-frame hold gain, `None` for the mode default.
    pub fn set_external(&self, targets: [f64; NUM_JOINTS], gain: Option<u16>) {
        self.external.store(Arc::new(Stamped {
            value: ExternalTargets { targets, gain },
            at_us: self.now_us(),
        }));
        self.external_active.store(true, Ordering::Relaxed);
    }

    /// Engage or leave External drive without supplying a frame — `robot.setMode "external"`
    /// on, and the policy-handback paths off. Engaging with no frame yet is safe: the newest
    /// external stamp stays old, so the external deadman holds the robot until targets arrive.
    pub fn set_external_mode(&self, on: bool) {
        self.external_active.store(on, Ordering::Relaxed);
    }

    /// Whether External drive is engaged — what `robot.mode` reports as `external`.
    pub fn external_engaged(&self) -> bool {
        self.external_active.load(Ordering::Relaxed)
    }

    /// Take a pending mode switch. Taken, so the sequence runs once per request.
    pub fn take_mode_switch(&self) -> Option<u8> {
        match self.mode_switch.swap(MODE_NONE, Ordering::Relaxed) {
            MODE_NONE => None,
            code => Some(code),
        }
    }

    /// Ask for the sit-then-power-off sequence.
    pub fn request_shutdown(&self) {
        self.shutdown.store(true, Ordering::Relaxed);
    }

    /// Take a pending shutdown request. Taken rather than read so the sequence starts once.
    pub fn take_shutdown(&self) -> bool {
        self.shutdown.swap(false, Ordering::Relaxed)
    }

    pub fn set_enabled(&self, on: bool) {
        self.enabled.store(on, Ordering::Relaxed);
    }

    /// The current enable state — what `robot.enable`'s `toggle` flips. The loop reads its
    /// copy through [`Self::snapshot`]; this is for the IPC side, which owns the toggle.
    pub fn enabled(&self) -> bool {
        self.enabled.load(Ordering::Relaxed)
    }

    /// Ask the loop to power the joints and stand up.
    pub fn request_init(&self) {
        self.power.store(POWER_INIT, Ordering::Relaxed);
    }

    /// Ask the loop to cut power to the joints.
    ///
    /// Also clears `enabled`: a robot that has been asked to go limp is not one the policy should
    /// keep driving, and leaving that flag set would have the next tick bring it straight back up.
    pub fn request_relax(&self) {
        self.enabled.store(false, Ordering::Relaxed);
        // A robot asked to go limp is not being driven by anyone — leave External too, so the
        // next frame does not silently resume it and torque comes back up under a stale target.
        self.external_active.store(false, Ordering::Relaxed);
        self.power.store(POWER_RELAX, Ordering::Relaxed);
    }

    /// Take the pending request, leaving none.
    ///
    /// Called once per tick by the loop. A later request replaces an unread earlier one, which is
    /// the right resolution: if someone asked to stand up and then to relax within 20 ms, the
    /// second is what they meant.
    /// Ask for the chorale to start listening (`true`) or stop (`false`), optionally pinning
    /// which piece this duck picks when it conducts.
    pub fn request_chorale(&self, active: bool, piece: Option<u8>) {
        // The pin first, so the loop that consumes the request on its next tick sees it. Piece
        // ids start at 1, so zero is "no pin".
        self.chorale_piece
            .store(piece.unwrap_or(0), Ordering::Relaxed);
        self.chorale.store(
            if active { THEREMIN_UP } else { THEREMIN_DOWN },
            Ordering::Relaxed,
        );
    }

    /// The pending chorale request, cleared by the read.
    pub fn take_chorale_request(&self) -> Option<(bool, Option<u8>)> {
        let piece = match self.chorale_piece.load(Ordering::Relaxed) {
            0 => None,
            id => Some(id),
        };
        match self.chorale.swap(THEREMIN_NONE, Ordering::Relaxed) {
            THEREMIN_UP => Some((true, piece)),
            THEREMIN_DOWN => Some((false, None)),
            _ => None,
        }
    }

    /// A beacon `btd` heard. Queued rather than latched: two ducks' beacons in one tick are two
    /// observations, and a beat lost to last-writer-wins is a beat the phase lock never sees.
    pub fn heard_chorale(&self, heard: duck_ipc_proto::ChoraleHeard) {
        let mut queue = self.chorale_heard.lock().expect("not poisoned");
        // Bounded: the control loop drains this every tick, so a backlog means the loop is not
        // running — in which case the newest beacons are the only ones worth having.
        if queue.len() >= 64 {
            queue.remove(0);
        }
        queue.push(heard);
    }

    /// Everything heard since the last tick.
    pub fn take_chorale_heard(&self) -> Vec<duck_ipc_proto::ChoraleHeard> {
        std::mem::take(&mut *self.chorale_heard.lock().expect("not poisoned"))
    }

    /// Ask for the theremin to be picked up (`true`) or put down (`false`).
    pub fn request_theremin(&self, active: bool) {
        self.theremin.store(
            if active { THEREMIN_UP } else { THEREMIN_DOWN },
            Ordering::Relaxed,
        );
    }

    /// The pending theremin request, cleared by the read.
    pub fn take_theremin_request(&self) -> Option<bool> {
        match self.theremin.swap(THEREMIN_NONE, Ordering::Relaxed) {
            THEREMIN_UP => Some(true),
            THEREMIN_DOWN => Some(false),
            _ => None,
        }
    }

    pub fn take_power_request(&self) -> Option<PowerRequest> {
        match self.power.swap(POWER_NONE, Ordering::Relaxed) {
            POWER_INIT => Some(PowerRequest::Init),
            POWER_RELAX => Some(PowerRequest::Relax),
            _ => None,
        }
    }

    pub fn snapshot(&self) -> Snapshot {
        let now = self.now_us();
        let twist = self.twist.load();
        let head = self.head.load();
        let pose = **self.pose.load();
        let external = if self.external_active.load(Ordering::Relaxed) {
            let e = self.external.load();
            Some(ExternalDrive {
                targets: e.value.targets,
                age: Duration::from_micros(now.saturating_sub(e.at_us)),
                gain: e.value.gain,
            })
        } else {
            None
        };
        Snapshot {
            command: Command {
                twist: twist.value,
                head: head.value,
                // The loop owns the smoothing that turns the pose intent into this block;
                // the raw target travels in `pose` below. Nominal zero is the trained
                // encoding, not a placeholder.
                body: BodyPose::default(),
            },
            twist_age: Duration::from_micros(now.saturating_sub(twist.at_us)),
            enabled: self.enabled.load(Ordering::Relaxed),
            pose,
            mouth: f64::from_bits(self.mouth.load(std::sync::atomic::Ordering::Relaxed)),
            external,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The hold's three states, which the ride's two different exits depend on. A `false`
    /// from a live client and a `true` that went stale must not read the same: one cuts the
    /// ride, the other plays it out.
    #[test]
    fn a_stale_hold_reads_as_decayed_not_released() {
        use duck_ipc_proto::{SoundParams, SoundTag};
        let intents = Intents::new();
        assert_eq!(
            intents.wheee_hold(),
            WheeeHold::Released,
            "nothing has ever held it"
        );

        intents.request_sound(SoundParams {
            tag: SoundTag::Wheee,
            hold: Some(true),
        });
        assert_eq!(intents.wheee_hold(), WheeeHold::Held);

        std::thread::sleep(WHEEE_HOLD_FRESH + Duration::from_millis(50));
        assert_eq!(
            intents.wheee_hold(),
            WheeeHold::Decayed,
            "a client that stopped re-notifying is not a client that released"
        );

        intents.request_sound(SoundParams {
            tag: SoundTag::Wheee,
            hold: Some(false),
        });
        assert_eq!(intents.wheee_hold(), WheeeHold::Released);
    }

    /// Before any client has spoken, the twist must already look stale. A robot that comes
    /// up believing it has a live driver would run its deadman timer down from `now`,
    /// giving a window where nothing is commanding it and nothing knows.
    #[test]
    fn the_twist_starts_stale() {
        let intents = Intents::new();
        let snap = intents.snapshot();
        assert_eq!(snap.command.twist, [0.0; 3]);
        assert!(
            snap.twist_age >= Duration::ZERO,
            "age must be measured from the epoch, not from first use"
        );
        assert!(!snap.enabled, "nothing drives until something asks");
    }

    /// Setting the head must not disturb the twist or its age, and vice versa. This is the
    /// whole reason they are separate slots: a combined one would need read-modify-write
    /// and two clients would clobber each other.
    #[test]
    fn the_slots_are_independent() {
        let intents = Intents::new();
        intents.set_twist([0.5, 0.0, 0.2]);
        std::thread::sleep(Duration::from_millis(5));
        intents.set_head([0.1, 0.2, 0.3, 0.4]);

        let snap = intents.snapshot();
        assert_eq!(
            snap.command.twist,
            [0.5, 0.0, 0.2],
            "head write clobbered twist"
        );
        assert_eq!(snap.command.head, [0.1, 0.2, 0.3, 0.4]);
        assert!(
            snap.twist_age >= Duration::from_millis(5),
            "a head write must not refresh the twist's deadman clock"
        );
    }

    /// The age is what the deadman reads, so a fresh write has to visibly reset it.
    #[test]
    fn writing_the_twist_refreshes_its_age() {
        let intents = Intents::new();
        std::thread::sleep(Duration::from_millis(10));
        let stale = intents.snapshot().twist_age;

        intents.set_twist([0.1, 0.0, 0.0]);
        let fresh = intents.snapshot().twist_age;

        assert!(
            fresh < stale,
            "expected {fresh:?} to be younger than {stale:?}"
        );
    }

    /// `stop` zeroes velocity without disabling the policy — the robot should stand, not
    /// go limp or stop being driven.
    #[test]
    fn stop_zeroes_the_twist_and_leaves_the_policy_enabled() {
        let intents = Intents::new();
        intents.set_enabled(true);
        intents.set_twist([1.0, 1.0, 1.0]);

        intents.stop();
        let snap = intents.snapshot();
        assert_eq!(snap.command.twist, [0.0; 3]);
        assert!(snap.enabled, "stop is not disable");
    }

    /// The body block has no intent behind it yet and must stay at the trained nominal.
    #[test]
    fn the_body_command_stays_nominal() {
        let intents = Intents::new();
        intents.set_twist([1.0, 0.0, 0.0]);
        assert_eq!(intents.snapshot().command.body, BodyPose::default());
    }

    // ── external drive (robot.setJoints) ─────────────────────────────────────

    /// Nothing is external until something asks: a fresh `Intents` has no external drive.
    #[test]
    fn there_is_no_external_drive_until_asked() {
        let intents = Intents::new();
        assert!(!intents.external_engaged());
        assert!(intents.snapshot().external.is_none());
    }

    /// `set_external` supplies the targets, records the gain and engages the mode in one step —
    /// the RFC's "setJoints implicitly enables External".
    #[test]
    fn setting_external_targets_engages_the_mode_and_snapshots_them() {
        let intents = Intents::new();
        let mut targets = [0.0; NUM_JOINTS];
        targets[0] = 0.3;
        targets[NUM_JOINTS - 1] = -0.2;

        intents.set_external(targets, Some(175));
        assert!(intents.external_engaged());

        let ext = intents.snapshot().external.expect("external is engaged");
        assert_eq!(ext.targets, targets);
        assert_eq!(ext.gain, Some(175));
    }

    /// The external clock is the deadman-critical invariant: streaming joints must not refresh
    /// the twist's deadman (a stalled controller cannot mask itself by looking like a live
    /// gamepad), and driving the twist must not refresh the external frame's age.
    #[test]
    fn the_external_clock_is_independent_of_the_twist_deadman() {
        let intents = Intents::new();

        // Age the twist, then write an external frame.
        intents.set_twist([0.5, 0.0, 0.0]);
        std::thread::sleep(Duration::from_millis(10));
        intents.set_external([0.0; NUM_JOINTS], None);

        let snap = intents.snapshot();
        assert!(
            snap.twist_age >= Duration::from_millis(10),
            "an external write must not refresh the twist's deadman clock"
        );
        let ext = snap.external.expect("engaged");
        assert!(
            ext.age < snap.twist_age,
            "the fresh external frame must be younger than the aged twist"
        );

        // And the reverse: a twist write does not touch the external frame's age.
        std::thread::sleep(Duration::from_millis(10));
        intents.set_twist([0.1, 0.0, 0.0]);
        let ext_after = intents.snapshot().external.expect("still engaged");
        assert!(
            ext_after.age >= Duration::from_millis(10),
            "a twist write must not refresh the external frame's clock"
        );
    }

    /// Leaving External mode removes it from the snapshot at once — the loop's cue to hand the
    /// joints back to the policy.
    #[test]
    fn leaving_external_mode_clears_the_snapshot() {
        let intents = Intents::new();
        intents.set_external([0.0; NUM_JOINTS], None);
        assert!(intents.snapshot().external.is_some());

        intents.set_external_mode(false);
        assert!(!intents.external_engaged());
        assert!(intents.snapshot().external.is_none());
    }

    /// `robot.relax` leaves External: a robot told to go limp is not being driven, and the mode
    /// must not silently resume torque under a stale target on the next frame.
    #[test]
    fn relax_leaves_external() {
        let intents = Intents::new();
        intents.set_external([0.0; NUM_JOINTS], Some(200));
        assert!(intents.external_engaged());

        intents.request_relax();
        assert!(!intents.external_engaged(), "relax must leave External");
        assert!(intents.snapshot().external.is_none());
    }
}
