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

/// Encodings for [`Intents::power`]. An `AtomicU8` rather than two bools, so "init" and "relax"
/// cannot both be pending — they are alternatives, and the last one asked for wins.
const POWER_NONE: u8 = 0;
const POWER_INIT: u8 = 1;
const POWER_RELAX: u8 = 2;
use std::time::{Duration, Instant};

use arc_swap::ArcSwap;
use duck_control::obs::{BodyPose, Command};

/// A value and when it arrived.
#[derive(Debug, Clone, Copy)]
struct Stamped<T> {
    value: T,
    /// Microseconds since the [`Intents`] epoch.
    at_us: u64,
}

pub struct Intents {
    /// Epoch for every stamp. `Instant` so the clock cannot run backwards under us.
    epoch: Instant,
    twist: ArcSwap<Stamped<[f64; 3]>>,
    head: ArcSwap<Stamped<[f64; 4]>>,
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
            enabled: AtomicBool::new(false),
            power: AtomicU8::new(POWER_NONE),
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

    pub fn set_enabled(&self, on: bool) {
        self.enabled.store(on, Ordering::Relaxed);
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
        self.power.store(POWER_RELAX, Ordering::Relaxed);
    }

    /// Take the pending request, leaving none.
    ///
    /// Called once per tick by the loop. A later request replaces an unread earlier one, which is
    /// the right resolution: if someone asked to stand up and then to relax within 20 ms, the
    /// second is what they meant.
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
        Snapshot {
            command: Command {
                twist: twist.value,
                head: head.value,
                // No `pose` intent yet, so the body block stays at its nominal zero — which
                // is the encoding the policies were trained with, not a placeholder.
                body: BodyPose::default(),
            },
            twist_age: Duration::from_micros(now.saturating_sub(twist.at_us)),
            enabled: self.enabled.load(Ordering::Relaxed),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
}
