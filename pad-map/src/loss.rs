//! A pad that was driving and went away: the robot asks where it went, and after a second sits
//! down, so it is not left standing in the middle of a room. The pad coming back is greeted.
//!
//! Shared by `padd` (the Bluetooth pad dropping out) and `netpadd` (the UDP client going quiet),
//! so the robot reacts the same way whichever pad it lost. The state machine and the reading of
//! the robot's answers live here; the two requests themselves are each daemon's, because one
//! talks to robotd synchronously and the other from a tokio task.

use std::time::{Duration, Instant};

use duck_ipc_proto as proto;

/// The pad gone this long sits the robot down. Past the deadman, which has already stopped any
/// walking at half a second, and short enough that a robot left standing in the middle of a room
/// is sitting before anybody wonders why it froze.
pub const PAD_LOST_SIT: Duration = Duration::from_secs(1);

/// How long to keep asking for that sit when the robot refuses it — mid-kick, mid-pick, mid-rise.
/// Each refused move ends within a few seconds; past this, something else is going on and the
/// journal says so rather than the pad asking forever.
pub const PAD_LOST_SIT_TRIES: Duration = Duration::from_secs(5);

/// How often to ask, while a sit is due. Fast enough that the one-second promise holds.
pub const PAD_LOST_POLL: Duration = Duration::from_millis(100);

/// What to do about a pad that went away, tick by tick.
///
/// Only a pad that was *driving* can be lost: a daemon that starts with no pad has nothing to
/// react to, and must not sit a robot somebody else is driving.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum PadLoss {
    /// The pad is here, or never was.
    #[default]
    Present,
    /// Gone since then; the sit is not due yet, or is being asked for.
    Lost { since: Instant },
    /// Gone, and the sit has been dealt with — done, or given up on.
    Settled,
}

/// What [`PadLoss::tick`] asks the daemon to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PadLossAction {
    Nothing,
    /// Try to sit the robot now — see [`sit_settled`] — and [`PadLoss::settle`] once it is, or
    /// ask again next poll.
    Sit,
    /// The tries ran out.
    GiveUp,
}

impl PadLoss {
    pub fn lost(&mut self, now: Instant) {
        *self = Self::Lost { since: now };
    }

    /// The pad is back. True when it had been lost, for the greeting.
    pub fn found(&mut self) -> bool {
        let was_lost = *self != Self::Present;
        *self = Self::Present;
        was_lost
    }

    pub fn tick(&self, now: Instant) -> PadLossAction {
        match *self {
            Self::Lost { since } => {
                let gone = now.duration_since(since);
                if gone >= PAD_LOST_SIT + PAD_LOST_SIT_TRIES {
                    PadLossAction::GiveUp
                } else if gone >= PAD_LOST_SIT {
                    PadLossAction::Sit
                } else {
                    PadLossAction::Nothing
                }
            }
            Self::Present | Self::Settled => PadLossAction::Nothing,
        }
    }

    /// The robot is sitting, or there was nothing to sit: stop asking.
    pub fn settle(&mut self) {
        if matches!(self, Self::Lost { .. }) {
            *self = Self::Settled;
        }
    }

    /// Whether the daemon should poll quickly, because a sit is due or about to be.
    pub fn pending(&self) -> bool {
        matches!(self, Self::Lost { .. })
    }
}

/// One of the robot's sounds, as a notification: a cue, not something worth waiting on.
pub fn sound(tag: proto::SoundTag) -> proto::Call {
    proto::Call::RobotSound(proto::SoundParams { tag, hold: None })
}

/// The sit itself. Asked only after [`already_sitting`] says no: `sit_toggle` stands a seated
/// robot up, and a pad dropping out next to a sitting robot must not do that.
pub fn sit_call() -> proto::Call {
    proto::Call::RobotDo(proto::DoParams {
        skill: "sit_toggle".to_owned(),
    })
}

/// Whether `robot.policies` says the robot is sitting already, so there is nothing to do.
pub fn already_sitting(policies: Option<&proto::Response>) -> bool {
    let sitting = policies
        .and_then(|answer| answer.result_as::<proto::PoliciesResult>().ok())
        .and_then(|policies| policies.sitting);
    if sitting == Some(true) {
        tracing::info!("pad gone: the robot is already sitting");
        return true;
    }
    false
}

/// Whether the answer to [`sit_call`] leaves nothing more to do: it is sitting down, or the policy
/// is not driving and so has nothing to sit with (the robot is then standing still at home, stiff,
/// which is where a lost pad should leave it anyway). False when the robot refused for a reason
/// that passes — mid-move, typically — and the sit should be asked for again.
pub fn sit_settled(answer: Option<&proto::Response>) -> bool {
    match answer.and_then(|answer| answer.result_as::<proto::IntentResult>().ok()) {
        Some(result) if result.accepted => {
            tracing::warn!("pad gone: sitting the robot down");
            true
        }
        Some(result)
            if result
                .reason
                .as_deref()
                .is_some_and(|r| r.contains("not driving")) =>
        {
            true
        }
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A pad lost while driving: nothing for a second, then sit — asked again until the robot
    /// takes it, for a few seconds, then given up on. Settled, it asks nothing more.
    #[test]
    fn a_lost_pad_sits_the_robot_after_a_second() {
        let t0 = Instant::now();
        let at = |ms: u64| t0 + Duration::from_millis(ms);
        let mut loss = PadLoss::default();
        assert_eq!(loss.tick(at(5_000)), PadLossAction::Nothing, "never lost");

        loss.lost(t0);
        assert!(loss.pending());
        assert_eq!(loss.tick(at(900)), PadLossAction::Nothing);
        assert_eq!(loss.tick(at(1_000)), PadLossAction::Sit);
        assert_eq!(
            loss.tick(at(3_000)),
            PadLossAction::Sit,
            "refused: ask again"
        );
        assert_eq!(loss.tick(at(6_000)), PadLossAction::GiveUp);

        loss.settle();
        assert!(!loss.pending());
        assert_eq!(loss.tick(at(9_000)), PadLossAction::Nothing);
    }

    /// The pad coming back is greeted only when it had been lost, and stops any sit still due.
    #[test]
    fn a_pad_coming_back_is_greeted_once() {
        let t0 = Instant::now();
        let mut loss = PadLoss::default();
        assert!(!loss.found(), "a pad that was never lost is not a return");

        loss.lost(t0);
        assert!(loss.found(), "back before the sit");
        assert_eq!(
            loss.tick(t0 + Duration::from_secs(2)),
            PadLossAction::Nothing,
            "and no sit after it is back"
        );

        loss.lost(t0);
        loss.settle();
        assert!(loss.found(), "back after the sit");
        assert!(!loss.found(), "once");
    }

    fn answer(result: serde_json::Value) -> proto::Response {
        serde_json::from_value(serde_json::json!({"jsonrpc": "2.0", "id": 1, "result": result}))
            .expect("a response")
    }

    /// The robot's answers, read the way the sit needs them: sitting already ends it, an accepted
    /// sit or a policy that is not driving ends it, anything else is asked again.
    #[test]
    fn the_answers_decide_whether_the_sit_is_done() {
        assert!(!already_sitting(None), "no answer is not sitting");
        assert!(!sit_settled(None), "no answer: ask again");
        assert!(sit_settled(Some(&answer(
            serde_json::json!({"accepted": true})
        ))));
        assert!(sit_settled(Some(&answer(
            serde_json::json!({"accepted": false, "reason": "the policy is not driving"})
        ))));
        assert!(!sit_settled(Some(&answer(
            serde_json::json!({"accepted": false, "reason": "a skill is running"})
        ))));
    }
}
