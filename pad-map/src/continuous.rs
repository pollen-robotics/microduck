//! The continuous intents — velocity, head, pose — and when they need saying again.

use std::time::{Duration, Instant};

use duck_ipc_proto as proto;

/// How long an unchanged frame may go unsent while the robot is being asked to stand still.
///
/// The sticks are *polled*, so an untouched pad re-sent the same three zeros fifty times a
/// second — a `serde_json` encode, a `write_all`, a `flush`, and a `serde_json` parse on
/// `robotd`'s side, a hundred messages a second in the modes that send two, to say nothing
/// changed. Ten a second says it as well.
///
/// **Nothing about the robot's safety rests on this number**, which is why it can be picked
/// for legibility rather than argued against `[safety] deadman_ms` — a value this daemon
/// cannot read and does not know. A frame is only ever held back when it is byte-identical
/// to the last one sent *and* asks for no motion ([`Continuous::may_hold`]); a stick that is
/// doing anything goes out on every tick as it always did. What the heartbeat buys is the
/// report: without it `robotd`'s twist would age past the deadman while a pad sat connected
/// and idle, and `robot.state` would carry `limited_by: ["deadman"]` for a robot that is
/// stationary because it was asked to be.
pub const HEARTBEAT: Duration = Duration::from_millis(100);

/// The continuous intents, encoded once a tick and sent when they say something new.
#[derive(Default)]
pub struct Continuous {
    /// This tick's frame. Kept across ticks so the encode reuses its buffer.
    line: Vec<u8>,
    /// The bytes last put on the socket, to compare this tick's against.
    last: Vec<u8>,
    /// When that was. `None` until the first send, which therefore always happens.
    at: Option<Instant>,
}

impl Continuous {
    /// This tick's intents as the bytes to put on robotd's socket, or `None` when they are the
    /// ones already there and may be held back (see [`Continuous::may_hold`]).
    ///
    /// Returning `Some` records the frame as sent. The caller writes it or dies trying: both
    /// daemons exit on a failed write, so there is no "returned but not sent" state to undo.
    /// One buffer for the whole frame: the calls describe a single instant, and a peer that read
    /// half of one would act on a head pose without the velocity that came with it.
    pub fn next(&mut self, calls: &[proto::Call], now: Instant) -> Option<&[u8]> {
        self.line.clear();
        for call in calls {
            // Encoding plain data into a Vec cannot fail.
            serde_json::to_writer(&mut self.line, &proto::Request::notify(call))
                .expect("a Call serialises");
            self.line.push(b'\n');
        }
        if self.line == self.last && self.may_hold(calls, now) {
            return None;
        }
        // Swapped rather than cloned: the displaced buffer is next tick's scratch.
        std::mem::swap(&mut self.last, &mut self.line);
        self.at = Some(now);
        Some(&self.last)
    }

    /// Whether an unchanged frame may be left unsent this tick.
    ///
    /// Only while it asks for no velocity. The deadman zeroes the twist and nothing else, so
    /// on a frame that already commands zero, letting it fire changes nothing about what the
    /// robot does — and on a frame that commands motion it would stop a robot whose stick is
    /// still held. That is the whole of the argument, and it holds whatever `deadman_ms` is
    /// set to.
    ///
    /// A held stick therefore keeps sending at the full rate. That is the case where the
    /// robot is walking and the daemon has something to say; this is about the one where it
    /// is not and does not.
    fn may_hold(&self, calls: &[proto::Call], now: Instant) -> bool {
        let asks_for_motion = calls.iter().any(|call| match call {
            proto::Call::RobotMove(p) => p.vx != 0.0 || p.vy != 0.0 || p.vyaw != 0.0,
            _ => false,
        });
        !asks_for_motion && self.at.is_some_and(|at| now.duration_since(at) < HEARTBEAT)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// What went out this tick, as text, or `None` for nothing at all.
    fn sent(continuous: &mut Continuous, calls: &[proto::Call], now: Instant) -> Option<String> {
        continuous
            .next(calls, now)
            .map(|bytes| String::from_utf8_lossy(bytes).into_owned())
    }

    fn moving() -> Vec<proto::Call> {
        vec![proto::Call::RobotMove(proto::MoveParams {
            vx: 0.2,
            vy: 0.0,
            vyaw: 0.0,
        })]
    }

    fn still() -> Vec<proto::Call> {
        vec![proto::Call::RobotMove(proto::MoveParams::default())]
    }

    /// The sticks are polled, so an untouched pad produces the same frame fifty times a
    /// second. Sending it fifty times is what this stops.
    #[test]
    fn an_unchanged_stationary_frame_is_not_resent() {
        let mut continuous = Continuous::default();
        let at = Instant::now();
        let first = sent(&mut continuous, &still(), at);
        assert!(
            first
                .expect("the first frame must go out")
                .contains("robot.move")
        );
        for tick in 1..5 {
            let now = at + Duration::from_millis(20 * tick);
            assert!(
                continuous.next(&still(), now).is_none(),
                "an idle pad must say nothing"
            );
        }
    }

    /// **The safety property, and the reason the heartbeat needs no argument about
    /// `deadman_ms`.** A stick that is asking the robot to walk is re-sent every tick
    /// whatever the clock says, because a deadman that fires on a held stick stops a robot
    /// somebody is driving.
    #[test]
    fn a_frame_that_asks_for_motion_is_always_sent() {
        let mut continuous = Continuous::default();
        let at = Instant::now();

        assert!(continuous.next(&moving(), at).is_some(), "first");

        // The same bytes, one tick later — well inside the heartbeat.
        let second = sent(&mut continuous, &moving(), at + Duration::from_millis(20));
        assert!(
            second.is_some_and(|s| s.contains("robot.move")),
            "a held stick must keep being sent"
        );
    }

    /// The heartbeat is what keeps `robotd` from reporting a deadman on a robot that is
    /// standing still because it was asked to.
    #[test]
    fn a_stationary_frame_goes_out_again_on_the_heartbeat() {
        let mut continuous = Continuous::default();
        let at = Instant::now();

        assert!(continuous.next(&still(), at).is_some(), "first");

        assert!(
            continuous
                .next(&still(), at + HEARTBEAT - Duration::from_millis(1))
                .is_none(),
            "not due yet"
        );

        let due = sent(&mut continuous, &still(), at + HEARTBEAT);
        assert!(due.is_some_and(|s| s.contains("robot.move")), "due");
    }

    /// A stick that moves is heard on the tick it moved, not on the next heartbeat.
    #[test]
    fn a_changed_frame_is_sent_at_once() {
        let mut continuous = Continuous::default();
        let at = Instant::now();

        assert!(continuous.next(&still(), at).is_some(), "first");

        let changed = sent(&mut continuous, &moving(), at + Duration::from_millis(20));
        assert!(
            changed.is_some_and(|s| s.contains("robot.move")),
            "a stick that moved must not wait for a heartbeat"
        );
    }

    /// Head mode's two intents describe one instant and go out in one write. Split across
    /// two, a reader could act on a head pose without the velocity that came with it.
    ///
    /// One buffer is one write: the caller puts exactly what `next` returned on the socket.
    #[test]
    fn a_two_call_frame_is_one_write_and_two_lines() {
        let mut continuous = Continuous::default();
        let calls = vec![
            proto::Call::RobotMove(proto::MoveParams::default()),
            proto::Call::RobotHead(proto::HeadParams {
                neck_pitch: 0.1,
                head_pitch: 0.0,
                head_yaw: 0.0,
                head_roll: 0.0,
            }),
        ];

        let sent = sent(&mut continuous, &calls, Instant::now()).expect("sent");
        assert_eq!(
            sent.matches('\n').count(),
            2,
            "two intents, two lines: {sent:?}"
        );
        let lines: Vec<&str> = sent.lines().collect();
        assert_eq!(lines.len(), 2, "two intents, two lines: {sent:?}");
        assert!(lines[0].contains("robot.move"));
        assert!(lines[1].contains("robot.head"));
        assert!(sent.ends_with('\n'), "every line must be terminated");
    }

    /// A head frame carries a zero velocity, so an untouched pad in head mode holds too —
    /// which is the mode that was sending a hundred messages a second.
    #[test]
    fn an_untouched_pad_in_head_mode_holds_both_intents() {
        let mut continuous = Continuous::default();
        let at = Instant::now();
        let calls = vec![
            proto::Call::RobotMove(proto::MoveParams::default()),
            proto::Call::RobotHead(proto::HeadParams::default()),
        ];

        assert!(continuous.next(&calls, at).is_some(), "first");
        assert!(
            continuous
                .next(&calls, at + Duration::from_millis(20))
                .is_none()
        );
    }
}
