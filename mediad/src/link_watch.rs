//! A peer that was driving the robot and went quiet: the link is gone, so the robot sits down.
//!
//! The deadman in `robotd` already stops the walking half a second after the twists stop
//! arriving — but it leaves the robot standing, policy on, in the middle of a room, and the client
//! that would have sat it down (`padd` does, when its pad drops out) cannot reach the robot any
//! more. Only this end of the link can tell the link is gone, so this end acts: the robot asks
//! where its driver went (the "inquire" sound) and sits down if it is standing.
//!
//! **Silence is not the same as gone**, and the rule is written to tell them apart:
//!
//! - Only a session that has sent `robot.move` is watched. A console open to look at the camera
//!   is never anybody's driver.
//! - The console sends twists at 10 Hz while a key is held, then one stop, then nothing: a
//!   person letting go, not a lost link. So silence counts only when the last twist asked for
//!   motion, or when the peer had been repeating a standstill at a steady cadence — `padd`
//!   heartbeats its zeros every 100 ms, and a peer that does that does not fall silent by choice.
//! - A session that closes counts the same way, so ending a `duckctl webrtc-drive` sits the robot
//!   down as switching the pad off does, while closing an idle console tab does not.

use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use duck_ipc_proto as proto;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::UnixStream;

/// Quiet this long, from a peer that should not be quiet, is a lost link. Twice the console's
/// twist period and ten of `padd`'s heartbeats.
pub const LINK_LOST: Duration = Duration::from_secs(1);

/// How many standstill twists in a row make a peer a heartbeat rather than a person who let go.
/// The console sends exactly one.
const STEADY_ZEROES: u32 = 3;

/// How long to keep asking for the sit while the robot refuses it mid-move.
const SIT_TRIES: Duration = Duration::from_secs(5);

/// One session's view of its peer as a driver.
#[derive(Debug)]
pub struct LinkWatch {
    /// The session has sent `robot.move` at all.
    drove: bool,
    last_line: Instant,
    /// The last twist asked for motion.
    moving: bool,
    /// Standstill twists in a row since the last motion.
    zeroes: u32,
    /// Already acted on this silence; cleared by the next line.
    fired: bool,
}

impl LinkWatch {
    pub fn new(now: Instant) -> Self {
        Self {
            drove: false,
            last_line: now,
            moving: false,
            zeroes: 0,
            fired: false,
        }
    }

    /// A line arrived from the peer.
    pub fn line(&mut self, line: &str, now: Instant) {
        self.last_line = now;
        self.fired = false;
        let Some(twist) = twist(line) else { return };
        self.drove = true;
        if twist.iter().any(|v| *v != 0.0) {
            self.moving = true;
            self.zeroes = 0;
        } else {
            self.moving = false;
            self.zeroes = self.zeroes.saturating_add(1);
        }
    }

    /// Whether the robot should sit now, because the peer has gone quiet. True once per silence.
    pub fn lost(&mut self, now: Instant) -> bool {
        if self.fired || now.duration_since(self.last_line) < LINK_LOST || !self.was_driving() {
            return false;
        }
        self.fired = true;
        true
    }

    /// Whether the robot should sit because the session ended under a driver.
    pub fn closed(&mut self) -> bool {
        if self.fired || !self.was_driving() {
            return false;
        }
        self.fired = true;
        true
    }

    fn was_driving(&self) -> bool {
        self.drove && (self.moving || self.zeroes >= STEADY_ZEROES)
    }
}

/// The twist in a `robot.move` line, or `None` for any other line.
fn twist(line: &str) -> Option<[f64; 3]> {
    let value: serde_json::Value = serde_json::from_str(line).ok()?;
    if value.get("method")?.as_str()? != proto::method::ROBOT_MOVE {
        return None;
    }
    let params = value.get("params");
    let axis = |name: &str| {
        params
            .and_then(|p| p.get(name))
            .and_then(|v| v.as_f64())
            .unwrap_or(0.0)
    };
    Some([axis("vx"), axis("vy"), axis("vyaw")])
}

/// The driver is gone: ask where it went, and sit down if standing. Best effort throughout — a
/// robot that cannot be reached or will not sit is logged, never an error for the session.
pub async fn sit_down(robot: PathBuf) {
    tracing::warn!("the peer driving the robot went quiet — sitting it down");
    if let Err(e) = sit_down_at(&robot).await {
        tracing::warn!(error = %e, "could not sit the robot down after the link was lost");
    }
}

async fn sit_down_at(robot: &Path) -> std::io::Result<()> {
    let stream = UnixStream::connect(robot).await?;
    let (read, mut write) = stream.into_split();
    let mut lines = BufReader::new(read).lines();
    let mut next_id = 1u64;

    let mut send = async |call: &proto::Call, id: Option<u64>| -> std::io::Result<()> {
        let request = match id {
            Some(id) => proto::Request::call(proto::Id::Number(id), call),
            None => proto::Request::notify(call),
        };
        let mut line = serde_json::to_vec(&request).map_err(std::io::Error::other)?;
        line.push(b'\n');
        write.write_all(&line).await
    };

    send(
        &proto::Call::RobotSound(proto::SoundParams {
            tag: proto::SoundTag::Inquire,
            hold: None,
        }),
        None,
    )
    .await?;

    let deadline = Instant::now() + SIT_TRIES;
    loop {
        send(&proto::Call::RobotPolicies, Some(next_id)).await?;
        next_id += 1;
        let sitting = answer(&mut lines)
            .await?
            .and_then(|r| r.result_as::<proto::PoliciesResult>().ok())
            .and_then(|p| p.sitting);
        if sitting == Some(true) {
            tracing::info!("the robot is already sitting");
            return Ok(());
        }

        let sit = proto::Call::RobotDo(proto::DoParams {
            skill: "sit_toggle".to_owned(),
        });
        send(&sit, Some(next_id)).await?;
        next_id += 1;
        let result = answer(&mut lines)
            .await?
            .and_then(|r| r.result_as::<proto::IntentResult>().ok());
        match result {
            Some(result) if result.accepted => {
                tracing::warn!("sitting the robot down");
                return Ok(());
            }
            // Not driving: nothing will sit it, and a robot with the policy off is standing still
            // at home already.
            Some(result)
                if result
                    .reason
                    .as_deref()
                    .is_some_and(|r| r.contains("not driving")) =>
            {
                return Ok(());
            }
            _ if Instant::now() >= deadline => {
                tracing::warn!("the robot would not sit after {SIT_TRIES:?} of asking");
                return Ok(());
            }
            // Mid-move: ask again shortly.
            _ => tokio::time::sleep(Duration::from_millis(200)).await,
        }
    }
}

/// The next line from `robotd`, as a response, within a couple of seconds.
async fn answer(
    lines: &mut tokio::io::Lines<BufReader<tokio::net::unix::OwnedReadHalf>>,
) -> std::io::Result<Option<proto::Response>> {
    match tokio::time::timeout(Duration::from_secs(2), lines.next_line()).await {
        Ok(Ok(Some(line))) => Ok(serde_json::from_str(&line).ok()),
        Ok(Ok(None)) => Err(std::io::Error::other("robotd closed the connection")),
        Ok(Err(e)) => Err(e),
        Err(_) => Err(std::io::Error::other("robotd did not answer")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn twist_line(vx: f64) -> String {
        format!(
            r#"{{"jsonrpc":"2.0","method":"robot.move","params":{{"vx":{vx},"vy":0,"vyaw":0}}}}"#
        )
    }

    /// `padd`, idle, then the link drops: it was heartbeating a standstill, so its silence is a
    /// lost link — once.
    #[test]
    fn a_heartbeating_driver_that_goes_quiet_is_lost() {
        let t0 = Instant::now();
        let mut watch = LinkWatch::new(t0);
        for i in 0..10 {
            watch.line(&twist_line(0.0), t0 + Duration::from_millis(100 * i));
        }
        let last = t0 + Duration::from_millis(900);
        assert!(!watch.lost(last + Duration::from_millis(900)), "not yet");
        assert!(watch.lost(last + LINK_LOST));
        assert!(!watch.lost(last + LINK_LOST * 3), "once per silence");

        // Back, and lost again: it fires again.
        watch.line(&twist_line(0.0), last + LINK_LOST * 4);
        let back = last + LINK_LOST * 4;
        for i in 1..4 {
            watch.line(&twist_line(0.0), back + Duration::from_millis(100 * i));
        }
        assert!(watch.lost(back + LINK_LOST * 2));
    }

    /// Driving at the moment the link drops — the console with a key held — is lost.
    #[test]
    fn a_driver_mid_motion_that_goes_quiet_is_lost() {
        let t0 = Instant::now();
        let mut watch = LinkWatch::new(t0);
        watch.line(&twist_line(0.2), t0);
        assert!(watch.lost(t0 + LINK_LOST));
    }

    /// The console's person letting go: one stop, then nothing. Not a lost link, and closing the
    /// tab afterwards does not sit the robot either.
    #[test]
    fn one_stop_then_silence_is_a_person_letting_go() {
        let t0 = Instant::now();
        let mut watch = LinkWatch::new(t0);
        for i in 0..10 {
            watch.line(&twist_line(0.2), t0 + Duration::from_millis(100 * i));
        }
        let stop = t0 + Duration::from_secs(1);
        watch.line(&twist_line(0.0), stop);
        assert!(!watch.lost(stop + LINK_LOST * 5));
        assert!(!watch.closed());
    }

    /// A session that never drove — the camera, the health panel — is never anybody's driver.
    #[test]
    fn a_session_that_never_drove_is_never_lost() {
        let t0 = Instant::now();
        let mut watch = LinkWatch::new(t0);
        watch.line(r#"{"jsonrpc":"2.0","id":1,"method":"robot.health"}"#, t0);
        assert!(!watch.lost(t0 + LINK_LOST * 10));
        assert!(!watch.closed());
    }

    /// Ending a heartbeating session sits the robot, once.
    #[test]
    fn closing_a_heartbeating_session_counts() {
        let t0 = Instant::now();
        let mut watch = LinkWatch::new(t0);
        for i in 0..5 {
            watch.line(&twist_line(0.0), t0 + Duration::from_millis(100 * i));
        }
        assert!(watch.closed());
        assert!(!watch.closed());
    }

    #[test]
    fn only_robot_move_is_a_twist() {
        assert_eq!(twist(&twist_line(0.5)), Some([0.5, 0.0, 0.0]));
        assert_eq!(
            twist(r#"{"jsonrpc":"2.0","method":"robot.head","params":{}}"#),
            None
        );
        assert_eq!(twist("garbage"), None);
    }
}
