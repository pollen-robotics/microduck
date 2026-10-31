//! `netpadd` — a gamepad over UDP, as an intent client.
//!
//! A program on the LAN sends the pad's whole state, 28 bytes a datagram
//! (`docs/robot/udp-pad.md` owns the format), and this drives the robot with exactly `padd`'s
//! mapping — `pad-map`, shared. It is `padd` with a different pad: no privileged access, the same
//! socket, the same deadman semantics. A client that goes quiet is a pad that went away: nothing
//! more is sent, and robotd's deadman holds the robot.
//!
//! Only one of the two runs. `[netpad] enabled` decides, and both units ask it in
//! `ExecCondition=` (`--should-run`) — not `Conflicts=`, which an update's restart of every
//! shipped unit would turn into "Bluetooth wins" (`restart-order.md` §1).
//!
//! One tokio task on a current-thread runtime owns the socket, robotd's connection and the
//! receiver: no locks, no work-stealing jitter. `receiver.rs` decides; this file only waits and
//! writes. The one exception is the robot's health for the driving client's report, gathered by
//! `health.rs` on connections of its own so that a slow `robotd` answer never stalls the pad;
//! this loop only reads the newest snapshot.

mod health;
mod receiver;
mod robotd;

use std::path::PathBuf;
use std::time::{Duration, Instant};

use clap::Parser;
use duck_ipc_proto as proto;
use pad_map::{Config, Continuous, HEARTBEAT, Mapper, Out, PadLoss, PadLossAction};
use receiver::{Accept, Receiver, Step, Timing};

#[derive(Parser, Debug)]
#[command(
    name = "netpadd",
    about = "Drive the robot from gamepad state sent over UDP",
    version
)]
struct Args {
    /// `robotd`'s socket.
    #[arg(long, default_value = "/run/robotd.sock")]
    socket: PathBuf,
    /// Where `[netpad]` and the button bindings are read from.
    #[arg(long, default_value = robotd_params::DEFAULT_PATH)]
    config: PathBuf,
    /// Exit 0 if `[netpad] enabled` says this daemon should run, 1 if not — for `ExecCondition=`.
    #[arg(long)]
    should_run: bool,
    /// Over `pad_axes.deadzone`, as in `padd`.
    #[arg(long, value_parser = pad_map::parse_deadzone)]
    deadzone: Option<f64>,
    /// Over every `[pad_head]` limit, as in `padd`.
    #[arg(long, value_parser = pad_map::parse_max_head)]
    max_head: Option<f64>,
}

#[cfg(test)]
impl Args {
    fn for_test(socket: PathBuf, config: PathBuf) -> Self {
        Self {
            socket,
            config,
            should_run: false,
            deadzone: None,
            max_head: None,
        }
    }
}

/// Malformed and late datagrams are each logged with their count at most this often: a LAN can
/// hold a noisy neighbour, a backlog is hundreds at once, and one line per packet would bury the
/// journal.
const DROPPED_LOG: Duration = Duration::from_secs(10);

fn main() -> std::process::ExitCode {
    let args = Args::parse();
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_writer(std::io::stderr)
        .init();

    if args.should_run {
        return if pad_map::udp_selected(&args.config) {
            std::process::ExitCode::SUCCESS
        } else {
            std::process::ExitCode::from(1)
        };
    }

    duck_ipc_proto::log_startup_identity!("netpadd");

    // Strict here, unlike the bindings: a `[netpad]` this daemon cannot read is one it cannot
    // honour, and running on a guessed port would be worse than saying so.
    let params = match robotd_params::Params::load(&args.config, false) {
        Ok(p) => p,
        Err(e) => {
            tracing::error!(error = %e, "cannot read the config");
            return std::process::ExitCode::FAILURE;
        }
    };
    let runtime = match tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
    {
        Ok(r) => r,
        Err(e) => {
            tracing::error!(error = %e, "no runtime");
            return std::process::ExitCode::FAILURE;
        }
    };
    runtime.block_on(serve(args, params, None))
}

async fn serve(
    args: Args,
    params: robotd_params::Params,
    bound: Option<tokio::sync::oneshot::Sender<u16>>,
) -> std::process::ExitCode {
    let np = params.netpad.clone();
    let min_interval = Duration::from_secs_f64(1.0 / f64::from(np.max_hz));

    let mut robot = match robotd::Robotd::connect(&args.socket, min_interval).await {
        Ok(r) => r,
        Err(e) => {
            tracing::error!(error = %e, socket = %args.socket.display(), "cannot reach robotd");
            return std::process::ExitCode::FAILURE;
        }
    };
    let socket = match tokio::net::UdpSocket::bind(("0.0.0.0", np.port)).await {
        Ok(s) => s,
        Err(e) => {
            tracing::error!(error = %e, port = np.port, "cannot bind the pad port");
            return std::process::ExitCode::FAILURE;
        }
    };
    if let Err(e) = socket2::SockRef::from(&socket).set_tos_v4(pad_map::wire::TOS) {
        tracing::warn!(error = %e, "cannot mark the answers expedited; sent unmarked");
    }
    let port = socket.local_addr().map(|a| a.port()).unwrap_or(np.port);
    if let Some(tx) = bound {
        let _ = tx.send(port);
    }

    let roller = match robot.ask_roller().await {
        Ok(r) => r.unwrap_or(false),
        Err(e) => {
            tracing::error!(error = %e, "mode request failed");
            return std::process::ExitCode::FAILURE;
        }
    };
    let overrides = pad_map::Overrides {
        deadzone: args.deadzone,
        max_head: args.max_head,
    };
    let mut config = Config::defaults(roller, &overrides);
    config.reload(&args.config, &overrides);
    tracing::warn!(
        port,
        max_hz = np.max_hz,
        stale_ms = np.stale_ms,
        timeout_ms = np.timeout_ms,
        roller,
        "listening for a pad over UDP"
    );

    let mut rx = Receiver::new(Timing {
        min_interval,
        stale: Duration::from_millis(np.stale_ms),
        timeout: Duration::from_millis(np.timeout_ms),
        fallback: HEARTBEAT,
    });
    let (mut mapper, mut continuous) = (Mapper::new(), Continuous::default());
    let (mut out, mut frame) = (Vec::new(), Vec::new());
    let mut buf = [0u8; 512];
    let mut housekeeping = Instant::now() + pad_map::BINDINGS_POLL;
    // Both layers: an edit lands in the user layer, which the base's mtime never sees.
    let mut config_at = robotd_params::config_stamp(&args.config);
    let (mut malformed, mut malformed_logged) = (0u64, None::<Instant>);
    let (mut late, mut late_logged) = (0u64, None::<Instant>);
    let mut lateness = Lateness::default();
    let mut answers = Answers::default();
    let mut reserved_logged = false;
    // A client that was driving and went quiet: the robot sits after a second — `pad_map::loss`,
    // the same as `padd` does for a Bluetooth pad.
    let mut pad_loss = PadLoss::default();
    // The robot's health, for the driving client's report — read here, gathered elsewhere.
    let robot_health = health::spawn(&args.socket);
    let mut report_at = Instant::now() + pad_map::wire::REPORT_EVERY;
    let mut refused = None::<(&'static str, String, Instant)>;

    loop {
        let mut wake = rx
            .deadline()
            .map_or(housekeeping, |d| d.min(housekeeping))
            .min(report_at);
        if pad_loss.pending() {
            wake = wake.min(Instant::now() + pad_map::loss::PAD_LOST_POLL);
        }
        tokio::select! {
            biased;
            got = socket.recv_from(&mut buf) => {
                // Handle this one, then everything else already waiting, in arrival order: edges
                // need every datagram, and the receiver keeps only the newest state.
                let mut next = got.ok();
                while let Some((n, from)) = next {
                    let now = Instant::now();
                    let accept = rx.on_datagram(now, from, &buf[..n]);
                    if let Some(ms) = rx.lateness()
                        && let Some(s) = lateness.record(now, ms, accept == Accept::Late)
                    {
                        tracing::info!(
                            %from,
                            datagrams = s.datagrams,
                            p50_ms = s.p50,
                            p99_ms = s.p99,
                            max_ms = s.max,
                            late = s.late,
                            stalls = s.stalls,
                            "link: how late the pad's datagrams arrived, above the fastest, over the last {:?}",
                            DROPPED_LOG
                        );
                    }
                    if let Some(status) = answers.on(accept, &buf[..n], malformed, now) {
                        // Best effort and never awaited: a full buffer or an unreachable client
                        // drops the answer, never delays the next datagram or ends the loop.
                        let _ = socket.try_send_to(&status.encode(), from);
                    }
                    match accept {
                        Accept::Connected => {
                            // The robot's state at once, not up to a second later.
                            report_at = now;
                            tracing::warn!(%from, "pad client connected — driving");
                            // What it holds was pressed before we saw it. `Gone` already spent a
                            // hold in flight; this covers the first client after a start, which
                            // had no `Gone` before it — every update, every `robotctl configure`.
                            mapper.pad_gone();
                            if pad_loss.found() {
                                sound(&mut robot, proto::SoundTag::Greet).await;
                            }
                        }
                        Accept::Malformed(e) => {
                            malformed += 1;
                            if malformed_logged.is_none_or(|at| now.duration_since(at) >= DROPPED_LOG) {
                                tracing::warn!(%from, error = %e, malformed, "dropping datagrams that are not pad state");
                                malformed_logged = Some(now);
                            }
                        }
                        Accept::Late => {
                            late += 1;
                            if late_logged.is_none_or(|at| now.duration_since(at) >= DROPPED_LOG) {
                                tracing::warn!(%from, late, "dropping datagrams delivered later than stale_ms — a backlog on the link");
                                late_logged = Some(now);
                            }
                        }
                        Accept::Taken | Accept::Stale | Accept::OtherPeer => {}
                    }
                    next = socket.try_recv_from(&mut buf).ok();
                }
                if !reserved_logged && rx.reserved_bits_seen() != 0 {
                    tracing::warn!(bits = format!("{:#06x}", rx.reserved_bits_seen()), "client sends button bits this build does not know; ignored");
                    reserved_logged = true;
                }
            }
            _ = tokio::time::sleep_until(wake.into()) => {}
        }

        let now = Instant::now();
        if now >= housekeeping {
            housekeeping = now + pad_map::BINDINGS_POLL;
            let at = robotd_params::config_stamp(&args.config);
            if at != config_at {
                config_at = at;
                if config.reload(&args.config, &overrides) {
                    tracing::warn!("controller settings reloaded");
                }
            }
            match robot.ask_roller().await {
                Ok(Some(r)) if r != config.roller => {
                    config.roller = r;
                    tracing::warn!(roller = r, "drive mode changed — stick shaping follows");
                }
                Ok(_) => {}
                Err(e) => {
                    tracing::error!(error = %e, "mode request failed");
                    return std::process::ExitCode::FAILURE;
                }
            }
        }

        // Heard, because whoever lost the pad is looking at the robot, not the journal — but only
        // once the loss has lasted to the sit: a blip on bad Wi-Fi is not worth a quack.
        if pad_loss.ask(now) {
            sound(&mut robot, proto::SoundTag::Inquire).await;
        }
        match pad_loss.tick(now) {
            PadLossAction::Nothing => {}
            PadLossAction::Sit => match sit_if_standing(&mut robot).await {
                Ok(true) => pad_loss.settle(),
                // Refused — mid-move, typically. Asked again on the next poll.
                Ok(false) => {}
                Err(e) => {
                    tracing::error!(error = %e, "sit request failed");
                    return std::process::ExitCode::FAILURE;
                }
            },
            PadLossAction::GiveUp => {
                tracing::warn!(
                    "pad gone: the robot would not sit after {:?} of asking — leaving it as it is",
                    pad_map::loss::PAD_LOST_SIT_TRIES
                );
                pad_loss.settle();
            }
        }

        if now >= report_at {
            report_at = now + pad_map::wire::REPORT_EVERY;
            if let Some(peer) = rx.peer() {
                let report = robot_report(&robot_health.borrow(), refused.as_ref(), now);
                // As the answers are: never awaited, dropped when it cannot go.
                let _ = socket.try_send_to(&report.encode(), peer);
            }
        }

        match rx.poll(now) {
            Step::Idle => {}
            Step::Still => {
                // Nothing new for `stale_ms`: the walking stops now, not a deadman after the last
                // stick re-sent. Only the velocity — the head and the body pose are positions, and
                // zeroing them would throw the head to the centre on every blip.
                lateness.stalled();
                tracing::debug!("pad client stalled — the robot stopped where it stands");
                for call in &mut frame {
                    if let proto::Call::RobotMove(m) = call {
                        *m = proto::MoveParams::default();
                    }
                }
                if let Some(bytes) = continuous.next(&frame, now)
                    && let Err(e) = robot.write(bytes).await
                {
                    tracing::error!(error = %e, "send failed");
                    return std::process::ExitCode::FAILURE;
                }
            }
            Step::Gone => {
                tracing::warn!(
                    "pad client gone — robotd's deadman stops the walking; sitting down in {:?} if it stays gone",
                    pad_map::loss::PAD_LOST_SIT
                );
                mapper.pad_gone();
                pad_loss.lost(now);
            }
            Step::Tick(pad) => {
                mapper.tick(&pad, &config, now, &mut out, &mut frame);
                if rx.sender_sounds() {
                    out.retain(|o| !trigger_sound(o));
                }
                for o in out.drain(..) {
                    let result = match &o {
                        Out::Notify(call) => robot.notify(call).await.map(|()| None),
                        Out::Request(call) => robot.request(call).await,
                    };
                    match (result, &o) {
                        (Ok(response), Out::Request(call)) => {
                            pad_map::report(call, response.as_ref());
                            if let Some(reason) = pad_map::refusal(response.as_ref()) {
                                refused = Some((call.method(), reason, Instant::now()));
                            }
                        }
                        (Ok(_), Out::Notify(_)) => {}
                        (Err(e), Out::Notify(call) | Out::Request(call)) => {
                            tracing::error!(error = %e, call = call.method(), "send failed");
                            return std::process::ExitCode::FAILURE;
                        }
                    }
                }
                if let Some(bytes) = continuous.next(&frame, now)
                    && let Err(e) = robot.write(bytes).await
                {
                    tracing::error!(error = %e, "send failed");
                    return std::process::ExitCode::FAILURE;
                }
            }
        }
    }
}

/// How late the driving stream's datagrams arrive, summed up once per [`DROPPED_LOG`] while they
/// do: what the link is like — a spike now and then, a lag that never lets up, or stalls of seconds
/// — before deciding what to do about it. Summed when the next datagram lands, so a stall is
/// counted in the window it ends.
#[derive(Default)]
struct Lateness {
    since: Option<Instant>,
    ms: Vec<u32>,
    late: u32,
    stalls: u32,
}

#[derive(Debug, PartialEq, Eq)]
struct LatenessSummary {
    datagrams: usize,
    p50: u32,
    p99: u32,
    max: u32,
    /// Of `datagrams`, the ones dropped as late.
    late: u32,
    /// How often the robot was stopped for nothing new in `stale_ms`.
    stalls: u32,
}

impl Lateness {
    fn stalled(&mut self) {
        self.stalls += 1;
    }

    fn record(&mut self, now: Instant, ms: u32, dropped: bool) -> Option<LatenessSummary> {
        let since = *self.since.get_or_insert(now);
        self.ms.push(ms);
        self.late += u32::from(dropped);
        if now.duration_since(since) < DROPPED_LOG {
            return None;
        }
        self.ms.sort_unstable();
        let at = |q: usize| self.ms[(self.ms.len() - 1) * q / 100];
        let summary = LatenessSummary {
            datagrams: self.ms.len(),
            p50: at(50),
            p99: at(99),
            max: at(100),
            late: self.late,
            stalls: self.stalls,
        };
        *self = Self::default();
        Some(summary)
    }
}

/// The answer to each datagram — `pad_map::wire::Status`, which `udp-pad.md` documents. Nothing
/// for a malformed one: whatever sent it is not a pad client. A sender that is not the driving
/// one hears "busy" at most every `BUSY_EVERY`: it learns why it is ignored without the robot
/// answering every datagram of a client it is not listening to.
#[derive(Default)]
struct Answers {
    accepted: u32,
    stale: u32,
    busy_answered: Option<Instant>,
}

impl Answers {
    fn on(
        &mut self,
        accept: Accept,
        bytes: &[u8],
        malformed: u64,
        now: Instant,
    ) -> Option<pad_map::wire::Status> {
        use pad_map::wire::Verdict;
        let verdict = match accept {
            Accept::Connected | Accept::Taken => {
                self.accepted = self.accepted.wrapping_add(1);
                Verdict::Driving
            }
            Accept::Stale | Accept::Late => {
                self.stale = self.stale.wrapping_add(1);
                Verdict::Driving
            }
            Accept::OtherPeer => {
                if self
                    .busy_answered
                    .is_some_and(|at| now.duration_since(at) < pad_map::wire::BUSY_EVERY)
                {
                    return None;
                }
                self.busy_answered = Some(now);
                Verdict::Busy
            }
            Accept::Malformed(_) => return None,
        };
        // Decoded already by the receiver; only the seq is read again.
        let seq = pad_map::wire::Packet::decode(bytes).ok()?.seq;
        Some(pad_map::wire::Status {
            verdict,
            seq,
            accepted: self.accepted,
            stale: self.stale,
            malformed: malformed as u32,
        })
    }
}

/// The report for the driving client, from the newest health and the last refusal.
fn robot_report(
    snapshot: &health::Snapshot,
    refused: Option<&(&'static str, String, Instant)>,
    now: Instant,
) -> pad_map::wire::RobotReport {
    pad_map::wire::RobotReport {
        health: snapshot.health.clone(),
        health_error: snapshot.health_error.clone(),
        posture: snapshot.posture.clone(),
        posture_error: snapshot.posture_error.clone(),
        refused: refused
            .filter(|(_, _, at)| now.duration_since(*at) < pad_map::wire::REFUSAL_KEPT)
            .map(|(method, reason, at)| pad_map::wire::Refusal {
                method: (*method).to_owned(),
                reason: reason.clone(),
                ago_ms: now.duration_since(*at).as_millis() as u64,
            }),
    }
}

/// The quack and the wheee the triggers ask for — what a client that plays them itself
/// (`wire::SENDER_SOUNDS`) has the robot leave out. The mouth is a separate call and still
/// opens; the robot's own cues, a client gone and come back, are not the triggers' and still play.
fn trigger_sound(o: &Out) -> bool {
    matches!(
        o,
        Out::Notify(proto::Call::RobotSound(proto::SoundParams {
            tag: proto::SoundTag::Chirp | proto::SoundTag::Wheee,
            ..
        }))
    )
}

/// Play one of the robot's sounds, best effort: a cue, not something worth stopping for.
async fn sound(robot: &mut robotd::Robotd, tag: proto::SoundTag) {
    if let Err(e) = robot.notify(&pad_map::loss::sound(tag)).await {
        tracing::debug!(error = %e, "sound not sent");
    }
}

/// Sit the robot down unless it is sitting already. `Ok(true)` when there is nothing left to do,
/// `Ok(false)` when the robot refused for a reason that passes — `pad_map::loss` reads the
/// answers.
async fn sit_if_standing(robot: &mut robotd::Robotd) -> std::io::Result<bool> {
    let policies = robot.request(&proto::Call::RobotPolicies).await?;
    if pad_map::loss::already_sitting(policies.as_ref()) {
        return Ok(true);
    }
    let answer = robot.request(&pad_map::loss::sit_call()).await?;
    Ok(pad_map::loss::sit_settled(answer.as_ref()))
}

#[cfg(test)]
mod tests {
    #[test]
    fn lateness_is_summed_once_a_window() {
        let (mut l, t) = (super::Lateness::default(), std::time::Instant::now());
        for i in 0..100u32 {
            let at = t + std::time::Duration::from_millis(u64::from(i) * 33);
            assert_eq!(l.record(at, i % 10, false), None);
        }
        l.stalled();
        assert_eq!(
            l.record(t + super::DROPPED_LOG, 900, true),
            Some(super::LatenessSummary {
                datagrams: 101,
                p50: 5,
                p99: 9,
                max: 900,
                late: 1,
                stalls: 1
            })
        );
        assert_eq!(
            l.record(t + super::DROPPED_LOG, 3, false),
            None,
            "a new window"
        );
    }

    use super::*;
    use pad_map::wire::Packet;
    use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};

    /// A fake robotd: answers every request with an accepted IntentResult, `robot.mode` with
    /// walk, `robot.health` with a battery, and `robot.subscribe` with a fallen robot's state
    /// every 100 ms. `robot.do` is refused, so a refusal has something to report. Collects every
    /// method it is sent, with its params — except the health and state ones, which arrive on
    /// connections of their own whenever they like.
    async fn fake_robotd(
        path: std::path::PathBuf,
    ) -> tokio::sync::mpsc::UnboundedReceiver<(String, serde_json::Value)> {
        let listener = tokio::net::UnixListener::bind(&path).unwrap();
        let (tx, rx) = tokio::sync::mpsc::unbounded_channel();
        tokio::spawn(async move {
            while let Ok((stream, _)) = listener.accept().await {
                let tx = tx.clone();
                tokio::spawn(async move {
                    let (read, mut write) = stream.into_split();
                    let mut lines = BufReader::new(read).lines();
                    while let Ok(Some(line)) = lines.next_line().await {
                        let v: serde_json::Value = serde_json::from_str(&line).unwrap();
                        let method = v["method"].as_str().unwrap_or_default().to_owned();
                        let Some(id) = v.get("id") else {
                            let _ = tx.send((method, v["params"].clone()));
                            continue;
                        };
                        let result = match method.as_str() {
                            "robot.mode" => serde_json::json!({"mode": "walk"}),
                            "robot.health" => serde_json::json!({
                                "healthy": true, "battery": {"volts": 7.6, "percent": 61.0}
                            }),
                            "robot.do" => serde_json::json!({
                                "accepted": false, "reason": "not home yet"
                            }),
                            _ => serde_json::json!({"accepted": true}),
                        };
                        let reply =
                            serde_json::json!({"jsonrpc": "2.0", "id": id, "result": result});
                        if write
                            .write_all(format!("{reply}\n").as_bytes())
                            .await
                            .is_err()
                        {
                            return;
                        }
                        if method == "robot.subscribe" {
                            loop {
                                let state = serde_json::json!({"jsonrpc": "2.0",
                                    "method": "robot.state", "params": {"policy": "stand",
                                    "safety": {"fallen": true, "limp": false}}});
                                if write
                                    .write_all(format!("{state}\n").as_bytes())
                                    .await
                                    .is_err()
                                {
                                    return;
                                }
                                tokio::time::sleep(Duration::from_millis(100)).await;
                            }
                        }
                        if method != "robot.health" {
                            let _ = tx.send((method, v["params"].clone()));
                        }
                    }
                });
            }
        });
        rx
    }

    #[tokio::test]
    async fn a_datagram_reaches_robotd_as_a_move_and_silence_stops_it() {
        let dir = tempfile::tempdir().unwrap();
        let sock = dir.path().join("robotd.sock");
        let mut seen = fake_robotd(sock.clone()).await;

        let mut params = robotd_params::Params::default();
        params.netpad.port = 0; // any free port; `serve` reports which
        let (port_tx, port_rx) = tokio::sync::oneshot::channel();
        let args = Args::for_test(sock, dir.path().join("none.toml"));
        let serving = tokio::spawn(serve(args, params, Some(port_tx)));
        let port = port_rx.await.unwrap();

        let client = tokio::net::UdpSocket::bind("127.0.0.1:0").await.unwrap();
        let p = Packet::from_axes(
            1,
            0,
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0],
            pad_map::Buttons::NONE,
        );
        client
            .send_to(&p.encode(), ("127.0.0.1", port))
            .await
            .unwrap();

        let deadline = tokio::time::Instant::now() + std::time::Duration::from_secs(2);
        let mut methods = Vec::new();
        while !methods.iter().any(|m: &String| m == "robot.move") {
            let m = tokio::time::timeout_at(deadline, seen.recv())
                .await
                .expect("a move in 2 s");
            methods.push(m.unwrap().0);
        }
        assert_eq!(methods[0], "robot.mode", "the roller question comes first");
        // Alive on both sides of the silence: a `serve` that had returned would send nothing too,
        // and pass the silence check below for the wrong reason.
        assert!(!serving.is_finished(), "serve exited while driving");

        // Silence: past `stale_ms` the robot is stopped, once, and then no motion is sent at all
        // — not the last stick again, which would keep robotd's deadman fed. (What follows past
        // the timeout — the sound, then the sit — is `a_client_gone_sits_the_robot_after_a_second`.)
        tokio::time::sleep(std::time::Duration::from_millis(600)).await;
        let moves: Vec<_> = std::iter::from_fn(|| seen.try_recv().ok())
            .filter(|(m, _)| m == "robot.move")
            .map(|(_, p)| p)
            .collect();
        let stop = moves.last().expect("a stop");
        assert_eq!(
            (
                stop["vx"].as_f64(),
                stop["vy"].as_f64(),
                stop["vyaw"].as_f64()
            ),
            (Some(0.0), Some(0.0), Some(0.0)),
            "the last move is a stop: {moves:?}"
        );
        tokio::time::sleep(std::time::Duration::from_millis(300)).await;
        let late: Vec<_> = std::iter::from_fn(|| seen.try_recv().ok())
            .map(|(m, _)| m)
            .filter(|m| m == "robot.move" || m == "robot.head" || m == "robot.pose")
            .collect();
        assert!(
            late.is_empty(),
            "motion sent after the client went quiet: {late:?}"
        );
        assert!(!serving.is_finished(), "serve exited during the silence");
    }

    /// Start `serve` against a fake robotd, and a client socket aimed at it.
    async fn start() -> (
        tempfile::TempDir,
        tokio::sync::mpsc::UnboundedReceiver<(String, serde_json::Value)>,
        tokio::task::JoinHandle<std::process::ExitCode>,
        tokio::net::UdpSocket,
    ) {
        let dir = tempfile::tempdir().unwrap();
        let sock = dir.path().join("robotd.sock");
        let seen = fake_robotd(sock.clone()).await;
        let mut params = robotd_params::Params::default();
        params.netpad.port = 0;
        let (port_tx, port_rx) = tokio::sync::oneshot::channel();
        let args = Args::for_test(sock, dir.path().join("none.toml"));
        let serving = tokio::spawn(serve(args, params, Some(port_tx)));
        let port = port_rx.await.unwrap();
        let client = tokio::net::UdpSocket::bind("127.0.0.1:0").await.unwrap();
        client.connect(("127.0.0.1", port)).await.unwrap();
        (dir, seen, serving, client)
    }

    /// Send `buttons` at 30 Hz for `ms`, from `seq` on; returns the next seq.
    async fn hold(
        client: &tokio::net::UdpSocket,
        mut seq: u32,
        buttons: pad_map::Buttons,
        ms: u64,
    ) -> u32 {
        let until = tokio::time::Instant::now() + std::time::Duration::from_millis(ms);
        while tokio::time::Instant::now() < until {
            let p = Packet::from_axes(seq, 0, [0.0; 4], [0.0, 0.0], buttons);
            client.send(&p.encode()).await.unwrap();
            seq = seq.wrapping_add(1);
            tokio::time::sleep(std::time::Duration::from_millis(33)).await;
        }
        seq
    }

    fn drain(
        seen: &mut tokio::sync::mpsc::UnboundedReceiver<(String, serde_json::Value)>,
    ) -> Vec<(String, serde_json::Value)> {
        std::iter::from_fn(|| seen.try_recv().ok())
            .filter(|(m, _)| m != "robot.mode")
            .collect()
    }

    /// The next status answer within a second, passing over robot reports.
    async fn answer(sock: &tokio::net::UdpSocket) -> Option<pad_map::wire::Status> {
        let mut b = [0u8; 2048];
        let until = tokio::time::Instant::now() + Duration::from_secs(1);
        loop {
            let n = tokio::time::timeout_at(until, sock.recv(&mut b))
                .await
                .ok()?
                .ok()?;
            if let Ok(s) = pad_map::wire::Status::decode(&b[..n]) {
                return Some(s);
            }
        }
    }

    /// RT and LT pulled, then let go, with `flags` on every datagram; what robotd was sent.
    async fn triggers(flags: u8) -> Vec<(String, serde_json::Value)> {
        let (_dir, mut seen, serving, client) = start().await;
        let mut seq = 1u32;
        for (ms, t) in [(200, 0.0), (300, 1.0), (200, 0.0)] {
            let until = tokio::time::Instant::now() + Duration::from_millis(ms);
            while tokio::time::Instant::now() < until {
                let mut p = Packet::from_axes(seq, 0, [0.0; 4], [t, t], pad_map::Buttons::NONE);
                p.flags = flags;
                client.send(&p.encode()).await.unwrap();
                seq += 1;
                tokio::time::sleep(Duration::from_millis(33)).await;
            }
        }
        assert!(!serving.is_finished());
        drain(&mut seen)
    }

    /// `SENDER_SOUNDS`: the laptop quacks, so the robot does not — but its mouth still opens.
    #[tokio::test]
    async fn a_client_that_plays_the_sounds_keeps_the_mouth_and_drops_the_quack() {
        let opened = |calls: &[(String, serde_json::Value)]| {
            calls
                .iter()
                .any(|(m, p)| m == "robot.mouth" && p["open"].as_f64() > Some(0.5))
        };
        let tags = |calls: &[(String, serde_json::Value)]| -> Vec<String> {
            calls
                .iter()
                .filter(|(m, _)| m == "robot.sound")
                .filter_map(|(_, p)| p["tag"].as_str().map(str::to_owned))
                .collect()
        };

        let robot = triggers(0).await;
        assert!(opened(&robot), "{robot:?}");
        let t = tags(&robot);
        assert!(
            t.iter().any(|t| t == "chirp") && t.iter().any(|t| t == "wheee"),
            "{t:?}"
        );

        let laptop = triggers(pad_map::wire::SENDER_SOUNDS).await;
        assert!(opened(&laptop), "the mouth still opens: {laptop:?}");
        assert_eq!(tags(&laptop), Vec::<String>::new(), "no quack, no wheee");
    }

    /// Each datagram from the driving client is answered with its seq; a second sender hears
    /// it is locked out, once a second rather than once a datagram.
    #[tokio::test]
    async fn the_driver_hears_each_seq_and_another_sender_hears_busy_once_a_second() {
        use pad_map::wire::Verdict;
        let (_dir, _seen, serving, client) = start().await;
        for seq in [10, 11, 12] {
            let p = Packet::from_axes(seq, 0, [0.0; 4], [0.0, 0.0], pad_map::Buttons::NONE);
            client.send(&p.encode()).await.unwrap();
            let s = answer(&client).await.expect("an answer");
            assert_eq!((s.verdict, s.seq), (Verdict::Driving, seq));
        }
        let p = Packet::from_axes(11, 0, [0.0; 4], [0.0, 0.0], pad_map::Buttons::NONE);
        client.send(&p.encode()).await.unwrap();
        let s = answer(&client).await.expect("a stale one is answered too");
        assert_eq!((s.accepted, s.stale), (3, 1));

        let other = tokio::net::UdpSocket::bind("127.0.0.1:0").await.unwrap();
        other.connect(client.peer_addr().unwrap()).await.unwrap();
        for seq in 0..3 {
            let p = Packet::from_axes(seq, 0, [0.0; 4], [0.0, 0.0], pad_map::Buttons::NONE);
            other.send(&p.encode()).await.unwrap();
        }
        let s = answer(&other).await.expect("busy");
        assert_eq!(s.verdict, Verdict::Busy);
        assert_eq!(answer(&other).await, None, "once a second");
        assert!(!serving.is_finished());
    }

    /// The driving client is sent the robot's health, posture and last refusal, once a second
    /// and at once on connecting; the pad loop answers datagrams all the while.
    #[tokio::test]
    async fn the_driver_is_sent_the_robots_health_and_what_it_refused() {
        let (_dir, _seen, serving, client) = start().await;
        // Let the health and state tasks hear from the robot before the client arrives.
        tokio::time::sleep(Duration::from_millis(300)).await;
        let mut seq = hold(&client, 1, pad_map::Buttons::NONE, 100).await;
        let mut report = None;
        let until = tokio::time::Instant::now() + Duration::from_secs(3);
        let mut b = [0u8; 2048];
        while tokio::time::Instant::now() < until {
            // Keep driving, so the lock holds; press A, which the default config makes a skill.
            let buttons = if seq % 20 < 10 {
                pad_map::Buttons::A
            } else {
                pad_map::Buttons::NONE
            };
            seq = hold(&client, seq, buttons, 33).await;
            while let Ok(n) = client.try_recv(&mut b) {
                if let Ok(r) = pad_map::wire::RobotReport::decode(&b[..n])
                    && r.refused.is_some()
                {
                    report = Some(r);
                }
            }
            if report.is_some() {
                break;
            }
        }
        let r = report.expect("a report carrying the refusal within 3 s");
        let health = r.health.expect("health");
        assert!(health.healthy);
        assert_eq!(health.battery.unwrap().percent, 61.0);
        let posture = r.posture.expect("posture");
        assert!(posture.fallen && posture.policy == "stand", "{posture:?}");
        let refused = r.refused.unwrap();
        assert_eq!(
            (refused.method.as_str(), refused.reason.as_str()),
            ("robot.do", "not home yet")
        );
        assert!(!serving.is_finished());
    }

    /// The first client after `netpadd` starts — at every update and every `robotctl configure`
    /// — already holding Start. No `Gone` came first to spend the hold, so the connection must:
    /// letting go stands nothing up.
    #[tokio::test]
    async fn a_first_client_holding_start_stands_nothing_up() {
        let (_dir, mut seen, serving, client) = start().await;
        let seq = hold(&client, 1, pad_map::Buttons::START, 400).await;
        hold(&client, seq, pad_map::Buttons::NONE, 200).await;
        let sent = drain(&mut seen);
        assert!(
            !sent
                .iter()
                .any(|(m, _)| m == "robot.init" || m == "robot.enable"),
            "{sent:?}"
        );
        assert!(!serving.is_finished());
    }

    /// The client going quiet is a pad going away, as `padd` has it: a second later the robot asks
    /// where it went and sits, and greets it when it comes back.
    #[tokio::test]
    async fn a_client_gone_sits_the_robot_after_a_second() {
        let (_dir, mut seen, serving, client) = start().await;
        let seq = hold(&client, 1, pad_map::Buttons::NONE, 100).await;
        drain(&mut seen);

        // Silent: gone at timeout_ms (1000), the sit due a second after that.
        tokio::time::sleep(std::time::Duration::from_millis(1500)).await;
        let early = drain(&mut seen);
        assert!(
            !early
                .iter()
                .any(|(m, _)| m == "robot.do" || m == "robot.sound"),
            "not yet: {early:?}"
        );

        tokio::time::sleep(std::time::Duration::from_millis(900)).await;
        let later = drain(&mut seen);
        assert!(
            later
                .iter()
                .any(|(m, p)| m == "robot.do" && p["skill"] == "sit_toggle"),
            "sits after a second: {later:?}"
        );
        assert!(
            later
                .iter()
                .any(|(m, p)| m == "robot.sound" && p["tag"] == "inquire"),
            "and asks where the pad went: {later:?}"
        );

        hold(&client, seq.wrapping_add(1000), pad_map::Buttons::NONE, 100).await;
        let back = drain(&mut seen);
        assert!(
            back.iter()
                .any(|(m, p)| m == "robot.sound" && p["tag"] == "greet"),
            "greeted: {back:?}"
        );
        assert!(!serving.is_finished());
    }

    /// A blip — the client gone past `timeout_ms` and back well inside the second — is silent
    /// both ways: bad Wi-Fi drops out like this many times a minute, and a quack per blip is noise.
    #[tokio::test]
    async fn a_client_blip_is_silent() {
        let (_dir, mut seen, serving, client) = start().await;
        let seq = hold(&client, 1, pad_map::Buttons::NONE, 100).await;
        drain(&mut seen);

        tokio::time::sleep(std::time::Duration::from_millis(1200)).await;
        hold(&client, seq.wrapping_add(1000), pad_map::Buttons::NONE, 100).await;
        tokio::time::sleep(std::time::Duration::from_millis(1200)).await;
        let sent = drain(&mut seen);
        assert!(
            !sent
                .iter()
                .any(|(m, _)| m == "robot.sound" || m == "robot.do"),
            "{sent:?}"
        );
        assert!(!serving.is_finished());
    }

    /// A stall shorter than `timeout_ms` keeps the client: no sound, no sit, and a Start tapped
    /// once it is back is a tap — not a button taken as already held, as after a reconnection.
    #[tokio::test]
    async fn a_stalled_client_keeps_its_session() {
        let (_dir, mut seen, serving, client) = start().await;
        let seq = hold(&client, 1, pad_map::Buttons::NONE, 100).await;
        tokio::time::sleep(std::time::Duration::from_millis(600)).await;
        let seq = hold(&client, seq, pad_map::Buttons::START, 100).await;
        hold(&client, seq, pad_map::Buttons::NONE, 200).await;
        let sent = drain(&mut seen);
        assert!(
            sent.iter().any(|(m, _)| m == "robot.init"),
            "the tap stands it up: {sent:?}"
        );
        assert!(
            !sent
                .iter()
                .any(|(m, _)| m == "robot.sound" || m == "robot.do"),
            "{sent:?}"
        );
        assert!(!serving.is_finished());
    }
}
