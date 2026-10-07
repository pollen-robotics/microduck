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
//! receiver: no locks, no channels, no work-stealing jitter. `receiver.rs` decides; this file
//! only waits and writes.

mod receiver;
mod robotd;

use std::path::PathBuf;
use std::time::{Duration, Instant};

use clap::Parser;
use pad_map::{Config, Continuous, HEARTBEAT, Mapper, Out};
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
    /// Deflection below this counts as centre — `padd`'s value.
    #[arg(long, default_value_t = 0.1)]
    deadzone: f64,
    /// Full-deflection head travel, radians — `padd`'s value.
    #[arg(long, default_value_t = 2.5)]
    max_head: f64,
}

#[cfg(test)]
impl Args {
    fn for_test(socket: PathBuf, config: PathBuf) -> Self {
        Self {
            socket,
            config,
            should_run: false,
            deadzone: 0.1,
            max_head: 2.5,
        }
    }
}

/// Malformed datagrams are logged with their count at most this often: a LAN can hold a noisy
/// neighbour, and one line per packet would bury the journal.
const MALFORMED_LOG: Duration = Duration::from_secs(10);

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
    let (bindings, imu_head, drive) = pad_map::read_bindings(&args.config);
    let mut config = Config {
        bindings,
        imu_head,
        drive,
        deadzone: args.deadzone,
        max_head: args.max_head,
        roller,
    };
    tracing::warn!(
        port,
        max_hz = np.max_hz,
        timeout_ms = np.timeout_ms,
        roller,
        "listening for a pad over UDP"
    );

    let mut rx = Receiver::new(Timing {
        min_interval,
        timeout: Duration::from_millis(np.timeout_ms),
        fallback: HEARTBEAT,
    });
    let (mut mapper, mut continuous) = (Mapper::new(), Continuous::default());
    let (mut out, mut frame) = (Vec::new(), Vec::new());
    let mut buf = [0u8; 512];
    let mut housekeeping = Instant::now() + pad_map::BINDINGS_POLL;
    let mut config_at = std::fs::metadata(&args.config)
        .and_then(|m| m.modified())
        .ok();
    let (mut malformed, mut malformed_logged) = (0u64, None::<Instant>);
    let mut reserved_logged = false;

    loop {
        let wake = rx.deadline().map_or(housekeeping, |d| d.min(housekeeping));
        tokio::select! {
            biased;
            got = socket.recv_from(&mut buf) => {
                // Handle this one, then everything else already waiting, in arrival order: edges
                // need every datagram, and the receiver keeps only the newest state.
                let mut next = got.ok();
                while let Some((n, from)) = next {
                    let now = Instant::now();
                    match rx.on_datagram(now, from, &buf[..n]) {
                        Accept::Connected => tracing::warn!(%from, "pad client connected — driving"),
                        Accept::Malformed(e) => {
                            malformed += 1;
                            if malformed_logged.is_none_or(|at| now.duration_since(at) >= MALFORMED_LOG) {
                                tracing::warn!(%from, error = %e, malformed, "dropping datagrams that are not pad state");
                                malformed_logged = Some(now);
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
            let at = std::fs::metadata(&args.config)
                .and_then(|m| m.modified())
                .ok();
            if at != config_at {
                config_at = at;
                (config.bindings, config.imu_head, config.drive) =
                    pad_map::read_bindings(&args.config);
                tracing::warn!("button bindings reloaded");
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

        match rx.poll(now) {
            Step::Idle => {}
            Step::Gone => {
                tracing::warn!(
                    "pad client gone — sending nothing; robotd's deadman holds the robot"
                );
                mapper.pad_gone();
            }
            Step::Tick(pad) => {
                mapper.tick(&pad, &config, now, &mut out, &mut frame);
                for o in out.drain(..) {
                    let result = match &o {
                        Out::Notify(call) => robot.notify(call).await.map(|()| None),
                        Out::Request(call) => robot.request(call).await,
                    };
                    match (result, &o) {
                        (Ok(response), Out::Request(call)) => {
                            pad_map::report(call, response.as_ref())
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

#[cfg(test)]
mod tests {
    use super::*;
    use pad_map::wire::Packet;
    use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};

    /// A fake robotd: answers every request with an accepted IntentResult, and `robot.mode`
    /// with walk; collects every method name it is sent.
    async fn fake_robotd(path: std::path::PathBuf) -> tokio::sync::mpsc::UnboundedReceiver<String> {
        let listener = tokio::net::UnixListener::bind(&path).unwrap();
        let (tx, rx) = tokio::sync::mpsc::unbounded_channel();
        tokio::spawn(async move {
            let (stream, _) = listener.accept().await.unwrap();
            let (read, mut write) = stream.into_split();
            let mut lines = BufReader::new(read).lines();
            while let Ok(Some(line)) = lines.next_line().await {
                let v: serde_json::Value = serde_json::from_str(&line).unwrap();
                let method = v["method"].as_str().unwrap_or_default().to_owned();
                if let Some(id) = v.get("id") {
                    let result = if method == "robot.mode" {
                        serde_json::json!({"mode": "walk"})
                    } else {
                        serde_json::json!({"accepted": true})
                    };
                    let reply = serde_json::json!({"jsonrpc": "2.0", "id": id, "result": result});
                    write
                        .write_all(format!("{reply}\n").as_bytes())
                        .await
                        .unwrap();
                }
                let _ = tx.send(method);
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
            methods.push(m.unwrap());
        }
        assert_eq!(methods[0], "robot.mode", "the roller question comes first");
        // Alive on both sides of the silence: a `serve` that had returned would send nothing too,
        // and pass the silence check below for the wrong reason.
        assert!(!serving.is_finished(), "serve exited while driving");

        // Silence: after the timeout nothing more is sent.
        tokio::time::sleep(std::time::Duration::from_millis(400)).await;
        while seen.try_recv().is_ok() {}
        tokio::time::sleep(std::time::Duration::from_millis(300)).await;
        let late: Vec<_> = std::iter::from_fn(|| seen.try_recv().ok())
            .filter(|m| m != "robot.mode")
            .collect();
        assert!(
            late.is_empty(),
            "sent after the client went quiet: {late:?}"
        );
        assert!(!serving.is_finished(), "serve exited during the silence");
    }
}
