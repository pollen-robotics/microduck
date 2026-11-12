//! `btd` — the BLE front door onto the robot's API.
//!
//! Runs only on the robot. See the crate docs in `lib.rs` for what it is and why it owns
//! nothing; this file is argument parsing, logging and startup.

use std::path::PathBuf;
use std::process::ExitCode;

use btd::upstream::{NameChoice, Sockets};
use clap::Parser;

#[derive(Parser, Debug)]
#[command(
    version,
    about = "BLE transport adapter for the robot API",
    long_about = "Serves a GATT service that carries the same JSON-RPC lines as every other \
                  transport, forwarding each request to the service that owns it. Exposes a \
                  subset: status, update trigger and progress. Never motor control."
)]
struct Args {
    /// `updaterd`'s socket.
    #[arg(long, default_value = duck_ipc_proto::socket::UPDATER)]
    update_socket: PathBuf,

    /// `robotd`'s socket.
    #[arg(long, default_value = duck_ipc_proto::socket::ROBOT)]
    robot_socket: PathBuf,

    /// `configd`'s socket — wifi and the robot's identity.
    #[arg(long, default_value = duck_ipc_proto::socket::CONFIG)]
    config_socket: PathBuf,

    /// `nfcd`'s socket — writing a tag.
    #[arg(long, default_value = duck_ipc_proto::socket::NFC)]
    nfc_socket: PathBuf,

    /// Retained for compatibility with existing service overrides. Pairing is now required
    /// unless `--insecure-no-pairing` is explicitly selected for a controlled bench.
    #[arg(long, conflicts_with = "insecure_no_pairing")]
    require_pairing: bool,

    /// Disable BLE link encryption only on an isolated development bench.
    #[arg(long, hide = true, conflicts_with = "require_pairing")]
    insecure_no_pairing: bool,

    /// The robot's 32-byte setup-code key, as 64 hex characters in a file. Turns secure mode on.
    ///
    /// Without it, the key systemd imported as the `ble-psk` credential is used when there is one:
    /// `robotctl system setup-code export` leaves it from the ID EEPROM (or a dev board's
    /// installed key) before this starts, so secure mode follows the robot with no flag. The id
    /// bound into the handshake is the robot's first name, derived from the SoC serial — not a
    /// flag, so it cannot drift from the one the QR carries.
    #[arg(long, conflicts_with = "insecure_no_pairing")]
    secure_psk_file: Option<PathBuf>,

    /// Pin the advertised name, instead of asking `configd` what the robot is called.
    ///
    /// Bench use. The name someone sees in a phone's Bluetooth list is `configd`'s — set with
    /// `robotctl system set-name` or `system.setName` from an app — and passing this stops it being
    /// reconciled, so a rename no longer takes effect until the flag is removed.
    #[arg(long)]
    name: Option<String>,
}

fn hostname() -> String {
    // /etc/hostname rather than the `hostname` crate or a libc call: one file read, no
    // dependency, and it is what the board is actually configured with.
    std::fs::read_to_string("/etc/hostname")
        .map(|s| s.trim().to_owned())
        .ok()
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| "robot".to_owned())
}

/// **Single-threaded on purpose**, which `bluer`'s own examples also do.
///
/// A chunked request arrives as several `WriteValue` calls, and `dbus-crossroads` dispatches each
/// as its own task. On a multi-threaded runtime those tasks can be invoked out of order, and a
/// reordered chunk does not fail — it reassembles into something that parses as the wrong thing.
/// `{"id":1,"jsonrpc":"2.0","method":"system.info","params":{}}` arriving as chunks 1, 3, 2 becomes
/// `{"id":1,"jsonrpc":"2.info","params":{}}`: valid JSON, missing a field, and a parse error that
/// blames the client. It cost two rounds of debugging on hardware.
///
/// On one thread the dispatcher invokes handlers in the order it reads them off the D-Bus socket,
/// which is the order the client sent them — and the client acknowledges each write before sending
/// the next, so that order is well-defined.
///
/// Affordable because this daemon does no CPU work: it moves bytes between a radio and three unix
/// sockets. Anything blocking added later would stall the whole service, which is a reason to keep
/// it that way rather than an argument against it.
#[tokio::main(flavor = "current_thread")]
async fn main() -> ExitCode {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_writer(std::io::stderr)
        .init();

    let args = Args::parse();
    duck_ipc_proto::log_startup_identity!("btd");

    let sockets = Sockets {
        updater: args.update_socket,
        robot: args.robot_socket,
        config: args.config_socket,
        nfc: args.nfc_socket,
    };
    // The hostname is only a last resort now: `configd` derives a distinguishable default from the
    // board's SoC serial, so `radxa-zero3` appears only when `configd` cannot be reached at all.
    let name = NameChoice {
        pinned: args.name,
        fallback: hostname(),
    };

    let require_pairing = args.require_pairing || !args.insecure_no_pairing;
    // `--insecure-no-pairing` is a bench without the robot's own security, so an imported key does
    // not turn secure mode on behind its back.
    let psk_file = args
        .secure_psk_file
        .or_else(|| (!args.insecure_no_pairing).then(imported_psk).flatten());
    let secure = match psk_file {
        // The id is the robot's first name, the same one `robotctl system setup-code` prints into
        // the QR. A board with no SoC serial has no first name, and refusing beats guessing an id
        // that no printed code would carry.
        Some(path) => {
            let Some(id) = robotd_params::serial::this_first_name() else {
                tracing::error!(
                    "cannot start secure BLE mode: no SoC serial to derive the robot id from"
                );
                return ExitCode::FAILURE;
            };
            match btd::secure::Config::load(&path, id.clone()) {
                Ok(config) => {
                    tracing::info!(robot_id = %id, "secure BLE mode");
                    Some(config)
                }
                Err(error) => {
                    tracing::error!(%error, "cannot start secure BLE mode");
                    return ExitCode::FAILURE;
                }
            }
        }
        None => None,
    };
    if !require_pairing {
        // Loud, every start, because the whole point of choosing the usable default is that the
        // insecurity stays visible rather than becoming the thing nobody remembers.
        tracing::warn!(
            "serving WITHOUT pairing: the PIN and any wifi passphrase cross this link in clear, \
             readable by anyone in range. Development only — see app-path-design.md §5.5"
        );
    }

    run(sockets, name, require_pairing, secure).await
}

/// The setup-code key systemd imported for this service (`ImportCredential=ble-psk`), if any.
fn imported_psk() -> Option<PathBuf> {
    let path = PathBuf::from(std::env::var_os("CREDENTIALS_DIRECTORY")?).join("ble-psk");
    path.is_file().then_some(path)
}

#[cfg(target_os = "linux")]
async fn run(
    sockets: Sockets,
    name: NameChoice,
    require_pairing: bool,
    secure: Option<btd::secure::Config>,
) -> ExitCode {
    tokio::select! {
        // `serve` retries the radio in place and is not expected to return at all: an adapter that
        // is missing, unpowered or wedged is handled there rather than by dying and letting
        // `Restart=always` do it. Both arms are therefore a bug in `serve`, not a radio fault — kept
        // because the signature allows them, and non-zero because a `btd` that has stopped serving
        // BLE must not look healthy.
        result = btd::bluez::serve(sockets, name, require_pairing, secure) => match result {
            Ok(()) => {
                tracing::error!("the BLE service returned; it is supposed to retry instead");
                ExitCode::FAILURE
            }
            Err(e) => {
                tracing::error!(error = %e, "BLE service failed");
                ExitCode::FAILURE
            }
        },
        () = shutdown() => {
            tracing::info!("shutting down");
            ExitCode::SUCCESS
        }
    }
}

/// Off-Linux this daemon has nothing to serve, and says so rather than pretending.
///
/// The crate still builds and tests here, which is the point: `cargo test` on a laptop is the
/// onboarding path, and only the radio is Linux-only.
#[cfg(not(target_os = "linux"))]
async fn run(
    _sockets: Sockets,
    _name: NameChoice,
    _require_pairing: bool,
    _secure: Option<btd::secure::Config>,
) -> ExitCode {
    tracing::error!(
        "btd needs BlueZ, which is Linux-only. This binary exists here so the crate builds \
         and its tests run; it cannot serve BLE on this platform."
    );
    ExitCode::FAILURE
}

/// Resolve on SIGTERM (systemd stop) or SIGINT (Ctrl-C).
#[cfg(target_os = "linux")]
async fn shutdown() {
    use tokio::signal::unix::{SignalKind, signal};

    let mut term = match signal(SignalKind::terminate()) {
        Ok(s) => s,
        Err(e) => {
            tracing::warn!(error = %e, "cannot listen for SIGTERM");
            return std::future::pending().await;
        }
    };
    tokio::select! {
        _ = term.recv() => {}
        _ = tokio::signal::ctrl_c() => {}
    }
}

#[cfg(test)]
mod security_defaults {
    use super::*;

    #[test]
    fn pairing_requires_explicit_insecure_override_to_disable() {
        let normal = Args::try_parse_from(["btd"]).unwrap();
        assert!(!normal.insecure_no_pairing);
        let bench = Args::try_parse_from(["btd", "--insecure-no-pairing"]).unwrap();
        assert!(bench.insecure_no_pairing);
        assert!(
            Args::try_parse_from(["btd", "--require-pairing", "--insecure-no-pairing"]).is_err()
        );
    }
}
