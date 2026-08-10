//! `configd` — see the crate docs in `lib.rs` for why this is its own service.
//!
//! This file is the socket, the peer policy and the dispatch.

use std::path::PathBuf;
use std::process::ExitCode;
use std::sync::Arc;

use clap::Parser;
use configd::net::{FakeNet, Net};
use configd::pad::{FakePads, Pads};
use configd::power;
use configd::store::Store;
use configd::{driver, pad};
use duck_ipc_proto as proto;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::{UnixListener, UnixStream};

/// Owner and group read/write, nothing for others — the same as every other socket here. This
/// is the first layer of access control: reaching the socket at all requires the group.
const SOCKET_MODE: u32 = 0o660;

/// Refuse absurdly long lines rather than buffering them.
const MAX_LINE: usize = 64 * 1024;

const DEFAULT_STATE_DIR: &str = "/var/lib/robot/config";

#[derive(Parser, Debug)]
#[command(
    version,
    about = "Wifi and robot identity",
    long_about = "Serves net.* and system.* over a unix socket. Drives NetworkManager for wifi \
                  and never stores a credential. Lives apart from robotd because config must be \
                  reachable when the robot is not."
)]
struct Args {
    #[arg(long, default_value = proto::socket::CONFIG)]
    socket: PathBuf,

    /// Where the config file lives. Outside any release directory, so it survives an update
    /// *and* a rollback.
    #[arg(long, default_value = DEFAULT_STATE_DIR)]
    state_dir: PathBuf,

    /// Users permitted to make changes, by name. Read-only calls are never gated.
    ///
    /// Names rather than numbers, because `systemd-sysusers` allocates uids dynamically: a
    /// numeric uid in a shipped unit file is right on the board it was written for and wrong
    /// on the next one. Unresolvable names are a warning rather than a fatal error, so a
    /// robot missing an optional user still serves everything read-only.
    #[arg(long, value_delimiter = ',')]
    allow_user: Vec<String>,

    /// Groups permitted to make changes, by name.
    #[arg(long, value_delimiter = ',')]
    allow_group: Vec<String>,

    /// Serve an in-memory wifi stack instead of NetworkManager.
    ///
    /// The whole `net.*` surface, including the failures that are awkward to provoke against a
    /// real access point — a wrong passphrase especially — without a board or a radio.
    #[arg(long)]
    fake_net: bool,

    /// Serve an in-memory set of gamepads instead of BlueZ.
    ///
    /// The whole `pad.*` surface, including the cases that need hardware to arrange — two pads in
    /// pairing mode at once, or none at all.
    ///
    /// Separate from `--fake-net` rather than one `--fake`: faking the radio while driving a real
    /// NetworkManager is a combination worth having, and a single flag would make bench work choose
    /// between them.
    #[arg(long)]
    fake_pads: bool,
}

/// Who may change this robot's configuration.
///
/// Two tiers, matching `updaterd` (`architecture.md` §2.2): the socket's group decides who may
/// *talk*, and this decides who may *change*. Read-only calls are deliberately ungated so
/// support can inspect a robot it is not authorised to reconfigure — and so `btd` can report
/// wifi status without being trusted to join a network.
///
/// An unknown peer is denied. `SO_PEERCRED` failing is not something to shrug at when the
/// decision is "may this reboot the robot".
struct PeerPolicy {
    owner_uid: u32,
    allow_uids: Vec<u32>,
    allow_gids: Vec<u32>,
}

impl PeerPolicy {
    fn may_mutate(&self, peer: Option<&tokio::net::unix::UCred>) -> Result<(), String> {
        let Some(peer) = peer else {
            return Err("peer credentials unavailable; refusing a mutating request".into());
        };
        if peer.uid() == self.owner_uid
            || self.allow_uids.contains(&peer.uid())
            || self.allow_gids.contains(&peer.gid())
        {
            return Ok(());
        }
        Err(format!(
            // The flags are `--allow-user`/`--allow-group` and take *names*, not numbers — see the
            // arg docs above for why. Naming `--allow-uid` here sent anyone following the advice
            // straight into "unexpected argument", which is a worse failure than the original.
            "uid {} / gid {} may not change this robot's configuration; add its user or group to \
             --allow-user or --allow-group (in configd.service), or run as uid {}",
            peer.uid(),
            peer.gid(),
            self.owner_uid
        ))
    }
}

/// A user name to a uid.
///
/// `SO_PEERCRED` reports a numeric uid, so a name has to become a number somewhere. Doing it
/// here, once at startup, means a unit file can name `btd` and stay correct on a board where
/// sysusers allocated a different number.
fn resolve_uid(name: &str) -> Option<u32> {
    let cname = std::ffi::CString::new(name).ok()?;
    // Safety: `getpwnam` takes a NUL-terminated string and returns a pointer into a static
    // buffer or null. Read immediately and nothing is retained.
    let entry = unsafe { libc::getpwnam(cname.as_ptr()) };
    if entry.is_null() {
        tracing::warn!(
            user = name,
            "no such user; it cannot change this robot's configuration"
        );
        return None;
    }
    let uid = unsafe { (*entry).pw_uid };
    tracing::info!(user = name, uid, "may change configuration");
    Some(uid)
}

fn resolve_gid(name: &str) -> Option<u32> {
    let cname = std::ffi::CString::new(name).ok()?;
    // Safety: as above, for the group database.
    let entry = unsafe { libc::getgrnam(cname.as_ptr()) };
    if entry.is_null() {
        tracing::warn!(
            group = name,
            "no such group; it cannot change this robot's configuration"
        );
        return None;
    }
    let gid = unsafe { (*entry).gr_gid };
    tracing::info!(group = name, gid, "may change configuration");
    Some(gid)
}

struct Service {
    net: Arc<dyn Net>,
    pads: Arc<dyn Pads>,
    store: Store,
    policy: PeerPolicy,
}

fn log_startup_identity(service: &str) {
    let exe = std::env::current_exe()
        .map(|p| p.display().to_string())
        .unwrap_or_else(|_| "unknown".to_owned());
    tracing::warn!(
        service,
        build = %proto::build_info!(),
        exe,
        pid = std::process::id(),
        "starting"
    );
}

fn hostname() -> String {
    std::fs::read_to_string("/etc/hostname")
        .map(|s| s.trim().to_owned())
        .ok()
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| "robot".to_owned())
}

#[tokio::main]
async fn main() -> ExitCode {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_writer(std::io::stderr)
        .init();

    let args = Args::parse();
    log_startup_identity("configd");

    let net: Arc<dyn Net> = match backend(args.fake_net).await {
        Ok(net) => net,
        Err(e) => {
            tracing::error!(error = %e, "no wifi backend");
            return ExitCode::FAILURE;
        }
    };

    // Unlike wifi, an unreachable radio is **not** a reason to refuse to start. `configd` answers
    // `net.*` and `system.*` too, and a board whose Bluetooth has not appeared yet — which on this
    // one takes about 73 seconds — must not lose its wifi provisioning path over it. So a failure
    // here degrades to "no pads" and says why.
    let pads: Arc<dyn Pads> = match pad_backend(args.fake_pads).await {
        Ok(pads) => pads,
        Err(e) => {
            tracing::warn!(error = %e, "no gamepad backend; pad.* will report no pads");
            Arc::new(FakePads::with(Vec::new()))
        }
    };

    let service = Arc::new(Service {
        net,
        pads,
        store: Store::new(args.state_dir.join("config.json"), hostname()),
        policy: PeerPolicy {
            owner_uid: unsafe { libc::getuid() },
            allow_uids: args
                .allow_user
                .iter()
                .filter_map(|name| resolve_uid(name))
                .collect(),
            allow_gids: args
                .allow_group
                .iter()
                .filter_map(|name| resolve_gid(name))
                .collect(),
        },
    });

    tokio::select! {
        result = serve(service, args.socket) => match result {
            Ok(()) => ExitCode::SUCCESS,
            Err(e) => {
                tracing::error!(error = %e, "IPC server failed");
                ExitCode::FAILURE
            }
        },
        () = shutdown() => {
            tracing::info!("shutting down");
            ExitCode::SUCCESS
        }
    }
}

#[cfg(target_os = "linux")]
async fn backend(fake: bool) -> Result<Arc<dyn Net>, String> {
    if fake {
        tracing::warn!("serving a FAKE wifi stack; nothing here touches a real network");
        return Ok(Arc::new(FakeNet::new()));
    }
    // A failure here means NetworkManager is not on the bus at all, which on this board means
    // the wifi migration was never run. Say that, rather than reporting a D-Bus error nobody
    // can act on.
    configd::nm::NetworkManager::new()
        .await
        .map(|nm| Arc::new(nm) as Arc<dyn Net>)
        .map_err(|e| {
            format!(
                "cannot reach NetworkManager on the system bus ({e}). If this board still runs \
                 netplan, run scripts/migrate-network.sh first."
            )
        })
}

#[cfg(not(target_os = "linux"))]
async fn backend(_fake: bool) -> Result<Arc<dyn Net>, String> {
    // NetworkManager is Linux-only, so off-board there is only the fake — and saying so is more
    // useful than refusing to start, because `--fake-net` is how the surface is exercised from a
    // laptop anyway.
    tracing::warn!("not Linux: serving a FAKE wifi stack");
    Ok(Arc::new(FakeNet::new()))
}

#[cfg(target_os = "linux")]
async fn pad_backend(fake: bool) -> Result<Arc<dyn Pads>, String> {
    if fake {
        tracing::warn!("serving FAKE gamepads; nothing here touches a real radio");
        return Ok(Arc::new(FakePads::new()));
    }
    configd::bluez::BlueZ::new()
        .await
        .map(|bluez| Arc::new(bluez) as Arc<dyn Pads>)
        .map_err(|e| format!("cannot reach bluetoothd on the system bus ({e})"))
}

#[cfg(not(target_os = "linux"))]
async fn pad_backend(_fake: bool) -> Result<Arc<dyn Pads>, String> {
    // BlueZ is Linux-only. Same reasoning as the wifi backend: the fake is how `pad.*` is exercised
    // from a laptop, so serving it is more useful than refusing.
    tracing::warn!("not Linux: serving FAKE gamepads");
    Ok(Arc::new(FakePads::new()))
}

async fn serve(service: Arc<Service>, socket_path: PathBuf) -> std::io::Result<()> {
    // A leftover socket from a killed process must not stop us coming up.
    if socket_path.exists() {
        tracing::warn!(path = %socket_path.display(), "removing stale socket");
        let _ = std::fs::remove_file(&socket_path);
    }
    if let Some(parent) = socket_path.parent() {
        let _ = std::fs::create_dir_all(parent);
    }

    let listener = UnixListener::bind(&socket_path)?;

    use std::os::unix::fs::PermissionsExt;
    std::fs::set_permissions(&socket_path, std::fs::Permissions::from_mode(SOCKET_MODE))?;

    tracing::info!(
        path = %socket_path.display(),
        mode = format!("{SOCKET_MODE:o}"),
        "serving config IPC"
    );

    loop {
        let (stream, _) = match listener.accept().await {
            Ok(pair) => pair,
            Err(e) => {
                tracing::warn!(error = %e, "accept failed");
                continue;
            }
        };
        let service = Arc::clone(&service);
        tokio::spawn(async move {
            if let Err(e) = handle(service, stream).await {
                tracing::debug!(error = %e, "connection ended");
            }
        });
    }
}

async fn handle(service: Arc<Service>, stream: UnixStream) -> std::io::Result<()> {
    // Read once, per connection: credentials cannot change under a live socket, and asking per
    // request would be a syscall for an answer that is already known.
    let peer = stream.peer_cred().ok();
    let (read_half, mut write_half) = stream.into_split();
    let mut lines = BufReader::new(read_half).lines();

    while let Some(line) = lines.next_line().await? {
        if line.trim().is_empty() {
            continue;
        }
        if line.len() > MAX_LINE {
            let response = proto::Response::err(
                None,
                proto::Error::new(proto::code::INVALID_REQUEST, "request too large"),
            );
            write_line(&mut write_half, &response).await?;
            continue;
        }

        let request: proto::Request = match serde_json::from_str(&line) {
            Ok(request) => request,
            Err(e) => {
                let response = proto::Response::err(
                    None,
                    proto::Error::new(proto::code::PARSE_ERROR, e.to_string()),
                );
                write_line(&mut write_half, &response).await?;
                continue;
            }
        };

        // Notifications get no reply, per the spec.
        let Some(id) = request.id.clone() else {
            continue;
        };

        let response = match request.as_call() {
            Ok(call) => dispatch(&service, peer.as_ref(), id, &call).await,
            Err(e) => proto::Response::err(Some(id), e),
        };
        write_line(&mut write_half, &response).await?;
    }
    Ok(())
}

async fn dispatch(
    service: &Service,
    peer: Option<&tokio::net::unix::UCred>,
    id: proto::Id,
    call: &proto::Call,
) -> proto::Response {
    // Authorise before doing anything, and log the caller alongside the method: "who told this
    // robot to reboot" is the first thing support asks.
    if call.is_mutating() {
        if let Err(reason) = service.policy.may_mutate(peer) {
            tracing::warn!(method = call.method(), %reason, "refused");
            return proto::Response::err(
                Some(id),
                proto::Error::new(proto::code::PERMISSION_DENIED, reason),
            );
        }
        tracing::info!(
            method = call.method(),
            uid = peer.map(|p| p.uid()),
            gid = peer.map(|p| p.gid()),
            "authorised"
        );
    }

    match call {
        proto::Call::Hello(_) => proto::Response::ok(
            Some(id),
            &proto::HelloResult {
                api_version: proto::API_VERSION,
                daemon_version: proto::semver::Version::parse(env!("CARGO_PKG_VERSION")).ok(),
                revision: proto::build_info!().revision.map(str::to_owned),
            },
        ),

        proto::Call::NetStatus => reply(id, service.net.status().await),
        proto::Call::NetScan => reply(id, service.net.scan().await),
        proto::Call::NetConnect(params) => {
            // `params` redacts the key in its own Debug, which is what makes logging the request
            // safe. See `NetConnectParams` in duck-ipc-proto.
            tracing::info!(?params, "joining a network");
            reply(
                id,
                service
                    .net
                    .connect(&params.ssid, params.psk.as_deref())
                    .await,
            )
        }
        proto::Call::NetForget(params) => reply(id, service.net.forget(&params.ssid).await),

        proto::Call::SystemInfo => proto::Response::ok(
            Some(id),
            &proto::SystemInfoResult {
                name: service.store.name(),
                // No per-device identity exists yet; provisioning has to define one
                // (`updater-design.md` §5.7). `None` rather than something invented.
                serial: None,
                uptime_seconds: uptime_seconds(),
            },
        ),
        proto::Call::SystemSetName(params) => match service.store.set_name(&params.name) {
            Ok(name) => {
                // The advertised BLE name does not change until btd restarts, and saying so is
                // better than a client wondering why the phone still shows the old one.
                tracing::info!(%name, "renamed; btd advertises the new name after a restart");
                proto::Response::ok(Some(id), &proto::SetNameResult { name })
            }
            Err(e) => proto::Response::err(
                Some(id),
                proto::Error::new(proto::code::INVALID_PARAMS, e.to_string()),
            ),
        },
        proto::Call::SystemPairingPin => {
            let pin = service.store.name_and_pin_result();
            proto::Response::ok(Some(id), &pin)
        }
        proto::Call::SystemSetPairingPin(params) => {
            // The PIN itself is not logged. It is not much of a secret — a default one is
            // printed in this repo — but a per-robot one is meant to be, and a journal is the
            // wrong place for it.
            match service.store.set_pairing_pin(&params.pin) {
                Ok(pin) => {
                    tracing::info!("pairing PIN changed");
                    proto::Response::ok(
                        Some(id),
                        &proto::PairingPinResult {
                            is_default: pin == configd::store::DEFAULT_PIN,
                            pin,
                        },
                    )
                }
                Err(e) => proto::Response::err(
                    Some(id),
                    proto::Error::new(proto::code::INVALID_PARAMS, e.to_string()),
                ),
            }
        }
        proto::Call::PadStatus => match service.pads.status().await {
            Ok(pads) => proto::Response::ok(
                Some(id),
                &proto::PadStatusResult {
                    pads,
                    driver: driver::state().await,
                },
            ),
            Err(e) => {
                tracing::warn!(error = %e, "backend failed");
                proto::Response::err(Some(id), proto::Error::new(proto::code::INTERNAL_ERROR, e))
            }
        },
        proto::Call::PadPair(params) => {
            let timeout = pad::pair_timeout(params.timeout_seconds);
            tracing::info!(mac = ?params.mac, ?timeout, "pairing a gamepad");
            reply(id, service.pads.pair(params.mac.as_deref(), timeout).await)
        }
        proto::Call::PadForget(params) => reply(id, service.pads.forget(&params.mac).await),

        proto::Call::SystemReboot => {
            power::schedule();
            proto::Response::ok(
                Some(id),
                &proto::RebootResult {
                    in_seconds: power::REBOOT_DELAY.as_secs(),
                },
            )
        }

        // `update.*` is `updaterd`'s and `robot.*` is `robotd`'s. A client reaching here aimed at
        // the wrong socket, so say that rather than report a generic failure.
        other => proto::Response::err(
            Some(id),
            proto::Error::new(
                proto::code::METHOD_NOT_FOUND,
                format!("{} is not served by configd", other.method()),
            ),
        ),
    }
}

/// A backend result becomes a response. Backend failures are `INTERNAL_ERROR` because they mean
/// the machinery broke — a *refusal* is a successful call with a `Failed` outcome, which is a
/// distinction a client acts on.
fn reply<T: serde::Serialize>(id: proto::Id, result: Result<T, String>) -> proto::Response {
    match result {
        Ok(value) => proto::Response::ok(Some(id), &value),
        Err(e) => {
            tracing::warn!(error = %e, "backend failed");
            proto::Response::err(Some(id), proto::Error::new(proto::code::INTERNAL_ERROR, e))
        }
    }
}

/// Seconds since boot, from `/proc/uptime`. Zero where there is no procfs, which is only ever a
/// developer's laptop.
fn uptime_seconds() -> u64 {
    std::fs::read_to_string("/proc/uptime")
        .ok()
        .and_then(|s| s.split_whitespace().next()?.parse::<f64>().ok())
        .map(|secs| secs as u64)
        .unwrap_or(0)
}

async fn write_line<T: serde::Serialize>(
    out: &mut tokio::net::unix::OwnedWriteHalf,
    message: &T,
) -> std::io::Result<()> {
    let mut line = serde_json::to_vec(message)
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
    line.push(b'\n');
    out.write_all(&line).await?;
    out.flush().await
}

/// Resolve on SIGTERM (systemd stop) or SIGINT (Ctrl-C).
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
