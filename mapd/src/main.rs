//! `mapd`: the onboard map, and where the robot is in it.
//!
//! Subscribes to `robot.state` and `tof.frame`, feeds them to the `maploc` mapper on its own
//! thread, keeps the map in a file across reboots, and answers `map.*` on its socket. The design
//! lives in `docs/design/mapping-design.md`.

mod feed;
mod server;
mod store;
mod worker;

use std::os::unix::fs::PermissionsExt;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::{Context, Result};
use clap::Parser;
use duck_ipc_proto as proto;
use maploc::mapper::MapperConfig;
use tokio::net::UnixListener;

/// `robot` may connect, as for every other socket a client of the robot reads.
const SOCKET_MODE: u32 = 0o660;
const GROUP: &str = "robot";
/// How many inputs may wait for a busy worker: ~5 s of both streams.
const INPUT_QUEUE: usize = 256;

#[derive(Parser, Debug, Clone)]
#[command(about = "The onboard map: where the robot is in its home")]
struct Args {
    /// `robotd.toml`, for `[map]`.
    #[arg(long)]
    config: Option<PathBuf>,
    #[arg(long, default_value = proto::socket::MAP)]
    socket: PathBuf,
    #[arg(long, default_value = proto::socket::ROBOT)]
    robot_socket: PathBuf,
    #[arg(long, default_value = proto::socket::TOF)]
    tof_socket: PathBuf,
    /// Keep the map here instead of `[map] path`.
    #[arg(long)]
    map: Option<PathBuf>,
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> std::process::ExitCode {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .init();
    duck_ipc_proto::log_startup_identity!("mapd");
    match run(Args::parse()).await {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(e) => {
            tracing::error!(error = format!("{e:#}"), "mapd stopped");
            std::process::ExitCode::FAILURE
        }
    }
}

async fn run(args: Args) -> Result<()> {
    let config_path = args
        .config
        .clone()
        .unwrap_or_else(|| PathBuf::from(robotd_params::DEFAULT_PATH));
    // A file this daemon cannot read is not a reason to stop mapping — `robotd` is the one that
    // refuses a broken params file, loudly; here the defaults carry on and the journal says so.
    let params = match robotd_params::Params::load(&config_path, args.config.is_some()) {
        Ok(p) => p,
        Err(e) => {
            tracing::warn!(error = %e, path = %config_path.display(), "unusable params file; carrying on with the built-in defaults");
            robotd_params::Params::default()
        }
    }
    .map;

    let shared = Arc::new(Mutex::new(worker::Shared::default()));
    // The running worker, once there is one: the server asks it, and the shutdown saves through it.
    let mapping: Arc<Mutex<Option<Mapping>>> = Arc::new(Mutex::new(None));
    let worker: Arc<Mutex<Option<std::sync::mpsc::SyncSender<worker::Input>>>> =
        Arc::new(Mutex::new(None));
    if params.enabled {
        let started = start_mapping(&params, &args, &shared)?;
        *worker.lock().unwrap_or_else(|p| p.into_inner()) = Some(started.tx.clone());
        *mapping.lock().unwrap_or_else(|p| p.into_inner()) = Some(started);
    } else {
        tracing::warn!(
            "mapping is off ([map] enabled = false) — it starts when the duck on its own does; \
             answering map.status until then"
        );
        if let Ok(mut s) = shared.lock() {
            s.status.unavailable = Some(OFF.to_owned());
        }
        // The duck on its own maps: a robot finding its way around is what the map is for. It can
        // be switched on at any time (an NFC tag), so it is watched for rather than read once.
        let (args, mapping, worker, shared) = (
            args.clone(),
            mapping.clone(),
            worker.clone(),
            shared.clone(),
        );
        std::thread::Builder::new()
            .name("autonomy-watch".to_owned())
            .spawn(move || {
                wait_for_autonomy(&args.robot_socket);
                match start_mapping(&params, &args, &shared) {
                    Ok(started) => {
                        tracing::warn!("the duck is on its own — mapping");
                        if let Ok(mut s) = shared.lock() {
                            s.status.unavailable = None;
                        }
                        *worker.lock().unwrap_or_else(|p| p.into_inner()) =
                            Some(started.tx.clone());
                        *mapping.lock().unwrap_or_else(|p| p.into_inner()) = Some(started);
                    }
                    Err(e) => tracing::error!(error = format!("{e:#}"), "cannot start mapping"),
                }
            })
            .context("spawning the autonomy watch")?;
    }

    let listener = bind(&args.socket)?;
    let ctx = server::Ctx { shared, worker };
    let mut term = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())?;
    let mut int = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::interrupt())?;
    loop {
        tokio::select! {
            accepted = listener.accept() => match accepted {
                Ok((stream, _)) => {
                    let ctx = ctx.clone();
                    tokio::spawn(async move {
                        if let Err(e) = server::connection(stream, ctx).await {
                            tracing::debug!(error = %e, "connection ended");
                        }
                    });
                }
                Err(e) => {
                    tracing::warn!(error = %e, "accept failed; retrying in one second");
                    tokio::time::sleep(Duration::from_secs(1)).await;
                }
            },
            _ = term.recv() => break,
            _ = int.recv() => break,
        }
    }

    // Save on the way out: what the map learned since the last autosave, and the pose the boot
    // hint will start from.
    let running = mapping.lock().unwrap_or_else(|p| p.into_inner()).take();
    if let Some(Mapping { tx, thread }) = running {
        tracing::info!("stopping; saving the map");
        let saved = tokio::task::spawn_blocking(move || {
            let (reply, done) = std::sync::mpsc::sync_channel(1);
            if tx.send(worker::Input::Shutdown(reply)).is_ok() {
                let _ = done.recv_timeout(Duration::from_secs(8));
            }
            thread.is_finished()
        })
        .await
        .unwrap_or(false);
        if !saved {
            tracing::warn!("the map worker did not finish saving in time");
        }
    }
    Ok(())
}

/// Why `map.*` has nothing to say, while mapping is off.
const OFF: &str =
    "mapping is off ([map] enabled = false in robotd.toml, and the duck on its own is off)";

/// How often an idle `mapd` asks whether the duck on its own has been switched on.
const AUTONOMY_POLL: Duration = Duration::from_secs(2);

/// The mapper's thread and the way to reach it.
struct Mapping {
    tx: std::sync::mpsc::SyncSender<worker::Input>,
    thread: std::thread::JoinHandle<()>,
}

/// Start the worker and both feeds.
fn start_mapping(
    params: &robotd_params::MapParams,
    args: &Args,
    shared: &Arc<Mutex<worker::Shared>>,
) -> Result<Mapping> {
    let path = args.map.clone().unwrap_or(params.path.clone());
    let cfg = MapperConfig {
        tof_latency_ms: params.tof_latency_ms,
        ..MapperConfig::default()
    };
    let (tx, rx) = std::sync::mpsc::sync_channel(INPUT_QUEUE);
    let w = worker::Worker::new(
        cfg,
        path,
        Duration::from_secs(params.autosave_s.max(5)),
        shared.clone(),
    );
    let thread = std::thread::Builder::new()
        .name("map-worker".to_owned())
        .spawn(move || w.run(rx))
        .context("spawning the map worker")?;
    feed::spawn(
        feed::Source::Robot {
            hz: params.state_hz.clamp(5, 50),
        },
        args.robot_socket.clone(),
        tx.clone(),
        shared.clone(),
    );
    feed::spawn(
        feed::Source::Tof,
        args.tof_socket.clone(),
        tx.clone(),
        shared.clone(),
    );
    Ok(Mapping { tx, thread })
}

/// Block until `robotd` says the duck on its own is on. A `robotd` that is down, or too old to
/// say, is asked again: either can change under an update.
fn wait_for_autonomy(socket: &Path) {
    loop {
        if autonomy_on(socket).unwrap_or(false) {
            return;
        }
        std::thread::sleep(AUTONOMY_POLL);
    }
}

fn autonomy_on(socket: &Path) -> std::io::Result<bool> {
    use std::io::{BufRead, BufReader, Write};
    let mut stream = std::os::unix::net::UnixStream::connect(socket)?;
    stream.set_read_timeout(Some(Duration::from_secs(2)))?;
    let mut line = serde_json::to_vec(&proto::Request::call(
        proto::Id::Number(1),
        &proto::Call::RobotMode,
    ))?;
    line.push(b'\n');
    stream.write_all(&line)?;
    let mut answer = String::new();
    BufReader::new(stream).read_line(&mut answer)?;
    Ok(serde_json::from_str::<proto::Response>(&answer)
        .ok()
        .and_then(|r| r.result_as::<proto::ModeResult>().ok())
        .is_some_and(|m| m.autonomous))
}

fn bind(socket: &Path) -> Result<UnixListener> {
    if let Some(parent) = socket.parent() {
        // `RuntimeDirectory=mapd` made this on a board; tried anyway for a `mapd` run by hand.
        let _ = std::fs::create_dir_all(parent);
    }
    // systemd removes the runtime directory when the unit stops, so a leftover socket is from a
    // `mapd` killed outside its unit.
    if socket.exists() {
        let _ = std::fs::remove_file(socket);
    }
    let listener =
        UnixListener::bind(socket).with_context(|| format!("binding {}", socket.display()))?;
    std::fs::set_permissions(socket, std::fs::Permissions::from_mode(SOCKET_MODE))?;
    if let Err(e) = give_to_group(socket, GROUP) {
        tracing::warn!(error = %e, group = GROUP, socket = %socket.display(), "the map stays private to mapd — nothing else can read it");
    }
    tracing::info!(path = %socket.display(), "serving map.*");
    Ok(listener)
}

/// Make the saved map readable by `GROUP`: the same people the socket serves, and the ones who
/// copy the file off a robot to replay why it stayed lost.
pub fn share_with_robot_group(path: &Path) -> std::io::Result<()> {
    std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o640))?;
    give_to_group(path, GROUP)
}

/// Hand the socket to `GROUP`, so the people who may watch `robot.state` may read the map. The
/// same as `tofd`'s, including that a missing group is a warning: a laptop has none.
fn give_to_group(socket: &Path, group: &str) -> std::io::Result<()> {
    use std::ffi::CString;
    use std::os::unix::ffi::OsStrExt;

    let name = CString::new(group).map_err(std::io::Error::other)?;
    // SAFETY: `getgrnam` reads the group database and returns a pointer into storage it owns; the
    // name is a valid C string for the length of the call.
    let entry = unsafe { libc::getgrnam(name.as_ptr()) };
    if entry.is_null() {
        return Err(std::io::Error::other(format!(
            "no {group} group on this system"
        )));
    }
    // SAFETY: checked non-null above; `getgrnam` fills the struct when it returns a pointer.
    let gid = unsafe { (*entry).gr_gid };
    let path = CString::new(socket.as_os_str().as_bytes()).map_err(std::io::Error::other)?;
    // SAFETY: a valid C string path; `-1` for the owner leaves it alone.
    if unsafe { libc::chown(path.as_ptr(), u32::MAX, gid) } != 0 {
        return Err(std::io::Error::last_os_error());
    }
    Ok(())
}
