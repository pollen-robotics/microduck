//! Startup ownership against the actual binary, with fake I/O and private sockets.
use std::io::Read;
use std::os::unix::fs::MetadataExt;
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::Path;
use std::process::{Child, Command, ExitStatus, Stdio};
use std::time::{Duration, Instant};

use updater::robot::{Health, RobotClient, SocketRobotClient};

struct Robotd(Child);

impl Robotd {
    fn spawn(socket: &Path, extra: &[&str]) -> Self {
        // An explicit empty config keeps host /etc/robot settings out of the test.
        let params = socket.parent().unwrap().join("params.toml");
        std::fs::write(&params, "").unwrap();
        Self(
            Command::new(env!("CARGO_BIN_EXE_robotd"))
                .arg("--socket")
                .arg(socket)
                .arg("--params")
                .arg(params)
                .args(["--fake", "--no-policy"])
                .args(extra)
                .env("RUST_LOG", "warn")
                .stdout(Stdio::null())
                .stderr(Stdio::piped())
                .spawn()
                .expect("spawn robotd"),
        )
    }

    async fn wait_for_exit(&mut self) -> (ExitStatus, String) {
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            if let Some(status) = self.0.try_wait().unwrap() {
                let mut log = String::new();
                self.0
                    .stderr
                    .take()
                    .unwrap()
                    .read_to_string(&mut log)
                    .unwrap();
                return (status, log);
            }
            assert!(Instant::now() < deadline, "robotd did not exit");
            tokio::time::sleep(Duration::from_millis(25)).await;
        }
    }

    async fn assert_refused(&mut self) {
        let (status, log) = self.wait_for_exit().await;
        assert_eq!(status.code(), Some(1), "{status}: {log}");
        assert!(log.contains("cannot claim robot IPC socket"), "{log}");
        assert!(
            !log.contains("starting service="),
            "a refused startup must not publish the loser's identity: {log}"
        );
        assert!(
            !log.contains("--fake: no bus, no robot"),
            "a refused startup must not start the control loop: {log}"
        );
        assert!(
            !log.contains("cannot open the bus"),
            "a refused init must not try to open the bus: {log}"
        );
    }

    fn kill(&mut self) {
        self.0.kill().unwrap();
        self.0.wait().unwrap();
    }
}

impl Drop for Robotd {
    fn drop(&mut self) {
        // A failed assertion must not leave a daemon behind, either.
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

async fn wait_until_healthy(socket: &Path) {
    let client = SocketRobotClient::new(socket.to_owned());
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        let health = client.health(Duration::from_millis(500)).await;
        if matches!(health, Health::Healthy) {
            return;
        }
        assert!(
            Instant::now() < deadline,
            "robotd never became healthy: {health:?}"
        );
        tokio::time::sleep(Duration::from_millis(25)).await;
    }
}

/// The second process used to unlink the first one's listener and take all new clients,
/// while both control loops kept running. Its forced unhealthy response identifies a takeover.
#[tokio::test]
async fn a_second_daemon_cannot_replace_the_running_service() {
    let dir = tempfile::tempdir().unwrap();
    let socket = dir.path().join("robotd.sock");
    let mut first = Robotd::spawn(&socket, &[]);
    wait_until_healthy(&socket).await;
    let inode = std::fs::metadata(&socket).unwrap().ino();

    let mut second = Robotd::spawn(&socket, &["--unhealthy"]);
    second.assert_refused().await;

    assert!(first.0.try_wait().unwrap().is_none());
    assert_eq!(std::fs::metadata(&socket).unwrap().ino(), inode);
    wait_until_healthy(&socket).await;
}

/// `init` used to bypass the daemon's lock and open the motor bus itself. A missing,
/// private port makes reaching run_init observable without ever opening real hardware.
#[tokio::test]
async fn init_cannot_open_the_bus_while_a_daemon_is_running() {
    let dir = tempfile::tempdir().unwrap();
    let socket = dir.path().join("robotd.sock");
    let port = dir.path().join("missing-bus");
    let mut daemon = Robotd::spawn(&socket, &[]);
    wait_until_healthy(&socket).await;
    let inode = std::fs::metadata(&socket).unwrap().ino();

    let mut init = Robotd::spawn(&socket, &["--port", port.to_str().unwrap(), "init"]);
    init.assert_refused().await;

    assert!(daemon.0.try_wait().unwrap().is_none());
    assert_eq!(std::fs::metadata(&socket).unwrap().ino(), inode);
    wait_until_healthy(&socket).await;
}

/// A failed init must release ownership just like a successful ramp: the operator may
/// need to start the daemon to diagnose the bus. It must not create an IPC listener.
#[tokio::test]
async fn a_failed_init_releases_the_lock_without_creating_a_socket() {
    let dir = tempfile::tempdir().unwrap();
    let socket = dir.path().join("robotd.sock");
    let port = dir.path().join("missing-bus");
    let lock_path = dir.path().join("robotd.sock.lock");
    let mut init = Robotd::spawn(&socket, &["--port", port.to_str().unwrap(), "init"]);
    let (status, log) = init.wait_for_exit().await;
    assert_eq!(status.code(), Some(1), "{status}: {log}");
    #[cfg(target_os = "linux")]
    assert!(log.contains("cannot open the bus"), "{log}");
    #[cfg(not(target_os = "linux"))]
    assert!(log.contains("init needs a real bus"), "{log}");
    assert!(!socket.exists(), "init must not bind a listener");
    let inode = std::fs::metadata(&lock_path).unwrap().ino();

    let _daemon = Robotd::spawn(&socket, &[]);
    wait_until_healthy(&socket).await;
    assert_eq!(std::fs::metadata(&lock_path).unwrap().ino(), inode);
}

/// Both launches start together, before either has answered a client. Repeating the race
/// exercises both possible winners; waiting for the first daemon to be ready would miss it.
#[tokio::test]
async fn simultaneous_starts_leave_exactly_one_daemon_running() {
    for _ in 0..10 {
        let dir = tempfile::tempdir().unwrap();
        let socket = dir.path().join("robotd.sock");
        let gate = std::sync::Barrier::new(2);
        let (mut first, mut second) = std::thread::scope(|scope| {
            let start = || {
                gate.wait();
                Robotd::spawn(&socket, &[])
            };
            let first = scope.spawn(start);
            let second = scope.spawn(start);
            (first.join().unwrap(), second.join().unwrap())
        });

        let deadline = Instant::now() + Duration::from_secs(10);
        let winner = loop {
            match (first.0.try_wait().unwrap(), second.0.try_wait().unwrap()) {
                (Some(_), None) => {
                    first.assert_refused().await;
                    break &mut second;
                }
                (None, Some(_)) => {
                    second.assert_refused().await;
                    break &mut first;
                }
                (Some(a), Some(b)) => panic!("both daemons exited: {a}, {b}"),
                (None, None) => (),
            }
            assert!(Instant::now() < deadline, "both daemons kept running");
            tokio::time::sleep(Duration::from_millis(25)).await;
        };
        wait_until_healthy(&socket).await;
        assert!(winner.0.try_wait().unwrap().is_none());
    }
}

/// A PID file or an unconditionally exclusive create would survive SIGKILL and prevent
/// recovery. The kernel lock must release, and the abandoned socket must be replaceable.
#[tokio::test]
async fn a_killed_daemon_can_restart_using_the_same_lock_file() {
    let dir = tempfile::tempdir().unwrap();
    let socket = dir.path().join("robotd.sock");
    let lock_path = dir.path().join("robotd.sock.lock");
    let mut first = Robotd::spawn(&socket, &[]);
    wait_until_healthy(&socket).await;
    let lock_inode = std::fs::metadata(&lock_path).unwrap().ino();
    first.kill();
    assert!(socket.exists(), "SIGKILL leaves a stale socket");

    let _restarted = Robotd::spawn(&socket, &[]);
    wait_until_healthy(&socket).await;
    assert_eq!(std::fs::metadata(lock_path).unwrap().ino(), lock_inode);
    let mut duplicate = Robotd::spawn(&socket, &["--unhealthy"]);
    duplicate.assert_refused().await;
    wait_until_healthy(&socket).await;
}

/// Graceful shutdown must remove the socket but keep the lock inode reusable, so a
/// normal service restart cannot split contenders across two different lock files.
#[tokio::test]
async fn a_clean_shutdown_can_restart_using_the_same_lock_file() {
    let dir = tempfile::tempdir().unwrap();
    let socket = dir.path().join("robotd.sock");
    let lock_path = dir.path().join("robotd.sock.lock");
    let mut first = Robotd::spawn(&socket, &[]);
    wait_until_healthy(&socket).await;
    let inode = std::fs::metadata(&lock_path).unwrap().ino();
    assert_eq!(unsafe { libc::kill(first.0.id() as i32, libc::SIGTERM) }, 0);
    let (status, log) = first.wait_for_exit().await;
    assert!(status.success(), "{status}: {log}");
    assert!(!socket.exists());

    let _restarted = Robotd::spawn(&socket, &[]);
    wait_until_healthy(&socket).await;
    assert_eq!(std::fs::metadata(lock_path).unwrap().ino(), inode);
}

/// An older robotd does not hold our new lock. A listening socket is still owned and
/// must not be mistaken for the file a killed process left behind.
#[tokio::test]
async fn a_live_listener_without_a_lock_is_preserved() {
    let dir = tempfile::tempdir().unwrap();
    let socket = dir.path().join("robotd.sock");
    let _listener = UnixListener::bind(&socket).unwrap();
    let inode = std::fs::metadata(&socket).unwrap().ino();

    let mut daemon = Robotd::spawn(&socket, &[]);
    daemon.assert_refused().await;
    assert_eq!(std::fs::metadata(&socket).unwrap().ino(), inode);
    UnixStream::connect(&socket).expect("original listener remains reachable");
}

/// Linux AF_UNIX can refuse a connection with EAGAIN when the accept queue is full.
/// A busy listener must be preserved just like one that answers the liveness probe.
#[cfg(target_os = "linux")]
#[tokio::test]
async fn a_live_listener_with_a_full_accept_queue_is_preserved() {
    use std::io::ErrorKind;
    use std::os::fd::AsRawFd;

    let dir = tempfile::tempdir().unwrap();
    let socket = dir.path().join("robotd.sock");
    let listener = UnixListener::bind(&socket).unwrap();
    // Shrink the queue on this live listener so the test needs only a few connections.
    assert_eq!(unsafe { libc::listen(listener.as_raw_fd(), 1) }, 0);
    let inode = std::fs::metadata(&socket).unwrap().ino();
    let mut queued = Vec::new();
    let mut full = false;
    for _ in 0..8 {
        match tokio::time::timeout(
            Duration::from_secs(1),
            tokio::net::UnixStream::connect(&socket),
        )
        .await
        {
            Ok(Ok(stream)) => queued.push(stream),
            Ok(Err(e)) if e.kind() == ErrorKind::WouldBlock => {
                full = true;
                break;
            }
            Err(_) => {
                full = true;
                break;
            }
            Ok(Err(e)) => panic!("cannot fill the accept queue: {e}"),
        }
    }
    assert!(full && !queued.is_empty(), "accept queue was not saturated");

    let mut daemon = Robotd::spawn(&socket, &[]);
    daemon.assert_refused().await;
    assert_eq!(std::fs::metadata(&socket).unwrap().ino(), inode);
    listener.set_nonblocking(true).unwrap();
    while let Ok((stream, _)) = listener.accept() {
        drop(stream);
    }
    UnixStream::connect(&socket).expect("original listener still accepts new connections");
}

/// The lock must exclude a second launch even before the first has bound its socket.
/// Checking only whether a listener answers leaves that startup window unprotected.
#[tokio::test]
async fn a_held_lock_refuses_startup_before_a_socket_exists() {
    let dir = tempfile::tempdir().unwrap();
    let socket = dir.path().join("robotd.sock");
    let lock = std::fs::File::create(dir.path().join("robotd.sock.lock")).unwrap();
    lock.try_lock().unwrap();

    let port = dir.path().join("missing-bus");
    // `init` has no listener at all, so both entry points must respect the lock alone.
    for command in [vec![], vec!["--port", port.to_str().unwrap(), "init"]] {
        let mut process = Robotd::spawn(&socket, &command);
        process.assert_refused().await;
    }
    assert!(!socket.exists());
}

/// A path typo is not permission to delete a regular file or a dangling symlink.
#[tokio::test]
async fn non_socket_paths_are_not_removed() {
    let dir = tempfile::tempdir().unwrap();
    let socket = dir.path().join("robotd.sock");
    std::fs::write(&socket, "keep this file").unwrap();
    let mut daemon = Robotd::spawn(&socket, &[]);
    daemon.assert_refused().await;
    assert_eq!(std::fs::read_to_string(&socket).unwrap(), "keep this file");

    std::fs::remove_file(&socket).unwrap();
    let target = dir.path().join("missing");
    std::os::unix::fs::symlink(&target, &socket).unwrap();
    let mut daemon = Robotd::spawn(&socket, &[]);
    daemon.assert_refused().await;
    assert_eq!(std::fs::read_link(&socket).unwrap(), target);
}
