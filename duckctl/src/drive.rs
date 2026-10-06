//! `duckctl drive` — the pad plugged into this machine drives the robot over wifi.
//!
//! Nothing new runs on the robot and nothing new is spoken on the wire: this is `padd`, the same
//! binary and mapping as on the robot, run here against `robotd`'s socket forwarded over SSH —
//! the bench setup `padd`'s crate docs describe, done in one step instead of four.
//!
//! 1. One SSH connection, shared by every step below (OpenSSH's control master), so the SSH
//!    password or key is asked for once.
//! 2. The robot's own `padd` is stopped (`sudo` on the robot), or two drivers would fight over the
//!    sticks the moment a pad paired to the robot connected.
//! 3. The robot's `robotd.toml` is copied here, so the buttons and walking speeds (`[pad]`,
//!    `[pad_drive]`) are the robot's, not this build's defaults.
//! 4. `robotd`'s socket is forwarded to a local path, and `padd` runs against it in the foreground.
//! 5. However `padd` ends — Ctrl-C, the pad, a dropped link — the robot's `padd` is started again,
//!    the connection closed and the scratch directory removed.
//!
//! SSH rather than the WebRTC control channel because this is a developer's tool and SSH is the
//! access a developer already has; the console's browser gamepad waits on TLS
//! (`webrtc-console.md` §1.3).

use std::path::PathBuf;
use std::process::{Command, ExitStatus, Stdio};

/// Run a whole drive session against `user@address`. Returns when `padd` has ended and the robot
/// has been put back.
pub fn run(user: &str, address: &str) -> Result<(), Box<dyn std::error::Error>> {
    let padd = find_padd();
    let session = Session::open(user, address)?;

    eprintln!("stopping the robot's own padd (sudo on the robot may ask for its password)…");
    let stopped = session.remote_tty(&["sudo", "systemctl", "stop", "padd"])?;
    if !stopped.success() {
        return Err(
            "could not stop padd on the robot — two drivers would fight over the sticks, so \
             not driving. Is this account allowed to sudo there?"
                .into(),
        );
    }
    // From here on the robot's padd is down, and `session`'s drop starts it again.
    let session = session.with_robot_padd_stopped();

    match session.fetch_config() {
        Ok(()) => eprintln!("using the robot's button bindings and walking speeds"),
        Err(e) => eprintln!("note: could not read the robot's config ({e}); using the defaults"),
    }
    session.forward()?;

    eprintln!("driving {address} from this machine's pad — Ctrl-C to stop and hand the robot back");
    ignore_interrupts();
    let status = Command::new(&padd)
        .arg("--socket")
        .arg(session.socket())
        .arg("--config")
        .arg(session.config())
        .arg("--tap-socket")
        .arg(session.tap())
        .status()
        .map_err(|e| {
            format!(
                "could not run {}: {e}\n`padd` has to be installed on this machine: `cargo install \
                 --path padd` from the repository, beside `duckctl`.",
                padd.display()
            )
        })?;
    restore_interrupts();
    if !status.success() {
        eprintln!("padd ended: {status}");
    }
    // `session` drops here: the robot's padd is started again and the connection closed.
    Ok(())
}

/// `padd` next to this binary — where `cargo install` puts both — else whatever `PATH` finds.
fn find_padd() -> PathBuf {
    std::env::current_exe()
        .ok()
        .and_then(|exe| exe.parent().map(|dir| dir.join("padd")))
        .filter(|candidate| candidate.is_file())
        .unwrap_or_else(|| PathBuf::from("padd"))
}

/// One SSH control connection and the scratch directory its socket lives in.
struct Session {
    target: String,
    dir: PathBuf,
    /// Whether the robot's padd was stopped, and so has to be started again.
    restart_padd: bool,
}

impl Session {
    /// Open the shared connection. This is where SSH asks for a password or a key's passphrase.
    fn open(user: &str, address: &str) -> Result<Self, Box<dyn std::error::Error>> {
        // Short: a Unix socket path is limited to ~100 bytes, and macOS's temp dir is long.
        let dir = std::env::temp_dir().join(format!("duckctl-drive-{}", std::process::id()));
        std::fs::create_dir_all(&dir)?;
        let session = Self {
            target: format!("{user}@{address}"),
            dir,
            restart_padd: false,
        };
        eprintln!("connecting to {}…", session.target);
        let status = Command::new("ssh")
            .args(session.control_args())
            .args([
                "-o",
                "ControlMaster=yes",
                "-o",
                "ControlPersist=yes",
                "-N",
                "-f",
            ])
            .arg(&session.target)
            .status()
            .map_err(|e| format!("could not run ssh: {e}"))?;
        if !status.success() {
            return Err(format!("could not connect to {}", session.target).into());
        }
        Ok(session)
    }

    fn with_robot_padd_stopped(mut self) -> Self {
        self.restart_padd = true;
        self
    }

    fn control(&self) -> PathBuf {
        self.dir.join("ssh")
    }

    fn socket(&self) -> PathBuf {
        self.dir.join("robotd.sock")
    }

    fn config(&self) -> PathBuf {
        self.dir.join("robotd.toml")
    }

    fn tap(&self) -> PathBuf {
        self.dir.join("pad.sock")
    }

    fn control_args(&self) -> Vec<String> {
        vec![
            "-o".to_owned(),
            format!("ControlPath={}", self.control().display()),
        ]
    }

    /// A command on the robot with a terminal, for `sudo`'s password prompt.
    fn remote_tty(&self, command: &[&str]) -> std::io::Result<ExitStatus> {
        Command::new("ssh")
            .args(self.control_args())
            .arg("-t")
            .arg(&self.target)
            .args(command)
            .status()
    }

    /// Copy the robot's `robotd.toml` here. A failure leaves `padd` on its defaults.
    fn fetch_config(&self) -> Result<(), String> {
        let output = Command::new("ssh")
            .args(self.control_args())
            .arg(&self.target)
            .args(["cat", robotd_params_path()])
            .stderr(Stdio::piped())
            .output()
            .map_err(|e| e.to_string())?;
        if !output.status.success() {
            return Err(String::from_utf8_lossy(&output.stderr).trim().to_owned());
        }
        std::fs::write(self.config(), output.stdout).map_err(|e| e.to_string())
    }

    /// Forward `robotd`'s socket to [`Self::socket`], over the shared connection.
    fn forward(&self) -> Result<(), Box<dyn std::error::Error>> {
        let status = Command::new("ssh")
            .args(self.control_args())
            .args(["-o", "StreamLocalBindUnlink=yes", "-O", "forward", "-L"])
            .arg(format!("{}:{}", self.socket().display(), ROBOTD_SOCKET))
            .arg(&self.target)
            .status()?;
        if !status.success() {
            return Err("could not forward robotd's socket over ssh".into());
        }
        Ok(())
    }
}

impl Drop for Session {
    fn drop(&mut self) {
        if self.restart_padd {
            eprintln!("starting the robot's own padd again…");
            match self.remote_tty(&["sudo", "systemctl", "start", "padd"]) {
                Ok(status) if status.success() => {}
                _ => eprintln!(
                    "could not start padd on the robot again — `duckctl ssh -- sudo systemctl \
                     start padd` does it, and a reboot does too"
                ),
            }
        }
        let _ = Command::new("ssh")
            .args(self.control_args())
            .args(["-O", "exit"])
            .arg(&self.target)
            .stderr(Stdio::null())
            .status();
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

/// Where `robotd` listens on the robot.
const ROBOTD_SOCKET: &str = "/run/robotd.sock";

/// Where the robot keeps its config — `robotd_params::DEFAULT_PATH`, which this crate does not
/// otherwise depend on.
fn robotd_params_path() -> &'static str {
    "/etc/robot/robotd.toml"
}

/// While `padd` runs in the foreground, Ctrl-C is for it: the terminal sends the interrupt to the
/// whole foreground group, and this process has to outlive it to hand the robot back.
fn ignore_interrupts() {
    #[cfg(unix)]
    // SAFETY: setting a signal's disposition to SIG_IGN installs no handler code.
    unsafe {
        libc::signal(libc::SIGINT, libc::SIG_IGN);
    }
}

fn restore_interrupts() {
    #[cfg(unix)]
    // SAFETY: as above, restoring the default disposition.
    unsafe {
        libc::signal(libc::SIGINT, libc::SIG_DFL);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The paths `padd` is handed all live in the session's directory, and are short enough to be
    /// Unix socket paths (~100 bytes) on a machine with a long temp dir.
    #[test]
    fn the_session_paths_fit_a_unix_socket() {
        let session = Session {
            target: "microduck@192.168.10.136".to_owned(),
            dir: std::env::temp_dir().join("duckctl-drive-4294967295"),
            restart_padd: false,
        };
        for path in [session.socket(), session.tap(), session.control()] {
            assert!(path.starts_with(&session.dir));
            assert!(
                path.as_os_str().len() < 100,
                "{} is too long for a Unix socket",
                path.display()
            );
        }
        // Dropping a session that never connected must not try to restart anything.
        std::mem::forget(session);
    }

    #[test]
    fn padd_falls_back_to_the_path() {
        let found = find_padd();
        assert!(found.ends_with("padd"));
    }
}
