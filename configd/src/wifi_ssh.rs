//! ssh on wifi for 30 minutes, opened by a short press of the face board's button.
//!
//! The shipping image answers ssh on the head's USB-C link only (microduck_yocto#110). Its
//! `microduck-ssh-access` package watches for [`SWITCH`] and, when it appears, installs a `/run`
//! drop-in ([`DROP_IN`]) that reopens port 22 on every interface. The image owns opening; this
//! module presses that switch, and closes it again when the time is up.
//!
//! **Why the button is enough.** The face button sits behind a hole that needs a pointy object:
//! deliberate physical access to the robot, which is the line `system.openWifiSsh` (v54) crossed
//! when it let a phone in Bluetooth range open it, and why that was withdrawn (v55).
//!
//! **The deadline is in the switch file, in `CLOCK_BOOTTIME` seconds.** Not in memory, so a
//! `configd` restart still closes on time; not wall time, because the board has no RTC and the
//! clock jumps when NTP syncs. Boot time is only comparable within one boot, and that is all it
//! needs: `/run` is a tmpfs, so a reboot drops the switch, the drop-in and the deadline together,
//! and the image comes up with ssh on USB only. An empty switch — `sudo touch` over USB — has no
//! deadline and stays open until reboot, as it always did. A press on an open robot (re)starts the
//! 30 minutes.
//!
//! Closing restarts `sshd.socket`, which stops new connections; sessions already open are their
//! own `sshd@` units and carry on.

use std::io;
use std::path::Path;
use std::time::Duration;

/// The file the image's path unit watches.
pub const SWITCH: &str = "/run/microduck-ssh-wifi";

/// What the image installs alongside the path unit. Its absence means an image with no switch to
/// press: a board with no USB link, or an image older than #110.
pub const INSTALLED: &str = "/usr/share/microduck-ssh-access/30-wifi-until-reboot.conf";

/// Where `microduck-ssh-wifi.service` copies [`INSTALLED`] to open ssh on every interface.
pub const DROP_IN: &str = "/run/systemd/system/sshd.socket.d/30-wifi-until-reboot.conf";

/// The face button: gpio-keys `face-keys`, `KEY_ENTER` (face-board-v1 device tree).
pub const BUTTON: &str = "/dev/input/by-path/platform-face-keys-event";

/// How long a press opens ssh on wifi for.
pub const OPEN_FOR: Duration = Duration::from_secs(30 * 60);

/// A press released within this is a press. Longer is a hold, which is the factory reset's
/// (10 s, in the OS image) and must not open anything on the way.
pub const SHORT_PRESS: Duration = Duration::from_secs(3);

/// How often the deadline is checked. A close up to this late is fine.
const CHECK_EVERY: Duration = Duration::from_secs(10);

/// Seconds since boot, suspend included.
pub fn boottime() -> u64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: a valid out-pointer; CLOCK_BOOTTIME exists on every kernel this runs on.
    unsafe { libc::clock_gettime(libc::CLOCK_BOOTTIME, &mut ts) };
    ts.tv_sec as u64
}

/// What the switch file says.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Closed,
    /// Open until reboot: the switch exists with no deadline in it.
    UntilReboot,
    /// Open until this boot time, in seconds.
    Until(u64),
}

/// Read the switch. Anything in it that is not a deadline is "until reboot", the meaning an empty
/// `touch` has always had.
pub fn state(switch: &Path) -> State {
    match std::fs::read_to_string(switch) {
        Err(_) => State::Closed,
        Ok(text) => text
            .trim()
            .strip_prefix("close-at-boottime ")
            .and_then(|s| s.parse().ok())
            .map_or(State::UntilReboot, State::Until),
    }
}

/// Open ssh on wifi until `now + OPEN_FOR`, if this image has the switch. `Ok(false)` when it
/// does not, and nothing is written: a stray file in `/run` that nothing watches helps no one.
///
/// Written to a temporary name and renamed, so the path unit never sees a half-written deadline —
/// and [`state`] never reads one as "until reboot".
pub fn open(switch: &Path, installed: &Path, now: u64) -> io::Result<bool> {
    if !installed.exists() {
        return Ok(false);
    }
    let until = now + OPEN_FOR.as_secs();
    let tmp = switch.with_extension("tmp");
    std::fs::write(&tmp, format!("close-at-boottime {until}\n"))?;
    std::fs::rename(&tmp, switch)?;
    Ok(true)
}

/// Is the time up? Only a deadline ever is.
pub fn due(state: State, now: u64) -> bool {
    matches!(state, State::Until(until) if now >= until)
}

/// Close ssh on wifi: the image's opening, undone.
///
/// The switch goes first: `microduck-ssh-wifi.path` is `PathExists=`, and stopping its service
/// while the file is still there would start it straight back up.
#[cfg(target_os = "linux")]
pub async fn close(switch: &Path, drop_in: &Path) -> Result<(), String> {
    for path in [switch, drop_in] {
        match std::fs::remove_file(path) {
            Ok(()) => {}
            Err(e) if e.kind() == io::ErrorKind::NotFound => {}
            Err(e) => return Err(format!("removing {}: {e}", path.display())),
        }
    }
    let bus = zbus::Connection::system()
        .await
        .map_err(|e| e.to_string())?;
    let manager = |method: &'static str, body: Option<&'static str>| {
        let bus = bus.clone();
        async move {
            let reply = match body {
                None => {
                    bus.call_method(
                        Some("org.freedesktop.systemd1"),
                        "/org/freedesktop/systemd1",
                        Some("org.freedesktop.systemd1.Manager"),
                        method,
                        &(),
                    )
                    .await
                }
                Some(unit) => {
                    bus.call_method(
                        Some("org.freedesktop.systemd1"),
                        "/org/freedesktop/systemd1",
                        Some("org.freedesktop.systemd1.Manager"),
                        method,
                        &(unit, "replace"),
                    )
                    .await
                }
            };
            reply
                .map(|_| ())
                .map_err(|e| format!("systemd {method} {}: {e}", body.unwrap_or("")))
        }
    };
    // Forget the drop-in, let the path unit fire again on the next press, and rebind the socket
    // to the USB link alone.
    manager("Reload", None).await?;
    manager("StopUnit", Some("microduck-ssh-wifi.service")).await?;
    manager("RestartUnit", Some("sshd.socket")).await?;
    Ok(())
}

#[cfg(not(target_os = "linux"))]
pub async fn close(_switch: &Path, _drop_in: &Path) -> Result<(), String> {
    Err("not closing ssh: this is not the robot".to_owned())
}

/// Watch the button and the deadline for as long as the daemon runs.
pub async fn run() {
    #[cfg(target_os = "linux")]
    std::thread::Builder::new()
        .name("face-button".into())
        .spawn(watch_button)
        .ok();

    let switch = Path::new(SWITCH);
    let mut ticker = tokio::time::interval(CHECK_EVERY);
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    loop {
        ticker.tick().await;
        if !due(state(switch), boottime()) {
            continue;
        }
        match close(switch, Path::new(DROP_IN)).await {
            Ok(()) => tracing::warn!("ssh on wifi closed: its 30 minutes are up"),
            Err(e) => tracing::error!(error = %e, "could not close ssh on wifi"),
        }
    }
}

/// Blocking: `fetch_events` waits on the device. A board without the button returns at once.
#[cfg(target_os = "linux")]
fn watch_button() {
    use evdev::{EventType, KeyCode};
    use std::time::Instant;

    if !Path::new(BUTTON).exists() {
        tracing::debug!("no face button on this board");
        return;
    }
    loop {
        let mut device = match evdev::Device::open(BUTTON) {
            Ok(device) => device,
            Err(e) => {
                tracing::warn!(error = %e, "cannot open the face button; retrying");
                std::thread::sleep(Duration::from_secs(5));
                continue;
            }
        };
        tracing::info!("watching the face button: a short press opens ssh on wifi for 30 min");
        let mut down: Option<Instant> = None;
        loop {
            let events = match device.fetch_events() {
                Ok(events) => events.collect::<Vec<_>>(),
                Err(e) => {
                    tracing::warn!(error = %e, "lost the face button; reopening");
                    std::thread::sleep(Duration::from_secs(5));
                    break;
                }
            };
            for event in events {
                if event.event_type() != EventType::KEY || event.code() != KeyCode::KEY_ENTER.0 {
                    continue;
                }
                match event.value() {
                    1 => down = Some(Instant::now()),
                    0 => {
                        let Some(at) = down.take() else { continue };
                        if at.elapsed() > SHORT_PRESS {
                            continue;
                        }
                        match open(Path::new(SWITCH), Path::new(INSTALLED), boottime()) {
                            // Warn: a root-capable login just became reachable from the network,
                            // and the journal is where anyone auditing that will look.
                            Ok(true) => tracing::warn!("face button: ssh open on wifi for 30 min"),
                            Ok(false) => {
                                tracing::info!(
                                    "face button pressed; this image has no wifi ssh switch"
                                )
                            }
                            Err(e) => tracing::error!(error = %e, "could not open ssh on wifi"),
                        }
                    }
                    _ => {}
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_press_opens_for_thirty_minutes_and_then_is_due() {
        let dir = tempfile::tempdir().unwrap();
        let (switch, installed) = (dir.path().join("switch"), dir.path().join("installed"));
        std::fs::write(&installed, "").unwrap();
        assert!(open(&switch, &installed, 100).unwrap());
        let until = 100 + OPEN_FOR.as_secs();
        assert_eq!(state(&switch), State::Until(until));
        assert!(!due(state(&switch), until - 1));
        assert!(due(state(&switch), until));
    }

    #[test]
    fn a_second_press_restarts_the_thirty_minutes() {
        let dir = tempfile::tempdir().unwrap();
        let (switch, installed) = (dir.path().join("switch"), dir.path().join("installed"));
        std::fs::write(&installed, "").unwrap();
        open(&switch, &installed, 100).unwrap();
        open(&switch, &installed, 1000).unwrap();
        assert_eq!(state(&switch), State::Until(1000 + OPEN_FOR.as_secs()));
    }

    /// `sudo touch` over USB means until reboot, and the deadline check must never close it.
    #[test]
    fn an_empty_switch_is_open_until_reboot() {
        let dir = tempfile::tempdir().unwrap();
        let switch = dir.path().join("switch");
        assert_eq!(state(&switch), State::Closed);
        std::fs::write(&switch, "").unwrap();
        assert_eq!(state(&switch), State::UntilReboot);
        assert!(!due(State::UntilReboot, u64::MAX));
        assert!(!due(State::Closed, u64::MAX));
    }

    #[test]
    fn an_image_without_the_switch_is_left_alone() {
        let dir = tempfile::tempdir().unwrap();
        let switch = dir.path().join("switch");
        assert!(!open(&switch, &dir.path().join("absent"), 100).unwrap());
        assert!(!switch.exists());
    }
}
