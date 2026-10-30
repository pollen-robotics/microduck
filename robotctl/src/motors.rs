//! `robotctl set-pwm-limit` and `robotctl set-current-limit` — two EEPROM registers on the
//! servos, written straight onto the bus with `rustypot`.
//!
//! Not a `robotd` call, because `robotd` has no business changing them: they are a property of
//! the robot's build, written once on a bench, and the daemon's startup check
//! (`duck_control::model::EXPECTED_REGISTERS`) deliberately leaves them alone so a value set here
//! survives every restart. Going round the daemon has three consequences, and each is checked or
//! done here rather than left for the operator to remember:
//!
//! - **`robotd` must be stopped.** It owns the bus, and two writers on one half-duplex line
//!   corrupt each other's packets. This takes the same instance lock `robotd` claims at start
//!   (`<socket>.lock`) and holds it until the servos are back, so a running daemon is refused
//!   and one that systemd restarts mid-write — which is what happens while a gamepad is
//!   connected — fails its own claim instead of joining in.
//! - **Torque must be off.** Both are EEPROM registers, and an XL330 ignores an EEPROM write
//!   while its torque is on: the write is acknowledged and nothing changes. So torque goes off
//!   first, which drops the robot.
//! - **The servos are rebooted afterwards**, so the new limit is the one they run with.
//!
//! Every value is read back before the reboot. A servo that silently kept its old value is the
//! failure mode the torque rule produces, and it is invisible unless someone looks.

use std::fs::{File, TryLockError};
use std::io::ErrorKind;
use std::path::Path;
use std::time::Duration;

use duck_ipc_proto as proto;
use rustypot::servo::dynamixel::xl330::Xl330Controller;

use super::{Failure, exit};

type Result<T> = std::result::Result<T, Failure>;

/// The bus speed `robotd` runs the servos at. `duck_control::model::BAUD_RATE` is the source;
/// it is restated rather than imported because linking `duck-control` would bring the ONNX
/// runtime into this binary.
const BAUD_RATE: u32 = 1_000_000;

/// Generous next to the daemon's 30 ms: this is one servo at a time, not a tick.
const READ_TIMEOUT: Duration = Duration::from_millis(100);

/// Same margins as `duck_control::bus`: an EEPROM cell may not be committed when the servo
/// acknowledges, and a rebooting servo is off the bus for a few hundred milliseconds.
const EEPROM_SETTLE: Duration = Duration::from_millis(20);
const REBOOT_SETTLE: Duration = Duration::from_millis(500);

/// XL330 `pwm_limit` (address 36) runs 0–885, where 885 is 100% of the supply.
const PWM_MAX: u16 = 885;

/// XL330 `current_limit` (address 38) runs 0–1750, one mA per count.
const CURRENT_MAX_MA: u16 = 1750;

#[derive(Debug, Clone, Copy)]
pub(super) enum Limit {
    /// Percent of the maximum PWM.
    Pwm(f64),
    /// Milliamps.
    Current(u16),
}

impl Limit {
    fn register(self) -> &'static str {
        match self {
            Limit::Pwm(_) => "pwm_limit",
            Limit::Current(_) => "current_limit",
        }
    }

    /// The register's raw value, or why there is none.
    fn raw(self) -> Result<u16> {
        match self {
            Limit::Pwm(percent) if (0.0..=100.0).contains(&percent) => {
                Ok((percent / 100.0 * f64::from(PWM_MAX)).round() as u16)
            }
            Limit::Pwm(percent) => Err(usage(format!(
                "a PWM limit is a percentage from 0 to 100, got {percent}"
            ))),
            Limit::Current(ma) if ma <= CURRENT_MAX_MA => Ok(ma),
            Limit::Current(ma) => Err(usage(format!(
                "an XL330's current limit is at most {CURRENT_MAX_MA} mA, got {ma}"
            ))),
        }
    }

    /// A raw register value the way the operator wrote it.
    fn show(self, raw: u16) -> String {
        match self {
            Limit::Pwm(_) => format!(
                "{:.1}% ({raw})",
                f64::from(raw) * 100.0 / f64::from(PWM_MAX)
            ),
            Limit::Current(_) => format!("{raw} mA"),
        }
    }
}

/// Write `limit` to the servos `ids` (every joint when empty), then reboot them.
pub(super) fn run(robot_socket: &Path, port: &str, limit: Limit, ids: Vec<u8>) -> Result<()> {
    let raw = limit.raw()?;
    let ids = if ids.is_empty() {
        proto::JOINT_IDS.to_vec()
    } else {
        if let Some(bad) = ids.iter().find(|&&id| proto::joint_of(id).is_none()) {
            return Err(usage(format!(
                "{bad} is not a joint id; the joints are {:?}",
                proto::JOINT_IDS
            )));
        }
        ids
    };

    // Held to the end of this function: the servos are back up before robotd may start.
    let _lock = claim_robotd_lock(robot_socket)?;
    let mut bus = open(port)?;

    // Torque off everywhere first, and before any read: a servo that is still holding the
    // robot up while its neighbours let go is worse than all of them letting go together. One
    // that does not answer is not written, but does not stop the others.
    let torque: Vec<std::result::Result<(), String>> = ids
        .iter()
        .map(|&id| {
            bus.write_torque_enable(id, false)
                .map_err(|e| format!("torque off: {e}"))
        })
        .collect();

    let register = limit.register();
    let mut failed = Vec::new();
    for (&id, torque) in ids.iter().zip(torque) {
        match torque.and_then(|()| write_one(&mut bus, limit, id, raw)) {
            Ok(before) => println!(
                "{:>16} ({id}): {register} {} -> {}",
                name(id),
                limit.show(before),
                limit.show(raw)
            ),
            Err(why) => {
                println!("{:>16} ({id}): {why}", name(id));
                failed.push(id);
            }
        }
    }

    // Every servo, including one whose write failed: torque went off on all of them, and a
    // reboot is what leaves each in the same state whatever happened to its neighbour.
    for &id in &ids {
        // The status packet is a courtesy the servo may not manage before it resets, so only a
        // failure to send counts — and `ping` below is what says whether it came back.
        if let Err(e) = bus.reboot(id) {
            eprintln!("warning: reboot {}: {e}", name(id));
        }
    }
    std::thread::sleep(REBOOT_SETTLE);
    let silent: Vec<u8> = ids
        .iter()
        .copied()
        .filter(|&id| !matches!(bus.ping(id), Ok(true)))
        .collect();

    if !silent.is_empty() {
        return Err(bus_error(format!(
            "rebooted, but {} did not answer afterwards — check power and cabling",
            names(&silent)
        )));
    }
    if !failed.is_empty() {
        return Err(bus_error(format!("{register} was not set on {}", names(&failed))));
    }
    println!(
        "{register} set to {} on {} servo(s), rebooted. Torque is off; `robot init` or Start \
         once robotd is running again.",
        limit.show(raw),
        ids.len()
    );
    Ok(())
}

/// Write one servo's limit and read it back. Returns the value it had before.
fn write_one(
    bus: &mut Xl330Controller,
    limit: Limit,
    id: u8,
    raw: u16,
) -> std::result::Result<u16, String> {
    let read = |bus: &mut Xl330Controller| -> std::result::Result<u16, String> {
        let values = match limit {
            Limit::Pwm(_) => bus.read_pwm_limit(id),
            Limit::Current(_) => bus.read_current_limit(id),
        }
        .map_err(|e| format!("read: {e}"))?;
        values
            .first()
            .copied()
            .ok_or_else(|| "read: no answer".to_owned())
    };

    let before = read(bus)?;
    let written = match limit {
        Limit::Pwm(_) => bus.write_pwm_limit(id, raw),
        Limit::Current(_) => bus.write_current_limit(id, raw),
    };
    written.map_err(|e| format!("write: {e}"))?;
    std::thread::sleep(EEPROM_SETTLE);

    let after = read(bus)?;
    if after != raw {
        // The torque rule, almost always: the servo acknowledged and kept its value.
        return Err(format!(
            "wrote {}, reads back {} — is its torque still on?",
            limit.show(raw),
            limit.show(after)
        ));
    }
    Ok(before)
}

/// Take `robotd`'s instance lock, or say why it cannot be had.
///
/// The same file and the same `flock` as `robotd`'s `claim_lock`. Opened read-only and never
/// created: a lock is all this wants from it, and a robot where `robotd` has never run has no
/// daemon to keep out.
fn claim_robotd_lock(socket: &Path) -> Result<Option<File>> {
    let mut path = socket.as_os_str().to_os_string();
    path.push(".lock");
    let file = match File::open(&path) {
        Ok(file) => file,
        Err(e) if e.kind() == ErrorKind::NotFound => return Ok(None),
        Err(e) => {
            return Err(Failure::new(
                exit::FAILED,
                format!("opening {}: {e}", Path::new(&path).display()),
            ));
        }
    };
    match file.try_lock() {
        Ok(()) => Ok(Some(file)),
        Err(TryLockError::WouldBlock) => Err(Failure::new(
            exit::BUSY,
            "robotd is running and owns the motor bus. Stop it first — `sudo systemctl stop \
             robotd` — with the gamepad disconnected, or it is started again straight away."
                .to_owned(),
        )),
        Err(TryLockError::Error(e)) => Err(Failure::new(
            exit::FAILED,
            format!("locking {}: {e}", Path::new(&path).display()),
        )),
    }
}

fn open(port: &str) -> Result<Xl330Controller> {
    let serial = serialport::new(port, BAUD_RATE)
        .timeout(READ_TIMEOUT)
        .open()
        .map_err(|e| {
            let code = match e.kind() {
                serialport::ErrorKind::Io(ErrorKind::PermissionDenied) => exit::DENIED,
                _ => exit::FAILED,
            };
            Failure::new(code, format!("opening {port}: {e}"))
        })?;
    Ok(Xl330Controller::new().with_protocol_v2().with_serial_port(serial))
}

fn name(id: u8) -> &'static str {
    proto::joint_of(id).unwrap_or("?")
}

fn names(ids: &[u8]) -> String {
    ids.iter()
        .map(|&id| format!("{} ({id})", name(id)))
        .collect::<Vec<_>>()
        .join(", ")
}

fn usage(message: String) -> Failure {
    Failure::new(exit::USAGE, message)
}

fn bus_error(message: String) -> Failure {
    Failure::new(exit::FAILED, message)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pwm_percent_maps_onto_the_register_range() {
        assert_eq!(Limit::Pwm(100.0).raw().unwrap(), PWM_MAX);
        assert_eq!(Limit::Pwm(50.0).raw().unwrap(), 443);
        assert_eq!(Limit::Pwm(0.0).raw().unwrap(), 0);
        assert!(Limit::Pwm(100.5).raw().is_err());
        assert!(Limit::Pwm(-1.0).raw().is_err());
    }

    #[test]
    fn current_is_milliamps_up_to_the_xl330_maximum() {
        assert_eq!(Limit::Current(800).raw().unwrap(), 800);
        assert_eq!(Limit::Current(CURRENT_MAX_MA).raw().unwrap(), CURRENT_MAX_MA);
        assert!(Limit::Current(CURRENT_MAX_MA + 1).raw().is_err());
    }

    #[test]
    fn no_lock_file_means_no_daemon_to_keep_out() {
        let dir = tempfile::tempdir().unwrap();
        let lock = claim_robotd_lock(&dir.path().join("robotd.sock")).unwrap();
        assert!(lock.is_none());
    }

    #[test]
    fn a_held_lock_is_refused_as_busy() {
        let dir = tempfile::tempdir().unwrap();
        let socket = dir.path().join("robotd.sock");
        let held = File::create(dir.path().join("robotd.sock.lock")).unwrap();
        held.lock().unwrap();
        let err = claim_robotd_lock(&socket).unwrap_err();
        assert_eq!(err.code, exit::BUSY);
    }
}
