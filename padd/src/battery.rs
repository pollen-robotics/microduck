//! The connected pad's battery, asked of the pad and written where `configd` reads it.
//!
//! ## Why `padd` asks
//!
//! `pad.status` is `configd`'s, but `configd` cannot reach the pad: it runs with
//! `PrivateDevices=yes`, so its `/dev` is made once when it starts, and a pad's hidraw node appears
//! when the pad connects. `padd` already follows the pad from connect to disconnect, so it asks,
//! and writes what it heard to [`proto::pad_battery::FILE`], which `configd` and `robotctl monitor`
//! read while it is fresh. The file is the whole interface; neither side waits on the other.
//!
//! ## How a Switch pad is asked
//!
//! The board's kernel has no `hid-nintendo`, so a Switch pad is driven by `hid-generic` in its
//! simple HID mode, and those reports carry no battery. But **every subcommand reply does**: byte 2
//! of an `0x21` report is the battery, whatever was asked. So this sends one output report — rumble
//! neutral, subcommand `0x50`, "get the regulated voltage", which changes nothing on the pad — on
//! the pad's hidraw node, and reads the reply. A genuine pad answers the voltage too; the clones
//! seen so far acknowledge with no data, which is why only the battery byte is used. The pad stays
//! in the mode it was in: the reports `padd` drives from carry on as before.
//!
//! Its own open of the hidraw node gets its own queue of reports, so reading the reply takes
//! nothing from the input device the driving reads.
//!
//! ## When the kernel knows
//!
//! A kernel with `hid-nintendo` owns the pad's subcommands — writing them from here would cross its
//! own — and publishes the battery under the HID device's `power_supply/`. Then that is read, and
//! nothing is sent. That driver's level names are one step off the pad's own (its "high" is the pad's
//! 6 of 8), so they are mapped onto [`proto::PadBatteryLevel`], which uses the pad's.
//!
//! ## Which pads
//!
//! Nintendo's vendor id (`057E`), over Bluetooth or USB: the one family whose battery is known to be
//! readable this way. Any other pad has no file, and `pad.status` shows no battery for it — which is
//! "unknown", not "empty".

use std::io::{ErrorKind, Read, Write};
use std::os::unix::fs::OpenOptionsExt;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Condvar, Mutex, PoisonError};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use duck_ipc_proto as proto;
use gilrs::LinuxGamepadExt;

/// How often the pad is asked. A battery level moves over tens of minutes, and each ask is one
/// report each way on the radio the driving shares.
const EVERY: Duration = Duration::from_secs(60);

/// How long after a pad connects to ask it first: long enough for the link to settle and for
/// BlueZ to have made the hidraw node, short enough that someone opening the app sees it.
const SETTLE: Duration = Duration::from_secs(3);

/// How long to wait for the reply. A pad answers in a few milliseconds over a healthy link.
const REPLY_WITHIN: Duration = Duration::from_secs(1);

/// The vendor id in a HID device's sysfs name, `0005:057E:2009.0001`.
const NINTENDO: &str = "057E";

/// An output report asking for subcommand `0x50`: report id `0x01`, the packet counter, eight bytes
/// of neutral rumble (the "no rumble" pattern both sides of a Switch pad expect), the subcommand.
pub fn ask_report(counter: u8) -> [u8; 11] {
    [
        0x01,
        counter & 0x0f,
        0x00,
        0x01,
        0x40,
        0x40,
        0x00,
        0x01,
        0x40,
        0x40,
        0x50,
    ]
}

/// The battery in a report the pad sent, if it is one that carries it: a subcommand reply (`0x21`)
/// or a full-mode report (`0x30`). The simple-mode `0x3F` report does not.
pub fn battery_in(report: &[u8]) -> Option<proto::PadBattery> {
    match report {
        [0x21 | 0x30, _, byte, ..] => Some(proto::PadBattery::from_switch_byte(*byte)),
        _ => None,
    }
}

/// Where the battery comes from, for one pad.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Source {
    /// `hid-nintendo`'s `power_supply/<name>/` directory.
    Kernel(PathBuf),
    /// The pad's `/dev/hidrawN`, to ask.
    Hidraw(PathBuf),
}

/// A pad whose battery can be read: its address, and where from.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Target {
    mac: String,
    source: Source,
}

/// The pad behind the input node `node`, if it is a Nintendo one this can read.
///
/// From `/sys/class/input/eventN/device` (the input device, whose `uniq` is the address) up one to
/// the HID device, whose name carries the vendor and under which `hidraw/` and `power_supply/` are.
/// `sysfs` and `dev` are parameters so a test can lay the tree out in a temporary directory.
fn target(sysfs: &Path, dev: &Path, node: &Path) -> Option<Target> {
    let input = sysfs
        .join("class/input")
        .join(node.file_name()?)
        .join("device");
    let hid = std::fs::canonicalize(input.join("device")).ok()?;
    let vendor = hid.file_name()?.to_str()?.split(':').nth(1)?;
    if !vendor.eq_ignore_ascii_case(NINTENDO) {
        return None;
    }
    let mac = std::fs::read_to_string(input.join("uniq"))
        .ok()?
        .trim()
        .to_ascii_uppercase();
    if mac.is_empty() {
        return None;
    }
    let first = |dir: PathBuf| -> Option<PathBuf> {
        let mut names: Vec<_> = std::fs::read_dir(dir)
            .ok()?
            .filter_map(Result::ok)
            .map(|e| e.path())
            .collect();
        names.sort();
        names.into_iter().next()
    };
    let source = match first(hid.join("power_supply")) {
        Some(supply) if supply.join("capacity_level").exists() => Source::Kernel(supply),
        _ => Source::Hidraw(dev.join(first(hid.join("hidraw"))?.file_name()?)),
    };
    Some(Target { mac, source })
}

/// `hid-nintendo`'s reading, in the pad's own levels.
fn read_kernel(supply: &Path) -> Result<proto::PadBattery, String> {
    let read = |name: &str| {
        std::fs::read_to_string(supply.join(name))
            .map(|s| s.trim().to_owned())
            .map_err(|e| format!("{}: {e}", supply.join(name).display()))
    };
    let level = match read("capacity_level")?.as_str() {
        "Full" => proto::PadBatteryLevel::Full,
        "High" => proto::PadBatteryLevel::Medium,
        "Normal" => proto::PadBatteryLevel::Low,
        "Low" => proto::PadBatteryLevel::Critical,
        "Critical" => proto::PadBatteryLevel::Empty,
        other => return Err(format!("the kernel says the level is {other:?}")),
    };
    let charging = read("status").is_ok_and(|s| s == "Charging");
    Ok(proto::PadBattery { level, charging })
}

/// Ask the pad on its hidraw node, and read the battery off its reply.
fn ask_hidraw(node: &Path, counter: u8) -> Result<proto::PadBattery, String> {
    let mut file = std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .custom_flags(libc::O_NONBLOCK)
        .open(node)
        .map_err(|e| {
            if e.kind() == ErrorKind::PermissionDenied {
                format!(
                    "{}: {e} — the udev rule that gives padd the Nintendo pads' hidraw \
                     (70-robot-pad-hidraw.rules) is not in place",
                    node.display()
                )
            } else {
                format!("{}: {e}", node.display())
            }
        })?;
    file.write_all(&ask_report(counter))
        .map_err(|e| format!("asking the pad: {e}"))?;
    let until = Instant::now() + REPLY_WITHIN;
    let mut report = [0u8; 64];
    while Instant::now() < until {
        match file.read(&mut report) {
            Ok(n) => {
                if let Some(battery) = battery_in(&report[..n]) {
                    return Ok(battery);
                }
            }
            Err(e) if e.kind() == ErrorKind::WouldBlock => {
                std::thread::sleep(Duration::from_millis(10));
            }
            Err(e) => return Err(format!("reading the pad's reply: {e}")),
        }
    }
    Err(format!("the pad did not answer within {REPLY_WITHIN:?}"))
}

/// Write the reading where `configd` reads it, whole: through a temporary file and a rename.
fn write_reading(file: &Path, reading: &proto::pad_battery::Reading) -> std::io::Result<()> {
    let line = serde_json::to_vec(reading).map_err(std::io::Error::other)?;
    let partial = file.with_extension("json.partial");
    std::fs::write(&partial, line)?;
    std::fs::rename(&partial, file)
}

fn now_s() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_secs())
}

struct Shared {
    /// The input node the driving pad is on, as the main loop last said.
    wanted: Mutex<Option<PathBuf>>,
    wake: Condvar,
}

/// The battery reader: a thread of its own, told which pad by the main loop.
pub struct Battery {
    shared: Arc<Shared>,
}

impl Battery {
    /// Start the reader, writing to `file`.
    pub fn start(file: PathBuf) -> Self {
        let shared = Arc::new(Shared {
            wanted: Mutex::new(None),
            wake: Condvar::new(),
        });
        let thread_shared = shared.clone();
        std::thread::Builder::new()
            .name("pad-battery".into())
            .spawn(move || run(&thread_shared, &file))
            .expect("spawning the pad battery thread");
        Self { shared }
    }

    /// This is the driving pad. Safe to call every tick: it compares first. The gamepad rather
    /// than a path, as for the tap, so the node can only be the one gilrs drives from.
    pub fn watch(&self, pad: &gilrs::Gamepad<'_>) {
        let node = pad.devpath();
        let mut wanted = self
            .shared
            .wanted
            .lock()
            .unwrap_or_else(PoisonError::into_inner);
        if wanted.as_deref() != Some(node) {
            *wanted = Some(node.to_path_buf());
            self.shared.wake.notify_all();
        }
    }

    /// No pad.
    pub fn idle(&self) {
        let mut wanted = self
            .shared
            .wanted
            .lock()
            .unwrap_or_else(PoisonError::into_inner);
        if wanted.is_some() {
            *wanted = None;
            self.shared.wake.notify_all();
        }
    }
}

fn run(shared: &Shared, file: &Path) {
    let mut current: Option<PathBuf> = None;
    let mut pad: Option<Target> = None;
    let mut next = Instant::now();
    let mut counter: u8 = 0;
    let mut last: Option<proto::PadBattery> = None;
    let mut said_failure = false;
    loop {
        let wanted = {
            let guard = shared.wanted.lock().unwrap_or_else(PoisonError::into_inner);
            // Until the pad changes, or it is time to ask; with nothing to ask, only the former.
            let wait = if pad.is_some() {
                next.saturating_duration_since(Instant::now())
            } else {
                Duration::from_secs(3600)
            };
            let (guard, _) = shared
                .wake
                .wait_timeout_while(guard, wait, |w| *w == current)
                .unwrap_or_else(PoisonError::into_inner);
            guard.clone()
        };

        if wanted != current {
            current = wanted;
            // A reading belongs to the pad it was read from: gone with it, before anything else.
            let _ = std::fs::remove_file(file);
            last = None;
            said_failure = false;
            pad = current
                .as_deref()
                .and_then(|node| target(Path::new("/sys"), Path::new("/dev"), node));
            if let Some(found) = &pad {
                tracing::info!(mac = %found.mac, source = ?found.source, "reading the pad's battery");
            }
            next = Instant::now() + SETTLE;
            continue;
        }

        let Some(found) = &pad else { continue };
        if Instant::now() < next {
            continue;
        }
        next = Instant::now() + EVERY;
        let read = match &found.source {
            Source::Kernel(supply) => read_kernel(supply),
            Source::Hidraw(node) => {
                counter = counter.wrapping_add(1);
                ask_hidraw(node, counter)
            }
        };
        match read {
            Ok(battery) => {
                if last != Some(battery) {
                    tracing::info!(
                        mac = %found.mac, level = battery.level.word(), charging = battery.charging,
                        "pad battery"
                    );
                }
                last = Some(battery);
                said_failure = false;
                let reading = proto::pad_battery::Reading {
                    mac: found.mac.clone(),
                    battery,
                    read_s: now_s(),
                };
                if let Err(e) = write_reading(file, &reading) {
                    tracing::warn!(file = %file.display(), error = %e, "cannot keep the pad battery");
                }
            }
            // Once per pad, not once a minute: a pad that never answers is one line, and the
            // file going stale is what tells `configd` to stop showing the last level.
            Err(why) if !said_failure => {
                said_failure = true;
                tracing::warn!(mac = %found.mac, %why, "no pad battery");
            }
            Err(_) => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A sysfs and /dev laid out as the board has them for the Pro Controller over Bluetooth:
    /// `class/input/eventN/device` → the input device, its parent the HID device.
    fn lay_out(root: &Path, hid_name: &str, uniq: &str, kernel: bool) -> PathBuf {
        let hid = root.join("sys/devices/virtual/misc/uhid").join(hid_name);
        let input = hid.join("input/input4");
        std::fs::create_dir_all(&input).unwrap();
        std::fs::write(input.join("uniq"), format!("{uniq}\n")).unwrap();
        std::os::unix::fs::symlink(&hid, input.join("device")).unwrap();
        std::fs::create_dir_all(hid.join("hidraw/hidraw0")).unwrap();
        if kernel {
            let supply = hid.join("power_supply/nintendo_switch_controller_battery_x");
            std::fs::create_dir_all(&supply).unwrap();
            std::fs::write(supply.join("capacity_level"), "High\n").unwrap();
            std::fs::write(supply.join("status"), "Charging\n").unwrap();
        }
        let class = root.join("sys/class/input/event4");
        std::fs::create_dir_all(&class).unwrap();
        std::os::unix::fs::symlink(&input, class.join("device")).unwrap();
        root.join("sys")
    }

    #[test]
    fn a_nintendo_pad_is_asked_on_its_hidraw() {
        let dir = tempfile::tempdir().unwrap();
        let sys = lay_out(
            dir.path(),
            "0005:057E:2009.0001",
            "98:b6:e9:5e:bb:db",
            false,
        );
        let found = target(&sys, Path::new("/dev"), Path::new("/dev/input/event4")).unwrap();
        assert_eq!(found.mac, "98:B6:E9:5E:BB:DB");
        assert_eq!(found.source, Source::Hidraw(PathBuf::from("/dev/hidraw0")));
    }

    #[test]
    fn with_the_kernel_driver_its_reading_is_read_and_nothing_is_sent() {
        let dir = tempfile::tempdir().unwrap();
        let sys = lay_out(dir.path(), "0005:057E:2009.0001", "98:b6:e9:5e:bb:db", true);
        let found = target(&sys, Path::new("/dev"), Path::new("/dev/input/event4")).unwrap();
        let Source::Kernel(supply) = found.source else {
            panic!("expected the kernel's reading, got {:?}", found.source)
        };
        // The driver's "High" is the pad's 6 of 8.
        assert_eq!(
            read_kernel(&supply).unwrap(),
            proto::PadBattery {
                level: proto::PadBatteryLevel::Medium,
                charging: true
            }
        );
    }

    #[test]
    fn any_other_pad_is_not_asked() {
        let dir = tempfile::tempdir().unwrap();
        let sys = lay_out(
            dir.path(),
            "0005:045E:0B13.0002",
            "78:86:2e:bb:13:28",
            false,
        );
        assert!(target(&sys, Path::new("/dev"), Path::new("/dev/input/event4")).is_none());
    }

    #[test]
    fn the_ask_and_the_answer_are_the_bytes_the_pad_on_the_bench_used() {
        assert_eq!(
            ask_report(0x13),
            [
                0x01, 0x03, 0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40, 0x50
            ]
        );
        // The reply duck-d556's Pro Controller sent (2026-11-17), trimmed after the subcommand.
        let reply = [
            0x21, 0x10, 0x80, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x7f, 0x00, 0xf8, 0x7f, 0x00, 0x80,
            0x50, 0x00,
        ];
        assert_eq!(
            battery_in(&reply),
            Some(proto::PadBattery {
                level: proto::PadBatteryLevel::Full,
                charging: false
            })
        );
        // The simple-mode report it streams around it carries none.
        assert_eq!(battery_in(&[0x3f, 0x00, 0x00, 0x0f]), None);
        assert_eq!(battery_in(&[]), None);
    }

    #[test]
    fn a_reading_is_written_whole_and_reads_back() {
        let dir = tempfile::tempdir().unwrap();
        let file = dir.path().join("pad-battery.json");
        let reading = proto::pad_battery::Reading {
            mac: "98:B6:E9:5E:BB:DB".into(),
            battery: proto::PadBattery {
                level: proto::PadBatteryLevel::Low,
                charging: false,
            },
            read_s: now_s(),
        };
        write_reading(&file, &reading).unwrap();
        assert_eq!(
            proto::pad_battery::read_fresh(&file, now_s()),
            Some(reading)
        );
        assert!(!file.with_extension("json.partial").exists());
    }
}
