//! What the photo tag does: the robot poses, counts down with its flashlight, takes a picture and
//! sits with its beak open.
//!
//! A photo tag holds one text, [`TAG`], the way a mode tag holds its mode's name. Touched, it runs
//! this sequence, start to end, while the reader waits:
//!
//! 1. A quack, so the person holding the tag knows it was read.
//! 2. Standing, still: laser tracking off (it would walk the robot to a dot), `robot.stop`, and a
//!    seated robot stood up with `robot.do sit` — a toggle, so the robot's `robot.state` policy is
//!    read first. A robot whose policy is not driving (`held`: limp, or stood up by `robot.init`
//!    alone) is started with `robot.enable`, the pad's Start, so the tag works with no pad around;
//!    the sequence waits up to [`START_WITHIN`] for it to stand. A robot busy with something else
//!    (a skill, a ground pick, a rest) is left alone and the touch ends there.
//! 3. The head lifts a little: `robot.look` at a point ahead and above, and the joints it answers
//!    resent through `robot.head` for as long as the pose must hold, which keeps the idle
//!    look-around from taking the head back after its two seconds.
//! 4. [`COUNTDOWN`] of flashlight blinks at [`BLINK`] brightness, each shorter than the last
//!    ([`blinks`]), then the flash at [`FLASH_AT`] with a quack, a frame from `mediad`, the light
//!    off. Dim on purpose: the light points at the eyes of the person posing, and one LED does
//!    not light a scene anyway.
//! 5. The frame is saved as a JPEG named for the local time, in the photo directory
//!    (`/var/lib/robot/photos` on a robot).
//! 6. The robot sits and opens its beak.
//!
//! It stops there for now. What comes next is the e-ink tag: the robot waits for it in its beak,
//! writes the photo to it, then lowers its head and opens its beak to let it go.
//!
//! The camera is `mediad`'s local frame socket, the one its console snapshot reads: a raw frame and
//! the angle the camera is mounted at, turned upright here. A robot without a camera running still
//! sits and opens its beak — the pose is half of what the person was promised — and the journal
//! says why there is no picture.

use std::io::{BufRead, BufReader, Read, Write};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::time::Duration;

use duck_ipc_proto as proto;

use crate::ndef::Record;

/// What the tag is written with.
pub const TAG: &str = "take_photo";

/// "Tag read": the same quack a pairing or mode tag gets.
pub const HEARD: proto::SoundTag = crate::pairing::HEARD;

/// From the first blink to the picture.
pub const COUNTDOWN: Duration = Duration::from_secs(5);

/// The countdown's blinks, and the flash, as shares of the flashlight's full brightness.
pub const BLINK: f64 = 0.10;
pub const FLASH_AT: f64 = 0.15;

/// The shutter: a quack as the flash goes on, so the person posing hears the picture taken.
pub const SHUTTER: proto::SoundTag = proto::SoundTag::Chirp;

/// Where the robot looks for the picture, trunk frame: a metre ahead and about half a metre up,
/// which lifts the head towards a face without throwing it back.
pub const LOOK_UP: proto::LookParams = proto::LookParams {
    x: 1.0,
    y: 0.0,
    z: 0.55,
    neck_pitch: 0.0,
};

/// A seated robot standing up: `sitstand_rise_s` is 1 s by default, and the policy settles after.
const RISE: Duration = Duration::from_millis(1500);
/// How long a started robot has to come up standing: from limp it ramps torque on and goes to its
/// home pose first.
pub const START_WITHIN: Duration = Duration::from_secs(8);
/// How often its policy is read meanwhile.
const START_POLL: Duration = Duration::from_millis(250);
/// After `robot.stop`, for the walk to wind down before the pose.
const STILL: Duration = Duration::from_millis(500);
/// For the head to reach the pose before the countdown starts.
const LIFT: Duration = Duration::from_millis(600);
/// How long the light is on before the frame is taken, for the exposure to follow it.
const FLASH: Duration = Duration::from_millis(250);
/// Sitting down: `sitstand_ramp_s` is 2 s by default.
const SIT: Duration = Duration::from_secs(2);
/// The head is resent at least this often while the pose holds; the idle look-around starts after
/// two seconds without one.
const HOLD_EVERY: Duration = Duration::from_millis(400);

/// The first blink's period; the next ones shrink by [`SHRINK`] until shorter than [`FASTEST`].
const SLOWEST: f64 = 1.0;
const SHRINK: f64 = 0.8;
const FASTEST: f64 = 0.06;

/// Whether one of these records is the photo tag. The whole record, surrounding whitespace aside,
/// the same rule as [`crate::ndef::asks_to_pair`].
pub fn in_records(records: &[Record]) -> bool {
    records.iter().any(|r| r.value().trim() == TAG)
}

/// The blink periods of the countdown: a second apart at first, faster and faster, adding up to
/// `total`. Each period is the light on for half of it and off for the other half.
pub fn blinks(total: Duration) -> Vec<Duration> {
    let mut periods = Vec::new();
    let mut p = SLOWEST;
    while p >= FASTEST {
        periods.push(p);
        p *= SHRINK;
    }
    let scale = total.as_secs_f64() / periods.iter().sum::<f64>();
    periods
        .into_iter()
        .map(|p| Duration::from_secs_f64(p * scale))
        .collect()
}

/// The rest of the robot, as far as the photo tag is concerned.
pub trait Driver {
    fn sound(&mut self, tag: proto::SoundTag) -> Result<(), String>;
    /// `robot.laserTrack`.
    fn laser_track(&mut self, active: bool) -> Result<proto::LaserTrackResult, String>;
    /// `robot.stop`.
    fn stop(&mut self) -> Result<proto::IntentResult, String>;
    /// The `policy` of one `robot.state` frame: `walk`, `stand`, `sit`, `rise`, `held`, ...
    fn policy(&mut self) -> Result<String, String>;
    /// `robot.enable` on: the pad's Start.
    fn start(&mut self) -> Result<proto::IntentResult, String>;
    /// `robot.do sit`, which stands a seated robot up and sits a standing one.
    fn sit_toggle(&mut self) -> Result<proto::IntentResult, String>;
    /// `robot.look`.
    fn look(&mut self, at: proto::LookParams) -> Result<proto::LookResult, String>;
    /// `robot.head`, a notification.
    fn head(&mut self, head: proto::HeadParams) -> Result<(), String>;
    /// `robot.flashlight`, white, at `brightness` of full; 0 switches it off.
    fn flashlight(&mut self, brightness: f64) -> Result<proto::IntentResult, String>;
    /// `robot.mouth`, a notification. It holds until the next one.
    fn mouth(&mut self, open: f64) -> Result<(), String>;
    /// A frame from the camera, upright, as a JPEG.
    fn capture(&mut self) -> Result<Vec<u8>, String>;
    /// Keep the picture; where it went.
    fn save(&mut self, jpeg: &[u8]) -> Result<PathBuf, String>;
    /// Wait. A method so a test can run the sequence without waiting through it.
    fn sleep(&mut self, d: Duration);
}

/// How a touch ended, for the journal and the tests.
#[derive(Debug, PartialEq, Eq)]
pub enum Outcome {
    /// The picture is saved, and the robot sat down with its beak open.
    Taken(PathBuf),
    /// The pose ran, there is no picture, and why.
    NoPhoto(String),
    /// The robot was busy with something else, or did not come up standing once started: its
    /// policy then.
    NotStanding(String),
    /// `robotd` could not be asked, or refused a step the sequence needs.
    Failed(String),
}

pub fn on_touch(robot: &mut dyn Driver) -> Outcome {
    if let Err(why) = robot.sound(HEARD) {
        tracing::info!(%why, "the robot would not quack for the tag");
    }

    // Standing, still. Laser tracking off first: it walks the robot to the dot.
    match robot.laser_track(false) {
        Ok(r) if r.accepted => {}
        Ok(r) => tracing::info!(reason = ?r.reason, "laser tracking not switched off"),
        Err(why) => tracing::info!(%why, "laser tracking not switched off"),
    }
    if let Err(why) = asked("robot.stop", robot.stop()) {
        return Outcome::Failed(why);
    }
    let mut policy = match robot.policy() {
        Ok(p) => p,
        Err(why) => return Outcome::Failed(format!("robot.state: {why}")),
    };
    if matches!(policy.as_str(), "held" | "homing") {
        if let Err(why) = asked("robot.enable", robot.start()) {
            return Outcome::Failed(why);
        }
        let mut waited = Duration::ZERO;
        while !matches!(policy.as_str(), "walk" | "stand") && waited < START_WITHIN {
            robot.sleep(START_POLL);
            waited += START_POLL;
            policy = match robot.policy() {
                Ok(p) => p,
                Err(why) => return Outcome::Failed(format!("robot.state: {why}")),
            };
        }
    }
    match policy.as_str() {
        "walk" | "stand" => robot.sleep(STILL),
        "sit" => {
            if let Err(why) = asked("robot.do sit", robot.sit_toggle()) {
                return Outcome::Failed(why);
            }
            robot.sleep(RISE);
        }
        "rise" => robot.sleep(RISE),
        _ => return Outcome::NotStanding(policy),
    }

    // The head up, held through the countdown.
    let head = match robot.look(LOOK_UP) {
        Ok(r) => r.head,
        Err(why) => return Outcome::Failed(format!("robot.look: {why}")),
    };
    hold(robot, head, LIFT);
    for period in blinks(COUNTDOWN) {
        for brightness in [BLINK, 0.0] {
            if let Err(why) = asked("robot.flashlight", robot.flashlight(brightness)) {
                // A board without a light still takes the picture, only without the countdown.
                tracing::info!(%why, "no flashlight for the countdown");
            }
            hold(robot, head, period / 2);
        }
    }
    let _ = robot.flashlight(FLASH_AT);
    if let Err(why) = robot.sound(SHUTTER) {
        tracing::info!(%why, "no shutter quack");
    }
    hold(robot, head, FLASH);
    let shot = robot.capture();
    let _ = robot.flashlight(0.0);

    let outcome = match shot.and_then(|jpeg| robot.save(&jpeg)) {
        Ok(path) => Outcome::Taken(path),
        Err(why) => Outcome::NoPhoto(why),
    };

    // Sit, beak open.
    if let Err(why) = asked("robot.do sit", robot.sit_toggle()) {
        return Outcome::Failed(why);
    }
    robot.sleep(SIT);
    if let Err(why) = robot.mouth(1.0) {
        return Outcome::Failed(format!("robot.mouth: {why}"));
    }
    outcome
}

/// Resend the head for `d`, often enough that the idle look-around never starts.
fn hold(robot: &mut dyn Driver, head: proto::HeadParams, mut d: Duration) {
    loop {
        if let Err(why) = robot.head(head) {
            tracing::info!(%why, "robot.head");
        }
        let step = d.min(HOLD_EVERY);
        robot.sleep(step);
        d -= step;
        if d.is_zero() {
            return;
        }
    }
}

fn asked(what: &str, answer: Result<proto::IntentResult, String>) -> Result<(), String> {
    match answer {
        Err(e) => Err(format!("{what}: {e}")),
        Ok(r) if r.accepted => Ok(()),
        Ok(r) => Err(format!(
            "{what} refused: {}",
            r.reason.unwrap_or_else(|| "no reason given".into())
        )),
    }
}

// ── the camera ───────────────────────────────────────────────────────────────

/// One frame from `mediad`'s local frame socket, as an upright JPEG.
///
/// The same exchange as `mediad`'s console snapshot — the version handshake, then `media.frame`,
/// whose answer is a header line followed by that many bytes of UYVY — on a blocking socket, since
/// this daemon has no runtime to put an async one on.
pub fn capture(socket: &Path, within: Duration) -> Result<Vec<u8>, String> {
    let fail = |e: std::io::Error| format!("{}: {e}", socket.display());
    let mut stream = UnixStream::connect(socket).map_err(fail)?;
    stream.set_read_timeout(Some(within)).map_err(fail)?;
    let mut reader = BufReader::new(stream.try_clone().map_err(fail)?);

    let hello = proto::Request::call(
        proto::Id::Number(1),
        &proto::Call::Hello(proto::HelloParams {
            api_version: proto::API_VERSION,
        }),
    );
    send(&mut stream, &hello).map_err(fail)?;
    let hello: proto::HelloResult = answer(&mut reader, 1)?;
    if hello.api_version != proto::API_VERSION {
        // Logged and served, never refused: `media.frame` has not changed shape across versions.
        tracing::info!(
            ours = proto::API_VERSION,
            theirs = hello.api_version,
            "mediad speaks another API version"
        );
    }
    let frame = proto::Request {
        jsonrpc: "2.0".into(),
        id: Some(proto::Id::Number(2)),
        method: proto::method::MEDIA_FRAME.into(),
        params: None,
    };
    send(&mut stream, &frame).map_err(fail)?;
    let header: proto::MediaFrameHeader = answer(&mut reader, 2)?;
    if !header.valid_uyvy() {
        return Err("the camera sent a frame of impossible size".into());
    }
    let mut pixels = vec![0; header.bytes];
    reader.read_exact(&mut pixels).map_err(fail)?;
    jpeg(&header, &pixels)
}

fn send(stream: &mut UnixStream, request: &proto::Request) -> std::io::Result<()> {
    let mut line = serde_json::to_vec(request).map_err(std::io::Error::other)?;
    line.push(b'\n');
    stream.write_all(&line)
}

fn answer<T: for<'de> serde::Deserialize<'de>>(
    reader: &mut impl BufRead,
    id: u64,
) -> Result<T, String> {
    let mut line = Vec::new();
    // Bounded: a header is a few hundred bytes, and pixels must never be read as a line.
    reader
        .take(4097)
        .read_until(b'\n', &mut line)
        .map_err(|e| format!("camera: {e}"))?;
    if line.len() > 4096 || !line.ends_with(b"\n") {
        return Err("camera: answer too long, or cut short".into());
    }
    let response: proto::Response =
        serde_json::from_slice(&line).map_err(|e| format!("camera: unparsable answer: {e}"))?;
    if response.id != Some(proto::Id::Number(id)) {
        return Err("camera: answer to another request".into());
    }
    if let Some(error) = response.error {
        return Err(format!("camera: {}", error.message));
    }
    response
        .result_as()
        .map_err(|e| format!("camera: unexpected answer: {e}"))
}

/// JPEG quality: a keepsake, not a stream frame.
const QUALITY: u8 = 90;

/// Upright, full size, as a JPEG. Turned here because a file has nowhere to carry the mount angle
/// the header names — the reason `mediad`'s PNG snapshot turns it too.
pub fn jpeg(header: &proto::MediaFrameHeader, pixels: &[u8]) -> Result<Vec<u8>, String> {
    let turn = uyvy::Turn::from_degrees(header.rotate)
        .ok_or_else(|| format!("camera mounted at {}°, not a quarter turn", header.rotate))?;
    let mut rgb = Vec::new();
    let (width, height) = uyvy::rgb_from_uyvy(
        pixels,
        header.width as usize,
        header.height as usize,
        header.width.max(header.height) as usize,
        turn,
        &mut rgb,
    );
    let mut out = Vec::new();
    image::codecs::jpeg::JpegEncoder::new_with_quality(&mut out, QUALITY)
        .encode(
            &rgb,
            width as u32,
            height as u32,
            image::ExtendedColorType::Rgb8,
        )
        .map_err(|e| format!("encoding the photo: {e}"))?;
    Ok(out)
}

// ── where it is kept ─────────────────────────────────────────────────────────

/// Write the picture as `<dir>/<name>.jpg`, through a temporary file so a photo directory never
/// holds half a picture. A second photo in the same second gets `-2`, `-3`, ...
pub fn save(dir: &Path, name: &str, jpeg: &[u8]) -> Result<PathBuf, String> {
    let fail = |e: std::io::Error| format!("{}: {e}", dir.display());
    std::fs::create_dir_all(dir).map_err(fail)?;
    let path = (1..)
        .map(|n| match n {
            1 => dir.join(format!("{name}.jpg")),
            n => dir.join(format!("{name}-{n}.jpg")),
        })
        .find(|p| !p.exists())
        .expect("an unbounded range has a free name");
    let partial = path.with_extension("jpg.partial");
    std::fs::write(&partial, jpeg).map_err(fail)?;
    std::fs::rename(&partial, &path).map_err(fail)?;
    Ok(path)
}

/// The local time as a file name: `2026-11-11_14-03-27`. Local because the person looking for the
/// picture knows when they took it on their own clock.
pub fn local_name(at: std::time::SystemTime) -> String {
    let secs = at
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs()) as libc::time_t;
    // SAFETY: `localtime_r` writes only into the `tm` it is given, which is zeroed and lives on
    // this stack for the call; `secs` is a valid `time_t` it only reads.
    let tm = unsafe {
        let mut tm: libc::tm = std::mem::zeroed();
        if libc::localtime_r(&secs, &mut tm).is_null() {
            return format!("{secs}");
        }
        tm
    };
    format!(
        "{:04}-{:02}-{:02}_{:02}-{:02}-{:02}",
        tm.tm_year + 1900,
        tm.tm_mon + 1,
        tm.tm_mday,
        tm.tm_hour,
        tm.tm_min,
        tm.tm_sec
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Fake {
        policy: String,
        /// What the policy becomes once `robot.enable` is asked; `None` stays where it was.
        started: Option<String>,
        camera: Result<Vec<u8>, String>,
        no_light: bool,
        calls: Vec<String>,
        slept: Duration,
        /// The longest the head went without being resent.
        head_gap: Duration,
        since_head: Duration,
        /// Between `look` and `capture`, when the head must be held.
        holding: bool,
    }

    impl Default for Fake {
        fn default() -> Self {
            Self {
                policy: "stand".into(),
                started: Some("stand".into()),
                camera: Ok(b"jpeg".to_vec()),
                no_light: false,
                calls: Vec::new(),
                slept: Duration::ZERO,
                head_gap: Duration::ZERO,
                since_head: Duration::ZERO,
                holding: false,
            }
        }
    }

    impl Driver for Fake {
        fn sound(&mut self, _: proto::SoundTag) -> Result<(), String> {
            self.calls.push("quack".into());
            Ok(())
        }
        fn laser_track(&mut self, active: bool) -> Result<proto::LaserTrackResult, String> {
            self.calls
                .push(format!("laser {}", if active { "on" } else { "off" }));
            Ok(proto::LaserTrackResult::default())
        }
        fn stop(&mut self) -> Result<proto::IntentResult, String> {
            self.calls.push("stop".into());
            Ok(proto::IntentResult::accepted())
        }
        fn policy(&mut self) -> Result<String, String> {
            Ok(self.policy.clone())
        }
        fn start(&mut self) -> Result<proto::IntentResult, String> {
            self.calls.push("start".into());
            if let Some(p) = &self.started {
                self.policy = p.clone();
            }
            Ok(proto::IntentResult::accepted())
        }
        fn sit_toggle(&mut self) -> Result<proto::IntentResult, String> {
            self.calls.push("sit".into());
            Ok(proto::IntentResult::accepted())
        }
        fn look(&mut self, _: proto::LookParams) -> Result<proto::LookResult, String> {
            self.calls.push("look".into());
            self.holding = true;
            Ok(proto::LookResult {
                head: proto::HeadParams::default(),
                clamped: false,
            })
        }
        fn head(&mut self, _: proto::HeadParams) -> Result<(), String> {
            self.calls.push("head".into());
            self.since_head = Duration::ZERO;
            Ok(())
        }
        fn flashlight(&mut self, brightness: f64) -> Result<proto::IntentResult, String> {
            self.calls.push(format!("light {brightness}"));
            Ok(if self.no_light {
                proto::IntentResult::refused("this board has no flashlight")
            } else {
                proto::IntentResult::accepted()
            })
        }
        fn mouth(&mut self, open: f64) -> Result<(), String> {
            self.calls.push(format!("mouth {open}"));
            Ok(())
        }
        fn capture(&mut self) -> Result<Vec<u8>, String> {
            self.calls.push("capture".into());
            self.holding = false;
            self.camera.clone()
        }
        fn save(&mut self, _: &[u8]) -> Result<PathBuf, String> {
            self.calls.push("save".into());
            Ok("/photos/now.jpg".into())
        }
        fn sleep(&mut self, d: Duration) {
            self.slept += d;
            if !self.holding {
                return;
            }
            self.since_head += d;
            self.head_gap = self.head_gap.max(self.since_head);
        }
    }

    /// The calls with the countdown's blinks and the head resends folded away.
    fn shape(calls: &[String]) -> Vec<String> {
        let capture = calls.iter().position(|c| c == "capture").unwrap_or(0);
        let mut out: Vec<String> = Vec::new();
        for (i, c) in calls.iter().enumerate() {
            if c == "head" {
                continue;
            }
            // Every light switch before the flash, the last one ahead of the capture, is a blink.
            let last_on = calls[..capture]
                .iter()
                .rposition(|c| c.starts_with("light"));
            if c.starts_with("light") && Some(i) < last_on {
                if out.last().map(String::as_str) != Some("blinks") {
                    out.push("blinks".into());
                }
                continue;
            }
            out.push(c.clone());
        }
        out
    }

    #[test]
    fn the_tag_is_its_text_alone() {
        let r = |s: &str| Record::Text(s.into());
        assert!(in_records(&[r("take_photo")]));
        assert!(in_records(&[r(" take_photo\n")]));
        assert!(!in_records(&[r("take_photo please")]));
        assert!(!in_records(&[r("walk_mode")]));
    }

    #[test]
    fn a_standing_robot_poses_counts_down_shoots_and_sits_with_its_beak_open() {
        let mut robot = Fake::default();
        assert_eq!(
            on_touch(&mut robot),
            Outcome::Taken("/photos/now.jpg".into())
        );
        assert_eq!(
            shape(&robot.calls),
            [
                "quack",
                "laser off",
                "stop",
                "look",
                "blinks",
                "light 0.15",
                "quack",
                "capture",
                "light 0",
                "save",
                "sit",
                "mouth 1"
            ]
        );
    }

    #[test]
    fn a_seated_robot_stands_up_first() {
        let mut robot = Fake {
            policy: "sit".into(),
            ..Fake::default()
        };
        assert!(matches!(on_touch(&mut robot), Outcome::Taken(_)));
        let shape = shape(&robot.calls);
        assert_eq!(&shape[..5], ["quack", "laser off", "stop", "sit", "look"]);
        assert_eq!(shape.iter().filter(|c| *c == "sit").count(), 2);
    }

    /// No pad needed: a robot whose policy is not driving is started, as Start would.
    #[test]
    fn a_robot_not_driving_is_started_first() {
        let mut robot = Fake {
            policy: "held".into(),
            ..Fake::default()
        };
        assert!(matches!(on_touch(&mut robot), Outcome::Taken(_)));
        assert_eq!(
            &shape(&robot.calls)[..5],
            ["quack", "laser off", "stop", "start", "look"]
        );
    }

    #[test]
    fn a_robot_that_does_not_come_up_is_left_there() {
        let mut robot = Fake {
            policy: "held".into(),
            started: None,
            ..Fake::default()
        };
        assert_eq!(on_touch(&mut robot), Outcome::NotStanding("held".into()));
        assert_eq!(robot.calls, ["quack", "laser off", "stop", "start"]);
        assert!(robot.slept >= START_WITHIN);
    }

    #[test]
    fn a_robot_busy_with_a_skill_is_left_alone() {
        let mut robot = Fake {
            policy: "skill".into(),
            ..Fake::default()
        };
        assert_eq!(on_touch(&mut robot), Outcome::NotStanding("skill".into()));
        assert_eq!(robot.calls, ["quack", "laser off", "stop"]);
    }

    /// The pose is half the promise: no camera still ends seated with the beak open.
    #[test]
    fn no_camera_still_sits_and_opens_the_beak() {
        let mut robot = Fake {
            camera: Err("no such socket".into()),
            ..Fake::default()
        };
        assert_eq!(
            on_touch(&mut robot),
            Outcome::NoPhoto("no such socket".into())
        );
        assert!(!robot.calls.contains(&"save".to_string()));
        assert_eq!(robot.calls.last().unwrap(), "mouth 1");
    }

    #[test]
    fn no_flashlight_still_takes_the_picture() {
        let mut robot = Fake {
            no_light: true,
            ..Fake::default()
        };
        assert!(matches!(on_touch(&mut robot), Outcome::Taken(_)));
    }

    /// Never two seconds without a head, or the idle look-around takes it back mid-countdown.
    #[test]
    fn the_head_is_held_through_the_countdown() {
        let mut robot = Fake::default();
        on_touch(&mut robot);
        assert!(robot.calls.iter().filter(|c| *c == "head").count() > 10);
        assert!(robot.head_gap <= HOLD_EVERY, "{:?}", robot.head_gap);
    }

    /// Dim, so it does not hurt: the blinks at a tenth, the flash at 15 %.
    #[test]
    fn the_light_is_dim() {
        let mut robot = Fake::default();
        on_touch(&mut robot);
        let lit: Vec<&str> = robot
            .calls
            .iter()
            .filter(|c| c.starts_with("light") && *c != "light 0")
            .map(String::as_str)
            .collect();
        let (flash, blinks) = lit.split_last().unwrap();
        assert_eq!(*flash, "light 0.15");
        assert!(
            blinks.len() > 8 && blinks.iter().all(|c| *c == "light 0.1"),
            "{blinks:?}"
        );
    }

    #[test]
    fn the_countdown_speeds_up_and_lasts_five_seconds() {
        let b = blinks(COUNTDOWN);
        assert!(b.len() >= 8, "{b:?}");
        assert!(b.windows(2).all(|w| w[1] < w[0]), "{b:?}");
        let total: Duration = b.iter().sum();
        assert!(
            total.abs_diff(COUNTDOWN) < Duration::from_millis(5),
            "{total:?}"
        );
        assert!(
            b[0] > Duration::from_millis(800) && *b.last().unwrap() < Duration::from_millis(80)
        );
    }

    #[test]
    fn a_photo_is_never_overwritten() {
        let dir = std::env::temp_dir().join(format!("nfc-photo-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let a = save(&dir, "2026-11-11_14-03-27", b"one").unwrap();
        let b = save(&dir, "2026-11-11_14-03-27", b"two").unwrap();
        assert_eq!(a.file_name().unwrap(), "2026-11-11_14-03-27.jpg");
        assert_eq!(b.file_name().unwrap(), "2026-11-11_14-03-27-2.jpg");
        assert_eq!(std::fs::read(&a).unwrap(), b"one");
        assert_eq!(std::fs::read(&b).unwrap(), b"two");
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn the_name_is_a_date_and_a_time() {
        let name = local_name(std::time::SystemTime::now());
        assert_eq!(name.len(), "2026-11-11_14-03-27".len(), "{name}");
        assert_eq!(&name[10..11], "_");
    }

    #[test]
    fn a_frame_comes_out_upright_and_as_a_jpeg() {
        let header = proto::MediaFrameHeader {
            width: 8,
            height: 4,
            format: "UYVY".into(),
            bytes: 8 * 4 * 2,
            captured_at_unix_us: 0,
            rotate: 90,
        };
        let jpeg = jpeg(&header, &vec![128; header.bytes]).unwrap();
        assert_eq!(&jpeg[..2], [0xFF, 0xD8]);
    }
}
