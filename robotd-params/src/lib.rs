//! `robotd`'s startup parameters: the schema, the defaults, and the validation.
//!
//! A file rather than a wall of CLI flags — the prototype grew 142 of them and most were
//! variants, dead skills and dead sensors, all of which are gone. **Read once at startup,
//! not watched**; live reload is deferred (`docs/design/robotd-design.md` §4.2). That fact
//! is load-bearing for tooling: *any* change to the file requires a `robotd` restart, so an
//! editor never has to ask which keys are live.
//!
//! It lives outside `releases/<ver>/` so it survives an update *and* a rollback: this is
//! per-robot configuration, not shipped defaults (`architecture.md` §3).
//!
//! A crate of its own rather than a module of `robotd`, for one consumer: `robotctl
//! configure` edits the file interactively, and doing that against a copied schema is how a
//! copied schema drifts. [`registry`] is the machine-readable index of every key — what it
//! is, what it defaults to, what values it takes — and its completeness is enforced by a
//! test that walks [`Params`]'s own serialization, so a new section cannot be added without
//! the registry (and therefore the editor) learning about it.

pub mod edit;
pub mod registry;

use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

/// Where a release is mounted.
pub const RELEASE_DIR: &str = "/opt/robot/daemon/current";

/// Where the official policy set lives — **outside the release directory**, on purpose.
///
/// A gait retrain should not need a daemon release, and a daemon fix should not re-download six
/// megabytes of unchanged weights. So the two version independently, and the daemon reads its
/// policies from one place regardless of what put them there.
///
/// Today what puts them there is the daemon's own postinstall hook, which seeds this directory
/// from the copies the release still carries. That is a bootstrap, not the destination: it stops
/// the moment anything installs a real set here and repoints `current`, and `current` being a
/// symlink beside a `releases/` directory is exactly the shape the updater already swaps
/// atomically. See `docs/design/policy-channel-design.md` §9.
pub const POLICY_DIR: &str = "/opt/robot/policies/current";

/// Where policies fetched from the Hub one at a time live — `robotctl policy load <slot> <repo>`.
///
/// Outside every release directory, per `updater-design.md` §5.7: a policy somebody chose has to
/// survive an update and a rollback. The layout mirrors the repo — `<org>/<name>/<revision>/` —
/// which is what makes the path in `robotd.toml` say where a policy came from without anything
/// having to look it up. `updater::policy::LIBRARY_ROOT` is the same string on the writing side.
pub const POLICY_LIBRARY: &str = "/var/lib/robot/policies";

/// Where a provisioned robot keeps it, alongside the updater's own config.
pub const DEFAULT_PATH: &str = "/etc/robot/robotd.toml";

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct Params {
    pub bus: Bus,
    pub control: Control,
    pub update_gate: UpdateGate,
    pub policy: PolicyParams,
    pub safety: SafetyParams,
    pub audio: AudioParams,
    pub theremin: ThereminParams,
    pub chorale: ChoraleParams,
    pub media: MediaParams,
    pub detect: DetectParams,
    /// Which pad button runs which skill. `padd` reads this, not `robotd`.
    pub pad: PadParams,
}

/// Which pad button runs which skill.
///
/// **The five one-shot buttons, and only those.** `Start` toggles the policy, `Y` and `B` switch
/// what the sticks mean, held `Select` powers the robot off and held `D-pad up` changes drive
/// mode — none of those is a `robot.do`, and turning them into a general button-to-action
/// vocabulary is a larger thing than binding a skill needs. It would also put "the button that
/// stops the robot" behind a config key, which is the one binding worth not being able to lose.
///
/// Empty means the mapping the prototype had and muscle memory expects. A named button is
/// rebound; the rest stay as they were. The pad is full — every face button already does
/// something — so binding a new skill nearly always means taking a button from an old one, which
/// is why every one of the five is nameable rather than only the free ones.
///
/// A name here is not checked against anything at parse time: which skills exist is a property of
/// the robot, and `padd` learns it from `robot.subscribe`. An unknown name is refused by `robotd`
/// with the list it does know, which is a better error than a config file could give.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct PadParams {
    /// A (South). The ground pick, by default.
    pub a: String,
    /// B (East) is body-pose mode and is not bindable; X (West) is the roulade.
    pub x: String,
    /// The left bumper — `LeftTrigger` in gilrs, which names the *analog* trigger
    /// `LeftTrigger2`. Getting that backwards binds a skill to a control nobody presses.
    pub lb: String,
    /// The right bumper, likewise.
    pub rb: String,
    /// D-pad down. The sit toggle, by default.
    pub dpad_down: String,
}

impl Default for PadParams {
    fn default() -> Self {
        Self {
            a: "ground_pick".to_owned(),
            x: "roulade".to_owned(),
            lb: "kick_left".to_owned(),
            rb: "kick_right".to_owned(),
            dpad_down: "sit_toggle".to_owned(),
        }
    }
}

impl PadParams {
    /// The bindable buttons, in the order a listing should print them.
    pub const BUTTONS: [&'static str; 5] = ["a", "x", "lb", "rb", "dpad_down"];

    /// What a button runs, or `None` for a name this build has no button for.
    pub fn skill(&self, button: &str) -> Option<&str> {
        Some(match button {
            "a" => &self.a,
            "x" => &self.x,
            "lb" => &self.lb,
            "rb" => &self.rb,
            "dpad_down" => &self.dpad_down,
            _ => return None,
        })
    }

    /// Bind a button to a skill. `false` for a button this build does not have.
    pub fn bind(&mut self, button: &str, skill: &str) -> bool {
        let slot = match button {
            "a" => &mut self.a,
            "x" => &mut self.x,
            "lb" => &mut self.lb,
            "rb" => &mut self.rb,
            "dpad_down" => &mut self.dpad_down,
            _ => return false,
        };
        *slot = skill.to_owned();
        true
    }

    /// Every button name, for the "expected one of" half of a refusal.
    pub fn names() -> String {
        Self::BUTTONS.join(", ")
    }
}

/// The one video mode a robot streams in, as a name rather than four numbers.
///
/// **Frame size, rate and a matching bitrate move together or not at all.** They are not
/// independent settings: 1080p at the 2 Mb/s that suits 720p is a smear, and 720p at 6 Mb/s
/// spends a link's headroom on nothing. Offering `width`, `height`, `fps` and `bitrate` as four
/// keys would make every wrong combination of them expressible — including the ones the capture
/// path cannot produce at all, and a pipeline that will not start costs the WebRTC *control*
/// channel along with the video, because the two are bundled (`remote-webrtc.md`).
///
/// So the ladder is fixed, and every rung is 16:9 — the sensor's own aspect. A mode that changed
/// the shape of the picture would be cropping or squashing rather than lowering quality, which is
/// not what anybody picking "smaller" is asking for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum Quality {
    /// The sensor's full frame. The most detail, and the rung least likely to hold 30 fps on
    /// this ISP path — [`MediaParams`] says what is measured and what is not.
    #[serde(rename = "1080p30")]
    Q1080p30,
    /// What every measurement in `mediad` was taken at, and the default.
    #[default]
    #[serde(rename = "720p30")]
    Q720p30,
    /// Same picture, half the frames: the rung for a link that cannot carry 30.
    #[serde(rename = "720p15")]
    Q720p15,
    /// Small and cheap, for a bad link or a busy CPU.
    #[serde(rename = "360p30")]
    Q360p30,
}

/// Every mode, in the order an editor cycles them — and the strings the file uses.
///
/// One list, so the registry's choices, the file's values and [`Quality`] itself cannot disagree;
/// [`tests::every_quality_label_round_trips`] pins it to the enum in both directions.
pub const QUALITY_LABELS: &[&str] = &["1080p30", "720p30", "720p15", "360p30"];

impl Quality {
    /// The modes, in [`QUALITY_LABELS`] order.
    pub const ALL: [Quality; 4] = [
        Quality::Q1080p30,
        Quality::Q720p30,
        Quality::Q720p15,
        Quality::Q360p30,
    ];

    /// The name this mode has in the file.
    pub fn label(self) -> &'static str {
        match self {
            Quality::Q1080p30 => "1080p30",
            Quality::Q720p30 => "720p30",
            Quality::Q720p15 => "720p15",
            Quality::Q360p30 => "360p30",
        }
    }

    /// Frame size in pixels. Every rung is 16:9 and every dimension is a multiple of 8, which
    /// is what the ISP's scaler and the encoder's macroblocks both want.
    pub fn size(self) -> (u32, u32) {
        match self {
            Quality::Q1080p30 => (1920, 1080),
            Quality::Q720p30 | Quality::Q720p15 => (1280, 720),
            Quality::Q360p30 => (640, 360),
        }
    }

    pub fn width(self) -> u32 {
        self.size().0
    }

    pub fn height(self) -> u32 {
        self.size().1
    }

    pub fn fps(self) -> u32 {
        match self {
            Quality::Q720p15 => 15,
            _ => 30,
        }
    }

    /// What this mode streams at when `[media] bitrate` is unset — bits per second.
    ///
    /// Scaled with the pixel rate rather than picked per rung: 720p30 is the measured 2 Mb/s
    /// `mediad` has always used, and the others are that number times their share of the pixels
    /// per second, rounded to something a human can read. Congestion control moves from here, so
    /// this is a starting point rather than a cap.
    pub fn default_bitrate(self) -> u32 {
        match self {
            Quality::Q1080p30 => 4_000_000,
            Quality::Q720p30 => 2_000_000,
            Quality::Q720p15 => 1_000_000,
            Quality::Q360p30 => 800_000,
        }
    }
}

/// How `mediad` decides what bitrate to actually send at.
///
/// **This is a CPU setting as much as a network one.** The estimator is not free: on the board,
/// with one peer connected, `rtpgccbwe` is the single largest consumer in the process — 7.6% of a
/// core against `v4l2src`'s 0.3% — because it works per packet while capture works per DMABuf
/// handle. Turning it off deletes that thread.
///
/// What it costs is adaptivity, and that is not a small thing: adapting the rate to the link is
/// the whole reason `webrtcsink` is handed raw video rather than pre-encoded H.264
/// (`mediad::pipeline`). On a link that stays good — a robot one hop away on its own LAN — the
/// estimator spends CPU discovering a ceiling it will never hit. On a link that degrades, it is
/// what keeps a picture rather than a stall.
///
/// **It also decides what `bitrate` means.** With an estimator running, `bitrate` is a starting
/// point it ramps away from within seconds. Disabled, nothing moves it, and `bitrate` is the rate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum CongestionControl {
    /// Nothing adapts. `bitrate` is exactly what is sent, and a link that degrades degrades the
    /// picture rather than the rate.
    Disabled,
    /// `webrtcsink`'s own sender-side heuristic. Cheaper than the estimator, blunter than it.
    Homegrown,
    /// Google Congestion Control, and `webrtcsink`'s own default — so this is what every robot has
    /// been running, and naming it here changes nothing.
    #[default]
    Gcc,
}

/// Every mode, in the order an editor cycles them.
///
/// These are `webrtcsink`'s own property nicknames rather than names of ours: they are what gets
/// set on the element, and a second vocabulary in between would be one more thing to get wrong.
/// Note `gcc`, not `googcc` — [`tests::every_congestion_label_round_trips`] pins the spelling.
pub const CONGESTION_LABELS: &[&str] = &["disabled", "homegrown", "gcc"];

impl CongestionControl {
    pub const ALL: [CongestionControl; 3] = [
        CongestionControl::Disabled,
        CongestionControl::Homegrown,
        CongestionControl::Gcc,
    ];

    /// The `congestion-control` nickname `webrtcsink` knows this by.
    pub fn nick(self) -> &'static str {
        match self {
            CongestionControl::Disabled => "disabled",
            CongestionControl::Homegrown => "homegrown",
            CongestionControl::Gcc => "gcc",
        }
    }
}

/// `[media]` — what `mediad` streams.
///
/// **These were command-line flags in `mediad.service`, and that is why this section exists.**
/// The release installer rewrites that unit file, so the only supported way to change a flag was
/// a systemd drop-in — a mechanism nobody reaches for to answer "why is the video soft?". Here
/// they are three keys in the file `robotctl configure` already edits.
///
/// `mediad` reads this file at startup and nothing else does anything to it, so a change needs
/// `systemctl restart mediad` — not `robotd`. The editor offers the right one.
///
/// **What is measured and what is not.** 720p30 is the rung every number in `mediad::pipeline`
/// comes from: 29.3 fps off the ISP main path, with the capture format and buffer depth that took
/// three bench sessions to find. The sensor is pinned to a 1920x1080 mode that runs at 30 and the
/// ISP scales down from it, so 1080p30 asks for no scaling at all — what is unmeasured there is
/// whether the capture path and the encoder hold 30 fps at 2.25x the pixels. A rung that does not
/// hold runs slower; it is not a pipeline that fails to start.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct MediaParams {
    /// Stream the head camera. `false` streams a test pattern instead, which is what a board
    /// with no camera wants: the pipeline starts, so the WebRTC control channel exists.
    pub camera: bool,
    /// Frame size and rate, as one name. [`Quality`] says why it is one key and not four.
    pub quality: Quality,
    /// Starting video bitrate, bits per second. Unset follows the quality —
    /// [`Quality::default_bitrate`] — which is what almost every robot wants.
    ///
    /// A *starting* point unless `congestion_control` is `disabled`, which is the one setting that
    /// makes this the rate.
    pub bitrate: Option<u32>,
    /// Whether the send rate adapts to the link, and by what. [`CongestionControl`] has the
    /// trade — it is the largest single CPU consumer in this process.
    pub congestion_control: CongestionControl,
}

impl Default for MediaParams {
    fn default() -> Self {
        Self {
            // On, because a robot with a camera is the case, and a board without one shows a
            // test pattern rather than nothing only if somebody turns this off.
            camera: true,
            quality: Quality::default(),
            bitrate: None,
            // `webrtcsink`'s own default, named rather than inherited: what the element defaults
            // to is a fact about a plugin we ship from a pinned release, and the day it changes
            // should not be the day every robot's send rate changes with it.
            congestion_control: CongestionControl::default(),
        }
    }
}

/// `[detect]` — finding other ducks in the camera.
///
/// **Read by `mediad`, not by `robotd`**, which is a first for this file: the frames are on
/// `mediad`'s tee and perception belongs next to the sensor. It lives here anyway, because this is
/// the file `robotctl configure` edits and a robot has one place where its switches are — a second
/// config file for the second daemon that wants one is how a fleet ends up with settings nobody can
/// find.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct DetectParams {
    /// Off by default. The detector costs a model in the release, ~50 ms of CPU per frame and some
    /// heat; a robot that nothing asks to look for ducks should not be paying for it.
    pub enabled: bool,
    /// Where to look, and therefore *what runs it*: a `.rknn` goes to the NPU, an `.onnx` runs on
    /// the CPU. Absent means the release's own model, NPU first — see [`DetectParams::model`].
    pub model: Option<PathBuf>,
    /// Frames per second to run the detector at.
    ///
    /// **2 Hz is a thermal number, not a taste.** Flat out on a Radxa Zero 3 this reaches 95 °C and
    /// the CPU throttles to 408 MHz, which is a robot that walks badly to see well. Two looks a
    /// second is plenty for "is there a duck over there", and costs about a tenth of one core.
    pub hz: f64,
    /// Confidence a detection needs, against **this** model.
    ///
    /// A quantised model's output tensor carries its own scale, so the number that means 0.9 on the
    /// float model does not mean 0.9 here — the INT8 scores of the shipped model saturate around
    /// 1.4. Tuned on the board, not inherited from training.
    pub threshold: f32,
}

impl Default for DetectParams {
    fn default() -> Self {
        Self {
            enabled: false,
            model: None,
            hz: 2.0,
            threshold: 0.35,
        }
    }
}

impl MediaParams {
    /// The bitrate the daemon will actually start at.
    pub fn bitrate_resolved(&self) -> u32 {
        self.bitrate
            .unwrap_or_else(|| self.quality.default_bitrate())
    }
}

impl DetectParams {
    /// The models to try, best first. Empty when the detector is off.
    ///
    /// **A list, not a choice**, because whether the NPU works is not something this file can know.
    /// The `.rknn` is preferred — it is why the detector is cheap — but a board whose NPU is
    /// disabled in its device tree (which is how Armbian ships the Radxa Zero 3) or which never ran
    /// `setup-npu.sh` has no runtime to load it with. Falling through to the `.onnx` means such a
    /// board still sees, on the CPU, instead of logging one warning and doing nothing for ever.
    ///
    /// An explicit `model` is the operator being specific, so it is tried alone.
    pub fn models(&self) -> Vec<PathBuf> {
        if !self.enabled {
            return Vec::new();
        }
        if let Some(path) = &self.model {
            if is_none_sentinel(path) {
                return Vec::new();
            }
            return vec![path.clone()];
        }
        let release = PathBuf::from(RELEASE_DIR).join("models");
        [
            release.join("duck_detect.rknn"),
            release.join("duck_detect.onnx"),
        ]
        .into_iter()
        .filter(|path| path.exists())
        .collect()
    }
}

/// `[chorale]` — several ducks singing one piece.
///
/// `accept` is **false by default, and that is the whole section.** A chorale is not only a sound:
/// it moves the mouth and it moves the head. A robot that began animating because another robot
/// walked into the room would be doing motion nobody asked for, in someone's living room, and two
/// people's ducks in a café have no business pairing up. Off also means *invisible* rather than
/// visibly declining — a duck that has not opted in puts nothing on the air at all.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct ChoraleParams {
    /// Whether this robot may sing with others at all. `false` — and it is derived rather than
    /// written out, so that the default cannot be changed by editing one word.
    pub accept: bool,
}

/// `[theremin]` — the ToF theremin: what counts as a hand, and where the depth frames come
/// from.
///
/// The interesting field is `statuses`, and it is the reason this section exists at all. ST
/// documents 5 and 9 as "range valid", and a build that believes only those stops seeing a
/// hand at about 30 cm on this sensor — past that a moving hand comes back as 4 or 13,
/// *consistency failed*, carrying a distance that is fine for a pitch. That took a bench
/// session to find, so the set is configurable: a duck whose theremin has a short reach wants
/// more codes in, and one that plays phantom notes at nothing wants fewer.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct ThereminParams {
    /// Master switch. On by default: the instrument still has to be picked up with
    /// `robot.theremin`, so what this turns off is the *ability* to, on a duck where the
    /// feature is unwanted or the sensor is known bad.
    pub enabled: bool,
    /// `tofd`'s depth stream.
    pub socket: PathBuf,
    /// Nearest playable range, metres.
    pub near_m: f64,
    /// Farthest playable range, metres.
    pub far_m: f64,
    /// Fewest zones that make a hand.
    pub min_zones: usize,
    /// ST status bytes whose distance is believed. See the section docs — this is the one
    /// that decides how far the instrument reaches.
    pub statuses: Vec<u8>,
    /// How long a note is held through a sensor dropout, milliseconds. This is what keeps a
    /// flickering zone from chopping a note into gravel.
    pub hold_ms: u64,
}

impl Default for ThereminParams {
    fn default() -> Self {
        let hand = kinematics::hand::Config::default();
        Self {
            enabled: true,
            socket: PathBuf::from(duck_ipc_proto::socket::TOF),
            near_m: hand.near_m,
            far_m: hand.far_m,
            min_zones: hand.min_zones,
            statuses: hand.statuses,
            hold_ms: hand.hold.as_millis() as u64,
        }
    }
}

impl ThereminParams {
    /// The hand-detection config these params describe.
    pub fn hand(&self) -> kinematics::hand::Config {
        kinematics::hand::Config {
            near_m: self.near_m,
            far_m: self.far_m,
            min_zones: self.min_zones,
            statuses: self.statuses.clone(),
            hold: std::time::Duration::from_millis(self.hold_ms),
        }
    }
}

/// `[audio]` — the voice and the microphone. All optional equipment: a robot without a
/// codec (or a bank) walks identically and stays quiet, so nothing here reaches a health
/// verdict.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct AudioParams {
    /// Master switch: no sounds, no mic worker.
    pub enabled: bool,
    /// ALSA playback device — the TLV320AIC3104 codec.
    pub device: String,
    /// Where the per-robot voice bank lives. The release's postinstall renders it there
    /// (`sounds ensure-bank`), seeded from the SoC serial.
    pub bank: PathBuf,
    /// Quack once as the control loop comes up. On by default because on a headless board
    /// it is the audible "robotd is running"; off for anyone who restarts the daemon all
    /// day and would rather it did so quietly.
    pub greet: bool,
    /// Listen for petting on the onboard mic and coo about it. Absent means **off**: the
    /// per-mode resolution the prototype shipped (on for walking) cooed at every incidental
    /// head scratch, which wore thin fast. Set `true` to opt in.
    pub pet_detect: Option<bool>,
    /// The petting classifier. Absent means the release's copy; the literal `"none"`
    /// disables it outright.
    pub pet_model: Option<PathBuf>,
    /// Probability above which petting starts, and below which it ends (hysteresis).
    pub pet_enter_threshold: f32,
    pub pet_exit_threshold: f32,
}

impl Default for AudioParams {
    fn default() -> Self {
        Self {
            enabled: true,
            device: "plughw:aic3104".to_owned(),
            bank: PathBuf::from("/var/lib/robot/sounds"),
            greet: true,
            pet_detect: None,
            pet_model: None,
            pet_enter_threshold: 0.95,
            pet_exit_threshold: 0.85,
        }
    }
}

impl AudioParams {
    /// Whether the mic worker runs, resolved against the drive mode.
    pub fn pet_detect_resolved(&self, _mode: Mode) -> bool {
        // Off unless asked for, in either mode. It used to resolve per mode as the prototype's
        // launcher did (on for walking, off for the roller) — and cooing at every incidental
        // head scratch turned out to be more annoying than charming in daily use. The mode is
        // still passed so flipping this back is a one-line change, not a signature change.
        self.pet_detect.unwrap_or(false)
    }

    /// The capture PCM for the mic worker: the playback device with subdevice 0. Only
    /// appended when the operator has not already spelled a subdevice out — `plughw:aic3104`
    /// in `robotd.toml` is the default and needs it, but the equally natural full spec
    /// `plughw:aic3104,0` would otherwise become `plughw:aic3104,0,0`, which no card
    /// answers to. That lands the worker in its restart loop for the life of the daemon.
    pub fn capture_device(&self) -> String {
        if self.device.contains(',') {
            self.device.clone()
        } else {
            format!("{},0", self.device)
        }
    }

    /// The classifier path, or `None` when disabled with the `"none"` sentinel.
    pub fn pet_model_resolved(&self) -> Option<PathBuf> {
        match &self.pet_model {
            Some(p) if is_none_sentinel(p) => None,
            Some(p) => Some(p.clone()),
            None => Some(PathBuf::from(RELEASE_DIR).join("models/pet_detect.onnx")),
        }
    }
}

/// Which drive configuration this robot runs. One robot, two personalities: legs, or the
/// roller. They differ in policies *and* tuning, so the mode is one switch here rather than
/// six paths an operator has to keep consistent — the prototype's launcher kept two whole
/// command lines for the same reason. Switching is an edit plus `systemctl restart robotd`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Mode {
    #[default]
    Walk,
    Roller,
}

impl Mode {
    pub fn as_str(self) -> &'static str {
        match self {
            Mode::Walk => "walk",
            Mode::Roller => "roller",
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct PolicyParams {
    /// Whether to load a policy at all.
    ///
    /// False means slice 1's behaviour: run the loop, hold the pose, stay healthy. That is a
    /// legitimate configuration — it is the safest thing to be doing while hammering
    /// install/rollback cycles at a bench — and it is distinct from a policy that was wanted
    /// and could not be loaded, which is unhealthy.
    pub enabled: bool,
    /// `walk` (default) or `roller`. Changes which policies load *and* the tuning defaults
    /// below — every unset field resolves per mode, so a roller robot needs one line.
    pub mode: Mode,
    /// Policy paths. Absent means the mode's default inside the release directory, so a
    /// normal update ships them; point one elsewhere to try a build without cutting a
    /// release. The literal `"none"` disables a slot outright — the prototype's convention.
    pub walk: Option<PathBuf>,
    /// Standing policy. Without one the walking policy runs at every velocity.
    pub stand: Option<PathBuf>,
    /// Commanded sit↔stand (posture flag in the twist `vx` slot). Sit toggle, shutdown sit
    /// and the seated-boot rise all need it.
    pub sitstand: Option<PathBuf>,
    /// Phase-scripted ground pick. In roller mode this slot holds the crouch.
    pub ground_pick: Option<PathBuf>,
    pub kick_left: Option<PathBuf>,
    pub kick_right: Option<PathBuf>,
    /// Episodic forward roll. Ships by default in both modes, as the prototype now does.
    pub roulade: Option<PathBuf>,
    /// Scales raw policy output into a joint offset. Absent resolves per mode: 0.9 walking
    /// (the prototype's alpha default), 0.8 roller.
    pub action_scale: Option<f64>,
    pub standing_action_scale: f64,
    /// Standing runs softer, at this fraction of `gain`.
    pub standing_gain_ratio: f64,
    /// Position P gain while running.
    pub gain: u16,
    /// First-order low-pass on the head joint targets, `1.0` = pass-through. Default 0.5
    /// in both modes — the value the alpha policies are *trained* with, so it must match
    /// or transfer degrades. (The roller preset used to ship it off; the prototype rebased
    /// its roller line on the alpha defaults, and this follows.)
    pub head_lowpass: Option<f64>,
    /// Same, for the ten leg joints. Walking default 0.7.
    pub legs_lowpass: Option<f64>,
    /// One ground-pick cycle, seconds. The move ends at the set's `end_phase` (70% of the
    /// cycle, as the prototype does). Absent resolves from the installed set's phase-encoded
    /// entry for this mode, else 4.0 walking, 3.0 roller (the crouch).
    pub ground_pick_period: Option<f64>,
    /// Action scale while the ground pick runs. Absent: the set's entry, else 1.0 walking,
    /// 0.8 roller.
    pub ground_pick_action_scale: Option<f64>,
    /// Gain multiplier while the ground pick runs.
    pub ground_pick_gain_ratio: f64,
    /// The one-shot skills this robot has, in priority order.
    ///
    /// Empty means the built-in three — kicks and roulade, with the numbers they have always
    /// had — so a board that updates onto this keeps working with nothing written. An entry
    /// merges by name: naming `roulade` changes that one, naming anything else adds a skill.
    /// The file stays a list of decisions rather than a copy of the defaults, which is what
    /// `robotctl configure` promises about every other key here.
    #[serde(default, rename = "skill", skip_serializing_if = "Vec::is_empty")]
    pub skills: Vec<SkillDef>,
    /// Scale actions with battery voltage: effective scale × (nominal / measured). The
    /// servos' effective kP tracks their supply, so this holds the robot's response steady
    /// as the pack sags. Off by default, as in the prototype.
    pub voltage_adapt: bool,
    /// Reference voltage for `voltage_adapt` — the supply the gains were identified at.
    pub nominal_voltage: f64,
}

/// The literal that disables an optional policy slot, per the prototype's `--x-policy None`.
///
/// Public because three places need to agree on it: this crate resolving config, `robotctl`
/// accepting `policy load <slot> none`, and `robotd` recognising it on the wire. A second spelling
/// of it somewhere would be a slot that looks disabled in a file and is not.
pub fn is_none_sentinel(path: &std::path::Path) -> bool {
    path.as_os_str().eq_ignore_ascii_case("none")
}

/// A one-shot skill: a network, how long it drives, and what it changes while it does.
///
/// **The thing this replaces was three hardcoded arms that differed in four numbers.** Kicks and
/// roulade wrote the same all-zero command into the same kind of window; they differed in
/// duration, action scale, gain ratio, and whether holding the button chained another. A
/// community policy like `polite-bow` — zero command, four seconds, selecting it is the trigger —
/// is a fifth set of the same four numbers, and could not be added without a daemon release.
///
/// Deliberately *only* the zero-command family. `walk` and `stand` are the fallback pair chosen
/// by command magnitude, `sitstand` is latched and driven internally by the shutdown sit and the
/// seated-boot rise, and `ground_pick` writes a scripted phase rather than a constant. Those stay
/// where they are until something needs them not to; see `docs/ideas/policy-moves.md`.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SkillDef {
    /// What a client asks for — `robot.do {skill: "roulade"}` — and what a pad button binds to.
    pub name: String,

    /// The `.onnx`. Absent means this robot's own copy, by convention `<name>.onnx` in the
    /// policy directory, so a built-in needs no path and a fetched one carries the path
    /// `robotctl policy load` wrote.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub path: Option<PathBuf>,

    /// Seconds the network drives before handing back to walk or stand — or, when `unwind_s`
    /// is set, before it starts coming back.
    pub duration: f64,

    /// The twist this skill's network is fed while it runs.
    ///
    /// Zeros for most of them, which is what a policy trained with `zero_command_padding`
    /// expects and what made kicks, roulade and `polite-bow` the same arm. A non-zero constant
    /// is how a policy with its own command encoding becomes a one-shot: the published flamingo
    /// reads `[flag, side, 0]`, so `[1, 1, 0]` is "lift the left leg".
    ///
    /// Head and body stay zeroed either way. Every one-shot published so far declares them
    /// unused, and a skill that wanted them live would be a different shape than this.
    #[serde(default)]
    pub command: [f64; 3],

    /// The twist fed for `unwind_s` after the window, before handing back.
    ///
    /// **This is what lets a policy with no ending of its own be a one-shot.** An episodic
    /// policy returns itself to a safe pose — `polite-bow` is standing again after its four
    /// seconds — so handing straight back to walk is fine. A perpetual one does not: it holds
    /// until told otherwise, and handing back mid-hold gives walk a robot balanced on one foot.
    /// Driving the idle command for a moment first is the daemon supplying the ending the policy
    /// does not have, which is exactly what the sit toggle already does on its way up.
    #[serde(default)]
    pub unwind: [f64; 3],

    /// Seconds spent on `unwind` before handing back. Zero — the default — means the policy ends
    /// itself and the window simply expires.
    #[serde(default)]
    pub unwind_s: f64,

    /// Whether a request arriving while it runs starts another when this one finishes — how a
    /// client maps "the button is held" onto a one-shot. Roulade does; a kick does not.
    #[serde(default)]
    pub chain: bool,

    /// What this skill changes about the robot while it runs, and only while it runs.
    #[serde(default)]
    pub params: SkillOverrides,
}

/// Parameters a skill changes for its duration, restored when it ends.
///
/// **Raw parameter names, not a vocabulary of our own.** `cmd_alpha` rather than an invented
/// `smoothing = "off"`: one set of names for a person to learn, and `robotctl configure` already
/// documents every one of them.
///
/// **No fall-gate override here either, and for a sharper reason: a running skill already has
/// the fall reflex switched off.** The limp-fall predictor is only consulted while the
/// controller is not `busy()`, and any active skill makes it busy — so a move that leans past
/// the gate cannot trip it, and a field to raise the gate would have been decoration. What that
/// *does* mean is that the reflex is off for the whole of a skill, which was uncontroversial for
/// a half-second kick and is worth a second look for anything long.
///
/// **No `cmd_alpha` here, and its absence is the point.** Smoothing is applied to the *client's*
/// command on its way in, and a skill never reads that: the loop builds a fresh command block
/// from `command` or `unwind` and feeds it straight to the network. So a skill's twist is
/// unsmoothed by construction — which is exactly what a policy reading a flag rather than a
/// velocity needs, and the reason driving one through `robot.move` into the walk slot needed
/// `cmd_alpha = 1.0` set globally and remembered afterwards.
///
/// **A named set rather than "any key".** A skill that could widen a joint limit or lengthen the
/// deadman would be reaching past the layer that makes a stranger's policy safe to try at all —
/// which is the entire argument for allowing one on the robot. Everything here is tuning or a
/// threshold that a *move* legitimately owns for its own duration.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct SkillOverrides {
    /// Scales raw policy output into a joint offset, as `[policy] action_scale` does.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub action_scale: Option<f64>,

    /// Multiplier on the running gain, so a move can be softer or stiffer than the gait.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gain_ratio: Option<f64>,
}

impl SkillDef {
    /// The file this skill runs, resolved the way every other policy path resolves.
    pub fn resolved_path(&self) -> Option<PathBuf> {
        match &self.path {
            Some(p) if is_none_sentinel(p) => None,
            Some(p) => Some(p.clone()),
            None => Some(PathBuf::from(POLICY_DIR).join(format!("{}.onnx", self.name))),
        }
    }
}

/// What the official policy set says about itself, installed beside the `.onnx` files.
///
/// The set is fetched from the Hub and versioned there, so what it contains — and how long each
/// one-shot runs — is a property of the set rather than of this build. Without this, adding a
/// tenth policy to the set meant a daemon release: one edit to the seeder's download list so it
/// arrives, and another here so it is a skill. That is the same coupling this whole exercise
/// removed for a stranger's policy, still in place for our own.
///
/// Absent is normal and not an error: a board seeded before the set carried one, or one where
/// the fetch has not happened yet. The three built-ins below are the fallback.
#[derive(Debug, Clone, Default, serde::Deserialize)]
#[serde(default)]
pub struct SetManifest {
    pub policies: Vec<SetPolicy>,
}

/// One policy in the official set.
///
/// **The same field names a single-policy repo uses**, plus `file` to say which `.onnx` it
/// describes. That is deliberate: the community convention is one repo per policy with a flat
/// manifest, and the official set is nine policies in one repo. Sharing the vocabulary means
/// asking a publisher to *add fields*, not to adopt a second format, and it means one reader
/// understands both.
#[derive(Debug, Clone, Default, serde::Deserialize)]
#[serde(default)]
pub struct SetPolicy {
    /// The `.onnx`, as it is named in the repo and on disk. The only field a standalone manifest
    /// has no use for, since a repo with one policy has nothing to disambiguate.
    pub file: String,
    /// What a client asks for, when this is a one-shot. Absent means the file's stem, so
    /// `roulade.onnx` needs no name while `ball_kick_left.onnx` says `kick_left` — the names are
    /// roles and the files are training runs, an indirection worth keeping.
    pub name: Option<String>,
    /// `"episodic"`, `"perpetual"` or `"scripted"`, and the difference is who supplies the
    /// ending.
    ///
    /// An episodic policy runs for `duration_s` and returns itself to a safe pose. On a constant
    /// command it is a skill on its own; on a phase command (`command.encoding = "phase"`) it is
    /// the ground pick — the daemon writes the phase, and the numbers here are how fast. A
    /// perpetual one has no length of its own — how long to hold a foot up is a person's choice,
    /// so it takes a config entry rather than appearing. A scripted one is episodic but
    /// interruptible: the daemon drives it through a command it can change mid-flight, which is
    /// what the sit↔stand posture flag is.
    pub kind: Option<String>,
    /// Seconds it runs, for a policy that ends itself. For a phase policy this is
    /// `period_s × end_phase`, recorded so a reader need not multiply.
    pub duration_s: Option<f64>,
    /// Whether a request arriving while it runs starts another when this one finishes — how a
    /// client maps "the button is held" onto a one-shot.
    pub chain: bool,
    /// Scales raw output into a joint offset, when this policy wants its own.
    pub action_scale: Option<f64>,
    /// Seconds a perpetual policy needs to get back to its idle command — or, for the sit↔stand,
    /// how long the rise on posture flag 0 gets before the gait takes over.
    pub unwind_s: Option<f64>,
    /// Seconds the network takes to reach its commanded posture after the flag flips. The
    /// sit↔stand is trained on a 2 s slewed target, so the seat is a ~2 s glide; the shutdown sit
    /// waits twice that before cutting torque.
    pub ramp_s: Option<f64>,
    /// Which drive mode this policy belongs to. Absent means walking. The roller crouch is the
    /// ground pick of `"roller"` mode, and the two must not be confused with each other.
    pub mode: Option<Mode>,
    /// The command block: how the daemon is meant to drive this network.
    pub command: Option<SetCommand>,
}

/// The machine-readable half of a manifest's command block.
///
/// Three encodings exist in the set, and `encoding` names which one this is:
///
/// - absent or `"constant"`: the skill family — a fixed twist for the window, `idle` on the way
///   back. Every kick, the roulade, and every community one-shot so far.
/// - `"phase"`: `[cos 2πφ, sin 2πφ, 0]` with φ advancing from 0 over `period_s` seconds and the
///   move handing back at `end_phase`. The ground pick, and the roller crouch.
/// - `"posture_flag"`: one slot carries `sit` or `stand`. The sit↔stand.
#[derive(Debug, Clone, Default, serde::Deserialize)]
#[serde(default)]
pub struct SetCommand {
    pub encoding: Option<String>,
    /// The twist that means "stop doing the thing".
    pub idle: Option<[f64; 3]>,
    /// Phase encoding: seconds per full cycle.
    pub period_s: Option<f64>,
    /// Phase encoding: the fraction of the cycle at which the move hands back. The pick's rise
    /// is over well before 1.0, and running to 1.0 replays the reach on the way out.
    pub end_phase: Option<f64>,
    /// Posture flag: the value that means "sit".
    pub sit: Option<f64>,
    /// Posture flag: the value that means "stand".
    pub stand: Option<f64>,
}

/// The ground pick's cycle: what a phase-encoded set entry declares.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PhaseTiming {
    /// Seconds per full cycle.
    pub period_s: f64,
    /// Fraction of the cycle at which the move hands back.
    pub end_phase: f64,
    /// Action scale while it runs, if the entry says.
    pub action_scale: Option<f64>,
}

/// The sit↔stand's timing: what the scripted set entry declares.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SitStandTiming {
    /// Seconds the rise runs on the sitstand network before the gait takes over.
    pub rise_s: f64,
    /// Seconds the seat takes to settle after the flag flips.
    pub ramp_s: f64,
}

impl SetPolicy {
    pub fn skill_name(&self) -> String {
        self.name.clone().unwrap_or_else(|| {
            std::path::Path::new(&self.file)
                .file_stem()
                .map(|s| s.to_string_lossy().into_owned())
                .unwrap_or_else(|| self.file.clone())
        })
    }

    pub fn is_episodic(&self) -> bool {
        self.kind.as_deref() == Some("episodic")
    }

    pub fn is_scripted(&self) -> bool {
        self.kind.as_deref() == Some("scripted")
    }

    fn encoding(&self) -> Option<&str> {
        self.command.as_ref().and_then(|c| c.encoding.as_deref())
    }

    /// The daemon writes this policy's command as a phase. A `period_s` with no `encoding` is
    /// taken the same way — the field has no other meaning.
    pub fn is_phase(&self) -> bool {
        self.encoding() == Some("phase")
            || self.command.as_ref().is_some_and(|c| c.period_s.is_some())
    }

    /// The daemon writes this policy's command as a posture flag.
    pub fn is_posture_flag(&self) -> bool {
        self.encoding() == Some("posture_flag")
    }

    /// The mode this policy belongs to; absent means walking.
    pub fn mode(&self) -> Mode {
        self.mode.unwrap_or_default()
    }

    /// **An episodic policy on a constant command**: what becomes a skill on its own.
    ///
    /// A phase-encoded one does not — it is the ground pick, whose command the daemon generates,
    /// and loading it as a generic one-shot would feed it all-zeros: a robot moving plausibly and
    /// wrongly, which `duck_control::obs`'s header calls the hardest failure to see.
    pub fn is_zero_command_skill(&self) -> bool {
        self.is_episodic() && !self.is_phase() && !self.is_posture_flag()
    }

    /// The phase timing, for an episodic policy the daemon drives through a phase.
    pub fn phase_timing(&self) -> Option<PhaseTiming> {
        if !self.is_episodic() || !self.is_phase() {
            return None;
        }
        let command = self.command.as_ref()?;
        Some(PhaseTiming {
            period_s: command.period_s?,
            end_phase: command.end_phase.unwrap_or(DEFAULT_GROUND_PICK_END_PHASE),
            action_scale: self.action_scale,
        })
    }

    /// The sit↔stand timing, for the scripted posture-flag policy.
    pub fn sitstand_timing(&self) -> Option<SitStandTiming> {
        if !self.is_scripted() || !self.is_posture_flag() {
            return None;
        }
        Some(SitStandTiming {
            rise_s: self.unwind_s.unwrap_or(DEFAULT_SITSTAND_RISE_S),
            ramp_s: self.ramp_s.unwrap_or(DEFAULT_SITSTAND_RAMP_S),
        })
    }
}

impl SetManifest {
    /// The ground pick of one mode, if the set declares it: the phase-encoded episodic entry
    /// tagged with that mode (walking when untagged). The first one wins, and a set that lists
    /// two for the same mode has made a mistake this cannot see.
    pub fn ground_pick(&self, mode: Mode) -> Option<PhaseTiming> {
        self.policies
            .iter()
            .filter(|p| p.mode() == mode)
            .find_map(|p| p.phase_timing())
    }

    /// The sit↔stand's timing, if the set declares it. The sitstand is mode-independent — both
    /// presets load the same network — so the first scripted posture-flag entry is it.
    pub fn sitstand(&self) -> Option<SitStandTiming> {
        self.policies.iter().find_map(|p| p.sitstand_timing())
    }

    /// The entries that are skills on their own: episodic, constant-command, and not named as
    /// something the daemon drives itself.
    ///
    /// **A set cannot claim a name the daemon drives itself.** The ground pick and the sit toggle
    /// live in their own arm of the cascade, and a second entry answering to the same name would
    /// shadow one with a network fed an all-zero command it was never trained on. The manifest
    /// lives on the Hub and cannot be checked from here, so the guard belongs on the board.
    ///
    /// It guards the *name* and the *encoding*, not the file. A set that marks
    /// `alpha_ground_pick.onnx` episodic with neither a name nor a phase command still produces a
    /// skill — called `alpha_ground_pick`, running a phase-scripted network on zeros. That is a
    /// publisher's mistake rather than a trap: it shadows nothing, it is plainly visible in
    /// `robotctl policy list`, and nothing invokes it unless somebody asks for it by that name.
    /// Catching it would mean a hardcoded list of our own filenames, which is the coupling this
    /// whole manifest exists to remove.
    pub fn skills(&self) -> impl Iterator<Item = &SetPolicy> {
        self.policies
            .iter()
            .filter(|p| p.is_zero_command_skill())
            .filter(|p| !DAEMON_OWNED_SKILLS.contains(&p.skill_name().as_str()))
    }
}

/// Skill names the daemon implements itself, which nothing else may take over.
///
/// Public because there are three ways to add a skill — a set's manifest, `robotctl policy add`
/// and `robot.setSkill` — and a list only one of them checks is a guard for one of them. Each
/// has its own arm of the control cascade, so a table entry answering to either name is
/// unreachable: `robot.do` matches the built-in first, and the entry sits in the list being
/// offered and never run.
pub const DAEMON_OWNED_SKILLS: [&str; 2] = ["ground_pick", "sit_toggle"];

/// The ground pick hands back at this fraction of its cycle when the set does not say — the
/// prototype's cutoff. Ending at 100% replays the reach on the way out.
pub const DEFAULT_GROUND_PICK_END_PHASE: f64 = 0.7;
/// How long the sitstand network rises (posture flag 0) before the gait takes over, when the
/// set does not say. 1 s is enough on the robot — velstand owns the tail of the rise fine.
pub const DEFAULT_SITSTAND_RISE_S: f64 = 1.0;
/// How long the seat takes after the flag flips, when the set does not say: the ~2 s glide the
/// sit↔stand is trained on (`POSTURE_RAMP_S`).
pub const DEFAULT_SITSTAND_RAMP_S: f64 = 2.0;

/// Read the installed set's manifest, if it has one.
pub fn set_manifest() -> Option<SetManifest> {
    let text = std::fs::read_to_string(PathBuf::from(POLICY_DIR).join("manifest.json")).ok()?;
    serde_json::from_str(&text).ok()
}

/// The skills a robot has when its config says nothing.
///
/// From the installed set where it says, and from the three below where it does not.
///
/// A board whose set predates the manifest keeps its kicks and its roulade with no config
/// written and no migration run, which is the whole reason absence resolves to something rather
/// than nothing. This goes when every tagged set carries one.
fn builtin_skills(manifest: Option<&SetManifest>) -> Vec<SkillDef> {
    // What the set itself declares. A policy is a skill only if it says it is episodic, drives on
    // a constant command, and how long it runs — a gait is not something to ask for by name, a
    // perpetual one needs a hold length that only a person can choose, and a phase-encoded one
    // is the ground pick.
    if let Some(manifest) = manifest {
        let from_set: Vec<SkillDef> = manifest
            .skills()
            .filter_map(|p| {
                Some(SkillDef {
                    name: p.skill_name(),
                    path: Some(PathBuf::from(POLICY_DIR).join(&p.file)),
                    duration: p.duration_s?,
                    chain: p.chain,
                    unwind: p.command.as_ref().and_then(|c| c.idle).unwrap_or_default(),
                    unwind_s: p.unwind_s.unwrap_or(0.0),
                    params: SkillOverrides {
                        action_scale: p.action_scale,
                        ..Default::default()
                    },
                    ..Default::default()
                })
            })
            .collect();
        if !from_set.is_empty() {
            return from_set;
        }
    }

    fallback_skills()
}

/// The three skills every robot has had since the prototype, for a board whose set says nothing
/// about itself — and the timing a skill slot borrows when it names one the set left out.
fn fallback_skills() -> Vec<SkillDef> {
    let kick = |name: &str, file: &str| SkillDef {
        name: name.to_owned(),
        // The kick files are `ball_kick_*.onnx`, which is not `<name>.onnx` — the names are the
        // roles and the files are the training runs. The set's manifest keeps that indirection
        // in its own `name` field; this is the same thing for a set that predates it.
        path: Some(PathBuf::from(POLICY_DIR).join(file)),
        duration: 0.5,
        chain: false,
        command: [0.0; 3],
        unwind: [0.0; 3],
        unwind_s: 0.0,
        params: SkillOverrides::default(),
    };
    vec![
        // Order is priority, replacing the hardcoded `roulade > kick` precedence.
        SkillDef {
            name: "roulade".to_owned(),
            path: None,
            duration: 1.0,
            // Holding the button chains rolls, which is how the prototype maps a held trigger
            // onto a one-shot.
            chain: true,
            command: [0.0; 3],
            unwind: [0.0; 3],
            unwind_s: 0.0,
            params: SkillOverrides::default(),
        },
        kick("kick_left", "ball_kick_left.onnx"),
        kick("kick_right", "ball_kick_right.onnx"),
    ]
}

fn fallback_skill(name: &str) -> Option<SkillDef> {
    fallback_skills().into_iter().find(|s| s.name == name)
}

/// One policy slot, named — the seven `[policy]` path keys as a value rather than a field name.
///
/// It exists because three places now need to turn the string `"ground_pick"` into *that
/// particular key*: `robot.loadPolicy` on the wire, the `toml_edit` write `robotctl policy load`
/// performs, and the per-slot report `robot.policies` answers with. Spelling the mapping out in
/// each of them is how one of them ends up writing `policy.groundpick` to a file nobody reads
/// back until the robot will not walk.
///
/// [`Slot::as_str`] is the serde key, not a display name, and
/// [`tests::every_slot_is_a_registry_key`] is what keeps that true.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Slot {
    Walk,
    Stand,
    SitStand,
    GroundPick,
    KickLeft,
    KickRight,
    Roulade,
}

impl Slot {
    /// Every slot, in the order `[policy]` lists them and the order a report should print them.
    pub const ALL: [Slot; 7] = [
        Slot::Walk,
        Slot::Stand,
        Slot::SitStand,
        Slot::GroundPick,
        Slot::KickLeft,
        Slot::KickRight,
        Slot::Roulade,
    ];

    /// The slots that are one-shot skills: each names the `[[policy.skill]]` entry of the same
    /// name, and its path is what that skill runs. See `PolicyParams::resolved_skills`.
    pub const SKILLS: [Slot; 3] = [Slot::KickLeft, Slot::KickRight, Slot::Roulade];

    /// The serde key, exactly as `[policy]` spells it.
    pub fn as_str(self) -> &'static str {
        match self {
            Slot::Walk => "walk",
            Slot::Stand => "stand",
            Slot::SitStand => "sitstand",
            Slot::GroundPick => "ground_pick",
            Slot::KickLeft => "kick_left",
            Slot::KickRight => "kick_right",
            Slot::Roulade => "roulade",
        }
    }

    /// `section.key`, which is what the registry and `toml_edit` want.
    pub fn config_key(self) -> String {
        format!("policy.{}", self.as_str())
    }

    /// Parse a slot name off the wire. `None` for anything else, so a caller can refuse with
    /// the list of names it does know rather than failing to deserialize.
    pub fn parse(name: &str) -> Option<Slot> {
        Slot::ALL.into_iter().find(|s| s.as_str() == name)
    }

    /// Every slot name, for the "expected one of" half of a refusal.
    pub fn names() -> String {
        Slot::ALL
            .iter()
            .map(|s| s.as_str())
            .collect::<Vec<_>>()
            .join(", ")
    }
}

impl std::fmt::Display for Slot {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

/// `[policy]` with every absent field resolved against the mode's defaults.
///
/// This is what the rest of `robotd` consumes — nothing downstream should ever have to ask
/// "walk or roller?" to know the action scale.
#[derive(Debug, Clone, PartialEq)]
pub struct ResolvedPolicy {
    pub enabled: bool,
    pub mode: Mode,
    pub walk: PathBuf,
    pub stand: Option<PathBuf>,
    pub sitstand: Option<PathBuf>,
    pub ground_pick: Option<PathBuf>,
    pub kick_left: Option<PathBuf>,
    pub kick_right: Option<PathBuf>,
    pub roulade: Option<PathBuf>,
    pub action_scale: f64,
    pub standing_action_scale: f64,
    pub standing_gain_ratio: f64,
    pub gain: u16,
    pub head_lowpass: Option<f64>,
    pub legs_lowpass: Option<f64>,
    pub ground_pick_period: f64,
    /// Fraction of the cycle at which the ground pick hands back.
    pub ground_pick_end_phase: f64,
    pub ground_pick_action_scale: f64,
    pub ground_pick_gain_ratio: f64,
    /// Seconds the sitstand network rises before the gait takes over.
    pub sitstand_rise_s: f64,
    /// Seconds the seat takes to settle after the flag flips.
    pub sitstand_ramp_s: f64,
    /// The one-shot skills, config merged over the built-ins, in priority order.
    pub skills: Vec<SkillDef>,
    pub voltage_adapt: bool,
    pub nominal_voltage: f64,
}

impl ResolvedPolicy {
    /// The file that will actually be loaded into one slot, after mode defaults are applied.
    /// `None` means the slot is empty — a capability this robot does not have.
    pub fn slot(&self, slot: Slot) -> Option<&std::path::Path> {
        match slot {
            Slot::Walk => Some(self.walk.as_path()),
            Slot::Stand => self.stand.as_deref(),
            Slot::SitStand => self.sitstand.as_deref(),
            Slot::GroundPick => self.ground_pick.as_deref(),
            Slot::KickLeft => self.kick_left.as_deref(),
            Slot::KickRight => self.kick_right.as_deref(),
            Slot::Roulade => self.roulade.as_deref(),
        }
    }
}

impl PolicyParams {
    /// The built-in skills with config merged over them, by name.
    ///
    /// Merge rather than replace, for the reason every other key in this file resolves the way it
    /// does: the file is a list of decisions, not a copy of the defaults. Adding `polite-bow` is
    /// one entry and does not mean re-declaring the three that were already there — and forgetting
    /// to re-declare one cannot silently remove it, which is the failure mode of the other rule.
    ///
    /// A named skill keeps the built-in's position in the priority order; a new one goes last.
    ///
    /// **The three skill slots are applied last and win.** `kick_left`, `kick_right` and `roulade`
    /// are `[policy]` keys like `walk`, and `robotctl policy load roulade <file>` writes that key —
    /// so the file it names has to be the one the `roulade` skill runs, or the load reports a
    /// file the robot never touches, which is what it did. A slot naming a skill the set does not
    /// declare adds it with the built-in's timing; `"none"` switches it off, as it does for a
    /// `[[policy.skill]]` entry.
    pub fn resolved_skills(&self) -> Vec<SkillDef> {
        self.resolved_skills_with(set_manifest().as_ref())
    }

    /// [`Self::resolved_skills`] against a manifest already read — or none, which is the
    /// fallback three.
    pub fn resolved_skills_with(&self, manifest: Option<&SetManifest>) -> Vec<SkillDef> {
        let mut resolved = builtin_skills(manifest);
        for configured in &self.skills {
            match resolved.iter_mut().find(|s| s.name == configured.name) {
                Some(builtin) => *builtin = configured.clone(),
                None => resolved.push(configured.clone()),
            }
        }
        for slot in Slot::SKILLS {
            let Some(path) = self.slot(slot) else {
                continue;
            };
            match resolved.iter_mut().find(|s| s.name == slot.as_str()) {
                Some(skill) => skill.path = Some(path.clone()),
                None => {
                    if let Some(mut skill) = fallback_skill(slot.as_str()) {
                        skill.path = Some(path.clone());
                        resolved.push(skill);
                    }
                }
            }
        }
        // A skill whose path is the `"none"` sentinel is switched off, which is how a built-in is
        // removed without a second mechanism for it.
        resolved.retain(|s| s.resolved_path().is_some());
        resolved
    }

    /// What config says about one slot: `None` unset (resolve the mode's default), `Some("none")`
    /// disabled outright, `Some(path)` an override.
    pub fn slot(&self, slot: Slot) -> &Option<PathBuf> {
        match slot {
            Slot::Walk => &self.walk,
            Slot::Stand => &self.stand,
            Slot::SitStand => &self.sitstand,
            Slot::GroundPick => &self.ground_pick,
            Slot::KickLeft => &self.kick_left,
            Slot::KickRight => &self.kick_right,
            Slot::Roulade => &self.roulade,
        }
    }

    /// Set or clear one slot's override. Clearing is what `robotctl policy reset` does, and it
    /// is why this takes an `Option` rather than having a second method for it.
    pub fn set_slot(&mut self, slot: Slot, path: Option<PathBuf>) {
        let field = match slot {
            Slot::Walk => &mut self.walk,
            Slot::Stand => &mut self.stand,
            Slot::SitStand => &mut self.sitstand,
            Slot::GroundPick => &mut self.ground_pick,
            Slot::KickLeft => &mut self.kick_left,
            Slot::KickRight => &mut self.kick_right,
            Slot::Roulade => &mut self.roulade,
        };
        *field = path;
    }

    pub fn resolved(&self) -> ResolvedPolicy {
        self.resolved_with(set_manifest().as_ref())
    }

    /// [`Self::resolved`] against a manifest already read — or none, which is what a board whose
    /// set predates the manifest has, and resolves to the prototype's numbers.
    ///
    /// **The set says how its own policies run.** The ground pick's cycle and the sit↔stand's rise
    /// used to be literals here, per mode, which meant a retrained pick with a longer cycle was a
    /// daemon release. They come from the set's phase-encoded and posture-flag entries now; the
    /// literals stay as the fallback, and a `[policy]` key still overrides either, because the
    /// file is the list of a person's decisions.
    pub fn resolved_with(&self, manifest: Option<&SetManifest>) -> ResolvedPolicy {
        let release = |name: &str| PathBuf::from(POLICY_DIR).join(name);
        let path = |field: &Option<PathBuf>, default: Option<&str>| -> Option<PathBuf> {
            match field {
                Some(p) if is_none_sentinel(p) => None,
                Some(p) => Some(p.clone()),
                None => default.map(release),
            }
        };

        let (walk_default, stand, sitstand, ground_pick) = match self.mode {
            Mode::Walk => (
                "alpha_walking.onnx",
                Some("alpha_stand.onnx"),
                Some("alpha_sitstand.onnx"),
                Some("alpha_ground_pick.onnx"),
            ),
            // The prototype's roller preset, since rebased on the alpha defaults: roller
            // policy, crouch on the ground-pick trigger, and everything else — sit/stand,
            // kicks, the trained low-pass — as the walking mode has it. `stand` stays
            // unloaded, deliberately: the prototype loads the standing network in roller
            // mode and then skips every standing transition while `roller_mode` is set, so
            // it never runs — not loading it is the same robot without the dead session.
            Mode::Roller => (
                "roller.onnx",
                None,
                Some("alpha_sitstand.onnx"),
                Some("roller_crouch.onnx"),
            ),
        };

        // What each skill slot reports is what the skill of that name will run — derived from the
        // list rather than resolved beside it, so `robot.policies` cannot name a file the robot is
        // not running.
        let skills = self.resolved_skills_with(manifest);
        let skill_file = |name: &str| {
            skills
                .iter()
                .find(|s| s.name == name)
                .and_then(|s| s.resolved_path())
        };

        // The set's own timing for the two networks the daemon drives itself, for this mode.
        let pick = manifest.and_then(|m| m.ground_pick(self.mode));
        let seat = manifest.and_then(|m| m.sitstand());

        ResolvedPolicy {
            enabled: self.enabled,
            mode: self.mode,
            // `walk` is the one slot that cannot be empty — a robot with no walking network has
            // nothing to run — so the `"none"` sentinel does not apply to it and falls back to
            // the mode's default instead.
            //
            // This used to be `.expect("walk always has a default")`, and a config saying
            // `walk = "none"` panicked whichever thread resolved it. That thread is the control
            // loop, so one line in a file killed the robot's control until somebody edited it
            // back — the exact "a bad config line must not brick the board" failure the degraded
            // health rule exists to prevent. `robot.loadPolicy` refuses to write it and
            // `drop_unloadable_overrides` clears it at startup and reports degraded; this is the
            // floor under both.
            walk: path(&self.walk, Some(walk_default))
                .unwrap_or_else(|| PathBuf::from(POLICY_DIR).join(walk_default)),
            stand: path(&self.stand, stand),
            sitstand: path(&self.sitstand, sitstand),
            ground_pick: path(&self.ground_pick, ground_pick),
            kick_left: skill_file("kick_left"),
            kick_right: skill_file("kick_right"),
            roulade: skill_file("roulade"),
            action_scale: self.action_scale.unwrap_or(match self.mode {
                Mode::Walk => 0.9,
                Mode::Roller => 0.8,
            }),
            standing_action_scale: self.standing_action_scale,
            standing_gain_ratio: self.standing_gain_ratio,
            gain: self.gain,
            head_lowpass: Some(self.head_lowpass.unwrap_or(0.5)).filter(|a| *a < 1.0),
            legs_lowpass: Some(self.legs_lowpass.unwrap_or(0.7)).filter(|a| *a < 1.0),
            ground_pick_period: self
                .ground_pick_period
                .or(pick.map(|t| t.period_s))
                .unwrap_or(match self.mode {
                    Mode::Walk => 4.0,
                    Mode::Roller => 3.0,
                }),
            ground_pick_end_phase: pick
                .map(|t| t.end_phase)
                .unwrap_or(DEFAULT_GROUND_PICK_END_PHASE),
            ground_pick_action_scale: self
                .ground_pick_action_scale
                .or(pick.and_then(|t| t.action_scale))
                .unwrap_or(match self.mode {
                    Mode::Walk => 1.0,
                    Mode::Roller => 0.8,
                }),
            ground_pick_gain_ratio: self.ground_pick_gain_ratio,
            sitstand_rise_s: seat.map_or(DEFAULT_SITSTAND_RISE_S, |t| t.rise_s),
            sitstand_ramp_s: seat.map_or(DEFAULT_SITSTAND_RAMP_S, |t| t.ramp_s),
            skills,
            voltage_adapt: self.voltage_adapt,
            nominal_voltage: self.nominal_voltage,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct SafetyParams {
    /// Projected-gravity z above which the robot counts as going down. Upright is about
    /// -1.0; on its side is near 0.
    pub fall_gravity_z: f64,
    /// How long that has to hold. Debounced so a firm footfall is not a fall.
    pub fall_debounce_ms: u64,
    /// Intent age past which the velocity is zeroed. Stop, not limp.
    pub deadman_ms: u64,
    /// The gain limp-fall yields at — low enough to give way rather than fight the floor.
    pub gain_limp: u16,
    /// Sit down and power the machine off when the battery EMA reaches the empty floor
    /// (6.6 V — `duck_control::model::BATTERY_EMPTY_V`). The EMA moves over ~10 s, so a
    /// load sag cannot trip it.
    pub battery_empty_shutdown: bool,

    /// Go limp *while falling*, to land soft instead of fighting the floor all the way
    /// down. **On by default** since it was validated on a robot — the whole point is that
    /// the fleet lands soft, and a mode every board has to opt into individually is a mode
    /// most boards do not have.
    ///
    /// The only thing the daemon does about a fall. Drop to `gain_limp`, let the robot
    /// collapse, pose it back to standing once it has landed, then hand it to the standing
    /// policy — which stands up far more cleanly from a still robot than from one that has
    /// been thrashing since the fall began. With it off, a fall changes nothing: the policy
    /// keeps driving and the humans stay in charge.
    pub limp_fall: bool,
    /// Projected-gravity z the robot must already be past before a fall prediction counts
    /// — about 26° of tilt, which ordinary walking does not reach.
    pub limp_fall_tilt_z: f64,
    /// Where the extrapolation must reach to count as falling. Same sense as
    /// `fall_gravity_z`, and by default the same number.
    pub limp_fall_predict_z: f64,
    /// How far ahead the tilt rate is extrapolated.
    pub limp_fall_lookahead_ms: u64,
    /// How long the fall verdict must hold before the gains drop. Three ticks at 50 Hz —
    /// longer than a footfall impulse, short enough to leave most of the fall to limp
    /// through.
    pub limp_fall_debounce_ms: u64,
    /// Angular-rate magnitude below which the robot counts as having landed, rad/s.
    pub limp_fall_still_rate: f64,
    /// How long it has to stay that still before the limp ends.
    pub limp_fall_still_ms: u64,
    /// Hard cap on the limp, however the landing reads. A robot that never goes still —
    /// held in someone's hands, or resting against something that keeps nudging it —
    /// must not stay limp forever.
    pub limp_fall_max_ms: u64,
    /// How long the ramp back to the standing pose takes, once the robot has landed.
    /// 0.6 s — settled on at the robot. The joints travel across the floor unloaded rather
    /// than lifting anything, so a full second was mostly dead time before the stand-up;
    /// 0.6 keeps some margin over the 0.3 that also worked.
    pub limp_fall_pose_ms: u64,
    /// Gain for that ramp. The joints have to actually travel across the floor, so it is
    /// not the limp gain; it is the softened standing gain rather than the walking one.
    pub limp_fall_pose_gain: u16,
}

impl Default for PolicyParams {
    fn default() -> Self {
        Self {
            enabled: true,
            mode: Mode::Walk,
            walk: None,
            stand: None,
            sitstand: None,
            ground_pick: None,
            kick_left: None,
            kick_right: None,
            roulade: None,
            action_scale: None,
            standing_action_scale: 1.0,
            // The prototype's `--standing-kp-ratio`.
            standing_gain_ratio: 0.8,
            gain: 200,
            head_lowpass: None,
            legs_lowpass: None,
            ground_pick_period: None,
            ground_pick_action_scale: None,
            ground_pick_gain_ratio: 1.0,
            skills: Vec::new(),
            voltage_adapt: false,
            nominal_voltage: 7.4,
        }
    }
}

impl Default for SafetyParams {
    fn default() -> Self {
        Self {
            fall_gravity_z: -0.5,
            fall_debounce_ms: 200,
            deadman_ms: 500,
            gain_limp: 50,
            battery_empty_shutdown: true,
            limp_fall: true,
            limp_fall_tilt_z: -0.90,
            limp_fall_predict_z: -0.5,
            limp_fall_lookahead_ms: 300,
            limp_fall_debounce_ms: 60,
            limp_fall_still_rate: 1.0,
            limp_fall_still_ms: 200,
            limp_fall_max_ms: 1500,
            limp_fall_pose_ms: 600,
            limp_fall_pose_gain: 160,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct Bus {
    /// Serial port the servos and the IMU board share. The Radxa Zero 3W wires them to
    /// `/dev/ttyS2`.
    pub port: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct Control {
    /// Control loop rate. 50 Hz is inherited from the prototype, where it was chosen on a
    /// Pi Zero 2W — re-derive it on the Radxa rather than trusting it.
    pub hz: u32,
    /// Per-tick EMA on the velocity command: `cmd += α × (target − cmd)`. The prototype's
    /// `--cmd-alpha` — what turns a stick snap into a ramp the gait can follow. `1.0` is
    /// pass-through.
    pub cmd_alpha: f64,
    /// Same, for head targets and the body pose.
    pub head_alpha: f64,
}

/// Thresholds that decide `healthy` — and therefore whether an update is kept.
///
/// **Not** the thresholds for everything `robot.health` reports. That answer also describes the
/// battery, the motor temperatures and the loop counters, and none of those may reach a verdict
/// (`docs/design/robotd-design.md` §3.4) — so none of them has a setting here. Naming this section
/// `[health]` invited exactly that mistake: it reads like "how the robot is doing", when what it
/// configures is the one question auto-rollback turns on.
///
/// Everything here is a property of the *software*. A future `[thermal]` section for a motor
/// temperature that should throttle the robot would be a different thing, and belongs under a
/// different name.
///
/// The section was called `[health]`. Renamed outright rather than aliased: a board carrying
/// the old name gets a parse error naming the section, which is a better outcome than a robot
/// quietly running on default thresholds nobody chose.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct UpdateGate {
    /// Below this achieved rate the robot reports unhealthy, which is what makes the
    /// updater's auto-rollback mean something. A loop running at 60% of target is alive,
    /// answers every request, and is badly broken.
    pub min_achieved_hz: f64,
    /// How many periods may pass with no tick before the loop counts as **wedged**.
    ///
    /// This detects a dead loop, not a slow one — `min_achieved_hz` owns degradation. Keep
    /// the two apart: set this near the period and it fires on ordinary scheduler jitter,
    /// which on a loaded board would report a perfectly good release unhealthy and roll it
    /// back. A loop that has not ticked in half a second is genuinely gone; one that
    /// ticked 80 ms late is just late.
    pub stall_periods: u32,
    /// Consecutive bus read failures tolerated before reporting unhealthy. One dropped
    /// transaction is ordinary; a run of them means the bus is gone.
    pub max_consecutive_errors: u32,
}

impl Default for Bus {
    fn default() -> Self {
        Self {
            port: "/dev/ttyS2".into(),
        }
    }
}

impl Default for Control {
    fn default() -> Self {
        Self {
            hz: 50,
            cmd_alpha: 0.2,
            head_alpha: 0.2,
        }
    }
}

impl Default for UpdateGate {
    fn default() -> Self {
        Self {
            // 90% of the default rate. Generous enough not to trip on a slow tick, tight
            // enough that a loop losing every tenth cycle is not called healthy.
            min_achieved_hz: 45.0,
            // 500 ms at the default rate. Deliberately far from the period: three periods
            // is 60 ms, which ordinary scheduler jitter exceeds on a busy machine, and a
            // health check that trips on jitter rolls back good releases.
            stall_periods: 25,
            max_consecutive_errors: 10,
        }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum ParamsError {
    #[error("reading {path}: {source}")]
    Read {
        path: String,
        #[source]
        source: std::io::Error,
    },
    #[error("parsing {path}: {source}")]
    Parse {
        path: String,
        #[source]
        source: toml::de::Error,
    },
    #[error("{path}: control.hz must be between 1 and 1000, got {got}")]
    Rate { path: String, got: u32 },
    #[error(
        "{path}: media.bitrate must be between {min} and {max} bits per second, got {got} — \
         the unit is bits, so 2 Mb/s is 2000000"
    )]
    Bitrate {
        path: String,
        got: u32,
        min: u32,
        max: u32,
    },
}

/// The band `media.bitrate` is accepted in, bits per second.
///
/// The floor is not taste: it is where a typo lands. `bitrate = 2000` is somebody who meant
/// kilobits, and 2 kb/s is a stream that never produces a picture — far better refused at the
/// editor than debugged off a board. The ceiling is what the link and the VPU are for; above it
/// the encoder is being asked for something no robot's wifi will carry.
pub const BITRATE_MIN: u32 = 100_000;
pub const BITRATE_MAX: u32 = 20_000_000;

impl Params {
    /// Load from `path`. A missing file at the *default* location is not an error — an
    /// unprovisioned board should still come up on defaults rather than refuse to start,
    /// and a daemon that will not start is much harder to diagnose remotely than one
    /// running on known defaults. A file explicitly named on the command line must exist.
    pub fn load(path: &Path, explicit: bool) -> Result<Self, ParamsError> {
        let text = match std::fs::read_to_string(path) {
            Ok(text) => text,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound && !explicit => {
                tracing::warn!(path = %path.display(), "no params file; using defaults");
                return Ok(Self::default());
            }
            Err(source) => {
                return Err(ParamsError::Read {
                    path: path.display().to_string(),
                    source,
                });
            }
        };

        // Strict first. A file this build fully understands parses exactly as it always has —
        // one pass, serde's own spans, no second guess — and that is the overwhelmingly common
        // case. The lenient path below only ever runs on a file that has already failed.
        let params = match toml::from_str::<Params>(&text) {
            Ok(params) => params,
            // Not an unknown-key problem: a syntax error, or a value of the wrong type. The
            // strict error is what gets reported, because it is the one carrying a line and a
            // column.
            Err(source) => {
                let Some((reparsed, ignored)) = without_unknown_keys(&text) else {
                    return Err(ParamsError::Parse {
                        path: path.display().to_string(),
                        source,
                    });
                };
                tracing::warn!(
                    path = %path.display(),
                    ignored = %ignored.join(", "),
                    "this build has no such keys; they are ignored and their values do nothing"
                );
                // Pruned, and what is left still does not parse — a real error sharing a file
                // with an inert one. This reports the real one, which costs the position: serde
                // is now looking at a table rather than at the text. The alternative was worse.
                // Returning the strict error here would name the key this release just declared
                // harmless as the reason the daemon will not start, and send whoever reads it to
                // delete a section that was never the problem.
                reparsed.map_err(|source| ParamsError::Parse {
                    path: path.display().to_string(),
                    source,
                })?
            }
        };
        params.validate(path)?;
        Ok(params)
    }

    /// Reject values that would produce a loop that cannot work, at startup rather than as
    /// a division by zero three seconds later.
    fn validate(&self, path: &Path) -> Result<(), ParamsError> {
        if self.control.hz == 0 || self.control.hz > 1000 {
            return Err(ParamsError::Rate {
                path: path.display().to_string(),
                got: self.control.hz,
            });
        }
        // Checked here rather than in `mediad`, so `robotctl configure` refuses to write it:
        // the daemon that would choke on this one is not the daemon whose gate the editor runs.
        if let Some(bitrate) = self.media.bitrate
            && !(BITRATE_MIN..=BITRATE_MAX).contains(&bitrate)
        {
            return Err(ParamsError::Bitrate {
                path: path.display().to_string(),
                got: bitrate,
                min: BITRATE_MIN,
                max: BITRATE_MAX,
            });
        }
        Ok(())
    }

    pub fn period(&self) -> std::time::Duration {
        std::time::Duration::from_secs_f64(1.0 / self.control.hz as f64)
    }
}

/// Re-parse a file that `deny_unknown_fields` rejected, dropping the keys this build has no
/// place for, and say which they were. `None` when nothing was dropped — the parse failed for
/// some other reason and the caller should report that instead.
///
/// **Why unknown keys are no longer fatal.** They were, and the reasoning was sound as far as it
/// went: a silently ignored `min_acheived_hz` leaves an operator believing they moved a threshold
/// they did not. What that argument missed is the other way a key becomes unknown — the build
/// changed underneath a file nobody typed into. A robot running a branch had `[chorale]` in its
/// `robotd.toml`; updating it to a `main` without that feature produced a `robotd` that would not
/// start, four consecutive rollbacks, and a bench session spent on a robot that was fine. The
/// section was inert. Refusing to run over it is a far larger penalty than the mistake it guards
/// against, and it lands on exactly the transitions — a downgrade, a branch, a release that
/// dropped a feature — where the operator did nothing wrong at all.
///
/// So the value is kept and the enforcement moved: every dropped key is named at `warn`, and
/// `robotctl configure` writes only keys the registry knows. What is gone is a robot that will
/// not walk because of a line in a config file that does nothing.
///
/// **The registry is the authority on what a key is**, not serde. It has to be: serde's answer
/// arrives as prose inside an error, and this needs the question asked per key. That is safe
/// because it is not a second copy of the schema —
/// [`registry::tests::the_registry_covers_every_key_exactly`] pins it to [`Params`] in both
/// directions, so a key the registry does not know is a key `Params` does not have.
///
/// `deny_unknown_fields` stays on the structs. It is what makes that test possible, and it is
/// the backstop here: if the registry ever did drift, the pruned table would still be rejected
/// rather than quietly deserialised into something else.
#[allow(clippy::type_complexity)]
fn without_unknown_keys(text: &str) -> Option<(Result<Params, toml::de::Error>, Vec<String>)> {
    let mut table: toml::Table = text.parse().ok()?;
    let mut ignored: Vec<String> = Vec::new();

    table.retain(|section, value| {
        let Some(fields) = value.as_table_mut() else {
            // A bare value at the top level — `hz = 50` written outside any section, which is
            // the shape a hand-edit takes when someone forgets the header. No registry key can
            // name it, and reporting it by its bare name is what tells them why.
            ignored.push(section.to_string());
            return false;
        };
        if !registry::has_section(section) {
            // Reported as the section rather than as each of its keys: `[chorale]` is one
            // decision someone made, not four mistakes.
            ignored.push(format!("[{section}]"));
            return false;
        }
        fields.retain(|key, _| {
            if registry::entry_for(&format!("{section}.{key}")).is_some() {
                true
            } else {
                ignored.push(format!("{section}.{key}"));
                false
            }
        });
        true
    });

    if ignored.is_empty() {
        // Nothing here was an unknown key, so the strict parse failed for a reason this cannot
        // help with. `None` says exactly that, and keeps the caller's two cases apart.
        return None;
    }
    ignored.sort();
    Some((toml::Value::Table(table).try_into::<Params>(), ignored))
}

#[cfg(test)]
mod tests {
    /// [`Slot::as_str`] must be the *serde key*, because `robotctl policy load` writes
    /// `policy.<slot>` into `robotd.toml` with it. A display name that merely reads well —
    /// `sit_stand`, `groundPick` — would write a key `Params` then ignores as unknown, and the
    /// symptom is a load that reports success and changes nothing until the next reboot proves
    /// it never stuck.
    ///
    /// The registry is the right thing to check against rather than `PolicyParams`'s fields,
    /// because it is itself pinned complete against serde's own field list.
    #[test]
    fn every_slot_is_a_registry_key() {
        for slot in super::Slot::ALL {
            let key = slot.config_key();
            let entry = crate::registry::REGISTRY
                .iter()
                .find(|e| e.key == key)
                .unwrap_or_else(|| panic!("{key} is not a key of robotd.toml"));
            assert_eq!(
                entry.kind,
                crate::registry::Kind::OptionalPath,
                "{key} must be a path slot"
            );
        }
    }

    /// **The set says what skills a robot has.** Adding a tenth policy used to mean two edits
    /// in this repository and a daemon release to carry them; it is a tag on the Hub now.
    #[test]
    fn a_set_manifest_decides_which_policies_are_skills() {
        let manifest: super::SetManifest = serde_json::from_value(serde_json::json!({
            "policies": [
                // A gait: perpetual, so not something to ask for by name.
                { "file": "alpha_walking.onnx", "kind": "perpetual" },
                // A perpetual one-shot: no length of its own, so it takes a config entry rather
                // than appearing.
                { "file": "flamingo.onnx", "kind": "perpetual",
                  "unwind_s": 1.5, "command": { "idle": [0, 0, 0] } },
                { "file": "roulade.onnx", "kind": "episodic", "duration_s": 1.0, "chain": true },
                { "file": "ball_kick_left.onnx", "name": "kick_left",
                  "kind": "episodic", "duration_s": 0.5 },
                { "file": "new_trick.onnx", "kind": "episodic", "duration_s": 2.0,
                  "action_scale": 0.8 }
            ]
        }))
        .unwrap();

        let skills: Vec<(&str, f64)> = manifest
            .skills()
            .map(|p| (p.file.as_str(), p.duration_s.unwrap()))
            .collect();
        assert_eq!(
            skills,
            vec![
                ("roulade.onnx", 1.0),
                ("ball_kick_left.onnx", 0.5),
                ("new_trick.onnx", 2.0)
            ],
            "gaits and perpetual one-shots are not skills on their own"
        );
    }

    /// **A set cannot shadow a skill the daemon drives itself.**
    ///
    /// The manifest lives on the Hub, so nothing in this repository can check it before a board
    /// downloads it — the guard has to be on the board. `ground_pick` and `sit_toggle` have their
    /// own arm of the cascade, and a set entry answering to either name would put a second
    /// network behind it, fed an all-zero command it was never trained on.
    ///
    /// The guard is on the name, and this pins what that does and does not cover: a set that
    /// mislabels a scripted policy without renaming it produces a junk skill under its own file
    /// stem. That shadows nothing and is visible in `robotctl policy list`; catching it would
    /// take a hardcoded list of our filenames, which is what this manifest exists to remove.
    #[test]
    fn a_set_cannot_shadow_a_skill_the_daemon_drives() {
        let manifest: super::SetManifest = serde_json::from_value(serde_json::json!({
            "policies": [
                // Named as the daemon's own: shadowing, and refused.
                { "file": "alpha_sitstand.onnx", "name": "sit_toggle",
                  "kind": "episodic", "duration_s": 2.0 },
                // Mislabelled but not renamed: a junk skill, and it is allowed through.
                { "file": "alpha_ground_pick.onnx", "kind": "episodic", "duration_s": 4.0 },
                // Labelled correctly: episodic on a phase command is the ground pick, and the
                // encoding keeps it out of the skill list whatever it is called.
                { "file": "roller_crouch.onnx", "name": "crouch", "kind": "episodic",
                  "duration_s": 3.5, "mode": "roller",
                  "command": { "encoding": "phase", "period_s": 5.0, "end_phase": 0.7 } },
                { "file": "roulade.onnx", "kind": "episodic", "duration_s": 1.0 }
            ]
        }))
        .unwrap();

        let claimed: Vec<String> = manifest.skills().map(|p| p.skill_name()).collect();
        assert_eq!(
            claimed,
            vec!["alpha_ground_pick".to_string(), "roulade".to_string()],
            "sit_toggle is refused, the phase-encoded crouch is the ground pick; the mislabelled \
             one is a visible mistake, not a trap"
        );
    }

    /// A name is the role and a file is the training run, so `ball_kick_left.onnx` answers to
    /// `kick_left` while `roulade.onnx` needs no name at all.
    #[test]
    fn a_set_policy_names_itself_after_its_file_unless_it_says_otherwise() {
        let manifest: super::SetManifest = serde_json::from_value(serde_json::json!({
            "policies": [
                { "file": "roulade.onnx" },
                { "file": "ball_kick_left.onnx", "name": "kick_left" }
            ]
        }))
        .unwrap();
        let names: Vec<String> = manifest.policies.iter().map(|p| p.skill_name()).collect();
        assert_eq!(names, ["roulade", "kick_left"]);
    }

    /// A manifest that says nothing this build understands must not empty the robot. An older
    /// set, or one written by a newer publisher, falls back rather than removing every skill.
    #[test]
    fn a_set_manifest_with_no_skills_falls_back() {
        let manifest: super::SetManifest = serde_json::from_value(serde_json::json!({
            "policies": [{ "file": "alpha_walking.onnx", "kind": "perpetual" }]
        }))
        .unwrap();
        assert!(
            manifest
                .policies
                .iter()
                .all(|p| p.kind.as_deref() != Some("episodic")),
            "nothing here is a skill, so builtin_skills keeps the three it knows"
        );
    }

    /// The manifest the set actually publishes, as this build reads it. One place to see the
    /// whole shape; the tests below take it apart.
    fn published_set() -> super::SetManifest {
        serde_json::from_value(serde_json::json!({
            "schema_version": 1,
            "policies": [
                { "file": "alpha_walking.onnx", "kind": "perpetual" },
                { "file": "alpha_stand.onnx",   "kind": "perpetual" },
                { "file": "roller.onnx",        "kind": "perpetual", "mode": "roller",
                  "action_scale": 0.8 },
                { "file": "alpha_sitstand.onnx", "name": "sitstand", "kind": "scripted",
                  "command": { "encoding": "posture_flag", "slot": "twist.vx",
                               "sit": 1.0, "stand": 0.0, "idle": [0.0, 0.0, 0.0] },
                  "ramp_s": 2.5, "unwind_s": 1.5 },
                { "file": "alpha_ground_pick.onnx", "name": "ground_pick", "kind": "episodic",
                  "duration_s": 2.8,
                  "command": { "encoding": "phase", "slots": "twist.vx,twist.vy",
                               "period_s": 4.0, "end_phase": 0.7 } },
                { "file": "roller_crouch.onnx", "name": "crouch", "kind": "episodic",
                  "duration_s": 3.5, "mode": "roller", "action_scale": 0.8,
                  "command": { "encoding": "phase", "slots": "twist.vx,twist.vy",
                               "period_s": 5.0, "end_phase": 0.7 } },
                { "file": "roulade.onnx",         "kind": "episodic", "duration_s": 1.0,
                  "chain": true },
                { "file": "ball_kick_left.onnx",  "name": "kick_left",  "kind": "episodic",
                  "duration_s": 0.5 },
                { "file": "ball_kick_right.onnx", "name": "kick_right", "kind": "episodic",
                  "duration_s": 0.5 }
            ]
        }))
        .unwrap()
    }

    /// **The set says how fast its own ground pick runs, per mode.** The pick's cycle and the
    /// crouch's were literals here — 4.0 and 3.0 — and the crouch is trained on a 5 s cycle, so
    /// a board ran it at 3 s until somebody noticed. A phase-encoded episodic entry tagged with
    /// a mode is that mode's ground pick, and its numbers are the defaults.
    #[test]
    fn the_set_declares_each_modes_ground_pick() {
        let set = published_set();

        let walk = super::PolicyParams::default().resolved_with(Some(&set));
        assert_eq!(walk.ground_pick_period, 4.0);
        assert_eq!(walk.ground_pick_end_phase, 0.7);
        assert_eq!(
            walk.ground_pick_action_scale, 1.0,
            "the pick says nothing, mode default"
        );

        let roller = super::PolicyParams {
            mode: super::Mode::Roller,
            ..Default::default()
        }
        .resolved_with(Some(&set));
        assert_eq!(
            roller.ground_pick_period, 5.0,
            "the crouch's own cycle, not the literal"
        );
        assert_eq!(roller.ground_pick_action_scale, 0.8);
        assert!(
            roller.ground_pick.unwrap().ends_with("roller_crouch.onnx"),
            "and the slot still loads the crouch"
        );
    }

    /// The file is a list of decisions: a `[policy]` key still beats the set.
    #[test]
    fn a_config_key_overrides_the_sets_ground_pick_timing() {
        let set = published_set();
        let tuned = super::PolicyParams {
            mode: super::Mode::Roller,
            ground_pick_period: Some(6.0),
            ground_pick_action_scale: Some(0.7),
            ..Default::default()
        }
        .resolved_with(Some(&set));
        assert_eq!(tuned.ground_pick_period, 6.0);
        assert_eq!(tuned.ground_pick_action_scale, 0.7);
        assert_eq!(
            tuned.ground_pick_end_phase, 0.7,
            "there is no key for the cutoff"
        );
    }

    /// **The set says how the sit↔stand is timed.** The rise was a literal second and the
    /// shutdown sit a literal four; the scripted posture-flag entry carries both.
    #[test]
    fn the_set_declares_the_sitstands_timing() {
        let set = published_set();
        let resolved = super::PolicyParams::default().resolved_with(Some(&set));
        assert_eq!(resolved.sitstand_rise_s, 1.5);
        assert_eq!(resolved.sitstand_ramp_s, 2.5);
        assert!(
            resolved.sitstand.unwrap().ends_with("alpha_sitstand.onnx"),
            "scripted is recorded, not turned into a skill"
        );
    }

    /// **A phase-encoded or posture-flag policy is never a zero-command skill.** Both are driven
    /// by the daemon through commands it generates; loading either as a generic one-shot would
    /// run it on all-zeros. The published set's skills are exactly the three the prototype had.
    #[test]
    fn the_published_set_yields_the_three_skills_and_nothing_else() {
        let set = published_set();
        let resolved = super::PolicyParams::default().resolved_with(Some(&set));
        let names: Vec<&str> = resolved.skills.iter().map(|s| s.name.as_str()).collect();
        assert_eq!(names, ["roulade", "kick_left", "kick_right"]);
        let roulade = &resolved.skills[0];
        assert_eq!(roulade.duration, 1.0);
        assert!(roulade.chain);
    }

    /// **Absence is the prototype.** No manifest, or one that predates these fields, resolves to
    /// the literals the daemon has always used — a board is never left with a pick that will not
    /// end or a rise that never hands back.
    #[test]
    fn a_set_that_says_nothing_about_timing_leaves_the_prototypes_numbers() {
        let old: super::SetManifest = serde_json::from_value(serde_json::json!({
            "policies": [
                { "file": "alpha_ground_pick.onnx", "kind": "scripted" },
                { "file": "roller_crouch.onnx", "kind": "scripted" },
                { "file": "alpha_sitstand.onnx", "kind": "perpetual" },
                { "file": "roulade.onnx", "kind": "episodic", "duration_s": 1.0, "chain": true }
            ]
        }))
        .unwrap();
        for manifest in [None, Some(&old)] {
            let walk = super::PolicyParams::default().resolved_with(manifest);
            assert_eq!(walk.ground_pick_period, 4.0);
            assert_eq!(
                walk.ground_pick_end_phase,
                super::DEFAULT_GROUND_PICK_END_PHASE
            );
            assert_eq!(walk.ground_pick_action_scale, 1.0);
            assert_eq!(walk.sitstand_rise_s, super::DEFAULT_SITSTAND_RISE_S);
            assert_eq!(walk.sitstand_ramp_s, super::DEFAULT_SITSTAND_RAMP_S);
            let roller = super::PolicyParams {
                mode: super::Mode::Roller,
                ..Default::default()
            }
            .resolved_with(manifest);
            assert_eq!(roller.ground_pick_period, 3.0);
            assert_eq!(roller.ground_pick_action_scale, 0.8);
        }
    }

    /// A phase entry with a `period_s` but no `encoding` is still a phase entry — the field
    /// means nothing else — and one with no `end_phase` hands back at the prototype's cutoff.
    #[test]
    fn a_period_alone_makes_a_phase_entry() {
        let set: super::SetManifest = serde_json::from_value(serde_json::json!({
            "policies": [
                { "file": "alpha_ground_pick.onnx", "kind": "episodic", "duration_s": 3.5,
                  "command": { "period_s": 5.0 } }
            ]
        }))
        .unwrap();
        assert_eq!(
            set.ground_pick(super::Mode::Walk),
            Some(super::PhaseTiming {
                period_s: 5.0,
                end_phase: 0.7,
                action_scale: None
            })
        );
        assert_eq!(set.skills().count(), 0, "not a zero-command skill");
        assert_eq!(
            set.ground_pick(super::Mode::Roller),
            None,
            "untagged means walking"
        );
    }

    /// **A robot with no `[pad]` behaves exactly as it always has.** The mapping is the
    /// prototype's and muscle memory depends on it, so the defaults are not a fresh choice.
    #[test]
    fn the_default_bindings_are_the_prototypes() {
        let pad = super::PadParams::default();
        assert_eq!(pad.a, "ground_pick");
        assert_eq!(pad.x, "roulade");
        assert_eq!(pad.lb, "kick_left");
        assert_eq!(pad.rb, "kick_right");
        assert_eq!(pad.dpad_down, "sit_toggle");
    }

    /// Binding one button leaves the rest alone — the file is a list of decisions, and rebinding
    /// X must not silently take the kicks off the bumpers.
    #[test]
    fn binding_one_button_leaves_the_others() {
        let params: super::Params =
            toml::from_str("[pad]\nx = \"polite-bow\"\n").expect("a pad section");
        assert_eq!(params.pad.x, "polite-bow");
        assert_eq!(params.pad.lb, "kick_left", "untouched");
        assert_eq!(params.pad.a, "ground_pick", "untouched");
    }

    /// An empty binding is a button switched off on purpose, which is different from a button
    /// bound to something that does not exist — `padd` sends nothing rather than a bad name.
    #[test]
    fn an_empty_binding_is_a_button_switched_off() {
        let params: super::Params = toml::from_str("[pad]\ndpad_down = \"\"\n").unwrap();
        assert_eq!(params.pad.skill("dpad_down"), Some(""));
        assert_eq!(params.pad.skill("nonsense"), None, "not a button at all");
    }

    /// Every bindable button must be reachable through the accessors, or `robotctl pad bind`
    /// would refuse a button the config file happily accepts.
    #[test]
    fn every_listed_button_can_be_read_and_bound() {
        let mut pad = super::PadParams::default();
        for button in super::PadParams::BUTTONS {
            assert!(pad.skill(button).is_some(), "{button} is not readable");
            assert!(pad.bind(button, "polite-bow"), "{button} is not bindable");
            assert_eq!(pad.skill(button), Some("polite-bow"));
        }
        assert!(!pad.bind("triangle", "x"), "and nothing else is");
    }

    /// **Absence resolves to the three a robot has always had.** A board updating onto this
    /// writes no config and runs no migration, and still has its kicks and its roulade.
    #[test]
    fn no_configured_skills_means_the_built_in_three() {
        let resolved = super::PolicyParams::default().resolved();
        let names: Vec<&str> = resolved.skills.iter().map(|s| s.name.as_str()).collect();
        assert_eq!(names, vec!["roulade", "kick_left", "kick_right"]);
        assert!(
            resolved.skills.iter().all(|s| s.resolved_path().is_some()),
            "each names a file"
        );
    }

    /// Adding one skill does not mean re-declaring the others — the file is a list of decisions.
    /// A rule where config replaced the lot would make forgetting an entry a silent removal.
    #[test]
    fn a_new_skill_is_added_without_re_declaring_the_built_ins() {
        use std::path::PathBuf;

        let params = super::PolicyParams {
            skills: vec![super::SkillDef {
                name: "polite-bow".into(),
                path: Some(PathBuf::from(
                    "/var/lib/robot/policies/x/y/main/policy.onnx",
                )),
                duration: 4.0,
                chain: false,
                ..Default::default()
            }],
            ..Default::default()
        };

        let names: Vec<String> = params
            .resolved()
            .skills
            .iter()
            .map(|s| s.name.clone())
            .collect();
        assert_eq!(names, ["roulade", "kick_left", "kick_right", "polite-bow"]);
    }

    /// Naming a built-in changes it and keeps its place in the priority order — a retuned
    /// roulade must not become the last thing the cascade considers.
    #[test]
    fn naming_a_built_in_retunes_it_in_place() {
        let params = super::PolicyParams {
            skills: vec![super::SkillDef {
                name: "roulade".into(),
                path: None,
                duration: 2.5,
                chain: false,
                params: super::SkillOverrides {
                    action_scale: Some(0.7),
                    ..Default::default()
                },
                ..Default::default()
            }],
            ..Default::default()
        };

        let resolved = params.resolved();
        assert_eq!(resolved.skills[0].name, "roulade", "still first");
        assert_eq!(resolved.skills[0].duration, 2.5);
        assert_eq!(resolved.skills[0].params.action_scale, Some(0.7));
        assert_eq!(resolved.skills.len(), 3, "and nothing was added");
    }

    /// The `"none"` sentinel removes a built-in, so taking one away needs no second mechanism —
    /// it is the same word that switches off a policy slot.
    #[test]
    fn a_built_in_can_be_switched_off_by_name() {
        use std::path::PathBuf;

        let params = super::PolicyParams {
            skills: vec![super::SkillDef {
                name: "kick_left".into(),
                path: Some(PathBuf::from("none")),
                duration: 0.5,
                chain: false,
                ..Default::default()
            }],
            ..Default::default()
        };

        let names: Vec<String> = params
            .resolved()
            .skills
            .iter()
            .map(|s| s.name.clone())
            .collect();
        assert_eq!(names, ["roulade", "kick_right"]);
    }

    /// A skill with no path runs `<name>.onnx` from this robot's own set, so a built-in needs no
    /// path written and a fetched one carries what `policy load` recorded.
    #[test]
    fn a_skill_without_a_path_runs_its_own_name() {
        let resolved = super::PolicyParams::default().resolved();
        let roulade = &resolved.skills[0];
        assert_eq!(
            roulade.resolved_path().unwrap(),
            std::path::Path::new(super::POLICY_DIR).join("roulade.onnx")
        );
        // The kicks are the exception the built-ins spell out: their files are named for the
        // training run, not the role.
        let kick = resolved
            .skills
            .iter()
            .find(|s| s.name == "kick_left")
            .unwrap();
        assert!(
            kick.resolved_path()
                .unwrap()
                .ends_with("ball_kick_left.onnx"),
            "{:?}",
            kick.resolved_path()
        );
    }

    /// **A skill slot is the file the skill runs.** `robotctl policy load roulade <file>` writes
    /// `[policy] roulade`, and until this the daemon reported that file in the slot while the
    /// `roulade` skill went on running the built-in — a load that changed the report and nothing
    /// else.
    #[test]
    fn a_skill_slot_override_is_what_the_skill_runs() {
        use std::path::PathBuf;

        let params = super::PolicyParams {
            roulade: Some(PathBuf::from("/srv/roll.onnx")),
            kick_left: Some(PathBuf::from("/srv/left.onnx")),
            ..Default::default()
        };
        let resolved = params.resolved();

        let file = |name: &str| {
            resolved
                .skills
                .iter()
                .find(|s| s.name == name)
                .and_then(|s| s.resolved_path())
        };
        assert_eq!(file("roulade"), Some(PathBuf::from("/srv/roll.onnx")));
        assert_eq!(file("kick_left"), Some(PathBuf::from("/srv/left.onnx")));
        assert!(
            file("kick_right")
                .unwrap()
                .ends_with("ball_kick_right.onnx"),
            "an untouched slot keeps the built-in"
        );
        // And the report agrees with the list, in both directions.
        assert_eq!(resolved.roulade, file("roulade"));
        assert_eq!(resolved.kick_left, file("kick_left"));
        assert_eq!(resolved.kick_right, file("kick_right"));
    }

    /// The slot beats a `[[policy.skill]]` entry of the same name: `policy load` is the later,
    /// more explicit decision, and it keeps the entry's timing.
    #[test]
    fn a_skill_slot_overrides_the_entry_path_and_keeps_its_timing() {
        use std::path::PathBuf;

        let params = super::PolicyParams {
            roulade: Some(PathBuf::from("/srv/roll.onnx")),
            skills: vec![super::SkillDef {
                name: "roulade".into(),
                path: Some(PathBuf::from("/srv/other.onnx")),
                duration: 2.5,
                chain: true,
                ..Default::default()
            }],
            ..Default::default()
        };
        let roulade = params
            .resolved()
            .skills
            .into_iter()
            .find(|s| s.name == "roulade")
            .unwrap();
        assert_eq!(roulade.path, Some(PathBuf::from("/srv/roll.onnx")));
        assert_eq!(roulade.duration, 2.5);
    }

    /// `"none"` in a skill slot switches the skill off, exactly as it does in the entry.
    #[test]
    fn a_skill_slot_set_to_none_removes_the_skill() {
        use std::path::PathBuf;

        let params = super::PolicyParams {
            kick_right: Some(PathBuf::from("none")),
            ..Default::default()
        };
        let resolved = params.resolved();
        let names: Vec<&str> = resolved.skills.iter().map(|s| s.name.as_str()).collect();
        assert_eq!(names, ["roulade", "kick_left"]);
        assert_eq!(resolved.kick_right, None, "and the report says so");
    }

    /// **A policy that does not end itself needs the daemon to end it.**
    ///
    /// `polite-bow` is episodic: four seconds later it is standing again, so the window simply
    /// expires and walk takes over a robot that is upright. The published flamingo is not — it
    /// holds a foot up until told otherwise — and handing back mid-hold would give walk a robot
    /// balanced on one leg. `unwind` is the daemon supplying the ending, which is what makes a
    /// perpetual policy usable as a one-shot at all.
    #[test]
    fn a_skill_can_declare_how_it_comes_back() {
        let params = super::PolicyParams {
            skills: vec![super::SkillDef {
                name: "flamingo".into(),
                path: Some(std::path::PathBuf::from("/srv/flamingo.onnx")),
                duration: 5.0,
                chain: false,
                // [flag, side, 0]: lift, then stand back on two feet before handing over.
                command: [1.0, 1.0, 0.0],
                unwind: [0.0, 1.0, 0.0],
                unwind_s: 3.0,
                ..Default::default()
            }],
            ..Default::default()
        };
        let resolved = params.resolved();
        let flamingo = resolved.skills.last().unwrap();
        assert_eq!(flamingo.command, [1.0, 1.0, 0.0]);
        assert_eq!(flamingo.unwind, [0.0, 1.0, 0.0]);
        assert_eq!(flamingo.unwind_s, 3.0);
    }

    /// And the common case declares none of it. A zero command with no unwind is what every
    /// one-shot published so far is, and writing that out would be noise in every config file.
    #[test]
    fn the_built_ins_need_no_command_or_unwind() {
        for skill in super::PolicyParams::default().resolved().skills {
            assert_eq!(skill.command, [0.0; 3], "{} drives on zeros", skill.name);
            assert_eq!(skill.unwind_s, 0.0, "{} ends itself", skill.name);
        }
    }

    /// **A config that disables the walking slot must not panic.**
    ///
    /// It reached a board: `robotctl policy load walk none` wrote `walk = "none"`, and resolving
    /// that killed the control thread — the daemon stayed up answering its socket while the robot
    /// stopped ticking, and a restart panicked again at startup because the file still said it.
    /// One line in a config file must never be able to do that.
    #[test]
    fn disabling_the_walking_slot_falls_back_rather_than_panicking() {
        use super::{Mode, PolicyParams, Slot};
        use std::path::PathBuf;

        for mode in [Mode::Walk, Mode::Roller] {
            let mut params = PolicyParams {
                mode,
                ..Default::default()
            };
            params.set_slot(Slot::Walk, Some(PathBuf::from("none")));
            let resolved = params.resolved();
            assert!(
                resolved.walk.starts_with(super::POLICY_DIR),
                "{mode:?} must fall back to its own walking policy, got {}",
                resolved.walk.display()
            );
        }
    }

    /// Every *other* slot may legitimately be switched off, which is what running a community
    /// policy that owns the whole command block needs. Only `walk` is special.
    #[test]
    fn every_other_slot_can_be_switched_off() {
        use super::{PolicyParams, Slot};
        use std::path::PathBuf;

        let mut params = PolicyParams::default();
        for slot in Slot::ALL {
            params.set_slot(slot, Some(PathBuf::from("none")));
        }
        let resolved = params.resolved();
        for slot in Slot::ALL {
            if slot == Slot::Walk {
                continue;
            }
            assert_eq!(resolved.slot(slot), None, "{slot} must be off");
        }
    }

    /// **The default policy path must not be inside the release directory.**
    ///
    /// That coupling is the whole thing this move undoes: while it held, a gait retrain needed a
    /// daemon release and a daemon fix re-shipped six megabytes of unchanged weights. It would
    /// also come back silently — a default rewritten in terms of `RELEASE_DIR` still resolves to
    /// a real file on a real board, and nothing else would notice.
    #[test]
    fn the_default_policies_live_outside_the_release() {
        let resolved = super::PolicyParams::default().resolved();
        for slot in super::Slot::ALL {
            let Some(path) = resolved.slot(slot) else {
                continue;
            };
            assert!(
                path.starts_with(super::POLICY_DIR),
                "{slot} resolves to {}",
                path.display()
            );
            assert!(
                !path.starts_with(super::RELEASE_DIR),
                "{slot} is back inside the release: {}",
                path.display()
            );
        }
    }

    /// Round-tripping every slot through its own name, so a rename cannot half-land: `parse`
    /// and `as_str` disagreeing would make a slot loadable under a name nothing reports.
    #[test]
    fn slot_names_round_trip() {
        for slot in super::Slot::ALL {
            assert_eq!(super::Slot::parse(slot.as_str()), Some(slot));
        }
        assert_eq!(super::Slot::parse("groundpick"), None);
        assert_eq!(super::Slot::parse(""), None);
    }

    /// The accessors have to agree with `resolved()`, which is the function everything
    /// downstream actually consumes. A `slot()` that read the wrong field would report one
    /// policy while the loop ran another.
    #[test]
    fn slot_accessors_agree_with_the_resolved_paths() {
        use super::{PolicyParams, Slot};
        use std::path::PathBuf;

        let mut params = PolicyParams::default();
        for slot in Slot::ALL {
            params.set_slot(slot, Some(PathBuf::from(format!("/tmp/{slot}.onnx"))));
        }
        let resolved = params.resolved();
        for slot in Slot::ALL {
            assert_eq!(
                params.slot(slot).as_deref(),
                Some(std::path::Path::new(&format!("/tmp/{slot}.onnx"))),
                "{slot} reads back what was set"
            );
            assert_eq!(
                resolved.slot(slot),
                Some(std::path::Path::new(&format!("/tmp/{slot}.onnx"))),
                "{slot} resolves to its override"
            );
        }
    }

    /// Clearing an override must fall back to the mode's default rather than emptying the slot —
    /// that is the whole of `policy reset`, and getting it wrong would leave a robot with no gait
    /// after an undo.
    #[test]
    fn clearing_a_slot_falls_back_to_the_mode_default() {
        use super::{PolicyParams, Slot};
        use std::path::PathBuf;

        let mut params = PolicyParams::default();
        params.set_slot(Slot::Walk, Some(PathBuf::from("/tmp/mine.onnx")));
        assert!(params.resolved().walk.ends_with("mine.onnx"));

        params.set_slot(Slot::Walk, None);
        assert_eq!(
            params.resolved().walk,
            std::path::Path::new(super::POLICY_DIR).join("alpha_walking.onnx"),
            "reset must restore the default, not empty the slot"
        );
    }

    use super::*;

    fn write(dir: &std::path::Path, body: &str) -> std::path::PathBuf {
        let path = dir.join("robotd.toml");
        std::fs::write(&path, body).unwrap();
        path
    }

    /// The capture device is derived from the playback one, and the derivation must be
    /// idempotent: an operator who writes the full ALSA spec gets the device they wrote,
    /// not one with a second subdevice glued on that no card answers to.
    #[test]
    fn the_capture_device_does_not_double_its_subdevice() {
        let plain = AudioParams {
            device: "plughw:aic3104".to_owned(),
            ..AudioParams::default()
        };
        assert_eq!(plain.capture_device(), "plughw:aic3104,0");

        let spelled_out = AudioParams {
            device: "plughw:aic3104,0".to_owned(),
            ..AudioParams::default()
        };
        assert_eq!(spelled_out.capture_device(), "plughw:aic3104,0");
    }

    /// An unprovisioned board must still come up. A daemon that refuses to start because a
    /// config file is absent is far harder to diagnose on a robot than one running on
    /// documented defaults.
    #[test]
    fn a_missing_default_file_falls_back_to_defaults() {
        let dir = tempfile::tempdir().unwrap();
        let p = Params::load(&dir.path().join("absent.toml"), false).unwrap();
        assert_eq!(p.control.hz, 50);
    }

    /// But a file named explicitly on the command line must exist — silently ignoring
    /// `--params /path/typo.toml` would run the robot on settings nobody chose.
    #[test]
    fn an_explicitly_named_missing_file_is_an_error() {
        let dir = tempfile::tempdir().unwrap();
        assert!(Params::load(&dir.path().join("absent.toml"), true).is_err());
    }

    /// Partial files are the normal case — a board overrides the port and nothing else.
    #[test]
    fn absent_sections_take_their_defaults() {
        let dir = tempfile::tempdir().unwrap();
        let path = write(dir.path(), "[bus]\nport = \"/dev/ttyUSB0\"\n");
        let p = Params::load(&path, true).unwrap();
        assert_eq!(p.bus.port, "/dev/ttyUSB0");
        assert_eq!(p.control.hz, 50);
        assert_eq!(p.update_gate.stall_periods, 25);
    }

    /// [`QUALITY_LABELS`] is what the registry offers and what the file may contain, and
    /// [`Quality::ALL`] is what the daemon can do — a rung in one and not the other is either a
    /// choice the editor writes and `mediad` cannot read, or a mode nobody can select.
    #[test]
    fn every_quality_label_round_trips() {
        assert_eq!(QUALITY_LABELS.len(), Quality::ALL.len());
        for (label, quality) in QUALITY_LABELS.iter().zip(Quality::ALL) {
            assert_eq!(*label, quality.label());
            let parsed: Params =
                toml::from_str(&format!("[media]\nquality = \"{label}\"\n")).expect("parses");
            assert_eq!(parsed.media.quality, quality);
        }
    }

    /// The labels are `webrtcsink`'s own property nicknames — the strings that get set on the
    /// element. `gcc`, not `googcc`: a nickname this file spelled its own way would be a config
    /// key that parses, validates, saves, and then silently leaves the element on its default.
    #[test]
    fn every_congestion_label_round_trips() {
        assert_eq!(CONGESTION_LABELS.len(), CongestionControl::ALL.len());
        for (label, mode) in CONGESTION_LABELS.iter().zip(CongestionControl::ALL) {
            assert_eq!(*label, mode.nick());
            let parsed: Params =
                toml::from_str(&format!("[media]\ncongestion_control = \"{label}\"\n"))
                    .expect("parses");
            assert_eq!(parsed.media.congestion_control, mode);
        }
        // `gcc` is webrtcsink's own default, so a robot with no key set must land there — naming
        // it must not change what every robot has been running.
        assert_eq!(
            MediaParams::default().congestion_control,
            CongestionControl::Gcc
        );
    }

    /// The starting bitrate follows the picture unless somebody says otherwise — the whole
    /// reason `bitrate` is optional rather than a number to keep in step by hand.
    #[test]
    fn an_unset_bitrate_follows_the_quality() {
        let mut media = MediaParams::default();
        for quality in Quality::ALL {
            media.quality = quality;
            assert_eq!(media.bitrate_resolved(), quality.default_bitrate());
        }
        media.bitrate = Some(3_000_000);
        assert_eq!(media.bitrate_resolved(), 3_000_000);
    }

    /// A bitrate in the wrong unit is the mistake this band exists to catch: `2000` is somebody
    /// who meant kilobits, and it would produce a stream with no picture in it.
    #[test]
    fn a_bitrate_in_kilobits_is_refused() {
        let dir = tempfile::tempdir().unwrap();
        let path = write(dir.path(), "[media]\nbitrate = 2000\n");
        assert!(Params::load(&path, true).is_err());
        let path = write(dir.path(), "[media]\nbitrate = 2000000\n");
        assert_eq!(
            Params::load(&path, true).unwrap().media.bitrate_resolved(),
            2_000_000
        );
    }

    /// Today's shipped behaviour, pinned: a robot with no `[media]` section streams its camera
    /// at exactly what `mediad`'s flags used to default to. This section changed where those
    /// numbers live and must not have changed the numbers.
    #[test]
    fn the_defaults_are_what_mediad_streamed_before_the_section_existed() {
        let media = Params::default().media;
        assert!(media.camera, "mediad.service carried --camera");
        assert_eq!(media.quality.size(), (1280, 720));
        assert_eq!(media.quality.fps(), 30);
        assert_eq!(media.bitrate_resolved(), 2_000_000);
    }

    /// The shipped example must agree with the built-in defaults, or the file documents a
    /// robot that does not exist — and an operator reading it would draw wrong conclusions
    /// about what their board is actually doing.
    #[test]
    fn the_shipped_example_matches_the_defaults() {
        let shipped = include_str!("../../deploy/robotd.toml");
        let from_file: Params = toml::from_str(shipped).expect("deploy/robotd.toml must parse");
        let built_in = Params::default();

        assert_eq!(from_file.bus.port, built_in.bus.port);
        assert_eq!(from_file.control.hz, built_in.control.hz);
        assert_eq!(from_file.control.cmd_alpha, built_in.control.cmd_alpha);
        assert_eq!(from_file.control.head_alpha, built_in.control.head_alpha);
        assert_eq!(from_file.policy.resolved(), built_in.policy.resolved());
        assert_eq!(from_file.safety.limp_fall, built_in.safety.limp_fall);
        assert_eq!(
            from_file.safety.battery_empty_shutdown,
            built_in.safety.battery_empty_shutdown
        );
        assert_eq!(
            from_file.update_gate.min_achieved_hz,
            built_in.update_gate.min_achieved_hz
        );
        assert_eq!(
            from_file.update_gate.stall_periods,
            built_in.update_gate.stall_periods
        );
        assert_eq!(
            from_file.update_gate.max_consecutive_errors,
            built_in.update_gate.max_consecutive_errors
        );
        assert_eq!(from_file.media.camera, built_in.media.camera);
        assert_eq!(from_file.media.quality, built_in.media.quality);
        assert_eq!(
            from_file.media.bitrate_resolved(),
            built_in.media.bitrate_resolved()
        );
        assert_eq!(
            from_file.media.congestion_control,
            built_in.media.congestion_control
        );
    }

    /// The resolved walk-mode defaults are the prototype's **current alpha configuration**
    /// — the values `microduck_runtime` ships as built-in defaults, which its installer
    /// deliberately passes no flags to override. Changing any of these silently changes how
    /// the robot moves relative to the thing this daemon replaces.
    #[test]
    fn walk_mode_resolves_to_the_prototype_alpha_config() {
        let p = Params::default().policy.resolved();
        assert_eq!(p.mode, Mode::Walk);
        assert_eq!(p.action_scale, 0.9);
        assert_eq!(p.standing_action_scale, 1.0);
        assert_eq!(p.standing_gain_ratio, 0.8, "--standing-kp-ratio");
        assert_eq!(p.gain, 200);
        assert_eq!(
            p.head_lowpass,
            Some(0.5),
            "trained with the filter ON at 0.5"
        );
        assert_eq!(
            p.legs_lowpass,
            Some(0.7),
            "trained with the filter ON at 0.7"
        );
        assert_eq!(p.ground_pick_period, 4.0);
        assert_eq!(p.ground_pick_action_scale, 1.0);
        assert_eq!(p.ground_pick_gain_ratio, 1.0);
        // The three one-shots and their numbers, which used to be four flat keys here.
        let skill = |name: &str| {
            p.skills
                .iter()
                .find(|s| s.name == name)
                .unwrap_or_else(|| panic!("no {name} skill"))
        };
        assert_eq!(skill("kick_left").duration, 0.5);
        assert_eq!(skill("kick_right").duration, 0.5);
        assert_eq!(
            skill("roulade").duration,
            1.0,
            "one roll, the measured time"
        );
        assert!(skill("roulade").chain, "holding the button chains rolls");
        assert!(!skill("kick_left").chain);
        assert!(!p.voltage_adapt, "off by default in the prototype");
        assert_eq!(p.nominal_voltage, 7.4);

        let name = |p: &Option<std::path::PathBuf>| {
            p.as_ref()
                .and_then(|p| p.file_name())
                .map(|n| n.to_string_lossy().into_owned())
        };
        assert_eq!(p.walk, PathBuf::from(POLICY_DIR).join("alpha_walking.onnx"));
        assert_eq!(name(&p.stand).as_deref(), Some("alpha_stand.onnx"));
        assert_eq!(name(&p.sitstand).as_deref(), Some("alpha_sitstand.onnx"));
        assert_eq!(
            name(&p.ground_pick).as_deref(),
            Some("alpha_ground_pick.onnx")
        );
        assert_eq!(name(&p.kick_left).as_deref(), Some("ball_kick_left.onnx"));
        assert_eq!(name(&p.kick_right).as_deref(), Some("ball_kick_right.onnx"));
        assert_eq!(name(&p.roulade).as_deref(), Some("roulade.onnx"));
    }

    /// Command smoothing matches the prototype's `--cmd-alpha` / `--head-alpha`.
    #[test]
    fn command_smoothing_defaults_match_the_prototype() {
        let c = Control::default();
        assert_eq!(c.cmd_alpha, 0.2);
        assert_eq!(c.head_alpha, 0.2);
    }

    /// One line — `mode = "roller"` — must reproduce the prototype's whole roller preset,
    /// which its installer rebased on the alpha defaults: the roller policy and its tuning
    /// (kp 200, scale 0.8, the crouch on the ground-pick trigger at 3 s / 0.8), and
    /// everything else exactly as walking mode has it — sit/stand, kicks, roulade, the
    /// trained low-pass. Only the standing network stays out (the prototype loads it and
    /// then skips every standing transition in roller mode, so it never runs).
    #[test]
    fn roller_mode_resolves_to_the_prototype_roller_preset() {
        let dir = tempfile::tempdir().unwrap();
        let path = write(dir.path(), "[policy]\nmode = \"roller\"\n");
        let p = Params::load(&path, true).unwrap().policy.resolved();

        assert_eq!(p.mode, Mode::Roller);
        assert_eq!(p.walk, PathBuf::from(POLICY_DIR).join("roller.onnx"));
        assert_eq!(
            p.stand, None,
            "the prototype never runs standing in roller mode"
        );
        assert!(
            p.sitstand
                .as_ref()
                .unwrap()
                .ends_with("alpha_sitstand.onnx"),
            "the rebased roller line keeps the sit"
        );
        assert!(
            p.kick_left
                .as_ref()
                .unwrap()
                .ends_with("ball_kick_left.onnx")
        );
        assert!(
            p.kick_right
                .as_ref()
                .unwrap()
                .ends_with("ball_kick_right.onnx")
        );
        assert!(p.roulade.as_ref().unwrap().ends_with("roulade.onnx"));
        assert!(
            p.ground_pick
                .as_ref()
                .unwrap()
                .ends_with("roller_crouch.onnx")
        );
        assert_eq!(p.action_scale, 0.8);
        assert_eq!(p.ground_pick_period, 3.0);
        assert_eq!(p.ground_pick_action_scale, 0.8);
        assert_eq!(
            p.head_lowpass,
            Some(0.5),
            "the rebased roller line keeps the trained filters"
        );
        assert_eq!(p.legs_lowpass, Some(0.7));
        assert_eq!(p.gain, 200);
    }

    /// `"none"` disables an optional slot outright — the prototype's `--sitstand-policy None`
    /// convention — and `1.0` turns a low-pass into a pass-through, which is how its preset
    /// spells "off".
    #[test]
    fn none_and_unity_are_the_off_switches() {
        let dir = tempfile::tempdir().unwrap();
        let path = write(
            dir.path(),
            "[policy]\nsitstand = \"None\"\nhead_lowpass = 1.0\n",
        );
        let p = Params::load(&path, true).unwrap().policy.resolved();
        assert_eq!(p.sitstand, None);
        assert_eq!(
            p.head_lowpass, None,
            "alpha 1.0 is a pass-through, so store it as off"
        );
        assert_eq!(
            p.legs_lowpass,
            Some(0.7),
            "the other filter keeps its default"
        );
    }

    /// A typo in a key is named and ignored, and the setting it was aimed at keeps its default.
    ///
    /// It used to be fatal, on the argument that silently ignoring `min_acheived_hz` leaves the
    /// operator believing they moved a threshold they did not. The value of that is real and is
    /// why the key is named at `warn`; what it does not justify is a robot that will not start.
    /// See [`without_unknown_keys`].
    #[test]
    fn a_typo_is_ignored_and_leaves_the_real_key_alone() {
        let dir = tempfile::tempdir().unwrap();
        let path = write(dir.path(), "[update_gate]\nmin_acheived_hz = 10.0\n");
        let p = Params::load(&path, true).expect("a typo must not stop the robot starting");
        assert_eq!(
            p.update_gate.min_achieved_hz,
            UpdateGate::default().min_achieved_hz,
            "the misspelt key must not have moved the real one"
        );

        let (_, ignored) = without_unknown_keys("[update_gate]\nmin_acheived_hz = 10.0\n")
            .expect("an unknown key is what this file has");
        assert_eq!(ignored, ["update_gate.min_acheived_hz"]);
    }

    /// The renamed section, reported as the section rather than as each key under it.
    ///
    /// `install.sh` never overwrites `robotd.toml`, so a board carrying `[health]` keeps it
    /// across every update. That used to mean a `robotd` that would not start; it now means a
    /// line in the journal naming the section and a robot that walks.
    #[test]
    fn the_old_health_section_name_is_ignored_by_name() {
        let dir = tempfile::tempdir().unwrap();
        let path = write(dir.path(), "[health]\nmin_achieved_hz = 40.0\n");
        assert!(Params::load(&path, true).is_ok());

        let (_, ignored) = without_unknown_keys("[health]\nmin_achieved_hz = 40.0\n").unwrap();
        assert_eq!(ignored, ["[health]"], "one decision, not one line per key");
    }

    /// The incident this came from. A robot running the `duck-chorale` branch had `[chorale]`
    /// in its `robotd.toml`; `main` has no such feature, so the update to it produced a `robotd`
    /// that exited on every start, a health gate that timed out, and four rollbacks in a row —
    /// over a section that does nothing.
    #[test]
    fn a_section_from_another_branch_does_not_stop_the_robot() {
        let dir = tempfile::tempdir().unwrap();
        let path = write(
            dir.path(),
            "[control]\nhz = 50\n\n[chorale]\naccept = true\n",
        );
        let p = Params::load(&path, true).expect("an inert section must not be fatal");
        assert_eq!(p.control.hz, 50, "the rest of the file must still be read");
    }

    /// The other half, so leniency cannot become "accept anything". A value of the wrong type is
    /// still an error, with its position, exactly as before.
    #[test]
    fn a_bad_value_is_still_rejected_with_its_position() {
        let dir = tempfile::tempdir().unwrap();
        let path = write(dir.path(), "[control]\nhz = \"fast\"\n");
        let err = Params::load(&path, true)
            .expect_err("a string where a number belongs is not something to warn about")
            .to_string();
        assert!(err.contains("line"), "{err}");
    }

    /// A real error sharing a file with an inert one. The error must be about `hz`, and must not
    /// be about `[chorale]` — naming the section this release just declared harmless as the
    /// reason the daemon will not start is how someone spends an afternoon deleting the wrong
    /// thing.
    #[test]
    fn a_bad_value_beside_an_unknown_section_names_the_bad_value() {
        let dir = tempfile::tempdir().unwrap();
        let path = write(
            dir.path(),
            "[chorale]\naccept = true\n\n[control]\nhz = \"fast\"\n",
        );
        let err = Params::load(&path, true)
            .expect_err("the file still does not parse")
            .to_string();
        assert!(err.contains("hz"), "{err}");
        assert!(!err.contains("chorale"), "{err}");
    }

    /// And a file that is not TOML at all fails as it always did, rather than being pruned into
    /// something that parses.
    #[test]
    fn a_syntax_error_is_still_a_syntax_error() {
        let dir = tempfile::tempdir().unwrap();
        let path = write(dir.path(), "[control\nhz = 50\n");
        assert!(Params::load(&path, true).is_err());
    }

    /// A file this build understands completely takes the strict path and reports nothing.
    #[test]
    fn a_clean_file_has_nothing_to_report() {
        assert!(without_unknown_keys("[control]\nhz = 50\n").is_none());
    }

    /// Zero would divide by zero when computing the period; absurdly high would spin.
    #[test]
    fn an_impossible_rate_is_rejected_at_startup() {
        let dir = tempfile::tempdir().unwrap();
        for hz in ["0", "5000"] {
            let path = write(dir.path(), &format!("[control]\nhz = {hz}\n"));
            assert!(Params::load(&path, true).is_err(), "hz = {hz} was accepted");
        }
    }
}
