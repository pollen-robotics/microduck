//! Emotions: a head, body and beak timeline with a sound, played as a skill.
//!
//! An emotion is two files in `[emotions] dir`, named after it: `sad.json`, the keyframes, and
//! `sad.wav`, the sound, which starts with them. `robot.do sad` plays it, so it goes on a pad
//! button like any skill (`robotctl pad bind y sad`), and adding one is copying two files and
//! restarting `robotd` — no release.
//!
//! **It drives the policy's commands, nothing below them.** While it plays, the head command, the
//! body pose and the mouth come from the keyframes and the twist is zero, exactly as if one client
//! sent `robot.head`, `robot.pose`, `robot.mouth` and a zero `robot.move` every tick — the pad's
//! body + head mode, scripted. So it reaches the motors through the same policy and the same
//! safety as a stick does, and a fall ends it like anything else. The sticks do nothing until it is
//! over; the robot then goes back to whatever the clients last asked for.
//!
//! The keyframes are the format of `pollen-robotics/microduck_emotions`, read as they are:
//!
//! ```json
//! {"keyframes": [{"t": 0.0, "neck": 0.0, "head_pitch": 0.1, "head_yaw": 0.0, "head_roll": 0.0,
//!                 "body_pitch": 0.0, "body_z": 0.0, "mouth": 0.0, "skill": null}, ...]}
//! ```
//!
//! Head values are the head command (radians, `head_pitch` > 0 is beak down), the body pitch and
//! height are the body pose, the mouth is 0..1. A missing field is 0, other fields are ignored, and
//! the emotion ends at the last keyframe. `"skill": "sit"` sits the robot at that keyframe, if it is
//! standing; it stays seated afterwards, and the sit toggle stands it up.

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Instant;

use serde::Deserialize;

use crate::intents::{PoseIntent, Snapshot};

/// One keyframe as the file has it.
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
struct Key {
    t: f64,
    neck: f64,
    head_pitch: f64,
    head_yaw: f64,
    head_roll: f64,
    body_pitch: f64,
    body_z: f64,
    mouth: f64,
    skill: Option<String>,
}

#[derive(Debug, Deserialize)]
struct File {
    keyframes: Vec<Key>,
}

/// What an emotion asks for at one instant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Frame {
    /// Neck pitch, head pitch, head yaw, head roll — the head command's order.
    pub head: [f64; 4],
    /// z, roll, pitch — the body pose's order.
    pub body: [f64; 3],
    pub mouth: f64,
}

impl Frame {
    /// Put this frame in place of what the clients asked for, for this tick only.
    ///
    /// The snapshot rather than the intent slots: a client still sending (the pad heartbeat)
    /// cannot win half the ticks, and when the emotion ends the robot is back on the clients'
    /// own values with nothing to restore. The head reads as freshly steered, so the idle
    /// look-around keeps out of the way. The twist keeps its real age: a fresh one would arm the
    /// deadman's report, and a robot with no pad would then log a lost driver every second after.
    pub fn apply(&self, snapshot: &mut Snapshot) {
        snapshot.command.twist = [0.0; 3];
        snapshot.command.head = self.head;
        snapshot.head_age = std::time::Duration::ZERO;
        snapshot.pose = PoseIntent {
            body: self.body,
            active: true,
        };
        snapshot.mouth = self.mouth;
    }
}

/// One emotion, loaded.
#[derive(Debug)]
pub struct Emotion {
    keys: Vec<Key>,
    /// When it sits the robot down, seconds from its start.
    sit_at: Option<f64>,
    /// Its sound, played from the start. `None` for a silent one.
    sound: Option<PathBuf>,
}

impl Emotion {
    /// Parse one keyframes file. Refused rather than repaired: a file with keyframes out of order
    /// or a value that is not a number would move the head somewhere nobody designed.
    fn parse(text: &str, sound: Option<PathBuf>) -> Result<Self, String> {
        let file: File = serde_json::from_str(text).map_err(|e| e.to_string())?;
        let keys = file.keyframes;
        if keys.is_empty() {
            return Err("no keyframes".to_owned());
        }
        for (i, k) in keys.iter().enumerate() {
            let values = [
                k.t,
                k.neck,
                k.head_pitch,
                k.head_yaw,
                k.head_roll,
                k.body_pitch,
                k.body_z,
                k.mouth,
            ];
            if values.iter().any(|v| !v.is_finite()) {
                return Err(format!("keyframe {i} has a value that is not a number"));
            }
            if i > 0 && k.t <= keys[i - 1].t {
                return Err(format!("keyframe {i} is not after the one before it"));
            }
        }
        let sit_at = keys
            .iter()
            .find(|k| k.skill.as_deref() == Some("sit"))
            .map(|k| k.t);
        Ok(Self {
            keys,
            sit_at,
            sound,
        })
    }

    pub fn sound(&self) -> Option<&Path> {
        self.sound.as_deref()
    }

    /// The frame `t` seconds in, interpolated between keyframes, or `None` past the last one.
    pub fn frame_at(&self, t: f64) -> Option<Frame> {
        let last = self.keys.last()?;
        if t > last.t {
            return None;
        }
        let next = self.keys.partition_point(|k| k.t < t);
        let (a, b) = match next {
            0 => (&self.keys[0], &self.keys[0]),
            i => (&self.keys[i - 1], &self.keys[i]),
        };
        let w = if b.t > a.t {
            (t - a.t) / (b.t - a.t)
        } else {
            0.0
        };
        let lerp = |x: f64, y: f64| x + (y - x) * w;
        Some(Frame {
            head: [
                lerp(a.neck, b.neck),
                lerp(a.head_pitch, b.head_pitch),
                lerp(a.head_yaw, b.head_yaw),
                lerp(a.head_roll, b.head_roll),
            ],
            body: [
                lerp(a.body_z, b.body_z),
                0.0,
                lerp(a.body_pitch, b.body_pitch),
            ],
            mouth: lerp(a.mouth, b.mouth).clamp(0.0, 1.0),
        })
    }
}

/// Every emotion this robot has, by name. Read once at startup, like the policies.
#[derive(Debug, Default)]
pub struct Library(Vec<(String, Arc<Emotion>)>);

impl Library {
    /// Every `*.json` in `dir`, with the `.wav` of the same name when there is one.
    ///
    /// A directory that is not there is a robot with no emotions, not a fault. A file that will
    /// not parse is skipped with a warning naming it, so one bad file does not cost the others.
    pub fn load(dir: &Path) -> Self {
        let entries = match std::fs::read_dir(dir) {
            Ok(entries) => entries,
            Err(e) => {
                if e.kind() != std::io::ErrorKind::NotFound {
                    tracing::warn!(dir = %dir.display(), error = %e, "emotions not read");
                }
                return Self::default();
            }
        };
        let mut files: Vec<PathBuf> = entries
            .filter_map(|e| Some(e.ok()?.path()))
            .filter(|p| p.extension().is_some_and(|x| x == "json"))
            .collect();
        files.sort();
        let mut emotions = Vec::new();
        for path in files {
            let Some(name) = path.file_stem().and_then(|s| s.to_str()) else {
                continue;
            };
            let wav = path.with_extension("wav");
            let sound = wav.is_file().then_some(wav);
            match std::fs::read_to_string(&path)
                .map_err(|e| e.to_string())
                .and_then(|text| Emotion::parse(&text, sound))
            {
                Ok(emotion) => emotions.push((name.to_owned(), Arc::new(emotion))),
                Err(e) => {
                    tracing::warn!(file = %path.display(), error = %e, "emotion skipped")
                }
            }
        }
        if !emotions.is_empty() {
            let names: Vec<&str> = emotions.iter().map(|(n, _)| n.as_str()).collect();
            tracing::info!(dir = %dir.display(), emotions = ?names, "emotions loaded");
        }
        Self(emotions)
    }

    pub fn index(&self, name: &str) -> Option<usize> {
        self.0.iter().position(|(n, _)| n == name)
    }

    pub fn get(&self, index: usize) -> Option<(&str, &Arc<Emotion>)> {
        self.0.get(index).map(|(n, e)| (n.as_str(), e))
    }

    pub fn names(&self) -> impl Iterator<Item = &str> {
        self.0.iter().map(|(n, _)| n.as_str())
    }
}

/// The emotion being played.
#[derive(Debug)]
pub struct Playing {
    pub name: String,
    emotion: Arc<Emotion>,
    since: Instant,
    /// The sit has been dealt with, so it is asked for once.
    sat: bool,
}

impl Playing {
    pub fn start(name: &str, emotion: Arc<Emotion>, now: Instant) -> Self {
        Self {
            name: name.to_owned(),
            emotion,
            since: now,
            sat: false,
        }
    }

    /// This tick's frame, or `None` once it is over.
    pub fn frame(&self, now: Instant) -> Option<Frame> {
        self.emotion
            .frame_at(now.duration_since(self.since).as_secs_f64())
    }

    /// True on the one tick the emotion reaches its sit.
    pub fn sit_due(&mut self, now: Instant) -> bool {
        let due = !self.sat
            && self
                .emotion
                .sit_at
                .is_some_and(|at| now.duration_since(self.since).as_secs_f64() >= at);
        self.sat |= due;
        due
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn emotion(json: &str) -> Emotion {
        Emotion::parse(json, None).expect("valid")
    }

    #[test]
    fn frames_interpolate_between_keyframes_and_end_after_the_last() {
        let e = emotion(
            r#"{"keyframes": [{"t": 0.0, "head_pitch": 0.0, "mouth": 0.0},
                              {"t": 0.1, "head_pitch": 0.4, "mouth": 1.0, "body_pitch": 0.1}]}"#,
        );
        let mid = e.frame_at(0.05).expect("playing");
        assert!((mid.head[1] - 0.2).abs() < 1e-9);
        assert!((mid.mouth - 0.5).abs() < 1e-9);
        assert!((mid.body[2] - 0.05).abs() < 1e-9);
        assert_eq!(e.frame_at(0.1).expect("last keyframe").head[1], 0.4);
        assert_eq!(e.frame_at(0.11), None, "over after the last keyframe");
    }

    /// The format is the emotions repo's, which carries fields this player has no use for
    /// (`twist`, `soften`, `relax`) and older files without `body_z`. Neither may refuse a file.
    #[test]
    fn unknown_fields_are_ignored_and_missing_ones_are_zero() {
        let e = emotion(
            r#"{"step_s": 0.1, "keyframes": [{"t": 0.0, "twist": [0, 0, 0], "soften": false,
                                              "relax": false, "head_yaw": 0.3}]}"#,
        );
        let f = e.frame_at(0.0).expect("playing");
        assert_eq!(f.head, [0.0, 0.0, 0.3, 0.0]);
        assert_eq!(f.body, [0.0; 3]);
    }

    #[test]
    fn a_file_out_of_order_or_empty_is_refused() {
        assert!(Emotion::parse(r#"{"keyframes": []}"#, None).is_err());
        assert!(
            Emotion::parse(r#"{"keyframes": [{"t": 0.2}, {"t": 0.1}]}"#, None).is_err(),
            "keyframes out of order would play backwards"
        );
    }

    /// Asked once, at its keyframe: the sit is a toggle, and a second request would stand the
    /// robot back up.
    #[test]
    fn the_sit_is_due_once_at_its_keyframe() {
        let e = Arc::new(emotion(
            r#"{"keyframes": [{"t": 0.0}, {"t": 0.3, "skill": "sit"}, {"t": 1.0, "skill": "sit"}]}"#,
        ));
        let t0 = Instant::now();
        let mut p = Playing::start("devastated", e, t0);
        assert!(!p.sit_due(t0));
        assert!(p.sit_due(t0 + std::time::Duration::from_millis(300)));
        assert!(!p.sit_due(t0 + std::time::Duration::from_millis(900)));
    }

    #[test]
    fn the_library_reads_a_directory_and_pairs_each_wav() {
        let dir = tempfile::tempdir().expect("tempdir");
        std::fs::write(
            dir.path().join("yes.json"),
            r#"{"keyframes": [{"t": 0.0}]}"#,
        )
        .unwrap();
        std::fs::write(dir.path().join("yes.wav"), b"RIFF").unwrap();
        std::fs::write(
            dir.path().join("quiet.json"),
            r#"{"keyframes": [{"t": 0.0}]}"#,
        )
        .unwrap();
        std::fs::write(dir.path().join("broken.json"), "{").unwrap();
        let library = Library::load(dir.path());
        assert_eq!(library.names().collect::<Vec<_>>(), ["quiet", "yes"]);
        let (_, yes) = library.get(library.index("yes").unwrap()).unwrap();
        assert_eq!(yes.sound(), Some(dir.path().join("yes.wav").as_path()));
        let (_, quiet) = library.get(library.index("quiet").unwrap()).unwrap();
        assert_eq!(quiet.sound(), None);
        assert!(
            Library::load(&dir.path().join("absent"))
                .names()
                .next()
                .is_none()
        );
    }
}
