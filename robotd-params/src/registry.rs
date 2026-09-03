//! The machine-readable index of every key in `robotd.toml` — what an editor needs to know.
//!
//! [`Params`] and its `Default` impls are the truth about *values*; what they cannot carry at
//! runtime is the part a human-facing editor needs — a one-line description, what kind of value
//! a key takes, which named choices an enum offers. That is this table.
//!
//! **A table this shape usually drifts, so its completeness is a test, not a hope**:
//! [`tests::the_registry_covers_every_key_exactly`] serializes `Params::default()` and walks the
//! key tree — every leaf must appear here and nothing else may. Add a section to [`Params`] and
//! the build stays green but the test names the keys the registry is missing; the editor can
//! never silently not know about a setting. (The same trick that pins `deploy/robotd.toml` to
//! the defaults.)
//!
//! The one-line docs are deliberately *short* — the full reasoning lives in the doc comments on
//! [`Params`] and in the shipped `deploy/robotd.toml`, and an editor's footer is not the place
//! for four paragraphs on voltage adaptation.

/// What kind of value a key takes — what an editor should offer for it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// On or off. Toggle.
    Bool,
    /// On, off, or absent-meaning-auto (`Option<bool>` — e.g. pet detection resolves per
    /// mode when unset). Cycle through the three.
    TriBool,
    /// A whole number.
    Integer,
    /// A fractional number.
    Float,
    /// A fractional number, or absent meaning "resolved per mode / measured / default".
    OptionalFloat,
    /// A whole number, or absent meaning "follows something else" — `media.bitrate` follows
    /// the quality. Editors show what it resolves to.
    OptionalInteger,
    /// One of a fixed set of names.
    Choice(&'static [&'static str]),
    /// Free text (an ALSA device, a socket path...).
    Text,
    /// A filesystem path, or absent meaning this robot's own copy; the literal `"none"`
    /// disables the slot outright.
    OptionalPath,
    /// A list of whole numbers, edited as comma-separated text ("4, 5, 9").
    IntegerList,
    /// A **repeating table** — `[[policy.skill]]` — rather than a single value.
    ///
    /// Listed here and not editable in place. Not an oversight and not laziness: every other
    /// kind is one value with one cursor position, and a repeating table is a list a person adds
    /// to, removes from and reorders. Rendering that inside a key/value editor would be a worse
    /// tool than the commands that already do it — `robotctl policy` — which the doc line points
    /// at.
    ///
    /// It is *in* the registry so the completeness test keeps meaning what it says: a section
    /// this editor cannot edit is still a section it must know exists, or the next repeating
    /// table added to `Params` goes unnoticed.
    Table,
}

/// One key of `robotd.toml`.
#[derive(Debug, Clone, Copy)]
pub struct Entry {
    /// `section.key`, exactly as serde spells it.
    pub key: &'static str,
    pub kind: Kind,
    /// One line, for an editor's footer. The paragraph lives on the struct field.
    pub doc: &'static str,
    /// A *feature switch* — the handful of keys someone opens an editor to flip, as opposed
    /// to tuning they should read the full docs before touching. Editors list these first.
    pub feature: bool,
}

const fn entry(key: &'static str, kind: Kind, doc: &'static str) -> Entry {
    Entry {
        key,
        kind,
        doc,
        feature: false,
    }
}

const fn feature(key: &'static str, kind: Kind, doc: &'static str) -> Entry {
    Entry {
        key,
        kind,
        doc,
        feature: true,
    }
}

/// Every key, grouped by section, sections in the shipped file's order.
pub const REGISTRY: &[Entry] = &[
    // ── [bus] ────────────────────────────────────────────────────────────────
    entry("bus.port", Kind::Text, "Dynamixel serial port device"),
    // ── [control] ────────────────────────────────────────────────────────────
    entry("control.hz", Kind::Integer, "Control loop rate"),
    entry(
        "control.cmd_alpha",
        Kind::Float,
        "EMA smoothing on stick twist, 1.0 = pass-through",
    ),
    entry(
        "control.head_alpha",
        Kind::Float,
        "EMA smoothing on head targets, 1.0 = pass-through",
    ),
    // ── [update_gate] ────────────────────────────────────────────────────────
    entry(
        "update_gate.min_achieved_hz",
        Kind::Float,
        "Loop rate below which an update is refused as unhealthy",
    ),
    entry(
        "update_gate.stall_periods",
        Kind::Integer,
        "Periods with no tick at all before the loop counts as wedged",
    ),
    entry(
        "update_gate.max_consecutive_errors",
        Kind::Integer,
        "Consecutive bus read failures before reporting unhealthy",
    ),
    // ── [policy] ─────────────────────────────────────────────────────────────
    feature(
        "policy.enabled",
        Kind::Bool,
        "Load policies at all — off holds the pose and stays healthy (bench)",
    ),
    feature(
        "policy.mode",
        Kind::Choice(&["walk", "roller"]),
        "Legs or the roller: picks policies and tuning. Held DPad-Up switches it live; this is \
         the mode a reboot comes back in",
    ),
    entry(
        "policy.skill",
        Kind::Table,
        "One-shot skills, in priority order — add and remove with `robotctl policy`",
    ),
    entry(
        "policy.walk",
        Kind::OptionalPath,
        "Walking policy; unset = this robot's own",
    ),
    entry(
        "policy.stand",
        Kind::OptionalPath,
        "Standing policy; unset = this robot's own",
    ),
    entry(
        "policy.sitstand",
        Kind::OptionalPath,
        "Sit/stand policy; sit toggle and shutdown sit need it",
    ),
    entry(
        "policy.ground_pick",
        Kind::OptionalPath,
        "Ground-pick policy (roller: the crouch)",
    ),
    entry("policy.kick_left", Kind::OptionalPath, "Left-kick policy"),
    entry("policy.kick_right", Kind::OptionalPath, "Right-kick policy"),
    entry("policy.roulade", Kind::OptionalPath, "Forward-roll policy"),
    entry(
        "policy.action_scale",
        Kind::OptionalFloat,
        "Policy output to joint offset; unset resolves per mode",
    ),
    entry(
        "policy.standing_action_scale",
        Kind::Float,
        "Action scale while standing",
    ),
    entry(
        "policy.standing_gain_ratio",
        Kind::Float,
        "Standing runs at this fraction of gain",
    ),
    entry(
        "policy.gain",
        Kind::Integer,
        "Servo position P gain while running",
    ),
    entry(
        "policy.head_lowpass",
        Kind::OptionalFloat,
        "Low-pass on head targets; must match training (0.5)",
    ),
    entry(
        "policy.legs_lowpass",
        Kind::OptionalFloat,
        "Low-pass on leg targets; walking default 0.7",
    ),
    entry(
        "policy.ground_pick_period",
        Kind::OptionalFloat,
        "One ground-pick cycle, seconds; unset resolves per mode",
    ),
    entry(
        "policy.ground_pick_action_scale",
        Kind::OptionalFloat,
        "Action scale during the ground pick; unset resolves per mode",
    ),
    entry(
        "policy.ground_pick_gain_ratio",
        Kind::Float,
        "Gain multiplier during the ground pick",
    ),
    feature(
        "policy.voltage_adapt",
        Kind::Bool,
        "Scale actions with battery voltage (nominal/measured)",
    ),
    entry(
        "policy.nominal_voltage",
        Kind::Float,
        "Reference voltage the gains were identified at",
    ),
    // ── [safety] ─────────────────────────────────────────────────────────────
    entry(
        "safety.fall_gravity_z",
        Kind::Float,
        "Projected-gravity z above which the robot counts as fallen",
    ),
    entry(
        "safety.fall_debounce_ms",
        Kind::Integer,
        "How long past the threshold before the fall verdict holds",
    ),
    entry(
        "safety.deadman_ms",
        Kind::Integer,
        "Intent age past which velocity is zeroed",
    ),
    entry(
        "safety.gain_limp",
        Kind::Integer,
        "The gain limp-fall yields at",
    ),
    feature(
        "safety.battery_empty_shutdown",
        Kind::Bool,
        "Sit and power off when the battery EMA reaches empty",
    ),
    feature(
        "safety.limp_fall",
        Kind::Bool,
        "Go limp while falling, land soft, pose back, hand to standing",
    ),
    entry(
        "safety.limp_fall_tilt_z",
        Kind::Float,
        "Tilt the robot must already be past before a prediction counts",
    ),
    entry(
        "safety.limp_fall_predict_z",
        Kind::Float,
        "Where the extrapolated tilt must reach to count as falling",
    ),
    entry(
        "safety.limp_fall_lookahead_ms",
        Kind::Integer,
        "How far ahead the tilt rate is extrapolated",
    ),
    entry(
        "safety.limp_fall_debounce_ms",
        Kind::Integer,
        "How long the fall verdict holds before the gains drop",
    ),
    entry(
        "safety.limp_fall_still_rate",
        Kind::Float,
        "Angular rate below which the robot counts as landed, rad/s",
    ),
    entry(
        "safety.limp_fall_still_ms",
        Kind::Integer,
        "How long it must stay that still before the limp ends",
    ),
    entry(
        "safety.limp_fall_max_ms",
        Kind::Integer,
        "Hard cap on the limp, however the landing reads",
    ),
    entry(
        "safety.limp_fall_pose_ms",
        Kind::Integer,
        "Ramp back to the standing pose, once landed",
    ),
    entry(
        "safety.limp_fall_pose_gain",
        Kind::Integer,
        "Gain for that ramp — softened standing, not limp",
    ),
    // ── [detect] ─────────────────────────────────────────────────────────────
    feature(
        "detect.enabled",
        Kind::Bool,
        "Look for other ducks in the camera (mediad runs it; needs a restart)",
    ),
    entry(
        "detect.model",
        Kind::OptionalPath,
        "Model to run; unset = the release's, .rknn on the NPU before .onnx on the CPU",
    ),
    entry(
        "detect.hz",
        Kind::Float,
        "Looks per second. 2 is a thermal limit, not a preference — flat out cooks the board",
    ),
    entry(
        "detect.threshold",
        Kind::Float,
        "Confidence a detection needs, on this model's own scale (int8 scores are not 0..1)",
    ),
    // ── [chorale] ────────────────────────────────────────────────────────────
    feature(
        "chorale.accept",
        Kind::Bool,
        "Sing with nearby ducks — off means silent AND invisible on the air",
    ),
    // ── [theremin] ───────────────────────────────────────────────────────────
    feature(
        "theremin.enabled",
        Kind::Bool,
        "The ToF theremin may be picked up at all (robot.theremin still starts it)",
    ),
    entry("theremin.socket", Kind::Text, "tofd's depth stream"),
    entry(
        "theremin.near_m",
        Kind::Float,
        "Nearest playable range, metres",
    ),
    entry(
        "theremin.far_m",
        Kind::Float,
        "Farthest playable range, metres",
    ),
    entry(
        "theremin.min_zones",
        Kind::Integer,
        "Fewest zones that make a hand",
    ),
    entry(
        "theremin.statuses",
        Kind::IntegerList,
        "ToF status bytes believed — this list decides how far the instrument reaches",
    ),
    entry(
        "theremin.hold_ms",
        Kind::Integer,
        "How long a note rides over a sensor dropout, milliseconds",
    ),
    // ── [audio] ──────────────────────────────────────────────────────────────
    feature(
        "audio.enabled",
        Kind::Bool,
        "The voice and the microphone; off walks identically and stays quiet",
    ),
    entry("audio.device", Kind::Text, "ALSA playback device"),
    entry("audio.bank", Kind::Text, "Voice bank directory"),
    feature(
        "audio.greet",
        Kind::Bool,
        "Quack once at startup — the audible \"robotd is running\"",
    ),
    feature(
        "audio.pet_detect",
        Kind::TriBool,
        "Coo when petted; unset means off — an opt-in",
    ),
    entry(
        "audio.pet_model",
        Kind::OptionalPath,
        "Petting classifier; unset = the release's copy",
    ),
    entry(
        "audio.pet_enter_threshold",
        Kind::Float,
        "Petting starts above this probability",
    ),
    entry(
        "audio.pet_exit_threshold",
        Kind::Float,
        "…and ends below this one (hysteresis)",
    ),
    // ── [media] ──────────────────────────────────────────────────────────────
    feature(
        "media.camera",
        Kind::Bool,
        "Stream the head camera — off is a test pattern, for a board with no camera",
    ),
    feature(
        "media.quality",
        Kind::Choice(crate::QUALITY_LABELS),
        "Video frame size and rate; 720p30 is the rung mediad was measured at",
    ),
    entry(
        "media.bitrate",
        Kind::OptionalInteger,
        "Starting video bitrate, bits/s — unset follows the quality",
    ),
    entry(
        "media.congestion_control",
        Kind::Choice(crate::CONGESTION_LABELS),
        "Adapt the send rate to the link — disabled costs adaptivity and saves a core's worth",
    ),
    // ── [pad] ────────────────────────────────────────────────────────────────
    //
    // Which button runs which skill. Read by `padd`, not by `robotd` — but it lives in the same
    // file so `robotctl configure` stays the one editor a person has to know, and so a robot's
    // whole configuration is one thing to back up and one thing to diff.
    feature(
        "pad.a",
        Kind::Text,
        "Skill on the A button — `robotctl policy list` names what this robot has",
    ),
    feature("pad.x", Kind::Text, "Skill on the X button"),
    feature("pad.lb", Kind::Text, "Skill on the left bumper"),
    feature("pad.rb", Kind::Text, "Skill on the right bumper"),
    feature("pad.dpad_down", Kind::Text, "Skill on D-pad down"),
];

/// The registry entry for a key, if it is one.
pub fn entry_for(key: &str) -> Option<&'static Entry> {
    REGISTRY.iter().find(|entry| entry.key == key)
}

/// Does this build have a `[section]` at all?
///
/// Separate from [`entry_for`] because an unknown *section* and an unknown *key inside a known
/// section* deserve different reports: one is a feature this build does not have, the other is
/// almost always a typo of a key next to it. `Params::load` says so in those terms.
pub fn has_section(section: &str) -> bool {
    REGISTRY
        .iter()
        .any(|entry| entry.key.split_once('.').is_some_and(|(s, _)| s == section))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Params;

    /// Every field a section actually has, `Option`s included — extracted from serde's own
    /// unknown-field error, which lists the expected fields by name.
    ///
    /// The trick exists because the honest alternatives cannot see everything: serializing
    /// `Params::default()` omits every `Option` at `None`, so a walk over it would let a new
    /// optional field slip past the completeness test silently. `deny_unknown_fields` is
    /// already on every section, so the rejection message is guaranteed to exist — this just
    /// reads the list serde was going to print anyway.
    fn fields_of(section: &str) -> Vec<String> {
        let probe = format!("[{section}]\n__no_such_key__ = 0\n");
        let error = toml::from_str::<Params>(&probe)
            .expect_err("deny_unknown_fields must reject the probe")
            .to_string();
        let fields: Vec<String> = error
            .split('`')
            .skip(1)
            .step_by(2)
            .filter(|name| *name != "__no_such_key__")
            .map(str::to_owned)
            .collect();
        assert!(
            !fields.is_empty(),
            "serde's error format changed and the coverage test is blind: {error}"
        );
        fields
    }

    /// The drift-proofing this module exists for: the registry names every key `Params`
    /// has — `Option` fields included — and nothing else. Add a field or a section and this
    /// test lists exactly the keys the registry (and so `robotctl configure`) does not know.
    #[test]
    fn the_registry_covers_every_key_exactly() {
        // Sections first, from the top level's own rejection message.
        let top = toml::from_str::<Params>("__no_such_section__ = 0\n")
            .expect_err("unknown sections are rejected")
            .to_string();
        let sections: Vec<String> = top
            .split('`')
            .skip(1)
            .step_by(2)
            .filter(|name| *name != "__no_such_section__")
            .map(str::to_owned)
            .collect();
        // A sanity anchor so a serde message change cannot pass vacuously: the sections this
        // build certainly has must all be found.
        for known in [
            "bus",
            "control",
            "update_gate",
            "policy",
            "safety",
            "audio",
            "media",
        ] {
            assert!(sections.contains(&known.to_owned()), "{top}");
        }

        let registry: Vec<&str> = REGISTRY.iter().map(|entry| entry.key).collect();
        let mut missing: Vec<String> = Vec::new();
        for section in &sections {
            for field in fields_of(section) {
                let key = format!("{section}.{field}");
                if !registry.contains(&key.as_str()) {
                    missing.push(key);
                }
            }
        }
        assert!(
            missing.is_empty(),
            "keys Params has that the registry does not: {missing:?}"
        );
        // Every registry key must be a real key: setting it alone in a TOML must parse. This
        // is what catches a registry entry for a key that was renamed or removed.
        for entry in REGISTRY {
            let (section, key) = entry
                .key
                .split_once('.')
                .expect("registry keys are section.key");
            let probe = match entry.kind {
                Kind::Bool => format!("[{section}]\n{key} = true\n"),
                Kind::TriBool => format!("[{section}]\n{key} = true\n"),
                Kind::Integer | Kind::OptionalInteger => format!("[{section}]\n{key} = 1\n"),
                Kind::Float | Kind::OptionalFloat => format!("[{section}]\n{key} = 0.5\n"),
                Kind::Choice(choices) => {
                    format!("[{section}]\n{key} = \"{}\"\n", choices[0])
                }
                Kind::Text | Kind::OptionalPath => {
                    format!("[{section}]\n{key} = \"probe\"\n")
                }
                Kind::IntegerList => format!("[{section}]\n{key} = [1, 2]\n"),
                // A repeating table's probe is one empty entry — enough to prove the key parses
                // as a table array, which is the thing being asserted.
                Kind::Table => {
                    format!("[[{section}.{key}]]\nname = \"probe\"\nduration = 1.0\n")
                }
            };
            let parsed: Result<Params, _> = toml::from_str(&probe);
            assert!(
                parsed.is_ok(),
                "registry names {:?} but Params rejects it: {probe}",
                entry.key
            );
        }

        // No duplicates, which `entry_for`'s first-match would otherwise hide.
        let mut seen = registry.clone();
        seen.sort_unstable();
        seen.dedup();
        assert_eq!(seen.len(), registry.len(), "a key is listed twice");
    }

    /// Every `Choice` must list what the type actually accepts — and reject what it does not,
    /// so an editor cycling through choices can never write an invalid file.
    #[test]
    fn choices_match_the_types() {
        for entry in REGISTRY {
            let Kind::Choice(choices) = entry.kind else {
                continue;
            };
            let (section, key) = entry.key.split_once('.').expect("section.key");
            for choice in choices {
                let toml = format!("[{section}]\n{key} = \"{choice}\"\n");
                assert!(
                    toml::from_str::<Params>(&toml).is_ok(),
                    "{}: choice {choice:?} is rejected",
                    entry.key
                );
            }
            let toml = format!("[{section}]\n{key} = \"no-such-choice\"\n");
            assert!(
                toml::from_str::<Params>(&toml).is_err(),
                "{}: accepts values beyond its listed choices",
                entry.key
            );
        }
    }

    /// The feature switches are the editor's front page; a typo'd flag would quietly demote a
    /// switch to the tuning list, so pin the set.
    #[test]
    fn the_feature_switches_are_the_expected_set() {
        let features: Vec<&str> = REGISTRY
            .iter()
            .filter(|entry| entry.feature)
            .map(|entry| entry.key)
            .collect();
        assert_eq!(
            features,
            vec![
                "policy.enabled",
                "policy.mode",
                "policy.voltage_adapt",
                "safety.battery_empty_shutdown",
                "safety.limp_fall",
                "detect.enabled",
                "chorale.accept",
                "theremin.enabled",
                "audio.enabled",
                "audio.greet",
                "audio.pet_detect",
                "media.camera",
                "media.quality",
                // The five one-shot buttons. Front-page keys because "what does this button do"
                // is a question somebody asks holding the pad, not while reading tuning docs.
                "pad.a",
                "pad.x",
                "pad.lb",
                "pad.rb",
                "pad.dpad_down",
            ]
        );
    }
}
