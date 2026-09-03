//! Editing `robotd.toml` without disturbing it.
//!
//! The lossless writer for this crate's own schema: parse with `toml_edit`, set or remove exactly
//! the keys asked for, re-parse the candidate through [`Params::load`] before anything reaches the
//! disk, and write atomically. Comments, ordering and keys from releases this build does not know
//! all survive.
//!
//! ## Why it is here rather than in `robotctl`
//!
//! It began as part of `robotctl configure`, which was the only thing that wrote this file. Then
//! two more callers appeared: `robotctl policy` and `robotctl pad` write keys of their own, and a
//! daemon serving `pad.bind` over the radio has to write the same file — `padd` re-reads `[pad]`
//! every second, so a binding that only changed something in memory would be reverted before
//! anybody let go of the phone.
//!
//! A second implementation would drift from this one, and the thing it would drift on is the
//! validation: the guarantee worth keeping is that **nothing writes a file `robotd` would refuse
//! to start on**, and that guarantee is only as good as its least careful writer. So the writer
//! lives beside the schema it enforces, in the crate that defines it.
//!
//! `toml_edit` is pure Rust, so this crate's rule that nothing here grows a C toolchain or a
//! network stack still holds — which matters, because this is the crate on the recovery path.
//!
//! ## What is not here
//!
//! Which systemd unit a change needs restarted, how to restart it, and how any of this is
//! presented. Those are a host's business and an operator tool's business, and they stay in
//! `robotctl`.

use std::collections::BTreeMap;
use std::io::Write as _;
use std::path::{Path, PathBuf};

use crate::Params;
use crate::registry::{Entry, Kind, REGISTRY};
use toml_edit::DocumentMut;

/// One key's place in the world: what the file says, what the default is.
#[derive(Debug, Clone)]
pub struct Row {
    pub entry: &'static Entry,
    /// The value in the file, rendered, if the file sets it.
    pub set: Option<String>,
    /// The built-in default, rendered the same way.
    pub default: String,
    /// What an *unset* optional key actually resolves to — per mode, per release — when the
    /// daemon can say. The bare word `unset` told nobody anything.
    pub resolved: Option<String>,
}

impl Row {
    /// What the daemon would actually run with.
    pub fn effective(&self) -> &str {
        self.set.as_deref().unwrap_or(&self.default)
    }

    /// Whether the file overrides the default.
    pub fn overridden(&self) -> bool {
        self.set.is_some()
    }

    /// Whether the value *differs* from the default — the thing worth a colour. A file that
    /// writes the default out explicitly (the shipped example does) is not a divergence.
    pub fn differs(&self) -> bool {
        self.set.as_deref().is_some_and(|set| set != self.default)
    }
}

/// A pending edit: set the key to a value, or clear its override.
#[derive(Debug, Clone)]
pub enum Edit {
    Set(toml_edit::Value),
    Clear,
}

/// The editable state of one file: the parsed document, and the rows over it.
pub struct Model {
    pub path: PathBuf,
    doc: DocumentMut,
    defaults: toml::Value,
    /// Keyed by `section.key`. Applied to the document only on save.
    pub pending: BTreeMap<&'static str, Edit>,
    /// What has actually been written, across every save this session.
    ///
    /// Kept because `pending` is *cleared* by a save, and what wants restarting is decided after
    /// the editor has closed — reading `pending` there found nothing every time, so nothing was
    /// ever restarted and a `[detect]` change looked like a no-op.
    written: Vec<String>,
}

impl Model {
    /// Every key written this session, in the order it was first written.
    pub fn written(&self) -> &[String] {
        &self.written
    }

    /// Load the file — or start from an empty document when there is none, which is a real
    /// state: a robot may run entirely on defaults with no file at all.
    pub fn load(path: &Path) -> Result<Self, String> {
        let text = match std::fs::read_to_string(path) {
            Ok(text) => text,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => String::new(),
            Err(e) => return Err(format!("cannot read {}: {e}", path.display())),
        };
        Self::from_text(path, &text)
    }

    /// A model over text already in hand, for a caller that has read the file itself — and for
    /// testing any caller, which is what it is mostly for: building a config from a string beats
    /// a temporary file for saying what a case is about. [`Model::load`] is this plus the read.
    pub fn from_text(path: &Path, text: &str) -> Result<Self, String> {
        // The daemon's own parse first: a file robotd would refuse is not a file to edit
        // blind, and the error names the line.
        toml::from_str::<Params>(text).map_err(|e| format!("{}: {e}", path.display()))?;
        let doc: DocumentMut = text
            .parse()
            .map_err(|e| format!("{}: {e}", path.display()))?;
        Ok(Self {
            path: path.to_path_buf(),
            doc,
            defaults: toml::Value::try_from(Params::default()).expect("Params serializes"),
            pending: BTreeMap::new(),
            written: Vec::new(),
        })
    }

    /// Every key the daemon knows, in registry order, with pending edits shown as if applied.
    pub fn rows(&self) -> Vec<Row> {
        REGISTRY
            .iter()
            .map(|entry| {
                let set = match self.pending.get(entry.key) {
                    Some(Edit::Set(value)) => Some(render(value)),
                    Some(Edit::Clear) => None,
                    None => self.file_value(entry.key).map(|v| render(&v)),
                };
                let default = self.default_for(entry.key);
                let resolved = (set.is_none() && default == "unset")
                    .then(|| self.resolved_hint(entry.key))
                    .flatten();
                Row {
                    entry,
                    set,
                    default,
                    resolved,
                }
            })
            .collect()
    }

    /// The value the file currently sets for a key, if any.
    fn file_value(&self, key: &str) -> Option<toml_edit::Value> {
        let (section, name) = key.split_once('.').expect("registry keys are section.key");
        self.doc.get(section)?.get(name)?.as_value().cloned()
    }

    /// The built-in default, rendered — `unset` for the Option fields that resolve elsewhere.
    fn default_for(&self, key: &str) -> String {
        let (section, name) = key.split_once('.').expect("registry keys are section.key");
        match self.defaults.get(section).and_then(|s| s.get(name)) {
            Some(toml::Value::String(s)) => s.clone(),
            Some(value) => value.to_string(),
            // Not serialized: an `Option` at `None`. The registry doc says what unset means.
            None => "unset".to_owned(),
        }
    }

    /// What an unset key resolves to, through the daemon's own resolution — per-mode policy
    /// defaults, release-relative paths, the mic's mode-dependent switch. Parsed from the
    /// pending state, so flipping `mode` updates every hint that depends on it.
    fn resolved_hint(&self, key: &str) -> Option<String> {
        let params: Params = toml::from_str(&self.rendered()).ok()?;
        let policy = params.policy.resolved();
        let path = |p: Option<std::path::PathBuf>| {
            Some(match p {
                Some(p) => p.display().to_string(),
                None => "disabled".to_owned(),
            })
        };
        let float = |f: f64| Some(f.to_string());
        match key {
            "policy.walk" => Some(policy.walk.display().to_string()),
            "policy.stand" => path(policy.stand),
            "policy.sitstand" => path(policy.sitstand),
            "policy.ground_pick" => path(policy.ground_pick),
            "policy.kick_left" => path(policy.kick_left),
            "policy.kick_right" => path(policy.kick_right),
            "policy.roulade" => path(policy.roulade),
            "policy.action_scale" => float(policy.action_scale),
            "policy.head_lowpass" => policy.head_lowpass.and_then(float),
            "policy.legs_lowpass" => policy.legs_lowpass.and_then(float),
            "policy.ground_pick_period" => float(policy.ground_pick_period),
            "policy.ground_pick_action_scale" => float(policy.ground_pick_action_scale),
            "media.bitrate" => Some(params.media.bitrate_resolved().to_string()),
            "audio.pet_detect" => Some(
                params
                    .audio
                    .pet_detect_resolved(params.policy.mode)
                    .to_string(),
            ),
            "audio.pet_model" => path(params.audio.pet_model_resolved()),
            _ => None,
        }
    }

    /// Queue an edit, from the string a user typed or a toggle produced.
    ///
    /// Typing the default (or `unset`, for the optional kinds) clears the override instead of
    /// pinning it — a file full of explicitly-written defaults is the unreadable thing this
    /// tool exists to avoid.
    pub fn edit(&mut self, entry: &'static Entry, input: &str) -> Result<(), String> {
        let input = input.trim();
        let optional = matches!(
            entry.kind,
            Kind::TriBool | Kind::OptionalFloat | Kind::OptionalInteger | Kind::OptionalPath
        );
        if input == self.default_for(entry.key)
            || (optional && (input == "unset" || input.is_empty()))
        {
            self.pending.insert(entry.key, Edit::Clear);
            return Ok(());
        }
        let value: toml_edit::Value = match entry.kind {
            Kind::Bool | Kind::TriBool => match input {
                "true" | "on" | "yes" => true.into(),
                "false" | "off" | "no" => false.into(),
                _ => return Err(format!("{input:?} is not on/off")),
            },
            Kind::Integer | Kind::OptionalInteger => input
                .parse::<i64>()
                .map(Into::into)
                .map_err(|_| format!("{input:?} is not a whole number"))?,
            Kind::Float | Kind::OptionalFloat => input
                .parse::<f64>()
                .map(Into::into)
                .map_err(|_| format!("{input:?} is not a number"))?,
            Kind::Choice(choices) => {
                if !choices.contains(&input) {
                    return Err(format!("{input:?} is not one of {choices:?}"));
                }
                input.into()
            }
            Kind::Text | Kind::OptionalPath => input.into(),
            // A repeating table is not one value with one cursor, so this editor lists it and
            // points at the commands that do manage it. See `Kind::Table`.
            Kind::Table => {
                return Err("edit the one-shot skills with `robotctl policy`".to_owned());
            }
            Kind::IntegerList => {
                let mut array = toml_edit::Array::new();
                for word in input.split(',') {
                    let word = word.trim();
                    if word.is_empty() {
                        continue;
                    }
                    let number: i64 = word
                        .parse()
                        .map_err(|_| format!("{word:?} is not a whole number"))?;
                    array.push(number);
                }
                if array.is_empty() {
                    return Err("an empty list — give comma-separated numbers".to_owned());
                }
                array.into()
            }
        };
        self.pending.insert(entry.key, Edit::Set(value));
        Ok(())
    }

    /// The next value a toggle key produces — what SPACE does. `None` for kinds that want
    /// typed input instead.
    pub fn toggled(&self, row: &Row) -> Option<String> {
        match row.entry.kind {
            Kind::Bool => Some(
                if row.effective() == "true" {
                    "false"
                } else {
                    "true"
                }
                .into(),
            ),
            // auto → on → off → auto. `unset` is the auto state.
            Kind::TriBool => Some(match (row.overridden(), row.effective()) {
                (false, _) => "true".into(),
                (true, "true") => "false".into(),
                (true, _) => "unset".into(),
            }),
            Kind::Choice(choices) => {
                let current = row.effective();
                let at = choices.iter().position(|c| *c == current).unwrap_or(0);
                Some(choices[(at + 1) % choices.len()].into())
            }
            _ => None,
        }
    }

    /// The document with every pending edit applied, as text — what save writes.
    ///
    /// Comments and unknown keys survive untouched: `toml_edit` only changes what is set or
    /// removed, and clearing a key removes the key alone, never its section or its comments.
    pub fn rendered(&self) -> String {
        let mut doc = self.doc.clone();
        for (key, edit) in &self.pending {
            let (section, name) = key.split_once('.').expect("section.key");
            match edit {
                Edit::Set(value) => {
                    let table = doc
                        .entry(section)
                        .or_insert(toml_edit::Item::Table(toml_edit::Table::new()));
                    table[name] = toml_edit::Item::Value(value.clone());
                }
                Edit::Clear => {
                    if let Some(table) = doc.get_mut(section).and_then(|i| i.as_table_mut()) {
                        table.remove(name);
                    }
                }
            }
        }
        doc.to_string()
    }

    /// Add or replace one `[[policy.skill]]` entry, by name.
    ///
    /// Separate from [`Self::edit`] because a repeating table is not a key with a value: there
    /// is no cursor position for it, and the editor lists it rather than editing it in place
    /// (`registry::Kind::Table`). This is the writing half, and `robotctl policy` is what calls
    /// it — the same document, the same atomic save, and the same validation through the
    /// daemon's own loader, so the comments and everything else in the file survive.
    ///
    /// By name rather than by appending, so running the same command twice retunes a skill
    /// instead of giving a robot two of it.
    fn put_skill(&mut self, skill: &crate::SkillDef) -> Result<(), String> {
        // Here rather than only in the caller, because a skill can be added from three places
        // and a rule one of them enforces is not a rule. `robot.do` matches the two the daemon
        // drives itself before it looks at the table, so an entry answering to either name is
        // one this file offers and nothing can ever run.
        if crate::DAEMON_OWNED_SKILLS.contains(&skill.name.as_str()) {
            return Err(format!(
                "{} is driven by the robot itself and cannot be a table entry",
                skill.name
            ));
        }
        let mut entry = toml_edit::Table::new();
        entry["name"] = toml_edit::value(skill.name.clone());
        if let Some(path) = &skill.path {
            entry["path"] = toml_edit::value(path.display().to_string());
        }
        entry["duration"] = toml_edit::value(skill.duration);
        if skill.chain {
            entry["chain"] = toml_edit::value(true);
        }
        // Only what differs from a plain zero-command one-shot. A file full of explicit
        // defaults is the unreadable thing this whole editor exists to avoid.
        if skill.command != [0.0; 3] {
            entry["command"] = toml_edit::value(array_of(skill.command));
        }
        if skill.unwind_s > 0.0 {
            entry["unwind"] = toml_edit::value(array_of(skill.unwind));
            entry["unwind_s"] = toml_edit::value(skill.unwind_s);
        }

        let mut overrides = toml_edit::Table::new();
        for (key, value) in [
            ("action_scale", skill.params.action_scale),
            ("gain_ratio", skill.params.gain_ratio),
        ] {
            if let Some(value) = value {
                overrides[key] = toml_edit::value(value);
            }
        }
        if !overrides.is_empty() {
            overrides.set_implicit(false);
            entry["params"] = toml_edit::Item::Table(overrides);
        }

        let policy = self
            .doc
            .entry("policy")
            .or_insert(toml_edit::Item::Table(toml_edit::Table::new()));
        let policy = policy
            .as_table_mut()
            .ok_or_else(|| "[policy] is not a table".to_owned())?;
        let skills = policy
            .entry("skill")
            .or_insert(toml_edit::Item::ArrayOfTables(
                toml_edit::ArrayOfTables::new(),
            ));
        let skills = skills
            .as_array_of_tables_mut()
            .ok_or_else(|| "[[policy.skill]] is not a table array".to_owned())?;

        // By position, so the borrow ends before the push.
        let existing = skills
            .iter()
            .position(|t| t.get("name").and_then(|n| n.as_str()) == Some(skill.name.as_str()));
        match existing {
            Some(at) => *skills.get_mut(at).expect("just found it") = entry,
            None => skills.push(entry),
        }
        Ok(())
    }

    /// Remove a `[[policy.skill]]` entry by name. `false` if there was none.
    fn drop_skill(&mut self, name: &str) -> Result<bool, String> {
        let Some(skills) = self
            .doc
            .get_mut("policy")
            .and_then(|p| p.as_table_mut())
            .and_then(|p| p.get_mut("skill"))
            .and_then(|s| s.as_array_of_tables_mut())
        else {
            return Ok(false);
        };
        let before = skills.len();
        skills.retain(|t| t.get("name").and_then(|n| n.as_str()) != Some(name));
        Ok(skills.len() != before)
    }

    /// Validate the pending edits through the daemon's own gate, then write atomically.
    ///
    /// Validation goes through a real file and [`Params::load`] rather than a bare parse,
    /// because `load` is what `robotd` runs at startup — range checks included. What this tool
    /// writes, the daemon starts on.
    ///
    /// **The pending edits are re-applied to the file as it is now**, under the lock, rather
    /// than to the copy read when this model was loaded. A model can be open for as long as
    /// somebody leaves `robotctl configure` on screen, and `robotd` writes the same file for
    /// `pad.bind` while they do — saving the old document back would silently revert that.
    /// Only the keys this model actually edited are carried across; everything else is
    /// whatever the file says.
    pub fn save(&mut self) -> Result<(), String> {
        let lock = lock(&self.path)?;
        if let Ok(fresh) = std::fs::read_to_string(&self.path)
            && let Ok(doc) = fresh.parse::<DocumentMut>()
        {
            self.doc = doc;
        }
        let text = self.rendered();
        self.write_through(&lock, &text)?;
        // The document on disk is now the rendered one; fold the edits in.
        self.doc = text.parse().expect("just validated");
        for key in self.pending.keys() {
            let key = (*key).to_owned();
            if !self.written.contains(&key) {
                self.written.push(key);
            }
        }
        self.pending.clear();
        Ok(())
    }

    /// Stage, validate through the daemon's own loader, and rename into place.
    ///
    /// Takes the lock as proof rather than acquiring it, so the caller's read-modify-write is
    /// covered by one hold rather than by two that another writer can slip between.
    fn write_through(&self, _lock: &Lock, text: &str) -> Result<(), String> {
        let staged = self.path.with_extension("toml.new");
        let write = |path: &Path| -> std::io::Result<()> {
            let mut file = std::fs::File::create(path)?;
            file.write_all(text.as_bytes())?;
            file.sync_all()
        };
        write(&staged).map_err(|e| writable_hint(&staged, &e))?;
        if let Err(e) = Params::load(&staged, true) {
            let _ = std::fs::remove_file(&staged);
            return Err(format!(
                "refusing to write a config robotd would reject: {e}"
            ));
        }
        std::fs::rename(&staged, &self.path).map_err(|e| writable_hint(&self.path, &e))?;
        // The rename is only durable once the directory entry is. A robot switched off at the
        // wall is the normal case here, not the exceptional one — same discipline as `configd`'s
        // store and the update journal.
        if let Some(dir) = self.path.parent()
            && let Ok(dir) = std::fs::File::open(dir)
        {
            let _ = dir.sync_all();
        }
        Ok(())
    }
}

/// An exclusive hold on the config file, for the duration of one read-modify-write.
type Lock = std::fs::File;

/// Take it.
///
/// **Four writers share `robotd.toml`**: `robotctl configure`, `robotctl policy`, `robotctl pad`,
/// and `robotd` itself, which writes it serving `robot.loadPolicy`, `robot.setSkill` and
/// `pad.bind` from a phone. Two of them staging into the same `robotd.toml.new` at the same
/// moment is a file with one writer's half of the work, renamed into place by the other.
///
/// On a lock file beside the config rather than on the config itself, so the hold outlives the
/// rename that replaces it — the arrangement `configd`'s store uses, for the same reason.
fn lock(path: &Path) -> Result<Lock, String> {
    let lock_path = path.with_extension("lock");
    let file = std::fs::OpenOptions::new()
        .create(true)
        .write(true)
        .truncate(false)
        .open(&lock_path)
        .map_err(|e| writable_hint(&lock_path, &e))?;
    file.lock()
        .map_err(|e| format!("cannot lock {}: {e}", lock_path.display()))?;
    Ok(file)
}

/// Add or replace one `[[policy.skill]]` entry, by name.
///
/// Load, edit and write under one hold of the lock, because a repeating table cannot be queued
/// as a keyed edit and so cannot be rebased the way [`Model::save`] rebases one. `robotctl
/// policy add` and `robot.setSkill` both come through here.
pub fn set_skill(path: &Path, skill: &crate::SkillDef) -> Result<(), String> {
    let lock = lock(path)?;
    let mut model = Model::load(path)?;
    model.put_skill(skill)?;
    let text = model.doc.to_string();
    model.write_through(&lock, &text)
}

/// Take a skill out of the table. `false` if there was none, which is not a failure — it is the
/// answer to "is this robot's own back", and the caller says so differently.
pub fn remove_skill(path: &Path, name: &str) -> Result<bool, String> {
    let lock = lock(path)?;
    let mut model = Model::load(path)?;
    if !model.drop_skill(name)? {
        return Ok(false);
    }
    let text = model.doc.to_string();
    model.write_through(&lock, &text).map(|()| true)
}

/// Three floats as a TOML array.
fn array_of(values: [f64; 3]) -> toml_edit::Array {
    let mut array = toml_edit::Array::new();
    for value in values {
        array.push(value);
    }
    array
}

/// A value as the UI shows it — the data alone. Strings lose their quotes, and everything
/// loses its decor: `to_string` on a `toml_edit` value carries the whitespace and any inline
/// comment along, which is how `50 # do not touch` once ended up in a value cell.
pub fn render(value: &toml_edit::Value) -> String {
    match value {
        toml_edit::Value::String(s) => s.value().clone(),
        toml_edit::Value::Integer(v) => v.value().to_string(),
        toml_edit::Value::Float(v) => v.value().to_string(),
        toml_edit::Value::Boolean(v) => v.value().to_string(),
        toml_edit::Value::Datetime(v) => v.value().to_string(),
        other => {
            let mut bare = other.clone();
            bare.decor_mut().clear();
            bare.to_string().trim().to_owned()
        }
    }
}

/// Sections in registry order, for headers.
pub fn sections() -> Vec<&'static str> {
    let mut out: Vec<&'static str> = Vec::new();
    for entry in REGISTRY {
        let section = entry.key.split_once('.').expect("section.key").0;
        if out.last() != Some(&section) {
            out.push(section);
        }
    }
    out
}

/// Permission errors say what to do about it, because the file is root-owned by design.
///
/// It does not name a command any more. Four callers write this file and `robotd` is one of
/// them — it runs as root, so telling whoever asked over the radio to "run `sudo robotctl
/// configure`" would send them to a terminal to fix something sudo cannot reach.
fn writable_hint(path: &Path, e: &std::io::Error) -> String {
    if e.kind() == std::io::ErrorKind::PermissionDenied {
        format!(
            "cannot write {}: permission denied — it is root-owned, so this needs sudo",
            path.display()
        )
    } else {
        format!("cannot write {}: {e}", path.display())
    }
}

/// The pad's button bindings as the daemon resolves them — config over the defaults.
pub fn pad_bindings(path: &Path) -> Result<crate::PadParams, String> {
    let text = match std::fs::read_to_string(path) {
        Ok(text) => text,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => String::new(),
        Err(e) => return Err(format!("cannot read {}: {e}", path.display())),
    };
    toml::from_str::<Params>(&text)
        .map(|params| params.pad)
        .map_err(|e| format!("{}: {e}", path.display()))
}

/// Put a skill on a button, or clear it.
///
/// Through the same document and the same validation every other edit uses, so comments survive
/// and nothing lands that `robotd` would refuse to start on. A binding set to the default is
/// *removed* rather than written out, keeping the file a list of decisions — which is also what
/// makes `robotctl configure --list` mean something.
pub fn bind_pad(path: &Path, button: &str, skill: &str) -> Result<(), String> {
    let mut model = Model::load(path)?;
    let key = format!("pad.{button}");
    let entry = REGISTRY
        .iter()
        .find(|e| e.key == key)
        .ok_or_else(|| format!("{key} is not a key robotd knows"))?;
    model.edit(entry, skill)?;
    model.save()
}

/// Record which file a policy slot runs — or clear the key, which is what a reset is.
///
/// **The daemon's half of `robot.loadPolicy`.** A slot is `[policy] <slot>` in the config file
/// and nothing else, so a load that did not write here would be gone at the next restart; the
/// method persists, and this is where. `robotctl policy load` used to write it from the other
/// end, which made the same words mean different things depending on who asked.
///
/// `Ok(true)` when the file changed. `Ok(false)` is a request the file already said — worth
/// telling apart, because a slot the loop is already running *and* the file already names is a
/// command with nothing left to do.
///
/// Every slot is written in one document and one save, so a whole-robot reset cannot leave four
/// keys cleared and three not.
pub fn set_slots(path: &Path, slots: &[crate::Slot], to: Option<&Path>) -> Result<bool, String> {
    let mut model = Model::load(path)?;
    let before = model.rendered();
    let value = match to {
        Some(path) => path.display().to_string(),
        None => "unset".to_owned(),
    };
    for slot in slots {
        let key = slot.config_key();
        let entry = REGISTRY
            .iter()
            .find(|e| e.key == key)
            .ok_or_else(|| format!("{key} is not a key robotd knows"))?;
        model
            .edit(entry, &value)
            .map_err(|e| format!("{key}: {e}"))?;
    }
    // The rendered document rather than the keys: it is what `save` would write, so it cannot
    // disagree with itself about whether writing would change anything.
    if model.rendered() == before {
        return Ok(false);
    }
    model.save().map(|()| true)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The shipped example, which is real config with real comments — the thing edits must
    /// not destroy.
    const SHIPPED: &str = include_str!("../../deploy/robotd.toml");

    fn model(text: &str) -> Model {
        Model::from_text(Path::new("/test/robotd.toml"), text).expect("parses")
    }

    fn entry(key: &str) -> &'static Entry {
        crate::registry::entry_for(key).expect("a registry key")
    }

    /// An empty file is a robot on defaults: every row effective at its default, none
    /// overridden. The editor's baseline view.
    #[test]
    fn an_absent_file_shows_the_defaults() {
        let m = model("");
        for row in m.rows() {
            assert!(!row.overridden(), "{}", row.entry.key);
            assert!(!row.effective().is_empty(), "{}", row.entry.key);
        }
        // Spot-check values against the daemon's documented defaults.
        let rows = m.rows();
        let find = |key: &str| rows.iter().find(|r| r.entry.key == key).expect("known");
        assert_eq!(find("control.hz").effective(), "50");
        assert_eq!(find("policy.mode").effective(), "walk");
        assert_eq!(find("safety.limp_fall").effective(), "true");
        assert_eq!(find("audio.pet_detect").effective(), "unset");
    }

    /// Editing must not eat the file: comments, ordering and untouched keys all survive a
    /// set-and-save round trip. This is the property that makes the tool safe to point at a
    /// robot's real, hand-annotated config.
    #[test]
    fn comments_and_unknown_content_survive_an_edit() {
        let text = "# tuned by hand on 2026-03-01\n\
                    [control]\n\
                    hz = 50 # do not touch\n\n\
                    [audio]\n\
                    # the speaker crackles above 0.8\n\
                    enabled = true\n";
        let mut m = model(text);
        m.edit(entry("audio.enabled"), "false").expect("edits");
        let out = m.rendered();
        assert!(out.contains("# tuned by hand on 2026-03-01"), "{out}");
        assert!(out.contains("hz = 50 # do not touch"), "{out}");
        assert!(out.contains("# the speaker crackles above 0.8"), "{out}");
        assert!(out.contains("enabled = false"), "{out}");
    }

    /// A key set in a section the file does not have yet creates the section — the shipped
    /// file keeps everything commented out, so this is the *common* case, not the edge.
    #[test]
    fn setting_a_key_creates_its_section_when_needed() {
        let mut m = model("[control]\nhz = 50\n");
        m.edit(entry("policy.mode"), "roller").expect("edits");
        let out = m.rendered();
        assert!(out.contains("[policy]"), "{out}");
        assert!(out.contains("mode = \"roller\""), "{out}");
        // And it parses as the daemon would read it.
        let parsed: Params = toml::from_str(&out).expect("valid");
        assert_eq!(parsed.policy.mode.as_str(), "roller");
    }

    /// Clearing an override removes the key and the comment attached to it — "# why 40" is
    /// about the 40, and keeping it above nothing would be stranger than taking it along.
    /// Everything else survives, and typing the default is the same as clearing, so the file
    /// never accumulates written-out defaults.
    #[test]
    fn reverting_removes_the_override_and_its_own_comment_only() {
        let text =
            "# the board's story\n[control]\n# why 40: bench board\nhz = 40\ncmd_alpha = 0.3\n";
        let mut m = model(text);
        m.edit(entry("control.hz"), "50").expect("the default");
        let out = m.rendered();
        assert!(!out.contains("hz = 40"), "{out}");
        assert!(
            !out.contains("hz = 50"),
            "typed default must not be pinned: {out}"
        );
        assert!(
            !out.contains("why 40"),
            "the override's own comment goes with it: {out}"
        );
        assert!(out.contains("cmd_alpha = 0.3"), "{out}");
        assert!(out.contains("# the board's story"), "{out}");
    }

    /// The toggles: bool flips, tri-state cycles through auto, choices wrap around.
    #[test]
    fn toggling_produces_the_next_sensible_value() {
        let mut m = model("");
        let toggle = |m: &Model, key: &str| {
            let rows = m.rows();
            let row = rows.iter().find(|r| r.entry.key == key).expect("known");
            m.toggled(row)
        };
        assert_eq!(toggle(&m, "audio.enabled").as_deref(), Some("false"));
        assert_eq!(toggle(&m, "policy.mode").as_deref(), Some("roller"));
        // Tri-state: unset → on → off → unset.
        assert_eq!(toggle(&m, "audio.pet_detect").as_deref(), Some("true"));
        m.edit(entry("audio.pet_detect"), "true").expect("edits");
        assert_eq!(toggle(&m, "audio.pet_detect").as_deref(), Some("false"));
        m.edit(entry("audio.pet_detect"), "false").expect("edits");
        assert_eq!(toggle(&m, "audio.pet_detect").as_deref(), Some("unset"));
        // Numbers are typed, not toggled.
        assert_eq!(toggle(&m, "control.hz"), None);
    }

    /// Bad input is refused at the row, with the reason — not written and bounced by the
    /// validator later, when the user has moved on.
    #[test]
    fn bad_input_is_refused_where_it_is_typed() {
        let mut m = model("");
        assert!(m.edit(entry("control.hz"), "fast").is_err());
        assert!(m.edit(entry("policy.mode"), "hovercraft").is_err());
        assert!(m.edit(entry("audio.enabled"), "maybe").is_err());
        assert!(m.edit(entry("control.cmd_alpha"), "0.3.0").is_err());
        assert!(m.pending.is_empty(), "nothing queued: {:?}", m.pending);
    }

    /// The saved file must pass the daemon's own gate — a value the row-level checks cannot
    /// judge (hz range) is caught before the write, and the file on disk stays untouched.
    #[test]
    fn a_config_robotd_would_reject_is_never_written() {
        let dir = tempfile::tempdir().expect("tempdir");
        let path = dir.path().join("robotd.toml");
        std::fs::write(&path, "[control]\nhz = 50\n").expect("writes");
        let mut m = Model::load(&path).expect("loads");
        // 0 parses as an integer; only Params::load knows it divides by zero.
        m.edit(entry("control.hz"), "0").expect("row-level ok");
        let err = m.save().expect_err("must refuse");
        assert!(err.contains("robotd would reject"), "{err}");
        let on_disk = std::fs::read_to_string(&path).expect("reads");
        assert_eq!(on_disk, "[control]\nhz = 50\n", "disk untouched");
        // The staging file is cleaned up, not left beside the config.
        assert!(!path.with_extension("toml.new").exists());
    }

    /// A good save is atomic-by-rename, folds the edits in, and a fresh load agrees.
    #[test]
    fn a_save_round_trips_through_the_real_loader() {
        let dir = tempfile::tempdir().expect("tempdir");
        let path = dir.path().join("robotd.toml");
        std::fs::write(&path, SHIPPED).expect("writes");
        let mut m = Model::load(&path).expect("loads");
        m.edit(entry("policy.mode"), "roller").expect("edits");
        m.edit(entry("audio.enabled"), "false").expect("edits");
        m.save().expect("saves");
        assert!(m.pending.is_empty());

        let reloaded = Params::load(&path, true).expect("the daemon can start on it");
        assert_eq!(reloaded.policy.mode.as_str(), "roller");
        assert!(!reloaded.audio.enabled);
        // The shipped file's documentation survived the trip.
        let text = std::fs::read_to_string(&path).expect("reads");
        assert!(
            text.contains("Read once at startup") || text.lines().count() > 50,
            "the comments are gone: {} lines",
            text.lines().count()
        );
    }

    /// The shipped example loads into the editor, and every value it does set explicitly is
    /// the default — the editor-side echo of robotd's own example-matches-defaults test, and
    /// the reason a fresh robot's config shows no surprising overrides.
    #[test]
    fn the_shipped_example_sets_nothing_away_from_default() {
        let m = model(SHIPPED);
        for row in m.rows() {
            if let Some(set) = &row.set {
                assert_eq!(
                    set, &row.default,
                    "{} is shipped away from its default",
                    row.entry.key
                );
            }
        }
    }

    /// An unset bitrate shows what it will actually stream at, and follows the quality as it
    /// is cycled — the reason it is optional rather than a number to keep in step by hand.
    #[test]
    fn an_unset_bitrate_shows_what_the_quality_resolves_to() {
        let mut m = model("");
        let bitrate = |m: &Model| {
            m.rows()
                .into_iter()
                .find(|row| row.entry.key == "media.bitrate")
                .expect("known")
        };
        let row = bitrate(&m);
        assert_eq!(row.set, None);
        assert_eq!(row.resolved.as_deref(), Some("2000000"));

        m.edit(entry("media.quality"), "1080p30").expect("valid");
        assert_eq!(bitrate(&m).resolved.as_deref(), Some("4000000"));

        // Set explicitly, it is a value like any other and no longer a hint.
        m.edit(entry("media.bitrate"), "3000000").expect("valid");
        let row = bitrate(&m);
        assert_eq!(row.set.as_deref(), Some("3000000"));
        assert_eq!(row.resolved, None);

        // And `unset` puts it back to following the quality rather than pinning the default.
        m.edit(entry("media.bitrate"), "unset").expect("valid");
        assert_eq!(bitrate(&m).resolved.as_deref(), Some("4000000"));
    }

    /// The editor's own gate is `Params::load`, so a bitrate in the wrong unit never reaches
    /// the disk — the mistake is caught while the file is still the one that works.
    #[test]
    fn a_bitrate_in_kilobits_is_not_written() {
        let dir = tempfile::tempdir().expect("tempdir");
        let path = dir.path().join("robotd.toml");
        let mut m = Model::load(&path).expect("empty is a model");
        m.edit(entry("media.bitrate"), "2000")
            .expect("parses as a number");
        assert!(m.save().is_err(), "mediad would stream nothing at 2 kb/s");
        assert!(!path.exists(), "and nothing was written");
    }

    /// An inline comment is decor, not data: `hz = 50 # do not touch` is the value 50. This
    /// once rode into the value cell and made an at-default key look overridden and annotated.
    #[test]
    fn an_inline_comment_is_not_part_of_the_value() {
        let m = model("[control]\nhz = 50 # do not touch, bench board\n");
        let rows = m.rows();
        let hz = rows
            .iter()
            .find(|r| r.entry.key == "control.hz")
            .expect("known");
        assert_eq!(hz.set.as_deref(), Some("50"));
        assert!(!hz.differs(), "50 is the default, however it is annotated");
        assert!(hz.overridden(), "it is still written in the file");
    }

    /// The colour question: set-but-equal is not a divergence. Only a value actually away
    /// from the default differs.
    #[test]
    fn writing_the_default_out_is_not_a_divergence() {
        let m = model("[policy]\nenabled = true\nmode = \"roller\"\n");
        let rows = m.rows();
        let find = |key: &str| rows.iter().find(|r| r.entry.key == key).expect("known");
        assert!(!find("policy.enabled").differs(), "true is the default");
        assert!(find("policy.enabled").overridden());
        assert!(find("policy.mode").differs(), "roller is not");
    }

    /// `unset` was a word that told nobody anything; the daemon can usually say what unset
    /// *resolves to*, per mode — and the hint follows the mode when it changes.
    #[test]
    fn unset_keys_show_what_they_resolve_to() {
        let mut m = model("");
        let hint = |m: &Model, key: &str| {
            m.rows()
                .iter()
                .find(|r| r.entry.key == key)
                .expect("known")
                .resolved
                .clone()
        };
        let walk = hint(&m, "policy.walk").expect("resolves");
        assert!(walk.contains("alpha_walking"), "{walk}");
        assert_eq!(hint(&m, "policy.legs_lowpass").as_deref(), Some("0.7"));
        assert_eq!(
            hint(&m, "audio.pet_detect").as_deref(),
            Some("false"),
            "petting is an opt-in now, in every mode"
        );
        // Flip the mode and the hints follow — they are resolved through the pending state.
        m.edit(entry("policy.mode"), "roller").expect("edits");
        assert_eq!(
            hint(&m, "audio.pet_detect").as_deref(),
            Some("false"),
            "the roller does not"
        );
        let crouch = hint(&m, "policy.ground_pick").expect("resolves");
        assert!(
            crouch.contains("crouch") || crouch.contains("roller"),
            "{crouch}"
        );
        // A set key hints nothing — the value speaks for itself.
        m.edit(entry("policy.legs_lowpass"), "0.6").expect("edits");
        assert_eq!(hint(&m, "policy.legs_lowpass"), None);
    }

    /// **Binding a button writes one key and leaves the file alone otherwise.** It goes through
    /// the same document every other edit uses, so a hand-written config keeps its comments.
    #[test]
    fn binding_a_button_writes_one_key_and_keeps_the_comments() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("robotd.toml");
        std::fs::write(&path, "# hand-written\n[policy]\nmode = \"walk\"\n").unwrap();

        super::bind_pad(&path, "x", "polite-bow").unwrap();
        let written = std::fs::read_to_string(&path).unwrap();
        assert!(written.contains("x = \"polite-bow\""), "{written}");
        assert!(written.contains("# hand-written"), "{written}");
        assert!(
            !written.contains("lb ="),
            "only the button asked for: {written}"
        );

        let bindings = super::pad_bindings(&path).unwrap();
        assert_eq!(bindings.x, "polite-bow");
        assert_eq!(
            bindings.lb, "kick_left",
            "the rest resolve to their defaults"
        );
    }

    /// Binding a button back to its default *removes* the key rather than pinning it, so the
    /// file stays a list of decisions — which is what makes `configure --list` mean anything.
    #[test]
    fn binding_the_default_removes_the_key() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("robotd.toml");
        std::fs::write(&path, "[pad]\nx = \"polite-bow\"\n").unwrap();

        super::bind_pad(&path, "x", "roulade").unwrap();
        let written = std::fs::read_to_string(&path).unwrap();
        assert!(!written.contains("polite-bow"), "{written}");
        assert!(
            !written.contains("x ="),
            "the default is not pinned: {written}"
        );
        assert_eq!(super::pad_bindings(&path).unwrap().x, "roulade");
    }

    /// **Resetting a button clears it back to the default and leaves the file clean.** The undo
    /// for a session of trying skills has to actually undo, or `configure --list` keeps
    /// reporting a robot as modified after it has been put back.
    #[test]
    fn resetting_bindings_leaves_nothing_behind() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("robotd.toml");
        std::fs::write(&path, "# hand-written\n[policy]\nmode = \"walk\"\n").unwrap();

        super::bind_pad(&path, "x", "polite-bow").unwrap();
        super::bind_pad(&path, "rb", "flamingo").unwrap();

        let defaults = crate::PadParams::default();
        for button in crate::PadParams::BUTTONS {
            super::bind_pad(&path, button, defaults.skill(button).unwrap()).unwrap();
        }

        let model = super::Model::load(&path).unwrap();
        assert!(
            !model.rows().iter().any(|row| row.differs()),
            "a reset robot reports as unmodified"
        );
        let written = std::fs::read_to_string(&path).unwrap();
        assert!(!written.contains("polite-bow"), "{written}");
        assert!(written.contains("# hand-written"), "{written}");
    }

    /// A robot with no config file at all still has bindings — the defaults — rather than an
    /// error. `padd` may run before anything has ever been configured.
    #[test]
    fn a_missing_config_still_has_bindings() {
        let dir = tempfile::tempdir().unwrap();
        let bindings = super::pad_bindings(&dir.path().join("nothing.toml")).unwrap();
        assert_eq!(bindings.a, "ground_pick");
    }

    /// Sections come out in registry order, once each — the editor's headers.
    #[test]
    fn sections_are_ordered_and_unique() {
        let s = sections();
        assert_eq!(
            s,
            vec![
                "bus",
                "control",
                "update_gate",
                "policy",
                "safety",
                "detect",
                "chorale",
                "theremin",
                "audio",
                "media",
                // Last, and the editor shows sections in this order: the pad is what a robot's
                // buttons do, which is the thing somebody browses for rather than tunes.
                "pad"
            ]
        );
    }

    /// **Four processes write this file**, and two staging into the same `robotd.toml.new` at
    /// once is one writer's half renamed into place by the other. Every writer takes the lock
    /// for its whole read-modify-write, so concurrent adds compose instead of racing.
    #[test]
    fn concurrent_writers_do_not_lose_each_other_s_work() {
        let dir = tempfile::tempdir().expect("tempdir");
        let config = dir.path().join("robotd.toml");
        std::fs::write(&config, "[policy]\n").expect("write");

        let hands: Vec<_> = (0..8)
            .map(|i| {
                let config = config.clone();
                std::thread::spawn(move || {
                    set_skill(
                        &config,
                        &crate::SkillDef {
                            name: format!("skill-{i}"),
                            path: Some(PathBuf::from("/srv/bow.onnx")),
                            duration: 1.0,
                            ..Default::default()
                        },
                    )
                })
            })
            .collect();
        for hand in hands {
            hand.join().expect("thread").expect("written");
        }

        let parsed = Params::load(&config, true).expect("robotd would start on it");
        let mut names: Vec<String> = parsed
            .policy
            .skills
            .iter()
            .map(|s| s.name.clone())
            .collect();
        names.sort();
        assert_eq!(names.len(), 8, "every writer's entry survived: {names:?}");
    }

    /// The keyed edits rebase on save, so a model left open while something else writes the file
    /// carries its own change across rather than reverting the other. `robotctl configure` can
    /// sit on screen for minutes while a phone calls `pad.bind`.
    #[test]
    fn saving_an_open_model_keeps_what_somebody_else_wrote() {
        let dir = tempfile::tempdir().expect("tempdir");
        let config = dir.path().join("robotd.toml");
        std::fs::write(&config, "[policy]\n").expect("write");

        // Opened, and then left alone the way an editor is.
        let mut open = Model::load(&config).expect("loads");
        open.edit(entry("audio.enabled"), "false").expect("edits");

        set_skill(
            &config,
            &crate::SkillDef {
                name: "polite-bow".into(),
                path: Some(PathBuf::from("/srv/bow.onnx")),
                duration: 4.0,
                ..Default::default()
            },
        )
        .expect("added underneath");

        open.save().expect("saves");
        let written = std::fs::read_to_string(&config).expect("read");
        assert!(written.contains("enabled = false"), "{written}");
        assert!(
            written.contains("polite-bow"),
            "and the bow is still here: {written}"
        );
    }

    /// By name rather than by appending: running `robotctl policy add` twice retunes a skill
    /// instead of giving a robot two of it.
    #[test]
    fn adding_a_skill_twice_replaces_it() {
        let dir = tempfile::tempdir().expect("tempdir");
        let config = dir.path().join("robotd.toml");
        std::fs::write(&config, "# kept\n[policy]\nmode = \"walk\"\n").expect("write");

        let skill = |duration: f64| crate::SkillDef {
            name: "polite-bow".into(),
            path: Some(PathBuf::from("/srv/bow.onnx")),
            duration,
            ..Default::default()
        };
        set_skill(&config, &skill(4.0)).expect("added");
        set_skill(&config, &skill(6.0)).expect("retuned");

        let written = std::fs::read_to_string(&config).expect("read");
        assert_eq!(written.matches("[[policy.skill]]").count(), 1, "{written}");
        assert!(written.contains("duration = 6.0"), "{written}");
        assert!(written.contains("# kept"), "comments survive: {written}");
    }

    /// Only what differs from a plain zero-command one-shot is written — a file full of explicit
    /// defaults is the unreadable thing this editor exists to avoid.
    #[test]
    fn a_plain_skill_writes_no_command_or_unwind() {
        let dir = tempfile::tempdir().expect("tempdir");
        let config = dir.path().join("robotd.toml");
        std::fs::write(&config, "[policy]\n").expect("write");

        set_skill(
            &config,
            &crate::SkillDef {
                name: "polite-bow".into(),
                path: Some(PathBuf::from("/srv/bow.onnx")),
                duration: 4.0,
                ..Default::default()
            },
        )
        .expect("added");

        let written = std::fs::read_to_string(&config).expect("read");
        assert!(!written.contains("command"), "{written}");
        assert!(!written.contains("unwind"), "{written}");
        assert!(!written.contains("chain"), "{written}");
    }

    /// A policy that holds until told otherwise writes both halves, and what is written is
    /// checked through the daemon's own loader — what this writes, robotd starts on.
    #[test]
    fn a_two_phase_skill_writes_both_halves() {
        let dir = tempfile::tempdir().expect("tempdir");
        let config = dir.path().join("robotd.toml");
        std::fs::write(&config, "[policy]\n").expect("write");

        set_skill(
            &config,
            &crate::SkillDef {
                name: "flamingo".into(),
                path: Some(PathBuf::from("/srv/f.onnx")),
                duration: 5.0,
                command: [1.0, 1.0, 0.0],
                unwind: [0.0, 1.0, 0.0],
                unwind_s: 3.0,
                ..Default::default()
            },
        )
        .expect("added");

        let parsed = Params::load(&config, true).expect("robotd would accept it");
        let flamingo = parsed.policy.resolved().skills.pop().expect("the skill");
        assert_eq!(flamingo.name, "flamingo");
        assert_eq!(flamingo.command, [1.0, 1.0, 0.0]);
        assert_eq!(flamingo.unwind_s, 3.0);
    }

    /// Removing says whether there was anything to remove, so a typo says so instead of looking
    /// like success.
    #[test]
    fn removing_a_skill_says_whether_there_was_one() {
        let dir = tempfile::tempdir().expect("tempdir");
        let config = dir.path().join("robotd.toml");
        std::fs::write(&config, "[policy]\n").expect("write");

        assert_eq!(remove_skill(&config, "nothing"), Ok(false));
        set_skill(
            &config,
            &crate::SkillDef {
                name: "polite-bow".into(),
                path: Some(PathBuf::from("/srv/bow.onnx")),
                duration: 4.0,
                ..Default::default()
            },
        )
        .expect("added");
        assert_eq!(remove_skill(&config, "polite-bow"), Ok(true));
    }

    /// **A name the daemon drives itself is refused by the writer, not only by the wire.**
    /// `robotctl policy add` writes through here and `robot.setSkill` does not, so a guard that
    /// lived only in the route left the local command able to write an entry that `robot.do`
    /// would list and never run — it matches the built-in first.
    #[test]
    fn a_skill_cannot_take_a_name_the_daemon_drives_itself() {
        let dir = tempfile::tempdir().expect("tempdir");
        let config = dir.path().join("robotd.toml");
        std::fs::write(&config, "[policy]\n").expect("write");

        for name in crate::DAEMON_OWNED_SKILLS {
            let refusal = set_skill(
                &config,
                &crate::SkillDef {
                    name: name.to_owned(),
                    duration: 1.0,
                    ..Default::default()
                },
            )
            .expect_err("refused");
            assert!(refusal.contains(name), "{refusal}");
        }
        assert_eq!(
            std::fs::read_to_string(&config).expect("read"),
            "[policy]\n",
            "and nothing was written"
        );
    }

    /// Every slot has to resolve to a key `robotd` will actually read. A key this invented
    /// would be written, parsed as unknown, and silently ignored — a load that reported success
    /// and did nothing at the next boot.
    #[test]
    fn every_slot_resolves_to_a_key_robotd_reads() {
        for slot in crate::Slot::ALL {
            let key = slot.config_key();
            assert!(
                REGISTRY.iter().any(|e| e.key == key),
                "{key} is not a key robotd knows"
            );
        }
    }

    /// **The config half of `robot.loadPolicy`, end to end.** Load writes the key, reset removes
    /// it, and the comments around them survive both — the file belongs to the operator, and a
    /// daemon that reformatted it on every gait experiment would not be one anybody trusted with
    /// the file at all.
    #[test]
    fn load_writes_the_key_and_reset_removes_it_without_touching_the_file() {
        let dir = tempfile::tempdir().expect("tempdir");
        let config = dir.path().join("robotd.toml");
        std::fs::write(
            &config,
            "# hand-written, and it stays that way\n[policy]\n# which gait\nmode = \"walk\"\n",
        )
        .expect("write");

        let mine = Path::new("/srv/mine.onnx");
        assert_eq!(
            set_slots(&config, &[crate::Slot::Walk], Some(mine)),
            Ok(true)
        );
        let written = std::fs::read_to_string(&config).expect("read");
        assert!(written.contains("walk = \"/srv/mine.onnx\""), "{written}");
        assert!(written.contains("# hand-written"), "{written}");
        assert!(written.contains("# which gait"), "{written}");

        assert_eq!(set_slots(&config, &[crate::Slot::Walk], None), Ok(true));
        let cleared = std::fs::read_to_string(&config).expect("read");
        assert!(!cleared.contains("/srv/mine.onnx"), "{cleared}");
        assert!(
            cleared.contains("mode = \"walk\""),
            "reset touches one key: {cleared}"
        );
        assert!(cleared.contains("# hand-written"), "{cleared}");
    }

    /// Reset must clear an override rather than write today's default in its place. Pinning the
    /// release's own path into config would survive the release, and the next update would find
    /// a robot configured to run a file that no longer exists.
    #[test]
    fn reset_clears_the_key_rather_than_writing_the_default() {
        let dir = tempfile::tempdir().expect("tempdir");
        let config = dir.path().join("robotd.toml");
        std::fs::write(&config, "[policy]\nwalk = \"/srv/mine.onnx\"\n").expect("write");

        assert_eq!(set_slots(&config, &crate::Slot::ALL, None), Ok(true));
        let cleared = std::fs::read_to_string(&config).expect("read");
        assert!(!cleared.contains("walk ="), "{cleared}");
        assert!(
            !cleared.contains("alpha_walking"),
            "no default is pinned: {cleared}"
        );
    }

    /// **Resetting a slot nobody touched writes nothing**, and loading the file already
    /// configured is the same non-event. `false` is what lets the caller tell a request that
    /// reconciled a stale file from one that found nothing to reconcile.
    #[test]
    fn a_request_the_file_already_says_writes_nothing() {
        let dir = tempfile::tempdir().expect("tempdir");
        let config = dir.path().join("robotd.toml");
        std::fs::write(&config, "[policy]\nmode = \"walk\"\n").expect("write");
        assert_eq!(set_slots(&config, &crate::Slot::ALL, None), Ok(false));
        assert_eq!(set_slots(&config, &[crate::Slot::Walk], None), Ok(false));

        std::fs::write(&config, "[policy]\nwalk = \"/srv/mine.onnx\"\n").expect("write");
        assert_eq!(
            set_slots(
                &config,
                &[crate::Slot::Walk],
                Some(Path::new("/srv/mine.onnx"))
            ),
            Ok(false)
        );
    }

    /// A reset is still worth writing when the *file* disagrees with the robot. The loop can say
    /// "nothing to do" about what it is running while a hand-edited config still names an
    /// override nobody restarted into, and reconciling the two is why the write comes before the
    /// already-running check rather than after it.
    #[test]
    fn a_stale_config_key_is_still_worth_writing() {
        let dir = tempfile::tempdir().expect("tempdir");
        let config = dir.path().join("robotd.toml");
        std::fs::write(&config, "[policy]\nwalk = \"/srv/never-loaded.onnx\"\n").expect("write");

        assert_eq!(set_slots(&config, &[crate::Slot::Walk], None), Ok(true));
        let cleared = std::fs::read_to_string(&config).expect("read");
        assert!(!cleared.contains("never-loaded"), "{cleared}");
    }
}
