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
use crate::registry::{Entry, Kind, REGISTRY, RENAMED_SECTIONS};
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
///
/// At the provisioned path that file is the **user layer** ([`crate::USER_PATH`]), and the base
/// under it is read-only here. A row's `default` is then what the robot runs on with no override
/// — the base's value where it sets one, the built-in default where it does not — so clearing a
/// key goes back to what the image says, and typing the image's value clears the override rather
/// than pinning a copy that would stop following the image.
pub struct Model {
    /// The file this model writes — the user layer, when there is a base.
    pub path: PathBuf,
    doc: DocumentMut,
    /// The layer under [`Self::doc`], if it has one: where it is, and what it says.
    base: Option<(PathBuf, toml::Table)>,
    defaults: toml::Value,
    /// Keyed by `section.key`. Applied to the document only on save.
    pub pending: BTreeMap<&'static str, Edit>,
    /// What has actually been written, across every save this session.
    ///
    /// Kept because `pending` is *cleared* by a save, and what wants restarting is decided after
    /// the editor has closed — reading `pending` there found nothing every time, so nothing was
    /// ever restarted and a `[duck_detector]` change looked like a no-op.
    written: Vec<String>,
    /// Keys in the file this build does not know — left there by another branch or release.
    /// The daemon ignores them, so the editor opens anyway and leaves them in the file.
    ignored: Vec<String>,
    /// The same, in the base under a user layer — where [`adopt_base_edits`] leaves them.
    ignored_in_base: Vec<String>,
}

impl Model {
    /// Every key written this session, in the order it was first written.
    pub fn written(&self) -> &[String] {
        &self.written
    }

    /// Keys in the file this build has no place for, as [`Params::parse`] named them.
    pub fn ignored(&self) -> &[String] {
        &self.ignored
    }

    /// The base's unknown keys, with the base's path, when the model is over a user layer.
    pub fn ignored_in_base(&self) -> Option<(&Path, &[String])> {
        let (base, _) = self.base.as_ref()?;
        (!self.ignored_in_base.is_empty()).then_some((base.as_path(), &self.ignored_in_base[..]))
    }

    /// Load the file — or start from an empty document when there is none, which is a real
    /// state: a robot may run entirely on defaults with no file at all.
    ///
    /// At the provisioned path, the model is over the user layer with `path` as its base — and
    /// old edits are adopted first ([`adopt_base_edits`]), so every default this model shows and
    /// every edit queued against one is computed after they have left the base. Best effort: a
    /// reader that cannot write (`configure --list` without root, a laptop) still opens, and a
    /// writer adopts again, strictly, under its lock.
    pub fn load(path: &Path) -> Result<Self, String> {
        match crate::user_layer_for(path) {
            Some(user) => Self::adopt_and_load(path, &user),
            None => Self::open(path),
        }
    }

    fn adopt_and_load(base: &Path, user: &Path) -> Result<Self, String> {
        if !user.exists()
            && let Err(e) = adopt_base_edits(base, user)
        {
            tracing::debug!(error = %e, "old edits not adopted on open");
        }
        Self::load_layered(base, user)
    }

    /// [`Model::load`] without adopting, for a caller already holding the lock.
    fn open(path: &Path) -> Result<Self, String> {
        match crate::user_layer_for(path) {
            Some(user) => Self::load_layered(path, &user),
            None => Self::from_text(path, &read_or_empty(path)?),
        }
    }

    /// A model over the user layer at `user`, with the file at `base` under it.
    ///
    /// The base is held to the daemon's standard — the editor will not open over a base `robotd`
    /// would refuse — because every value this model calls a default comes from it.
    pub fn load_layered(base: &Path, user: &Path) -> Result<Self, String> {
        let base_text = read_or_empty(base)?;
        let (base_params, ignored_in_base) =
            Params::parse(&base_text).map_err(|e| format!("{}: {e}", base.display()))?;
        let base_table: toml::Table = base_text
            .parse()
            .map_err(|e| format!("{}: {e}", base.display()))?;
        let mut model = Self::from_text(user, &read_or_empty(user)?)?;
        model.defaults = toml::Value::try_from(base_params).expect("Params serializes");
        model.base = Some((base.to_path_buf(), base_table));
        model.ignored_in_base = ignored_in_base;
        Ok(model)
    }

    /// Both layers as one document — what the daemon would load, pending edits included.
    fn merged(&self) -> String {
        let user = self.rendered();
        match &self.base {
            Some((_, base)) => match user.parse::<toml::Table>() {
                Ok(user) => crate::merge_layers(base.clone(), user).to_string(),
                Err(_) => user,
            },
            None => user,
        }
    }

    /// Before the first edit to a list the base sets, a copy of the base's list in the user layer.
    ///
    /// A list in the user layer replaces the base's whole ([`crate::merge_layers`]), so adding one
    /// skill to a user layer that has no `[[policy.skill]]` yet would otherwise drop every skill
    /// the base offers.
    fn seed_skills_from_base(&mut self) -> Result<(), String> {
        let has_own = self
            .doc
            .get("policy")
            .and_then(|p| p.get("skill"))
            .is_some();
        let Some(skills) = self
            .base
            .as_ref()
            .and_then(|(_, base)| base.get("policy")?.get("skill"))
        else {
            return Ok(());
        };
        if has_own {
            return Ok(());
        }
        let mut wrapper = toml::Table::new();
        let mut policy = toml::Table::new();
        policy.insert("skill".to_owned(), skills.clone());
        wrapper.insert("policy".to_owned(), toml::Value::Table(policy));
        let copied: DocumentMut = wrapper
            .to_string()
            .parse()
            .map_err(|e| format!("copying the base's skills: {e}"))?;
        let item = copied["policy"]["skill"].clone();
        let policy = self
            .doc
            .entry("policy")
            .or_insert(toml_edit::Item::Table(toml_edit::Table::new()))
            .as_table_mut()
            .ok_or_else(|| "[policy] is not a table".to_owned())?;
        policy.insert("skill", item);
        Ok(())
    }

    /// A model over text already in hand, for a caller that has read the file itself — and for
    /// testing any caller, which is what it is mostly for: building a config from a string beats
    /// a temporary file for saying what a case is about. [`Model::load`] is this plus the read.
    pub fn from_text(path: &Path, text: &str) -> Result<Self, String> {
        // The daemon's own parse first: a file robotd would refuse is not a file to edit
        // blind, and the error names the line. One it would start on, unknown keys and all, is.
        let (_, ignored) = Params::parse(text).map_err(|e| format!("{}: {e}", path.display()))?;
        let mut doc: DocumentMut = text
            .parse()
            .map_err(|e| format!("{}: {e}", path.display()))?;
        Self::migrate_renamed_sections(&mut doc);
        Ok(Self {
            path: path.to_path_buf(),
            doc,
            base: None,
            defaults: toml::Value::try_from(Params::default()).expect("Params serializes"),
            pending: BTreeMap::new(),
            written: Vec::new(),
            ignored,
            ignored_in_base: Vec::new(),
        })
    }

    /// Sections that changed name, carried to the new one in the document itself.
    ///
    /// The loader accepts the old name through a serde alias, so a file written before the rename
    /// keeps working untouched. This is the other half: an *editor* that then set a key under the new
    /// name would leave the file with both sections, which the loader refuses as a duplicate — so the
    /// old header is renamed here, once, and the next save writes the file under the new name only.
    /// Nothing is written until something else is saved; a read-only browse changes no file.
    ///
    /// Called from [`Model::from_text`] and again in [`Model::save`], which re-reads the file
    /// from disk before rendering and would otherwise write the old header straight back.
    fn migrate_renamed_sections(doc: &mut DocumentMut) {
        for (old, new) in RENAMED_SECTIONS {
            if doc.contains_key(new) {
                // Both present: the loader's duplicate-field error already named it. Not ours to
                // guess which wins.
                continue;
            }
            if let Some(item) = doc.remove(old) {
                doc.insert(new, item);
            }
        }
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
        key.split('.')
            .try_fold(self.doc.as_item(), |item, part| item.get(part))?
            .as_value()
            .cloned()
    }

    /// The built-in default, rendered — `unset` for the Option fields that resolve elsewhere.
    fn default_for(&self, key: &str) -> String {
        match key
            .split('.')
            .try_fold(&self.defaults, |value, part| value.get(part))
        {
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
        let (params, _) = Params::parse(&self.merged()).ok()?;
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
            // Six numbers from a calibration, where a typo is a plausible wrong answer rather
            // than an error. Written by whatever measured them, not typed into an editor.
            Kind::Record(_) => {
                return Err(
                    "camera intrinsics come from a calibration; write them into robotd.toml \
                     directly, or leave them absent to publish the module's design figures"
                        .to_owned(),
                );
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
            let (section, name) = key.rsplit_once('.').expect("section.key");
            match edit {
                Edit::Set(value) => {
                    let mut item = doc.as_item_mut();
                    for part in section.split('.') {
                        let table = item.as_table_like_mut().expect("validated table");
                        if !table.contains_key(part) {
                            table.insert(part, toml_edit::Item::Table(toml_edit::Table::new()));
                        }
                        item = table.get_mut(part).expect("just inserted");
                    }
                    let table = item.as_table_like_mut().expect("validated table");
                    let value = toml_edit::Item::Value(value.clone());
                    if let Some(existing) = table.get_mut(name) {
                        *existing = value;
                    } else {
                        table.insert(name, value);
                    }
                }
                Edit::Clear => {
                    if let Some(table) = section
                        .split('.')
                        .try_fold(doc.as_item_mut(), |item, part| item.get_mut(part))
                        .and_then(|item| item.as_table_like_mut())
                    {
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
        self.seed_skills_from_base()?;
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
        self.seed_skills_from_base()?;
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
        // Before the re-read below, so the edits are rebased onto whatever adoption copied.
        if let Some((base, _)) = &self.base {
            adopt_locked(&lock, base, &self.path)?;
        }
        if let Ok(fresh) = std::fs::read_to_string(&self.path)
            && let Ok(mut doc) = fresh.parse::<DocumentMut>()
        {
            Self::migrate_renamed_sections(&mut doc);
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
    fn write_through(&self, lock: &Lock, text: &str) -> Result<(), String> {
        let staged = self.stage_validated(lock, text)?;
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

    /// The same staging and loader check [`Self::write_through`] does, then delete the
    /// staging file instead of renaming it into place.
    fn preview_through(&self, lock: &Lock, text: &str) -> Result<(), String> {
        let staged = self.stage_validated(lock, text)?;
        let _ = std::fs::remove_file(&staged);
        Ok(())
    }

    /// Write the candidate beside the target and run it through [`Params::load`].
    ///
    /// The staged file is the caller's to rename or remove; on a refusal it is already gone.
    fn stage_validated(&self, _lock: &Lock, text: &str) -> Result<PathBuf, String> {
        let staged = self.path.with_extension("toml.new");
        let write = |path: &Path| -> std::io::Result<()> {
            let mut file = std::fs::File::create(path)?;
            file.write_all(text.as_bytes())?;
            file.sync_all()
        };
        write(&staged).map_err(|e| writable_hint(&staged, &e))?;
        // Over the base when there is one: a user layer is only valid as the half of a robot it
        // is, and the loader would set aside one that failed this.
        let checked = match &self.base {
            Some((base, _)) => Params::with_user_layer(base, &staged).map(|_| ()),
            None => Params::load(&staged, true).map(|_| ()),
        };
        if let Err(e) = checked {
            let _ = std::fs::remove_file(&staged);
            return Err(format!(
                "refusing to write a config robotd would reject: {e}"
            ));
        }
        Ok(staged)
    }
}

/// An exclusive hold on the config file, for the duration of one read-modify-write.
type Lock = std::fs::File;

/// The file an edit to the config at `path` actually writes: the user layer, when it has one.
fn written_path(path: &Path) -> PathBuf {
    crate::user_layer_for(path).unwrap_or_else(|| path.to_path_buf())
}

/// A file's text, or nothing when there is no file.
fn read_or_empty(path: &Path) -> Result<String, String> {
    crate::read_optional(path)
        .map(Option::unwrap_or_default)
        .map_err(|e| format!("cannot read {}: {e}", path.display()))
}

/// The lock for an edit to the config at `path`, with any old edits adopted under it
/// ([`adopt_base_edits`]) — so this edit cannot be the one that creates the user layer and
/// strands them in the base.
fn lock_for_edit(path: &Path) -> Result<Lock, String> {
    let written = written_path(path);
    let lock = lock(&written)?;
    if written != path {
        adopt_locked(&lock, path, &written)?;
    }
    Ok(lock)
}

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
    apply_skill(path, skill, false)
}

/// Same checks as [`set_skill`], including that the would-be file is one `robotd` would start
/// on, then leave the config as it was.
///
/// `robotctl policy add --dry-run` writes through here rather than through a daemon flag:
/// the CLI path never calls `robot.setSkill`, so there is no IPC call to hang one on.
pub fn preview_skill(path: &Path, skill: &crate::SkillDef) -> Result<(), String> {
    apply_skill(path, skill, true)
}

fn apply_skill(path: &Path, skill: &crate::SkillDef, dry_run: bool) -> Result<(), String> {
    let lock = lock_for_edit(path)?;
    let mut model = Model::open(path)?;
    model.put_skill(skill)?;
    let text = model.doc.to_string();
    if dry_run {
        model.preview_through(&lock, &text)
    } else {
        model.write_through(&lock, &text)
    }
}

/// Take a skill out of the table. `false` if there was none, which is not a failure — it is the
/// answer to "is this robot's own back", and the caller says so differently.
pub fn remove_skill(path: &Path, name: &str) -> Result<bool, String> {
    let lock = lock_for_edit(path)?;
    let mut model = Model::open(path)?;
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
    let text = crate::layered_text(path)?;
    Params::parse(&text)
        .map(|(params, _)| params.pad)
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

/// Declare the board, written out even when it is the default.
///
/// Unlike every other edit, which clears a key set to its default: `zero3` is the default *and* a
/// declaration, and a robot whose file says it is one that was asked, where an absent key is one
/// that never was. Same document, same validation and same save as the rest.
pub fn set_board(path: &Path, board: crate::board::Board) -> Result<(), String> {
    let mut model = Model::load(path)?;
    model
        .pending
        .insert("board.version", Edit::Set(board.label().into()));
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

/// Every value a release's `deploy/robotd.toml` ever shipped live (uncommented), by key.
///
/// From the template's whole git history — 69 versions when this was written. It is what tells
/// [`adopt_base_edits`] a value somebody chose from one an older release put there: `install.sh`
/// copied the template once and never again, so a robot installed in the spring still carries
/// that spring's live values, and comparing against today's template alone would mistake them for
/// an owner's decisions and freeze them in. `every_live_template_value_is_listed` keeps it
/// complete going forward. `[board] version` is never adopted and needs no entry.
const EVER_SHIPPED: &[(&str, &str)] = &[
    ("bus.port", "\"/dev/ttyS2\""),
    ("control.hz", "50"),
    ("control.cmd_alpha", "0.2"),
    ("control.head_alpha", "0.2"),
    ("update_gate.stall_periods", "25"),
    ("update_gate.min_achieved_hz", "45.0"),
    ("update_gate.max_consecutive_errors", "10"),
    ("safety.gain_limp", "50"),
    ("safety.fall_gravity_z", "-0.5"),
    ("safety.fall_debounce_ms", "200"),
    ("safety.deadman_ms", "500"),
    ("safety.fall_recover", "false"),
    ("safety.battery_empty_shutdown", "true"),
    ("policy.enabled", "true"),
    ("policy.mode", "\"walk\""),
    ("policy.gain", "200"),
    ("policy.action_scale", "0.7"),
    ("policy.standing_action_scale", "1.0"),
    ("policy.standing_gain_ratio", "0.8"),
    ("policy.standing_gain_ratio", "0.6"),
    ("policy.voltage_adapt", "false"),
    ("policy.nominal_voltage", "7.4"),
    ("policy.kick_duration", "0.5"),
    ("policy.ground_pick_gain_ratio", "1.0"),
    ("policy.roulade_gain_ratio", "1.0"),
    ("policy.roulade_duration", "1.0"),
    ("policy.roulade_action_scale", "1.0"),
    (
        "policy.walk",
        "\"/opt/robot/daemon/current/policies/alpha_walking.onnx\"",
    ),
    (
        "policy.stand",
        "\"/opt/robot/daemon/current/policies/alpha_stand.onnx\"",
    ),
];

fn ever_shipped(key: &str, value: &toml::Value) -> bool {
    EVER_SHIPPED.iter().any(|(k, literal)| {
        *k == key
            && format!("v = {literal}")
                .parse::<toml::Table>()
                .is_ok_and(|t| t.get("v") == Some(value))
    })
}

/// The first time a robot runs a release with layers: move what its owner had changed in the base
/// into a new user layer. Returns the keys moved; an empty list when there is already a user
/// layer, which makes this a no-op on every later start.
///
/// **Why it is needed.** Before layers, every edit landed in the base, mixed in with what the
/// installer or image wrote. Left there, those edits read as the robot's *defaults* in the editor
/// — not marked, not in `configure --list`, and `u` "reverts" to them — and on a robot whose OS
/// updates by A/B slots, the next OS update throws them away with the rest of `/etc`.
///
/// **Moved, not copied**, for the same reason: a copy left in the base would still be the default
/// the editor compares against. The user layer is written first, so a crash between the two
/// writes leaves a value in both files, never in neither. A rollback to a release that predates
/// layers then runs on the template's values until it moves forward again — accepted, since no
/// such release is in production.
///
/// **An OS update that came first.** On an A/B robot the OS bundle's install hook copies the
/// running slot's base to [`carried_path`] when there is no user layer yet, since the new slot's
/// `/etc` is the image's. Its edits are adopted from there and the copy deleted; edits in the
/// current base, if any, win over it as the more recent.
///
/// What counts as an edit: a key this build knows whose value no release's template ever shipped
/// ([`EVER_SHIPPED`]). `[board] version` is the installer's and the image's, never adopted. A key
/// this build does not know is left in the base, where the loader ignores it and the editor names
/// it: it is the base's to keep or drop, and no value of it could be checked here. An owner who had typed exactly a
/// shipped value in is not told apart from the template, and their key goes on following the base;
/// on a robot without slots that changes nothing.
///
/// The user layer is written even when nothing is copied, so the question is asked once.
///
/// `robotd` calls this at startup, and every writer calls it under its own hold of the lock
/// before its first write: whichever runs first, no edit can create the user layer ahead of it.
pub fn adopt_base_edits(base: &Path, user: &Path) -> Result<Vec<String>, String> {
    adopt_locked(&lock(user)?, base, user)
}

/// [`adopt_base_edits`], for a caller already holding the user layer's lock.
fn adopt_locked(lock: &Lock, base: &Path, user: &Path) -> Result<Vec<String>, String> {
    match user.try_exists() {
        Ok(false) => {}
        Ok(true) => {
            strip_leftovers(base, user);
            return Ok(Vec::new());
        }
        Err(e) => return Err(format!("cannot read {}: {e}", user.display())),
    }
    let carried = carried_path(user);
    let (from_slot, mut keys) = if carried.exists() {
        edits_in(&carried)?
    } else {
        Default::default()
    };
    let (from_base, base_keys) = edits_in(base)?;
    let picked = crate::merge_layers(from_slot, from_base);
    keys.extend(base_keys.iter().cloned());
    keys.sort();
    keys.dedup();

    let header = format!(
        "# Settings changed on this robot, layered over {} key by key.\n\
         # Change them with `robotctl configure`. This file survives OS updates; that one does not.\n\
         {}\n",
        base.display(),
        if keys.is_empty() {
            String::new()
        } else {
            "# Moved out of the base config when this robot first ran a release with layers.\n"
                .to_owned()
        },
    );
    let model = Model::load_layered(base, user)?;
    model.write_through(lock, &format!("{header}{picked}"))?;

    // Only now that the user layer holds them. Neither failure loses anything: the values are in
    // the user layer, and what is left behind only shadows a default until the next OS update.
    if let Err(e) = strip_from_base(base, &base_keys) {
        tracing::warn!(error = %e, "adopted edits are still in the base config too");
    }
    if let Err(e) = std::fs::remove_file(&carried)
        && e.kind() != std::io::ErrorKind::NotFound
    {
        tracing::warn!(path = %carried.display(), error = %e, "cannot remove the carried config");
    }
    Ok(keys)
}

/// The owner edits in the file at `path`, as a table at their paths and as keys.
fn edits_in(path: &Path) -> Result<(toml::Table, Vec<String>), String> {
    let mut table: toml::Table = read_or_empty(path)?
        .parse()
        .map_err(|e| format!("{}: {e}", path.display()))?;
    crate::normalise_renamed(&mut table);
    let (mut picked, mut keys) = (toml::Table::new(), Vec::new());
    collect_edits(&table, "", &mut picked, &mut keys);
    Ok((picked, keys))
}

/// Finish a move a power cut or a failed write interrupted: an edit still in the base with the
/// same value in the user layer is taken out of the base, where it would hide as a default.
/// Best effort, like the strip it finishes.
fn strip_leftovers(base: &Path, user: &Path) {
    let Ok((in_base, keys)) = edits_in(base) else {
        return;
    };
    let Ok(mut in_user) = read_or_empty(user)
        .unwrap_or_default()
        .parse::<toml::Table>()
    else {
        return;
    };
    crate::normalise_renamed(&mut in_user);
    let at = |table: &toml::Table, key: &str| {
        let mut parts = key.split('.');
        let first = table.get(parts.next()?);
        parts
            .try_fold(first?, |value, part| value.get(part))
            .cloned()
    };
    let leftovers: Vec<String> = keys
        .into_iter()
        .filter(|key| at(&in_base, key).is_some_and(|v| Some(v) == at(&in_user, key)))
        .collect();
    if let Err(e) = strip_from_base(base, &leftovers) {
        tracing::warn!(error = %e, "adopted edits are still in the base config too");
    }
}

/// Where an OS update's install hook leaves the old slot's base for [`adopt_base_edits`].
pub fn carried_path(user: &Path) -> PathBuf {
    user.with_extension("toml.from-slot")
}

/// Take adopted keys out of the base, keeping its comments and everything else.
fn strip_from_base(base: &Path, keys: &[String]) -> Result<(), String> {
    if keys.is_empty() {
        return Ok(());
    }
    let mut doc: DocumentMut = read_or_empty(base)?
        .parse()
        .map_err(|e| format!("{}: {e}", base.display()))?;
    for key in keys {
        let parts: Vec<&str> = key.split('.').collect();
        // The base may still say a renamed section under its old name.
        let old_names = crate::registry::RENAMED_SECTIONS
            .iter()
            .filter(|(_, new)| *new == parts[0])
            .map(|(old, _)| *old);
        for section in std::iter::once(parts[0]).chain(old_names) {
            if let Some(table) = doc
                .get_mut(section)
                .and_then(toml_edit::Item::as_table_like_mut)
            {
                remove_key(table, &parts[1..]);
            }
        }
    }
    let text = doc.to_string();
    // robotd validates the base on its own before merging, and refuses to start on one that
    // fails: a kept value may have been valid only beside a moved one.
    Params::parse_file(&text, base, false)
        .and_then(|params| params.validate(base))
        .map_err(|e| format!("the base without the moved keys would be invalid: {e}"))?;
    let staged = base.with_extension("toml.adopting");
    std::fs::File::create(&staged)
        .and_then(|mut file| {
            file.write_all(text.as_bytes())?;
            file.sync_all()
        })
        .map_err(|e| writable_hint(&staged, &e))?;
    std::fs::rename(&staged, base).map_err(|e| writable_hint(base, &e))?;
    if let Some(dir) = base.parent()
        && let Ok(dir) = std::fs::File::open(dir)
    {
        let _ = dir.sync_all();
    }
    Ok(())
}

fn remove_key(table: &mut dyn toml_edit::TableLike, parts: &[&str]) {
    match parts {
        [] => {}
        [last] => {
            table.remove(last);
        }
        [first, rest @ ..] => {
            if let Some(below) = table
                .get_mut(first)
                .and_then(toml_edit::Item::as_table_like_mut)
            {
                remove_key(below, rest);
            }
        }
    }
}

/// The base's owner edits under `prefix`, into `picked` at the same paths.
fn collect_edits(
    node: &toml::Table,
    prefix: &str,
    picked: &mut toml::Table,
    keys: &mut Vec<String>,
) {
    for (name, value) in node {
        let key = if prefix.is_empty() {
            name.clone()
        } else {
            format!("{prefix}.{name}")
        };
        if crate::registry::entry_for(&key).is_some() {
            if key != "board.version" && !ever_shipped(&key, value) {
                let mut at = &mut *picked;
                for part in key.split('.').take(key.split('.').count() - 1) {
                    at = at
                        .entry(part)
                        .or_insert_with(|| toml::Value::Table(toml::Table::new()))
                        .as_table_mut()
                        .expect("only tables are made here");
                }
                at.insert(name.clone(), value.clone());
                keys.push(key);
            }
        } else if let toml::Value::Table(below) = value {
            collect_edits(below, &key, picked, keys);
        }
    }
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
            // X, Y and the bumpers ship with no skill, and an empty binding is a button switched off.
            if !matches!(row.entry.key, "pad.x" | "pad.y" | "pad.lb" | "pad.rb") {
                assert!(!row.effective().is_empty(), "{}", row.entry.key);
            }
        }
        // Spot-check values against the daemon's documented defaults.
        let rows = m.rows();
        let find = |key: &str| rows.iter().find(|r| r.entry.key == key).expect("known");
        assert_eq!(find("control.hz").effective(), "50");
        assert_eq!(find("policy.mode").effective(), "walk");
        assert_eq!(find("safety.limp_fall").effective(), "true");
        assert_eq!(find("policy.voltage_adapt").effective(), "true");
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

    /// The image's file and an owner's layer over it, in one directory.
    fn layered(dir: &Path, base: &str, user: Option<&str>) -> (PathBuf, PathBuf) {
        let base_path = dir.join("base.toml");
        let user_path = dir.join("user.toml");
        std::fs::write(&base_path, base).expect("writes");
        if let Some(user) = user {
            std::fs::write(&user_path, user).expect("writes");
        }
        (base_path, user_path)
    }

    /// **An edit never touches the image's file.** It is the one an OS update replaces, so an
    /// edit written there is an edit the next update throws away.
    #[test]
    fn a_layered_edit_writes_the_user_layer_and_only_the_key() {
        let dir = tempfile::tempdir().expect("tempdir");
        let (base, user) = layered(dir.path(), SHIPPED, None);
        let mut m = Model::load_layered(&base, &user).expect("loads");
        assert_eq!(m.path, user, "the model writes the user layer");
        m.edit(entry("control.hz"), "100").expect("valid");
        m.save().expect("saves");
        assert_eq!(
            std::fs::read_to_string(&base).unwrap(),
            SHIPPED,
            "base untouched"
        );
        let written = std::fs::read_to_string(&user).expect("the user layer exists now");
        assert!(written.contains("hz = 100"), "{written}");
        assert!(
            !written.contains("port"),
            "only what was changed: {written}"
        );
        let p = Params::load_layered(&base, &user, true).unwrap();
        assert_eq!(p.control.hz, 100);
    }

    /// With a base, "default" is what the robot runs on with no override — the image's value.
    /// Typing that value clears the override, so the key keeps following the image.
    #[test]
    fn the_base_is_the_default_a_user_override_falls_back_to() {
        let dir = tempfile::tempdir().expect("tempdir");
        let (base, user) = layered(
            dir.path(),
            "[control]\nhz = 75\n",
            Some("[control]\nhz = 100\n"),
        );
        let mut m = Model::load_layered(&base, &user).expect("loads");
        let hz = |m: &Model| {
            m.rows()
                .into_iter()
                .find(|r| r.entry.key == "control.hz")
                .unwrap()
        };
        assert_eq!(
            hz(&m).default,
            "75",
            "the image's value, not the built-in 50"
        );
        assert_eq!(hz(&m).effective(), "100");
        m.edit(entry("control.hz"), "75").expect("valid");
        assert!(matches!(m.pending.get("control.hz"), Some(Edit::Clear)));
        m.save().expect("saves");
        assert!(!std::fs::read_to_string(&user).unwrap().contains("hz"));
    }

    /// A key another branch left in the base is named too, and against the base: adoption leaves
    /// unknown keys there, so on a layered robot that is where they are.
    #[test]
    fn unknown_keys_in_the_base_are_named_with_the_base() {
        let dir = tempfile::tempdir().expect("tempdir");
        let (base, user) = layered(
            dir.path(),
            "[policy]\nmoonwalk = true\n",
            Some("[kazoo]\nvoice = \"alto\"\n"),
        );
        let m = Model::load_layered(&base, &user).expect("opens like robotd starts");
        assert_eq!(m.ignored(), ["[kazoo]"]);
        let (path, keys) = m.ignored_in_base().expect("the base's are kept apart");
        assert_eq!(path, base);
        assert_eq!(keys, ["policy.moonwalk"]);
    }

    /// The editor's gate is the loader's: a user layer that would be set aside is never written.
    #[test]
    fn a_user_layer_robotd_would_set_aside_is_never_written() {
        let dir = tempfile::tempdir().expect("tempdir");
        let (base, user) = layered(dir.path(), "[control]\nhz = 50\n", None);
        let mut m = Model::load_layered(&base, &user).expect("loads");
        m.edit(entry("control.hz"), "0").expect("row-level ok");
        assert!(m.save().is_err());
        // The adoption before the write did run and wrote its (empty) layer; the edit did not.
        let written = std::fs::read_to_string(&user).expect("adopted");
        assert!(!written.contains("hz"), "nothing of the edit: {written}");
        assert!(
            !user.with_extension("toml.new").exists(),
            "nothing staged left behind"
        );
    }

    /// Adding a skill to a user layer with no list of its own keeps the image's skills: the
    /// user's list replaces the base's whole, so it starts as a copy.
    #[test]
    fn a_first_skill_in_the_user_layer_keeps_the_base_skills() {
        let dir = tempfile::tempdir().expect("tempdir");
        let (base, user) = layered(
            dir.path(),
            "[[policy.skill]]\nname = \"wave\"\npath = \"/srv/wave.onnx\"\nduration = 1.0\n",
            None,
        );
        let mut m = Model::load_layered(&base, &user).expect("loads");
        m.put_skill(&crate::SkillDef {
            name: "polite-bow".into(),
            path: Some(PathBuf::from("/srv/bow.onnx")),
            duration: 1.0,
            ..Default::default()
        })
        .expect("adds");
        let merged: Params = toml::from_str(&m.merged()).expect("parses");
        let names: Vec<_> = merged
            .policy
            .skills
            .iter()
            .map(|s| s.name.as_str())
            .collect();
        assert_eq!(names, ["wave", "polite-bow"]);

        // And removing the image's one takes it out of the robot, not just out of the layer.
        assert!(m.drop_skill("wave").expect("drops"));
        let merged: Params = toml::from_str(&m.merged()).expect("parses");
        let names: Vec<_> = merged
            .policy
            .skills
            .iter()
            .map(|s| s.name.as_str())
            .collect();
        assert_eq!(names, ["polite-bow"]);
    }

    /// The template this build ships has to read as *nothing changed*. A new live value added to
    /// `deploy/robotd.toml` without an [`EVER_SHIPPED`] entry would otherwise be copied into
    /// every fresh robot's user layer as if its owner had chosen it, and frozen there.
    #[test]
    fn every_live_template_value_is_listed() {
        let table: toml::Table = SHIPPED.parse().expect("parses");
        let (mut picked, mut keys) = (toml::Table::new(), Vec::new());
        collect_edits(&table, "", &mut picked, &mut keys);
        assert!(keys.is_empty(), "add these to EVER_SHIPPED: {keys:?}");
    }

    /// A robot installed from an older template, with three things its owner changed among what
    /// that template shipped. Only the owner's arrive in the user layer, under today's names, and
    /// only they leave the base — comments, the board and the template's values stay.
    #[test]
    fn an_old_robots_edits_are_adopted_and_its_template_values_are_not() {
        let dir = tempfile::tempdir().expect("tempdir");
        let old = "[board]\nversion = \"beta\"\n\
                   [control]\nhz = 100\n# how smooth\ncmd_alpha = 0.2\n\
                   [safety]\ndeadman_ms = 500\n\
                   [policy]\ngain = 200\n\
                   [[policy.skill]]\nname = \"polite-bow\"\npath = \"/srv/bow.onnx\"\nduration = 1.0\n\
                   [imu_head]\ngain = 0.7\n\
                   [health]\nstall_periods = 3\n";
        let (base, user) = layered(dir.path(), old, None);

        let mut keys = adopt_base_edits(&base, &user).expect("adopts");
        keys.sort();
        assert_eq!(
            keys,
            ["control.hz", "pad_imu_head_control.gain", "policy.skill"],
            "not board.version, not the old template's values, not a section this build lacks"
        );
        let left = std::fs::read_to_string(&base).unwrap();
        for gone in ["hz = 100", "gain = 0.7", "polite-bow"] {
            assert!(!left.contains(gone), "{gone} moved out: {left}");
        }
        for kept in [
            "version = \"beta\"",
            "# how smooth",
            "cmd_alpha = 0.2",
            "gain = 200",
            "stall_periods = 3",
        ] {
            assert!(left.contains(kept), "{kept} stays: {left}");
        }

        let p = Params::load_layered(&base, &user, true).unwrap();
        assert!(p.user_layer_rejected.is_none());
        assert_eq!(p.control.hz, 100);
        assert_eq!(p.pad_imu_head_control.gain, 0.7);
        assert_eq!(p.policy.skills.len(), 1);

        // And the editor now shows them for what they are: changed from the default, which a copy
        // left in the base would have hidden.
        let m = Model::load_layered(&base, &user).expect("the editor opens it");
        let hz = m
            .rows()
            .into_iter()
            .find(|r| r.entry.key == "control.hz")
            .unwrap();
        assert!(hz.differs(), "default {}", hz.default);
    }

    /// An OS update that came before this release: the install hook left the old slot's base
    /// beside the user layer, and the new slot's base is the image's. The edits come from the
    /// carried copy, which is then gone; the image's base is not touched.
    #[test]
    fn edits_carried_across_an_os_update_are_adopted_and_the_copy_removed() {
        let dir = tempfile::tempdir().expect("tempdir");
        let (base, user) = layered(dir.path(), SHIPPED, None);
        let carried = carried_path(&user);
        std::fs::write(&carried, "[control]\nhz = 100\n[policy]\ngain = 180\n").unwrap();

        let keys = adopt_base_edits(&base, &user).expect("adopts");
        assert_eq!(keys, ["control.hz", "policy.gain"]);
        assert!(!carried.exists(), "the carried copy is consumed");
        assert_eq!(std::fs::read_to_string(&base).unwrap(), SHIPPED);
        let p = Params::load_layered(&base, &user, true).unwrap();
        assert_eq!(p.control.hz, 100);
        assert_eq!(p.policy.gain, 180);
    }

    /// An edit made before `robotd` ever adopted — `robotctl configure` with the daemon stopped —
    /// must not create the user layer over the old edits and strand them in the base.
    #[test]
    fn the_first_edit_adopts_the_old_edits_before_writing() {
        let dir = tempfile::tempdir().expect("tempdir");
        let (base, user) = layered(dir.path(), "[control]\nhz = 100\n", None);
        let mut m = Model::load_layered(&base, &user).expect("loads");
        m.edit(entry("policy.gain"), "150").expect("valid");
        m.save().expect("saves");
        let p = Params::load_layered(&base, &user, true).unwrap();
        assert_eq!(p.control.hz, 100, "the old edit, adopted");
        assert_eq!(p.policy.gain, 150, "and the new one");
        let written = std::fs::read_to_string(&user).unwrap();
        assert!(written.contains("hz = 100"), "in the user layer: {written}");
    }

    /// The editor opens after the move, not before: the old edit is a change from the built-in
    /// default, and typing it back after changing it is a change too — not a "revert" that the
    /// move would then turn into losing the edit.
    #[test]
    fn the_editor_opens_on_a_robot_whose_edits_have_moved() {
        let dir = tempfile::tempdir().expect("tempdir");
        let (base, user) = layered(dir.path(), "[policy]\ngain = 180\n", None);
        let mut m = Model::adopt_and_load(&base, &user).expect("loads");
        let gain = |m: &Model| {
            m.rows()
                .into_iter()
                .find(|r| r.entry.key == "policy.gain")
                .unwrap()
        };
        assert_eq!(gain(&m).default, "200", "the built-in, not the old edit");
        assert!(gain(&m).differs());
        m.edit(entry("policy.gain"), "150").expect("valid");
        m.edit(entry("policy.gain"), "180").expect("valid");
        m.save().expect("saves");
        let p = Params::load_layered(&base, &user, true).unwrap();
        assert_eq!(p.policy.gain, 180);
    }

    /// A strip that would leave a base robotd refuses on its own is not done: the band is valid
    /// only with both thresholds, and without the moved one the default pause sits below resume.
    #[test]
    fn a_strip_that_would_break_the_base_alone_is_refused() {
        let dir = tempfile::tempdir().expect("tempdir");
        let text = "[pickup]\npause_threshold = 0.9\nresume_threshold = 0.85\n";
        let (base, _) = layered(dir.path(), text, None);
        assert!(strip_from_base(&base, &["pickup.pause_threshold".to_owned()]).is_err());
        assert_eq!(
            std::fs::read_to_string(&base).unwrap(),
            text,
            "base untouched"
        );
        assert!(Params::load(&base, true).is_ok());
    }

    /// A move interrupted after the user layer was written — power cut, failed strip — is
    /// finished on the next pass: the leftover equal to the user's value leaves the base, and a
    /// base value the user layer overrides with something else stays.
    #[test]
    fn an_interrupted_move_is_finished_later() {
        let dir = tempfile::tempdir().expect("tempdir");
        let (base, user) = layered(
            dir.path(),
            "[control]\nhz = 100\n[policy]\ngain = 180\n",
            Some("[control]\nhz = 100\n[policy]\ngain = 150\n"),
        );
        assert!(adopt_base_edits(&base, &user).unwrap().is_empty());
        let left = std::fs::read_to_string(&base).unwrap();
        assert!(!left.contains("hz"), "the leftover is gone: {left}");
        assert!(
            left.contains("gain = 180"),
            "a different value stays: {left}"
        );
    }

    /// Once there is a user layer, this never runs again — not over the owner's later edits.
    #[test]
    fn adoption_happens_once() {
        let dir = tempfile::tempdir().expect("tempdir");
        let (base, user) = layered(
            dir.path(),
            "[control]\nhz = 100\n",
            Some("[control]\nhz = 60\n"),
        );
        assert!(adopt_base_edits(&base, &user).unwrap().is_empty());
        assert_eq!(
            std::fs::read_to_string(&user).unwrap(),
            "[control]\nhz = 60\n"
        );
    }

    /// A robot nobody changed gets an empty user layer — written anyway, so the question is asked
    /// once — and runs exactly as before.
    #[test]
    fn an_untouched_robot_adopts_nothing() {
        let dir = tempfile::tempdir().expect("tempdir");
        let (base, user) = layered(dir.path(), SHIPPED, None);
        assert!(adopt_base_edits(&base, &user).unwrap().is_empty());
        assert!(user.exists());
        assert_eq!(
            std::fs::read_to_string(&base).unwrap(),
            SHIPPED,
            "nothing to move"
        );
        let p = Params::load_layered(&base, &user, true).unwrap();
        assert!(p.user_layer_rejected.is_none());
        assert!(Model::load_layered(&base, &user).is_ok());
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

    /// A key another branch left behind — a `policy.moonwalk`, a whole `[kazoo]` — does
    /// not stop the editor opening a file `robotd` itself starts on. The keys are named, an edit
    /// beside them saves, and they are still in the file for the branch that reads them.
    #[test]
    fn keys_from_another_branch_are_ignored_and_kept() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("robotd.toml");
        std::fs::write(
            &path,
            "[policy]\nmode = \"walk\"\nmoonwalk = true\n\n\
             [kazoo]\nvoice = \"alto\"\n\n[pad]\nx = \"polite-bow\"\n",
        )
        .unwrap();

        let mut m = Model::load(&path).expect("opens like robotd starts");
        assert_eq!(m.ignored(), ["[kazoo]", "policy.moonwalk"]);
        m.edit(entry("audio.enabled"), "false").expect("edits");
        m.save().expect("saves");

        let written = std::fs::read_to_string(&path).unwrap();
        assert!(written.contains("moonwalk = true"), "{written}");
        assert!(written.contains("voice = \"alto\""), "{written}");
        assert!(!Params::load(&path, true).unwrap().audio.enabled);
        // The bindings robotd serves over `pad.bind` are the file's, not the defaults.
        assert_eq!(super::pad_bindings(&path).unwrap().x, "polite-bow");
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
        assert!(walk.contains("velstand"), "{walk}");
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
            bindings.b, "ground_pick",
            "the rest resolve to their defaults"
        );
    }

    /// Binding a button back to its default *removes* the key rather than pinning it, so the
    /// file stays a list of decisions — which is what makes `configure --list` mean anything.
    #[test]
    fn binding_the_default_removes_the_key() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("robotd.toml");
        std::fs::write(&path, "[pad]\nb = \"polite-bow\"\n").unwrap();

        super::bind_pad(&path, "b", "ground_pick").unwrap();
        let written = std::fs::read_to_string(&path).unwrap();
        assert!(!written.contains("polite-bow"), "{written}");
        assert!(
            !written.contains("b ="),
            "the default is not pinned: {written}"
        );
        assert_eq!(super::pad_bindings(&path).unwrap().b, "ground_pick");
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
        assert_eq!(bindings.a, "sit_toggle");
        assert_eq!(bindings.b, "ground_pick");
    }

    /// The shipped file, its `[board]` commented out, gains a real key — the default included,
    /// which an ordinary edit would have cleared — and keeps its comments.
    #[test]
    fn declaring_the_board_writes_even_the_default() {
        use crate::board::Board;
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("robotd.toml");
        std::fs::write(&path, SHIPPED).unwrap();
        assert_eq!(Board::declared(&path), None);

        set_board(&path, Board::Zero3).unwrap();
        assert_eq!(Board::declared(&path), Some(Board::Zero3));
        set_board(&path, Board::Beta).unwrap();
        assert_eq!(Board::declared(&path), Some(Board::Beta));

        let written = std::fs::read_to_string(&path).unwrap();
        assert!(written.contains("# Which electronic board"), "{written}");
        assert_eq!(written.matches("[board]").count(), 1, "{written}");
    }

    #[test]
    fn nested_axis_edits_preserve_defaults_comments_and_inline_tables() {
        for text in [
            "# my pad\n[pad_axes.drive.vyaw]\n# reverse steering\ninvert = false\n",
            "# my pad\npad_axes = { drive = { vyaw = { invert = false } } }\n",
        ] {
            let dir = tempfile::tempdir().unwrap();
            let path = dir.path().join("robotd.toml");
            std::fs::write(&path, text).unwrap();
            let mut model = Model::load(&path).unwrap();
            model
                .edit(entry("pad_axes.drive.vyaw.gain"), "0.5")
                .unwrap();
            model.save().unwrap();
            let params = crate::Params::load(&path, true).unwrap();
            assert_eq!(params.pad_axes.drive.vyaw.gain, 0.5);
            assert!(!params.pad_axes.drive.vyaw.invert);
            assert_eq!(
                params.pad_axes.drive.vyaw.source,
                crate::pad_axes::AxisSource::RightX
            );
            assert!(std::fs::read_to_string(&path).unwrap().contains("# my pad"));
            model
                .edit(entry("pad_axes.drive.vyaw.gain"), "1.0")
                .unwrap();
            model.save().unwrap();
            let written = std::fs::read_to_string(&path).unwrap();
            let document: toml::Value = toml::from_str(&written).unwrap();
            assert!(
                document["pad_axes"]["drive"]["vyaw"].get("gain").is_none(),
                "override still written: {written}"
            );
            assert_eq!(
                document["pad_axes"]["drive"]["vyaw"]["invert"].as_bool(),
                Some(false)
            );
            assert!(written.contains("# my pad"));
            if text.contains("# reverse steering") {
                assert!(written.contains("# reverse steering"));
            }
            assert_eq!(
                crate::Params::load(&path, true)
                    .unwrap()
                    .pad_axes
                    .drive
                    .vyaw
                    .gain,
                1.0
            );
        }
    }

    /// Sections come out in registry order, once each — the editor's headers.
    #[test]
    fn sections_are_ordered_and_unique() {
        let s = sections();
        assert_eq!(
            s,
            vec![
                "board",
                "bus",
                "control",
                "update_gate",
                "policy",
                "safety",
                "duck_detector",
                "chorale",
                "idle",
                "pickup",
                "theremin",
                "head_imu",
                "audio",
                "media",
                // Last, and the editor shows sections in this order: the pad is what a robot's
                // buttons do, which is the thing somebody browses for rather than tunes.
                "pad",
                "pad_imu_head_control",
                "pad_drive",
                "pad_axes",
                "pad_head",
                "pad_body",
                "pad_roller",
                "netpad"
            ]
        );
    }

    /// A file written before `[imu_head]` became `[pad_imu_head_control]` loads (the alias), and
    /// the editor carries the section to its new name so a save under the new key cannot leave two
    /// sections the loader would refuse as duplicates. The value set under the old name is what the
    /// new key reports.
    #[test]
    fn a_renamed_section_is_carried_to_its_new_name() {
        let dir = tempfile::tempdir().expect("tempdir");
        let config = dir.path().join("robotd.toml");
        std::fs::write(&config, "[imu_head]\nenabled = true\ngain = 0.5\n").expect("write");

        let mut m = Model::load(&config).expect("loads through the alias");
        let row = m
            .rows()
            .into_iter()
            .find(|r| r.entry.key == "pad_imu_head_control.gain")
            .expect("registry key");
        assert_eq!(
            row.set.as_deref(),
            Some("0.5"),
            "the old section's value shows under the new key"
        );

        m.edit(entry("pad_imu_head_control.enabled"), "false")
            .expect("valid");
        m.save().expect("saves");
        let text = std::fs::read_to_string(&config).expect("read");
        assert!(!text.contains("[imu_head]"), "old header gone:\n{text}");
        assert!(
            text.contains("[pad_imu_head_control]"),
            "new header present:\n{text}"
        );
        assert!(
            text.contains("gain = 0.5"),
            "untouched key carried over:\n{text}"
        );
        let params: Params = toml::from_str(&text).expect("the daemon loads the result");
        assert!(!params.pad_imu_head_control.enabled);
        assert_eq!(params.pad_imu_head_control.gain, 0.5);
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

    /// A dry-run has to run the same checks as a write — including that the name is not one
    /// the daemon drives itself — and then leave the file as it was. A preview that wrote
    /// would be the opposite of the promise.
    #[test]
    fn previewing_a_skill_validates_and_writes_nothing() {
        let dir = tempfile::tempdir().expect("tempdir");
        let config = dir.path().join("robotd.toml");
        let original = "[policy]\n";
        std::fs::write(&config, original).expect("write");

        preview_skill(
            &config,
            &crate::SkillDef {
                name: "polite-bow".into(),
                path: Some(PathBuf::from("/srv/bow.onnx")),
                duration: 4.0,
                ..Default::default()
            },
        )
        .expect("valid");
        assert_eq!(std::fs::read_to_string(&config).expect("read"), original);
        assert!(
            !config.with_extension("toml.new").exists(),
            "the staging file must not remain"
        );

        let refusal = preview_skill(
            &config,
            &crate::SkillDef {
                name: "ground_pick".into(),
                duration: 1.0,
                ..Default::default()
            },
        )
        .expect_err("refused");
        assert!(refusal.contains("ground_pick"), "{refusal}");
        assert_eq!(
            std::fs::read_to_string(&config).expect("read"),
            original,
            "a refused preview writes nothing either"
        );
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
