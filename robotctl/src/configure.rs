//! `robotctl configure` — edit `robotd.toml` without reading a wall of comments.
//!
//! The shipped `deploy/robotd.toml` is deliberately exhaustive: every key, documented at
//! paragraph length, all of it commented out. That is the right *reference* and a poor
//! *editing surface* — finding the one switch you want means scrolling four hundred lines of
//! prose. This is the editing surface: every key the daemon knows, the feature switches first,
//! current value against default, one line of doc, toggle and type in place.
//!
//! ## Where the truth lives
//!
//! Nothing here defines a key. The schema, the defaults, the validation and the one-line docs
//! all come from `robotd-params` — the same crate `robotd` itself parses the file with — and
//! its registry is pinned complete by a test over `Params`'s own serialization. When a section
//! is added to the daemon, this editor learns it at compile time or the build fails; it can be
//! wrong about nothing.
//!
//! ## How edits are applied
//!
//! The file is parsed with `toml_edit`, which preserves everything it does not touch —
//! comments, ordering, keys from releases this build does not know. Edits set or remove
//! exactly the keys changed. Before anything is written, the candidate is re-parsed through
//! `Params::load` — the daemon's own gate, range checks included — so this tool cannot write a
//! file `robotd` would refuse to start on.
//!
//! That preservation and that gate used to contradict each other, and the contradiction was
//! reachable: a board carrying a section from a branch kept it here by design, and then `load`
//! rejected the whole file, so no edit could be saved at all until someone deleted a section that
//! did nothing. `load` now names unknown keys and ignores them, which makes both halves true at
//! once. Typos are still caught, one layer up: this only ever writes keys the registry knows.
//!
//! Writes are atomic (temp file + rename beside the target), because half a config at the
//! moment of a power cut is a robot that will not start.
//!
//! ## Restart
//!
//! The daemons read the file **once at startup** (`robotd-params` docs) — so every change
//! requires a restart, and the exit flow offers one whenever anything was written. *Which*
//! daemon is derived from the keys that changed, not assumed: `[media]` is `mediad` reading the
//! same file, and a "restart robotd" offer over a video setting is an edit that reads as having
//! done nothing at all. [`unit_for`] is that mapping and [`units_for`] applies it.
//!
//! The file is root-owned; run as `sudo robotctl configure` to actually write.

use std::collections::BTreeMap;
use std::io::Write as _;
use std::path::{Path, PathBuf};

use robotd_params::Params;
use robotd_params::registry::{Entry, Kind, REGISTRY};
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

    fn from_text(path: &Path, text: &str) -> Result<Self, String> {
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

    /// Validate the pending edits through the daemon's own gate, then write atomically.
    ///
    /// Validation goes through a real file and [`Params::load`] rather than a bare parse,
    /// because `load` is what `robotd` runs at startup — range checks included. What this tool
    /// writes, the daemon starts on.
    pub fn save(&mut self) -> Result<(), String> {
        let text = self.rendered();
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
}

/// A value as the UI shows it — the data alone. Strings lose their quotes, and everything
/// loses its decor: `to_string` on a `toml_edit` value carries the whitespace and any inline
/// comment along, which is how `50 # do not touch` once ended up in a value cell.
fn render(value: &toml_edit::Value) -> String {
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

/// Permission errors get the actual fix, because the file is root-owned by design.
fn writable_hint(path: &Path, e: &std::io::Error) -> String {
    if e.kind() == std::io::ErrorKind::PermissionDenied {
        format!(
            "cannot write {}: permission denied — run `sudo robotctl configure`",
            path.display()
        )
    } else {
        format!("cannot write {}: {e}", path.display())
    }
}

/// Which daemon reads a section, and so which unit a change to it needs restarted.
///
/// `robotd` parses this file for itself; `[media]` and `[detect]` are `mediad` reading the same
/// file, because a per-board setting belongs in the per-board config rather than on a unit file the
/// release installer rewrites — and because the camera frames `[detect]` is about are on `mediad`'s
/// tee. Being wrong here is an edit that appears to do nothing until the next reboot — which is
/// exactly what the restart offer exists to prevent, so it is derived from the keys that changed
/// rather than assumed.
fn unit_for(section: &str) -> &'static str {
    match section {
        "media" | "detect" => "mediad",
        _ => "robotd",
    }
}

/// The units a set of `section.key` names requires restarting, in start order, without duplicates.
fn units_for_keys<'a>(keys: impl Iterator<Item = &'a str>) -> Vec<&'static str> {
    // `robotd` first, because `mediad.service` is `After=robotd.service`: restarting in the
    // other order means mediad reconnects to a robotd that is about to go away.
    let mut units: Vec<&'static str> = Vec::new();
    for key in keys {
        let (section, _) = key.split_once('.').expect("registry keys are section.key");
        let unit = unit_for(section);
        if !units.contains(&unit) {
            units.push(unit);
        }
    }
    units.sort_unstable_by_key(|unit| *unit != "robotd");
    units
}

/// The units the pending edits require restarting, in start order, without duplicates.
///
/// Empty is a real answer — no edits, nothing to restart — and the caller must not offer a
/// restart for it. Read *before* a save, which clears the pending map.
pub fn units_for(model: &Model) -> Vec<&'static str> {
    units_for_keys(model.pending.keys().copied())
}

/// The daemons that read the keys somebody just changed.
///
/// The same mapping as [`units_for`], from what a save recorded rather than from what is still
/// pending — which is what the exit flow has to work from, because the save already cleared the
/// other one.
pub fn units_to_restart(edited: &[String]) -> Vec<&'static str> {
    units_for_keys(edited.iter().map(String::as_str))
}

/// Restart units, reporting rather than hiding the outcome.
///
/// One `systemctl` invocation for all of them: it starts them in the units' own declared order,
/// which is what `After=` is for, and it means one password prompt rather than one per daemon.
pub fn restart_units(units: &[&str]) -> Result<(), String> {
    if units.is_empty() {
        return Ok(());
    }
    let status = std::process::Command::new("systemctl")
        .arg("restart")
        .args(units)
        .status()
        .map_err(|e| format!("cannot run systemctl: {e}"))?;
    if status.success() {
        Ok(())
    } else {
        Err(format!(
            "systemctl restart {} failed — run it with sudo",
            units.join(" ")
        ))
    }
}

/// A short human summary of the pending edits, for the confirm screen.
pub fn summary(model: &Model) -> Vec<String> {
    model
        .rows()
        .iter()
        .filter_map(|row| {
            let edit = model.pending.get(row.entry.key)?;
            Some(match edit {
                Edit::Set(value) => format!("{} = {}", row.entry.key, render(value)),
                Edit::Clear => format!("{} → default ({})", row.entry.key, row.default),
            })
        })
        .collect()
}

// ── the terminal UI ──────────────────────────────────────────────────────────
//
// One screen: feature switches first, then every section; a footer carrying the selected
// key's one-line doc; SPACE toggles what can be toggled, ENTER types what cannot. Kept to the
// `monitor`'s conventions (ratatui, `ratatui::init`/`restore`) and deliberately dumber — a
// config editor should feel like a settings menu, not a dashboard.

use ratatui::crossterm::event::{self, Event, KeyCode, KeyEventKind};
use ratatui::layout::{Constraint, Layout};
use ratatui::style::{Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Paragraph};

/// What the list shows at one line: a section header, or a key.
#[derive(Debug)]
enum Item {
    Header(&'static str),
    Key(usize),
}

/// Where input goes right now.
enum Focus {
    /// Moving around the list.
    List,
    /// Typing a value for the selected row.
    Editing {
        buffer: String,
        error: Option<String>,
    },
    /// Deciding what to do with the pending edits on the way out.
    Confirm,
    /// Everything written; offering the restart every change requires — of the daemons that
    /// actually read what changed, which is not always `robotd`.
    Restart { units: Vec<&'static str> },
}

/// Run the editor. Returns once the user has left, with everything saved or discarded.
pub fn run(path: &Path) -> Result<(), String> {
    // An interactive editor and nothing else: piped in or out, there is no sensible
    // behaviour to fall back to, and ratatui would panic trying to open the terminal.
    if !crate::monitor::stdout_is_a_terminal() {
        return Err("configure is interactive — run it in a terminal".to_owned());
    }
    let mut model = Model::load(path)?;
    let items = layout_items(&model);
    // First key, not the first header.
    let mut cursor = items
        .iter()
        .position(|item| matches!(item, Item::Key(_)))
        .unwrap_or(0);
    let mut focus = Focus::List;
    let mut saved = false;
    let mut status: Option<String> = None;

    let mut terminal = ratatui::init();
    let outcome = loop {
        let rows = model.rows();
        if let Err(e) = terminal.draw(|frame| {
            draw(
                frame,
                &model,
                &rows,
                &items,
                cursor,
                &focus,
                status.as_deref(),
            );
        }) {
            break Err(format!("terminal: {e}"));
        }

        let Ok(Event::Key(key)) = event::read() else {
            continue;
        };
        if key.kind != KeyEventKind::Press {
            continue;
        }
        status = None;

        match &mut focus {
            Focus::List => match key.code {
                KeyCode::Char('q') | KeyCode::Esc => {
                    if model.pending.is_empty() {
                        break Ok(saved);
                    }
                    focus = Focus::Confirm;
                }
                KeyCode::Up | KeyCode::Char('k') => cursor = step(&items, cursor, -1),
                KeyCode::Down | KeyCode::Char('j') => cursor = step(&items, cursor, 1),
                KeyCode::Char(' ') => {
                    if let Item::Key(index) = items[cursor] {
                        let row = &rows[index];
                        match model.toggled(row) {
                            Some(next) => {
                                let entry = row.entry;
                                if let Err(e) = model.edit(entry, &next) {
                                    status = Some(e);
                                }
                            }
                            None => {
                                focus = Focus::Editing {
                                    buffer: row.effective().to_owned(),
                                    error: None,
                                };
                            }
                        }
                    }
                }
                KeyCode::Enter => {
                    if let Item::Key(index) = items[cursor] {
                        focus = Focus::Editing {
                            buffer: rows[index].effective().to_owned(),
                            error: None,
                        };
                    }
                }
                KeyCode::Char('u') | KeyCode::Char('d') => {
                    if let Item::Key(index) = items[cursor] {
                        model.pending.insert(rows[index].entry.key, Edit::Clear);
                    }
                }
                _ => {}
            },
            Focus::Editing { buffer, error } => match key.code {
                KeyCode::Esc => focus = Focus::List,
                KeyCode::Enter => {
                    if let Item::Key(index) = items[cursor] {
                        match model.edit(rows[index].entry, buffer) {
                            Ok(()) => focus = Focus::List,
                            Err(e) => *error = Some(e),
                        }
                    }
                }
                KeyCode::Backspace => {
                    buffer.pop();
                    *error = None;
                }
                KeyCode::Char(c) => {
                    buffer.push(c);
                    *error = None;
                }
                _ => {}
            },
            Focus::Confirm => match key.code {
                KeyCode::Char('y') | KeyCode::Enter => {
                    // Read before the save, which clears `pending` — after it there is nothing
                    // left to say which daemons were affected.
                    let units = units_for(&model);
                    match model.save() {
                        Ok(()) => {
                            saved = true;
                            focus = Focus::Restart { units };
                        }
                        Err(e) => {
                            status = Some(e);
                            focus = Focus::List;
                        }
                    }
                }
                KeyCode::Char('n') => break Ok(saved),
                KeyCode::Esc => focus = Focus::List,
                _ => {}
            },
            Focus::Restart { .. } => match key.code {
                // The restart itself happens after `ratatui::restore`, outside the alternate
                // screen, so systemctl's output is visible.
                KeyCode::Char('y') | KeyCode::Enter => break Ok(true),
                KeyCode::Char('n') | KeyCode::Esc | KeyCode::Char('q') => {
                    focus = Focus::List;
                    break Ok(saved);
                }
                _ => {}
            },
        }
    };
    let restart_wanted = match &focus {
        Focus::Restart { units } => units.clone(),
        _ => Vec::new(),
    };
    // What was *written*, not what is pending: the save already happened inside the loop above and
    // cleared the pending map, which is why reading it here restarted nothing at all.
    let edited = model.written().to_vec();
    ratatui::restore();

    let saved = outcome?;
    if !restart_wanted.is_empty() {
        let names = restart_wanted.join(" and ");
        println!("restarting {names}…");
        restart_units(&restart_wanted)?;
        println!("{names} restarted");
    } else if saved {
        let names = units_to_restart(&edited).join(" and ");
        println!(
            "written to {} — changes apply on the next `systemctl restart {names}`",
            path.display()
        );
    }
    Ok(())
}

/// The list: feature switches first under their own header, then every section.
fn layout_items(model: &Model) -> Vec<Item> {
    let rows = model.rows();
    let mut items = Vec::new();
    items.push(Item::Header("features"));
    for (index, row) in rows.iter().enumerate() {
        if row.entry.feature {
            items.push(Item::Key(index));
        }
    }
    for section in sections() {
        let keys: Vec<usize> = rows
            .iter()
            .enumerate()
            .filter(|(_, row)| {
                let (s, _) = row.entry.key.split_once('.').expect("section.key");
                s == section && !row.entry.feature
            })
            .map(|(index, _)| index)
            .collect();
        // A section whose every key is a feature switch — `[chorale]`, which is one opt-in bool
        // and nothing else — has all of them hoisted into the features block above. Drawing its
        // header anyway leaves a heading with nothing under it, which reads as "this section has
        // no settings" rather than "its setting is up there".
        if keys.is_empty() {
            continue;
        }
        items.push(Item::Header(section));
        items.extend(keys.into_iter().map(Item::Key));
    }
    items
}

/// Move the cursor to the next key in `direction`, skipping headers, stopping at the ends.
fn step(items: &[Item], cursor: usize, direction: isize) -> usize {
    let mut at = cursor as isize;
    loop {
        at += direction;
        if at < 0 || at as usize >= items.len() {
            return cursor;
        }
        if matches!(items[at as usize], Item::Key(_)) {
            return at as usize;
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn draw(
    frame: &mut ratatui::Frame,
    model: &Model,
    rows: &[Row],
    items: &[Item],
    cursor: usize,
    focus: &Focus,
    status: Option<&str>,
) {
    let [list_area, footer_area] =
        Layout::vertical([Constraint::Min(3), Constraint::Length(4)]).areas(frame.area());

    // The visible window of the list, kept around the cursor.
    let height = list_area.height.saturating_sub(2) as usize;
    let first = cursor
        .saturating_sub(height / 2)
        .min(items.len().saturating_sub(height.max(1)));
    let mut lines: Vec<Line> = Vec::new();
    // Whether the row being drawn sits under the [features] header — recovered by scanning
    // back to the nearest header, since the window may start mid-block.
    let block_of = |at: usize| {
        items[..=at]
            .iter()
            .rev()
            .find_map(|item| match item {
                Item::Header(section) => Some(*section == "features"),
                Item::Key(_) => None,
            })
            .unwrap_or(false)
    };
    for (at, item) in items.iter().enumerate().skip(first).take(height.max(1)) {
        let in_features = block_of(at);
        match item {
            Item::Header(section) => {
                lines.push(Line::from(Span::styled(
                    format!("[{section}]"),
                    Style::new().add_modifier(Modifier::BOLD).cyan(),
                )));
            }
            Item::Key(index) => {
                let row = &rows[*index];
                // Inside a section the short name reads best; the features block gathers keys
                // from *different* sections, where a bare `enabled` twice over says nothing.
                let name = if in_features {
                    row.entry.key
                } else {
                    row.entry.key.split_once('.').expect("section.key").1
                };
                // Two markers, both meaning what they look like: `*` you changed it this
                // session and have not saved; `•` this robot diverges from the default. A
                // key merely *written* in the file at its default value gets no mark — that
                // distinction confused everyone it was shown to, starting with the author's
                // own demo file.
                let marker = if model.pending.contains_key(row.entry.key) {
                    "*"
                } else if row.differs() {
                    "•"
                } else {
                    " "
                };
                // The colour means one thing: this robot runs something other than the
                // default. A default written out explicitly is set (•) but not different.
                let value = if row.differs() {
                    Span::styled(
                        format!("{} (default {})", row.effective(), row.default),
                        Style::new().yellow(),
                    )
                } else if let Some(resolved) = &row.resolved {
                    Span::styled(format!("{resolved} (auto)"), Style::new().dim())
                } else {
                    Span::styled(row.effective().to_owned(), Style::new().dim())
                };
                let mut line = Line::from(vec![
                    Span::raw(format!(" {marker} ")),
                    Span::raw(format!("{name:<30}")),
                    value,
                ]);
                if at == cursor {
                    line = line.style(Style::new().add_modifier(Modifier::REVERSED));
                }
                lines.push(line);
            }
        }
    }
    frame.render_widget(
        Paragraph::new(lines).block(
            Block::default()
                .borders(Borders::ALL)
                .title(format!(" {} ", model.path.display())),
        ),
        list_area,
    );

    // Footer: what the selected key is, and what the keys do — or the active prompt.
    let footer: Vec<Line> = match focus {
        Focus::Editing { buffer, error } => vec![
            Line::from(format!("new value: {buffer}▏")),
            Line::from(match error {
                Some(e) => Span::styled(e.clone(), Style::new().red()),
                None => Span::raw("ENTER apply · ESC cancel"),
            }),
        ],
        Focus::Confirm => {
            let changes = summary(model).join(", ");
            vec![
                Line::from(format!("save {} change(s)? {changes}", model.pending.len())),
                Line::from("y save · n discard · ESC back"),
            ]
        }
        // Which daemons, by name: `[media]` is read by `mediad`, and "restart it" over a
        // change that needs the *other* daemon is how an edit reads as having done nothing.
        Focus::Restart { units } => {
            let names = units.join(" and ");
            let reads = if units.len() == 1 { "reads" } else { "read" };
            vec![
                Line::from(format!(
                    "written. {names} {reads} the config once at startup —"
                )),
                Line::from(format!("restart {names} now? y restart · n later")),
            ]
        }
        Focus::List => {
            let doc = match items.get(cursor) {
                Some(Item::Key(index)) => rows[*index].entry.doc,
                _ => "",
            };
            vec![
                Line::from(match status {
                    Some(s) => Span::styled(s.to_owned(), Style::new().red()),
                    None => Span::raw(doc),
                }),
                Line::from("↑↓ move · SPACE toggle · ENTER edit · u default · q quit"),
            ]
        }
    };
    frame.render_widget(
        Paragraph::new(footer).block(Block::default().borders(Borders::ALL)),
        footer_area,
    );
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
        robotd_params::registry::entry_for(key).expect("a registry key")
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

    /// The restart offer names the daemon that reads what changed. `[media]` is read by
    /// `mediad`, and offering a `robotd` restart for it is an edit that reads as having done
    /// nothing at all until somebody reboots.
    #[test]
    fn the_restart_offer_names_the_daemon_that_reads_the_change() {
        let mut m = model("");
        m.edit(entry("media.quality"), "360p30").expect("valid");
        assert_eq!(units_for(&m), vec!["mediad"]);

        let mut m = model("");
        m.edit(entry("control.hz"), "60").expect("valid");
        assert_eq!(units_for(&m), vec!["robotd"]);

        // Both, and robotd first: mediad.service is After=robotd.service, so the other order
        // reconnects mediad to a robotd that is about to go away.
        let mut m = model("");
        m.edit(entry("media.camera"), "false").expect("valid");
        m.edit(entry("audio.enabled"), "false").expect("valid");
        assert_eq!(units_for(&m), vec!["robotd", "mediad"]);

        // Nothing pending is nothing to restart, and the caller must not offer one.
        assert!(units_for(&model("")).is_empty());
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

    /// The whole first screen renders without panicking, features first — the same
    /// TestBackend trick the monitor's tests use, so the layout code is exercised without a
    /// terminal.
    #[test]
    fn the_first_screen_renders_with_features_first() {
        let m = model("[policy]\nmode = \"roller\"\n");
        let rows = m.rows();
        let items = layout_items(&m);
        let cursor = items
            .iter()
            .position(|item| matches!(item, Item::Key(_)))
            .expect("there are keys");
        let mut terminal =
            ratatui::Terminal::new(ratatui::backend::TestBackend::new(90, 30)).expect("terminal");
        terminal
            .draw(|frame| draw(frame, &m, &rows, &items, cursor, &Focus::List, None))
            .expect("draws");
        let screen = format!("{:?}", terminal.backend().buffer());
        assert!(screen.contains("[features]"), "features head the list");
        // In the features block keys keep their section — two bare `enabled`s from different
        // sections were indistinguishable.
        assert!(screen.contains("policy.enabled"), "{screen}");
        assert!(screen.contains("audio.enabled"), "{screen}");
        // The divergence annotation: mode is set away from default.
        assert!(screen.contains("roller (default walk)"), "{screen}");
    }

    /// A section whose only key is a feature switch draws no header.
    ///
    /// `[chorale]` is exactly that — one opt-in bool — and it appeared on the board as a bare
    /// heading with nothing under it, which reads as a section that forgot its settings. The
    /// switch itself must still be there, in the features block, or this "fix" hides it.
    #[test]
    fn an_all_feature_section_has_no_empty_header() {
        let m = model("");
        let items = layout_items(&m);
        let headers: Vec<&str> = items
            .iter()
            .filter_map(|item| match item {
                Item::Header(h) => Some(*h),
                Item::Key(_) => None,
            })
            .collect();
        assert!(
            !headers.contains(&"chorale"),
            "chorale's only key is a feature switch: {headers:?}"
        );
        // Every header that *is* drawn has at least one key under it.
        for (at, item) in items.iter().enumerate() {
            if matches!(item, Item::Header(_)) {
                assert!(
                    matches!(items.get(at + 1), Some(Item::Key(_))),
                    "empty header at {at}: {items:?}"
                );
            }
        }
        // And the switch is still reachable, up in the features block.
        let rows = m.rows();
        assert!(
            items.iter().any(|item| match item {
                Item::Key(index) => rows[*index].entry.key == "chorale.accept",
                Item::Header(_) => false,
            }),
            "chorale.accept must still be editable"
        );
    }

    /// A save records what it wrote, because that is what decides the restart.
    ///
    /// The bug this pins: `save` clears `pending`, and the restart decision is made after the
    /// editor closes — so reading `pending` there found an empty map, `units_to_restart` returned
    /// nothing, and turning the detector off looked like it had no effect at all. Twice, on a
    /// robot, before anybody suspected the editor rather than the daemon.
    #[test]
    fn a_save_remembers_what_it_wrote_so_the_right_daemon_restarts() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("robotd.toml");
        std::fs::write(&path, "").unwrap();
        let mut m = Model::load(&path).expect("loads");

        assert!(m.written().is_empty(), "nothing written yet");
        m.edit(entry("detect.enabled"), "true").expect("edits");
        assert!(!m.pending.is_empty());
        m.save().expect("saves");

        assert!(m.pending.is_empty(), "a save clears what is pending");
        assert_eq!(m.written(), ["detect.enabled".to_owned()]);
        assert_eq!(units_to_restart(m.written()), vec!["mediad"]);

        // A second save adds to the record rather than replacing it: somebody who changes the
        // detector and then the gait wants both daemons restarted.
        m.edit(entry("policy.mode"), "roller").expect("edits");
        m.save().expect("saves");
        assert_eq!(units_to_restart(m.written()), vec!["robotd", "mediad"]);
    }

    /// A `[detect]` change restarts `mediad`, not `robotd`.
    ///
    /// `robotd` owned every key in this file for long enough that the restart was hardcoded, and
    /// `[detect]` is read by `mediad` because the camera frames are on its tee. Restarting the
    /// wrong daemon is how somebody edits a value three times and swears it does nothing.
    #[test]
    fn the_section_decides_which_daemon_restarts() {
        let detect = vec!["detect.enabled".to_owned()];
        assert_eq!(units_to_restart(&detect), vec!["mediad"]);

        let policy = vec!["policy.mode".to_owned()];
        assert_eq!(units_to_restart(&policy), vec!["robotd"]);

        // Both, in the order they are least disruptive to restart: the control loop first, then the
        // camera — a robot that is standing up should not be waiting on a WebRTC teardown.
        let both = vec!["detect.hz".to_owned(), "audio.enabled".to_owned()];
        assert_eq!(units_to_restart(&both), vec!["robotd", "mediad"]);

        assert!(units_to_restart(&[]).is_empty());
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
                "maploc",
                "audio",
                "media"
            ]
        );
    }
}
