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
//! Not here. [`robotd_params::edit`] is the writer — `toml_edit` over the daemon's own schema,
//! validated through `Params::load` before anything reaches the disk, written atomically — and it
//! lives beside that schema because it is no longer the only caller. `robotctl policy` and
//! `robotctl pad` write keys of their own, and a daemon serving `pad.bind` over the radio has to
//! write the same file. A second implementation would drift, and what it would drift on is the
//! validation.
//!
//! What is left in this module is the part that is genuinely an operator tool's: which systemd
//! unit a change needs restarted, restarting it, printing a divergence list, and the full-screen
//! editor.
//!
//! ## Making a change take
//!
//! Mostly the daemons read the file **once at startup** (`robotd-params` docs), so mostly a
//! change needs a restart — and *which* daemon is derived from the keys that changed rather than
//! assumed: `[media]` is `mediad` reading the same file, and a "restart robotd" offer over a
//! video setting is an edit that reads as having done nothing at all.
//!
//! Mostly, not always, and the exceptions are where offering a restart is worst. `padd` re-reads
//! `[pad]` and `[pad_imu_head_control]` a second after the file changes, so a restart there drops the pad
//! session — and robotd's deadman with it — to apply what would have applied by itself. `robotd`
//! re-reads `[policy]` on a call, so a restart there takes motor control away from a standing
//! robot to change a number it would have taken standing up.
//!
//! [`Apply`] is what a key needs, [`apply_for`] is the mapping, and [`Plan`] is that answer for a
//! set of keys — what to restart, what to reload, and what needs nothing at all.
//!
//! The file is root-owned; run as `sudo robotctl configure` to actually write.

use std::path::Path;

// The editing model itself lives in `robotd-params`, beside the schema it validates against —
// see that module's header for why. Re-exported rather than imported privately because the rest
// of `robotctl` reaches for `configure::Model` and should not have to know it moved.
pub use robotd_params::edit::{Edit, Model, Row, bind_pad, pad_bindings, render, sections};

/// What a written key needs before the daemon that reads it is running on it.
///
/// Every variant names that daemon, because every message the exit flow prints names it: an offer
/// that says "restart" without saying *what* is how `[head_imu]` came to restart `robotd`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Apply {
    /// Read once at startup. The unit has to go down and come back, and everything it was doing
    /// stops for as long as that takes.
    Restart(&'static str),
    /// A running daemon will re-read it on a call. Cheaper than a restart in the way that
    /// matters: the motors stay powered.
    Reload(&'static str),
    /// The daemon re-reads the file by itself. Nothing to do but say so.
    Live(&'static str),
}

impl Apply {
    /// The daemon, whatever the answer was.
    fn unit(self) -> &'static str {
        match self {
            Self::Restart(unit) | Self::Reload(unit) | Self::Live(unit) => unit,
        }
    }
}

/// What a change to `section.key` needs, and from which daemon.
///
/// `robotd` parses this file for itself; `[media]` and `[duck_detector]` are `mediad` reading the same
/// file, because a per-board setting belongs in the per-board config rather than on a unit file the
/// release installer rewrites — and because the camera frames `[duck_detector]` is about are on `mediad`'s
/// tee. Being wrong here is an edit that appears to do nothing until the next reboot — which is
/// exactly what the offer exists to prevent, so it is derived from the keys that changed rather
/// than assumed.
///
/// **Every section is listed, and there is no fallback.** `[head_imu]` shipped reading as `robotd`
/// because a `_ => "robotd"` arm answered for it: enabling the head IMU restarted the daemon that
/// does not read the key and left `tofd` on the old value, so the switch did nothing and said
/// nothing. A section with no arm here now fails `every_registry_key_says_how_it_applies` rather
/// than picking up whichever daemon the catch-all happened to name.
///
/// Keyed by the whole `section.key` rather than the section because `[policy]` is not of one
/// mind: the daemon re-reads that section on a call, all of it but the two keys below.
fn apply_for(key: &str) -> Option<Apply> {
    let (section, name) = key.split_once('.')?;
    Some(match section {
        "media" | "duck_detector" => Apply::Restart("mediad"),
        // `padd` stats the file once a second and re-reads both of its sections when the mtime
        // moves — `padd/src/main.rs`, where the reload is a line above `tap.imu_control()` and
        // says why it is on every tick. So there is nothing to offer, and offering a restart
        // anyway is not free: the pad session goes with it, and robotd's deadman zeroes the
        // velocity of whatever was walking. `robotctl pad bind` has always said the true thing —
        // "padd picks this up within a second".
        //
        // `pad_imu_head_control` is the *controller's* IMU steering the head. Not `head_imu` below.
        "pad" | "pad_imu_head_control" => Apply::Live("padd"),
        // `tofd` reads `[head_imu]` out of robotd's file — see `tof/src/config.rs` for why it
        // reads that file rather than one of its own — and reads it once, at startup.
        "head_imu" => Apply::Restart("tofd"),
        // `[policy]` is the one section a running daemon takes back: `PolicyChange::Reload`
        // re-reads it whole and rebuilds the controller from it, which is how `robotctl policy
        // add` lands a skill without a restart. Two keys are not in that promise:
        //
        // - `mode` is deliberately kept across a reload, because `robot.setMode` does not write
        //   config and adopting the file's mode would undo a live switch as a side effect.
        // - `enabled` is read once into `RobotState`, and the reload call is *refused* while it
        //   is false — so the one direction anybody cares about, off to on, cannot be a reload.
        "policy" if name != "mode" && name != "enabled" => Apply::Reload("robotd"),
        "bus" | "control" | "update_gate" | "policy" | "safety" | "chorale" | "theremin"
        | "audio" => Apply::Restart("robotd"),
        _ => return None,
    })
}

/// What a set of written keys needs, with each daemon named once.
///
/// Three lists rather than one, because the three answers read differently on the way out: a
/// restart is a question, a reload is a lighter question, and `live` is an answer — the operator
/// is told it already applies and asked nothing.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Plan {
    /// Units to restart, in start order.
    pub restart: Vec<&'static str>,
    /// Units to ask for a re-read.
    pub reload: Vec<&'static str>,
    /// Units that have it already, or will within a second.
    pub live: Vec<&'static str>,
}

impl Plan {
    /// Is there nothing to ask the operator?
    ///
    /// True for no edits at all *and* for edits that are already live — the difference is
    /// `live`, and the caller says the two things differently.
    pub fn is_quiet(&self) -> bool {
        self.restart.is_empty() && self.reload.is_empty()
    }
}

/// What a set of `section.key` names needs, in the order the exit flow acts on it.
fn plan_for_keys<'a>(keys: impl Iterator<Item = &'a str>) -> Plan {
    let mut plan = Plan::default();
    for key in keys {
        // Unreachable for a registry key: `every_registry_key_says_how_it_applies` keeps
        // `apply_for` exhaustive. Saying nothing beats naming the wrong daemon — the exit path
        // then prints the file it wrote instead of restarting something that never reads it.
        let Some(apply) = apply_for(key) else {
            continue;
        };
        let list = match apply {
            Apply::Restart(_) => &mut plan.restart,
            Apply::Reload(_) => &mut plan.reload,
            Apply::Live(_) => &mut plan.live,
        };
        if !list.contains(&apply.unit()) {
            list.push(apply.unit());
        }
    }
    // A daemon that is going down anyway reads the whole file coming back up, so the weaker
    // answers for it are absorbed rather than listed. `[policy] mode` with `[policy] gain` is
    // exactly that shape: mode needs the restart, gain would have been a reload, and offering
    // "restart robotd, then reload robotd" would disturb a walking robot a second time to apply
    // what it already applied.
    plan.reload.retain(|unit| !plan.restart.contains(unit));
    plan.live
        .retain(|unit| !plan.restart.contains(unit) && !plan.reload.contains(unit));
    // `robotd` first, because `mediad.service` is `After=robotd.service`: restarting in the
    // other order means mediad reconnects to a robotd that is about to go away.
    plan.restart.sort_unstable_by_key(|unit| *unit != "robotd");
    plan
}

/// What the pending edits need.
///
/// A quiet plan is a real answer — no edits, or edits nobody has to do anything about — and the
/// caller must not offer a restart for it. Read *before* a save, which clears the pending map.
pub fn plan_for(model: &Model) -> Plan {
    plan_for_keys(model.pending.keys().copied())
}

/// What the keys somebody just changed need.
///
/// The same mapping as [`plan_for`], from what a save recorded rather than from what is still
/// pending — which is what the exit flow has to work from, because the save already cleared the
/// other one.
pub fn plan_for_written(edited: &[String]) -> Plan {
    plan_for_keys(edited.iter().map(String::as_str))
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

/// Print what this robot's config changes, and nothing else.
///
/// **"What has been changed on this robot" is the first question support asks**, and until now
/// the only way to answer it was the editor — a full-screen TUI, over ssh, on a robot somebody is
/// already having trouble with. The comparison was there all along; it was just unreachable
/// without taking over the terminal.
///
/// Divergences only, because that is the question. The shipped file sets four keys and comments
/// out the rest, so a robot that has never been touched prints nothing at all — which is itself
/// the answer, and a shorter one than a hundred lines of defaults.
///
/// A key written out with its default value is *not* a divergence and does not appear. The
/// shipped file does exactly that in places, and reporting it as a change would bury the two
/// lines that matter under the ones that do not.
pub fn list(path: &Path, json: bool) -> Result<(), String> {
    let model = Model::load(path)?;
    let changed: Vec<Row> = model.rows().into_iter().filter(Row::differs).collect();

    if json {
        let entries: Vec<serde_json::Value> = changed
            .iter()
            .map(|row| {
                serde_json::json!({
                    "key": row.entry.key,
                    "value": row.effective(),
                    "default": row.default,
                    "doc": row.entry.doc,
                })
            })
            .collect();
        println!(
            "{}",
            serde_json::to_string(&entries).map_err(|e| e.to_string())?
        );
        return Ok(());
    }

    if changed.is_empty() {
        println!(
            "{} changes nothing — every value is the default",
            path.display()
        );
        return Ok(());
    }

    let width = changed
        .iter()
        .map(|row| row.entry.key.len())
        .max()
        .unwrap_or(0);
    for row in &changed {
        println!(
            "{:width$}  {}  (default {})",
            row.entry.key,
            row.effective(),
            row.default
        );
    }
    println!(
        "\n{} {} differ from the default. `sudo robotctl configure` edits them; `u` reverts one.",
        changed.len(),
        if changed.len() == 1 { "key" } else { "keys" }
    );
    Ok(())
}

// ── the terminal UI ──────────────────────────────────────────────────────────
//
// One screen: feature switches first, then every section; a footer carrying the selected
// key's one-line doc; SPACE toggles what can be toggled, ENTER types what cannot. Kept to the
// `monitor`'s conventions (ratatui, `ratatui::init`/`restore`) and deliberately dumber — a
// config editor should feel like a settings menu, not a dashboard.

use ratatui::crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers};
use ratatui::layout::{Constraint, Layout, Rect};
use ratatui::style::{Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Clear, Paragraph};

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
    /// Fuzzy-searching the list from a popup (ctrl+f). The cursor follows the best match as
    /// the query grows; leaving the popup — ENTER or ESC alike — keeps it wherever it landed.
    Search {
        query: String,
        /// Where the cursor was when the popup opened: an emptied query goes back there.
        origin: usize,
        /// Which of the ranked hits the cursor sits on (↑↓ walk them).
        hit: usize,
    },
    /// Deciding what to do with the pending edits on the way out.
    Confirm,
    /// Everything written; offering what the change actually needs — of the daemons that
    /// actually read what changed, which is not always `robotd` and is not always a restart.
    /// Never reached for a [`Plan::is_quiet`] plan: there is nothing to ask.
    Apply { plan: Plan },
}

/// Run the editor. Returns once the user has left, with everything saved or discarded.
///
/// `robot_socket` is only reached for a `[policy]` change, and only if the operator accepts the
/// reload — the editor is useful on a board where nothing is running.
pub fn run(path: &Path, robot_socket: &Path) -> Result<(), String> {
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
                                    buffer: shown_value(row).to_owned(),
                                    error: None,
                                };
                            }
                        }
                    }
                }
                KeyCode::Enter => {
                    if let Item::Key(index) = items[cursor] {
                        focus = Focus::Editing {
                            buffer: shown_value(&rows[index]).to_owned(),
                            error: None,
                        };
                    }
                }
                KeyCode::Char('f') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                    focus = Focus::Search {
                        query: String::new(),
                        origin: cursor,
                        hit: 0,
                    };
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
            Focus::Search { query, origin, hit } => {
                match key.code {
                    // Both leave the selection where the search put it: the point of the
                    // search was to get there.
                    KeyCode::Esc | KeyCode::Enter => {
                        focus = Focus::List;
                        continue;
                    }
                    KeyCode::Char('f') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                        focus = Focus::List;
                        continue;
                    }
                    KeyCode::Down | KeyCode::Tab => *hit += 1,
                    KeyCode::Up | KeyCode::BackTab => *hit = hit.saturating_sub(1),
                    KeyCode::Backspace => {
                        query.pop();
                        *hit = 0;
                    }
                    KeyCode::Char(c) if !key.modifiers.contains(KeyModifiers::CONTROL) => {
                        query.push(c);
                        *hit = 0;
                    }
                    _ => {}
                }
                if query.is_empty() {
                    cursor = *origin;
                } else {
                    let hits = search(&items, &rows, query);
                    if !hits.is_empty() {
                        *hit = (*hit).min(hits.len() - 1);
                        cursor = hits[*hit];
                    }
                }
            }
            Focus::Confirm => match key.code {
                KeyCode::Char('y') | KeyCode::Enter => {
                    // Read before the save, which clears `pending` — after it there is nothing
                    // left to say which daemons were affected.
                    let plan = plan_for(&model);
                    match model.save() {
                        Ok(()) => {
                            saved = true;
                            if plan.is_quiet() {
                                // `padd` has it, or will within a second. Asking would be
                                // asking somebody to authorise a pad restart for nothing.
                                break Ok(true);
                            }
                            focus = Focus::Apply { plan };
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
            Focus::Apply { .. } => match key.code {
                // The restart itself happens after `ratatui::restore`, outside the alternate
                // screen, so systemctl's output is visible.
                KeyCode::Char('y') | KeyCode::Enter => break Ok(true),
                // Back to `List` so the tail below finds no plan to act on — declining has to
                // leave the same trace as never having been asked.
                KeyCode::Char('n') | KeyCode::Esc | KeyCode::Char('q') => {
                    focus = Focus::List;
                    break Ok(saved);
                }
                _ => {}
            },
        }
    };
    // What the operator agreed to on the way out — nothing, unless they were asked and said yes.
    let agreed = match &focus {
        Focus::Apply { plan } => plan.clone(),
        _ => Plan::default(),
    };
    // What was *written*, not what is pending: the save already happened inside the loop above and
    // cleared the pending map, which is why reading it here restarted nothing at all.
    let edited = model.written().to_vec();
    ratatui::restore();

    let saved = outcome?;
    if !saved {
        return Ok(());
    }

    if !agreed.restart.is_empty() {
        let names = agreed.restart.join(" and ");
        println!("restarting {names}…");
        restart_units(&agreed.restart)?;
        println!("{names} restarted");
    }
    for unit in &agreed.reload {
        // `robotd` is the only unit that can be here — `[policy]` is the one section a running
        // daemon takes back, and `reload_policies` is that one call — so the name is read off
        // the plan for the message and asserted rather than dispatched on.
        debug_assert_eq!(*unit, "robotd", "robotd owns the only reloadable section");
        println!("asking {unit} to re-read the config…");
        match crate::reload_policies(robot_socket) {
            Ok(true) => println!("{unit} is running on the new values"),
            Ok(false) => println!(
                "{unit} declined — policies are off on this robot, which takes a restart:\n  \
                 sudo systemctl restart {unit}"
            ),
            Err(e) => {
                println!("{unit} did not answer ({e}) — it will read the file at its next start")
            }
        }
    }
    if agreed.is_quiet() {
        // Declined, or never asked. Say where the change is and what it is waiting on, since
        // the two answers are different and only one of them is a thing to do.
        let plan = plan_for_written(&edited);
        println!("written to {}", path.display());
        if !plan.restart.is_empty() {
            println!(
                "  applies on:  sudo systemctl restart {}",
                plan.restart.join(" ")
            );
        }
        if !plan.reload.is_empty() {
            // No command to name: the reload is a call this editor makes, and `robotctl policy`
            // makes it as a side effect of its own writes. What is worth saying is that nothing
            // is running on the new value yet.
            println!(
                "  {} has it at its next start — nothing is running on it yet",
                plan.reload.join(" and ")
            );
        }
        if !plan.live.is_empty() {
            let names = plan.live.join(" and ");
            let reads = if plan.live.len() == 1 {
                "picks"
            } else {
                "pick"
            };
            println!("  {names} {reads} this up within a second — nothing to restart");
        }
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

/// The value the list draws for a row — and so the one an edit should start from. An optional
/// key left unset shows what it *resolves* to, `0.9 (auto)`; opening the editor on the literal
/// word `unset` instead meant erasing it before every edit.
fn shown_value(row: &Row) -> &str {
    match &row.resolved {
        Some(resolved) if !row.differs() => resolved,
        _ => row.effective(),
    }
}

/// Everything the list shows for one key, joined so a query can hit any of it: section, name,
/// the value as drawn, the default, and the one-line doc.
fn searchable(row: &Row) -> String {
    let (section, name) = row.entry.key.split_once('.').expect("section.key");
    format!(
        "{section} {name} {} {} {}",
        shown_value(row),
        row.default,
        row.entry.doc
    )
}

/// Item indices of every key matching `query`, best first. Ties keep list order, so a query
/// that fits several keys equally walks them top to bottom.
fn search(items: &[Item], rows: &[Row], query: &str) -> Vec<usize> {
    let mut hits: Vec<(i32, usize)> = items
        .iter()
        .enumerate()
        .filter_map(|(at, item)| match item {
            Item::Key(index) => Some((row_score(&rows[*index], query)?, at)),
            Item::Header(_) => None,
        })
        .collect();
    hits.sort_by(|a, b| b.0.cmp(&a.0).then(a.1.cmp(&b.1)));
    hits.into_iter().map(|(_, at)| at).collect()
}

/// How well one row answers `query`, or `None`. Every whitespace-separated word must hit; a
/// word hits when it is *typed into* the key name (subsequence, `actsc` → `action_scale`), or
/// appears verbatim anywhere else the row shows — section, value, default, doc. Only the name
/// gets the fuzzy treatment: letters in order across a sentence of doc match nearly every row,
/// which is what made `tof` land on `theremin.enabled` with forty hits behind it.
fn row_score(row: &Row, query: &str) -> Option<i32> {
    let name = row.entry.key.split_once('.').expect("section.key").1;
    let text = searchable(row).to_lowercase();
    let mut total = 0;
    for word in query.split_whitespace() {
        let lower = word.to_lowercase();
        total += if let Some(at) = name.to_lowercase().find(&lower) {
            let word_start = at == 0 || !name.as_bytes()[at - 1].is_ascii_alphanumeric();
            300 + if word_start { 20 } else { 0 } - at as i32
        } else if let Some(score) = fuzzy_score(word, name) {
            100 + score
        } else {
            let at = text.find(&lower)?;
            50 - (at / 10) as i32
        };
    }
    Some(total)
}

/// Case-insensitive subsequence match, scored: `None` if the letters of `needle` do not occur
/// in order in `hay`; otherwise higher is better — runs of adjacent matches and matches at word
/// starts (`_`, space, `.`) score well, gaps cost a little. Only ever run over a key name, which
/// is short enough for a subsequence to mean something.
fn fuzzy_score(needle: &str, hay: &str) -> Option<i32> {
    let needle: Vec<char> = needle.chars().flat_map(char::to_lowercase).collect();
    let hay: Vec<char> = hay.chars().flat_map(char::to_lowercase).collect();
    if needle.is_empty() {
        return Some(0);
    }
    let mut score = 0i32;
    let mut at = 0usize;
    let mut previous: Option<usize> = None;
    for &c in &needle {
        let found = hay[at..].iter().position(|&h| h == c)? + at;
        let word_start = found == 0 || !hay[found - 1].is_alphanumeric();
        score += match previous {
            Some(p) if found == p + 1 => 10,
            _ if word_start => 8,
            Some(p) => 2 - ((found - p - 1).min(10) as i32),
            None => 2,
        };
        previous = Some(found);
        at = found + 1;
    }
    Some(score)
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
        // Which daemons, by name, and what they actually need: `[media]` is read by `mediad`,
        // and "restart it" over a change that needs the *other* daemon is how an edit reads as
        // having done nothing. A reload is offered where one exists, because it keeps the motors
        // powered — it can still ramp a walking robot home, which is why it says so.
        Focus::Apply { plan } => {
            let mut what: Vec<String> = Vec::new();
            if !plan.restart.is_empty() {
                what.push(format!("restart {}", plan.restart.join(" and ")));
            }
            if !plan.reload.is_empty() {
                what.push(format!("reload {}", plan.reload.join(" and ")));
            }
            let reason = if plan.restart.is_empty() {
                "re-reads [policy] without dropping the motors — a walking robot ramps home first"
            } else if plan.reload.is_empty() {
                "reads the config once at startup"
            } else {
                "reads some of this at startup and the rest on a call"
            };
            vec![
                Line::from(format!("written. {reason} —")),
                Line::from(format!("{} now? y do it · n later", what.join(", then "))),
            ]
        }
        Focus::Search { .. } => {
            let doc = match items.get(cursor) {
                Some(Item::Key(index)) => rows[*index].entry.doc,
                _ => "",
            };
            vec![
                Line::from(doc),
                Line::from("type to search · ↑↓ next/prev hit · ENTER/ESC done"),
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
                Line::from("↑↓ move · SPACE toggle · ENTER edit · u default · ^f search · q quit"),
            ]
        }
    };
    frame.render_widget(
        Paragraph::new(footer).block(Block::default().borders(Borders::ALL)),
        footer_area,
    );

    // The search popup: a small box floated over the list, the list still visible around it so
    // the selection can be watched moving as the query grows.
    if let Focus::Search { query, hit, .. } = focus {
        let hits = if query.is_empty() {
            0
        } else {
            search(items, rows, query).len()
        };
        let title = if query.is_empty() {
            " search ".to_owned()
        } else if hits == 0 {
            " search · no match ".to_owned()
        } else {
            format!(" search · {}/{hits} ", (*hit).min(hits - 1) + 1)
        };
        let width = 60.min(list_area.width.saturating_sub(4)).max(20);
        let popup = Rect {
            x: list_area.x + (list_area.width.saturating_sub(width)) / 2,
            y: list_area.y + 2,
            width,
            height: 3,
        };
        frame.render_widget(Clear, popup);
        frame.render_widget(
            Paragraph::new(Line::from(format!("{query}▏"))).block(
                Block::default()
                    .borders(Borders::ALL)
                    .title(title)
                    .border_style(if hits == 0 && !query.is_empty() {
                        Style::new().red()
                    } else {
                        Style::new().cyan()
                    }),
            ),
            popup,
        );
    }
}

#[cfg(test)]
mod tests {
    /// And a robot mid-experiment names every leftover. This is the set a flamingo trial leaves
    /// behind, which is what the command exists for: `cmd_alpha` at pass-through, a slot pointed
    /// at somebody's file, another switched off, and a fall gate widened.
    #[test]
    fn a_touched_config_names_every_key_that_differs() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("robotd.toml");
        std::fs::write(
            &path,
            "[control]\ncmd_alpha = 1.0\n\
             [policy]\nwalk = \"/home/pierre/mine.onnx\"\nstand = \"none\"\n\
             [safety]\nlimp_fall_tilt_z = -0.80\n",
        )
        .unwrap();

        let model = super::Model::load(&path).unwrap();
        let mut changed: Vec<&'static str> = model
            .rows()
            .iter()
            .filter(|row| row.differs())
            .map(|row| row.entry.key)
            .collect();
        changed.sort();
        assert_eq!(
            changed,
            [
                "control.cmd_alpha",
                "policy.stand",
                "policy.walk",
                "safety.limp_fall_tilt_z"
            ]
        );
    }

    use super::*;

    /// ctrl+f: letters typed into a name find it, verbatim text finds it anywhere on the row,
    /// and letters merely scattered through a doc sentence find nothing — `tof` used to land
    /// on `theremin.enabled` with forty hits behind it.
    #[test]
    fn the_search_ranks_the_key_you_meant_first_and_ignores_scattered_letters() {
        assert!(fuzzy_score("gain", "gain").unwrap() > fuzzy_score("gain", "gait_in_a").unwrap());
        assert!(fuzzy_score("gan", "gain").is_some());
        assert_eq!(fuzzy_score("gainz", "gain"), None);
        assert!(
            fuzzy_score("LOW", "head_lowpass").is_some(),
            "case-insensitive"
        );

        let m = model("");
        let rows = m.rows();
        let items = layout_items(&m);
        let name_of = |at: usize| match items[at] {
            Item::Key(index) => rows[index].entry.key,
            Item::Header(_) => unreachable!("headers are never hits"),
        };
        let hits = search(&items, &rows, "nominal_volt");
        assert_eq!(name_of(hits[0]), "policy.nominal_voltage");
        let hits = search(&items, &rows, "deadman");
        assert_eq!(name_of(hits[0]), "safety.deadman_ms");
        // Typed into the name, not spelled out.
        let hits = search(&items, &rows, "actsc");
        assert_eq!(name_of(hits[0]), "policy.action_scale");
        // Section names are part of the text shown, so they match too.
        let hits = search(&items, &rows, "safety");
        assert!(name_of(hits[0]).starts_with("safety."));
        assert!(search(&items, &rows, "zzzzqqq").is_empty());

        // Every `tof` hit has the three letters together somewhere on the row, or typed into
        // its name — never spread across the doc.
        let hits = search(&items, &rows, "tof");
        assert!(!hits.is_empty());
        for at in hits {
            let Item::Key(index) = items[at] else {
                unreachable!()
            };
            let row = &rows[index];
            let name = row.entry.key.split_once('.').unwrap().1;
            assert!(
                searchable(row).to_lowercase().contains("tof")
                    || fuzzy_score("tof", name).is_some(),
                "{} matched tof without containing it",
                row.entry.key
            );
        }
        // The doc of `theremin.enabled` says "ToF theremin" — a fair hit, but a doc hit, so
        // the rows *about* the sensor come first.
        let hits = search(&items, &rows, "tof");
        assert_ne!(name_of(hits[0]), "theremin.enabled");
        assert!(hits.len() < 15, "{} rows hit tof", hits.len());
        // Several words all have to hit.
        assert!(
            search(&items, &rows, "safety limp")
                .iter()
                .all(|&at| name_of(at).contains("limp"))
        );
    }

    /// An `(auto)` row opens the editor on the value it resolves to, not on the word `unset`.
    #[test]
    fn editing_an_auto_key_starts_from_its_resolved_value() {
        let m = model("");
        let rows = m.rows();
        let bitrate = rows
            .iter()
            .find(|r| r.entry.key == "media.bitrate")
            .unwrap();
        assert_eq!(bitrate.effective(), "unset");
        assert_eq!(shown_value(bitrate), bitrate.resolved.as_deref().unwrap());
        let set = rows
            .iter()
            .find(|r| r.entry.key == "safety.deadman_ms")
            .unwrap();
        assert_eq!(shown_value(set), set.effective());
    }

    fn model(text: &str) -> Model {
        Model::from_text(Path::new("/test/robotd.toml"), text).expect("parses")
    }

    fn entry(key: &str) -> &'static robotd_params::registry::Entry {
        robotd_params::registry::entry_for(key).expect("a registry key")
    }

    /// The restart offer names the daemon that reads what changed. `[media]` is read by
    /// `mediad`, and offering a `robotd` restart for it is an edit that reads as having done
    /// nothing at all until somebody reboots.
    #[test]
    fn the_restart_offer_names_the_daemon_that_reads_the_change() {
        let mut m = model("");
        m.edit(entry("media.quality"), "360p30").expect("valid");
        assert_eq!(plan_for(&m).restart, vec!["mediad"]);

        let mut m = model("");
        m.edit(entry("control.hz"), "60").expect("valid");
        assert_eq!(plan_for(&m).restart, vec!["robotd"]);

        // Both, and robotd first: mediad.service is After=robotd.service, so the other order
        // reconnects mediad to a robotd that is about to go away.
        let mut m = model("");
        m.edit(entry("media.source"), "test").expect("valid");
        m.edit(entry("audio.enabled"), "false").expect("valid");
        assert_eq!(plan_for(&m).restart, vec!["robotd", "mediad"]);

        // Nothing pending is nothing to ask about, and the caller must not offer anything.
        assert!(plan_for(&model("")).is_quiet());
        assert!(plan_for(&model("")).live.is_empty());
    }

    /// `[head_imu]` is `tofd`'s, and it read as `robotd`'s on the board.
    ///
    /// Turning the head IMU on through this editor restarted `robotd` — which never looks at the
    /// key — and left `tofd` running on the value it loaded at boot. The switch was on in the
    /// file, off in the daemon, and the only sign was a startup line nobody re-reads. Regression
    /// test rather than an assertion folded into the case above, because the two IMU sections are
    /// a name apart (and now less so): `pad_imu_head_control` is the controller's, and `padd` re-reads it by itself.
    #[test]
    fn the_head_imu_switch_restarts_tofd_and_not_robotd() {
        let mut m = model("");
        m.edit(entry("head_imu.enabled"), "true").expect("valid");
        let plan = plan_for(&m);
        assert_eq!(plan.restart, vec!["tofd"]);
        assert!(plan.live.is_empty());

        let mut m = model("");
        m.edit(entry("pad_imu_head_control.enabled"), "true")
            .expect("valid");
        let plan = plan_for(&m);
        assert_eq!(
            plan.live,
            vec!["padd"],
            "the controller's IMU is padd's, and padd re-reads it"
        );
        assert!(plan.restart.is_empty(), "and it needs no restart");
    }

    /// `[pad]` and `[pad_imu_head_control]` are live: padd re-reads them, so there is nothing to offer.
    ///
    /// The inverse of the `[head_imu]` bug and the same mistake — a mapping that does not
    /// describe the daemon. `padd` stats the file every second and re-reads both sections when
    /// the mtime moves, which is why `robotctl pad bind` prints "padd picks this up within a
    /// second" rather than offering anything. Restarting it to apply what applies by itself
    /// takes the pad session down, and robotd's deadman then zeroes the velocity of whatever was
    /// walking — a real cost, paid for nothing.
    #[test]
    fn the_pad_sections_need_no_restart_at_all() {
        for key in [
            "pad.a",
            "pad.dpad_down",
            "pad_imu_head_control.enabled",
            "pad_imu_head_control.gain",
        ] {
            let mut m = model("");
            let value = match key {
                "pad_imu_head_control.enabled" => "true",
                "pad_imu_head_control.gain" => "0.5",
                _ => "walk",
            };
            m.edit(entry(key), value).expect("valid");
            let plan = plan_for(&m);
            assert!(plan.is_quiet(), "{key} must not ask for anything: {plan:?}");
            assert_eq!(plan.live, vec!["padd"], "{key}");
        }

        // Paired with a restart it stays quiet about padd and loud about the other one: the
        // offer is for the daemon that needs it, and padd is neither restarted nor mentioned in
        // it.
        let mut m = model("");
        m.edit(entry("pad.a"), "walk").expect("valid");
        m.edit(entry("safety.deadman_ms"), "800").expect("valid");
        let plan = plan_for(&m);
        assert_eq!(plan.restart, vec!["robotd"]);
        assert_eq!(plan.live, vec!["padd"]);
    }

    /// `[policy]` reloads instead of restarting — except the two keys a reload does not carry.
    ///
    /// `PolicyChange::Reload` re-reads that section whole and rebuilds the controller from it,
    /// which is how `robotctl policy add` lands a skill on a running robot. Restarting robotd
    /// for `action_scale` takes motor control away from a standing robot to change a number it
    /// would have taken standing up. `mode` is deliberately kept across a reload and `enabled`
    /// is read once at startup — with the reload call *refused* while it is false, so off to on
    /// cannot be one.
    #[test]
    fn the_policy_section_reloads_but_its_two_startup_keys_do_not() {
        let mut m = model("");
        m.edit(entry("policy.action_scale"), "0.4").expect("valid");
        let plan = plan_for(&m);
        assert_eq!(plan.reload, vec!["robotd"]);
        assert!(plan.restart.is_empty(), "the motors stay powered");

        let mut m = model("");
        m.edit(entry("policy.enabled"), "true").expect("valid");
        assert_eq!(
            plan_for(&m).restart,
            vec!["robotd"],
            "a reload is refused while policies are off"
        );

        let mut m = model("");
        m.edit(entry("policy.mode"), "roller").expect("valid");
        assert_eq!(
            plan_for(&m).restart,
            vec!["robotd"],
            "a reload keeps the running mode, so the file's would be ignored"
        );

        // One section, both answers, and the restart absorbs the reload. "restart robotd, then
        // reload robotd" would ramp a walking robot home a second time to apply what coming back
        // up already applied.
        let mut m = model("");
        m.edit(entry("policy.mode"), "roller").expect("valid");
        m.edit(entry("policy.gain"), "300").expect("valid");
        let plan = plan_for(&m);
        assert_eq!(plan.restart, vec!["robotd"]);
        assert!(plan.reload.is_empty(), "the restart is the reload");
    }

    /// Every key in the registry says how it applies.
    ///
    /// What went wrong with `[head_imu]` was not a wrong answer, it was a default one: a
    /// catch-all arm answered `robotd` for a section nobody had mapped, so adding a section was
    /// enough to ship a restart offer that restarts the wrong daemon. There is no catch-all now,
    /// and this fails for the next key added without an arm — at `cargo test`, not on a board,
    /// and not as a switch that silently does nothing.
    #[test]
    fn every_registry_key_says_how_it_applies() {
        let unmapped: Vec<&str> = robotd_params::registry::REGISTRY
            .iter()
            .map(|e| e.key)
            .filter(|key| apply_for(key).is_none())
            .collect();
        assert!(
            unmapped.is_empty(),
            "nothing says how {unmapped:?} applies — add an arm to `apply_for`"
        );
    }

    /// The offer says which daemon and which of the three answers, in words.
    ///
    /// The screen is the whole user interface to this mapping: an operator who reads "restart
    /// robotd" over a `[media]` change learns the wrong thing about their robot, and one who is
    /// asked to restart `padd` for a binding pays for nothing. Rendered rather than asserted on
    /// the `Plan`, because what went wrong on the board was what the screen *said*.
    #[test]
    fn the_offer_says_which_daemon_and_what_it_needs() {
        let screen_for = |focus: &Focus| {
            let m = model("");
            let rows = m.rows();
            let items = layout_items(&m);
            let mut terminal = ratatui::Terminal::new(ratatui::backend::TestBackend::new(100, 24))
                .expect("terminal");
            terminal
                .draw(|frame| draw(frame, &m, &rows, &items, 1, focus, None))
                .expect("draws");
            format!("{:?}", terminal.backend().buffer())
        };

        let restart = screen_for(&Focus::Apply {
            plan: plan_for_keys(["head_imu.enabled"].into_iter()),
        });
        assert!(restart.contains("restart tofd"), "{restart}");
        assert!(restart.contains("once at startup"), "{restart}");

        // A reload names itself as one, and says the thing an operator standing over the robot
        // wants to know: the motors stay powered, and a walking robot ramps home first.
        let reload = screen_for(&Focus::Apply {
            plan: plan_for_keys(["policy.gain"].into_iter()),
        });
        assert!(reload.contains("reload robotd"), "{reload}");
        assert!(!reload.contains("restart"), "{reload}");
        assert!(reload.contains("ramps home"), "{reload}");

        // Both, in the order they are done.
        let both = screen_for(&Focus::Apply {
            plan: plan_for_keys(["media.quality", "policy.gain"].into_iter()),
        });
        assert!(
            both.contains("restart mediad, then reload robotd"),
            "{both}"
        );
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

    /// A `[duck_detector]` change restarts `mediad`, not `robotd`.
    ///
    /// `robotd` owned every key in this file for long enough that the restart was hardcoded, and
    /// `[duck_detector]` is read by `mediad` because the camera frames are on its tee. Restarting the
    /// A save records what it wrote, because that is what decides the restart.
    ///
    /// The bug this pins: `save` clears `pending`, and the restart decision is made after the
    /// editor closes — so reading `pending` there found an empty map, the plan came back empty,
    /// and turning the detector off looked like it had no effect at all. Twice, on a robot,
    /// before anybody suspected the editor rather than the daemon.
    #[test]
    fn a_save_remembers_what_it_wrote_so_the_right_daemon_restarts() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("robotd.toml");
        std::fs::write(&path, "").unwrap();
        let mut m = Model::load(&path).expect("loads");

        assert!(m.written().is_empty(), "nothing written yet");
        m.edit(entry("duck_detector.enabled"), "true")
            .expect("edits");
        assert!(!m.pending.is_empty());
        m.save().expect("saves");

        assert!(m.pending.is_empty(), "a save clears what is pending");
        assert_eq!(m.written(), ["duck_detector.enabled".to_owned()]);
        assert_eq!(plan_for_written(m.written()).restart, vec!["mediad"]);

        // A second save adds to the record rather than replacing it: somebody who changes the
        // detector and then the gait wants both daemons restarted.
        m.edit(entry("policy.mode"), "roller").expect("edits");
        m.save().expect("saves");
        assert_eq!(
            plan_for_written(m.written()).restart,
            vec!["robotd", "mediad"]
        );
    }

    /// wrong daemon is how somebody edits a value three times and swears it does nothing.
    #[test]
    fn the_section_decides_which_daemon_restarts() {
        let detect = vec!["duck_detector.enabled".to_owned()];
        assert_eq!(plan_for_written(&detect).restart, vec!["mediad"]);

        let policy = vec!["policy.mode".to_owned()];
        assert_eq!(plan_for_written(&policy).restart, vec!["robotd"]);

        // Both, in the order they are least disruptive to restart: the control loop first, then the
        // camera — a robot that is standing up should not be waiting on a WebRTC teardown.
        let both = vec!["duck_detector.hz".to_owned(), "audio.enabled".to_owned()];
        assert_eq!(plan_for_written(&both).restart, vec!["robotd", "mediad"]);

        // A button change restarts nothing: `padd` reads it back off the file by itself, and
        // offering `robotd` here would drop motor control — a standing robot on the floor — to
        // apply a setting it never sees.
        let pad = vec!["pad.x".to_owned()];
        let plan = plan_for_written(&pad);
        assert!(plan.is_quiet());
        assert_eq!(plan.live, vec!["padd"]);

        assert_eq!(plan_for_written(&[]), Plan::default());
    }
}
