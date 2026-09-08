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
//! ## Restart
//!
//! The daemons read the file **once at startup** (`robotd-params` docs) — so every change
//! requires a restart, and the exit flow offers one whenever anything was written. *Which*
//! daemon is derived from the keys that changed, not assumed: `[media]` is `mediad` reading the
//! same file, and a "restart robotd" offer over a video setting is an edit that reads as having
//! done nothing at all. [`unit_for`] is that mapping and [`units_for`] applies it.
//!
//! The file is root-owned; run as `sudo robotctl configure` to actually write.

use std::path::Path;

// The editing model itself lives in `robotd-params`, beside the schema it validates against —
// see that module's header for why. Re-exported rather than imported privately because the rest
// of `robotctl` reaches for `configure::Model` and should not have to know it moved.
pub use robotd_params::edit::{Edit, Model, Row, bind_pad, pad_bindings, render, sections};

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
        // `padd` reads the bindings, and `robotd` never sees them. Offering a robotd restart for
        // a button change would drop motor control — putting a standing robot on the floor — to
        // apply a setting it does not read.
        "pad" => "padd",
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

    /// A `[detect]` change restarts `mediad`, not `robotd`.
    ///
    /// `robotd` owned every key in this file for long enough that the restart was hardcoded, and
    /// `[detect]` is read by `mediad` because the camera frames are on its tee. Restarting the
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

        // A button change restarts the daemon that reads it. Offering `robotd` here would drop
        // motor control — a standing robot on the floor — to apply a setting it never sees.
        let pad = vec!["pad.x".to_owned()];
        assert_eq!(units_to_restart(&pad), vec!["padd"]);

        assert!(units_to_restart(&[]).is_empty());
    }
}
