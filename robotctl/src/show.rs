//! `robotctl update show` — one update run, in full.
//!
//! `update log` says an update happened and how it ended. This says what it *did*: every phase
//! with the time it took, the manifest that was verified, the hook output that was collected, the
//! units that were restarted, the gate's verdict — and then the journal for the same window, so
//! the account includes the daemons the update restarted and not only `updaterd`'s side of it.
//!
//! **Times are UTC, and so is the spliced journal.** Two clocks on one screen is the way to make
//! a timeline unreadable, and `journalctl --utc` makes the halves agree for free. The alternative
//! — local time — would put a timezone database in the one crate whose dependency tree is kept
//! small on purpose, for the recovery path (see this crate's `Cargo.toml`).

use std::fmt::Write as _;

use duck_ipc_proto::{Outcome, Phase, RunEvent, RunRecord, RunTranscript};

/// How far past the run's last event to keep reading the journal.
///
/// The deferred restarts fire five seconds after the reply, and the daemons they restart log
/// their startup identity a moment after that — which is the single most useful thing in the
/// window and lands *after* the transcript has ended. A minute covers it without dragging in the
/// next unrelated thing the robot did.
const JOURNAL_TAIL: i64 = 60;

/// `updaterd` is always in the splice: it is the process that performed the run, and on the paths
/// with no unit events (a refusal, a dry run) it is the only thing that logged anything.
const ALWAYS_SPLICED: &str = "updaterd";

/// Journal lines the transcript above has already shown, verbatim.
///
/// `hooks::run` writes the hook's output to the journal a line at a time — deliberately, so it is
/// greppable — and the transcript records the same output as one event. Splicing both puts the
/// pre-install hook's ninety lines on screen twice in a row, which on the first real board run
/// was more than a third of the output.
///
/// Matched on the `tracing` target as journald renders it. If that rendering ever changes the
/// filter stops matching and the duplicates come back, which is the right way for this to fail:
/// the log is intact and merely repeated, rather than silently dropped.
const ALREADY_IN_THE_TRANSCRIPT: &str = "updater::hooks:";

/// Whether a journal line is worth showing under a transcript that already contains it.
pub fn worth_splicing(line: &str) -> bool {
    !line.contains(ALREADY_IN_THE_TRANSCRIPT) && !line.trim_start().starts_with("-- No entries --")
}

/// Render one run.
pub fn render(transcript: &RunTranscript) -> String {
    let mut out = String::new();
    let events = &transcript.events;

    let _ = writeln!(
        out,
        "run {} · {} · {}",
        transcript.run,
        transcript.component,
        events
            .first()
            .map(|first| format!("{} UTC", full_stamp(first.at)))
            .unwrap_or_else(|| "no events recorded".into())
    );

    // The verdict first, because it is what the reader came for. The timeline below explains it;
    // it should not be something they have to reach the bottom of a screenful to find.
    for record in events {
        if let RunEvent::Ended { summary, .. } = &record.event {
            let _ = writeln!(out, "  {summary}");
        }
    }
    for record in events {
        if let RunEvent::Began {
            target,
            source,
            requested_by,
            installed,
            ..
        } = &record.event
        {
            let _ = writeln!(
                out,
                "  asked for {target}, from {source}{}",
                installed
                    .as_ref()
                    .map(|v| format!(", onto {v}"))
                    .unwrap_or_default()
            );
            if let Some(who) = requested_by {
                let _ = writeln!(out, "  requested by {who}");
            }
        }
    }

    // A run with no ending is not a run that failed silently — it is one whose verdict is
    // somewhere else, and saying which somewhere is the difference between a dead end and a next
    // step. See `RunEvent::Ended`.
    if !events.is_empty()
        && !events
            .iter()
            .any(|r| matches!(r.event, RunEvent::Ended { .. }))
    {
        out.push_str(
            "  this run has no ending: it was cut short, most likely by updaterd restarting \
             itself\n  or by the power going away. A later run carries the verdict — check the \
             newest.\n",
        );
    }

    out.push('\n');
    let mut previous: Option<i64> = None;
    for record in events {
        // `previous` advances even for an event that renders no row — `began` is one — because it
        // is the clock, not the cursor. Advancing it only on rendered rows measured the first gap
        // from whatever came before the skipped event, which is a wrong number rather than a
        // missing one.
        if let Some(line) = event_line(record, previous) {
            out.push_str(&line);
        }
        previous = Some(record.at);
    }

    // Only where there is somewhere else to go. On a board with one run this would be a line
    // telling the reader they can look at the thing they are looking at.
    if let (Some(newest), Some(oldest)) =
        (transcript.available.first(), transcript.available.last())
        && newest != oldest
    {
        let _ = write!(
            out,
            "\n  runs {oldest} to {newest} are kept · `robotctl update log` says what each did\n"
        );
    }

    out
}

/// One event as one row, plus indented body for the multi-line ones.
fn event_line(record: &RunRecord, previous: Option<i64>) -> Option<String> {
    let mut out = String::new();
    let gap = match previous {
        // Only where it says something. A column of `+0s` is noise, and the number anyone wants
        // from a transcript is "what took the two minutes", which only the gaps answer.
        Some(previous) if record.at - previous >= 1 => elapsed(record.at - previous),
        _ => String::new(),
    };

    let (kind, detail) = match &record.event {
        // Already the header, in full and in prose. A row here would say it again, worse, and
        // three lines below itself.
        RunEvent::Began { .. } => return None,
        RunEvent::Phase { phase, detail } => (
            phase_name(*phase).to_owned(),
            detail.clone().unwrap_or_default(),
        ),
        RunEvent::Manifest {
            version,
            sha256,
            bytes,
            url,
            signed_by,
            source_revision,
        } => {
            let mut parts = vec![version.to_string()];
            if let Some(bytes) = bytes {
                parts.push(bytes_human(*bytes));
            }
            parts.push(format!("sha256 {}", short(sha256, 8)));
            if let Some(key) = signed_by {
                parts.push(format!("signed by {key}"));
            }
            if let Some(rev) = source_revision {
                parts.push(format!("rev {}", short(rev, 7)));
            }
            let mut line = row(record.at, &gap, "manifest", &parts.join(" · "));
            // Below rather than in the row: an artifact URL is long enough to push everything
            // else off the side of a terminal, and it is the one fact on this line that is read
            // only when something is wrong with where the bytes came from.
            if let Some(url) = url {
                let _ = writeln!(line, "{:>33}│ {url}", "");
            }
            return Some(line);
        }
        RunEvent::Hook {
            hook,
            exit_code,
            output,
        } => {
            let head = format!(
                "{hook}{}",
                match exit_code {
                    Some(0) | None => String::new(),
                    Some(code) => format!(", exit {code}"),
                }
            );
            let mut line = row(record.at, &gap, "hook", &head);
            for text in output.lines() {
                let _ = writeln!(line, "{:>33}│ {text}", "");
            }
            return Some(line);
        }
        RunEvent::Unit {
            unit,
            action,
            detail,
        } => (
            "unit".to_owned(),
            match detail {
                Some(why) => format!("{unit}: {action} — {why}"),
                None => format!("{unit}: {action}"),
            },
        ),
        RunEvent::Health { passed, detail } => (
            "health".to_owned(),
            match (passed, detail) {
                (true, None) => "the robot reported healthy".to_owned(),
                // Passing and being healthy are not the same fact, and rounding one to the other
                // is how a transcript ends up saying "healthy" about a board whose own journal
                // says "degraded" at the same second. See `engine::GatePassed`.
                (true, Some(why)) => format!("passed — {why}"),
                (false, Some(why)) => format!("FAILED — {why}"),
                (false, None) => "FAILED".to_owned(),
            },
        ),
        RunEvent::Note { text } => ("note".to_owned(), text.clone()),
        RunEvent::Ended { summary, .. } => ("ended".to_owned(), summary.clone()),
        RunEvent::Truncated { dropped } => (
            "truncated".to_owned(),
            format!("{dropped} further events were not recorded"),
        ),
        // Rendered rather than skipped: a reader must be able to see that something happened here
        // that this build cannot name, instead of a gap in the timeline they cannot account for.
        RunEvent::Unrecognised => (
            "?".to_owned(),
            "an event from a newer release, which this robotctl cannot read".to_owned(),
        ),
    };

    out.push_str(&row(record.at, &gap, &kind, &detail));
    Some(out)
}

fn row(at: i64, gap: &str, kind: &str, detail: &str) -> String {
    format!("  {}  {gap:>7}  {kind:<12} {detail}\n", stamp(at))
        .trim_end()
        .to_owned()
        + "\n"
}

/// Phase names as a person says them, not as the enum spells them.
fn phase_name(phase: Phase) -> &'static str {
    match phase {
        Phase::Idle => "idle",
        Phase::Preflight => "preflight",
        Phase::Checking => "checking",
        Phase::Downloading => "downloading",
        Phase::Verifying => "verifying",
        Phase::Extracting => "extracting",
        Phase::RunningPreHook => "pre-hook",
        Phase::Swapping => "swapping",
        Phase::RunningPostHook => "post-hook",
        Phase::Applying => "applying",
        Phase::HealthGate => "health gate",
        Phase::Committing => "committing",
        Phase::RollingBack => "ROLLING BACK",
    }
}

/// The window the journal should be read over, as `(since, until)` unix seconds.
///
/// `None` for a run with no events, where there is no window to read.
pub fn window(transcript: &RunTranscript) -> Option<(i64, i64)> {
    let first = transcript.events.first()?.at;
    let last = transcript.events.last()?.at;
    Some((first, last + JOURNAL_TAIL))
}

/// The units to read the journal for: `updaterd`, plus everything the run touched.
///
/// Taken from the run itself rather than from a hardcoded list, so the splice describes the
/// update that happened instead of the update this build expected — a release that adds a daemon
/// is covered the day it ships, with nothing here to update.
pub fn units(transcript: &RunTranscript) -> Vec<String> {
    let mut units = vec![ALWAYS_SPLICED.to_owned()];
    for record in &transcript.events {
        if let RunEvent::Unit { unit, .. } = &record.event
            && !units.contains(unit)
        {
            units.push(unit.clone());
        }
    }
    units
}

/// The `journalctl` invocation for a run's window — also what gets printed when it cannot be run.
pub fn journal_command(since: i64, until: i64, units: &[String]) -> Vec<String> {
    let mut argv = vec![
        "journalctl".to_owned(),
        "--utc".to_owned(),
        "--no-pager".to_owned(),
        format!("--since=@{since}"),
        format!("--until=@{until}"),
    ];
    for unit in units {
        argv.push(format!("--unit={unit}"));
    }
    argv
}

/// `HH:MM:SS`, UTC.
fn stamp(at: i64) -> String {
    let (_, _, _, h, m, s) = civil(at);
    format!("{h:02}:{m:02}:{s:02}")
}

/// `YYYY-MM-DD HH:MM:SS`, UTC — the header's one absolute reading.
pub fn full_stamp(at: i64) -> String {
    let (year, month, day, h, m, s) = civil(at);
    format!("{year:04}-{month:02}-{day:02} {h:02}:{m:02}:{s:02}")
}

/// Civil UTC date and time from a unix timestamp.
///
/// Hand-written, and deliberately: what must never be hand-rolled is a *timezone* database, and
/// this does not touch one — it is the fixed proleptic-Gregorian arithmetic (Howard Hinnant's
/// `civil_from_days`), which is why the whole rendering is in UTC. Adding a date crate to
/// `robotctl` for it would put a tz database on the recovery path to save fifteen lines of pure
/// arithmetic with tests under it.
pub(crate) fn civil(at: i64) -> (i64, u32, u32, u32, u32, u32) {
    let days = at.div_euclid(86_400);
    let secs = at.rem_euclid(86_400);

    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1_460 + doe / 36_524 - doe / 146_096) / 365;
    let year = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let month = if mp < 10 { mp + 3 } else { mp - 9 } as u32;

    (
        year + i64::from(month <= 2),
        month,
        day,
        (secs / 3_600) as u32,
        ((secs % 3_600) / 60) as u32,
        (secs % 60) as u32,
    )
}

/// A gap between two events, at the resolution a person cares about.
fn elapsed(seconds: i64) -> String {
    match seconds {
        s if s < 60 => format!("+{s}s"),
        s if s < 3_600 => format!("+{}m{:02}s", s / 60, s % 60),
        s => format!("+{}h{:02}m", s / 3_600, (s % 3_600) / 60),
    }
}

fn bytes_human(bytes: u64) -> String {
    const UNITS: [&str; 4] = ["B", "kB", "MB", "GB"];
    let mut value = bytes as f64;
    let mut unit = 0;
    while value >= 1000.0 && unit + 1 < UNITS.len() {
        value /= 1000.0;
        unit += 1;
    }
    if unit == 0 {
        format!("{bytes} B")
    } else {
        format!("{value:.1} {}", UNITS[unit])
    }
}

/// First `n` characters, with an ellipsis where anything was dropped.
fn short(text: &str, n: usize) -> String {
    match text.char_indices().nth(n) {
        Some((cut, _)) => format!("{}…", &text[..cut]),
        None => text.to_owned(),
    }
}

/// One line per attempt for `update log`, with the run number that opens it.
///
/// The index half of `git log` / `git show`: this is where someone finds the run they then ask
/// `update show` about, so the number has to be on it.
pub fn log_line(entry: &duck_ipc_proto::LogEntry) -> String {
    let run = match entry.run {
        Some(run) => format!("{run:>4}"),
        // Entries from before transcripts existed, and the rare attempt that failed before one
        // could be opened. Aligned with the rest rather than omitted, so the column stays a column.
        None => "   -".to_owned(),
    };
    let versions = match (&entry.from, &entry.to) {
        (Some(from), Some(to)) => format!("{from} → {to}"),
        (None, Some(to)) => to.to_string(),
        (Some(from), None) => format!("from {from}"),
        (None, None) => "unknown version".to_owned(),
    };
    let outcome = match &entry.outcome {
        Outcome::Success => "applied".to_owned(),
        Outcome::RolledBack { reason } => format!("ROLLED BACK — {reason}"),
        Outcome::Aborted { reason } => format!("refused — {reason}"),
    };
    format!(
        "{run}  {}  {:<10} {versions}: {outcome}",
        full_stamp(entry.at),
        entry.component
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use duck_ipc_proto::ComponentId;

    fn at(at: i64, event: RunEvent) -> RunRecord {
        RunRecord { at, event }
    }

    /// Pinned against dates checked by hand, including a leap day and a century boundary.
    #[test]
    fn civil_dates_match_known_timestamps() {
        assert_eq!(full_stamp(0), "1970-01-01 00:00:00");
        assert_eq!(full_stamp(1_700_000_000), "2023-11-14 22:13:20");
        // 2024-02-29, a leap day in a leap century.
        assert_eq!(full_stamp(1_709_208_000), "2024-02-29 12:00:00");
        // 2000-03-01, the day after the leap day the 100-year rule would have skipped.
        assert_eq!(full_stamp(951_868_800), "2000-03-01 00:00:00");
        assert_eq!(full_stamp(1_767_225_599), "2025-12-31 23:59:59");
    }

    #[test]
    fn gaps_read_at_the_resolution_a_person_wants() {
        assert_eq!(elapsed(3), "+3s");
        assert_eq!(elapsed(77), "+1m17s");
        assert_eq!(elapsed(7_400), "+2h03m");
    }

    /// The splice covers the daemons the run restarted, discovered from the run itself.
    #[test]
    fn the_journal_window_follows_the_units_the_run_touched() {
        let transcript = RunTranscript {
            run: 7,
            component: ComponentId::new("daemon"),
            events: vec![
                at(
                    100,
                    RunEvent::Phase {
                        phase: Phase::Preflight,
                        detail: None,
                    },
                ),
                at(
                    110,
                    RunEvent::Unit {
                        unit: "robotd".into(),
                        action: "restart".into(),
                        detail: None,
                    },
                ),
                at(
                    111,
                    RunEvent::Unit {
                        unit: "robotd".into(),
                        action: "restart".into(),
                        detail: None,
                    },
                ),
                at(
                    120,
                    RunEvent::Unit {
                        unit: "mediad".into(),
                        action: "restart".into(),
                        detail: None,
                    },
                ),
            ],
            available: vec![7],
        };

        assert_eq!(units(&transcript), vec!["updaterd", "robotd", "mediad"]);
        assert_eq!(window(&transcript), Some((100, 120 + JOURNAL_TAIL)));

        let argv = journal_command(100, 180, &units(&transcript));
        assert!(argv.contains(&"--since=@100".to_owned()), "{argv:?}");
        assert!(argv.contains(&"--unit=mediad".to_owned()), "{argv:?}");
        assert!(argv.contains(&"--utc".to_owned()), "{argv:?}");
    }

    /// The hook's own journal lines are already above, as one event with its output intact.
    ///
    /// On the first board run this was a third of the screen, printed twice in a row.
    #[test]
    fn the_splice_drops_what_the_transcript_already_showed() {
        let hook_line = "Aug 27 14:00:43 duck updaterd[2327]: 2026-08-27T14:00:43.876034Z  INFO \
                         updater::hooks: preinstall: checking the GStreamer stack \
                         hook=hooks/preinstall";
        assert!(!worth_splicing(hook_line));

        // Everything else `updaterd` says stays: the engine's own lines are not in the transcript.
        let engine_line = "Aug 27 14:00:55 duck updaterd[2327]: 2026-08-27T14:00:55.382481Z  WARN \
                           updater::engine: committing: the robot is degraded";
        assert!(worth_splicing(engine_line));
        // And so does every other daemon, which is the half the transcript never sees.
        assert!(worth_splicing(
            "Aug 27 14:00:54 duck systemd[1]: Started robotd.service - Robot control daemon."
        ));
        assert!(!worth_splicing("-- No entries --"));
    }

    /// Passing the gate and being healthy are different facts.
    #[test]
    fn a_degraded_commit_does_not_render_as_healthy() {
        let degraded = RunTranscript {
            run: 9,
            component: ComponentId::new("daemon"),
            events: vec![at(
                100,
                RunEvent::Health {
                    passed: true,
                    detail: Some(
                        "degraded, and committed anyway — this release cannot have caused it: no \
                         robot on the motor bus"
                            .into(),
                    ),
                },
            )],
            available: vec![9],
        };
        let out = render(&degraded);
        assert!(out.contains("no robot on the motor bus"), "{out}");
        assert!(!out.contains("reported healthy"), "{out}");
    }

    /// A run cut short must say so where the verdict would have been, or it reads as a robot that
    /// stopped mid-update and never came back.
    #[test]
    fn a_run_with_no_ending_says_where_the_verdict_went() {
        let transcript = RunTranscript {
            run: 3,
            component: ComponentId::new("daemon"),
            events: vec![at(
                100,
                RunEvent::Phase {
                    phase: Phase::Swapping,
                    detail: None,
                },
            )],
            available: vec![3],
        };
        let out = render(&transcript);
        assert!(out.contains("no ending"), "{out}");
        assert!(out.contains("A later run carries the verdict"), "{out}");
    }

    /// And a run that ended does *not* say that.
    #[test]
    fn a_finished_run_leads_with_its_verdict() {
        let transcript = RunTranscript {
            run: 4,
            component: ComponentId::new("daemon"),
            events: vec![
                at(
                    100,
                    RunEvent::Began {
                        component: ComponentId::new("daemon"),
                        target: "latest".into(),
                        installed: Some(semver::Version::new(0, 1, 3)),
                        source: "github.com/pollen-robotics/microduck".into(),
                        requested_by: Some("uid=1000 gid=1000 pid=2317".into()),
                    },
                ),
                at(
                    260,
                    RunEvent::Ended {
                        outcome: Some(Outcome::Success),
                        summary: "applied 0.1.3 → 0.1.4".into(),
                    },
                ),
            ],
            available: vec![4],
        };
        let out = render(&transcript);
        assert!(!out.contains("no ending"), "{out}");
        assert!(out.contains("applied 0.1.3 → 0.1.4"), "{out}");
        assert!(out.contains("requested by uid=1000"), "{out}");
        // The gap is what says the run took two and a half minutes.
        assert!(out.contains("+2m40s"), "{out}");
    }

    /// Hook output is the richest thing in a transcript and must survive rendering intact.
    #[test]
    fn hook_output_is_kept_line_by_line() {
        let transcript = RunTranscript {
            run: 5,
            component: ComponentId::new("daemon"),
            events: vec![at(
                100,
                RunEvent::Hook {
                    hook: "hooks/preinstall".into(),
                    exit_code: Some(0),
                    output: "installing onnxruntime\nchecking gstreamer".into(),
                },
            )],
            available: vec![5],
        };
        let out = render(&transcript);
        assert!(out.contains("│ installing onnxruntime"), "{out}");
        assert!(out.contains("│ checking gstreamer"), "{out}");
    }

    /// An old log entry has no run to point at, and the column must still line up.
    #[test]
    fn a_log_line_without_a_run_keeps_its_column() {
        let entry = duck_ipc_proto::LogEntry {
            at: 1_700_000_000,
            component: ComponentId::new("daemon"),
            from: None,
            to: Some(semver::Version::new(0, 1, 4)),
            outcome: Outcome::Success,
            run: None,
        };
        assert!(
            log_line(&entry).starts_with("   -  2023-11-14"),
            "{}",
            log_line(&entry)
        );

        let numbered = duck_ipc_proto::LogEntry {
            run: Some(42),
            ..entry
        };
        assert!(log_line(&numbered).starts_with("  42  2023-11-14"));
    }

    /// The whole rendering, on a run with one of everything.
    ///
    /// Asserts the shape a reader depends on rather than the exact bytes: the verdict above the
    /// timeline, the gutter that keeps hook output and the artifact URL out of the row, and the
    /// gaps that say where the minutes went. `docs/robot/cheatsheet.md` carries a rendered
    /// sample, which is the readable half of this.
    #[test]
    fn a_full_run_renders_as_a_timeline() {
        let t0 = 1_756_300_000;
        let transcript = RunTranscript {
            run: 42,
            component: ComponentId::new("daemon"),
            available: vec![42, 41, 40],
            events: vec![
                at(t0, RunEvent::Began {
                    component: ComponentId::new("daemon"),
                    target: "latest".into(),
                    installed: Some(semver::Version::new(0, 1, 3)),
                    source: "github.com/pollen-robotics/microduck".into(),
                    requested_by: Some("uid=1000 gid=1000 pid=2317".into()),
                }),
                at(t0, RunEvent::Phase { phase: Phase::Preflight, detail: None }),
                at(t0, RunEvent::Phase { phase: Phase::Checking, detail: None }),
                at(t0 + 1, RunEvent::Manifest {
                    version: semver::Version::new(0, 1, 4),
                    sha256: "3f9a1c2be4d7f08a91cc5517b2ad3e6690f1c0b4".into(),
                    bytes: Some(184_200_000),
                    url: Some("https://github.com/pollen-robotics/microduck/releases/download/daemon-v0.1.4/daemon-0.1.4.tar.zst".into()),
                    signed_by: Some("release.pub".into()),
                    source_revision: Some("88efc0341ab".into()),
                }),
                at(t0 + 1, RunEvent::Phase { phase: Phase::Downloading, detail: None }),
                at(t0 + 78, RunEvent::Note { text: "downloaded 184.2 MB to /opt/robot/daemon/staging/0.1.4/dl/daemon-0.1.4.tar.zst".into() }),
                at(t0 + 78, RunEvent::Phase { phase: Phase::Verifying, detail: None }),
                at(t0 + 82, RunEvent::Note { text: "artifact hash matches the manifest, and its signature verifies against release.pub".into() }),
                at(t0 + 82, RunEvent::Phase { phase: Phase::Extracting, detail: None }),
                at(t0 + 100, RunEvent::Phase { phase: Phase::RunningPreHook, detail: None }),
                at(t0 + 212, RunEvent::Hook {
                    hook: "hooks/preinstall".into(),
                    exit_code: Some(0),
                    output: "onnxruntime 1.20.1 already present\ngstreamer: h264 encode ok\n".into(),
                }),
                at(t0 + 212, RunEvent::Phase { phase: Phase::Swapping, detail: Some("0.1.3 → 0.1.4".into()) }),
                at(t0 + 212, RunEvent::Phase { phase: Phase::RunningPostHook, detail: None }),
                at(t0 + 213, RunEvent::Phase { phase: Phase::Applying, detail: None }),
                at(t0 + 214, RunEvent::Unit { unit: "robotd".into(), action: "restart".into(), detail: None }),
                at(t0 + 215, RunEvent::Note { text: "the new updaterd passed its self-test".into() }),
                at(t0 + 215, RunEvent::Phase { phase: Phase::HealthGate, detail: None }),
                at(t0 + 223, RunEvent::Health { passed: true, detail: None }),
                at(t0 + 223, RunEvent::Phase { phase: Phase::Committing, detail: None }),
                at(t0 + 223, RunEvent::Note { text: "pruned 0.1.1".into() }),
                at(t0 + 224, RunEvent::Unit { unit: "updaterd".into(), action: "restart in 5s".into(), detail: None }),
                at(t0 + 224, RunEvent::Unit { unit: "btd".into(), action: "restart in 5s".into(), detail: None }),
                at(t0 + 224, RunEvent::Ended {
                    outcome: Some(Outcome::Success),
                    summary: "applied 0.1.3 → 0.1.4".into(),
                }),
            ],
        };
        let out = render(&transcript);
        let lines: Vec<&str> = out.lines().collect();

        // The header: what happened, then what was asked for, then who asked.
        assert_eq!(lines[0], "run 42 · daemon · 2025-08-27 13:06:40 UTC");
        assert_eq!(lines[1], "  applied 0.1.3 → 0.1.4");
        assert!(
            lines[2].starts_with("  asked for latest, from github.com/"),
            "{}",
            lines[2]
        );
        assert!(lines[3].contains("requested by uid=1000"), "{}", lines[3]);

        // `began` is the header, and must not also be a row.
        assert!(!out.contains(" began"), "{out}");

        // The manifest row carries six facts and an artifact URL; the URL goes in the gutter so
        // the row itself stays readable. Free-text notes can be any length and are not this
        // assertion's business.
        let manifest = lines
            .iter()
            .find(|line| line.contains("manifest "))
            .expect("a manifest row");
        assert!(
            !manifest.contains("http"),
            "the artifact URL belongs in the gutter:\n{manifest}"
        );
        assert!(
            manifest.chars().count() <= 120,
            "the manifest row was {} columns:\n{manifest}",
            manifest.chars().count()
        );
        assert!(out.contains("│ https://github.com/"), "{out}");
        assert!(
            out.contains("│ onnxruntime 1.20.1 already present"),
            "{out}"
        );

        // The gaps are the point of the time column: they say where the minutes went.
        assert!(out.contains("+1m17s  note         downloaded"), "{out}");
        assert!(out.contains("+1m52s  hook"), "{out}");

        // And where else to look.
        assert!(out.contains("runs 40 to 42 are kept"), "{out}");
    }
}
