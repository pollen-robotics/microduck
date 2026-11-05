//! `robotctl stress verify`: is this run's data whole?
//!
//! It reads what a run left and says what is wrong with it, rather than leaving that to be found
//! as a crash in the plot script or a table quietly short of rows. `scripts/endurance-plot.py`
//! makes the same checks on what it is given.

use std::path::Path;

/// A step back smaller than this is two threads logging at once, not a clock.
const BACK_SLACK: f64 = 1.0;

/// What one run directory holds, and what is wrong with it.
#[derive(Debug, Default)]
pub struct Report {
    pub samples: usize,
    pub events: usize,
    pub problems: Vec<String>,
}

/// The rows of a CSV as `csv_field` writes it, with the line each starts on. Quoted fields may
/// hold commas, doubled quotes and newlines.
fn rows(text: &str) -> Vec<(usize, Vec<String>)> {
    let (mut rows, mut row, mut field) = (Vec::new(), Vec::new(), String::new());
    let (mut quoted, mut line, mut start) = (false, 1, 1);
    let mut chars = text.chars().peekable();
    while let Some(c) = chars.next() {
        match (quoted, c) {
            (true, '"') if chars.peek() == Some(&'"') => {
                chars.next();
                field.push('"');
            }
            (_, '"') => quoted = !quoted,
            (false, ',') => row.push(std::mem::take(&mut field)),
            (false, '\n') => {
                row.push(std::mem::take(&mut field));
                rows.push((start, std::mem::take(&mut row)));
                line += 1;
                start = line;
            }
            (_, c) => {
                if c == '\n' {
                    line += 1;
                }
                field.push(c);
            }
        }
    }
    rows
}

/// Check one CSV and return its row count. `resumes` is how many times the timer may restart: a
/// run appended to starts `t_s` over once per segment after the first.
fn check_csv(dir: &Path, name: &str, resumes: usize, problems: &mut Vec<String>) -> usize {
    let bytes = match std::fs::read(dir.join(name)) {
        Ok(bytes) => bytes,
        Err(e) => {
            problems.push(format!("{name}: {e}"));
            return 0;
        }
    };
    if bytes.is_empty() {
        problems.push(format!("{name}: empty"));
        return 0;
    }
    if bytes.last() != Some(&b'\n') {
        problems.push(format!(
            "{name}: the last row is cut short; a resume repairs it"
        ));
    }
    let text = String::from_utf8_lossy(&bytes);
    if text.contains('\u{fffd}') || text.contains('\0') {
        problems.push(format!("{name}: holds bytes that are not text"));
    }
    let table = rows(&text);
    let Some(((_, header), body)) = table.split_first() else {
        problems.push(format!("{name}: no header"));
        return 0;
    };
    let column = |wanted: &str| header.iter().position(|h| h == wanted);
    let number =
        |row: &[String], at: Option<usize>| at.and_then(|i| row.get(i)?.parse::<f64>().ok());
    let (wall_at, t_at) = (column("wall"), column("t_s"));
    let (mut misaligned, mut first_bad) = (0, 0);
    let (mut restarts, mut clock_back) = (0, 0);
    let (mut last_wall, mut last_t) = (f64::NEG_INFINITY, f64::NEG_INFINITY);
    for (line, row) in body {
        if row.len() != header.len() {
            misaligned += 1;
            if first_bad == 0 {
                first_bad = *line;
            }
            continue;
        }
        if let Some(t) = number(row, t_at) {
            if t < last_t - BACK_SLACK {
                restarts += 1;
            }
            last_t = t;
        }
        if let Some(wall) = number(row, wall_at) {
            if wall < last_wall - BACK_SLACK {
                clock_back += 1;
            }
            last_wall = wall;
        }
    }
    if misaligned > 0 {
        problems.push(format!(
            "{name}: {misaligned} rows do not have {} columns, the first on line {first_bad}",
            header.len()
        ));
    }
    if restarts > resumes {
        problems.push(format!(
            "{name}: the run's timer restarted {restarts} times, with {resumes} resumes to explain it"
        ));
    }
    if clock_back > 0 {
        problems.push(format!(
            "{name}: the wall clock goes backwards {clock_back} times"
        ));
    }
    body.len()
}

/// What is wrong with the run in `dir`, if anything.
pub fn check(dir: &Path) -> Report {
    let mut problems = Vec::new();
    let segments = match std::fs::read_to_string(dir.join("meta.json")) {
        Err(e) => {
            problems.push(format!("meta.json: {e}"));
            None
        }
        Ok(text) => match serde_json::from_str::<serde_json::Value>(&text) {
            Err(e) => {
                problems.push(format!("meta.json: unreadable: {e}"));
                None
            }
            Ok(meta) => {
                let segments = meta["segments"].as_array().map_or(0, Vec::len);
                if segments == 0 {
                    problems.push("meta.json: no segments".into());
                }
                Some(segments)
            }
        },
    };
    // With no meta.json to say how many there were, the timer's restarts prove nothing.
    let resumes = segments.map_or(usize::MAX, |n| n.saturating_sub(1));
    let samples = check_csv(dir, "samples.csv", resumes, &mut problems);
    let events = check_csv(dir, "events.csv", resumes, &mut problems);
    Report {
        samples,
        events,
        problems,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const META: &str = r#"{"segments":[{"boot_id":"a"}]}"#;
    const EVENTS: &str = "wall,t_s,kind,detail\n100.0,0.0,start,\"a, b\"\n101.0,1.0,stop,\n";
    const SAMPLES: &str = "wall,t_s,lap\n100.0,0.0,1\n110.0,10.0,1\n";

    fn run(meta: &str, samples: &str, events: &str) -> tempfile::TempDir {
        let dir = tempfile::tempdir().unwrap();
        std::fs::write(dir.path().join("meta.json"), meta).unwrap();
        std::fs::write(dir.path().join("samples.csv"), samples).unwrap();
        std::fs::write(dir.path().join("events.csv"), events).unwrap();
        dir
    }

    #[test]
    fn a_whole_run_has_nothing_wrong() {
        let dir = run(META, SAMPLES, EVENTS);
        let report = check(dir.path());
        assert!(report.problems.is_empty(), "{:?}", report.problems);
        assert_eq!((report.samples, report.events), (2, 2));
    }

    #[test]
    fn a_quoted_field_may_hold_a_newline_and_a_quote() {
        let table = rows("a,b\n\"x\ny\",\"say \"\"hi\"\"\"\n1,2\n");
        assert_eq!(table[1].1, ["x\ny", "say \"hi\""]);
        assert_eq!(
            table[2].0, 4,
            "line numbers count the one inside the quotes"
        );
    }

    #[test]
    fn an_empty_meta_is_reported() {
        let dir = run("", SAMPLES, EVENTS);
        let problems = check(dir.path()).problems;
        assert_eq!(problems.len(), 1, "{problems:?}");
        assert!(problems[0].contains("meta.json"), "{problems:?}");
    }

    #[test]
    fn a_missing_file_is_reported() {
        let dir = run(META, SAMPLES, EVENTS);
        std::fs::remove_file(dir.path().join("events.csv")).unwrap();
        let problems = check(dir.path()).problems;
        assert_eq!(problems.len(), 1, "{problems:?}");
        assert!(problems[0].contains("events.csv"), "{problems:?}");
    }

    #[test]
    fn a_torn_last_row_is_reported() {
        let dir = run(META, "wall,t_s,lap\n100.0,0.0,1\n110.0,", EVENTS);
        let problems = check(dir.path()).problems;
        assert!(
            problems.iter().any(|p| p.contains("cut short")),
            "{problems:?}"
        );
    }

    #[test]
    fn a_misaligned_row_is_reported_with_its_line() {
        let dir = run(META, "wall,t_s,lap\n100.0,0.0,1\n110.0,10.0\n", EVENTS);
        let problems = check(dir.path()).problems;
        assert_eq!(problems.len(), 1, "{problems:?}");
        assert!(problems[0].contains("first on line 3"), "{problems:?}");
    }

    #[test]
    fn a_timer_restart_needs_a_segment_to_explain_it() {
        let resumed = "wall,t_s,lap\n100.0,50.0,1\n200.0,0.0,1\n";
        let dir = run(META, resumed, EVENTS);
        assert_eq!(check(dir.path()).problems.len(), 1);
        let two = r#"{"segments":[{"boot_id":"a"},{"boot_id":"b"}]}"#;
        let dir = run(two, resumed, EVENTS);
        assert_eq!(check(dir.path()).problems, Vec::<String>::new());
    }

    #[test]
    fn a_wall_clock_going_backwards_is_reported() {
        let dir = run(META, "wall,t_s,lap\n200.0,0.0,1\n100.0,10.0,1\n", EVENTS);
        let problems = check(dir.path()).problems;
        assert!(
            problems.iter().any(|p| p.contains("backwards")),
            "{problems:?}"
        );
    }

    #[test]
    fn a_small_step_back_is_two_threads_logging() {
        let dir = run(META, "wall,t_s,lap\n100.5,10.5,1\n100.0,10.0,1\n", EVENTS);
        assert_eq!(check(dir.path()).problems, Vec::<String>::new());
    }
}
