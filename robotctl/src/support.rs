//! A bounded, redacted support report for a robot somebody cannot inspect over SSH.

use std::fmt::Write as _;
use std::path::Path;
use std::process::Command;

const MAX_SECTION_BYTES: usize = 256 * 1024;

/// Run a local diagnostic command without a shell. Failure is evidence, not a reason to
/// abandon the bundle: the useful case is precisely when a daemon is absent.
pub fn command(program: &str, args: &[String]) -> String {
    match Command::new(program).args(args).output() {
        Ok(output) => {
            let mut text = String::from_utf8_lossy(&output.stdout).into_owned();
            text.push_str(&String::from_utf8_lossy(&output.stderr));
            if !output.status.success() {
                let _ = writeln!(text, "[command exited {}]", output.status);
            }
            bounded(text)
        }
        Err(error) => format!("[could not run {program}: {error}]"),
    }
}

pub fn file(path: &Path) -> String {
    match std::fs::read_to_string(path) {
        Ok(text) => bounded(text),
        Err(error) => format!("[could not read {}: {error}]", path.display()),
    }
}

fn bounded(mut text: String) -> String {
    if text.len() > MAX_SECTION_BYTES {
        text.truncate(MAX_SECTION_BYTES);
        text.push_str("\n[truncated]\n");
    }
    text
}

/// Lines that can plausibly carry a credential. A support bundle must be useful to send
/// to somebody else, so failing closed here beats preserving one more journal line.
pub fn redact(text: &str) -> String {
    text.lines()
        .map(|line| {
            let lower = line.to_ascii_lowercase();
            if [
                "password",
                "passphrase",
                "psk",
                "token",
                "authorization",
                "secret",
                "credential",
                "api-key",
                "api_key",
                "access_key",
                "sessionid",
                "set-cookie",
            ]
            .iter()
            .any(|needle| lower.contains(needle))
            {
                "[redacted: possible credential]".to_owned()
            } else {
                line.to_owned()
            }
        })
        .collect::<Vec<_>>()
        .join("\n")
}

/// Assemble labelled, already-bounded command output into one portable text report.
pub fn render(sections: &[(&str, &str)]) -> String {
    let mut out = String::from("microduck support bundle v1\n");
    for (name, body) in sections {
        let _ = writeln!(out, "\n===== {name} =====");
        out.push_str(&redact(body));
        out.push('\n');
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn credentials_are_removed_case_insensitively() {
        let out = redact(
            "normal\nPSK=hunter2\nAuthorization: Bearer abc\n\
             Credential: value\nX-Api-Key: value\naccess_key=value\n\
             Set-Cookie: session=value\nother",
        );
        assert_eq!(
            out,
            "normal\n[redacted: possible credential]\n[redacted: possible credential]\n\
             [redacted: possible credential]\n[redacted: possible credential]\n\
             [redacted: possible credential]\n[redacted: possible credential]\nother"
        );
    }

    #[test]
    fn ordinary_diagnostics_are_preserved() {
        assert_eq!(
            redact("robotd active\nbattery 7.4 V\nupdate completed"),
            "robotd active\nbattery 7.4 V\nupdate completed"
        );
    }

    #[test]
    fn sections_are_labelled_and_redacted() {
        let out = render(&[("health", "ok"), ("journal", "token=abc")]);
        assert!(out.contains("===== health =====\nok"));
        assert!(out.contains("===== journal =====\n[redacted: possible credential]"));
    }
}
