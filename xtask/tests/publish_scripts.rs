//! The publish scripts are run by hand, so a mistyped flag has to say what is wrong at the
//! command line rather than becoming an unbound-variable error or, worse, a Space id.

use std::path::Path;
use std::process::{Command, Output};

fn run(script: &str, args: &[&str]) -> Output {
    Command::new("bash")
        .arg(
            Path::new(env!("CARGO_MANIFEST_DIR"))
                .parent()
                .expect("xtask/ has a parent")
                .join("scripts")
                .join(script),
        )
        .args(args)
        .output()
        .expect("bash")
}

/// Both scripts answer a bad argument with a reason and exit 2. Under `set -u` a missing
/// `--space` value was an "unbound variable" error with exit 1, which in a CI log reads like
/// the script crashed rather than like somebody forgot an argument.
#[test]
fn a_space_flag_without_a_value_is_a_usage_error() {
    for (script, args) in [
        ("publish-console.sh", vec!["--space"]),
        ("publish-space.sh", vec!["vision-demo", "--space"]),
    ] {
        let out = run(script, &args);
        assert_eq!(out.status.code(), Some(2), "{script}: {out:?}");
        let stderr = String::from_utf8_lossy(&out.stderr);
        assert!(stderr.contains("--space"), "{script}: {stderr}");
    }
}

/// A following option must not be taken as the Space id: that would aim the publish at a Space
/// named `--dry-run` and quietly leave the real --dry-run unapplied.
#[test]
fn a_space_flag_does_not_take_the_next_flag_as_its_value() {
    for (script, args) in [
        ("publish-console.sh", vec!["--space", "--dry-run"]),
        (
            "publish-space.sh",
            vec!["vision-demo", "--space", "--dry-run"],
        ),
    ] {
        let out = run(script, &args);
        assert_eq!(out.status.code(), Some(2), "{script}: {out:?}");
    }
}

/// The first argument to publish-space.sh is a NAME. `--space` there means the name is missing,
/// not a directory under spaces/ called "--space" (which used to fail with exit 1 and a
/// file-not-found message).
#[test]
fn publishing_without_a_name_is_a_usage_error() {
    let out = run("publish-space.sh", &["--space"]);
    assert_eq!(out.status.code(), Some(2), "{out:?}");
    assert!(
        String::from_utf8_lossy(&out.stderr).contains("usage"),
        "{out:?}"
    );
}
