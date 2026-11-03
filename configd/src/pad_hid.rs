//! Read-only HID identity evidence. A link or service being up is not an input node.
use std::path::Path;

/// Whether sysfs exposes a Bluetooth input node with this exact address in uniq.
/// A negative result means no match, not proof that a driver could establish that identity.
pub fn recognized(root: &Path, mac: &str) -> Option<bool> {
    let entries = std::fs::read_dir(root).ok()?;
    let mut complete = true;
    for entry in entries {
        let Ok(entry) = entry else {
            complete = false;
            continue;
        };
        if !entry.file_name().to_string_lossy().starts_with("event") {
            continue;
        }
        let device = entry.path().join("device");
        let bus = match std::fs::read_to_string(device.join("id/bustype")) {
            Ok(value) => value,
            Err(_) => {
                complete = false;
                continue;
            }
        };
        if bus.trim() != "0005" {
            continue;
        }
        match std::fs::read_to_string(device.join("uniq")) {
            Ok(uniq) if uniq.trim().eq_ignore_ascii_case(mac) => return Some(true),
            Ok(uniq) if !uniq.trim().is_empty() => {}
            _ => complete = false,
        }
    }
    complete.then_some(false)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn matches_only_exact_bluetooth_identity_and_reports_missing_evidence() {
        let dir = tempfile::tempdir().unwrap();
        let node = dir.path().join("event3/device");
        std::fs::create_dir_all(node.join("id")).unwrap();
        std::fs::write(node.join("id/bustype"), "0003\n").unwrap();
        std::fs::write(node.join("uniq"), "98:b6:e9:a4:fc:67\n").unwrap();
        assert_eq!(
            recognized(dir.path(), "98:B6:E9:A4:FC:67"),
            Some(false),
            "USB identity must not confirm Bluetooth"
        );
        std::fs::write(node.join("id/bustype"), "0005\n").unwrap();
        assert_eq!(recognized(dir.path(), "98:B6:E9:A4:FC:67"), Some(true));
        assert_eq!(
            recognized(dir.path(), "98:B6:ED:A4:FC:67"),
            Some(false),
            "no suffix matching"
        );
        std::fs::remove_file(node.join("uniq")).unwrap();
        assert_eq!(recognized(dir.path(), "98:B6:E9:A4:FC:67"), None);
        assert_eq!(
            recognized(&dir.path().join("absent"), "98:B6:E9:A4:FC:67"),
            None
        );
    }
}
