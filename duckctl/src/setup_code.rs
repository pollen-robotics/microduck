//! The setup codes this laptop holds, so a robot in secure mode is reached without typing its key.
//!
//! A robot whose `btd` has a setup code (`robotctl system setup-code`) serves nothing until the
//! client proves the key in a Noise handshake (`btd/src/secure.rs`), and the PIN path is closed.
//! The code is `md1:<first name>:<64 hex>`, which nobody types twice: `duckctl setup-code add`
//! reads it once, pasted from `robotctl system setup-code` or from the text under the QR, and every
//! later command finds it by the robot's name.
//!
//! ## Where, and what that file is
//!
//! `$XDG_CONFIG_HOME/duckctl/setup-codes`, else `~/.config/duckctl/setup-codes`, else under
//! `%APPDATA%`. Configuration rather than the address cache beside it, because deleting this one
//! is not free: it costs a trip to the robot for the code again. On unix it is written `0600`.
//!
//! **It is a credential.** Whoever copies it can talk to the robot over BLE, wifi passphrase and
//! all — the same thing the phone app holds in its protected storage, held here in a file.
//!
//! ## Found by name, and why there is more than one name per code
//!
//! One line per name: the name, a space, the code. The handshake binds the robot's *first name*
//! (the id in the code), but a scan reports the name the robot *advertises*, and those differ once
//! the robot is renamed. So the first name always has a line, and so does any other name the robot
//! was reached under: the `--name` given to `add`, and the new name after a `duckctl name`.

use std::path::PathBuf;

/// A parsed setup code: the robot's first name, which the handshake binds, and its key.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SetupCode {
    pub id: String,
    pub key: [u8; 32],
}

impl SetupCode {
    /// `md1:<first name>:<64 hex>`, or the whole QR line, which carries the label serial in front:
    /// `<label serial>:md1:…`. Surrounding whitespace is ignored, since it arrives pasted.
    pub fn parse(text: &str) -> Result<Self, String> {
        let text = text.trim();
        // The QR line is the code with the label serial in front, and a paste of either is fine.
        let code = match text.find("md1:") {
            Some(at) => &text[at..],
            None => return Err("a setup code starts with `md1:`".to_owned()),
        };
        let mut fields = code.split(':');
        fields.next();
        let (Some(id), Some(hex), None) = (fields.next(), fields.next(), fields.next()) else {
            return Err("a setup code is `md1:<robot id>:<64 hex characters>`".to_owned());
        };
        // The same bounds `btd` puts on the id it binds, so a code this accepts is one the robot
        // could have printed.
        if id.is_empty() || id.len() > 128 || !id.is_ascii() || id.contains(char::is_whitespace) {
            return Err(format!("{id:?} is not a robot id"));
        }
        if hex.len() != 64 || !hex.is_ascii() {
            return Err(format!(
                "the key is 64 hexadecimal characters, and this one is {}",
                hex.len()
            ));
        }
        let mut key = [0u8; 32];
        for (index, byte) in key.iter_mut().enumerate() {
            *byte = u8::from_str_radix(&hex[index * 2..index * 2 + 2], 16)
                .map_err(|_| "the key is hexadecimal, and this one is not".to_owned())?;
        }
        Ok(Self {
            id: id.to_owned(),
            key,
        })
    }

    /// Back to `md1:<id>:<hex>`, for the file.
    fn text(&self) -> String {
        let hex: String = self.key.iter().map(|b| format!("{b:02x}")).collect();
        format!("md1:{}:{hex}", self.id)
    }
}

/// `$XDG_CONFIG_HOME/duckctl/setup-codes`, else `~/.config/…`, else `%APPDATA%\duckctl\…`.
pub fn path() -> Option<PathBuf> {
    let base = std::env::var_os("XDG_CONFIG_HOME")
        .filter(|dir| !dir.is_empty())
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|home| PathBuf::from(home).join(".config")))
        .or_else(|| std::env::var_os("APPDATA").map(PathBuf::from))?;
    Some(base.join("duckctl").join("setup-codes"))
}

/// Every `(name, code)` line in `contents`. A line that does not parse is skipped rather than
/// failing the command: a hand edit gone wrong costs that robot's code, not every robot's.
pub fn entries(contents: &str) -> Vec<(String, SetupCode)> {
    contents
        .lines()
        .filter_map(|line| {
            let (name, code) = line.trim().split_once(' ')?;
            Some((name.to_owned(), SetupCode::parse(code).ok()?))
        })
        .collect()
}

/// The code for the robot the scan reported as `reported`, matched the way `--name` is.
pub fn lookup(contents: &str, reported: &str) -> Option<SetupCode> {
    entries(contents)
        .into_iter()
        .find(|(name, _)| crate::answers_to(reported, name))
        .map(|(_, code)| code)
}

/// `contents` with `code` filed under each of `names`, replacing whatever those names held.
pub fn with(contents: &str, code: &SetupCode, names: &[&str]) -> String {
    let mut lines: Vec<String> = entries(contents)
        .into_iter()
        .filter(|(name, _)| !names.contains(&name.as_str()))
        .map(|(name, code)| format!("{name} {}", code.text()))
        .collect();
    lines.extend(names.iter().map(|name| format!("{name} {}", code.text())));
    lines.join("\n") + "\n"
}

/// `contents` without any line for the robot `name` — every name its code is filed under, so a
/// removal does not leave the same key reachable through an alias.
pub fn without(contents: &str, name: &str) -> (String, usize) {
    let all = entries(contents);
    let ids: Vec<String> = all
        .iter()
        .filter(|(other, code)| other == name || code.id == name)
        .map(|(_, code)| code.id.clone())
        .collect();
    let kept: Vec<String> = all
        .iter()
        .filter(|(_, code)| !ids.contains(&code.id))
        .map(|(name, code)| format!("{name} {}", code.text()))
        .collect();
    let removed = all.len() - kept.len();
    let text = if kept.is_empty() {
        String::new()
    } else {
        kept.join("\n") + "\n"
    };
    (text, removed)
}

/// The file's contents, empty when there is no file yet.
pub fn read() -> String {
    path()
        .and_then(|path| std::fs::read_to_string(path).ok())
        .unwrap_or_default()
}

/// Replace the file, readable by this user only.
pub fn write(contents: &str) -> Result<PathBuf, String> {
    let path = path().ok_or("no home directory to keep setup codes in")?;
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    }
    let mut options = std::fs::OpenOptions::new();
    options.write(true).create(true).truncate(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        options.mode(0o600);
    }
    let mut file = options
        .open(&path)
        .map_err(|e| format!("{}: {e}", path.display()))?;
    // `mode` applies only when the file is created; one made by hand before this keeps its own.
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let _ = file.set_permissions(std::fs::Permissions::from_mode(0o600));
    }
    std::io::Write::write_all(&mut file, contents.as_bytes())
        .map_err(|e| format!("{}: {e}", path.display()))?;
    Ok(path)
}

/// File the code for a robot that now answers to `name` as well. Best effort, like the address
/// cache: failing costs the next command a `setup-code add`, which is what it cost before.
pub fn alias(code: &SetupCode, name: &str) {
    let contents = read();
    if lookup(&contents, name).is_some_and(|held| &held == code) {
        return;
    }
    let _ = write(&with(&contents, code, &[name]));
}

#[cfg(test)]
mod tests {
    use super::*;

    const KEY: &str = "00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff";

    #[test]
    fn parses_a_code_and_the_qr_line_alike() {
        let code = SetupCode::parse(&format!("md1:duck-ccf2:{KEY}")).unwrap();
        assert_eq!(code.id, "duck-ccf2");
        assert_eq!(code.key[1], 0x11);
        let qr = SetupCode::parse(&format!("  202004743264300098:md1:duck-ccf2:{KEY}\n")).unwrap();
        assert_eq!(qr, code);
        assert_eq!(code.text(), format!("md1:duck-ccf2:{KEY}"));
    }

    #[test]
    fn refuses_what_the_robot_could_not_have_printed() {
        assert!(SetupCode::parse(KEY).is_err());
        assert!(SetupCode::parse(&format!("md1:duck-ccf2:{}", &KEY[..62])).is_err());
        assert!(SetupCode::parse(&format!("md1:duck-ccf2:{}zz", &KEY[..62])).is_err());
        assert!(SetupCode::parse(&format!("md1::{KEY}")).is_err());
        assert!(SetupCode::parse(&format!("md1:duck-ccf2:{KEY}:extra")).is_err());
    }

    #[test]
    fn finds_a_code_under_any_name_it_was_filed_under() {
        let code = SetupCode::parse(&format!("md1:duck-ccf2:{KEY}")).unwrap();
        let file = with("", &code, &["duck-ccf2", "Ducky"]);
        assert_eq!(lookup(&file, "duck-ccf2"), Some(code.clone()));
        assert_eq!(lookup(&file, "Ducky"), Some(code.clone()));
        // The macOS form, GAP name and advertised name together.
        assert_eq!(lookup(&file, "radxa-zero3 [Ducky]"), Some(code));
        assert_eq!(lookup(&file, "duck-0000"), None);
    }

    #[test]
    fn a_new_code_replaces_the_old_one_under_the_same_name() {
        let old = SetupCode::parse(&format!("md1:duck-ccf2:{KEY}")).unwrap();
        let new = SetupCode::parse(&format!("md1:duck-ccf2:{}", "ab".repeat(32))).unwrap();
        let file = with(&with("", &old, &["duck-ccf2"]), &new, &["duck-ccf2"]);
        assert_eq!(entries(&file).len(), 1);
        assert_eq!(lookup(&file, "duck-ccf2"), Some(new));
    }

    #[test]
    fn removing_a_robot_removes_its_aliases_too() {
        let ccf2 = SetupCode::parse(&format!("md1:duck-ccf2:{KEY}")).unwrap();
        let other = SetupCode::parse(&format!("md1:duck-0001:{KEY}")).unwrap();
        let file = with(
            &with("", &ccf2, &["duck-ccf2", "Ducky"]),
            &other,
            &["duck-0001"],
        );
        let (left, removed) = without(&file, "Ducky");
        assert_eq!(removed, 2);
        assert_eq!(lookup(&left, "duck-ccf2"), None);
        assert!(lookup(&left, "duck-0001").is_some());
    }

    #[test]
    fn a_damaged_line_costs_only_itself() {
        let file = format!("garbage\nDucky md1:duck-ccf2:{KEY}\nhalf md1:duck-x:12\n");
        assert_eq!(entries(&file).len(), 1);
    }
}
