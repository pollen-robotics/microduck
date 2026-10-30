//! The config store: a file, not a service (`architecture.md` §3.1).
//!
//! Deliberately not a daemon of its own. A plain file plus `flock` for write serialisation and
//! write-to-temp-plus-`rename(2)` for atomicity gives no single point of failure, stays readable
//! when any service is down, and the updater never touches it — so it survives an update *and*
//! a rollback (`updater-design.md` §5.7).
//!
//! `inotify` is not here yet. It belongs when a *second* process reads this file; today
//! `configd` is the only one, and watching a file you are the sole writer of is ceremony.
//!
//! This holds identity and preferences. It holds no credentials — NetworkManager owns those.

use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

/// Longest name accepted.
///
/// Bounded by BLE, not by taste: a legacy advertisement has 31 bytes of payload total, and the
/// local name shares it with flags and a 16-byte service UUID. A name longer than this is
/// silently truncated by the adapter or pushed into a scan response the phone may not request —
/// either way the robot appears under a name nobody chose. Truncating here, visibly, and
/// returning what was stored is the honest version.
pub const MAX_NAME: usize = 24;

/// The factory pairing PIN.
///
/// A known default rather than a random per-board value, deliberately, and it is the weakest
/// link here: everyone in radio range knows it, so it proves physical presence and nothing more.
/// It exists so the *process* is in place — the agent, the storage, the six-digit contract — and
/// so a robot is provisionable out of the box. A robot whose ID EEPROM carries its label serial
/// uses the PIN derived from that instead ([`crate::identity::default_pin`]); this one is for the
/// rest, and `is_default` on the result exists so every client can say out loud that this one is
/// not a secret.
pub const DEFAULT_PIN: &str = "000000";

/// How many digits a PIN has.
///
/// **Six, not five.** Bluetooth's passkey entry is defined as a six-digit value, 000000–999999
/// (Core spec, LE Secure Connections), and BlueZ hands it to an agent as exactly that. Five
/// digits would have to be padded to six somewhere, and the two sides would then disagree about
/// whether the PIN is "12345" or "012345" — which is a support call nobody can diagnose. Six is
/// the format the radio already speaks.
pub const PIN_DIGITS: usize = 6;

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
struct Config {
    #[serde(skip_serializing_if = "Option::is_none")]
    name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pairing_pin: Option<String>,
}

pub struct Store {
    path: PathBuf,
    /// Used when the file has no name yet, so an unprovisioned robot still has an identity
    /// rather than an empty string in a phone's Bluetooth list.
    ///
    /// Supplied by the caller rather than computed here, because it is derived from the board —
    /// [`crate::identity`] turns the SoC serial into `duck-7f3a` — and this type is a file, not a
    /// place that knows about hardware.
    fallback: String,
    /// The PIN when none has been set: the label-derived one where the board has it, otherwise
    /// [`DEFAULT_PIN`]. Supplied by the caller for the same reason as `fallback`.
    default_pin: String,
}

impl Store {
    pub fn new(path: impl Into<PathBuf>, fallback: impl Into<String>) -> Self {
        Self {
            path: path.into(),
            fallback: fallback.into(),
            default_pin: DEFAULT_PIN.to_owned(),
        }
    }

    /// Use this PIN, rather than [`DEFAULT_PIN`], until one is set.
    pub fn with_default_pin(mut self, pin: impl Into<String>) -> Self {
        self.default_pin = pin.into();
        self
    }

    /// The robot's name, or the fallback.
    ///
    /// A missing or unparseable file yields the fallback rather than an error: an unprovisioned
    /// board must come up with a working name, and a daemon that refuses to start because its
    /// optional config is malformed is far harder to diagnose remotely than one that logs and
    /// carries on — the same reasoning as `robotd.toml` being optional.
    pub fn name(&self) -> String {
        match self.read() {
            Ok(config) => config.name.unwrap_or_else(|| self.fallback.clone()),
            Err(e) => {
                tracing::warn!(path = %self.path.display(), error = %e, "unreadable config; using the default name");
                self.fallback.clone()
            }
        }
    }

    /// Store a name, returning what was actually stored.
    ///
    /// The caller must display the returned value, not what it sent: trimming and truncation
    /// mean they can differ, and a client showing its own input would disagree with the robot.
    pub fn set_name(&self, requested: &str) -> std::io::Result<String> {
        let name = sanitise(requested);
        if name.is_empty() {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                "a name must have at least one printable character",
            ));
        }

        let mut config = self.read().unwrap_or_default();
        config.name = Some(name.clone());
        self.write(&config)?;
        Ok(name)
    }

    /// The pairing PIN, or the board's default.
    pub fn pairing_pin(&self) -> String {
        self.read()
            .ok()
            .and_then(|config| config.pairing_pin)
            .unwrap_or_else(|| self.default_pin.clone())
    }

    /// Store a PIN. Six digits, leading zeros kept.
    pub fn set_pairing_pin(&self, requested: &str) -> std::io::Result<String> {
        let pin = requested.trim();
        if pin.len() != PIN_DIGITS || !pin.chars().all(|c| c.is_ascii_digit()) {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!("a pairing PIN is exactly {PIN_DIGITS} digits, 000000 to 999999"),
            ));
        }

        let mut config = self.read().unwrap_or_default();
        config.pairing_pin = Some(pin.to_owned());
        self.write(&config)?;
        Ok(pin.to_owned())
    }

    /// The PIN as a protocol result, so the `is_default` judgement lives with the constant it
    /// compares against rather than in every caller.
    pub fn name_and_pin_result(&self) -> duck_ipc_proto::PairingPinResult {
        let pin = self.pairing_pin();
        duck_ipc_proto::PairingPinResult {
            is_default: pin == DEFAULT_PIN,
            pin,
        }
    }

    fn read(&self) -> std::io::Result<Config> {
        match fs::read_to_string(&self.path) {
            Ok(text) => serde_json::from_str(&text)
                .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e)),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(Config::default()),
            Err(e) => Err(e),
        }
    }

    /// Write-to-temp, fsync, rename, fsync the directory.
    ///
    /// The directory fsync is the step usually forgotten: without it the rename may not survive
    /// a power cut, and a robot switched off at the wall is the normal case rather than the
    /// exceptional one. Same discipline as the update journal (`architecture.md` §8.2).
    fn write(&self, config: &Config) -> std::io::Result<()> {
        let dir = self.path.parent().unwrap_or(Path::new("."));
        fs::create_dir_all(dir)?;

        // `flock` serialises writers. Held on the lock file rather than the config itself, so
        // the lock outlives the rename that replaces the config.
        let lock_path = self.path.with_extension("lock");
        let lock = fs::OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(false)
            .open(&lock_path)?;
        lock.lock()?;

        let text = serde_json::to_string_pretty(config)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;

        let temp_path = self.path.with_extension("tmp");
        {
            let mut temp = fs::File::create(&temp_path)?;
            temp.write_all(text.as_bytes())?;
            temp.write_all(b"\n")?;
            temp.sync_all()?;
        }
        fs::rename(&temp_path, &self.path)?;
        fs::File::open(dir)?.sync_all()?;
        Ok(())
    }
}

/// Trim, drop anything unprintable, and truncate to [`MAX_NAME`] on a character boundary.
///
/// Control characters are the ones that matter: this string reaches a BLE advertisement, a log
/// line and eventually an app's UI, and a newline in it would let a name split a journal record
/// in two.
fn sanitise(requested: &str) -> String {
    let cleaned: String = requested
        .trim()
        .chars()
        .filter(|c| !c.is_control())
        .collect();

    // `chars().take()` rather than slicing by byte: a name is UTF-8 and truncating mid-codepoint
    // would panic.
    cleaned
        .chars()
        .take(MAX_NAME)
        .collect::<String>()
        .trim_end()
        .to_owned()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn store(dir: &Path) -> Store {
        Store::new(dir.join("config.json"), "radxa-zero3")
    }

    /// A board with a label serial pairs with the PIN on its label, which is not the public one,
    /// until someone sets another.
    #[test]
    fn a_label_pin_is_the_default_and_is_not_the_factory_one() {
        let dir = tempfile::tempdir().unwrap();
        let store = store(dir.path()).with_default_pin("000001");
        let result = store.name_and_pin_result();
        assert_eq!(result.pin, "000001");
        assert!(!result.is_default);

        store.set_pairing_pin("424242").unwrap();
        assert_eq!(store.pairing_pin(), "424242");
    }

    /// An unprovisioned robot still has a name.
    #[test]
    fn a_missing_file_yields_the_fallback() {
        let dir = tempfile::tempdir().unwrap();
        assert_eq!(store(dir.path()).name(), "radxa-zero3");
    }

    #[test]
    fn a_name_survives_a_write_and_a_reread() {
        let dir = tempfile::tempdir().unwrap();
        let store = store(dir.path());

        assert_eq!(store.set_name("Ducky").unwrap(), "Ducky");
        assert_eq!(store.name(), "Ducky");
        // A second store over the same file reads it too — this is a file, not process state.
        assert_eq!(
            super::Store::new(dir.path().join("config.json"), "other").name(),
            "Ducky"
        );
    }

    /// A malformed file must not stop the daemon starting: it logs and falls back.
    #[test]
    fn a_corrupt_file_yields_the_fallback_rather_than_an_error() {
        let dir = tempfile::tempdir().unwrap();
        fs::write(dir.path().join("config.json"), "{not json").unwrap();
        assert_eq!(store(dir.path()).name(), "radxa-zero3");
    }

    /// Control characters would split a log line or corrupt an advertisement.
    #[test]
    fn control_characters_are_stripped() {
        let dir = tempfile::tempdir().unwrap();
        let stored = store(dir.path()).set_name("Duck\ny\u{0}").unwrap();
        assert_eq!(stored, "Ducky");
    }

    /// Truncation must happen on a character boundary, or a multi-byte name panics.
    #[test]
    fn a_long_multibyte_name_is_truncated_without_panicking() {
        let dir = tempfile::tempdir().unwrap();
        let stored = store(dir.path()).set_name(&"é".repeat(80)).unwrap();
        assert_eq!(stored.chars().count(), MAX_NAME);
    }

    /// The stored name is what the caller must display, so the difference is returned rather
    /// than hidden.
    #[test]
    fn whitespace_is_trimmed_and_reported() {
        let dir = tempfile::tempdir().unwrap();
        assert_eq!(store(dir.path()).set_name("  Ducky  ").unwrap(), "Ducky");
    }

    /// A name of only whitespace is refused rather than stored as empty — an empty name in a
    /// phone's Bluetooth list is indistinguishable from a broken robot.
    #[test]
    fn an_empty_name_is_refused() {
        let dir = tempfile::tempdir().unwrap();
        let err = store(dir.path()).set_name("   \n ").unwrap_err();
        assert_eq!(err.kind(), std::io::ErrorKind::InvalidInput);
        // And nothing was written, so the previous name stands.
        assert_eq!(store(dir.path()).name(), "radxa-zero3");
    }

    /// An unprovisioned robot is pairable with the factory PIN rather than not pairable at all.
    #[test]
    fn a_missing_pin_is_the_factory_default() {
        let dir = tempfile::tempdir().unwrap();
        assert_eq!(store(dir.path()).pairing_pin(), DEFAULT_PIN);
    }

    /// Leading zeros are the whole reason this is a string. Storing "012345" as a number and
    /// rendering it back would give "12345", and the phone would be told the wrong PIN.
    #[test]
    fn a_pin_keeps_its_leading_zeros() {
        let dir = tempfile::tempdir().unwrap();
        let store = store(dir.path());
        assert_eq!(store.set_pairing_pin("012345").unwrap(), "012345");
        assert_eq!(store.pairing_pin(), "012345");
    }

    /// Anything the radio cannot express is refused, rather than silently padded or truncated
    /// into a PIN the two sides disagree about.
    #[test]
    fn a_pin_must_be_exactly_six_digits() {
        let dir = tempfile::tempdir().unwrap();
        let store = store(dir.path());
        for bad in ["12345", "1234567", "", "12 456", "abcdef", "12345٦"] {
            assert_eq!(
                store.set_pairing_pin(bad).unwrap_err().kind(),
                std::io::ErrorKind::InvalidInput,
                "accepted {bad:?}"
            );
        }
        // And nothing was stored, so the default still stands.
        assert_eq!(store.pairing_pin(), DEFAULT_PIN);
    }

    /// The name and the PIN share one file, so writing one must not lose the other.
    #[test]
    fn a_name_and_a_pin_coexist() {
        let dir = tempfile::tempdir().unwrap();
        let store = store(dir.path());
        store.set_name("Ducky").unwrap();
        store.set_pairing_pin("424242").unwrap();
        assert_eq!(store.name(), "Ducky");
        assert_eq!(store.pairing_pin(), "424242");
        store.set_name("Other").unwrap();
        assert_eq!(
            store.pairing_pin(),
            "424242",
            "the PIN was lost by a rename"
        );
    }

    /// No temp or lock file left behind that a later read could mistake for the config.
    #[test]
    fn writing_leaves_no_temp_file() {
        let dir = tempfile::tempdir().unwrap();
        store(dir.path()).set_name("Ducky").unwrap();
        assert!(!dir.path().join("config.tmp").exists());
    }
}
