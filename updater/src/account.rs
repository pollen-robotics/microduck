//! Which Hugging Face account this robot belongs to — the duck's half of it.
//!
//! The flow itself is [`hf_robot_account`]: an OAuth device grant against Hugging Face, a
//! credential on disk, and a loop that renews it. It was written here and lives there now,
//! because a robot signing in to an account has nothing to do with updating one, and because
//! every other robot wants exactly the same thing.
//!
//! What stays is everything that is a fact about *this* robot rather than about the flow:
//!
//! - **[`DEFAULT_PATH`] and [`TOKEN_GROUP`]**, which are this board's filesystem and this board's
//!   user database.
//! - **The mapping onto [`proto`]**, below. The crate's types happen to serialise identically
//!   today, so `Response::ok` could be handed one directly and the wire would not change — and
//!   that is exactly the shortcut not to take. `API_VERSION` versions these structs; a field
//!   renamed in a crate we do not release must be a compile error here, not a silently different
//!   answer to a client that has been told the version did not move.
//! - **The error mapping**, in `impl From<hf_robot_account::Error> for crate::Error`, because
//!   which JSON-RPC code a refusal deserves is a question about this protocol.
//!
//! An account is what makes a robot reachable from *outside* the LAN: the relay proves the robot
//! belongs to an account, the rendezvous service shows a client only its own robots, and the pair
//! of those is the authorisation a bridged session arrives with. A duck on a LAN needs none of it.
//! `docs/design/remote-access-design.md` §2 owns that argument.

use hf_robot_account::{Config, FileStore, Identity, LoginCode, Status};

use std::path::Path;

use crate::proto;

pub use hf_robot_account::{Account, maintain};

/// Where the credential lives.
///
/// **Not in `robotd.toml`.** Every mechanism that exists for that file is wrong for a secret:
/// `robotctl configure --list` prints what a robot changes, the config editor shows the whole
/// file, and "what was changed on this robot" is a report we generate. A bearer token would be in
/// all three.
///
/// **Under `/var/lib/robot`, not `/etc/robot`.** A robot that updates its OS by A/B slots replaces
/// `/etc` with every image, and a sign-in is the owner's, not the image's: in `/etc` every OS update
/// signed the robot out. `/var/lib/robot` is the tree that survives both kinds of update. Directly
/// in it rather than in a directory of its own, because the store renames a temp file beside the
/// credential and creates no directories, and this one always exists.
pub const DEFAULT_PATH: &str = "/var/lib/robot/hf-token";

/// Where the credential lived before it moved. See [`migrate_legacy`].
pub const LEGACY_PATH: &str = "/etc/robot/hf-token";

/// Move a credential left at [`LEGACY_PATH`] to [`DEFAULT_PATH`], at `updaterd`'s startup, and
/// leave a symlink to it behind.
///
/// Without this, the release that moved the path would sign out every robot it was installed
/// on. It has to run while the old file is still visible — on an A/B robot, before the next OS
/// update replaces `/etc` — which is why it is `updaterd`'s startup and not anything later.
///
/// **The symlink is for a rollback.** A release that predates the move reads only the old path;
/// deleting the file there signed a rolled-back robot out. Through the link it reads the current
/// credential, and after `account.logout` deletes that, a dangling link reads as signed out — so
/// no second copy of the secret outlives a logout. A rolled-back `updaterd` that signs in again
/// replaces the link with a file, by renaming over it; the next start of a newer one finds a file
/// again, and the newer of the two credentials is the one kept.
pub fn migrate_legacy() {
    let (from, to) = (Path::new(LEGACY_PATH), Path::new(DEFAULT_PATH));
    match migrate(from, to) {
        Ok(true) => {
            tracing::warn!(from = %from.display(), to = %to.display(), "moved the account credential")
        }
        Ok(false) => {}
        Err(e) => tracing::error!(
            from = %from.display(),
            to = %to.display(),
            error = %e,
            "could not move the account credential; the robot reads as signed out until it is moved"
        ),
    }
}

/// [`migrate_legacy`] between any two paths. `Ok(true)` when a credential moved.
fn migrate(from: &Path, to: &Path) -> std::io::Result<bool> {
    use std::io::Write as _;
    use std::os::unix::fs::{MetadataExt as _, OpenOptionsExt as _};

    let meta = match std::fs::symlink_metadata(from) {
        Ok(meta) => meta,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(false),
        Err(e) => return Err(e),
    };
    // Already the link: moved on an earlier start.
    if meta.file_type().is_symlink() {
        return Ok(false);
    }
    // A credential at both paths: the newer is the current sign-in. One at the new path that is
    // at least as recent wins, and the old file only becomes the link.
    let current = match std::fs::metadata(to) {
        Ok(to_meta) => to_meta.modified()? >= meta.modified()?,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => false,
        Err(e) => return Err(e),
    };
    if current {
        link_in_place(from, to)?;
        return Ok(false);
    }
    let bytes = std::fs::read(from)?;
    // Private from the first byte, then given the old file's group and mode — the store's own
    // discipline, so the token is never briefly readable by anyone it was not readable by before.
    let tmp = to.with_extension("migrating");
    {
        let mut file = std::fs::OpenOptions::new()
            .write(true)
            .create(true)
            .truncate(true)
            .mode(0o600)
            .open(&tmp)?;
        file.write_all(&bytes)?;
        file.sync_all()?;
    }
    // A chown this process may not do leaves the file private, which is the safe failure.
    let _ = std::os::unix::fs::chown(&tmp, None, Some(meta.gid()));
    std::fs::set_permissions(&tmp, meta.permissions())?;
    std::fs::rename(&tmp, to)?;
    if let Some(dir) = to.parent() {
        std::fs::File::open(dir)?.sync_all()?;
    }
    link_in_place(from, to)?;
    Ok(true)
}

/// Replace the file at `at` with a symlink to `target`, by rename, so `at` is never missing.
fn link_in_place(at: &Path, target: &Path) -> std::io::Result<()> {
    let staged = at.with_extension("link");
    let _ = std::fs::remove_file(&staged);
    std::os::unix::fs::symlink(target, &staged)?;
    std::fs::rename(&staged, at)?;
    if let Some(dir) = at.parent() {
        std::fs::File::open(dir)?.sync_all()?;
    }
    Ok(())
}

/// The group that may read the token file.
///
/// `mediad` holds the relay and is the one process that needs the token, and `updaterd` runs as
/// root and writes it, so the file is root-owned, `0640`, with this group. Not `robot`: that is the
/// socket group, and `btd`, `padd`, `tofd` and every operator with a shell are in it.
/// `mediad.service` grants this one and `robot.conf` creates it.
pub const TOKEN_GROUP: &str = "robot-account";

/// Give the credential at `path` to [`TOKEN_GROUP`], at `updaterd`'s startup.
///
/// The store sets the group when it saves, and a signed-in robot saves only on a refresh, about a
/// week before a 30-day expiry. Without this, a board signed in before the group changed keeps the
/// old one until then.
pub fn settle_group(path: &Path) {
    match settle(path, TOKEN_GROUP) {
        Ok(true) => tracing::warn!(
            path = %path.display(),
            group = TOKEN_GROUP,
            "gave the account credential to its group"
        ),
        Ok(false) => {}
        Err(e) => tracing::error!(
            path = %path.display(),
            error = %e,
            "could not rewrite the account credential for its group"
        ),
    }
}

/// [`settle_group`] for any path and group. `Ok(true)` when the file now has the group and did not
/// before.
fn settle(path: &Path, group: &str) -> Result<bool, hf_robot_account::Error> {
    use std::os::unix::fs::MetadataExt as _;

    let settled =
        |meta: &std::fs::Metadata, gid: u32| meta.gid() == gid && meta.mode() & 0o777 == 0o640;
    let Some(gid) = crate::unix::group_id(group) else {
        return Ok(false);
    };
    let Ok(before) = std::fs::metadata(path) else {
        return Ok(false);
    };
    if settled(&before, gid) {
        return Ok(false);
    }
    let store = FileStore::at(path).readable_by_group(group);
    let Some(stored) = store.load() else {
        return Ok(false);
    };
    store.save(&stored)?;
    Ok(std::fs::metadata(path).is_ok_and(|after| settled(&after, gid)))
}

/// The account this robot belongs to, at [`DEFAULT_PATH`].
pub fn account() -> Account {
    Account::new(store(DEFAULT_PATH), Config::from_env())
}

/// As [`account`], with the credential somewhere other than a robot's.
///
/// **For a duck in MuJoCo, and it is what makes one signable-in at all.** The device flow, the
/// refresh and the file `mediad` reads are the same code on a laptop as on a board; the one thing
/// a laptop cannot do is write `/etc/robot/hf-token`. So the path moves and nothing else does — a
/// simulated duck signs in to a Hugging Face account by typing a code exactly as a real one does,
/// and appears in the same listing afterwards. `docs/robot/simulation.md` has the walkthrough.
///
/// Distinct from [`account_for_test`], which also redirects the *endpoint*: this one still talks to
/// Hugging Face, because a simulated duck's account is a real account.
pub fn account_at(path: impl Into<std::path::PathBuf>) -> Account {
    Account::new(store(path), Config::from_env())
}

/// As [`account`], against a token file and a Hugging Face of your choosing.
///
/// Only for tests, and it is not optional there: the real path is a robot's real credential, and
/// a test that ran against it would read — or on a developer's machine try to write — that.
#[doc(hidden)]
pub fn account_for_test(path: impl Into<std::path::PathBuf>, endpoint: String) -> Account {
    Account::new(store(path), Config::at(endpoint))
}

fn store(path: impl Into<std::path::PathBuf>) -> FileStore {
    FileStore::at(path).readable_by_group(TOKEN_GROUP)
}

// ── onto the wire ────────────────────────────────────────────────────────────

/// The answer to `account.login`.
pub fn login_result(code: LoginCode) -> proto::AccountLoginResult {
    proto::AccountLoginResult {
        user_code: code.user_code,
        verification_uri: code.verification_uri,
        verification_uri_complete: code.verification_uri_complete,
        expires_in: code.expires_in,
        interval: code.interval,
    }
}

/// The answer to `account.status`.
pub fn status_result(status: Status) -> proto::AccountStatusResult {
    proto::AccountStatusResult {
        account: status.account.map(identity),
        login: status.login.map(login_result),
        last_error: status.last_error,
    }
}

fn identity(who: Identity) -> proto::Account {
    proto::Account {
        username: who.username,
        token_expires_in: who.token_expires_in,
        refreshable: who.refreshable,
    }
}

/// The answer to `account.logout`.
pub fn logout_result(was: Option<String>) -> proto::AccountLogoutResult {
    proto::AccountLogoutResult { was }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// **The wire is this crate's contract, not the account crate's.**
    ///
    /// The two shapes agree field for field today, which is what makes it tempting to serialise
    /// the crate's struct straight onto the wire. This test is the reason not to: it pins the
    /// JSON a client actually receives, so a field renamed upstream shows up here — as a compile
    /// error in the mapping above, or failing that as this test — rather than as an `API_VERSION`
    /// that did not move and an answer that changed anyway.
    #[test]
    fn a_status_serialises_the_way_the_protocol_says() {
        let status = status_result(Status {
            account: Some(Identity {
                username: "PierreRouanet".into(),
                token_expires_in: 2_591_999,
                refreshable: true,
            }),
            login: Some(LoginCode {
                user_code: "A6MY-0314".into(),
                verification_uri: "https://hf.co/oauth/device".into(),
                verification_uri_complete: "https://hf.co/oauth/device".into(),
                expires_in: 240,
                interval: 5,
            }),
            last_error: None,
        });

        assert_eq!(
            serde_json::to_value(&status).unwrap(),
            serde_json::json!({
                "account": {
                    "username": "PierreRouanet",
                    "token_expires_in": 2_591_999,
                    "refreshable": true,
                },
                "login": {
                    "user_code": "A6MY-0314",
                    "verification_uri": "https://hf.co/oauth/device",
                    "verification_uri_complete": "https://hf.co/oauth/device",
                    "expires_in": 240,
                    "interval": 5,
                },
                "last_error": null,
            })
        );
    }

    /// A refusal keeps its code, and a login that cannot say who owns the robot still refuses.
    #[test]
    fn errors_keep_the_code_a_client_branches_on() {
        use hf_robot_account::Error as Hf;

        let already: crate::Error = Hf::AlreadySignedIn("PierreRouanet".into()).into();
        assert_eq!(already.code(), proto::code::INVALID_PARAMS);
        assert!(
            already.to_string().contains("--force"),
            "the daemon's message names the flag, which the crate cannot know: {already}"
        );

        let busy: crate::Error = Hf::LoginInFlight.into();
        assert_eq!(busy.code(), proto::code::BUSY);

        let down: crate::Error = Hf::Network("POST /oauth/device: timed out".into()).into();
        assert_eq!(down.code(), proto::code::NETWORK);
    }

    /// Write `text` at `path`, dated `secs` after the epoch, so which credential is newer is the
    /// test's to say rather than the clock's.
    fn token_at(path: &Path, text: &str, secs: u64) {
        std::fs::write(path, text).unwrap();
        std::fs::File::options()
            .write(true)
            .open(path)
            .unwrap()
            .set_modified(std::time::UNIX_EPOCH + std::time::Duration::from_secs(secs))
            .unwrap();
    }

    /// The release that moved the path must not sign a robot out: the credential moves, with
    /// its mode, and the old path becomes a link to it — what an older release reads after a
    /// rollback.
    #[test]
    fn a_legacy_credential_moves_with_its_mode_and_leaves_a_link() {
        use std::os::unix::fs::PermissionsExt as _;

        let dir = tempfile::tempdir().unwrap();
        let (from, to) = (dir.path().join("etc-token"), dir.path().join("token"));
        std::fs::write(&from, r#"{"access_token":"hf_abc"}"#).unwrap();
        std::fs::set_permissions(&from, std::fs::Permissions::from_mode(0o640)).unwrap();

        assert!(migrate(&from, &to).unwrap());
        assert_eq!(
            std::fs::read_to_string(&to).unwrap(),
            r#"{"access_token":"hf_abc"}"#
        );
        assert_eq!(
            std::fs::metadata(&to).unwrap().permissions().mode() & 0o777,
            0o640
        );
        assert!(
            std::fs::symlink_metadata(&from)
                .unwrap()
                .file_type()
                .is_symlink()
        );
        assert_eq!(
            std::fs::read_to_string(&from).unwrap(),
            r#"{"access_token":"hf_abc"}"#,
            "a release that reads the old path still finds the credential"
        );
        assert!(
            !migrate(&from, &to).unwrap(),
            "and the next start has nothing to do"
        );
    }

    /// A logout deletes the credential, and the link must not keep a copy of it alive: it
    /// dangles, which an older release reads as signed out.
    #[test]
    fn after_a_logout_the_link_reads_as_signed_out() {
        let dir = tempfile::tempdir().unwrap();
        let (from, to) = (dir.path().join("etc-token"), dir.path().join("token"));
        std::fs::write(&from, "token").unwrap();
        migrate(&from, &to).unwrap();

        std::fs::remove_file(&to).unwrap();
        let read = std::fs::read_to_string(&from).unwrap_err();
        assert_eq!(read.kind(), std::io::ErrorKind::NotFound);
        assert!(
            !migrate(&from, &to).unwrap(),
            "a dangling link is not a credential"
        );
    }

    /// A credential at the new path at least as recent as the old one is the current sign-in:
    /// it stays, and the old file only becomes the link.
    #[test]
    fn a_current_credential_wins_over_an_older_legacy_one() {
        let dir = tempfile::tempdir().unwrap();
        let (from, to) = (dir.path().join("etc-token"), dir.path().join("token"));
        token_at(&from, "old", 1_000);
        token_at(&to, "new", 2_000);

        assert!(!migrate(&from, &to).unwrap());
        assert_eq!(std::fs::read_to_string(&to).unwrap(), "new");
        assert_eq!(
            std::fs::read_to_string(&from).unwrap(),
            "new",
            "through the link"
        );
    }

    /// The constant, the unit that grants the group and the file that creates it name one group,
    /// and it is not the socket group.
    #[test]
    fn the_token_group_is_the_one_the_unit_and_sysusers_name() {
        let unit = include_str!("../../mediad/systemd/mediad.service");
        let granted = unit
            .lines()
            .find_map(|l| l.strip_prefix("SupplementaryGroups="))
            .expect("mediad.service names its groups");
        assert!(
            granted.split_whitespace().any(|g| g == TOKEN_GROUP),
            "mediad.service does not grant {TOKEN_GROUP}: {granted}"
        );

        let sysusers = include_str!("../systemd/sysusers.d/robot.conf");
        let created = format!("g {TOKEN_GROUP} -");
        assert!(
            sysusers.lines().any(|l| l.trim() == created),
            "robot.conf does not create {TOKEN_GROUP}"
        );

        assert_ne!(TOKEN_GROUP, "robot");
    }

    const CREDENTIAL: &str = r#"{"access_token":"hf_abc","username":"duck"}"#;

    fn group_name(gid: u32) -> Option<String> {
        // SAFETY: `getgrgid` returns a pointer into a static buffer, or null. The name is copied
        // out before anything else can call into the group database.
        let entry = unsafe { libc::getgrgid(gid) };
        if entry.is_null() {
            return None;
        }
        let name = unsafe { std::ffi::CStr::from_ptr((*entry).gr_name) };
        Some(name.to_string_lossy().into_owned())
    }

    /// The groups this process may give a file to without being root.
    fn my_groups() -> Vec<u32> {
        let mut gids = vec![0 as libc::gid_t; 256];
        // SAFETY: the buffer holds `gids.len()` entries and `getgroups` writes at most that many.
        let n = unsafe { libc::getgroups(gids.len() as libc::c_int, gids.as_mut_ptr()) };
        gids.truncate(n.max(0) as usize);
        gids
    }

    /// A board signed in before the group changed: the file has the old group, and startup gives
    /// it the new one without waiting for a refresh.
    #[test]
    fn a_credential_with_the_old_group_gets_the_new_one_at_startup() {
        use std::os::unix::fs::{MetadataExt as _, PermissionsExt as _};

        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("token");
        std::fs::write(&path, CREDENTIAL).unwrap();
        std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o640)).unwrap();
        let old = std::fs::metadata(&path).unwrap().gid();
        let Some((new, name)) = my_groups()
            .into_iter()
            .filter(|&g| g != old)
            .find_map(|g| group_name(g).map(|n| (g, n)))
        else {
            eprintln!("this process is in one group only; nothing to move the file to");
            return;
        };

        assert!(settle(&path, &name).unwrap());
        let meta = std::fs::metadata(&path).unwrap();
        assert_eq!(meta.gid(), new);
        assert_eq!(meta.mode() & 0o777, 0o640);
        assert_eq!(
            hf_robot_account::read_access_token(&path).as_deref(),
            Some("hf_abc"),
            "the sign-in survives the rewrite"
        );
        assert!(
            !settle(&path, &name).unwrap(),
            "and the next start writes nothing"
        );
    }

    /// A save whose chown failed left the file private; the next start relaxes it.
    #[test]
    fn a_private_credential_is_relaxed_for_its_group() {
        use std::os::unix::fs::{MetadataExt as _, PermissionsExt as _};

        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("token");
        std::fs::write(&path, CREDENTIAL).unwrap();
        std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o600)).unwrap();
        let name = group_name(std::fs::metadata(&path).unwrap().gid()).unwrap();

        assert!(settle(&path, &name).unwrap());
        assert_eq!(std::fs::metadata(&path).unwrap().mode() & 0o777, 0o640);
    }

    /// No group on this system, or no credential: nothing is written.
    #[test]
    fn nothing_is_written_without_the_group_or_the_file() {
        use std::os::unix::fs::{MetadataExt as _, PermissionsExt as _};

        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("token");
        std::fs::write(&path, CREDENTIAL).unwrap();
        std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o600)).unwrap();
        let inode = std::fs::metadata(&path).unwrap().ino();

        assert!(!settle(&path, "no-such-group-exists-here").unwrap());
        assert_eq!(std::fs::metadata(&path).unwrap().ino(), inode);

        let mine = group_name(std::fs::metadata(&path).unwrap().gid()).unwrap();
        assert!(!settle(&dir.path().join("absent"), &mine).unwrap());
    }

    /// Rolled back, an older `updaterd` signed in again: its rename replaced the link with a
    /// file, newer than the credential at the new path. Moving forward again, that sign-in is
    /// the one kept.
    #[test]
    fn a_sign_in_made_while_rolled_back_is_kept() {
        let dir = tempfile::tempdir().unwrap();
        let (from, to) = (dir.path().join("etc-token"), dir.path().join("token"));
        token_at(&to, "before the rollback", 1_000);
        token_at(&from, "signed in while rolled back", 2_000);

        assert!(migrate(&from, &to).unwrap());
        assert_eq!(
            std::fs::read_to_string(&to).unwrap(),
            "signed in while rolled back"
        );
        assert!(
            std::fs::symlink_metadata(&from)
                .unwrap()
                .file_type()
                .is_symlink()
        );
    }
}
