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

use crate::proto;

pub use hf_robot_account::{Account, maintain};

/// Where the credential lives.
///
/// **Not in `robotd.toml`.** Every mechanism that exists for that file is wrong for a secret:
/// `robotctl configure --list` prints what a robot changes, the config editor shows the whole
/// file, and "what was changed on this robot" is a report we generate. A bearer token would be in
/// all three.
pub const DEFAULT_PATH: &str = "/etc/robot/hf-token";

/// The group that may read the token file.
///
/// `mediad` runs as `User=mediad` with `SupplementaryGroups=robot`, and it is the process that
/// needs the token — it is the one holding the relay. `updaterd` runs as root and writes it. So
/// the file is `root:robot` and `0640`: readable by the daemons that belong to this robot, and by
/// nothing else with a login on the board.
pub const TOKEN_GROUP: &str = "robot";

/// The account this robot belongs to, at [`DEFAULT_PATH`].
pub fn account() -> Account {
    Account::new(store(DEFAULT_PATH), Config::from_env())
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
}
