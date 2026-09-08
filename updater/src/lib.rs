//! Config-driven update engine.
//!
//! The engine is robot-agnostic; everything robot-specific lives in
//! [`config::Config`]. Adapting to another robot should mean a new config file,
//! new signing keys, and possibly a new health probe — not a fork of this crate.
//! See `docs/design/updater-design.md` §10.
//!
//! Design docs: [`updater-design.md`] for the update system, [`architecture.md`]
//! for the surrounding services.
//!
//! [`updater-design.md`]: ../../../docs/design/updater-design.md
//! [`architecture.md`]: ../../../docs/design/architecture.md
//!
//! # Deliberate non-goals
//!
//! - **No premature abstraction over the runtime.** Async (tokio) is used where
//!   the daemon genuinely waits — serving IPC while an update runs, timing out
//!   the health probe and hook subprocesses, cancelling an in-flight update.
//!   CPU-bound and fast-filesystem work ([`store`], [`verify`], [`journal`])
//!   stays synchronous and is called via `spawn_blocking`; making it async would
//!   add noise without buying anything.
//! - **No premature crate splitting.** This was one crate with two binaries until a
//!   second service needed the wire types; then — and only then — [`proto`] moved out
//!   into [`duck_ipc_proto`]. `robotd` and `robotctl` depend on that, not on this crate,
//!   so nothing on the recovery path inherits the engine's http/tar/zstd/crypto tree.
//!   `updaterd` remains this crate's only binary.
//! - **No OS/kernel updates.** Application-level only; see
//!   `updater-design.md` §11.
//! - **No hardware capability matrix.** v1 targets one hardware configuration
//!   (§5.6).

/// Which Hugging Face account this robot belongs to — the credential a remote session needs
/// before it can exist. `docs/design/remote-access-design.md` §2.
pub mod account;
pub mod config;
pub mod engine;
pub mod faults;
pub mod fsutil;
pub mod hooks;
pub mod ipc;
pub mod journal;
pub mod manifest;
pub mod orphan;
pub mod policy;
pub mod preflight;
pub mod reconcile;
/// The IPC contract, re-exported from the [`duck_ipc_proto`] crate. Re-exported under this
/// path so the engine's own code and `updater::proto::*` users need not care where it lives.
pub use duck_ipc_proto as proto;
pub mod robot;
pub mod source;
mod spawn;
pub mod store;
pub mod transcript;
/// Names to numbers, for the two lookups this crate makes against the user database.
pub mod unix;
pub mod verify;

use std::path::PathBuf;

/// Engine failures.
///
/// Each variant maps to a distinct JSON-RPC error code the client can act on,
/// because "update failed" alone is useless to a client and to support.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("unknown component: {0}")]
    UnknownComponent(String),

    /// The component is configured, but that version isn't on disk.
    #[error("{component} has no installed release {version}")]
    NotInstalled {
        component: String,
        version: semver::Version,
    },

    /// Refusing to move to an older version than the one installed.
    #[error("refusing to downgrade from {installed} to {candidate}")]
    WouldDowngrade {
        installed: semver::Version,
        candidate: semver::Version,
    },

    /// `--staging` resolved to a candidate older than what the board is running, which means
    /// the staging channel has nothing newer to offer.
    ///
    /// Distinct from [`Self::WouldDowngrade`] because the operator's next move is different.
    /// That one is a rollback-attack guard: it says a *mirror* may have gone backwards, and
    /// the right response is to distrust the source. This one says the source is fine and the
    /// channel is simply behind — usually because the last releases were promoted straight to
    /// stable and published no candidate. Answering "refusing to downgrade" sent the one
    /// person who hit it looking for a broken mirror.
    ///
    /// Only [`crate::proto::Target::Staging`] reaches it. `StagingExact` is how someone names
    /// an older candidate deliberately, so the message names that command as the way past.
    #[error(
        "the newest release candidate is {candidate}, and this board is already on \
         {installed} — nothing more recent is available on the staging channel. A release \
         promoted straight to stable publishes no candidate, so staging stays at the last \
         version that had one. There is nothing here to test. To install this older candidate \
         anyway, name it:\n  robotctl update apply {component} --staging --version {candidate}"
    )]
    StagingBehind {
        component: String,
        installed: semver::Version,
        candidate: semver::Version,
    },

    /// The candidate does not contain a binary an installed unit execs.
    ///
    /// Carries a preformatted message rather than its parts, because the useful half is the
    /// remedy — see [`crate::orphan::refusal`], which builds and tests it.
    #[error("{0}")]
    WouldOrphanUnit(String),

    #[error("another update is already in progress")]
    Busy,

    /// A second `account.login` while one is still waiting for approval.
    ///
    /// Shares [`code::BUSY`] with the update above — a client should retry or wait either way —
    /// but not its message: two logins collide during setup, when a console page and a phone are
    /// both pointed at the same robot, and telling that person an *update* is in progress sends
    /// them looking in the wrong place. What they need is that a code is already out and where to
    /// read it.
    #[error(
        "a login is already waiting for approval — `account status` has the code, or --force to \
         start a new one"
    )]
    LoginInFlight,

    /// `account.login` on a robot that already belongs to somebody, without `--force`.
    ///
    /// A refusal rather than a silent replacement, because this call is routed to BLE and to a
    /// datachannel: on a LAN, anybody who can reach the robot can start a login, and one that
    /// quietly rebound the robot would turn "was on the wifi" into remote access that outlives
    /// being there. The message names the account and the flag, because the legitimate case —
    /// a robot changing hands — is common enough to deserve the instruction rather than a hunt.
    #[error(
        "this robot already belongs to {0}. Sign that account out first, or pass --force to \
         replace it"
    )]
    AlreadySignedIn(String),

    #[error("config error: {0}")]
    Config(String),

    #[error("{path}: {source}")]
    Io {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },

    #[error("network error: {0}")]
    Network(String),

    /// The release exists but carries no assets, which is what every release looks like until
    /// its build finishes uploading them.
    ///
    /// Distinct from [`Self::Network`] because the old answer — "has no asset named
    /// \"manifest.json\" (has: )" under a "network error" prefix — describes a malformed
    /// release and a broken link, and it is neither. The release is fine and the board is
    /// online; the artifacts are minutes away. Both people who hit it went looking for the
    /// wrong thing.
    #[error(
        "release {tag} has no assets yet, so there is nothing here to install. Its build has \
         not finished uploading them — publishing a release creates it before that build \
         starts, so the first few minutes of every release look like this and nothing is wrong \
         with it. The release page lists the assets as they land:\n  \
         https://github.com/{repo}/releases/tag/{tag}\nRun the same command again once they \
         are there."
    )]
    ReleaseNotReady { repo: String, tag: String },

    /// Signature or hash mismatch. Never retried automatically — a failure here
    /// means the bytes are not ours.
    #[error("verification failed: {0}")]
    Verification(String),

    #[error("incompatible with this robot: {0}")]
    Incompatible(String),

    #[error("preflight check failed: {0}")]
    Preflight(String),

    #[error("hook {hook} failed: {detail}")]
    Hook { hook: String, detail: String },

    /// The replacement `updaterd` could not start.
    ///
    /// Distinct from `HealthCheck` on purpose: the robot is fine, the *release* cannot run the
    /// process that would install the next one. Naming it is the whole value — a rollback reason of
    /// "unreachable" sent three separate investigations down the wrong path.
    #[error("the new updaterd failed its self-test: {0}")]
    SelfTest(String),

    #[error("health check failed: {0}")]
    Health(String),

    /// The update failed *and* the rollback failed. The most serious outcome —
    /// surfaced distinctly so support sees it immediately rather than reading it
    /// as an ordinary failure.
    #[error("rollback failed after a failed update: {0}")]
    RollbackFailed(String),

    /// The artifact verified but expands beyond the configured bounds. Distinct
    /// from `Verification` so it never reads as tampering.
    #[error("artifact exceeds configured archive limits: {0}")]
    ArchiveTooLarge(String),

    /// Asked to show a run this board has no transcript for.
    ///
    /// Carries what it *does* have, because the number was almost certainly a typo or a
    /// guess: a bare "no such run" leaves the caller with nowhere to go next, and the list
    /// is three lines away in the same directory.
    #[error("{}", crate::transcript::no_such_run(*.run, .available, *.earlier))]
    NoSuchRun {
        /// `None` when the caller asked for the most recent run and there is none.
        run: Option<u64>,
        available: Vec<u64>,
        /// Attempts the update log holds with no transcript behind them.
        ///
        /// Every board passes through this state exactly once: the release that added
        /// transcripts cannot have recorded its own installation, because the `updaterd` that
        /// performed it was the one before. Saying "no update has recorded a transcript" to
        /// somebody looking at a log full of updates is true and reads as a broken feature.
        earlier: usize,
    },

    #[error("on-disk state is inconsistent: {0}")]
    Corrupt(String),

    #[error("{0}")]
    Internal(String),
}

impl Error {
    /// JSON-RPC error code for this failure.
    pub fn code(&self) -> i32 {
        use crate::proto::code;
        match self {
            Error::UnknownComponent(_) => code::UNKNOWN_COMPONENT,
            Error::NotInstalled { .. } => code::NOT_INSTALLED,
            // Shares `NOT_INSTALLED` rather than minting a code, for the reason `StagingBehind`
            // and `WouldOrphanUnit` share theirs: it is the same answer in a different noun —
            // the component is known, the specific thing named is not on this board — and what
            // the caller needs is the message, which names the runs that are.
            Error::NoSuchRun { .. } => code::NOT_INSTALLED,
            Error::WouldDowngrade { .. } => code::WOULD_DOWNGRADE,
            // Shares the downgrade code rather than adding one, for the reason `WouldOrphanUnit`
            // shares `INCOMPATIBLE` below: to a client this is the same answer — "refused, the
            // target is older than what is installed" — and what a person needs is the message.
            // A new code would be an `API_VERSION` bump for a refusal no client branches on.
            Error::StagingBehind { .. } => code::WOULD_DOWNGRADE,
            Error::Busy | Error::LoginInFlight => code::BUSY,
            // The request is well-formed and refused because of what it did not say, which is
            // what `INVALID_PARAMS` is: passing `force` is the fix, and it is a parameter.
            Error::AlreadySignedIn(_) => code::INVALID_PARAMS,
            Error::Network(_) => code::NETWORK,
            // Shares the network code rather than adding one, for the reason `StagingBehind`
            // shares the downgrade code above — with a second reason here. To a client this is
            // the same answer as any other fetch that came up empty, and the one behaviour a
            // client should have is the one `NETWORK` already asks for: retry later. That is
            // exactly right for a release whose upload is still in flight, so a new code would
            // be an `API_VERSION` bump that changed no client's mind about what to do.
            Error::ReleaseNotReady { .. } => code::NETWORK,
            Error::Verification(_) => code::VERIFICATION_FAILED,
            Error::Incompatible(_) => code::INCOMPATIBLE,
            // Shares the incompatible code rather than adding one, for the reason `SelfTest`
            // shares health's: to a client this is the same answer — "that release will not run
            // on this board as it stands" — and what distinguishes it is the message. A new code
            // would also be an `API_VERSION` bump for a refusal no client branches on.
            Error::WouldOrphanUnit(_) => code::INCOMPATIBLE,
            Error::ArchiveTooLarge(_) => code::ARCHIVE_TOO_LARGE,
            Error::Preflight(_) => code::PREFLIGHT_FAILED,
            Error::Hook { .. } => code::HOOK_FAILED,
            Error::Health(_) => code::HEALTH_CHECK_FAILED,
            // Shares the health code rather than adding one: to a client this is the same class of
            // answer — "the new release did not pass its checks and was reverted" — and the
            // distinction that matters is in the message, which names the binary and the reason.
            Error::SelfTest(_) => code::HEALTH_CHECK_FAILED,
            Error::RollbackFailed(_) => code::ROLLBACK_FAILED,
            Error::Config(_) | Error::Io { .. } | Error::Corrupt(_) | Error::Internal(_) => {
                code::INTERNAL_ERROR
            }
        }
    }

    /// Convert for the wire.
    pub fn to_rpc_error(&self) -> crate::proto::Error {
        crate::proto::Error::new(self.code(), self.to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Busy must not look like a generic internal error: scripts retry on it.
    #[test]
    fn busy_has_its_own_code() {
        assert_eq!(Error::Busy.code(), crate::proto::code::BUSY);
        assert_ne!(Error::Busy.code(), crate::proto::code::INTERNAL_ERROR);
    }

    /// A tampered artifact must be distinguishable from a network blip, so it is
    /// never retried automatically.
    #[test]
    fn verification_failure_is_distinct_from_network() {
        assert_ne!(
            Error::Verification("bad sig".into()).code(),
            Error::Network("timeout".into()).code()
        );
    }
}
