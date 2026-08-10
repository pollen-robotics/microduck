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

pub mod config;
pub mod engine;
pub mod faults;
pub mod fsutil;
pub mod hooks;
pub mod ipc;
pub mod journal;
pub mod manifest;
pub mod preflight;
/// The IPC contract, re-exported from the [`duck_ipc_proto`] crate. Re-exported under this
/// path so the engine's own code and `updater::proto::*` users need not care where it lives.
pub use duck_ipc_proto as proto;
pub mod robot;
pub mod source;
pub mod store;
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

    #[error("another update is already in progress")]
    Busy,

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
            Error::WouldDowngrade { .. } => code::WOULD_DOWNGRADE,
            Error::Busy => code::BUSY,
            Error::Network(_) => code::NETWORK,
            Error::Verification(_) => code::VERIFICATION_FAILED,
            Error::Incompatible(_) => code::INCOMPATIBLE,
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
