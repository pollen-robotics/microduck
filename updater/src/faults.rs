//! Deliberate failure injection.
//!
//! Rollback is the feature most likely to be quietly broken, because it only
//! runs when something else already went wrong. Making failures injectable turns
//! "rollback presumably works" into a CI assertion
//! (`docs/design/updater-design.md` §16.1).
//!
//! Compiled in unconditionally rather than behind `#[cfg(test)]`, so the same
//! binary that ships can be exercised on a bench robot. Enabling any of these
//! requires an explicit flag that production configs never set.

/// Points at which the engine can be told to fail.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Faults {
    /// Corrupt the artifact after download, before verification. Must produce a
    /// hash mismatch and no install.
    pub corrupt_artifact: bool,
    /// Fail the post-install hook. Must roll back.
    pub fail_post_hook: bool,
    /// Report unhealthy after apply. Must roll back.
    pub fail_health: bool,
    /// Make the health probe hang rather than answer, to prove the timeout works
    /// (socket open, no reply — the case that stalls naive clients).
    pub hang_health: bool,
    /// Abort the process immediately after the symlink swap, to prove a
    /// `kill -9` mid-swap leaves a consistent state.
    pub abort_after_swap: bool,
    /// Pretend the filesystem is full during staging.
    pub simulate_disk_full: bool,
    /// Make rollback itself fail, exercising the worst path: failed update *and*
    /// failed recovery, which must be reported loudly rather than silently.
    pub fail_rollback: bool,
}

impl Faults {
    pub fn none() -> Self {
        Self::default()
    }

    pub fn any_enabled(&self) -> bool {
        *self != Self::default()
    }

    /// Parse `--inject-fault` names, refusing unless the config permits it.
    ///
    /// Fails closed on an unknown name too: silently ignoring a typo would make a
    /// test look like it passed when the fault it meant to inject never fired.
    pub fn from_names(names: &[String], allowed: bool) -> Result<Self, crate::Error> {
        if names.is_empty() {
            return Ok(Self::none());
        }
        if !allowed {
            return Err(crate::Error::Config(
                "fault injection requested but `allow_fault_injection` is not set in the \
                 config; refusing (this must never be enabled on a client robot)"
                    .into(),
            ));
        }

        let mut faults = Self::none();
        for name in names {
            match name.as_str() {
                "corrupt_artifact" => faults.corrupt_artifact = true,
                "fail_post_hook" => faults.fail_post_hook = true,
                "fail_health" => faults.fail_health = true,
                "hang_health" => faults.hang_health = true,
                "abort_after_swap" => faults.abort_after_swap = true,
                "simulate_disk_full" => faults.simulate_disk_full = true,
                "fail_rollback" => faults.fail_rollback = true,
                other => {
                    return Err(crate::Error::Config(format!(
                        "unknown fault {other:?}; valid: corrupt_artifact, fail_post_hook, \
                         fail_health, hang_health, abort_after_swap, simulate_disk_full, \
                         fail_rollback"
                    )));
                }
            }
        }
        Ok(faults)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_faults_needs_no_permission() {
        assert!(Faults::from_names(&[], false).is_ok());
    }

    /// The gate that makes compiling faults into the shipped binary acceptable.
    #[test]
    fn injection_is_refused_without_config_permission() {
        let err = Faults::from_names(&["fail_health".into()], false).unwrap_err();
        assert!(err.to_string().contains("allow_fault_injection"), "{err}");
    }

    #[test]
    fn permitted_injection_parses() {
        let faults = Faults::from_names(&["fail_health".into()], true).unwrap();
        assert!(faults.fail_health);
        assert!(faults.any_enabled());
    }

    /// A typo must fail loudly: a silently-ignored fault makes a test look like it
    /// passed when nothing was injected.
    #[test]
    fn unknown_fault_is_rejected() {
        assert!(Faults::from_names(&["fail_helth".into()], true).is_err());
    }
}
