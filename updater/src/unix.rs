//! Names to numbers: the two lookups this crate needs from the user database.
//!
//! Everything in this crate that cares about identity is configured by **name** —
//! `deploy/updater.toml` names `btd` and `mediad` in `allow_users`, and the account token is
//! group-owned by `robot` — because `systemd-sysusers` allocates dynamically and a number written
//! down is right on one board and wrong on the next. `SO_PEERCRED` and `chown(2)` deal in
//! numbers, so the translation has to happen somewhere, and doing it here means one SAFETY
//! argument rather than one per caller.
//!
//! Not shared with `configd`, `padd` and `tof`, which have their own copies: the obvious common
//! home is `duck-ipc-proto`, and that crate is types only — every service speaks it, including
//! the ones on the recovery path, so it may not grow a libc dependency for the convenience of a
//! few callers. Within *this* crate there is no such excuse, which is why the lib and the binary
//! share these rather than each keeping a copy.

/// The uid of a user by name, or `None` if there is no such user.
pub fn user_id(name: &str) -> Option<u32> {
    let cname = std::ffi::CString::new(name).ok()?;
    // SAFETY: `getpwnam` takes a NUL-terminated string and returns a pointer into a static
    // buffer, or null. The one field we need is read immediately and nothing is retained, so
    // the next caller overwriting that buffer cannot be observed here.
    let entry = unsafe { libc::getpwnam(cname.as_ptr()) };
    if entry.is_null() {
        return None;
    }
    Some(unsafe { (*entry).pw_uid })
}

/// The gid of a group by name, or `None` if there is no such group.
pub fn group_id(name: &str) -> Option<u32> {
    let cname = std::ffi::CString::new(name).ok()?;
    // SAFETY: as [`user_id`], for the group database.
    let entry = unsafe { libc::getgrnam(cname.as_ptr()) };
    if entry.is_null() {
        return None;
    }
    Some(unsafe { (*entry).gr_gid })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `root` and `root`/`wheel` are the only names portable enough to assert on, and the
    /// property worth pinning is the other half anyway: a name that does not exist is `None`
    /// rather than a panic or a zero — `0` is root, so a lookup that returned it on failure
    /// would hand the wrong caller everything.
    #[test]
    fn a_name_that_does_not_exist_is_none() {
        assert_eq!(user_id("no-such-user-exists-here"), None);
        assert_eq!(group_id("no-such-group-exists-here"), None);
        assert_eq!(user_id("root\0with-a-nul"), None, "a NUL is not a name");
        assert_eq!(user_id("root"), Some(0), "root is uid 0 everywhere");
    }
}
