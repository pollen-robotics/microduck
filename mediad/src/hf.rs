//! The account credential, as the two things in this daemon that need it read it.
//!
//! `updaterd` owns the file — it performs the login, it renews the token, and
//! `updater::account::Store` is what writes it. This is the reading half, and it is deliberately
//! the smallest possible one: the field it needs and nothing else, so a record that grows a field
//! there does not break a parse here. A test in `updater` pins the key, because the writer is
//! what can break the contract.
//!
//! Read on every use rather than cached. A login that happens while this daemon is running has to
//! take effect without a restart — the login arrives over BLE, minutes or months after `mediad`
//! started — and re-reading a small file on a path that already waits thirty seconds costs
//! nothing.

use std::path::Path;

use serde::Deserialize;

/// The access token, or `None` when this robot belongs to nobody.
pub fn access_token(path: &Path) -> Option<String> {
    #[derive(Deserialize)]
    struct Credential {
        access_token: String,
    }

    let bytes = std::fs::read(path).ok()?;
    match serde_json::from_slice::<Credential>(&bytes) {
        Ok(credential) if !credential.access_token.is_empty() => Some(credential.access_token),
        Ok(_) => None,
        Err(e) => {
            tracing::warn!(
                path = %path.display(),
                error = %e,
                "the account credential does not parse; treating this robot as signed out"
            );
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_field_out_of_whatever_updaterd_wrote() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("hf-token");

        assert_eq!(access_token(&path), None, "no file is no account");

        std::fs::write(
            &path,
            r#"{"access_token":"hf_abc","refresh_token":"r","expires_at":1,
                "username":"x","added_later":true}"#,
        )
        .unwrap();
        assert_eq!(access_token(&path).as_deref(), Some("hf_abc"));

        std::fs::write(&path, r#"{"refresh_token":"only"}"#).unwrap();
        assert_eq!(
            access_token(&path),
            None,
            "a record with no token is no use"
        );

        std::fs::write(&path, "{ not json").unwrap();
        assert_eq!(
            access_token(&path),
            None,
            "corrupt is signed out, not fatal"
        );
    }
}
