//! Versioned NDJSON envelope for Noise handshake and encrypted transport frames.
//!
//! One envelope is one existing BLE line. The payload is base64 so no ciphertext byte can
//! become a newline delimiter. The receiver must reject all `Data` until key confirmation,
//! and all plaintext JSON-RPC once a secure session starts.

use base64::Engine as _;
use base64::engine::general_purpose::STANDARD_NO_PAD;
use serde::{Deserialize, Serialize};

/// Existing `btd` accepts at most 8 KiB per inbound line, including this envelope.
pub const MAX_CLIENT_LINE: usize = 8 * 1024;

/// Messages sent from the robot are allowed to be larger than requests.
pub const MAX_ROBOT_LINE: usize = 64 * 1024;

/// The maximum ciphertext that fits in one client-to-robot envelope.
pub const MAX_CLIENT_CIPHERTEXT: usize = 6000;

/// One authenticated-session wire message.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Frame {
    Start(Vec<u8>),
    Challenge(Vec<u8>),
    Confirm(Vec<u8>),
    Ready(Vec<u8>),
    Data(Vec<u8>),
}

/// An invalid or oversized envelope. The caller must close the BLE session.
#[derive(Debug)]
pub enum Error {
    Json(serde_json::Error),
    Base64(base64::DecodeError),
    TooLarge,
}

impl From<serde_json::Error> for Error {
    fn from(error: serde_json::Error) -> Self {
        Self::Json(error)
    }
}

impl From<base64::DecodeError> for Error {
    fn from(error: base64::DecodeError) -> Self {
        Self::Base64(error)
    }
}

#[derive(Serialize, Deserialize)]
#[serde(
    tag = "kind",
    content = "data",
    rename_all = "snake_case",
    deny_unknown_fields
)]
enum Envelope {
    Start(String),
    Challenge(String),
    Confirm(String),
    Ready(String),
    Data(String),
}

impl Frame {
    /// Encode a complete frame as one NDJSON line, without the trailing newline.
    pub fn encode(&self, limit: usize) -> Result<String, Error> {
        let (kind, bytes) = match self {
            Self::Start(bytes) => (0, bytes),
            Self::Challenge(bytes) => (1, bytes),
            Self::Confirm(bytes) => (2, bytes),
            Self::Ready(bytes) => (3, bytes),
            Self::Data(bytes) => (4, bytes),
        };
        let data = STANDARD_NO_PAD.encode(bytes);
        let envelope = match kind {
            0 => Envelope::Start(data),
            1 => Envelope::Challenge(data),
            2 => Envelope::Confirm(data),
            3 => Envelope::Ready(data),
            _ => Envelope::Data(data),
        };
        let line = serde_json::to_string(&envelope)?;
        if line.len() + 1 > limit {
            return Err(Error::TooLarge);
        }
        Ok(line)
    }

    /// Decode a complete line. Reject oversized input before parsing or allocating decoded data.
    pub fn decode(line: &str, limit: usize) -> Result<Self, Error> {
        if line.len() + 1 > limit {
            return Err(Error::TooLarge);
        }
        let envelope: Envelope = serde_json::from_str(line)?;
        let (kind, data) = match envelope {
            Envelope::Start(data) => (0, data),
            Envelope::Challenge(data) => (1, data),
            Envelope::Confirm(data) => (2, data),
            Envelope::Ready(data) => (3, data),
            Envelope::Data(data) => (4, data),
        };
        let bytes = STANDARD_NO_PAD.decode(data)?;
        Ok(match kind {
            0 => Self::Start(bytes),
            1 => Self::Challenge(bytes),
            2 => Self::Confirm(bytes),
            3 => Self::Ready(bytes),
            _ => Self::Data(bytes),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn binary_ciphertext_is_safe_in_ndjson() {
        let frame = Frame::Data(vec![0, 10, 13, 255]);
        let line = frame.encode(MAX_CLIENT_LINE).unwrap();
        assert!(!line.contains('\n'));
        assert_eq!(Frame::decode(&line, MAX_CLIENT_LINE).unwrap(), frame);
    }

    #[test]
    fn rejects_plaintext_and_oversized_frames() {
        assert!(Frame::decode(r#"{"method":"net.connect"}"#, MAX_CLIENT_LINE).is_err());
        assert!(
            Frame::Data(vec![0; MAX_CLIENT_LINE])
                .encode(MAX_CLIENT_LINE)
                .is_err()
        );
        let line = Frame::Data(vec![1; 20]).encode(MAX_CLIENT_LINE).unwrap();
        assert!(Frame::decode(&line, 5).is_err());
    }
}
