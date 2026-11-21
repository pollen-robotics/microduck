//! Candidate BLE authentication shared by phone and robot.
//!
//! This crate is not wired into either endpoint. Its API keeps application data unavailable
//! until both peers prove knowledge of the per-robot PSK. A production integration needs a
//! provisioned secret, protected phone storage, and an independent protocol review.

use snow::{Builder, HandshakeState, TransportState};
use std::path::Path;

pub mod tcp;
pub mod wire;

const PATTERN: &str = "Noise_NNpsk0_25519_ChaChaPoly_SHA256";
const CLIENT_CONFIRM: &[u8] = b"microduck client confirmed v1";
const ROBOT_CONFIRM: &[u8] = b"microduck robot confirmed v1";
const MAX_MESSAGE: usize = 65_535;

/// A per-robot development credential loaded from a protected file.
#[derive(Clone)]
pub struct ProvisionedKey {
    robot_id: String,
    key: [u8; 32],
}

impl ProvisionedKey {
    /// Load a 64-character hexadecimal key. The caller must keep the source file private.
    pub fn load(path: &Path, robot_id: String) -> Result<Self, &'static str> {
        if robot_id.is_empty() || robot_id.len() > 128 || !robot_id.is_ascii() {
            return Err("invalid robot identifier");
        }
        let text = std::fs::read_to_string(path).map_err(|_| "cannot read BLE credential")?;
        let text = text.trim_end_matches(['\r', '\n']);
        if text.len() != 64 || !text.is_ascii() {
            return Err("BLE credential must be 64 hexadecimal characters");
        }
        let mut key = [0u8; 32];
        for (index, byte) in key.iter_mut().enumerate() {
            *byte = u8::from_str_radix(&text[index * 2..index * 2 + 2], 16)
                .map_err(|_| "BLE credential must be hexadecimal")?;
        }
        Ok(Self { key, robot_id })
    }

    /// Borrow the secret for an authenticated handshake.
    pub fn key(&self) -> &[u8; 32] {
        &self.key
    }
    /// Borrow the public identifier bound into the handshake.
    pub fn robot_id(&self) -> &str {
        &self.robot_id
    }
}

/// Authentication or frame validation failure. The caller must discard the BLE session.
#[derive(Debug)]
pub enum Error {
    Noise(snow::Error),
    InvalidConfirmation,
    InvalidRobotId,
    FrameTooLarge,
    Closed,
}

impl From<snow::Error> for Error {
    fn from(value: snow::Error) -> Self {
        Self::Noise(value)
    }
}

fn handshake(psk: &[u8; 32], robot_id: &str, initiator: bool) -> Result<HandshakeState, Error> {
    if robot_id.is_empty() || robot_id.len() > 128 || !robot_id.is_ascii() {
        return Err(Error::InvalidRobotId);
    }
    let prologue = format!("microduck-ble-auth-v1:{robot_id}");
    let builder = Builder::new(PATTERN.parse()?)
        .psk(0, psk)?
        .prologue(prologue.as_bytes())?;
    Ok(if initiator {
        builder.build_initiator()?
    } else {
        builder.build_responder()?
    })
}

fn write_handshake(state: &mut HandshakeState) -> Result<Vec<u8>, Error> {
    let mut out = vec![0; MAX_MESSAGE];
    let len = state.write_message(&[], &mut out)?;
    out.truncate(len);
    Ok(out)
}

fn read_handshake(state: &mut HandshakeState, message: &[u8]) -> Result<(), Error> {
    let mut payload = vec![0; MAX_MESSAGE];
    let len = state.read_message(message, &mut payload)?;
    if len != 0 {
        return Err(Error::InvalidConfirmation);
    }
    Ok(())
}

fn seal(state: &mut TransportState, plaintext: &[u8]) -> Result<Vec<u8>, Error> {
    if plaintext.len() > MAX_MESSAGE - 16 {
        return Err(Error::FrameTooLarge);
    }
    let mut out = vec![0; plaintext.len() + 16];
    let len = state.write_message(plaintext, &mut out)?;
    out.truncate(len);
    Ok(out)
}

fn open(state: &mut TransportState, ciphertext: &[u8]) -> Result<Vec<u8>, Error> {
    if ciphertext.len() > MAX_MESSAGE {
        return Err(Error::FrameTooLarge);
    }
    let mut out = vec![0; ciphertext.len()];
    let len = state.read_message(ciphertext, &mut out)?;
    out.truncate(len);
    Ok(out)
}

/// The phone's first message and state awaiting the robot's response.
pub struct ClientStart(HandshakeState);

impl ClientStart {
    /// Begin a fresh exchange for the robot identifier printed with the QR secret.
    pub fn begin(psk: &[u8; 32], robot_id: &str) -> Result<(Self, Vec<u8>), Error> {
        let mut state = handshake(psk, robot_id, true)?;
        let first = write_handshake(&mut state)?;
        Ok((Self(state), first))
    }

    /// Verify the robot's Noise response and send encrypted client confirmation.
    pub fn receive(mut self, response: &[u8]) -> Result<(ClientConfirm, Vec<u8>), Error> {
        read_handshake(&mut self.0, response)?;
        let mut transport = self.0.into_transport_mode()?;
        let confirmation = seal(&mut transport, CLIENT_CONFIRM)?;
        Ok((ClientConfirm(transport), confirmation))
    }
}

/// The phone cannot send application RPCs until the robot confirms the key.
pub struct ClientConfirm(TransportState);

impl ClientConfirm {
    /// Verify the robot's encrypted confirmation and open the application session.
    pub fn receive(mut self, response: &[u8]) -> Result<Session, Error> {
        if open(&mut self.0, response)? != ROBOT_CONFIRM {
            return Err(Error::InvalidConfirmation);
        }
        Ok(Session(Some(self.0)))
    }
}

/// The robot's Noise response and state awaiting client confirmation.
pub struct RobotConfirm(TransportState);

impl RobotConfirm {
    /// Process the first phone message using the provisioned per-robot secret.
    pub fn respond(psk: &[u8; 32], robot_id: &str, first: &[u8]) -> Result<(Self, Vec<u8>), Error> {
        let mut state = handshake(psk, robot_id, false)?;
        read_handshake(&mut state, first)?;
        let response = write_handshake(&mut state)?;
        Ok((Self(state.into_transport_mode()?), response))
    }

    /// Verify the phone's encrypted confirmation before opening the robot RPC gate.
    pub fn receive(mut self, confirmation: &[u8]) -> Result<(Session, Vec<u8>), Error> {
        if open(&mut self.0, confirmation)? != CLIENT_CONFIRM {
            return Err(Error::InvalidConfirmation);
        }
        let reply = seal(&mut self.0, ROBOT_CONFIRM)?;
        Ok((Session(Some(self.0)), reply))
    }
}

/// Authenticated and encrypted, with independent Noise counters in each direction.
pub struct Session(Option<TransportState>);

impl Session {
    /// Protect one complete application message. Frame boundaries belong to the BLE transport.
    pub fn seal(&mut self, message: &[u8]) -> Result<Vec<u8>, Error> {
        let state = self.0.as_mut().ok_or(Error::Closed)?;
        if message.len() > MAX_MESSAGE - 16 {
            return Err(Error::FrameTooLarge);
        }
        let result = seal(state, message);
        if result.is_err() {
            self.0 = None;
        }
        result
    }

    /// Authenticate and decrypt one complete application message. Any failure closes this session.
    pub fn open(&mut self, message: &[u8]) -> Result<Vec<u8>, Error> {
        let state = self.0.as_mut().ok_or(Error::Closed)?;
        let result = open(state, message);
        if result.is_err() {
            self.0 = None;
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const KEY: [u8; 32] = [0x42; 32];

    fn connected() -> (Session, Session) {
        let (phone, first) = ClientStart::begin(&KEY, "robot-123").unwrap();
        let (robot, second) = RobotConfirm::respond(&KEY, "robot-123", &first).unwrap();
        let (phone, proof) = phone.receive(&second).unwrap();
        let (robot, reply) = robot.receive(&proof).unwrap();
        (phone.receive(&reply).unwrap(), robot)
    }

    #[test]
    fn both_directions_are_protected_after_confirmation() {
        let (mut phone, mut robot) = connected();
        let wifi = phone
            .seal(br#"{"method":"net.connect","password":"secret"}"#)
            .unwrap();
        assert!(!wifi.windows(6).any(|bytes| bytes == b"secret"));
        assert_eq!(
            robot.open(&wifi).unwrap(),
            br#"{"method":"net.connect","password":"secret"}"#
        );
        let answer = robot.seal(b"joined").unwrap();
        assert_eq!(phone.open(&answer).unwrap(), b"joined");
    }

    #[test]
    fn wrong_secret_and_robot_identity_fail_before_rpc() {
        let (_phone, first) = ClientStart::begin(&KEY, "robot-123").unwrap();
        let wrong_key = [0x43; 32];
        assert!(RobotConfirm::respond(&wrong_key, "robot-123", &first).is_err());

        let (_phone, first) = ClientStart::begin(&KEY, "robot-123").unwrap();
        assert!(RobotConfirm::respond(&KEY, "robot-456", &first).is_err());
    }

    #[test]
    fn replay_and_tampering_fail() {
        let (mut phone, mut robot) = connected();
        let ciphertext = phone.seal(b"move").unwrap();
        assert_eq!(robot.open(&ciphertext).unwrap(), b"move");
        assert!(robot.open(&ciphertext).is_err());
        assert!(matches!(robot.open(&ciphertext), Err(Error::Closed)));

        let (mut phone, mut robot) = connected();
        let mut tampered = phone.seal(b"stop").unwrap();
        tampered[0] ^= 1;
        assert!(robot.open(&tampered).is_err());
        assert!(matches!(robot.open(&tampered), Err(Error::Closed)));
    }
}
