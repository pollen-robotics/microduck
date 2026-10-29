//! Local raw maintenance contract. These operations never cross a relay transport.
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Pid {
    pub p: u16,
    pub i: u16,
    pub d: u16,
}

/// Native position-mode register readback, not desired controller settings.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Motor {
    pub joint: String,
    pub id: u8,
    pub observed_at_ns: u64,
    pub model_number: Option<u16>,
    pub firmware_version: Option<u8>,
    pub hardware_error: Option<u8>,
    pub voltage_v: Option<f64>,
    pub temperature_c: Option<u8>,
    pub current_limit: Option<u16>,
    pub pwm_limit: Option<u16>,
    pub velocity_limit: Option<u32>,
    pub shutdown: Option<u8>,
    pub torque: Option<bool>,
    pub position_counts: Option<i32>,
    pub goal_rad: Option<f64>,
    pub pid: Option<Pid>,
    pub homing_offset_counts: Option<i32>,
    pub expected_homing_offset_counts: i32,
    pub before_startup_fix: Option<i32>,
    pub operating_mode: Option<u8>,
    pub drive_mode: Option<u8>,
    pub min_position_rad: Option<f64>,
    pub max_position_rad: Option<f64>,
    pub profile_velocity: Option<u32>,
    pub profile_acceleration: Option<u32>,
    pub error: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Acquire {
    pub request_id: u64,
    pub mode: String,
    pub acknowledge_raw: bool,
}
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Session {
    pub session_id: u64,
    pub request_id: u64,
}
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Torque {
    pub session_id: u64,
    pub request_id: u64,
    pub joints: Vec<String>,
    pub on: bool,
}
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Gains {
    pub session_id: u64,
    pub request_id: u64,
    pub gains: BTreeMap<String, Pid>,
}
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Command {
    pub session_id: u64,
    pub request_id: u64,
    pub positions_rad: BTreeMap<String, f64>,
    pub valid_for_ms: u32,
}
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Keepalive {
    pub session_id: u64,
    pub sequence: u64,
}
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Cancel {
    pub request_id: u64,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Call {
    Capabilities,
    Inspect,
    Status,
    Refresh(Cancel),
    Acquire(Acquire),
    Torque(Torque),
    Gains(Gains),
    Command(Command),
    Keepalive(Keepalive),
    Stop(Session),
    Release(Session),
    Cancel(Cancel),
}
impl Call {
    pub fn mutating(&self) -> bool {
        !matches!(
            self,
            Self::Capabilities | Self::Inspect | Self::Status | Self::Refresh(_)
        )
    }
    pub fn method(&self) -> &'static str {
        match self {
            Self::Capabilities => "robot.diagnostics.capabilities",
            Self::Inspect => "robot.diagnostics.inspect",
            Self::Status => "robot.diagnostics.status",
            Self::Refresh(_) => "robot.diagnostics.refresh",
            Self::Acquire(_) => "robot.diagnostics.acquire",
            Self::Torque(_) => "robot.diagnostics.torque",
            Self::Gains(_) => "robot.diagnostics.gains",
            Self::Command(_) => "robot.diagnostics.command",
            Self::Keepalive(_) => "robot.diagnostics.keepalive",
            Self::Stop(_) => "robot.diagnostics.stop",
            Self::Release(_) => "robot.diagnostics.release",
            Self::Cancel(_) => "robot.diagnostics.cancel",
        }
    }
    pub fn params(&self) -> Option<serde_json::Value> {
        use serde_json::to_value;
        Some(
            match self {
                Self::Capabilities | Self::Inspect | Self::Status => return None,
                Self::Refresh(p) | Self::Cancel(p) => to_value(p),
                Self::Acquire(p) => to_value(p),
                Self::Torque(p) => to_value(p),
                Self::Gains(p) => to_value(p),
                Self::Command(p) => to_value(p),
                Self::Keepalive(p) => to_value(p),
                Self::Stop(p) | Self::Release(p) => to_value(p),
            }
            .expect("finite diagnostic parameters"),
        )
    }
    pub fn parse(method: &str, params: Option<&serde_json::Value>) -> Result<Self, crate::Error> {
        fn decode<T: serde::de::DeserializeOwned>(
            v: Option<&serde_json::Value>,
        ) -> Result<T, crate::Error> {
            serde_json::from_value(v.cloned().unwrap_or(serde_json::json!({})))
                .map_err(|e| crate::Error::new(crate::code::INVALID_PARAMS, e.to_string()))
        }
        Ok(match method {
            "robot.diagnostics.capabilities" => Self::Capabilities,
            "robot.diagnostics.inspect" => Self::Inspect,
            "robot.diagnostics.status" => Self::Status,
            "robot.diagnostics.refresh" => Self::Refresh(decode(params)?),
            "robot.diagnostics.acquire" => Self::Acquire(decode(params)?),
            "robot.diagnostics.torque" => Self::Torque(decode(params)?),
            "robot.diagnostics.gains" => Self::Gains(decode(params)?),
            "robot.diagnostics.command" => Self::Command(decode(params)?),
            "robot.diagnostics.keepalive" => Self::Keepalive(decode(params)?),
            "robot.diagnostics.stop" => Self::Stop(decode(params)?),
            "robot.diagnostics.release" => Self::Release(decode(params)?),
            "robot.diagnostics.cancel" => Self::Cancel(decode(params)?),
            _ => {
                return Err(crate::Error::new(
                    crate::code::METHOD_NOT_FOUND,
                    "unknown diagnostic method",
                ));
            }
        })
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Outcome {
    pub joint: String,
    pub status: String,
    pub error: Option<String>,
}
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Reply {
    pub accepted: bool,
    pub reason: Option<String>,
    pub session_id: Option<u64>,
    pub sequence: u64,
    pub committed_at_ns: u64,
    pub commit_tick: u64,
    pub lease_ms: u64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub snapshot: Option<Vec<Motor>>,
    pub outcomes: Vec<Outcome>,
}
impl Reply {
    pub fn refused(reason: impl Into<String>) -> Self {
        Self {
            accepted: false,
            reason: Some(reason.into()),
            session_id: None,
            sequence: 0,
            committed_at_ns: 0,
            commit_tick: 0,
            lease_ms: 0,
            snapshot: None,
            outcomes: vec![],
        }
    }
}
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Status {
    pub exit_pending: bool,
    pub mode: Option<String>,
    pub session_id: Option<u64>,
    pub lease_remaining_ms: u64,
    pub controlled: Vec<String>,
    pub stop_unconfirmed: Vec<String>,
    pub motors: Vec<Motor>,
    pub last_reply: Option<Reply>,
    pub pickup_enforced: Option<bool>,
    pub snapshot_at_ns: u64,
    pub command_deadlines_ms: BTreeMap<String, u64>,
    pub sent_targets_rad: BTreeMap<String, f64>,
    pub last_command: Option<Command>,
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn diagnostic_calls_roundtrip_and_reject_unknown_fields() {
        let calls = [
            Call::Capabilities,
            Call::Inspect,
            Call::Status,
            Call::Refresh(Cancel { request_id: 1 }),
            Call::Acquire(Acquire {
                request_id: 1,
                mode: "maintenance_raw".into(),
                acknowledge_raw: true,
            }),
            Call::Torque(Torque {
                session_id: 1,
                request_id: 2,
                joints: vec!["mouth".into()],
                on: true,
            }),
            Call::Gains(Gains {
                session_id: 1,
                request_id: 3,
                gains: BTreeMap::from([("mouth".into(), Pid { p: 100, i: 0, d: 0 })]),
            }),
            Call::Command(Command {
                session_id: 1,
                request_id: 4,
                positions_rad: BTreeMap::from([("mouth".into(), 0.0)]),
                valid_for_ms: 100,
            }),
            Call::Keepalive(Keepalive {
                session_id: 1,
                sequence: 1,
            }),
            Call::Stop(Session {
                session_id: 1,
                request_id: 5,
            }),
            Call::Release(Session {
                session_id: 1,
                request_id: 6,
            }),
            Call::Cancel(Cancel { request_id: 1 }),
        ];
        for c in calls {
            let outer = crate::Call::RobotDiagnostics(c.clone());
            assert_eq!(
                crate::Request::call(crate::Id::Number(1), &outer)
                    .as_call()
                    .unwrap(),
                outer
            );
            if let Some(mut p) = c.params() {
                p.as_object_mut()
                    .unwrap()
                    .insert("unknown".into(), true.into());
                assert!(Call::parse(c.method(), Some(&p)).is_err());
            }
        }
    }
}
