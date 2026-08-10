//! IPC contracts between the robot's services and their clients.
//!
//! Two namespaces over one wire format:
//!
//!  - `update.*` — `updaterd`'s API, spoken by `robotctl` and later `btd`.
//!  - `robot.*`  — `robotd`'s API. Small on purpose: it is what `updaterd` needs in order
//!    to decide whether an update is safe and whether it worked.
//!
//! **Wire format: JSON-RPC 2.0, one object per line (NDJSON), over a unix socket.**
//! Framing is a single newline. Progress is pushed as a JSON-RPC notification, a message
//! with no `id`, so a client that reconnects mid-update resubscribes and keeps receiving
//! them.
//!
//! ```text
//! → {"jsonrpc":"2.0","id":1,"method":"update.apply","params":{...}}
//! ← {"jsonrpc":"2.0","method":"update.progress","params":{...}}   (no id)
//! ← {"jsonrpc":"2.0","method":"update.progress","params":{...}}
//! ← {"jsonrpc":"2.0","id":1,"result":{...}}
//! ```
//!
//! A method and its parameters are always paired through [`Call`]: build a request with
//! [`Request::call`], read one back with [`Request::as_call`]. There is no way to send a
//! method with another method's parameters.
//!
//! Why JSON-RPC, why a unix socket, and what was measured against both:
//! `docs/design/architecture.md` §2.2.
//!
//! Dependencies stay at serde, serde_json and semver. Every service speaks these types,
//! including the ones on the recovery path, so nothing here may pull in http, tar, crypto
//! or an async runtime.

use serde::{Deserialize, Serialize};
use serde_json::Value;

pub const JSONRPC_VERSION: &str = "2.0";

/// Protocol version, exchanged via [`Call::Hello`].
///
/// Bumped on any incompatible change. A peer speaking a different version is refused
/// rather than misparsed — a stale `robotctl` in someone's shell is normal.
///
/// v2 added `HelloResult::revision`. v3 added the `net.*` and `system.*` namespaces. v4 added
/// `system.authenticate`, which a BLE client must now pass before anything else is served — a v3
/// client would otherwise have every call refused with no idea why. v5 added the `pad.*`
/// namespace, which is additive — a v4 client loses nothing by not knowing it — and bumps anyway,
/// because the version's job is to say "these two peers were not built together". During
/// v6 added `robot.init` and `robot.relax`, so powering the joints stops being a subcommand that
/// fights the daemon for the motor bus. During
/// prototyping the wire shape simply changes and this bumps; no accommodation is made for
/// peers that predate a field, because there are none in the field yet.
pub const API_VERSION: u32 = 6;

pub const DEFAULT_SOCKET: &str = "/run/updaterd.sock";

/// Where each service listens by default.
///
/// These are defaults matching the shipped units, not a contract the daemons are bound by —
/// every one takes a `--socket` override, and `updaterd` reads `robot_socket` from its config.
/// They live here because more than one client needs them: `robotctl` and `btd` both connect
/// to all three, and a path duplicated per client is a path that drifts per client.
pub mod socket {
    /// `updaterd`. Same value as [`super::DEFAULT_SOCKET`], which predates this module.
    pub const UPDATER: &str = super::DEFAULT_SOCKET;
    pub const ROBOT: &str = "/run/robotd.sock";
    pub const CONFIG: &str = "/run/configd.sock";
}

/// The robot's joint order, as every positional vector on the wire is indexed.
///
/// It lives here rather than in `duck_control::model` because it *is* protocol:
/// [`RobotState::joints`] and [`RobotState::targets`] are bare arrays of numbers, and a
/// client that cannot name index 3 cannot display them. `duck-control` re-exports this
/// table, so the wire order and the order the servos are driven in are one list, not two
/// that must be kept in step.
///
/// Left leg (5) · neck/head/mouth (5) · right leg (5).
pub const JOINT_NAMES: [&str; 15] = [
    "left_hip_yaw",
    "left_hip_roll",
    "left_hip_pitch",
    "left_knee",
    "left_ankle",
    "neck_pitch",
    "head_pitch",
    "head_yaw",
    "head_roll",
    "mouth",
    "right_hip_yaw",
    "right_hip_roll",
    "right_hip_pitch",
    "right_knee",
    "right_ankle",
];

/// Method names, as they go on the wire. Namespaced so a new namespace cannot collide
/// with `update.*`. [`Call`] is the typed form.
pub mod method {
    pub const HELLO: &str = "hello";

    pub const CHECK: &str = "update.check";
    pub const APPLY: &str = "update.apply";
    pub const ROLLBACK: &str = "update.rollback";
    pub const RESET_TO_GOLDEN: &str = "update.resetToGolden";
    pub const SELECT: &str = "update.select";
    pub const PIN: &str = "update.pin";
    pub const STATUS: &str = "update.status";
    pub const LIST_INSTALLED: &str = "update.listInstalled";
    pub const LOG: &str = "update.log";
    pub const SUBSCRIBE: &str = "update.subscribe";

    /// Server → client notification. Never carries an `id`.
    pub const PROGRESS: &str = "update.progress";

    // ── robotd's side ────────────────────────────────────────────────────────
    //
    // `updaterd` calls these. Every one must be answerable while the robot is in a bad
    // state — that is the whole point of asking.

    /// May the control loop be restarted right now?
    pub const ROBOT_SAFE_TO_RESTART: &str = "robot.safeToRestart";
    /// Did the robot come up correctly? The post-update health gate.
    pub const ROBOT_HEALTH: &str = "robot.health";
    /// Which model API version does this build implement?
    pub const ROBOT_MODEL_API: &str = "robot.modelApi";
    /// Is a telepresence session live?
    pub const ROBOT_SESSION_ACTIVE: &str = "robot.remoteSessionActive";

    // ── intents ──────────────────────────────────────────────────────────────
    //
    // What a client asks the robot to *do*, as opposed to what `updaterd` asks it about.
    // Clients send intents, never joint commands: `robotd` stays authoritative on what is
    // executable (`architecture.md` §6).
    //
    // Two kinds, and JSON-RPC's two message families map onto them exactly:
    //
    //   * **Continuous** — `move`, `head`. Sent as *notifications* (no `id`, no reply),
    //     20–50 Hz, last-writer-wins, expiring. No response traffic at rate, and when they
    //     later travel over WebRTC they belong on the unreliable channel, because a
    //     retransmitted 80 ms-old stick position is worse than useless (`architecture.md`
    //     §5.2). The message family already says which channel it wants.
    //   * **Discrete** — `stop`, `enable`. Sent as *requests*, answered, because the caller
    //     needs to know whether it was accepted and why not.

    /// Velocity twist. Continuous; send as a notification.
    pub const ROBOT_MOVE: &str = "robot.move";
    /// Head joint targets. Continuous; send as a notification.
    pub const ROBOT_HEAD: &str = "robot.head";
    /// Stop moving — zero the velocity. Not "go limp".
    pub const ROBOT_STOP: &str = "robot.stop";
    /// Turn policy execution on or off.
    pub const ROBOT_ENABLE: &str = "robot.enable";

    // ── power to the joints ──────────────────────────────────────────────────
    //
    // The pair, and they are a pair: nothing else in this API turns the motors on or off.
    // `robot.enable` is about the *policy* — it can bring a limp robot up as a side effect of
    // being asked to drive, but "stand up" and "let go" are their own decisions and deserve
    // their own names.
    //
    // Both belong to `robotd` rather than to a subcommand, which is the point of adding them:
    // `robotd init` opens the motor bus itself, so it needs the daemon stopped, and two writers
    // on one UART corrupt each other's replies. The daemon owns the bus; ask the daemon.

    /// Power the joints and ramp to the home pose.
    ///
    /// Unlike [`ROBOT_ENABLE`] this needs no policy: "stand up" is a reasonable thing to ask of a
    /// robot with no walking network at all, and it is what a bench robot needs before anything
    /// else can be tested.
    pub const ROBOT_INIT: &str = "robot.init";

    /// Cut power to the joints. **The robot will collapse** if nothing is holding it.
    ///
    /// Named `relax` rather than `limp` because `gain_limp` already means something else — the soft
    /// yield a fallen robot is commanded at, which keeps torque on. This is the register.
    pub const ROBOT_RELAX: &str = "robot.relax";

    /// Turn the connection into a stream of [`ROBOT_STATE`] notifications.
    pub const ROBOT_SUBSCRIBE: &str = "robot.subscribe";
    /// Server → client. Never carries an `id`.
    ///
    /// One stream for every consumer — `robotctl monitor`, a digital-twin viewer, later the
    /// app through `mediad`. It replaces the prototype's five bespoke channels: a 180-byte
    /// binary frame on 9870, JPEG on 9871, a UDP command socket on 9872, maploc on
    /// 9874/9875, and the web hub's `/state.json`. Adding a field there meant editing four
    /// places that could silently disagree; here it is one struct, and older clients ignore
    /// what they do not recognise.
    pub const ROBOT_STATE: &str = "robot.state";
    // ── configd's side ───────────────────────────────────────────────────────
    //
    // Wifi and the robot's identity. Served by `configd` rather than `robotd` because config
    // must be reachable when the robot is dead — provisioning wifi is exactly what a client
    // needs when things are broken (`architecture.md` §3.1).
    //
    // NetworkManager owns the credentials; these methods drive it. We never store a PSK.

    /// What is the wifi doing — SSID, signal, addresses.
    pub const NET_STATUS: &str = "net.status";
    /// Which networks can this robot see?
    pub const NET_SCAN: &str = "net.scan";
    /// Join a network, storing it for next time.
    pub const NET_CONNECT: &str = "net.connect";
    /// Forget a stored network.
    pub const NET_FORGET: &str = "net.forget";

    /// Name, serial, uptime.
    pub const SYSTEM_INFO: &str = "system.info";
    /// Rename the robot. This is the name a phone sees.
    pub const SYSTEM_SET_NAME: &str = "system.setName";
    /// Reboot, cleanly, through systemd.
    pub const SYSTEM_REBOOT: &str = "system.reboot";
    /// The Bluetooth pairing PIN. Never reachable over Bluetooth itself.
    pub const SYSTEM_PAIRING_PIN: &str = "system.pairingPin";
    /// Set the Bluetooth pairing PIN.
    pub const SYSTEM_SET_PAIRING_PIN: &str = "system.setPairingPin";
    /// Prove knowledge of the pairing PIN. Answered by the transport, not by a service.
    pub const SYSTEM_AUTHENTICATE: &str = "system.authenticate";

    // ── pad.* ────────────────────────────────────────────────────────────────
    //
    // A gamepad, as a *thing paired to the robot* rather than as a control transport. `padd`
    // reads the pad and sends intents; this namespace only decides which pad the board knows
    // about, which is a Bluetooth question and therefore `configd`'s (it is the service that
    // already owns the radio's configuration side, and the one running as root).
    //
    // Pairing is deliberately not `padd`'s own job: `padd` is an *unprivileged intent client*,
    // and the whole point of it having no privileged access is that it exercises the same API the
    // phone app will. Letting it configure BlueZ would have undone that.

    /// Which pads this robot knows, and whether `padd` is driving from one.
    pub const PAD_STATUS: &str = "pad.status";
    /// Pair the gamepad that is in pairing mode now.
    pub const PAD_PAIR: &str = "pad.pair";
    /// Forget a pad, so it stops reconnecting.
    pub const PAD_FORGET: &str = "pad.forget";
}

/// JSON-RPC error codes.
///
/// -32768..-32000 is spec-reserved; application errors use a private range. The
/// distinctions let a client act: retry on [`BUSY`], report "correctly refused" rather
/// than "something broke".
pub mod code {
    // Spec-reserved.
    pub const PARSE_ERROR: i32 = -32700;
    pub const INVALID_REQUEST: i32 = -32600;
    pub const METHOD_NOT_FOUND: i32 = -32601;
    pub const INVALID_PARAMS: i32 = -32602;
    pub const INTERNAL_ERROR: i32 = -32603;

    // Application-specific.
    pub const BUSY: i32 = 1;
    pub const UNKNOWN_COMPONENT: i32 = 2;
    pub const PROTOCOL_MISMATCH: i32 = 3;
    pub const PREFLIGHT_FAILED: i32 = 4;
    pub const NETWORK: i32 = 5;
    pub const VERIFICATION_FAILED: i32 = 6;
    pub const INCOMPATIBLE: i32 = 7;
    pub const HOOK_FAILED: i32 = 8;
    pub const HEALTH_CHECK_FAILED: i32 = 9;
    /// Update failed *and* rollback failed. Distinct so support sees the most serious
    /// outcome immediately.
    pub const ROLLBACK_FAILED: i32 = 10;
    /// The component exists but that version is not installed — as opposed to
    /// [`UNKNOWN_COMPONENT`], "no such robot part".
    pub const NOT_INSTALLED: i32 = 11;
    /// A newer version is installed and the request would move backwards.
    pub const WOULD_DOWNGRADE: i32 = 12;
    /// Verified, but larger than the configured archive limits allow.
    pub const ARCHIVE_TOO_LARGE: i32 = 13;
    /// The caller may connect but may not perform this operation — "ask an
    /// administrator", not "something broke".
    pub const PERMISSION_DENIED: i32 = 14;
}

/// Request identifier. `None` on a [`Request`] makes it a notification.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Id {
    Number(u64),
    Text(String),
}

// ── calls ────────────────────────────────────────────────────────────────────

/// A method together with its parameters.
///
/// Every request is built from one of these and read back as one, so a method can never be
/// paired with another method's parameters — the drift this crate exists to prevent.
#[derive(Debug, Clone, PartialEq)]
pub enum Call {
    /// Version handshake. The first call on a connection.
    Hello(HelloParams),

    // ── update.* ─────────────────────────────────────────────────────────────
    Check(ComponentParams),
    Apply(ApplyParams),
    Rollback(ComponentParams),
    ResetToGolden(ComponentParams),
    Select(SelectParams),
    Pin(PinParams),
    Status,
    ListInstalled(ComponentParams),
    Log(LogParams),
    /// Turns the connection into a stream of [`method::PROGRESS`] notifications.
    Subscribe,

    // ── robot.* ──────────────────────────────────────────────────────────────
    RobotSafeToRestart,
    RobotHealth,
    RobotModelApi,
    RobotRemoteSessionActive,

    // ── intents ──────────────────────────────────────────────────────────────
    /// Continuous. Send as a notification.
    RobotMove(MoveParams),
    /// Continuous. Send as a notification.
    RobotHead(HeadParams),
    RobotStop,
    RobotEnable(EnableParams),
    /// Power the joints and ramp to the home pose. No policy needed.
    RobotInit,
    /// Cut power to the joints. The robot collapses if nothing holds it.
    RobotRelax,
    RobotSubscribe(SubscribeParams),
    // ── net.* ────────────────────────────────────────────────────────────────
    NetStatus,
    NetScan,
    NetConnect(NetConnectParams),
    NetForget(NetForgetParams),

    // ── system.* ─────────────────────────────────────────────────────────────
    SystemInfo,
    SystemSetName(SetNameParams),
    SystemReboot,
    /// Read the pairing PIN.
    ///
    /// Exists so `btd` can answer a BLE passkey request without owning config. It must never be
    /// routed to BLE — a PIN an unpaired peer can read authorises nothing — and `btd`'s routing
    /// table has a test saying so.
    SystemPairingPin,
    SystemSetPairingPin(SetPairingPinParams),
    /// Prove knowledge of the robot's pairing PIN.
    ///
    /// Answered by the **transport** rather than by any service, which makes it unlike every
    /// other call here. BLE cannot express a fixed, printed-on-the-robot passkey — the spec has
    /// the *displaying* side generate a random one, and a headless robot can display nothing — so
    /// the PIN check moved from the link layer to this one, where we define the rules. See
    /// `docs/design/app-path-design.md` §5.
    SystemAuthenticate(AuthenticateParams),

    // ── pad.* ────────────────────────────────────────────────────────────────
    PadStatus,
    PadPair(PadPairParams),
    PadForget(PadForgetParams),
}

impl Call {
    /// The wire method name.
    pub fn method(&self) -> &'static str {
        match self {
            Call::Hello(_) => method::HELLO,
            Call::Check(_) => method::CHECK,
            Call::Apply(_) => method::APPLY,
            Call::Rollback(_) => method::ROLLBACK,
            Call::ResetToGolden(_) => method::RESET_TO_GOLDEN,
            Call::Select(_) => method::SELECT,
            Call::Pin(_) => method::PIN,
            Call::Status => method::STATUS,
            Call::ListInstalled(_) => method::LIST_INSTALLED,
            Call::Log(_) => method::LOG,
            Call::Subscribe => method::SUBSCRIBE,
            Call::RobotSafeToRestart => method::ROBOT_SAFE_TO_RESTART,
            Call::RobotHealth => method::ROBOT_HEALTH,
            Call::RobotModelApi => method::ROBOT_MODEL_API,
            Call::RobotRemoteSessionActive => method::ROBOT_SESSION_ACTIVE,
            Call::RobotMove(_) => method::ROBOT_MOVE,
            Call::RobotHead(_) => method::ROBOT_HEAD,
            Call::RobotStop => method::ROBOT_STOP,
            Call::RobotEnable(_) => method::ROBOT_ENABLE,
            Call::RobotInit => method::ROBOT_INIT,
            Call::RobotRelax => method::ROBOT_RELAX,
            Call::RobotSubscribe(_) => method::ROBOT_SUBSCRIBE,
            Call::NetStatus => method::NET_STATUS,
            Call::NetScan => method::NET_SCAN,
            Call::NetConnect(_) => method::NET_CONNECT,
            Call::NetForget(_) => method::NET_FORGET,
            Call::SystemInfo => method::SYSTEM_INFO,
            Call::SystemSetName(_) => method::SYSTEM_SET_NAME,
            Call::SystemReboot => method::SYSTEM_REBOOT,
            Call::SystemPairingPin => method::SYSTEM_PAIRING_PIN,
            Call::SystemSetPairingPin(_) => method::SYSTEM_SET_PAIRING_PIN,
            Call::SystemAuthenticate(_) => method::SYSTEM_AUTHENTICATE,
            Call::PadStatus => method::PAD_STATUS,
            Call::PadPair(_) => method::PAD_PAIR,
            Call::PadForget(_) => method::PAD_FORGET,
        }
    }

    /// Does this change the robot's software?
    ///
    /// `updaterd` authorises exactly these against the caller's uid/gid; read-only calls
    /// are ungated, so support can inspect a robot it is not allowed to change.
    pub fn is_mutating(&self) -> bool {
        matches!(
            self,
            Call::Apply(_)
                | Call::Rollback(_)
                | Call::ResetToGolden(_)
                | Call::Select(_)
                | Call::Pin(_)
                // Changing the robot's *configuration* is mutating too. Joining a network is
                // not a read, and a reboot is the most disruptive thing a client can ask for.
                | Call::NetConnect(_)
                | Call::NetForget(_)
                | Call::SystemSetName(_)
                | Call::SystemReboot
                | Call::SystemSetPairingPin(_)
                // Bonding a pad to this robot changes what may drive it, which is the most
                // consequential thing in this namespace — a paired pad can enable the policy.
                // `pad.status` is a read and stays ungated.
                | Call::PadPair(_)
                | Call::PadForget(_)
        )
    }

    /// The component this call is about, where it names one.
    pub fn component(&self) -> Option<&ComponentId> {
        match self {
            Call::Check(p)
            | Call::Rollback(p)
            | Call::ResetToGolden(p)
            | Call::ListInstalled(p) => Some(&p.component),
            Call::Apply(p) => Some(&p.component),
            Call::Select(p) => Some(&p.component),
            Call::Pin(p) => Some(&p.component),
            _ => None,
        }
    }

    /// Parameters as they go on the wire. Methods that take none send `{}`, so every
    /// request has the same shape.
    fn params(&self) -> Value {
        fn encode(params: &impl Serialize) -> Value {
            // Plain structs of strings, bools, ints and versions: this cannot fail.
            serde_json::to_value(params).unwrap_or(Value::Null)
        }
        match self {
            Call::Hello(p) => encode(p),
            Call::Check(p)
            | Call::Rollback(p)
            | Call::ResetToGolden(p)
            | Call::ListInstalled(p) => encode(p),
            Call::Apply(p) => encode(p),
            Call::Select(p) => encode(p),
            Call::Pin(p) => encode(p),
            Call::Log(p) => encode(p),
            Call::RobotMove(p) => encode(p),
            Call::RobotHead(p) => encode(p),
            Call::RobotEnable(p) => encode(p),
            Call::RobotSubscribe(p) => encode(p),
            Call::NetConnect(p) => encode(p),
            Call::NetForget(p) => encode(p),
            Call::SystemSetName(p) => encode(p),
            Call::SystemSetPairingPin(p) => encode(p),
            Call::SystemAuthenticate(p) => encode(p),
            Call::PadPair(p) => encode(p),
            Call::PadForget(p) => encode(p),
            Call::Status
            | Call::Subscribe
            | Call::RobotSafeToRestart
            | Call::RobotHealth
            | Call::RobotModelApi
            | Call::RobotRemoteSessionActive
            | Call::RobotStop
            | Call::RobotInit
            | Call::RobotRelax => Value::Object(serde_json::Map::new()),
            Call::NetStatus
            | Call::NetScan
            | Call::SystemInfo
            | Call::SystemReboot
            | Call::SystemPairingPin
            | Call::PadStatus => Value::Object(serde_json::Map::new()),
        }
    }

    /// Decode a method name and parameters as they arrived.
    ///
    /// The two failures stay apart because a caller acts on them differently: an unknown
    /// method is [`code::METHOD_NOT_FOUND`], parameters that do not fit are
    /// [`code::INVALID_PARAMS`]. Methods taking no parameters ignore whatever arrived.
    fn parse(method_name: &str, params: Option<&Value>) -> Result<Self, Error> {
        fn decode<T: for<'de> Deserialize<'de>>(params: Option<&Value>) -> Result<T, Error> {
            serde_json::from_value(params.cloned().unwrap_or(Value::Null))
                .map_err(|e| Error::new(code::INVALID_PARAMS, e.to_string()))
        }

        Ok(match method_name {
            method::HELLO => Call::Hello(decode(params)?),
            method::CHECK => Call::Check(decode(params)?),
            method::APPLY => Call::Apply(decode(params)?),
            method::ROLLBACK => Call::Rollback(decode(params)?),
            method::RESET_TO_GOLDEN => Call::ResetToGolden(decode(params)?),
            method::SELECT => Call::Select(decode(params)?),
            method::PIN => Call::Pin(decode(params)?),
            method::STATUS => Call::Status,
            method::LIST_INSTALLED => Call::ListInstalled(decode(params)?),
            method::LOG => Call::Log(decode(params)?),
            method::SUBSCRIBE => Call::Subscribe,
            method::ROBOT_SAFE_TO_RESTART => Call::RobotSafeToRestart,
            method::ROBOT_HEALTH => Call::RobotHealth,
            method::ROBOT_MODEL_API => Call::RobotModelApi,
            method::ROBOT_SESSION_ACTIVE => Call::RobotRemoteSessionActive,
            method::ROBOT_MOVE => Call::RobotMove(decode(params)?),
            method::ROBOT_HEAD => Call::RobotHead(decode(params)?),
            method::ROBOT_STOP => Call::RobotStop,
            method::ROBOT_ENABLE => Call::RobotEnable(decode(params)?),
            method::ROBOT_INIT => Call::RobotInit,
            method::ROBOT_RELAX => Call::RobotRelax,
            method::ROBOT_SUBSCRIBE => Call::RobotSubscribe(decode(params)?),
            method::NET_STATUS => Call::NetStatus,
            method::NET_SCAN => Call::NetScan,
            method::NET_CONNECT => Call::NetConnect(decode(params)?),
            method::NET_FORGET => Call::NetForget(decode(params)?),
            method::SYSTEM_INFO => Call::SystemInfo,
            method::SYSTEM_SET_NAME => Call::SystemSetName(decode(params)?),
            method::SYSTEM_REBOOT => Call::SystemReboot,
            method::SYSTEM_PAIRING_PIN => Call::SystemPairingPin,
            method::SYSTEM_SET_PAIRING_PIN => Call::SystemSetPairingPin(decode(params)?),
            method::SYSTEM_AUTHENTICATE => Call::SystemAuthenticate(decode(params)?),
            method::PAD_STATUS => Call::PadStatus,
            // The only method here whose parameters are *all* optional, so an absent `params`
            // member has to mean "defaults" rather than a parse error: `{"method":"pad.pair"}` is
            // the everyday call, and a hand-written client will send exactly that. Every other
            // method either needs its parameters or takes none at all, which is why this is one
            // line here rather than a change to `decode`.
            method::PAD_PAIR => {
                let empty = Value::Object(serde_json::Map::new());
                Call::PadPair(decode(params.or(Some(&empty)))?)
            }
            method::PAD_FORGET => Call::PadForget(decode(params)?),
            other => {
                return Err(Error::new(
                    code::METHOD_NOT_FOUND,
                    format!("unknown method {other:?}"),
                ));
            }
        })
    }
}

// ── envelopes ────────────────────────────────────────────────────────────────

/// A request or a notification, as it appears on the wire.
///
/// `method` and `params` stay raw here so a server can tell an unknown method from
/// parameters it could not parse. Build one with [`Self::call`] or
/// [`Self::notify_progress`] and read it back with [`Self::as_call`] or
/// [`Self::as_progress`]: those are the typed paths.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Request {
    pub jsonrpc: String,
    /// Absent on a notification.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub id: Option<Id>,
    pub method: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub params: Option<Value>,
}

impl Request {
    pub fn call(id: Id, call: &Call) -> Self {
        Self {
            jsonrpc: JSONRPC_VERSION.to_owned(),
            id: Some(id),
            method: call.method().to_owned(),
            params: Some(call.params()),
        }
    }

    /// A call sent as a notification: no `id`, so no response is expected.
    ///
    /// This is how continuous intents travel. At 50 Hz a reply per message would be pure
    /// overhead, and there is nothing useful to say about a velocity that is superseded
    /// 20 ms later. Discrete intents use [`Self::call`] instead, because "refused, and here
    /// is why" is an answer the caller needs.
    pub fn notify(call: &Call) -> Self {
        Self {
            jsonrpc: JSONRPC_VERSION.to_owned(),
            id: None,
            method: call.method().to_owned(),
            params: Some(call.params()),
        }
    }

    /// A robot-state notification: no `id`, so no response is expected.
    pub fn notify_state(state: &RobotState) -> Self {
        Self {
            jsonrpc: JSONRPC_VERSION.to_owned(),
            id: None,
            method: method::ROBOT_STATE.to_owned(),
            params: Some(serde_json::to_value(state).unwrap_or(Value::Null)),
        }
    }

    /// Read a robot-state notification back.
    pub fn as_state(&self) -> Option<RobotState> {
        if self.method != method::ROBOT_STATE {
            return None;
        }
        serde_json::from_value(self.params.clone()?).ok()
    }

    /// A progress notification: no `id`, so no response is expected.
    pub fn notify_progress(progress: &Progress) -> Self {
        Self {
            jsonrpc: JSONRPC_VERSION.to_owned(),
            id: None,
            method: method::PROGRESS.to_owned(),
            params: Some(serde_json::to_value(progress).unwrap_or(Value::Null)),
        }
    }

    pub fn is_notification(&self) -> bool {
        self.id.is_none()
    }

    /// The typed call, or the error to answer with.
    pub fn as_call(&self) -> Result<Call, Error> {
        Call::parse(&self.method, self.params.as_ref())
    }

    /// The payload of a [`method::PROGRESS`] notification.
    pub fn as_progress(&self) -> Result<Progress, Error> {
        if self.method != method::PROGRESS {
            return Err(Error::new(
                code::METHOD_NOT_FOUND,
                format!("{:?} is not a progress notification", self.method),
            ));
        }
        serde_json::from_value(self.params.clone().unwrap_or(Value::Null))
            .map_err(|e| Error::new(code::INVALID_PARAMS, e.to_string()))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Response {
    pub jsonrpc: String,
    /// `None` when the request could not be parsed well enough to recover an id.
    pub id: Option<Id>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub result: Option<Value>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<Error>,
}

impl Response {
    /// A success response.
    ///
    /// A result that cannot be serialised becomes an [`code::INTERNAL_ERROR`] response:
    /// visibly wrong, rather than a silent `null` the client would read as an answer.
    pub fn ok(id: Option<Id>, result: &impl Serialize) -> Self {
        match serde_json::to_value(result) {
            Ok(value) => Self {
                jsonrpc: JSONRPC_VERSION.to_owned(),
                id,
                result: Some(value),
                error: None,
            },
            Err(e) => Self::err(id, Error::new(code::INTERNAL_ERROR, e.to_string())),
        }
    }

    pub fn err(id: Option<Id>, error: Error) -> Self {
        Self {
            jsonrpc: JSONRPC_VERSION.to_owned(),
            id,
            result: None,
            error: Some(error),
        }
    }

    pub fn result_as<T: for<'de> Deserialize<'de>>(&self) -> Result<T, serde_json::Error> {
        serde_json::from_value(self.result.clone().unwrap_or(Value::Null))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Error {
    pub code: i32,
    /// Displayable in the app. Specific enough to diagnose from a support ticket.
    pub message: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub data: Option<Value>,
}

impl Error {
    pub fn new(code: i32, message: impl Into<String>) -> Self {
        Self {
            code,
            message: message.into(),
            data: None,
        }
    }
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "[{}] {}", self.code, self.message)
    }
}

impl std::error::Error for Error {}

// ── params ───────────────────────────────────────────────────────────────────

/// Name of a component as declared in `updater.toml` (`daemon`, `model`).
///
/// A string, not an enum: the engine is config-driven so one binary serves different
/// robots.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct ComponentId(pub String);

impl ComponentId {
    pub fn new(s: impl Into<String>) -> Self {
        Self(s.into())
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl std::fmt::Display for ComponentId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl From<&str> for ComponentId {
    fn from(s: &str) -> Self {
        Self(s.to_owned())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct HelloParams {
    pub api_version: u32,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ComponentParams {
    pub component: ComponentId,
}

// ── intent parameters ────────────────────────────────────────────────────────
//
// **Units and frame, stated once so no consumer has to rediscover them.** Everything is
// radians and radians per second, in the robot's trunk frame, right-handed: `x` forward,
// `y` left, `z` up. Positive `vyaw` turns left.
//
// This paragraph is load-bearing. The prototype accumulated
// `--laser-track-yaw-sign`, `--laser-track-pitch-sign`, `--laser-fk-pitch-sign`,
// `--laser-fk-neck-sign` and `--imu-z-rotation-deg` precisely because the convention was
// never written down, so every new consumer determined it empirically and disagreed.
// Fixing it in the protocol deletes that entire category of flag.

/// Velocity twist. Continuous intent — see [`method::ROBOT_MOVE`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct MoveParams {
    /// Forward, m/s.
    pub vx: f64,
    /// Left, m/s.
    pub vy: f64,
    /// Yaw rate, rad/s, positive turns left.
    pub vyaw: f64,
}

/// Head joint targets, radians. Continuous intent — see [`method::ROBOT_HEAD`].
///
/// Joint-space rather than a gaze direction. Both forms are wanted eventually and both will
/// be exposed; this is the one the gamepad and calibration produce, and it is what the
/// policy's observation actually carries, so it is the one that exists first.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct HeadParams {
    pub neck_pitch: f64,
    pub head_pitch: f64,
    pub head_yaw: f64,
    pub head_roll: f64,
}

/// How often a subscriber wants [`method::ROBOT_STATE`].
///
/// Decimation is per-subscriber and happens server-side, so a dashboard asking for 10 Hz
/// costs the robot a tenth of what a digital twin asking for 50 Hz does — and neither can
/// slow the control loop, which publishes into a bounded buffer and never waits.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct SubscribeParams {
    /// Absent means every tick.
    pub hz: Option<u32>,
}

/// Answer to [`Call::RobotSubscribe`].
///
/// Carries what is **constant for the life of the process** — which policy this `robotd` is
/// running — so a client can name it without the per-tick frame repeating it fifty times a
/// second. [`RobotState::policy`] says which policy *drove this tick* (`walk`, `stand`,
/// `held`); this says which network that is, which is the question anyone comparing two
/// gaits is actually asking.
///
/// `accepted` keeps the shape [`IntentResult`] had here, so a client reading only that field
/// is unaffected.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct SubscribeResult {
    pub accepted: bool,
    /// Walking policy, as a file name rather than a path: the directory is the release
    /// directory, which `robotctl version` already reports, and the file name is the part
    /// that differs between two builds someone is comparing.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub walk: Option<String>,
    /// Standing policy, when one is configured. Without it the walking policy runs at every
    /// velocity — a real configuration, and one worth being able to see.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stand: Option<String>,
    /// Why nothing is driving, when nothing is: the policy is disabled in params, or it was
    /// wanted and could not be loaded. Those are different situations — the first is a
    /// legitimate bench configuration, the second is a robot that should be rolled back —
    /// and both are invisible in a stream whose `policy` field just says `held`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unavailable: Option<String>,
}

/// Whether the policy should run. Discrete intent — see [`method::ROBOT_ENABLE`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EnableParams {
    pub on: bool,
}

/// What an apply should move to.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Target {
    /// Whatever the source advertises as newest.
    Latest,
    /// An exact version — the primitive that makes release testing scriptable.
    Exact(semver::Version),
    /// A named ref — a branch, in practice. The source maps it to a tag it can fetch.
    ///
    /// Exists so nobody has to type `0.2.0-dev.17.abc1234` to install a teammate's branch.
    /// The version inside is still unique per build; this is a *pointer* to whichever build
    /// that branch published last, which is why the tag it resolves to moves.
    ///
    /// Like [`Target::Exact`], this deliberately bypasses the downgrade guard: a dev build
    /// is a semver prerelease and therefore sorts *below* the release it precedes, so every
    /// install of one looks like a downgrade. Refusing them would make the flow useless,
    /// and an operator naming a ref is stating intent as explicitly as naming a version.
    Ref(String),
    /// The newest **release candidate** — what `release.yml` published to `staging` and
    /// nobody has promoted yet.
    ///
    /// A candidate is unreachable any other way. It is flagged as a prerelease on GitHub, so
    /// [`Target::Latest`] skips it by design — that filter is what keeps a robot from drifting
    /// onto a build no one has validated, and it has no opt-out. This variant is the opt-*in*:
    /// an operator with root saying "the one being tested", once.
    ///
    /// The candidate carries the same version the promoted release will (`0.3.0`, not
    /// `0.3.0-rc1`) and is signed with the same release key. What separates the two streams is
    /// the tag it lives under, which is why resolving this needs its own prefix rather than a
    /// flag on the existing one.
    Staging,
    /// A named candidate, when the newest is not the one wanted — reinstalling the candidate a
    /// board already ran after a rollback, or comparing two of them.
    StagingExact(semver::Version),
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ApplyOptions {
    /// Run every check (fetch, verify, compatibility, space) and stop before the symlink
    /// swap.
    #[serde(default)]
    pub dry_run: bool,
    /// Skip *only* the "no active remote session" preflight check. Never bypasses
    /// signature, hash or compatibility — those have no override.
    #[serde(default)]
    pub interrupt_sessions: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ApplyParams {
    pub component: ComponentId,
    pub target: Target,
    #[serde(default)]
    pub options: ApplyOptions,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SelectParams {
    pub component: ComponentId,
    pub version: semver::Version,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PinParams {
    pub component: ComponentId,
    /// `None` unpins.
    pub version: Option<semver::Version>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LogParams {
    pub limit: usize,
}

/// Join a wifi network.
///
/// [`Debug`] is hand-written to redact `psk`, and that is the point of the type. Every other
/// params struct derives it, and a derived one here would put a customer's wifi password into
/// the journal the first time any service logged a request it could not handle — `configd`,
/// `btd` and `robotctl` all log calls, and the credential-carrying one must not be readable
/// afterwards by anyone who can run `journalctl`.
#[derive(Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct NetConnectParams {
    pub ssid: String,
    /// `None` for an open network. Either a passphrase or a 64-hex pre-shared key;
    /// NetworkManager accepts both and we pass it through unexamined.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub psk: Option<String>,
}

impl std::fmt::Debug for NetConnectParams {
    /// Whether a key was supplied is diagnostically useful — "wrong password" and "no password
    /// sent for a secured network" are different bugs — so the *presence* is shown and the
    /// value never is.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("NetConnectParams")
            .field("ssid", &self.ssid)
            .field(
                "psk",
                if self.psk.is_some() {
                    &"<redacted>"
                } else {
                    &"<none>"
                },
            )
            .finish()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct NetForgetParams {
    pub ssid: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SetNameParams {
    pub name: String,
}

/// Prove knowledge of the pairing PIN.
///
/// [`Debug`] is hand-written to redact the PIN, for the same reason [`NetConnectParams`] is: this
/// is the only thing standing between a paired-but-unauthenticated peer and the robot, and a
/// journal is the wrong place for it.
#[derive(Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthenticateParams {
    pub pin: String,
}

impl std::fmt::Debug for AuthenticateParams {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AuthenticateParams")
            .field("pin", &"<redacted>")
            .finish()
    }
}

/// Answer to [`Call::SystemAuthenticate`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthenticateResult {
    pub authenticated: bool,
    /// Tries left before the transport closes the session. Zero means this was the last one.
    ///
    /// Reported so a client can say "two attempts left" rather than silently losing its
    /// connection — and so a brute-force attempt is visibly rationed.
    pub attempts_remaining: u32,
}

/// Set the Bluetooth pairing PIN.
///
/// A **string, not an integer**, because leading zeros are significant: the default is `000000`,
/// and a `u32` would store that as 0 and display it as "0". The robot and the phone must agree
/// on six characters.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SetPairingPinParams {
    pub pin: String,
}

// ── pad.* parameters ─────────────────────────────────────────────────────────

/// Pair the gamepad that is in pairing mode now.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct PadPairParams {
    /// Which pad, when the address is already known. **Omit it in the normal case**: the point of
    /// this call is not having to find a MAC address first, so the robot looks for a pad that
    /// is in pairing mode and takes it. Supplying one narrows the search to that address, which
    /// is what a room with several pads in it needs.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mac: Option<String>,

    /// How long to look, in seconds. `None` means the service's own default.
    ///
    /// A parameter because the caller knows something the robot does not: whoever typed this is
    /// standing there holding the pad's pairing button, and a phone app offering "keep looking"
    /// needs a longer window than a script does.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub timeout_seconds: Option<u32>,
}

/// Forget one pad, by address.
///
/// The address, not "the connected one": forgetting is what you do to a pad that is *not* in the
/// room any more — a colleague's controller that still steals the bond on boot — so identifying it
/// by its current connection state would name the wrong thing.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PadForgetParams {
    pub mac: String,
}

// ── results ──────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct HelloResult {
    pub api_version: u32,
    pub daemon_version: Option<semver::Version>,
    /// Source revision of the **running** binary, or `None` for a build that did not come
    /// from CI (someone's laptop). Always serialised, including as `null`, so the wire
    /// shape does not depend on the value.
    pub revision: Option<String>,
}

/// Where an in-flight update has got to. Mirrors the state machine in
/// `docs/design/updater-design.md` §7.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Phase {
    Idle,
    Preflight,
    Checking,
    Downloading,
    Verifying,
    Extracting,
    RunningPreHook,
    Swapping,
    RunningPostHook,
    Applying,
    HealthGate,
    Committing,
    RollingBack,
}

/// Payload of a [`method::PROGRESS`] notification.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Progress {
    pub component: ComponentId,
    pub phase: Phase,
    /// 0-100 where meaningful (downloads); `None` otherwise.
    pub percent: Option<u8>,
    pub detail: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ComponentStatus {
    pub component: ComponentId,
    pub installed: Option<semver::Version>,
    pub phase: Phase,
    /// `None` when no health probe is configured.
    pub healthy: Option<bool>,
    pub pinned: Option<semver::Version>,
    pub last_attempt: Option<LogEntry>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct InstalledRelease {
    pub version: semver::Version,
    pub active: bool,
    pub golden: bool,
    /// Git SHA of the build, for provenance.
    pub source_revision: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "state", rename_all = "snake_case")]
pub enum CheckResult {
    UpToDate {
        installed: semver::Version,
    },
    Available {
        installed: Option<semver::Version>,
        candidate: semver::Version,
        /// True when `min_supported` makes this update non-optional.
        mandatory: bool,
        changelog: Option<String>,
    },
    /// A newer version exists but cannot be installed here.
    Incompatible {
        candidate: semver::Version,
        reason: String,
    },
}

/// Result of an apply / rollback / select.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "outcome", rename_all = "snake_case")]
pub enum ApplyResult {
    Applied {
        from: Option<semver::Version>,
        to: semver::Version,
    },
    AlreadyCurrent {
        version: semver::Version,
    },
    /// Everything verified; stopped before the swap because `dry_run` was set.
    DryRunPassed {
        candidate: semver::Version,
    },
    /// Applied, failed its gate, reverted. The robot is on `reverted_to`.
    RolledBack {
        attempted: semver::Version,
        reverted_to: Option<semver::Version>,
        reason: String,
    },
    /// Failed its gate with **nowhere to revert to** — a first install that never came up,
    /// no previous release and no golden configured. Distinct from `RolledBack` because
    /// nothing was reverted: the robot needs operator or factory intervention.
    Stuck {
        version: semver::Version,
        reason: String,
    },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LogEntry {
    /// Unix seconds.
    pub at: i64,
    pub component: ComponentId,
    pub from: Option<semver::Version>,
    pub to: Option<semver::Version>,
    pub outcome: Outcome,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Outcome {
    Success,
    RolledBack {
        reason: String,
    },
    /// Refused before anything changed.
    Aborted {
        reason: String,
    },
}

/// Answer to [`Call::RobotSafeToRestart`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SafeToRestartResult {
    pub safe: bool,
    /// Why not, when `safe` is false. Displayable.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
}

/// Answer to [`Call::RobotHealth`].
///
/// A robot that is up but *not* healthy must say so rather than fail to answer: the
/// difference decides whether an update rolls back for a known reason or for a timeout.
// No `Eq`: the battery reading is a float. Nothing compares these for exact equality
// outside tests, where `PartialEq` is what `assert_eq!` needs anyway.
//
// `Default` is "nothing known": not healthy, nothing measured. That is the honest starting
// point — and it means the next reported field added here does not break every caller that
// builds one.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct HealthResult {
    pub healthy: bool,
    /// Set when the reason is a property of the *board*, not of the running release.
    ///
    /// The whole point of the health gate is to answer "did this release break the robot?".
    /// A robot with no servo power answers nothing about the release: it reported exactly
    /// the same before the swap, and reverting cannot change it — so rolling back only
    /// wastes an update and churns the boot counter. Such conditions are reported here so
    /// the gate can commit anyway, while a release that genuinely broke the control loop
    /// still reverts.
    ///
    /// Only meaningful when `healthy` is false. Defaults to false, so an older `robotd`
    /// that does not send it keeps the previous strict behaviour.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub degraded: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
    /// Motor-bus voltage, when it has been read.
    ///
    /// **Reported, never judged.** Nothing here may influence `healthy` or `degraded`: a flat
    /// pack is a fact about the robot, and a release rolled back over one would be replaced by
    /// a release judged on the same flat pack — so the robot could not be updated at all until
    /// someone charged it. It rides on this method because this is the one a human already
    /// asks (`robotctl health`), not because the gate has any use for it.
    ///
    /// Absent means *not known yet* — the first second after startup, a bus that cannot
    /// answer, or an older `robotd`. Absent is not zero volts, and a client must not render
    /// it as an empty battery.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub battery: Option<Battery>,
    /// Hottest servo, when temperatures have been read. Same rule as the battery: reported,
    /// never judged.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub motors: Option<MotorThermal>,
    /// Board temperature in °C — the hottest of the SoC's thermal zones.
    ///
    /// Distinct from [`Self::motors`], and the pair is the point: a robot that has been walking
    /// has hot *servos*, while a board in a warm enclosure with a blocked vent has a hot *SoC*
    /// and cool motors. They fail differently and are fixed differently, and one number cannot
    /// stand for both.
    ///
    /// Absent off Linux, and on a kernel without thermal sysfs.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cpu_temp_c: Option<f64>,
    /// The control loop's own numbers — the ones `healthy` was decided from.
    ///
    /// Carried so a verdict can be *checked* rather than taken on faith. "unhealthy: control
    /// loop at 43.9 Hz" is a better bug report when the reader can also see that the loop has
    /// ticked two million times and missed none of its deadlines, which says late wakeups
    /// rather than overrunning work.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub control_loop: Option<LoopHealth>,
    /// What the motor bus is doing. Present on every answer; the zero values are meaningful
    /// ("no failures"), not missing data.
    #[serde(default)]
    pub bus: BusHealth,
    /// Orientation source. Absent from an older `robotd`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub imu: Option<ImuHealth>,
}

/// The control loop's rate and timing.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct LoopHealth {
    /// What the loop is configured to run at, so the achieved figure means something without
    /// the reader having the params file open.
    pub target_hz: f64,
    /// Achieved rate over the last window. `None` until the first window closes — which is
    /// *unknown*, not zero: a rate of 0 Hz describes a stopped loop, and printing that for
    /// the first second of every robot's uptime would be a lie.
    pub achieved_hz: Option<f64>,
    pub ticks: u64,
    /// Ticks whose work overran the period, cumulative. Distinct from a rate shortfall: this
    /// is the loop doing too much, a low rate is the loop being woken late, and telling them
    /// apart is the difference between optimising and fixing a timer.
    pub missed: u64,
    /// Age of the last completed tick. Large means wedged.
    pub last_tick_age_ms: u64,
}

/// The motor bus, as the loop sees it.
///
/// `#[serde(default)]` for the reason spelled out on [`ImuHealth`], and it applies here even more
/// plainly: these are failure counters whose zero the doc comments below already call meaningful.
/// An older `robotd` that omits one is saying "no failures", not "unknown".
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct BusHealth {
    /// Consecutive failed reads; any success resets it. One is ordinary on a serial bus,
    /// which is why the cumulative count is not what is reported.
    pub consecutive_errors: u32,
    /// Failed attempts to bring the bus up at all. Non-zero means the loop has never
    /// commanded anything and is still waiting for a robot to answer — the signature of
    /// servo power being off.
    pub startup_failures: u32,
}

/// The IMU board, which rides the motor bus.
///
/// `#[serde(default)]` on the struct, not on each field, and for the same reason the parent
/// [`HealthResult`] carries `Default`: **a field added here must not make a newer reader reject
/// an older `robotd` outright.** It did once. `consecutive_stale_blocks` was added below and
/// released, and a branch predating it sent an `imu` section without the field — so a resident
/// `updaterd` failed to parse the whole reply, `health` collapsed it to `Unreachable`, and the
/// gate reverted a release from a robot that was serving its socket and running its loop at
/// 50 Hz. An hour to find, because nothing in "not healthy within 30s: unreachable" points at a
/// missing JSON field.
///
/// Sound here because every zero is *honest*: not converged, no stale reads, no run. Each one
/// reads as "nothing to report", which is exactly what an older sender is saying. That argument
/// is what makes this safe, and it is why the sibling sections carrying measurements —
/// [`Battery`], [`MotorThermal`], [`LoopHealth`] — do **not** get the same treatment: a
/// defaulted `percent: 0.0` would render as a flat pack on a robot with a full one.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct ImuHealth {
    /// Has the orientation filter converged?
    pub ready: bool,
    /// Reads that returned the previous sample unchanged, cumulative since startup.
    ///
    /// Sporadic hits are ordinary and say nothing about whether orientation is live *now*: the
    /// control loop and the board keep their own clocks, so a tick landing inside one board
    /// refresh legitimately sees the same bytes twice. Useful for scale — a handful over an
    /// hour is a healthy board — and misleading on its own, which is why it travels with the
    /// run below rather than being reported alone.
    pub stale_blocks: u64,
    /// Length of the current unbroken run of stale reads; any fresh block resets it to zero.
    ///
    /// This is the one worth alarming on. A board that has stopped fusing keeps answering the
    /// `sync_read` — so the bus reports no error and `ready` stays true — and repeats itself on
    /// every tick, which makes the run climb without bound. See [`ImuHealth::frozen`].
    pub consecutive_stale_blocks: u64,
}

impl ImuHealth {
    /// Run length at which orientation is called frozen rather than hiccuping.
    ///
    /// 25 reads is half a second at 50 Hz: long enough that no ordinary hiccup reaches it, short
    /// enough to be prompt, and the same span `SflpDecoder::ready` waits for before it will
    /// treat the chip's output as a measurement. `duck-control`'s journal warning uses the same
    /// number — deliberately, so the log and the report agree about what "frozen" means — but
    /// keeps its own copy, because the hardware layer does not depend on this IPC vocabulary.
    pub const FROZEN_RUN: u64 = 25;

    /// Is orientation frozen *now*, as opposed to having hiccuped at some point?
    ///
    /// The distinction is the whole reason both counters exist: reporting any non-zero total as
    /// a possible dead IMU meant a healthy robot wore an alarm for its entire uptime, and a
    /// warning that fires on a healthy robot is a warning nobody reads.
    pub fn frozen(&self) -> bool {
        self.consecutive_stale_blocks >= Self::FROZEN_RUN
    }
}

/// Servo case temperature, reduced to the part worth acting on.
///
/// The hottest joint rather than a mean over fifteen: a knee holding a squat runs far hotter
/// than the mouth, and averaging hides the one servo approaching the overheat shutdown its
/// error mask latches on.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MotorThermal {
    /// Name of the hottest joint, as [`JOINT_NAMES`] spells it.
    pub hottest: String,
    pub max_c: f64,
    pub mean_c: f64,
}

/// Motor-bus voltage, and what fraction of a pack that is.
///
/// Both, deliberately. Volts is the measurement; percent is a *mapping* over a pack the
/// robot knows and a client should not have to (`duck_control::model::battery_percent`).
/// The prototype shipped volts only, and the mapping was duplicated into the app — which is
/// how two screens end up disagreeing about the same battery.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Battery {
    pub volts: f64,
    pub percent: f64,
}

/// Answer to a discrete intent — [`Call::RobotStop`], [`Call::RobotEnable`].
///
/// `accepted: false` is a normal outcome, not an error: safety may refuse to enable a
/// policy on a fallen robot, and the caller needs to know *why* rather than receiving a
/// JSON-RPC error that reads as "something broke".
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IntentResult {
    pub accepted: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
}

impl IntentResult {
    pub fn accepted() -> Self {
        Self {
            accepted: true,
            reason: None,
        }
    }

    pub fn refused(reason: impl Into<String>) -> Self {
        Self {
            accepted: false,
            reason: Some(reason.into()),
        }
    }
}

/// What the robot is doing, pushed as [`method::ROBOT_STATE`].
///
/// **It reports what was refused, not just what happened.** Safety clamps things
/// constantly, and a client watching the robot ignore its command with no explanation
/// cannot tell a bug from a limit. That is why `applied` and `limited_by` exist beside
/// `requested` rather than the stream carrying only outcomes.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RobotState {
    /// Seconds since the daemon started. Monotonic: it is for correlating samples, not for
    /// telling the time.
    pub t: f64,
    #[serde(rename = "move")]
    pub movement: MoveState,
    pub head: [f64; 4],
    /// Which policy drove this tick: `walk`, `stand`, or `held` when none did.
    pub policy: String,
    pub safety: SafetyState,
    #[serde(rename = "loop")]
    pub control_loop: LoopState,
    /// Measured joint angles, radians, indexed as [`JOINT_NAMES`].
    pub joints: Vec<f64>,
    /// What was commanded, so a viewer can show tracking error rather than guessing at it.
    pub targets: Vec<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MoveState {
    pub requested: [f64; 3],
    pub applied: [f64; 3],
    /// Empty when the command went through untouched.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub limited_by: Vec<String>,
}

// `Eq` is gone with the arrival of a float: gravity is a measurement, and exact equality on
// one is not a comparison anybody should be offered.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SafetyState {
    pub fallen: bool,
    /// Gains have been dropped so the robot yields.
    pub limp: bool,
    /// Projected gravity in the trunk frame, the input `fallen` is decided from. Upright is
    /// about `[0, 0, -1]`.
    ///
    /// Reported because the verdict alone is not diagnosable: "the robot is down" and "the
    /// IMU is mounted differently than this build assumes" produce an identical `fallen`, and
    /// telling them apart otherwise means stopping the daemon and reaching for another tool.
    #[serde(default)]
    pub gravity: [f64; 3],
    /// Position P gain last written to the servos, or `None` before the first write.
    ///
    /// What the robot is actually running at, not what was asked for: safety overrides the
    /// requested gain when it decides the robot has fallen, and that override was invisible.
    #[serde(default)]
    pub gain: Option<u16>,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct LoopState {
    /// Achieved rate over the last window. Zero until the first window closes.
    pub hz: f64,
    /// Ticks whose work overran the period, cumulative.
    pub missed: u64,
}

/// Answer to [`Call::RobotModelApi`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ModelApiResult {
    /// Sensor-input / actuator-output contract this build implements
    /// (`updater-design.md` §5.5).
    pub model_api: u32,
}

/// Answer to [`Call::RobotRemoteSessionActive`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SessionActiveResult {
    pub active: bool,
}

// ── net.* results ────────────────────────────────────────────────────────────

/// What the wifi link is doing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NetState {
    /// Associated and addressed.
    Connected,
    /// Trying. A client should poll rather than conclude anything.
    Connecting,
    /// A wifi device exists and is idle.
    Disconnected,
    /// No wifi device, or nothing managing it. Distinct from `Disconnected` because it is a
    /// provisioning problem rather than a network one — on this robot it means the board is
    /// still on netplan (`scripts/migrate-network.sh`).
    Unavailable,
}

/// Answer to [`Call::NetStatus`]. Every field beyond `state` is absent when not connected.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct NetStatusResult {
    pub state: NetState,
    pub ssid: Option<String>,
    /// 0-100. NetworkManager's own scale, not dBm — a percentage is what a phone shows.
    pub signal: Option<u8>,
    pub ip4: Option<String>,
    pub ip6: Option<String>,
    /// The wifi interface's hardware address. Useful as a stable robot identifier until
    /// provisioning gives us a real serial (`updater-design.md` §5.7).
    pub mac: Option<String>,
    pub iface: Option<String>,
}

/// How a network is secured. What a client needs in order to know whether to ask for a
/// password, and which kind.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Security {
    Open,
    /// WEP. Reported so a client can say "too old to join" rather than failing obscurely.
    Wep,
    WpaPsk,
    Wpa3Sae,
    /// 802.1X. Needs a username and certificate flow this API does not have, so it is
    /// reported and refused rather than half-attempted.
    Enterprise,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Network {
    pub ssid: String,
    /// 0-100.
    pub signal: u8,
    pub security: Security,
    /// True when a stored profile already exists, so a client can offer "connect" rather than
    /// asking for a password it does not need.
    pub saved: bool,
}

/// Answer to [`Call::NetScan`], strongest first.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct NetScanResult {
    pub networks: Vec<Network>,
}

/// Why a join failed.
///
/// The distinction exists because it is the whole reason NetworkManager was chosen over
/// netplan: "you typed the password wrong" is the single most common provisioning failure, and
/// a client that cannot say so leaves the user with nothing to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConnectFailure {
    /// Authentication rejected. Ask for the password again.
    BadKey,
    /// The SSID was not there. Ask them to move closer or check the name.
    NotFound,
    /// Associated but never finished — usually DHCP. Retrying may work.
    Timeout,
    /// Refused before trying: enterprise security, or a PSK missing for a secured network.
    Unsupported,
    Other,
}

/// Answer to [`Call::NetConnect`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "outcome", rename_all = "snake_case")]
pub enum ConnectResult {
    Connected {
        ssid: String,
        /// Present once DHCP has finished, which is what makes the robot actually reachable.
        ip4: Option<String>,
    },
    Failed {
        reason: ConnectFailure,
        /// NetworkManager's own words, for a support ticket. Never shown as the primary
        /// message: `reason` is what a client should act on.
        detail: Option<String>,
    },
}

/// Answer to [`Call::NetForget`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ForgetResult {
    /// False when there was no such stored network — not an error, and a client should not
    /// present it as one.
    pub removed: bool,
}

// ── system.* results ─────────────────────────────────────────────────────────

/// Answer to [`Call::SystemInfo`].
///
/// Version deliberately absent: `hello` carries the running build and `update.status` the
/// installed release, and those are different questions (`architecture.md` §8.3). Repeating
/// one of them here would be the third place to get it wrong.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SystemInfoResult {
    /// The robot's name, as advertised over BLE and shown in an app.
    pub name: String,
    /// Per-device identity, once provisioning defines one. `None` until then rather than a
    /// fabricated value.
    pub serial: Option<String>,
    pub uptime_seconds: u64,
}

/// Answer to [`Call::SystemSetName`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SetNameResult {
    /// The name as stored, which may be a trimmed version of what was asked for. A client
    /// should display this rather than what it sent.
    pub name: String,
}

/// Answer to [`Call::SystemPairingPin`] and [`Call::SystemSetPairingPin`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PairingPinResult {
    /// Six digits, leading zeros included.
    pub pin: String,
    /// True while the robot is still on the factory PIN.
    ///
    /// Worth a field rather than leaving callers to compare against a constant: a default PIN
    /// authorises nothing, because everyone in radio range knows it, and every client should be
    /// able to say so without hardcoding the value.
    pub is_default: bool,
}

/// Answer to [`Call::SystemReboot`].
///
/// The reboot is *scheduled*, not immediate, and the delay is what makes this answerable at
/// all: a daemon that rebooted inside the call would drop the connection before responding,
/// and every client would have to treat a broken pipe as success.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RebootResult {
    pub in_seconds: u64,
}

// ── pad.* results ────────────────────────────────────────────────────────────

/// A gamepad this robot knows about.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Pad {
    /// `78:86:2E:BB:13:28`. The identity everything else here is keyed on.
    pub mac: String,
    /// As the pad calls itself — "Xbox Wireless Controller". Empty when BlueZ has no name for it
    /// yet, which happens between discovery and pairing.
    pub name: String,
    /// Bonded: keys exchanged, so it can reconnect without pairing again.
    pub paired: bool,
    /// Trusted: BlueZ accepts its connection **without anyone approving it**, which is what makes
    /// the pad work after a reboot with nobody logged in. A paired-but-untrusted pad looks paired
    /// and does not reconnect, which is why this is reported separately rather than folded in.
    pub trusted: bool,
    /// Connected right now. This is the one that answers "why is the robot not moving".
    pub connected: bool,
}

/// Whether `padd` — the process that turns a pad into intents — is running.
///
/// Reported alongside the pads because a connected pad and a dead `padd` is the failure that looks
/// like working hardware, and it is not otherwise visible without knowing to ask systemd.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DriverState {
    /// Running. With a connected pad, the robot is drivable.
    Active,
    /// The unit exists and is not running. Someone stopped it, or it is failed.
    Inactive,
    /// No `padd.service` on this board — a release older than the one that added it.
    Absent,
    /// Could not ask: no systemd, or the query failed. Distinct from `Absent`, because "I do not
    /// know" must not read as "it is not installed".
    Unknown,
}

/// Answer to [`Call::PadStatus`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PadStatusResult {
    /// Every pad the robot is bonded to, connected first.
    pub pads: Vec<Pad>,
    pub driver: DriverState,
}

/// Why pairing a pad failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PadPairFailure {
    /// Nothing that looks like a gamepad turned up. Usually the pad is not in pairing mode — on an
    /// Xbox controller that is the sync button, and the light flashes fast rather than slow.
    NotFound,
    /// Several pads were in pairing mode, so the robot refused to guess. Retry with `mac`.
    Ambiguous,
    /// Found and then lost: it appeared in discovery but did not finish bonding in time.
    Timeout,
    /// No Bluetooth adapter. On this board `hci0` does not exist until roughly 73 seconds after
    /// power-on, so this is a real answer early in a boot and not necessarily broken hardware.
    NoAdapter,
    /// BlueZ refused the bond. The classic cause on this board is `Privacy = device` missing from
    /// `/etc/bluetooth/main.conf` — the pad pairs and drops straight back out.
    Rejected,
    Other,
}

/// Answer to [`Call::PadPair`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "outcome", rename_all = "snake_case")]
pub enum PadPairResult {
    Paired {
        pad: Pad,
    },
    Failed {
        reason: PadPairFailure,
        /// BlueZ's own words, for a support ticket. `reason` is what a client acts on.
        detail: Option<String>,
    },
}

/// Answer to [`Call::PadForget`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PadForgetResult {
    /// False when no such pad was bonded — not an error, and a client should not present it as
    /// one. Same contract as [`ForgetResult`].
    pub removed: bool,
}

/// Re-exported so consumers spell version types with the *same* `semver` this crate
/// compiled against. Without it, a crate depending on `semver` separately can end up with
/// two incompatible copies of `Version` and a type error that reads as nonsense.
pub use semver;

// ── build identity ───────────────────────────────────────────────────────────

/// What a binary reports about itself: version, source revision, build time.
///
/// Lives here so every service answers "what was running when this happened?" the same
/// way. A version number alone does not answer it — two builds of `0.2.0` from different
/// commits are otherwise indistinguishable.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct BuildInfo {
    /// Crate version. All workspace crates share one version line because they ship in one
    /// artifact.
    pub version: &'static str,
    /// Git SHA, or `None` for a build that did not come from CI.
    ///
    /// Read from `DUCK_REVISION` **at compile time**: a shipped robot has no git
    /// repository. CI sets it; a laptop build honestly reports that it does not know.
    pub revision: Option<&'static str>,
    /// RFC 3339 build timestamp from `DUCK_BUILD_TIME`, or `None` locally.
    pub built_at: Option<&'static str>,
}

impl std::fmt::Display for BuildInfo {
    /// One line, greppable, and explicit about what is unknown — a support log that simply
    /// lacks a revision is ambiguous between "local build" and "we forgot to log it".
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.version)?;
        match self.revision {
            Some(rev) => write!(f, " (rev {rev}")?,
            None => write!(f, " (rev unknown, not a CI build")?,
        }
        match self.built_at {
            Some(at) => write!(f, ", built {at})"),
            None => write!(f, ")"),
        }
    }
}

/// Build identity of the **calling crate**.
///
/// A macro rather than a function because `env!` must expand in the caller: called from a
/// function here it would report this crate's version for everyone.
#[macro_export]
macro_rules! build_info {
    () => {
        $crate::BuildInfo {
            version: env!("CARGO_PKG_VERSION"),
            revision: option_env!("DUCK_REVISION"),
            built_at: option_env!("DUCK_BUILD_TIME"),
        }
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One of every [`Call`] variant, so the tests below cannot silently skip one.
    fn every_call() -> Vec<Call> {
        let component = ComponentId::new("daemon");
        let version = semver::Version::new(1, 4, 2);
        vec![
            Call::Hello(HelloParams {
                api_version: API_VERSION,
            }),
            Call::Check(ComponentParams {
                component: component.clone(),
            }),
            Call::Apply(ApplyParams {
                component: component.clone(),
                target: Target::Exact(version.clone()),
                options: ApplyOptions {
                    dry_run: true,
                    interrupt_sessions: false,
                },
            }),
            Call::Rollback(ComponentParams {
                component: component.clone(),
            }),
            Call::ResetToGolden(ComponentParams {
                component: component.clone(),
            }),
            Call::Select(SelectParams {
                component: component.clone(),
                version: version.clone(),
            }),
            Call::Pin(PinParams {
                component: component.clone(),
                version: Some(version),
            }),
            Call::Status,
            Call::ListInstalled(ComponentParams { component }),
            Call::Log(LogParams { limit: 20 }),
            Call::Subscribe,
            Call::RobotSafeToRestart,
            Call::RobotHealth,
            Call::RobotModelApi,
            Call::RobotRemoteSessionActive,
            Call::RobotMove(MoveParams {
                vx: 0.2,
                vy: -0.1,
                vyaw: 0.4,
            }),
            Call::RobotHead(HeadParams {
                neck_pitch: 0.35,
                head_pitch: -0.1,
                head_yaw: 0.2,
                head_roll: 0.0,
            }),
            Call::RobotStop,
            Call::RobotEnable(EnableParams { on: true }),
            Call::RobotInit,
            Call::RobotRelax,
            Call::RobotSubscribe(SubscribeParams { hz: Some(10) }),
            Call::NetStatus,
            Call::NetScan,
            Call::NetConnect(NetConnectParams {
                ssid: "Pollen Guest".into(),
                psk: Some("hunter2 with spaces".into()),
            }),
            Call::NetForget(NetForgetParams {
                ssid: "Old Network".into(),
            }),
            Call::SystemInfo,
            Call::SystemSetName(SetNameParams {
                name: "duck-01".into(),
            }),
            Call::SystemReboot,
            Call::SystemPairingPin,
            Call::SystemSetPairingPin(SetPairingPinParams {
                pin: "042042".into(),
            }),
            Call::SystemAuthenticate(AuthenticateParams {
                pin: "000000".into(),
            }),
            Call::PadStatus,
            Call::PadPair(PadPairParams {
                mac: Some("78:86:2E:BB:13:28".into()),
                timeout_seconds: Some(20),
            }),
            Call::PadForget(PadForgetParams {
                mac: "78:86:2E:BB:13:28".into(),
            }),
        ]
    }

    /// `every_call` is a hand-written list, so a new variant is silently untested unless
    /// someone remembers to add it. Pin the count: adding a `Call` without extending the
    /// list fails here, which is the only thing standing between a new method and it never
    /// being round-tripped at all.
    #[test]
    fn every_call_covers_every_variant() {
        assert_eq!(
            every_call().len(),
            35,
            "a Call variant was added or removed — update every_call() and this count"
        );
    }

    /// `pad.pair` with nothing in it is the *normal* call — "pair whatever pad is in pairing
    /// mode" — and its fields are `skip_serializing_if`, so it is different bytes on the wire from
    /// the populated form `every_call` covers. Both shapes have to survive.
    #[test]
    fn pairing_a_pad_needs_no_parameters() {
        let call = Call::PadPair(PadPairParams::default());
        let params = call.params();
        assert_eq!(params, Value::Object(serde_json::Map::new()), "{params}");
        assert_eq!(Call::parse(call.method(), Some(&params)).unwrap(), call);
        // And an omitted `params` entirely, which is what a hand-written client sends.
        assert_eq!(Call::parse(call.method(), None).unwrap(), call);
    }

    /// Every call must survive the wire unchanged.
    ///
    /// This is what makes `method`, `params` and `parse` one contract: a method wired to
    /// the wrong parameter type in any one of them fails here rather than on a robot.
    #[test]
    fn every_call_round_trips_over_the_wire() {
        for call in every_call() {
            let line = serde_json::to_string(&Request::call(Id::Number(1), &call)).unwrap();
            let request: Request = serde_json::from_str(&line).unwrap();

            assert_eq!(request.method, call.method(), "{line}");
            assert_eq!(request.as_call().unwrap(), call, "{line}");
        }
    }

    #[test]
    fn a_call_serialises_as_jsonrpc() {
        let call = Call::Apply(ApplyParams {
            component: ComponentId::new("daemon"),
            target: Target::Exact(semver::Version::new(1, 4, 2)),
            options: ApplyOptions {
                dry_run: true,
                interrupt_sessions: false,
            },
        });
        let line = serde_json::to_string(&Request::call(Id::Number(1), &call)).unwrap();

        assert!(line.contains(r#""jsonrpc":"2.0""#), "{line}");
        assert!(line.contains(r#""method":"update.apply""#), "{line}");
        assert!(line.contains(r#""dry_run":true"#), "{line}");
    }

    /// An unknown method and unparseable parameters are different failures, and a client
    /// acts on them differently.
    #[test]
    fn unknown_methods_and_bad_params_get_different_codes() {
        let unknown = Request {
            jsonrpc: JSONRPC_VERSION.to_owned(),
            id: Some(Id::Number(1)),
            method: "update.doSomethingElse".to_owned(),
            params: None,
        };
        assert_eq!(unknown.as_call().unwrap_err().code, code::METHOD_NOT_FOUND);

        let malformed = Request {
            jsonrpc: JSONRPC_VERSION.to_owned(),
            id: Some(Id::Number(1)),
            method: method::APPLY.to_owned(),
            params: Some(serde_json::json!({ "wrong": "shape" })),
        };
        assert_eq!(malformed.as_call().unwrap_err().code, code::INVALID_PARAMS);
    }

    /// A no-parameter method must accept whatever a client sent for `params` — `{}`, `null`
    /// or nothing at all. Refusing one of those would be a protocol trap with no upside.
    #[test]
    fn methods_without_params_accept_any_params_field() {
        for params in [
            None,
            Some(Value::Null),
            Some(serde_json::json!({})),
            Some(serde_json::json!({ "ignored": 1 })),
        ] {
            let request = Request {
                jsonrpc: JSONRPC_VERSION.to_owned(),
                id: Some(Id::Number(1)),
                method: method::ROBOT_HEALTH.to_owned(),
                params: params.clone(),
            };
            assert_eq!(
                request.as_call().unwrap(),
                Call::RobotHealth,
                "params: {params:?}"
            );
        }
    }

    /// Only the calls that replace software are authorised. A read-only call caught here
    /// would lock support out of a robot it is meant to be able to inspect.
    #[test]
    fn only_software_changing_calls_are_mutating() {
        let mutating: Vec<&'static str> = every_call()
            .iter()
            .filter(|call| call.is_mutating())
            .map(Call::method)
            .collect();

        assert_eq!(
            mutating,
            vec![
                method::APPLY,
                method::ROLLBACK,
                method::RESET_TO_GOLDEN,
                method::SELECT,
                method::PIN,
                method::NET_CONNECT,
                method::NET_FORGET,
                method::SYSTEM_SET_NAME,
                method::SYSTEM_REBOOT,
                method::SYSTEM_SET_PAIRING_PIN,
                // Bonding a pad decides what may drive this robot. `pad.status` must stay off this
                // list: reading which pads are paired is exactly the kind of inspection support
                // needs on a robot it is not allowed to reconfigure.
                method::PAD_PAIR,
                method::PAD_FORGET,
            ]
        );
    }

    /// Every call naming a component must expose it: `updaterd` logs it alongside the
    /// caller's uid, and a missing one makes the audit line useless.
    #[test]
    fn component_carrying_calls_expose_it() {
        for call in every_call() {
            let carries_one = matches!(
                call.method(),
                method::CHECK
                    | method::APPLY
                    | method::ROLLBACK
                    | method::RESET_TO_GOLDEN
                    | method::SELECT
                    | method::PIN
                    | method::LIST_INSTALLED
            );
            assert_eq!(call.component().is_some(), carries_one, "{}", call.method());
        }
    }

    #[test]
    fn notifications_carry_no_id() {
        let note = Request::notify_progress(&Progress {
            component: ComponentId::new("daemon"),
            phase: Phase::Downloading,
            percent: Some(42),
            detail: None,
        });

        let line = serde_json::to_string(&note).unwrap();
        assert!(!line.contains("\"id\""), "{line}");
        assert!(note.is_notification());
    }

    /// The id is genuinely absent on a notification, not null, and it must still parse when
    /// the server sends one to a subscriber that reconnected.
    #[test]
    fn a_notification_parses_without_an_id_field() {
        let line = r#"{"jsonrpc":"2.0","method":"update.progress","params":{"component":"model","phase":"health_gate","percent":null,"detail":null}}"#;
        let request: Request = serde_json::from_str(line).unwrap();

        assert!(request.is_notification());
        assert_eq!(request.as_progress().unwrap().phase, Phase::HealthGate);
    }

    /// A response carries a result or an error, never both.
    #[test]
    fn responses_omit_the_half_they_do_not_use() {
        let ok = Response::ok(
            Some(Id::Number(7)),
            &CheckResult::UpToDate {
                installed: semver::Version::new(1, 0, 0),
            },
        );
        let line = serde_json::to_string(&ok).unwrap();
        assert!(!line.contains("\"error\""), "{line}");
        let back: Response = serde_json::from_str(&line).unwrap();
        assert!(matches!(
            back.result_as::<CheckResult>().unwrap(),
            CheckResult::UpToDate { .. }
        ));

        let failed = Response::err(
            Some(Id::Number(7)),
            Error::new(code::BUSY, "another update is in progress"),
        );
        let line = serde_json::to_string(&failed).unwrap();
        assert!(!line.contains("\"result\""), "{line}");
        assert!(line.contains("\"code\":1"), "{line}");
    }

    /// An omitted `reason` must stay omitted: `updaterd` distinguishes "unhealthy with a
    /// reason" from "unhealthy".
    #[test]
    fn robot_results_round_trip() {
        let healthy = HealthResult {
            healthy: true,
            ..Default::default()
        };
        let line = serde_json::to_string(&healthy).unwrap();
        assert!(!line.contains("reason"), "{line}");
        assert_eq!(
            serde_json::from_str::<HealthResult>(&line).unwrap(),
            healthy
        );

        let sick = HealthResult {
            reason: Some("motors not responding".into()),
            ..Default::default()
        };
        let line = serde_json::to_string(&sick).unwrap();
        assert_eq!(serde_json::from_str::<HealthResult>(&line).unwrap(), sick);
    }

    /// `move` and `loop` are Rust keywords, so the fields are renamed on the wire. A typo
    /// in either rename is invisible in Rust and breaks every consumer, so pin the JSON.
    #[test]
    fn robot_state_uses_the_documented_field_names() {
        let state = RobotState {
            t: 1.5,
            movement: MoveState {
                requested: [0.4, 0.0, 0.0],
                applied: [0.15, 0.0, 0.0],
                limited_by: vec!["deadman".into()],
            },
            head: [0.0; 4],
            policy: "walk".into(),
            safety: SafetyState {
                fallen: false,
                limp: false,
                gravity: [0.0, 0.0, -1.0],
                gain: Some(200),
            },
            control_loop: LoopState {
                hz: 49.8,
                missed: 0,
            },
            joints: vec![0.0; 15],
            targets: vec![0.0; 15],
        };

        let line = serde_json::to_string(&Request::notify_state(&state)).unwrap();
        assert!(line.contains(r#""method":"robot.state""#), "{line}");
        assert!(line.contains(r#""move":"#), "{line}");
        assert!(line.contains(r#""loop":"#), "{line}");
        assert!(!line.contains("movement"), "{line}");
        assert!(!line.contains("control_loop"), "{line}");

        let back: Request = serde_json::from_str(&line).unwrap();
        assert!(back.is_notification(), "state carries no id");
        assert_eq!(back.as_state().unwrap(), state);
    }

    /// An unlimited command must not carry an empty array — a consumer checking
    /// truthiness on `limited_by` should see the field absent, not present-and-empty.
    #[test]
    fn an_unlimited_command_omits_limited_by() {
        let movement = MoveState {
            requested: [0.0; 3],
            applied: [0.0; 3],
            limited_by: Vec::new(),
        };
        let line = serde_json::to_string(&movement).unwrap();
        assert!(!line.contains("limited_by"), "{line}");
    }

    /// `degraded` must default to false, so an older `robotd` that never sends the field
    /// keeps the strict behaviour: unhealthy means roll back.
    #[test]
    fn health_without_the_degraded_field_is_not_degraded() {
        let answer: HealthResult =
            serde_json::from_str(r#"{"healthy":false,"reason":"motors not responding"}"#).unwrap();
        assert!(!answer.degraded);
    }

    /// And it is absent from the wire when false, so the common answers stay small.
    #[test]
    fn degraded_is_omitted_when_false_and_present_when_true() {
        let plain = HealthResult {
            healthy: true,
            ..Default::default()
        };
        assert!(!serde_json::to_string(&plain).unwrap().contains("degraded"));

        let bench = HealthResult {
            degraded: true,
            reason: Some("no answer from the motor bus".into()),
            ..Default::default()
        };
        let line = serde_json::to_string(&bench).unwrap();
        assert!(line.contains(r#""degraded":true"#), "{line}");
        assert_eq!(serde_json::from_str::<HealthResult>(&line).unwrap(), bench);
    }

    /// An `imu` section from a `robotd` that predates a field must still parse.
    ///
    /// The regression this exists for reverted a good release. `consecutive_stale_blocks` was
    /// added below and released; a branch that had merged `main` before that sent the section
    /// without it, and the resident `updaterd` rejected the whole reply — so a robot serving its
    /// socket with the loop at 50 Hz was reported as "not healthy within 30s: unreachable".
    ///
    /// Literal JSON rather than a struct with a field omitted, because a struct cannot express
    /// "this field does not exist", which is the entire failure.
    #[test]
    fn an_imu_section_missing_its_newest_field_still_parses() {
        let answer: HealthResult =
            serde_json::from_str(r#"{"healthy":true,"imu":{"ready":true,"stale_blocks":3}}"#)
                .unwrap();

        let imu = answer
            .imu
            .expect("the section was sent, so it must survive");
        assert_eq!(imu.stale_blocks, 3, "what was sent must be kept");
        assert_eq!(
            imu.consecutive_stale_blocks, 0,
            "and what was not sent reads as nothing to report"
        );
        assert!(
            !imu.frozen(),
            "a default run must never look like a dead IMU"
        );
    }

    /// Same for the bus counters, where a missing counter means "no failures" by construction.
    #[test]
    fn a_bus_section_missing_a_counter_still_parses() {
        let answer: HealthResult =
            serde_json::from_str(r#"{"healthy":true,"bus":{"consecutive_errors":2}}"#).unwrap();

        assert_eq!(answer.bus.consecutive_errors, 2);
        assert_eq!(answer.bus.startup_failures, 0);
    }

    /// An absent battery must stay absent, not become zero volts.
    ///
    /// This is the answer for the first second after startup, for a bus that cannot reply,
    /// and for an older `robotd` that has never heard of the field. A `0.0` default would
    /// make every one of those render as a flat pack — alarming, and wrong.
    #[test]
    fn a_missing_battery_is_unknown_not_empty() {
        let answer: HealthResult = serde_json::from_str(r#"{"healthy":true}"#).unwrap();
        assert!(answer.battery.is_none());

        let unread = HealthResult {
            healthy: true,
            ..Default::default()
        };
        assert!(!serde_json::to_string(&unread).unwrap().contains("battery"));

        let measured = HealthResult {
            battery: Some(Battery {
                volts: 7.62,
                percent: 63.75,
            }),
            ..unread
        };
        let line = serde_json::to_string(&measured).unwrap();
        assert!(line.contains(r#""volts":7.62"#), "{line}");
        assert_eq!(
            serde_json::from_str::<HealthResult>(&line).unwrap(),
            measured
        );
    }

    /// A local build must say so, rather than looking like a release whose revision was
    /// simply not logged.
    #[test]
    fn build_info_is_explicit_about_an_unknown_revision() {
        let local = BuildInfo {
            version: "0.2.0",
            revision: None,
            built_at: None,
        };
        assert_eq!(local.to_string(), "0.2.0 (rev unknown, not a CI build)");

        let released = BuildInfo {
            version: "0.2.0",
            revision: Some("abc1234"),
            built_at: Some("2026-07-28T12:00:00Z"),
        };
        assert_eq!(
            released.to_string(),
            "0.2.0 (rev abc1234, built 2026-07-28T12:00:00Z)"
        );
    }

    #[test]
    fn build_info_macro_reports_the_calling_crate() {
        assert_eq!(build_info!().version, env!("CARGO_PKG_VERSION"));
    }

    /// A wifi passphrase must never reach a log, and this is the only params struct where that
    /// is true — so the redaction is hand-written and therefore able to rot. `{:?}` is what
    /// every `tracing` call site uses, so that is what is checked.
    #[test]
    fn a_wifi_key_is_redacted_from_debug_output() {
        let secret = "correct horse battery staple";
        let params = NetConnectParams {
            ssid: "Home".into(),
            psk: Some(secret.into()),
        };

        let debug = format!("{params:?}");
        assert!(
            !debug.contains(secret),
            "the key reached Debug output: {debug}"
        );
        assert!(debug.contains("Home"), "{debug}");
        // Presence still visible: "wrong password" and "no password sent" are different bugs.
        assert!(debug.contains("redacted"), "{debug}");

        let open = NetConnectParams {
            ssid: "Cafe".into(),
            psk: None,
        };
        assert!(format!("{open:?}").contains("none"), "{open:?}");
    }

    /// The PIN must be redacted for the same reason a wifi key is: it is the only thing standing
    /// between a paired peer and the robot.
    #[test]
    fn a_pairing_pin_is_redacted_from_debug_output() {
        let params = AuthenticateParams {
            pin: "482913".into(),
        };
        let debug = format!("{params:?}");
        assert!(
            !debug.contains("482913"),
            "the PIN reached Debug output: {debug}"
        );
        assert!(debug.contains("redacted"), "{debug}");
        // And still reaches the wire, or nothing could check it.
        assert!(serde_json::to_string(&params).unwrap().contains("482913"));
    }

    /// Redaction must not extend to the wire, or `configd` would receive no key at all.
    #[test]
    fn a_wifi_key_still_serialises() {
        let params = NetConnectParams {
            ssid: "Home".into(),
            psk: Some("s3cret".into()),
        };
        let line = serde_json::to_string(&params).unwrap();
        assert!(line.contains("s3cret"), "{line}");
        assert_eq!(
            serde_json::from_str::<NetConnectParams>(&line).unwrap(),
            params
        );

        // An open network omits the field rather than sending null, so a backend can tell
        // "no key" from "empty key".
        let open = NetConnectParams {
            ssid: "Cafe".into(),
            psk: None,
        };
        assert!(!serde_json::to_string(&open).unwrap().contains("psk"));
    }

    /// `Target` must survive the wire in all five forms, and the three that carry data must
    /// not be confusable. `latest` is a bare string while the others are single-key objects,
    /// which is what an externally-tagged enum with `rename_all = "snake_case"` produces —
    /// pinned here because this JSON is a contract with `btd` and the app, not an
    /// implementation detail free to change when someone adjusts a derive.
    #[test]
    fn target_round_trips_in_every_form() {
        let cases = [
            (Target::Latest, r#""latest""#),
            (
                Target::Exact(semver::Version::new(1, 2, 3)),
                r#"{"exact":"1.2.3"}"#,
            ),
            (Target::Ref("my-branch".into()), r#"{"ref":"my-branch"}"#),
            (Target::Staging, r#""staging""#),
            (
                Target::StagingExact(semver::Version::new(0, 3, 0)),
                r#"{"staging_exact":"0.3.0"}"#,
            ),
        ];
        for (target, expected) in cases {
            let line = serde_json::to_string(&target).unwrap();
            assert_eq!(line, expected, "{target:?}");
            assert_eq!(serde_json::from_str::<Target>(&line).unwrap(), target);
        }
    }

    /// A branch name with slashes is a valid git ref and must survive verbatim. `feature/foo`
    /// is the common case, and anything clever here would mangle it silently.
    #[test]
    fn a_ref_with_a_slash_survives_the_wire() {
        let target = Target::Ref("feature/nested/name".into());
        let line = serde_json::to_string(&target).unwrap();
        assert_eq!(serde_json::from_str::<Target>(&line).unwrap(), target);
    }

    /// A local build reports no revision, and that must reach the wire as `null` rather
    /// than an absent field — one shape whatever the value.
    #[test]
    fn hello_result_round_trips_with_and_without_a_revision() {
        let local = HelloResult {
            api_version: API_VERSION,
            daemon_version: Some(semver::Version::new(0, 1, 0)),
            revision: None,
        };
        let line = serde_json::to_string(&local).unwrap();
        assert!(line.contains("\"revision\":null"), "{line}");
        assert_eq!(serde_json::from_str::<HelloResult>(&line).unwrap(), local);

        let released = HelloResult {
            revision: Some("abc1234".into()),
            ..local
        };
        let line = serde_json::to_string(&released).unwrap();
        assert_eq!(
            serde_json::from_str::<HelloResult>(&line).unwrap(),
            released
        );
    }
}
