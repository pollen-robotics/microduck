//! Which calls BLE may make, and which socket answers them.
//!
//! BLE exposes a **subset** of the robot API (`architecture.md` §4.1): provisioning, status,
//! and the update trigger with its progress. It is too slow and too constrained for the full
//! surface, and — more to the point — a radio anybody within a few metres can talk to is not
//! the transport over which to offer "reset this robot to factory state".
//!
//! One table decides both questions, because they are the same question: a call is permitted
//! exactly when this file names the service that answers it.
//!
//! **The match is deliberately exhaustive.** Adding a variant to [`proto::Call`] makes this
//! file fail to compile, so a new method cannot reach the radio because someone forgot this
//! file existed. A `_ => None` wildcard would be the safe default in the moment and the wrong
//! one over time: it would silently deny new methods, and the first symptom would be a phone
//! app that cannot see a feature nobody remembered to route.

use duck_ipc_proto as proto;

/// The service that owns the answer to a call.
///
/// One socket per service, connected directly — there is no broker (`architecture.md` §2.2).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Upstream {
    /// `updaterd`, at `proto::DEFAULT_SOCKET`.
    Updater,
    /// `robotd`.
    Robot,
    /// `configd` — wifi and the robot's identity.
    Config,
}

/// What happens to a call that arrives over BLE.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Route {
    /// Forwarded verbatim to a service.
    To(Upstream),
    /// Answered by `btd` itself. Only `system.authenticate`: the PIN check belongs to the
    /// transport, because BLE cannot express a fixed printed passkey and the check therefore had
    /// to move up a layer (`docs/design/app-path-design.md` §5).
    Local,
    /// Not available over this transport.
    Refused,
}

/// Where this call goes, or `None` if BLE may not make it.
///
/// Read the `None` arms as the security boundary: each one is a deliberate decision that a
/// phone in the room does not get to do this.
pub fn upstream_for(call: &proto::Call) -> Option<Upstream> {
    use proto::Call::*;
    match call {
        // The version handshake. Must be reachable or no client can establish anything.
        Hello(_) => Some(Upstream::Updater),

        // ── the update subset §4.1 names ────────────────────────────────────
        //
        // `Apply` is intended: BLE implies physical presence plus pairing (§4.2), and "update
        // the robot from the phone" is M6's headline. It also has to pass `updaterd`'s own peer
        // policy, and does — `deploy/updater.toml` names `btd` in `allow_users`, which is a
        // narrower claim than granting the robot group. Routing it here without that grant would
        // have produced a phone button that always returned PERMISSION_DENIED.
        Apply(_) => Some(Upstream::Updater),
        Check(_) => Some(Upstream::Updater),
        Status => Some(Upstream::Updater),
        Subscribe => Some(Upstream::Updater),
        // Read-only, and what support asks for first. `update.log` is the record that
        // survives a wiped journal (§8.2), so a phone able to read it is worth having.
        Log(_) => Some(Upstream::Updater),
        ListInstalled(_) => Some(Upstream::Updater),

        // Is the robot alright? The one `robot.*` call an app has any use for.
        RobotHealth => Some(Upstream::Robot),

        // ── provisioning, which is what §4.1 puts BLE here for ──────────────
        //
        // This is the case the whole transport exists to serve: a robot that has never seen a
        // network cannot be configured over that network, so BLE is the only way in. All four
        // are permitted, including the two that change things.
        NetStatus => Some(Upstream::Config),
        NetScan => Some(Upstream::Config),
        // Carries a wifi passphrase, which §7 requires to travel over a paired, encrypted link.
        // It does: the characteristic sets `encrypt_authenticated_write` and the PIN agent makes
        // the bond an authenticated one (`crate::pairing`). Routing this before that existed
        // would have been the ordering mistake.
        NetConnect(_) => Some(Upstream::Config),
        NetForget(_) => Some(Upstream::Config),

        // Name and identity. Renaming from the app is the reason `system.setName` exists.
        SystemInfo => Some(Upstream::Config),
        SystemSetName(_) => Some(Upstream::Config),

        // Rebooting is drastic but recoverable, and it is what an app offers when a robot is
        // confused — the alternative being "unplug it", which for a walking robot is worse.
        // Unlike `resetToGolden` it discards nothing.
        SystemReboot => Some(Upstream::Config),

        // Answered by `btd`, so it has no upstream. See `route_for`.
        SystemAuthenticate(_) => None,

        // The pairing PIN, and the one refusal in this file that is load-bearing rather than
        // conservative: a PIN readable by an unpaired peer authorises nothing at all. `btd`
        // reads it over the unix socket to answer BlueZ's passkey request, and BLE never can.
        SystemPairingPin | SystemSetPairingPin(_) => None,

        // ── refused ─────────────────────────────────────────────────────────

        // Operator surgery. Choosing which installed release runs, or pinning one, is a
        // considered decision made with `robotctl` and a record of who did it — not a
        // mistap in a phone UI.
        Select(_) | Pin(_) => None,

        // Recovery, and deliberately not here *yet*. The engine reverts a bad release on its
        // own (health gate plus boot counter), so the phone needs no button for the ordinary
        // case. Recovery mode (§8.2) is what should reopen this, with its own thinking about
        // what a stranger holding a broken robot is allowed to trigger.
        Rollback(_) => None,

        // Factory reset in all but name: back to the golden image, discarding every release
        // since. Never over a radio.
        ResetToGolden(_) => None,

        // `updaterd`'s private questions to `robotd` — may I restart the control loop, which
        // model API is this, is a telepresence session live. Internal plumbing of the update
        // decision, of no use to a client and misleading if exposed: a phone reading
        // `safeToRestart` would learn nothing it could act on.
        RobotSafeToRestart | RobotModelApi | RobotRemoteSessionActive => None,

        // Motor control. **Never over BLE**, which is what §4.1 means by a subset: BLE is too
        // slow and too constrained for the full surface, and teleop belongs on WebRTC's
        // unreliable `teleop` datachannel where a stale command is dropped rather than
        // retransmitted (§5.2). A 20-byte notification budget and a link that does not exist for
        // the first ~73s of a boot is not a control transport.
        RobotMove(_) | RobotHead(_) | RobotEnable(_) => None,

        // `robot.stop` deserves its own line, because refusing it looks wrong. An emergency stop
        // in the app is exactly what someone reaches for, and §6 does say local should preempt
        // remote — but a stop button that works over an unbonded, high-latency, sometimes-absent
        // radio is worse than no button, because it *looks* like an e-stop and is not one. The
        // deadman in `robotd` already stops the robot when intents stop arriving, which is the
        // mechanism that does not depend on a phone being in range. A real e-stop is physical.
        // Reconsider deliberately if the app ever needs it, with that caveat stated in the UI.
        RobotStop => None,

        // High-rate telemetry. `robot.subscribe` streams state at up to the control rate; over
        // BLE that is a firehose into a 20-byte pipe, and a client would get a decimated,
        // unpredictably-lagged view it could not reason about. `robot.health` is the question an
        // app actually has.
        RobotSubscribe(_) => None,
    }
}

/// The full routing decision, including the one call the transport answers itself.
pub fn route_for(call: &proto::Call) -> Route {
    match call {
        proto::Call::SystemAuthenticate(_) => Route::Local,
        other => match upstream_for(other) {
            Some(upstream) => Route::To(upstream),
            None => Route::Refused,
        },
    }
}

/// The JSON-RPC error to answer a refused call with.
///
/// [`proto::code::PERMISSION_DENIED`] rather than `METHOD_NOT_FOUND`, because the two mean
/// different things to whoever is holding the phone: this method exists and this transport
/// may not use it — "try `robotctl`", not "upgrade your app".
pub fn refusal(call: &proto::Call) -> proto::Error {
    proto::Error::new(
        proto::code::PERMISSION_DENIED,
        format!(
            "{} is not available over Bluetooth; use robotctl on the robot",
            call.method()
        ),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use duck_ipc_proto::{ComponentId, semver};

    fn component() -> ComponentId {
        ComponentId::new("daemon")
    }

    /// Exactly which mutating calls BLE may make, named one by one.
    ///
    /// The list is the security boundary, so it is spelled out rather than counted: adding a
    /// mutating method and routing it should have to change this line and say why in the
    /// commit. `update.apply` is the update trigger §4.1 names; the rest are provisioning,
    /// which is what BLE is *for* — a robot that has never seen a network cannot be configured
    /// over that network.
    #[test]
    fn only_these_mutating_calls_are_reachable_over_ble() {
        let mutating_and_allowed: Vec<&str> = every_call()
            .iter()
            .filter(|c| c.is_mutating() && upstream_for(c).is_some())
            .map(proto::Call::method)
            .collect();

        assert_eq!(
            mutating_and_allowed,
            vec![
                proto::method::APPLY,
                proto::method::NET_CONNECT,
                proto::method::NET_FORGET,
                proto::method::SYSTEM_SET_NAME,
                proto::method::SYSTEM_REBOOT,
            ]
        );
    }

    /// The PIN must never be readable or writable over the radio.
    ///
    /// This is the one refusal here that is not merely cautious: pairing is what authorises a
    /// BLE client at all (§4.2), and a passkey an unpaired peer could ask for — or worse,
    /// overwrite — would make the whole mechanism theatre. `btd` gets it over the unix socket.
    #[test]
    fn the_pairing_pin_is_not_reachable_over_ble() {
        assert_eq!(upstream_for(&proto::Call::SystemPairingPin), None);
        assert_eq!(
            upstream_for(&proto::Call::SystemSetPairingPin(
                proto::SetPairingPinParams {
                    pin: "000000".into()
                }
            )),
            None
        );
    }

    /// Provisioning must be reachable, and reach `configd` — the case BLE exists for.
    #[test]
    fn provisioning_reaches_configd() {
        for call in [
            proto::Call::NetStatus,
            proto::Call::NetScan,
            proto::Call::NetConnect(proto::NetConnectParams {
                ssid: "Home".into(),
                psk: None,
            }),
            proto::Call::NetForget(proto::NetForgetParams {
                ssid: "Home".into(),
            }),
            proto::Call::SystemInfo,
            proto::Call::SystemSetName(proto::SetNameParams {
                name: "duck".into(),
            }),
            proto::Call::SystemReboot,
        ] {
            assert_eq!(
                upstream_for(&call),
                Some(Upstream::Config),
                "{}",
                call.method()
            );
        }
    }

    /// The refusals, named individually. If a future change makes one of these reachable it
    /// should have to delete a line here and say why in the commit.
    #[test]
    fn the_refused_calls_stay_refused() {
        for call in [
            proto::Call::Rollback(proto::ComponentParams {
                component: component(),
            }),
            proto::Call::ResetToGolden(proto::ComponentParams {
                component: component(),
            }),
            proto::Call::Select(proto::SelectParams {
                component: component(),
                version: semver::Version::new(1, 0, 0),
            }),
            proto::Call::Pin(proto::PinParams {
                component: component(),
                version: None,
            }),
            proto::Call::RobotSafeToRestart,
            proto::Call::RobotModelApi,
            proto::Call::RobotRemoteSessionActive,
        ] {
            assert_eq!(upstream_for(&call), None, "{}", call.method());
        }
    }

    /// A phone must be able to establish a session, see the robot's state, start an update
    /// and watch it. Without all four the transport is not useful for what it exists to do.
    #[test]
    fn the_app_path_is_reachable() {
        let expected = [
            (
                proto::Call::Hello(proto::HelloParams {
                    api_version: proto::API_VERSION,
                }),
                Upstream::Updater,
            ),
            (proto::Call::Status, Upstream::Updater),
            (proto::Call::Subscribe, Upstream::Updater),
            (proto::Call::RobotHealth, Upstream::Robot),
        ];
        for (call, want) in expected {
            assert_eq!(upstream_for(&call), Some(want), "{}", call.method());
        }
    }

    /// A refusal must be distinguishable from "no such method", because the two ask the user
    /// for different things.
    #[test]
    fn a_refusal_says_permission_denied_and_names_the_method() {
        let call = proto::Call::ResetToGolden(proto::ComponentParams {
            component: component(),
        });
        let err = refusal(&call);

        assert_eq!(err.code, proto::code::PERMISSION_DENIED);
        assert!(
            err.message.contains(proto::method::RESET_TO_GOLDEN),
            "{}",
            err.message
        );
    }

    /// Every variant, so the tests above cannot silently skip one. The exhaustive match in
    /// `upstream_for` is what forces this list to be maintained: a new variant breaks the
    /// build there, and whoever fixes it arrives here next.
    fn every_call() -> Vec<proto::Call> {
        let version = semver::Version::new(1, 4, 2);
        vec![
            proto::Call::Hello(proto::HelloParams {
                api_version: proto::API_VERSION,
            }),
            proto::Call::Check(proto::ComponentParams {
                component: component(),
            }),
            proto::Call::Apply(proto::ApplyParams {
                component: component(),
                target: proto::Target::Latest,
                options: proto::ApplyOptions::default(),
            }),
            proto::Call::Rollback(proto::ComponentParams {
                component: component(),
            }),
            proto::Call::ResetToGolden(proto::ComponentParams {
                component: component(),
            }),
            proto::Call::Select(proto::SelectParams {
                component: component(),
                version: version.clone(),
            }),
            proto::Call::Pin(proto::PinParams {
                component: component(),
                version: Some(version),
            }),
            proto::Call::Status,
            proto::Call::ListInstalled(proto::ComponentParams {
                component: component(),
            }),
            proto::Call::Log(proto::LogParams { limit: 20 }),
            proto::Call::Subscribe,
            proto::Call::RobotSafeToRestart,
            proto::Call::RobotHealth,
            proto::Call::RobotModelApi,
            proto::Call::RobotRemoteSessionActive,
            proto::Call::NetStatus,
            proto::Call::NetScan,
            proto::Call::NetConnect(proto::NetConnectParams {
                ssid: "Home".into(),
                psk: Some("secret".into()),
            }),
            proto::Call::NetForget(proto::NetForgetParams {
                ssid: "Home".into(),
            }),
            proto::Call::SystemInfo,
            proto::Call::SystemSetName(proto::SetNameParams {
                name: "duck".into(),
            }),
            proto::Call::SystemReboot,
            proto::Call::RobotMove(proto::MoveParams {
                vx: 0.1,
                vy: 0.0,
                vyaw: 0.0,
            }),
            proto::Call::RobotHead(proto::HeadParams {
                neck_pitch: 0.0,
                head_pitch: 0.0,
                head_yaw: 0.0,
                head_roll: 0.0,
            }),
            proto::Call::RobotStop,
            proto::Call::RobotEnable(proto::EnableParams { on: true }),
            proto::Call::RobotSubscribe(proto::SubscribeParams { hz: Some(10) }),
            proto::Call::SystemPairingPin,
            proto::Call::SystemSetPairingPin(proto::SetPairingPinParams {
                pin: "000000".into(),
            }),
        ]
    }
}
