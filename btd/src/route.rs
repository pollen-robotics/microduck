//! Which calls BLE may make, and which of a service's connections carries them.
//!
//! BLE exposes a **subset** of the robot API (`architecture.md` §4.1): provisioning, status,
//! and the update commands with their progress. It is too slow and too constrained for the full
//! surface, and — more to the point — a radio anybody within a few metres can talk to is not
//! the transport over which to offer "reset this robot to factory state".
//!
//! **Two questions, and only one of them is BLE's.** *Which service answers a call, and how long
//! answering holds a connection* is a property of the call, and lives in
//! [`proto::Call::destination`] where every transport reads the same answer. *Whether BLE may make
//! it* is this file. They were one table until a second transport needed the first half and none
//! of the second; `docs/design/remote-webrtc.md` §5 records the split.
//!
//! **The permission match is deliberately exhaustive.** Adding a variant to [`proto::Call`] makes
//! this file fail to compile, so a new method cannot reach the radio because someone forgot this
//! file existed. A `_ => false` wildcard would be the safe default in the moment and the wrong one
//! over time: it would silently deny new methods, and the first symptom would be a phone app that
//! cannot see a feature nobody remembered to route. Every transport needs its own such match for
//! the same reason — a shared one with a wildcard would be the hole in all of them at once.

use duck_ipc_proto as proto;

/// How long a call holds a connection. Defined once, in the protocol crate.
pub use proto::Lane;

/// The service that owns the answer to a call, restricted to the three `btd` holds sockets to.
///
/// Narrower than [`proto::Service`] on purpose. `padd` and `tofd` answer calls too, and `btd` has
/// no connection to either: `padd` is the unprivileged client whose whole value is having no
/// special access, and giving the BLE transport a socket to it would be the first thing to make
/// that untrue. The conversion below therefore *fails* for them, which turns a comment into
/// something the compiler enforces.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Upstream {
    /// `updaterd`, at `proto::DEFAULT_SOCKET`.
    Updater,
    /// `robotd`.
    Robot,
    /// `configd` — wifi and the robot's identity.
    Config,
}

impl TryFrom<proto::Service> for Upstream {
    type Error = ();

    fn try_from(service: proto::Service) -> Result<Self, Self::Error> {
        match service {
            proto::Service::Updater => Ok(Upstream::Updater),
            proto::Service::Robot => Ok(Upstream::Robot),
            proto::Service::Config => Ok(Upstream::Config),
            // Not sockets `btd` holds. Unreachable in practice, because `permits` refuses every
            // call these answer — and an error rather than a panic so that staying true is not
            // something this file has to be careful about.
            proto::Service::Pad | proto::Service::Tof => Err(()),
        }
    }
}

/// What happens to a call that arrives over BLE.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Route {
    /// Forwarded verbatim to a service, on that service's connection for this lane.
    To(Upstream, Lane),
    /// Answered by `btd` itself. Only `system.authenticate`: the PIN check belongs to the
    /// transport, because BLE cannot express a fixed printed passkey and the check therefore had
    /// to move up a layer (`docs/design/app-path-design.md` §5).
    Local,
    /// Not available over this transport.
    Refused,
}

/// May a call arriving over BLE be served at all?
///
/// Read the `false` arms as the security boundary: each one is a deliberate decision that a
/// phone in the room does not get to do this.
fn permits(call: &proto::Call) -> bool {
    use proto::Call::*;
    match call {
        // The version handshake. Must be reachable or no client can establish anything.
        Hello(_) => true,

        // Answered by `btd` itself rather than forwarded. Permitted because it is the one call a
        // session must be able to make before it has made any other.
        SystemAuthenticate(_) => true,

        // ── the update subset §4.1 names ────────────────────────────────────
        //
        // `Apply` is intended: BLE implies physical presence plus pairing (§4.2), and "update
        // the robot from the phone" is M6's headline. It also has to pass `updaterd`'s own peer
        // policy, and does — `deploy/updater.toml` names `btd` in `allow_users`, which is a
        // narrower claim than granting the robot group. Routing it here without that grant would
        // have produced a phone button that always returned PERMISSION_DENIED.
        Apply(_) => true,
        Check(_) => true,
        Status => true,
        Subscribe => true,
        // Read-only, and what support asks for first. `update.log` is the record that
        // survives a wiped journal (§8.2), so a phone able to read it is worth having.
        Log(_) => true,
        // The detail behind one of those log lines, and the same claim: read-only, and the
        // question an owner whose update failed actually has. Bigger than every other reply here
        // — a few kilobytes for an ordinary run, since `hooks::MAX_OUTPUT` bounds the largest
        // part of it, against a `updater::transcript` ceiling of two megabytes for a pathological
        // one — so an app should ask for it on a tap rather than on a refresh. That is the app's
        // decision to make: what this function decides is whether a phone in the room may see it,
        // and a phone that may read the log may read what is behind it.
        Show(_) => true,
        ListInstalled(_) => true,

        // Going back. Both are permitted, and both are less consequential than the `Apply`
        // above them: they move the robot to a release that has already run on this board,
        // download nothing, and are gated and auto-reverted like any other transition
        // (`Engine::rollback` and `Engine::select` both go through `transition_to`).
        //
        // They were refused until the update path was driven from a phone, on the reasoning that
        // the engine reverts a bad release on its own. It does — the one that fails its health
        // gate. That is not the case an owner reaches for a phone about, which is a release that
        // installs, passes its gate, and then behaves *worse*: a policy that walks unsteadily
        // rather than not at all, a pad that stops reconnecting. Nothing reverts that but a
        // person, and the person is holding a phone and has no ssh.
        //
        // `Rollback` is the undo — the previous release, no arguments, one tap. `Select` is the
        // same authority plus a version number, and it is what a list of installed releases is
        // *for*: `ListInstalled` is already routed above, so an app can show them, and being able
        // to show them without being able to choose one would be the odd half.
        Rollback(_) => true,
        Select(_) => true,

        // Is the robot alright? The one `robot.*` call an app has any use for.
        RobotHealth => true,

        // ── provisioning, which is what §4.1 puts BLE here for ──────────────
        //
        // This is the case the whole transport exists to serve: a robot that has never seen a
        // network cannot be configured over that network, so BLE is the only way in. All four
        // are permitted, including the two that change things.
        NetStatus => true,
        NetScan => true,
        // Carries a wifi passphrase, which §7 requires to travel over a paired, encrypted link.
        // It does: the characteristic sets `encrypt_authenticated_write` and the PIN agent makes
        // the bond an authenticated one (`crate::pairing`). Routing this before that existed
        // would have been the ordering mistake.
        NetConnect(_) => true,
        NetForget(_) => true,

        // Name and identity. Renaming from the app is the reason `system.setName` exists.
        SystemInfo => true,
        SystemSetName(_) => true,

        // Which daemons are up and which release each is running. Routed because an app that can
        // trigger an update should be able to show whether it took — and because the one daemon it
        // cannot report on this way is `btd` itself, which answering at all proves is running.
        SystemServices => true,

        // Rebooting is drastic but recoverable, and it is what an app offers when a robot is
        // confused — the alternative being "unplug it", which for a walking robot is worse.
        // Unlike `resetToGolden` it discards nothing.
        SystemReboot => true,

        // ── the gamepad ─────────────────────────────────────────────────────
        //
        // Pairing a controller from the phone, which is where it belongs: whoever is holding the
        // robot is holding the pad, and the alternative is an ssh session. The same physical-presence
        // argument §4.2 makes for `net.connect` covers it — a pad has to be in the room, in pairing
        // mode, in a fifteen-second window — and it is `configd` that does the work either way.
        //
        // `pad.pair` is the more consequential of the two, because a bonded pad can enable the
        // policy afterwards. That is deliberate: it is the same authority as standing next to the
        // robot with a controller, and the PIN gate is what stands in front of it.
        PadStatus => true,
        PadPair(_) => true,
        PadForget(_) => true,

        // ── refused ─────────────────────────────────────────────────────────

        // The pairing PIN, and the one refusal in this file that is load-bearing rather than
        // conservative: a PIN readable by an unpaired peer authorises nothing at all. `btd`
        // reads it over the unix socket to answer BlueZ's passkey request, and BLE never can.
        SystemPairingPin | SystemSetPairingPin(_) => false,

        // Pinning, and it stays refused while `Select` above it does not. The difference is what
        // the mistake looks like afterwards: a wrong `select` is one release away from being
        // undone and the robot says which release it is on, whereas a robot pinned by a mistap
        // refuses every later update and reports itself as up to date. That is the one failure
        // here that looks exactly like correct behaviour, and it needs `robotctl` and a person
        // who meant it.
        Pin(_) => false,

        // Factory reset in all but name: back to the golden image, discarding every release
        // since. Never over a radio — and note that `Rollback` and `Select` being routed does
        // not weaken this, because neither discards anything.
        ResetToGolden(_) => false,

        // `updaterd`'s private questions to `robotd` — may I restart the control loop, which
        // model API is this, is a telepresence session live. Internal plumbing of the update
        // decision, of no use to a client and misleading if exposed: a phone reading
        // `safeToRestart` would learn nothing it could act on.
        RobotSafeToRestart | RobotModelApi | RobotRemoteSessionActive => false,

        // Teleop. **Never over BLE**, which is what §4.1 means by a subset: BLE is too slow and
        // too constrained for the full surface, and teleop belongs on WebRTC's datachannel
        // (`docs/design/remote-webrtc.md` §2). A 20-byte notification budget and a link that does
        // not exist for the first ~73s of a boot is not a control transport. The body pose and
        // the mouth ride with it: all of these are a stream of small updates, and the argument is
        // about the stream, not about any one of them.
        RobotMove(_) | RobotHead(_) | RobotLook(_) | RobotEnable(_) | RobotPose(_)
        | RobotMouth(_) => false,

        // **A skill is not teleop**, and it sat in that group for longer than it deserved. It is
        // one request — "do the bow" — not fifty a second, so the notification budget and the
        // latency argument above simply do not reach it. Nor does it need a control link at all:
        // the deadman zeroes the twist by itself, so a robot with nothing driving it stands still
        // and bows.
        //
        // What the refusals in this file mostly turn on is who is *watching*, and BLE answers
        // that better than anything else here does — the radio reaches about ten metres, so a
        // phone that can send this is in the room with the robot by construction. It is also the
        // authenticated transport: the characteristic takes `encrypt_authenticated_write` and the
        // bond is PIN-checked, which WebRTC has no equivalent of (`remote-webrtc.md` §4).
        //
        // Which skills a robot has is config, so a client asks `robot.policies` rather than
        // assuming a list; an unknown name is refused with the names it does know.
        RobotDo(_) => true,

        // Harmless and rather charming from a phone — but it rides the same refusal as the
        // rest of robot.* until the app path exists to want it: opening one call to the
        // radio ahead of a client that can use it buys nothing and widens the surface.
        //
        // The theremin sits here rather than with motor control even though it moves the
        // mouth, because what it is is a sound: the mouth is following the note. Same
        // refusal either way, and the same reason to lift it — an app that can play the duck.
        RobotSound(_) | RobotTheremin(_) | RobotChorale(_) => false,

        // The chorale's own namespace is between `btd` and `robotd` — it is how this daemon is told
        // what to advertise and how it reports what it heard. Not a client surface at all, so a
        // phone asking for it is asking for something that does not exist for it.
        ChoraleSubscribe | ChoraleBeaconSet(_) | ChoraleHeard(_) => false,

        // Powering the machine off from a phone in the room is `system.reboot` without the
        // coming back. The sit-then-power-off flow wants whoever asked to be watching the
        // robot, and that is `robotctl` or the pad's long-press, deliberately.
        RobotShutdown => false,

        // Only a stick-mapping hint for local clients like `padd`. An app gets the same answer
        // through `system.info` territory when it ever needs one; no reason to open another read
        // to the radio today.
        RobotMode => false,

        // Switching modes means the robot goes home, loads other policies and drives differently
        // — and the reason to switch is that somebody just put wheels on it. That is a decision
        // made in the room, holding the pad, the same place `robot.shutdown` is refused for.
        RobotSetMode(_) => false,

        // Loading a policy is `robot.setMode` with a wider blast radius: it puts an arbitrary
        // `.onnx` in charge of fifteen servos, and that file can come from a stranger on the Hub.
        // Everything that makes it survivable — the shape gate, the clamps, the fall reflex — is
        // unchanged whoever asked, so this was never about danger; it was that trying a gait
        // means watching the robot try it, and there was no client that did.
        //
        // Both halves have since turned. There is a client, and BLE is the transport that best
        // meets the watching condition: ten metres of radio range means whoever tapped it is
        // looking at the robot, and the bond is PIN-authenticated. A load that fails keeps the
        // controller that was running, so the failure mode is "nothing happened", not "gaitless
        // robot on the floor".
        //
        // **This method persists.** `robotd` writes the slot key into `robotd.toml` before it
        // queues the swap, so a gait chosen from a phone is the gait the robot boots into. It
        // used to be `robotctl`'s half alone, which made the command and the method the same
        // words with different durability depending on who asked — the ephemeral "try it until
        // reboot" mode `policy-channel-design.md` §3 rejected, arrived at by accident.
        //
        // So this is a durable remote change, like `pad.bind` below, and belongs on the
        // PIN-bonded transport for that reason as much as for the watching one. Undoing it is
        // `robot.loadPolicy` with no path, which is reachable from the same phone.
        RobotLoadPolicy(_) => true,

        // Which button runs which skill, and changing one. **This is the transport those exist
        // for**: `robotctl pad bind` is for whoever is holding the robot, and a phone that can
        // already ask for a skill is the obvious place to decide which button asks for it.
        //
        // `pad.bind` writes the config file, and has to: `padd` re-reads `[pad]` every second,
        // so a binding held in memory would be reverted before the caller let go of the phone.
        //
        // A name is checked against the skills this robot has before anything is written, so the
        // failure mode is a refusal naming them rather than a button that does nothing.
        PadBindings | PadBind(_) => true,

        // The skill table: what this robot can be asked to do, and adding to it. The last thing
        // in the policy path that only a terminal on the robot could reach — `[[policy.skill]]`
        // is a repeating table, so `robotctl policy add` wrote it directly and there was nothing
        // to route.
        //
        // `robotd` writes the file and reloads itself, so one call is the whole operation. A
        // client that had to remember a second one and forgot would leave a robot whose config
        // and behaviour disagree until the next restart.
        RobotSkills | RobotSetSkill(_) | RobotRemoveSkill(_) => true,

        // What each slot is running, and which skills this robot has. Read-only, and the read a
        // client makes before it can offer either of the two above: there is no compiled-in list
        // of skills to assume any more, so this is how a phone knows there is a bow to ask for.
        RobotPolicies => true,

        // Re-reading the slots after something else edited the config. Same blast radius as
        // `robot.loadPolicy` and the same answer, and a client that can load wants this for the
        // case where the file changed underneath it.
        RobotReloadPolicies => true,

        // Is there a newer official policy set, and what else is on the Hub. Both reach the
        // network, and neither changes anything on the robot — the same kind of question as
        // `update.check`, which has been routed here since the update path was driven from a
        // phone.
        //
        // They were refused on the grounds that answering "yes, there is a newer gait" for a
        // client that could not then install one is an odd thing to offer. That was fair while it
        // was true and stops being an argument the moment `policy.install` is routed, which is
        // the next arm.
        PolicyCheck | PolicySearch(_) => true,

        // And installing one. This is the most obviously *appealing* thing in this file: a
        // stranger's gait, from a phone, onto the robot in front of you.
        //
        // What makes it survivable is unchanged and was always the point — the manifest gate
        // before the download, the shape gate at load, the joint clamps, the fall reflex — so
        // this was never about danger. It was about who is *watching*, and BLE answers that
        // better than anything else here: ten metres of radio range means whoever tapped it is
        // looking at the robot, and the bond is PIN-checked.
        //
        // `policy.install` is `is_mutating`, so `updaterd` authorises it against the peer's
        // uid — which is `btd`'s, not the phone's, exactly as it already is for `update.apply`.
        // The transport is the gate here, not the credential.
        PolicyFetch(_) | PolicyInstall(_) => true,

        // ── the account, which BLE is the right transport for ────────────────
        //
        // Signing the robot in to a Hugging Face account is what makes it reachable from outside
        // the LAN at all, and this is the transport that should carry it — for the reason
        // `remote-webrtc.md` §4 gives about BLE generally: it is PIN-bonded, and ten metres of
        // radio range means whoever tapped the button is in the room with the robot.
        //
        // It also happens to be the *only* transport that can do it on a robot fresh out of a
        // box. A duck with no wifi has no console to open and no LAN to open it from, and BLE is
        // already how such a robot is given a network — so a wizard that joins the wifi and then
        // signs the robot in is one flow on one transport, which is what the mini's app does.
        //
        // The code the flow produces has to be *read by a person*, so the reply carrying it is
        // the point: this call sends about eighty bytes back, well inside what framing chunks,
        // and needs no link at all while the user is off approving it — see
        // `duck_ipc_proto::API_VERSION`'s v23 note on why `login` answers with a code and not
        // with a token. That is what makes an iPhone dropping the GATT link mid-flow a
        // non-event rather than a lost login.
        AccountLogin(_) | AccountStatus | AccountLogout => true,

        // Power to the joints. A phone button that drops the robot on the floor is not one to
        // offer, and `robot.init` is its counterpart: standing a robot up moves every joint at once,
        // which wants the person doing it to be looking at the robot rather than at a screen. Both
        // are `robotctl` on the robot, deliberately.
        RobotInit | RobotRelax => false,

        // `robot.stop` deserves its own line, because refusing it looks wrong. An emergency stop
        // in the app is exactly what someone reaches for, and §6 does say local should preempt
        // remote — but a stop button that works over an unbonded, high-latency, sometimes-absent
        // radio is worse than no button, because it *looks* like an e-stop and is not one. The
        // deadman in `robotd` already stops the robot when intents stop arriving, which is the
        // mechanism that does not depend on a phone being in range. A real e-stop is physical.
        // Reconsider deliberately if the app ever needs it, with that caveat stated in the UI.
        RobotStop => false,

        // High-rate telemetry. `robot.subscribe` streams state at up to the control rate; over
        // BLE that is a firehose into a 20-byte pipe, and a client would get a decimated,
        // unpredictably-lagged view it could not reason about. `robot.health` is the question an
        // app actually has.
        RobotSubscribe(_) => false,

        // The same objection as `robot.subscribe`, only more so: this is every evdev event the pad
        // sends, over a hundred reports a second, and it exists to *measure the cadence of its own
        // delivery*. Carried over BLE the measurement would be of the phone's link rather than the
        // pad's, which is worse than refusing — it would be a number that looks like an answer.
        //
        // It is also not `btd`'s to forward: `padd` is deliberately not one of the sockets `btd`
        // holds, which `Upstream`'s conversion now enforces rather than merely documents.
        PadInput => false,

        // Depth frames, and the same two objections as the pad tap. A 64-zone frame
        // fifteen times a second is a firehose into a 20-byte pipe; and it is served by
        // `tofd`, which is not one of the sockets `btd` holds. When a phone has a
        // reason to see what the robot sees, it will be through `mediad`'s video path
        // (`architecture.md` §5.2), where depth belongs next to the frame it annotates.
        TofStream => false,
    }
}

/// Where this call goes and on which connection, or `None` if BLE may not make it — or if no
/// service answers it, which for `system.authenticate` is the same answer with a different reason.
/// [`route_for`] tells those two apart.
pub fn destination_for(call: &proto::Call) -> Option<(Upstream, Lane)> {
    if !permits(call) {
        return None;
    }
    let (service, lane) = call.destination()?;
    Some((Upstream::try_from(service).ok()?, lane))
}

/// The service that answers a call, ignoring which connection carries it.
///
/// The permission question on its own, which is what most callers and every test about the
/// security boundary are asking.
pub fn upstream_for(call: &proto::Call) -> Option<Upstream> {
    destination_for(call).map(|(upstream, _)| upstream)
}

/// The full routing decision, including the one call the transport answers itself.
pub fn route_for(call: &proto::Call) -> Route {
    match call {
        proto::Call::SystemAuthenticate(_) => Route::Local,
        other => match destination_for(other) {
            Some((upstream, lane)) => Route::To(upstream, lane),
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
    // The shared list, not a local copy. Two copies of this had already drifted — 115 lines here
    // against 82 — which is how `pad.input` came to be missing from one of them.
    use duck_ipc_proto::test_support::every_call;
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
                // Going back, both of them. Routed when the update path was driven from a phone:
                // an owner whose robot got worse after an update has no other way to undo it, and
                // neither call discards anything or downloads anything.
                proto::method::ROLLBACK,
                proto::method::SELECT,
                // Replacing the policy set, and fetching a stranger's policy onto the board.
                // Routed for the reason provisioning is: BLE's physical-presence claim (§4.2)
                // holds — ten metres of radio range, a PIN-checked bond — and trying a gait is
                // the thing that most wants whoever asked to be looking at the robot. Everything
                // that makes it survivable is the same whoever asked: the manifest gate before
                // the download, the shape gate at load, the clamps, the fall reflex.
                proto::method::POLICY_INSTALL,
                proto::method::POLICY_FETCH,
                // Binding the robot to a Hugging Face account, and unbinding it. Provisioning,
                // like the two below it and for the same reason: a robot out of a box has no
                // network, so it has no console and no LAN to open one from, and this is the
                // transport that reaches it. Physical presence is not stretched either — the
                // person approving the code is holding the phone that is bonded to the robot.
                proto::method::ACCOUNT_LOGIN,
                proto::method::ACCOUNT_LOGOUT,
                proto::method::NET_CONNECT,
                proto::method::NET_FORGET,
                proto::method::SYSTEM_SET_NAME,
                proto::method::SYSTEM_REBOOT,
                // Bonding a gamepad, which afterwards can enable the walking policy. Allowed for
                // the same reason as provisioning: it takes a pad held in pairing mode next to the
                // robot, so BLE's physical-presence claim (§4.2) is not being stretched — and the
                // alternative is an ssh session, which is not a thing an owner has.
                proto::method::PAD_PAIR,
                proto::method::PAD_FORGET,
            ]
        );
    }

    /// Pairing a controller from the phone reaches `configd`, which is the service that owns the
    /// radio's configuration. `btd` must not answer this itself: it owns nothing (§4.1).
    #[test]
    fn a_pad_can_be_paired_from_the_phone() {
        for call in [
            proto::Call::PadStatus,
            proto::Call::PadPair(proto::PadPairParams::default()),
            proto::Call::PadForget(proto::PadForgetParams {
                mac: "78:86:2E:BB:13:28".into(),
            }),
        ] {
            assert_eq!(
                upstream_for(&call),
                Some(Upstream::Config),
                "{}",
                call.method()
            );
        }
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
    ///
    /// Two lines were deleted from it when the update path was driven from a phone —
    /// `update.rollback` and `update.select` — and the reasoning is on their arms in
    /// `destination_for`. What is left is a factory reset, a pin whose mistake looks like correct
    /// behaviour, and `updaterd`'s private questions to `robotd`.
    #[test]
    fn the_refused_calls_stay_refused() {
        for call in [
            proto::Call::ResetToGolden(proto::ComponentParams {
                component: component(),
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

    /// **A phone can ask for a skill, see what the robot has, and change what it runs.**
    ///
    /// The three together, because none of them is much use alone: `robot.policies` is how a
    /// client learns there is a bow to ask for — which skills exist is config, so there is no
    /// list to compile in — and `robot.do` is the asking.
    #[test]
    fn a_phone_can_run_a_skill_and_change_a_policy() {
        let expected = [
            proto::Call::RobotPolicies,
            proto::Call::RobotDo(proto::DoParams {
                skill: "polite-bow".to_owned(),
            }),
            proto::Call::RobotLoadPolicy(proto::LoadPolicyParams {
                slot: Some("walk".to_owned()),
                path: Some("/opt/robot/policies/current/alpha_walking.onnx".to_owned()),
            }),
            proto::Call::RobotReloadPolicies,
            // The pair `robotctl pad bind` had no wire surface for at all.
            proto::Call::PadBindings,
            proto::Call::PadBind(proto::PadBindParams {
                button: "x".to_owned(),
                skill: Some("polite-bow".to_owned()),
            }),
            // The skill table — the last thing here only a terminal on the robot could reach.
            proto::Call::RobotSkills,
            proto::Call::RobotSetSkill(proto::SkillParams::default()),
            proto::Call::RobotRemoveSkill(proto::SkillNameParams {
                name: "polite-bow".to_owned(),
            }),
        ];
        for call in expected {
            assert_eq!(
                upstream_for(&call),
                Some(Upstream::Robot),
                "{}",
                call.method()
            );
        }
    }

    /// **The Hub, from a phone.** What is out there, whether the official set has moved, and
    /// installing one — the four that reach the network, all on `updaterd` beside `update.check`.
    ///
    /// `policy.install` and `policy.fetch` are mutating, so they are also named one by one in
    /// [`only_these_mutating_calls_are_reachable_over_ble`]; this is the half that says they
    /// arrive somewhere.
    #[test]
    fn a_phone_can_browse_and_install_from_the_hub() {
        for call in [
            proto::Call::PolicyCheck,
            proto::Call::PolicySearch(proto::PolicySearchParams {
                query: "microduck".to_owned(),
            }),
            proto::Call::PolicyInstall(proto::PolicyInstallParams::default()),
        ] {
            assert_eq!(
                upstream_for(&call),
                Some(Upstream::Updater),
                "{}",
                call.method()
            );
        }
    }

    /// **A skill is not teleop, and teleop is still refused.**
    ///
    /// `robot.do` spent a while grouped with these, and the distinction is the whole reason it
    /// could be opened: one request against a stream of fifty a second. If somebody ever moves
    /// `robot.move` into that arm by widening a pattern, this is what says no.
    #[test]
    fn teleop_stays_off_the_radio() {
        for call in [
            proto::Call::RobotMove(proto::MoveParams {
                vx: 0.0,
                vy: 0.0,
                vyaw: 0.0,
            }),
            proto::Call::RobotHead(proto::HeadParams {
                neck_pitch: 0.0,
                head_pitch: 0.0,
                head_yaw: 0.0,
                head_roll: 0.0,
            }),
            proto::Call::RobotEnable(proto::EnableParams {
                on: true,
                toggle: false,
            }),
        ] {
            assert_eq!(upstream_for(&call), None, "{}", call.method());
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

    /// Nothing a phone does during an update may share a connection with the update.
    ///
    /// This is the defect the lanes exist for, and it is asserted as a property rather than as a
    /// table: whatever else changes, `update.apply` must not be able to block a status poll, a
    /// check, or the progress stream, because every daemon here serves one connection one request
    /// at a time. The three calls below are the three an app makes *while* an update runs.
    #[test]
    fn an_apply_shares_its_connection_with_nothing_a_client_does_during_one() {
        let apply = destination_for(&proto::Call::Apply(proto::ApplyParams {
            component: component(),
            target: proto::Target::Latest,
            options: proto::ApplyOptions::default(),
        }))
        .expect("apply is routed");

        for call in [
            proto::Call::Status,
            proto::Call::Subscribe,
            proto::Call::Check(proto::ComponentParams {
                component: component(),
            }),
        ] {
            let during = destination_for(&call).expect("routed");
            assert_eq!(during.0, apply.0, "{} is served by updaterd", call.method());
            assert_ne!(
                during.1,
                apply.1,
                "{} would queue behind an apply",
                call.method()
            );
        }
    }

    /// The progress stream must be alone on its lane, which is a stronger claim than the test
    /// above: a connection handed to `stream_progress` reads no further requests *ever*, so a
    /// second call sharing it is not delayed but lost.
    #[test]
    fn nothing_else_travels_on_the_stream_lane() {
        let others: Vec<&str> = every_call()
            .iter()
            .filter(|c| !matches!(c, proto::Call::Subscribe))
            .filter(|c| destination_for(c).is_some_and(|(_, lane)| lane == Lane::Stream))
            .map(proto::Call::method)
            .collect();

        assert_eq!(others, Vec::<&str>::new());
        assert_eq!(
            destination_for(&proto::Call::Subscribe).map(|(_, lane)| lane),
            Some(Lane::Stream)
        );
    }

    /// A call that holds its connection for as long as the robot needs is never on the lane the
    /// quick answers use. Named one by one, because the cost of getting one wrong is a session
    /// that stops answering and the fix is one word.
    #[test]
    fn the_calls_that_take_their_time_are_off_the_prompt_lane() {
        for call in [
            proto::Call::Apply(proto::ApplyParams {
                component: component(),
                target: proto::Target::Latest,
                options: proto::ApplyOptions::default(),
            }),
            proto::Call::Rollback(proto::ComponentParams {
                component: component(),
            }),
            proto::Call::Select(proto::SelectParams {
                component: component(),
                version: semver::Version::new(1, 0, 0),
            }),
            proto::Call::Check(proto::ComponentParams {
                component: component(),
            }),
            proto::Call::NetScan,
            proto::Call::NetConnect(proto::NetConnectParams {
                ssid: "Home".into(),
                psk: None,
            }),
            proto::Call::PadPair(proto::PadPairParams::default()),
        ] {
            let (_, lane) = destination_for(&call).expect("routed");
            assert_ne!(lane, Lane::Prompt, "{}", call.method());
        }
    }

    /// Going back is reachable, and reaches `updaterd`. The pair of them is what §2.4 of
    /// `docs/project/update-over-ble.md` decided.
    #[test]
    fn going_back_is_reachable_from_the_phone() {
        for call in [
            proto::Call::Rollback(proto::ComponentParams {
                component: component(),
            }),
            proto::Call::Select(proto::SelectParams {
                component: component(),
                version: semver::Version::new(0, 5, 1),
            }),
        ] {
            assert_eq!(
                destination_for(&call),
                Some((Upstream::Updater, Lane::Operation)),
                "{}",
                call.method()
            );
        }
    }

    /// A permitted call must be one `btd` can actually deliver.
    ///
    /// This is the test the split made necessary. Permission and destination are now decided in
    /// two places, so it became possible to permit a call that `btd` holds no socket for —
    /// `pad.input` and `tof.stream` are served by `padd` and `tofd`, and `btd` connects to
    /// neither. Before, one table answered both questions and the mistake could not be written.
    ///
    /// `system.authenticate` is the deliberate exception: permitted, and answered by `btd` itself
    /// rather than forwarded, which is exactly what `route_for` reports as `Local`.
    #[test]
    fn everything_permitted_is_deliverable() {
        for call in every_call() {
            if !permits(&call) {
                continue;
            }
            if matches!(call, proto::Call::SystemAuthenticate(_)) {
                assert_eq!(
                    route_for(&call),
                    Route::Local,
                    "system.authenticate must be answered by btd itself"
                );
                continue;
            }
            assert!(
                destination_for(&call).is_some(),
                "{} is permitted over BLE but btd cannot deliver it — it is served by a socket \
                 btd does not hold, so either permit it and give btd that socket, or refuse it",
                call.method()
            );
        }
    }

    /// And the converse: a refused call must not be deliverable, whatever the shared table says.
    ///
    /// Cheap, and it pins the composition order. `destination_for` consulting the shared
    /// destination *before* the permission check would pass every other test in this file and
    /// quietly route the whole API to the radio.
    #[test]
    fn nothing_refused_is_deliverable() {
        for call in every_call() {
            if permits(&call) {
                continue;
            }
            assert_eq!(
                destination_for(&call),
                None,
                "{} is refused over BLE but destination_for offered a route",
                call.method()
            );
            assert_eq!(route_for(&call), Route::Refused, "{}", call.method());
        }
    }
}
