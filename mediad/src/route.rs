//! Which calls a WebRTC peer may make.
//!
//! The sibling of `btd::route`, and deliberately structured the same way: *which service answers a
//! call and how long answering holds a connection* lives once, in [`proto::Call::destination`],
//! and this file answers only *may a peer over this transport ask it*.
//! `docs/design/remote-webrtc.md` §5 records why the two were split.
//!
//! **The match is exhaustive, and that is the point of having one per transport.** Adding a
//! variant to [`proto::Call`] fails the build here as well as in `btd`, so a new method cannot
//! reach a remote peer because nobody remembered this file. A shared table with a `_` wildcard
//! would have been the hole in both transports at once.
//!
//! ## Why this subset is wider than BLE's
//!
//! BLE's is narrow for two reasons that do not apply here: the radio is slow — a 20-byte
//! notification budget — and anyone within a few metres can talk to it. A datachannel is neither
//! slow nor limited to the room, so the calls BLE refuses *on capacity grounds* are exactly the
//! ones this transport exists to carry: intents, telemetry, the pad tap, the depth stream.
//!
//! What stays out falls into three groups, and the reasons are different in kind:
//!
//! 1. **It authorises a different transport.** The pairing PIN. A peer that can rewrite it can
//!    lock a phone out of BLE, which is the recovery path.
//! 2. **It would drop the session it was asked over.** The update mutations, and reconfiguring
//!    wifi. §8 covers what update needs before it can be permitted; it is a deferral rather than
//!    a rule.
//! 3. **It is not a client's question.** `updaterd`'s internal queries to `robotd`.
//!
//! ## What it does *not* decide
//!
//! Whether the peer is allowed to be here at all. §4: there is no authorisation on the robot —
//! a LAN peer may drive it, and a bridged peer has already authenticated to the rendezvous
//! service on both sides. This file is about which calls exist over the transport, not about who
//! is holding it.

use duck_ipc_proto as proto;

/// Where a call arriving over a WebRTC datachannel goes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Route {
    /// Forwarded verbatim to a service, on that service's connection for this lane.
    To(proto::Service, proto::Lane),
    /// Not available over this transport.
    Refused,
}

/// May a call arriving over a WebRTC datachannel be served?
///
/// Read the `false` arms as the boundary. Each is a decision that a peer holding a session does
/// not get to do this, and each says which of the three kinds of reason it is.
fn permits(call: &proto::Call) -> bool {
    use proto::Call::*;
    match call {
        // The version handshake. Must be reachable or no client can establish anything.
        Hello(_) => true,

        // ── the robot, which is what this transport is for ───────────────────
        //
        // Every one of these is refused over BLE on capacity grounds — "a 20-byte notification
        // budget and a link that does not exist for the first ~73s of a boot is not a control
        // transport". A datachannel is a control transport, so this is the transport those
        // refusals were pointing at.
        RobotMove(_) | RobotHead(_) | RobotLook(_) | RobotPose(_) | RobotMouth(_) => true,
        // Raw per-joint targets from an off-robot controller — the External drive stream. This is
        // exactly the case the doc's "the calls BLE refuses on capacity grounds are the ones this
        // transport exists to carry" points at: a joint stream is high-rate teleop, and the
        // datachannel is its transport. The peer holding the session has the camera — it is
        // looking at the robot — and the safety layer (anatomical clamp, per-tick step, external
        // deadman) bounds every frame regardless. Refused over BLE, permitted here.
        RobotSetJoints(_) => true,
        // The theremin rides with the sounds: it is one, and a browser that can quack a duck
        // may pick its instrument up too.
        RobotDo(_) | RobotSound(_) | RobotTheremin(_) => true,

        // The chorale is between robots, over BLE — a browser is neither in the room nor a duck.
        // Its daemon-to-daemon plumbing has even less business on a WebRTC channel.
        RobotChorale(_) | ChoraleSubscribe | ChoraleBeaconSet(_) | ChoraleHeard(_) => false,
        RobotHealth | RobotMode => true,

        // Telemetry at up to the control rate. BLE called this "a firehose into a 20-byte pipe",
        // which it is; here it is a stream on a channel built for streams.
        RobotSubscribe(_) => true,

        // Power to the joints, and standing up — both of which move every joint at once, and both
        // of which BLE refuses because they want "the person doing it to be looking at the robot
        // rather than at a screen".
        //
        // **That condition is met here rather than waived.** A peer holding this session has the
        // camera: it is looking at the robot, which is precisely what a phone in the room over
        // Bluetooth was not. Permitted for that reason, and it would be worth revisiting if a
        // control-only session without video ever becomes a normal thing.
        RobotEnable(_) | RobotInit | RobotRelax => true,

        // `robot.stop` is permitted here and refused over BLE, and the difference is honesty
        // rather than authority. BLE's objection was that a stop button over "an unbonded,
        // high-latency, sometimes-absent radio" is worse than no button because it *looks* like an
        // e-stop and is not one. The `control` channel is reliable and ordered, and the deadman
        // already stops the robot when intents stop arriving — so a stop here does what the button
        // says. It is still not a physical e-stop, and the UI should not imply it is.
        RobotStop => true,

        // Sit down, then power off. Same argument as `robot.init`: the peer can see the robot sit,
        // which is the condition BLE could not meet.
        RobotShutdown => true,

        // Not this one, even though the peer can watch. A mode switch says "this duck now has
        // wheels on it", which is a claim about hardware only somebody in the room can make —
        // and getting it wrong drives the robot with the wrong policy. The pad in that room is
        // where it belongs.
        RobotSetMode(_) => false,

        // ── the streams BLE pointed here ────────────────────────────────────
        //
        // `pad.input` exists to measure the cadence of its own delivery, and `btd` refuses it
        // partly because over BLE "the measurement would be of the phone's link rather than the
        // pad's". A datachannel does not have that problem to the same degree, and `mediad` may
        // hold a socket to `padd` — `btd` deliberately may not, being the transport with a claim
        // to privilege that `padd` is defined by not having.
        PadInput => true,
        // And the depth stream, which `btd`'s own refusal names this transport as the home for:
        // "it will be through `mediad`'s video path, where depth belongs next to the frame it
        // annotates".
        TofStream => true,

        // ── reading the robot's software ─────────────────────────────────────
        //
        // All read-only, all useful to a remote client, none of them touching `current`.
        // `Show` is the largest reply in this list — a run's whole transcript — and a
        // datachannel is the one transport here with the room for it.
        Check(_) | Status | Subscribe | Log(_) | Show(_) | ListInstalled(_) => true,

        // ── identity and status ─────────────────────────────────────────────
        SystemInfo | SystemServices | SystemSetName(_) => true,
        // Drops this session, and unlike an update leaves nothing mid-transition: the robot comes
        // back and the client reconnects. It is what you offer a confused robot.
        SystemReboot => true,
        // Read-only network state. Useful for showing a remote operator why the link is poor.
        NetStatus | NetScan => true,
        PadStatus | PadForget(_) => true,

        // ── refused: it authorises a different transport ─────────────────────
        //
        // Not because it would compromise *this* transport — §4 leaves that open by design — but
        // because a peer that can rewrite the pairing PIN can lock a phone out of BLE, which is
        // the recovery path. The PIN stays off every network transport for the same reason it is
        // unroutable to BLE itself.
        SystemPairingPin | SystemSetPairingPin(_) => false,

        // ── refused: it would drop the session it was asked over ─────────────
        //
        // Applying, rolling back or selecting a release restarts `mediad`, which drops the session
        // the client is watching progress on. **A deferral, not a rule** — a phone updating a
        // robot over WebRTC is wanted, and `remote-webrtc.md` §8 lists the two things it needs: a
        // client that reconnects and re-subscribes, and `RobotRemoteSessionActive` learning to
        // tell a bystander session from the one that requested the update.
        Apply(_) | Rollback(_) | Select(_) => false,

        // Reconfiguring wifi moves the robot to another network and takes this session with it.
        // And unlike the update case there is nothing to defer *to*: provisioning is what BLE
        // exists for, because "a robot that has never seen a network cannot be configured over
        // that network" — while a robot reachable over WebRTC demonstrably has one.
        NetConnect(_) | NetForget(_) => false,

        // Bonding a gamepad needs a pad in the room, in pairing mode, in a fifteen-second window.
        // A remote peer cannot satisfy any of that, so permitting it would only offer a button
        // that times out.
        PadPair(_) => false,

        // ── refused: never over a network transport ──────────────────────────
        //
        // Factory reset in all but name: back to the golden image, discarding every release since.
        // `Rollback` being merely deferred above does not weaken this, because rollback discards
        // nothing.
        ResetToGolden(_) => false,

        // Pinning, and it stays refused where `Select` is only deferred. A wrong `select` is one
        // release away from being undone and the robot says which release it is on; a robot pinned
        // by mistake refuses every later update and reports itself as up to date. That is the one
        // failure here that looks exactly like correct behaviour, and it needs `robotctl` and a
        // person who meant it.
        Pin(_) => false,

        // ── refused: not a client's question ────────────────────────────────
        //
        // `updaterd`'s private queries to `robotd`. Internal plumbing of the update decision, of
        // no use to a client and misleading if exposed.
        RobotSafeToRestart | RobotModelApi | RobotRemoteSessionActive => false,

        // ── refused: this transport does not authenticate ────────────────────
        //
        // Answering would be a lie. §4: there is no gate here — a LAN peer may drive the robot,
        // and a bridged one authenticated to the rendezvous service before arriving. Since no
        // service answers this call either ([`proto::Call::destination`] returns `None` for it),
        // there is nothing to forward and nothing to check, so it is refused by name rather than
        // silently succeeding. If a gate is ever added, this is the line that changes.
        SystemAuthenticate(_) => false,
    }
}

/// Where this call goes, or that it is refused.
pub fn route_for(call: &proto::Call) -> Route {
    if !permits(call) {
        return Route::Refused;
    }
    match call.destination() {
        Some((service, lane)) => Route::To(service, lane),
        // Permitted but owned by no service. Only `system.authenticate` is in that position, and
        // `permits` refuses it — so this is unreachable, and refusing rather than panicking keeps
        // it that way without this function having to be careful.
        None => Route::Refused,
    }
}

/// The refusal a peer gets, naming the method so a client can report which call was declined.
pub fn refusal(call: &proto::Call) -> proto::Error {
    proto::Error::new(
        proto::code::METHOD_NOT_FOUND,
        format!("{} is not available over WebRTC", call.method()),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Everything permitted must be deliverable, for the reason `btd`'s twin of this test exists:
    /// permission and destination are decided in two places, so permitting a call no service
    /// answers became writable.
    #[test]
    fn everything_permitted_is_deliverable() {
        for call in proto::test_support::every_call() {
            if !permits(&call) {
                continue;
            }
            assert!(
                matches!(route_for(&call), Route::To(..)),
                "{} is permitted over WebRTC but routes nowhere",
                call.method()
            );
        }
    }

    /// And nothing refused is deliverable. This pins the composition order: consulting
    /// `destination` before `permits` would pass every other test here and route the whole API to
    /// any peer that connected.
    #[test]
    fn nothing_refused_is_deliverable() {
        for call in proto::test_support::every_call() {
            if permits(&call) {
                continue;
            }
            assert_eq!(
                route_for(&call),
                Route::Refused,
                "{} is refused but routes somewhere",
                call.method()
            );
        }
    }

    /// The two calls that must never reach a network transport, whatever else changes.
    ///
    /// Named individually rather than checked as a group: this is the list that a later widening
    /// of the subset must not quietly grow past, and a test that says "some things are refused"
    /// would not notice.
    #[test]
    fn the_pin_and_the_factory_reset_are_never_available() {
        for call in proto::test_support::every_call() {
            if matches!(
                call,
                proto::Call::SystemPairingPin
                    | proto::Call::SystemSetPairingPin(_)
                    | proto::Call::ResetToGolden(_)
                    | proto::Call::Pin(_)
            ) {
                assert_eq!(route_for(&call), Route::Refused, "{}", call.method());
            }
        }
    }

    /// The calls this transport exists for. If a refactor ever refuses one of these, the feature
    /// has stopped working and the test should say so in those terms.
    #[test]
    fn the_control_surface_this_transport_exists_for_is_available() {
        for call in proto::test_support::every_call() {
            let wanted = matches!(
                call,
                proto::Call::RobotMove(_)
                    | proto::Call::RobotHead(_)
                    | proto::Call::RobotLook(_)
                    | proto::Call::RobotStop
                    | proto::Call::RobotSubscribe(_)
                    | proto::Call::TofStream
                    | proto::Call::PadInput
            );
            if wanted {
                assert!(
                    matches!(route_for(&call), Route::To(..)),
                    "{} is what WebRTC is for and it is refused",
                    call.method()
                );
            }
        }
    }

    /// A WebRTC peer reaches services `btd` deliberately holds no socket to. Worth pinning,
    /// because it is the concrete difference between the two transports' needs: `mediad` will hold
    /// five connections where `btd` holds three.
    #[test]
    fn reaches_padd_and_tofd_which_btd_cannot() {
        let mut seen = std::collections::BTreeSet::new();
        for call in proto::test_support::every_call() {
            if let Route::To(service, _) = route_for(&call) {
                seen.insert(format!("{service:?}"));
            }
        }
        for service in ["Updater", "Robot", "Config", "Pad", "Tof"] {
            assert!(
                seen.contains(service),
                "no permitted call reaches {service}; seen {seen:?}"
            );
        }
    }
}
