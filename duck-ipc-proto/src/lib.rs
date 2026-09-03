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
/// because the version's job is to say "these two peers were not built together". v6 added
/// `robot.init` and `robot.relax`, so powering the joints stops being a subcommand that fights the
/// daemon for the motor bus. During prototyping the wire shape simply changes and this bumps; no
/// accommodation is made for peers that predate a field, because there are none in the field yet.
///
/// # The rule, since a bump has consequences and nothing else states them
///
/// **A bump promises nothing in either direction.** It is not "additive unless stated": v5 was
/// additive and v4 was not, and the constant does not distinguish them. Every binary on a board is
/// expected to come from one release, and the install path is what delivers that.
///
/// **No daemon refuses a call because this number differs.** `updaterd` used to refuse `hello` on
/// an exact `!=`. The premise was sound — a bump promises nothing — and the conclusion did not
/// follow from it: what actually breaks a mismatched peer is a *route it cannot reach* or a
/// *parameter shape that moved*, and each of those refuses itself, by name, on the one call that
/// cannot be served. A gate on the handshake fired on all of them instead, including the calls that
/// were perfectly serveable and including `update apply`, which is how a skew ends. The difference
/// is now *reported* — in `updaterd`'s journal, and in [`HelloResult::api_version`] for a client to
/// compare against its own — and refused nowhere. See `updater/src/ipc.rs`.
///
/// **What refuses instead, and it is narrower on purpose.** A method this release does not have is
/// [`code::METHOD_NOT_FOUND`] naming the method ([`Request::as_call`]). A member of `params` this
/// release does not know is [`code::INVALID_PARAMS`] naming the member, because every params type
/// denies unknown fields — which is the strictness the handshake was reaching for, moved to where it
/// can tell a changed call from an unchanged one. Both were wanted regardless of the gate: v7 added
/// `ApplyOptions::from_dir`, and an older `updaterd` that merely *ignored* it would have installed
/// from its configured source while the operator believed they were sideloading a directory. Silence
/// was the danger there, not disagreement. It reaches only forward — a daemon built before this
/// cannot refuse what it was already ignoring — which is one more reason the bump is still worth
/// making.
///
/// **What a bump therefore costs.** Two peers from different releases are still not interchangeable:
/// a method whose shape moved will fail. It fails on that method now rather than on the handshake,
/// and for the few seconds after an update it costs nothing at all — `robotctl` is a symlink into
/// `current`, so it follows the new release immediately, while `updaterd` is mid-restart of itself.
/// A client that is *persistently* older is a copy taken from somewhere other than
/// `/usr/local/bin/robotctl`.
/// # v7 — `ApplyOptions::from_dir`
///
/// First ships in 0.5.0, which is therefore the version a board has to be on before
/// `robotctl update apply --from` can put anything there.
///
/// **It bumps even though `from_dir` is an optional field an older daemon would happily parse.**
/// That is the reason to bump rather than a reason not to: serde ignores what it does not know, so a
/// v6 `updaterd` would accept `update.apply --from /some/dir` and quietly install from its
/// *configured* source instead — a mirror of the release the operator meant to sideload, or nothing
/// at all.
///
/// Refusing the call outright is what the handshake used to be for, and it is now the params
/// decoder's job: an unknown member of `params` is an [`code::INVALID_PARAMS`] naming the member. A
/// v6 daemon predates that and still ignores the field, which is exactly why the bump stays — it is
/// the honest label for two peers that do not share a release.
/// # v8 — `pad.input`
///
/// A namespace addition of the kind v5 was, and it bumps for the same stated reason: the constant
/// says "these two peers were not built together", not "nothing you knew has changed".
///
/// **What it costs here is smaller than usual, and worth being precise about.** The new method is
/// served by `padd` on a socket of its own, so no existing client loses anything by not knowing it,
/// and a `robotctl` that asks an older `padd` for it finds no socket at all rather than a refusal.
/// The bump lands on nothing that refuses: it records that two peers differ, and the rule that every
/// binary on a board comes from one release is kept by the install path rather than by a handshake.
///
/// # v9 — the skill intents
///
/// `robot.do` (ground pick, kicks, sit↔stand, roulade), `robot.pose` (standing body pose),
/// `robot.mouth`, `robot.shutdown` (sit, then power off) and `robot.mode` (walk vs roller),
/// ported from `microduck_runtime`; `robot.enable` gains `toggle`, the pad's Start
/// evaluated robot-side. Additive, and bumps anyway, per the rule above.
///
/// # v10 — `robot.sound`
///
/// The robot's voice: play a voice-bank tag (chirp, greet, coo, ...), with `wheee` as a
/// held ride the client keeps alive per tick. Additive, same rule.
///
/// # v11 — `tof.stream`
///
/// The head ToF sensor's 8×8 depth frames, served by `tofd` on its own socket. A new
/// namespace, like `pad.*` was: nothing existing changes shape, and a client built before
/// this simply never asks.
///
/// # v12 — `robot.look`
///
/// Gaze as a point instead of joint angles: `robot.head`'s doc always promised both forms,
/// and this is the second one — the daemon runs the IK and answers with the joints it chose.
///
/// # v14 — `robot.chorale`, `chorale.*`
///
/// Several ducks singing one piece in four parts, synchronised over BLE advertisements with no
/// shared clock. `robot.chorale` is the switch; the `chorale.*` namespace is how `btd` — which owns
/// the radio — and `robotd` — which owns the behaviour and the voice — divide the work. Additive:
/// a client that never asks is unaffected, and `chorale.*` is between daemons rather than for
/// clients at all.
///
/// # v13 — `robot.theremin`
///
/// The head ToF becomes an instrument: a hand's distance is the pitch, and the mouth opens
/// with it. Additive, same rule as every method before it — plus an optional `theremin`
/// block in `robot.state`, absent while the instrument is down, so a client from v12 sees
/// exactly the frame it saw before.
///
/// # v15 — `robot.setMode`
///
/// Walk and roller stop being a startup constant: the mode can be switched while the robot runs,
/// which is what the gamepad's held D-pad up does. Additive as a method — but `robot.mode`'s
/// answer, and the policy names in the `robot.subscribe` acknowledgement, can now *change* during
/// a session. A client that read either once and cached it forever was already making an
/// assumption this method breaks; nothing about a frame's shape changes.
///
/// # v16 — `update.show`
///
/// The per-run update transcript: what `updaterd` actually did, phase by phase, with the
/// manifest it verified, the hook output it collected and the units it restarted. Additive
/// as a method — a client that never asks is unaffected — and `update.log`'s entries gain a
/// `run` number pointing at one, which an older client ignores as an unknown member because
/// results are not `deny_unknown_fields`. An older `updaterd` answers `update.show` with
/// [`code::METHOD_NOT_FOUND`] naming it, which is the designed skew behaviour rather than a
/// handshake refusal.
///
/// # v23 — `account.*`
///
/// A robot can belong to a Hugging Face account, which is the thing that has to be true before it
/// can be reached from outside its own LAN: the robot's relay proves to a rendezvous service that
/// it belongs to an account, the service shows a client only its own robots, and those two
/// together are the authorisation a bridged session arrives with. Nothing about the *robot's* own
/// gate changes — there still is not one — see `docs/design/remote-access-design.md` §7.
///
/// Additive as methods, and `updaterd` serves them for the reasons `policy.*` is there: it is the
/// daemon with a network stack, and the credential it stores is also what would reach a private
/// Hub repo. An older `updaterd` answers [`code::METHOD_NOT_FOUND`] naming the method, which is
/// the designed skew behaviour rather than a handshake refusal.
///
/// **`account.login` answers with a code rather than with a token**, and that shape is not an
/// implementation detail: the flow is RFC 8628, so somebody has to read a short code and approve
/// it in a browser, and the client that showed them the code may well be gone by the time they
/// do — a phone that opens a browser backgrounds itself, and iOS then tears the GATT link down.
/// So `login` returns immediately, `updaterd` owns the waiting, and `status` is what a client
/// comes back to. A login that reported success by holding the connection open would work on a
/// laptop and fail on the device it is for.
///
/// # v22 — `robot.skills`, `robot.setSkill`, `robot.removeSkill`
///
/// The last thing in the policy path a terminal on the robot could reach and nothing else could.
/// `[[policy.skill]]` is a repeating table, so `robotctl policy add` wrote it directly — there
/// was no method to route, and a remote client could fetch a policy and fill a *slot* with it but
/// never make it answer to `robot.do` by name.
///
/// `robotd` writes the file and reloads itself, so one call is the whole operation. The
/// alternative was a client writing and then remembering `robot.reloadPolicies`, and one that
/// forgot would leave a robot whose config and behaviour disagree until the next restart.
///
/// [`SkillParams`] is the same shape read and written, and every optional field means "the
/// default for this" — so a client may list, change one field and send it back. An entry that
/// already exists supplies whatever the call leaves out.
///
/// [`SkillsResult`] also carries `built_in`: the names `robot.do` answers to that are not table
/// entries, because the daemon drives them itself. A client listing what a robot can do needs
/// both, and `policies.skills` alone omitted two of the five the pad ships bound to — the same
/// trap `do_names` exists to close on the robot side.
///
/// # v21 — `pad.bindings`, `pad.bind`
///
/// Which button runs which skill stops being a thing only somebody at a keyboard on the robot can
/// change. `robotctl pad bind` edited the config file directly, so unlike every other command it
/// had no wire surface at all and routing could not help.
///
/// **Two methods in `pad.*` that `configd` does not serve.** The rest of that namespace is about
/// the radio and is `configd`'s; a binding is about what a button does to the robot, and checking
/// a name needs the skill list, which only `robotd` has. Routing is per method throughout, so
/// this costs nothing — `policy.*` and `robot.loadPolicy` already split one concept across two
/// daemons the same way. An older `updaterd` or `configd` reached by mistake answers
/// [`code::METHOD_NOT_FOUND`] naming it.
///
/// Additive as methods. `pad.bind` is the first call on either radio transport that **writes the
/// config file**, and it has to be: `padd` re-reads `[pad]` every second, so a binding held in
/// memory would be reverted before the caller let go of the phone. That also means it needs no
/// reload and no restart.
///
/// # v20 — `robot.policies` carries the skills
///
/// One field, and it is what makes any of this usable from something that is not `robotctl`:
/// which one-shot skills a robot has is config now, so a client cannot assume a list and cannot
/// offer a "bow" button without being told there is a bow. The names were already published in
/// `robot.subscribe`'s acknowledgement, but that is a 50 Hz stream and the question is asked
/// once — the wrong shape for a phone, and BLE deliberately does not route it.
///
/// Additive: a client that ignores the field is unaffected, and an older `robotd` omits it,
/// which deserialises to empty rather than failing because results are not
/// `deny_unknown_fields`. A client reading an empty list should treat it as "this robot did not
/// say" rather than "this robot has no skills" — the same distinction `unavailable` draws for
/// the gait.
///
/// # v19 — `policy.*` and `robot.reloadPolicies`
///
/// The official policy set stops needing a daemon release to change. `policy.check` asks the Hub
/// what revisions exist against the one installed, `policy.install` fetches one, and
/// `robot.reloadPolicies` is how the thing that swapped the set tells `robotd` to re-read it —
/// necessary because installing a set swaps a symlink *underneath* unchanged paths, so
/// `robot.loadPolicy` would correctly conclude there was nothing to do.
///
/// Additive as methods. `policy.*` is the first namespace `updaterd` serves that is not
/// `update.*`, and it is there rather than in `robotd` because it needs a network stack, and not
/// in `robotctl` because that must not link one. See `docs/design/policy-channel-design.md` §8.
///
/// # v18 — `robot.policies`, `robot.loadPolicy`
///
/// Which `.onnx` fills a slot stops being a restart-only decision: a policy can be swapped into
/// one slot while the robot runs, and dropped again, without editing a file by hand. Additive as
/// methods, and the state they expose was previously unavailable at all rather than available in
/// another shape, so nothing an older client reads changes meaning — the policy names in the
/// `robot.subscribe` acknowledgement could already change mid-session as of v15.
///
/// **`robot.loadPolicy` persists.** A slot is `[policy] <slot>` in `robotd.toml` and nothing
/// else, so the daemon writes the key before it queues the swap and a gait chosen over the wire
/// is the gait the robot boots into. Worth stating on the wire rather than only in the daemon,
/// because it is the difference between a call a client offers as "try this" and one it offers
/// as "use this" — and the undo is the same method with no path, not a restart.
///
/// An older `robotd` answers either with [`code::METHOD_NOT_FOUND`] naming the method, which is
/// the designed skew behaviour and not a handshake refusal. See
/// `docs/design/policy-channel-design.md` §8.
///
/// # v17 — a unit state that can say "crash loop"
///
/// [`UnitState`] gains `Restarting` and `Failed`, which `system.services` can now answer with.
/// Not additive in the way a new method is: an older client deserialising a `Vec<ServiceUnit>`
/// rejects a member it has no variant for, and `robotctl` reads that reply with `.ok()` — so an
/// older `robotctl` against this `configd` prints no `units` block at all rather than a wrong one.
/// Both come out of the same release and an apply restarts both, so the skew lasts as long as the
/// update does; a board left mid-update sees a missing block, not a lie.
pub const API_VERSION: u32 = 23;

/// The observation width every policy this robot family runs is built against.
///
/// Here rather than only in `duck_control` because it is a **contract with whoever publishes a
/// policy**, not an implementation detail: it is what a repo's manifest declares, what
/// `updaterd` refuses a mismatched policy on before downloading it, and what `robotd` refuses it
/// on at load. `duck_control::obs` is still where the observation is built, and a test there
/// asserts the two agree.
pub const POLICY_OBS_LEN: usize = 61;

/// The action count, on the same footing as [`POLICY_OBS_LEN`]: fifteen servos, fourteen of them
/// driven by a policy.
pub const POLICY_ACTION_LEN: usize = 14;

/// The robot this daemon drives, as a policy manifest spells it. Refusing a policy published for
/// something else is cheaper than discovering it in the way the robot moves.
pub const ROBOT_MODEL: &str = "microduck";

/// The longest an update may legitimately go quiet, in seconds — the pre-install hook's ceiling.
///
/// **Here rather than in `updater`, because it is a contract with every client.** The phase
/// notification for a hook arrives *before* the hook runs, so a client watching an apply sees
/// nothing at all while the hook works — and that hook installs what a release needs and the board
/// may not have: ONNX Runtime, and around 100 MB of apt for `mediad`'s GStreamer stack on a board
/// that has never had it. A client whose idle budget is shorter than this reports a working update
/// as a robot that stopped answering, which is exactly what `duckctl` did the first time this
/// ceiling moved.
///
/// `updaterd` enforces it and every client sizes its own budget above it. Both read this constant,
/// so the two cannot disagree.
pub const UPDATE_MAX_SILENCE_SECONDS: u64 = 600;

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
    /// `padd`'s raw input tap — [`super::method::PAD_INPUT`], and nothing else.
    ///
    /// Under `/run/padd/` because that is `padd`'s `RuntimeDirectory=`, which is the only place a
    /// process running under `ProtectSystem=strict` may create a file. systemd removes the
    /// directory when the unit stops, so a socket left behind cannot outlive the daemon that
    /// would have answered on it.
    pub const PAD: &str = "/run/padd/pad.sock";

    /// `tofd`'s depth stream — [`super::method::TOF_STREAM`], and nothing else.
    /// Under `/run/tofd/` for the same reason as the pad's: it is that unit's
    /// `RuntimeDirectory=`, so systemd removes the socket when the daemon stops.
    pub const TOF: &str = "/run/tofd/tof.sock";
}

/// Where each daemon publishes what it is running: `/run/<service>/identity.json`.
///
/// One directory per service rather than one shared directory, because that is what
/// `RuntimeDirectory=<service>` in a unit file gives — and it has to be that, for two reasons no
/// tidier layout survives. `btd` and `padd` run under `ProtectSystem=strict`, so the filesystem is
/// read-only to them *except* what systemd grants, and `RuntimeDirectory` is the grant. And systemd
/// deletes the directory when the unit stops, so a stopped daemon cannot leave a stale identity
/// behind claiming to be running.
pub fn identity_path(service: &str) -> std::path::PathBuf {
    runtime_root().join(service).join("identity.json")
}

/// Where the per-service runtime directories live: `/run`, unless `DUCK_RUNTIME_DIR` says otherwise.
///
/// Not a configuration knob for a robot — nothing on the board sets it. It exists so this is testable
/// at all, since a test cannot write to `/run`, and it is the same reason a daemon run by hand on a
/// laptop can publish an identity: `/run` is root-owned there too, and on macOS it does not exist.
pub fn runtime_root() -> std::path::PathBuf {
    std::env::var_os("DUCK_RUNTIME_DIR")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| std::path::PathBuf::from("/run"))
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
    pub const SHOW: &str = "update.show";
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
    /// Point the camera at a trunk-frame point; the daemon runs the gaze IK.
    /// Discrete; send as a request — the answer says what the head will do.
    pub const ROBOT_LOOK: &str = "robot.look";
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

    // ── skills ───────────────────────────────────────────────────────────────
    //
    // One-shot scripted moves, ported from `microduck_runtime`. Each swaps a dedicated
    // policy in for a fixed window (or, for sit, until toggled back); the observation
    // layout is the same 61-D vector throughout, so a skill is a session choice plus a
    // command-block encoding, not a new contract.

    /// Run a one-shot skill, or toggle sit↔stand. Answered — a refusal names the
    /// scripted move already holding the robot.
    pub const ROBOT_DO: &str = "robot.do";
    /// Standing body pose: z / roll / pitch offsets. Continuous; send as a notification.
    ///
    /// `active: false` snaps the pose back to nominal — that is the prototype's B-button
    /// exit, which zeroes instantly rather than gliding.
    pub const ROBOT_POSE: &str = "robot.pose";
    /// Mouth opening, 0 (closed) to 1 (open). Continuous; send as a notification. The mouth
    /// is not part of any policy — this is the only thing that moves it.
    pub const ROBOT_MOUTH: &str = "robot.mouth";
    /// Play one of the robot's voice-bank sounds — chirp, greet, coo... `wheee` is the
    /// held ride: `hold: true` starts it and must keep arriving (a notification per tick,
    /// like the mouth) or the ride ends; `hold: false` releases it deliberately. The two
    /// endings differ: a deliberate release cuts the ride, a hold that stops arriving plays
    /// it out through its end segment.
    ///
    /// Refused, with a reason, by a robot that has no voice — audio disabled, or no bank
    /// rendered. Sounds are never refused for circumstance (a chirp out of a fallen robot
    /// is diagnostics, not danger), but "accepted" from a robot that cannot make a sound
    /// would make `robotctl quack` lie about which duck answered.
    pub const ROBOT_SOUND: &str = "robot.sound";
    /// Pick the ToF theremin up, or put it down: the head's depth sensor becomes an
    /// instrument, and the distance of a hand in front of the beak is the pitch — and the
    /// mouth opening, which rises with it, so the note is visible as well as audible.
    ///
    /// Discrete; send as a request, and idempotent both ways. The answer says whether the
    /// robot took the instrument — it has a voice, and depth frames are arriving. From then
    /// on the nearest return inside the playable band is the hand: an explicit mode with
    /// nothing clever inside it, because the clever version could not be relied on to mean
    /// the same thing twice on real frames.
    ///
    /// [`ROBOT_STATE`]'s `theremin` block carries the live pitch, the mouth, and a line of
    /// what the sensor said about the frame — which is the field diagnostic for this whole
    /// feature.
    pub const ROBOT_THEREMIN: &str = "robot.theremin";
    /// Start or stop looking for other ducks to sing with. Discrete; the answer is
    /// [`super::ChoraleResult`].
    ///
    /// What it starts is *listening*, not singing: the robot begins broadcasting a beacon saying
    /// it is willing, and watching for others. Two willing ducks in radio range then start a piece
    /// between themselves with nobody in charge — the lower id conducts — and a third joins what it
    /// finds already under way.
    ///
    /// Refused outright by a robot whose config has not opted in (`[chorale] accept`), which is
    /// false by default: a chorale moves the mouth and the head, and a robot that began animating
    /// because another robot walked into the room would be doing unrequested motion.
    pub const ROBOT_CHORALE: &str = "robot.chorale";
    /// Sit down gracefully, then power the machine off. The prototype's Select long-press.
    pub const ROBOT_SHUTDOWN: &str = "robot.shutdown";
    /// Which drive mode this `robotd` is in: `walk` or `roller`.
    ///
    /// `[policy] mode` is where it starts; [`ROBOT_SET_MODE`] moves it while the robot runs, so
    /// this is a question with a changing answer rather than a startup constant. Exists so `padd`
    /// can shape its stick mapping to the mode without owning config it has no business reading.
    pub const ROBOT_MODE: &str = "robot.mode";

    /// Switch drive mode: `walk` or `roller`.
    ///
    /// Held on the gamepad's D-pad up, as the prototype held it — putting wheels on a duck is a
    /// thing you do in the room with it, not from a laptop. The robot returns to its home pose,
    /// loads the other mode's policies, and drives again; the config is untouched, so a reboot
    /// comes back in the configured mode.
    pub const ROBOT_SET_MODE: &str = "robot.setMode";

    /// What each policy slot is running, and where that file came from.
    ///
    /// Distinct from [`ROBOT_MODE`], which says which *set* of slots is in play. This says what
    /// is in them — and, when a load failed, what is in them instead of what was asked for.
    pub const ROBOT_POLICIES: &str = "robot.policies";

    /// Put a different `.onnx` in one slot, or drop an override and go back to the default.
    ///
    /// Answered like [`ROBOT_SET_MODE`] and for the same reason: the swap happens at the home
    /// pose, seconds later, so the reply says the request was accepted and
    /// [`ROBOT_POLICIES`] says what came of it.
    pub const ROBOT_LOAD_POLICY: &str = "robot.loadPolicy";

    /// Re-read every policy slot from disk, whatever the paths say.
    ///
    /// Distinct from [`ROBOT_LOAD_POLICY`] because the paths do not change: installing a newer
    /// official set swaps a symlink underneath them, so every slot still resolves to the same
    /// string and a load would correctly conclude there is nothing to do. This is how the thing
    /// that swapped the symlink says otherwise.
    pub const ROBOT_RELOAD_POLICIES: &str = "robot.reloadPolicies";

    // ── policy.* ─────────────────────────────────────────────────────────────
    //
    // Served by `updaterd`, not `robotd`, and the split is the same one everywhere else here:
    // `robotd` says what it is *running*, `updaterd` is the thing with a network stack and is
    // therefore what asks the Hub what exists. `robotctl` must not link an HTTP client — it is on
    // the recovery path and its dependency list is deliberately short — so the fetch cannot live
    // in the CLI either. `docs/design/policy-channel-design.md` §8.

    /// Is there a newer policy set than the one installed?
    pub const POLICY_CHECK: &str = "policy.check";
    /// Install a policy set from the Hub and make it live.
    pub const POLICY_INSTALL: &str = "policy.install";
    /// Fetch one policy from any Hub repo into this robot's library.
    pub const POLICY_FETCH: &str = "policy.fetch";
    /// Search the Hub for policies.
    pub const POLICY_SEARCH: &str = "policy.search";

    // ── account.* ────────────────────────────────────────────────────────────
    //
    // Which Hugging Face account this robot belongs to. Served by `updaterd` for `policy.*`'s
    // reasons exactly — it is the daemon with a network stack, and `robotctl` must not link one —
    // and because the credential it stores is also what reaches a *private* Hub repo, which is
    // already this daemon's job.
    //
    // The account is what makes a robot reachable from outside the LAN: the relay proves the robot
    // belongs to an account, the rendezvous service shows a client only its own robots, and the
    // pair of those is the authorisation a bridged session arrives with.
    // `docs/design/remote-access-design.md` §2.

    /// Begin an OAuth device-code login. Answers immediately with a code to show somebody.
    pub const ACCOUNT_LOGIN: &str = "account.login";
    /// Which account this robot belongs to, and whether a login is in flight.
    pub const ACCOUNT_STATUS: &str = "account.status";
    /// Forget the account. The robot stops being reachable from outside the LAN.
    pub const ACCOUNT_LOGOUT: &str = "account.logout";

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
    /// What systemd says about each daemon, and which release each is running from.
    pub const SYSTEM_SERVICES: &str = "system.services";
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
    pub const PAD_BINDINGS: &str = "pad.bindings";
    pub const PAD_BIND: &str = "pad.bind";
    pub const ROBOT_SKILLS: &str = "robot.skills";
    pub const ROBOT_SET_SKILL: &str = "robot.setSkill";
    pub const ROBOT_REMOVE_SKILL: &str = "robot.removeSkill";

    /// Subscribe to the raw input stream of the pad `padd` is driving from.
    ///
    /// **The one method in this namespace `padd` answers itself**, on [`super::socket::PAD`]
    /// rather than `configd`'s socket. That is not a lapse in the split above: the three calls
    /// before it are Bluetooth *configuration*, and this is the input device — which only the
    /// process already reading it can name, since it is the node gilrs picked.
    ///
    /// It exists for one question the rest of the system cannot answer. `padd` polls the last
    /// known stick value and keeps sending it at 50 Hz, so a radio that stops delivering reports
    /// still produces fresh intents: `robotd` sees a live driver, the deadman never fires, and the
    /// robot walks on a stale command. Nothing in `robot.state` can show that. The raw event
    /// stream can, because a report that never arrived leaves a measurable hole in the cadence.
    pub const PAD_INPUT: &str = "pad.input";

    /// One report from the pad, pushed after [`PAD_INPUT`].
    pub const PAD_REPORT: &str = "pad.report";

    // ── tof.* ────────────────────────────────────────────────────────────────
    /// Subscribe to the head ToF sensor's depth frames, on [`super::socket::TOF`].
    ///
    /// **The one method in this namespace, and `tofd` answers it itself.** Like
    /// the pad tap, this is a sensor stream on the socket of the daemon that owns
    /// the sensor — `robotd` is not in the path, because nothing in the control
    /// loop reads depth and putting perception in front of it would be the
    /// coupling `architecture.md` §1 splits `mediad` off to avoid.
    ///
    /// The answer describes the sensor (or says why there is none); frames then
    /// arrive as [`TOF_FRAME`] notifications until the connection closes.
    pub const TOF_STREAM: &str = "tof.stream";

    /// `btd` → `robotd`: subscribe to what to put on the air. Answered, then `robotd` streams
    /// [`CHORALE_BEACON`] notifications for as long as the connection lasts.
    ///
    /// The radio is `btd`'s and the behaviour is `robotd`'s, so neither can do this alone. This is
    /// the direction that needs a subscription rather than a call: `robotd` decides *when* the
    /// beacon changes, and it changes on the beat.
    pub const CHORALE_SUBSCRIBE: &str = "chorale.subscribe";
    /// `robotd` → `btd`: advertise this, or stop. A notification on a [`CHORALE_SUBSCRIBE`]
    /// stream; the params are [`super::ChoraleAdvertise`].
    pub const CHORALE_BEACON: &str = "chorale.beacon";
    /// `btd` → `robotd`: another duck's beacon was heard. A notification — the params are
    /// [`super::ChoraleHeard`].
    pub const CHORALE_HEARD: &str = "chorale.heard";

    /// One 8×8 depth frame, pushed after [`TOF_STREAM`].
    pub const TOF_FRAME: &str = "tof.frame";
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
    /// Retired: nothing emits this any more. A version *difference* is logged and served, and
    /// what refuses is the route that is genuinely missing — see [`super::API_VERSION`].
    ///
    /// The constant stays, and the number is not reused, because a board running an `updaterd`
    /// from 0.5.1 or earlier still answers `hello` with it and `robotctl` still maps it to an
    /// exit status. Deleting it would make that board's refusal read as a generic failure.
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
    /// One run's full transcript. `update.log` says which runs exist; this says what one did.
    Show(ShowParams),
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
    /// Discrete. Send as a request; the answer is [`LookResult`].
    RobotLook(LookParams),
    RobotStop,
    RobotEnable(EnableParams),
    /// Power the joints and ramp to the home pose. No policy needed.
    RobotInit,
    /// Cut power to the joints. The robot collapses if nothing holds it.
    RobotRelax,
    /// Run a one-shot skill, or toggle sit↔stand.
    RobotDo(DoParams),
    /// Standing body pose. Continuous. Send as a notification.
    RobotPose(PoseParams),
    /// Mouth opening. Continuous. Send as a notification.
    RobotMouth(MouthParams),
    /// Play a voice-bank sound.
    RobotSound(SoundParams),
    /// Pick the ToF theremin up or put it down. Discrete; the answer is [`ThereminResult`].
    RobotTheremin(ThereminParams),
    /// Start or stop looking for other ducks to sing with. Discrete; the answer is
    /// [`ChoraleResult`].
    RobotChorale(ChoraleParams),
    /// `btd` subscribing to what it should advertise. Answered, then a stream of
    /// [`method::CHORALE_BEACON`] notifications.
    ChoraleSubscribe,
    /// `robotd` telling `btd` what to advertise. A notification.
    ChoraleBeaconSet(ChoraleAdvertise),
    /// `btd` telling `robotd` what it heard. A notification.
    ChoraleHeard(ChoraleHeard),
    /// Sit down, then power the machine off.
    RobotShutdown,
    /// Which drive mode this robotd runs: walk or roller.
    RobotMode,
    /// Switch drive mode; see [`method::ROBOT_SET_MODE`].
    RobotSetMode(SetModeParams),
    /// What each policy slot runs; see [`method::ROBOT_POLICIES`].
    RobotPolicies,
    /// Load one slot, or reset it; see [`method::ROBOT_LOAD_POLICY`].
    RobotLoadPolicy(LoadPolicyParams),
    /// Re-read every slot from disk; see [`method::ROBOT_RELOAD_POLICIES`].
    RobotReloadPolicies,

    // ── policy.* ─────────────────────────────────────────────────────────────
    /// What is installed and what the Hub offers; see [`method::POLICY_CHECK`].
    PolicyCheck,
    /// Install a set and make it live; see [`method::POLICY_INSTALL`].
    PolicyInstall(PolicyInstallParams),
    /// Fetch one policy into the library; see [`method::POLICY_FETCH`].
    PolicyFetch(PolicyFetchParams),
    /// Search the Hub; see [`method::POLICY_SEARCH`].
    PolicySearch(PolicySearchParams),

    // ── account.* ────────────────────────────────────────────────────────────
    /// Start a device-code login; see [`method::ACCOUNT_LOGIN`].
    AccountLogin(AccountLoginParams),
    /// Who this robot belongs to; see [`method::ACCOUNT_STATUS`].
    AccountStatus,
    /// Forget the account; see [`method::ACCOUNT_LOGOUT`].
    AccountLogout,
    RobotSubscribe(SubscribeParams),
    // ── net.* ────────────────────────────────────────────────────────────────
    NetStatus,
    NetScan,
    NetConnect(NetConnectParams),
    NetForget(NetForgetParams),

    // ── system.* ─────────────────────────────────────────────────────────────
    SystemInfo,
    SystemServices,
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
    PadBindings,
    PadBind(PadBindParams),
    RobotSkills,
    RobotSetSkill(SkillParams),
    RobotRemoveSkill(SkillNameParams),
    /// Subscribe to the raw pad input stream. Answered by `padd`, not `configd`.
    PadInput,
    /// Subscribe to the ToF depth stream. Answered by `tofd`.
    TofStream,
}

/// The service that owns the answer to a call.
///
/// One socket per service, connected directly — there is no broker (`architecture.md` §2.2). A
/// transport adapter holds connections to the services whose calls it carries, and to no others:
/// `btd` holds three, and `padd` being absent from them is deliberate rather than incidental —
/// `padd` is the unprivileged client whose whole value is having no special access.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Service {
    /// `updaterd`, at [`DEFAULT_SOCKET`].
    Updater,
    /// `robotd` — the control loop.
    Robot,
    /// `configd` — wifi, identity, the pairing PIN, gamepad bonding.
    Config,
    /// `padd` — the raw gamepad input stream.
    Pad,
    /// `tofd` — the depth stream.
    Tof,
}

/// How long answering a call holds a connection, and therefore which connection carries it.
///
/// **Every service here serves one connection one request at a time.** So a single connection per
/// service would put every call on one queue behind the slowest thing on it, and two orderings a
/// client reaches for first are broken by that:
///
/// - `update.apply` then `update.status` — the status line waits in a socket `updaterd` will not
///   read for minutes, so the client times out having heard nothing while the robot is fine.
/// - `update.subscribe` then `update.apply` — worse. The subscription owns its connection until
///   the peer goes away and never reads another request, so the apply is written into a socket
///   nobody reads: it never runs, never replies and never errors. An update the owner asked for
///   that the robot silently did not perform.
///
/// Grouping by *how long a call holds a connection* and giving each group its own is what fixes
/// both. It is at most four sockets per service per session, which costs nothing and is bounded
/// without bookkeeping — the alternative, a connection per call, needs the adapter to know when a
/// call ended, which needs it to parse replies. It deliberately never does.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Lane {
    /// Answers as fast as the daemon can look something up.
    Prompt,
    /// Seconds: a read that goes to the network or sweeps a radio.
    Slow,
    /// As long as it takes, and it changes the robot: an update, joining a network, bonding a pad.
    Operation,
    /// Never answers. The service writes notifications until the peer goes away.
    Stream,
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
            Call::Show(_) => method::SHOW,
            Call::Subscribe => method::SUBSCRIBE,
            Call::RobotSafeToRestart => method::ROBOT_SAFE_TO_RESTART,
            Call::RobotHealth => method::ROBOT_HEALTH,
            Call::RobotModelApi => method::ROBOT_MODEL_API,
            Call::RobotRemoteSessionActive => method::ROBOT_SESSION_ACTIVE,
            Call::RobotMove(_) => method::ROBOT_MOVE,
            Call::RobotHead(_) => method::ROBOT_HEAD,
            Call::RobotLook(_) => method::ROBOT_LOOK,
            Call::RobotStop => method::ROBOT_STOP,
            Call::RobotEnable(_) => method::ROBOT_ENABLE,
            Call::RobotInit => method::ROBOT_INIT,
            Call::RobotRelax => method::ROBOT_RELAX,
            Call::RobotDo(_) => method::ROBOT_DO,
            Call::RobotPose(_) => method::ROBOT_POSE,
            Call::RobotMouth(_) => method::ROBOT_MOUTH,
            Call::RobotSound(_) => method::ROBOT_SOUND,
            Call::RobotTheremin(_) => method::ROBOT_THEREMIN,
            Call::RobotChorale(_) => method::ROBOT_CHORALE,
            Call::ChoraleSubscribe => method::CHORALE_SUBSCRIBE,
            Call::ChoraleBeaconSet(_) => method::CHORALE_BEACON,
            Call::ChoraleHeard(_) => method::CHORALE_HEARD,
            Call::RobotShutdown => method::ROBOT_SHUTDOWN,
            Call::RobotMode => method::ROBOT_MODE,
            Call::RobotSetMode(_) => method::ROBOT_SET_MODE,
            Call::RobotPolicies => method::ROBOT_POLICIES,
            Call::RobotLoadPolicy(_) => method::ROBOT_LOAD_POLICY,
            Call::RobotReloadPolicies => method::ROBOT_RELOAD_POLICIES,
            Call::PolicyCheck => method::POLICY_CHECK,
            Call::PolicyInstall(_) => method::POLICY_INSTALL,
            Call::PolicyFetch(_) => method::POLICY_FETCH,
            Call::PolicySearch(_) => method::POLICY_SEARCH,
            Call::AccountLogin(_) => method::ACCOUNT_LOGIN,
            Call::AccountStatus => method::ACCOUNT_STATUS,
            Call::AccountLogout => method::ACCOUNT_LOGOUT,
            Call::RobotSubscribe(_) => method::ROBOT_SUBSCRIBE,
            Call::NetStatus => method::NET_STATUS,
            Call::NetScan => method::NET_SCAN,
            Call::NetConnect(_) => method::NET_CONNECT,
            Call::NetForget(_) => method::NET_FORGET,
            Call::SystemInfo => method::SYSTEM_INFO,
            Call::SystemServices => method::SYSTEM_SERVICES,
            Call::SystemSetName(_) => method::SYSTEM_SET_NAME,
            Call::SystemReboot => method::SYSTEM_REBOOT,
            Call::SystemPairingPin => method::SYSTEM_PAIRING_PIN,
            Call::SystemSetPairingPin(_) => method::SYSTEM_SET_PAIRING_PIN,
            Call::SystemAuthenticate(_) => method::SYSTEM_AUTHENTICATE,
            Call::PadStatus => method::PAD_STATUS,
            Call::PadPair(_) => method::PAD_PAIR,
            Call::PadForget(_) => method::PAD_FORGET,
            Call::PadBindings => method::PAD_BINDINGS,
            Call::PadBind(_) => method::PAD_BIND,
            Call::RobotSkills => method::ROBOT_SKILLS,
            Call::RobotSetSkill(_) => method::ROBOT_SET_SKILL,
            Call::RobotRemoveSkill(_) => method::ROBOT_REMOVE_SKILL,
            Call::PadInput => method::PAD_INPUT,
            Call::TofStream => method::TOF_STREAM,
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
                // Powering the machine off is at least as disruptive as rebooting it.
                | Call::RobotShutdown
                // Bonding a pad to this robot changes what may drive it, which is the most
                // consequential thing in this namespace — a paired pad can enable the policy.
                // `pad.status` is a read and stays ungated.
                | Call::PadPair(_)
                | Call::PadForget(_)
                // Replacing the policy set changes what drives fifteen servos, which is at least
                // as consequential as bonding a pad. `policy.check` is a read and stays ungated,
                // like `pad.status` and every other question about what a robot is.
                | Call::PolicyInstall(_)
                // Putting a stranger's network on the board is at least as consequential as
                // replacing the official set. `policy.search` and `policy.fetch`'s read-only
                // cousins stay ungated — asking what exists changes nothing.
                | Call::PolicyFetch(_)
                // Signing the robot in binds it to a Hugging Face account, and signing it out
                // takes it away again. That is the most consequential pair here by one measure
                // nothing else in this list shares: it decides who can reach the robot *from
                // outside the building*, and it outlives every session and every reboot.
                // `account.status` is a read and stays ungated — a support engineer should be
                // able to ask which account a robot thinks it belongs to.
                | Call::AccountLogin(_)
                | Call::AccountLogout
        )
    }

    /// The service that owns the answer to this call, and how long answering it holds a
    /// connection. `None` for a call no service answers.
    ///
    /// **This is a property of the call, not of a transport.** Who owns `net.connect` does not
    /// change depending on whether a phone asked over Bluetooth or a browser asked over a
    /// datachannel, and neither does the fact that `configd` will sit on the connection for the
    /// better part of a minute while it waits for NetworkManager. Whether a given transport may
    /// *make* the call is the separate question, and it stays with the transport — see
    /// `btd::route` for the one that exists, and `docs/design/remote-webrtc.md` §5 for why the two
    /// were split.
    ///
    /// It lives here, beside [`Call::mutates`], because it is the same kind of fact: something
    /// every adapter needs and none should decide for itself.
    pub fn destination(&self) -> Option<(Service, Lane)> {
        use Lane::*;
        use Service::*;
        Some(match self {
            // The version handshake. `updaterd` answers it because it is the service on the
            // recovery path — the one that must reply when the rest of the robot does not.
            Call::Hello(_) => (Updater, Prompt),

            // ── updaterd ────────────────────────────────────────────────────
            //
            // `Apply`, `Rollback` and `ResetToGolden` all move `current`, and `updaterd`
            // single-flights mutations behind a file lock and answers `BUSY` for a second one —
            // so sharing one lane is the behaviour to have rather than a compromise.
            Call::Apply(_) | Call::Rollback(_) | Call::Select(_) | Call::ResetToGolden(_) => {
                (Updater, Operation)
            }
            // Reaches the network to ask a mirror what exists, so seconds. Deliberately off
            // `Operation`: "is there an update?" asked during one has an immediate answer
            // (`BUSY`), and queueing it behind the update would turn that into a spinner that
            // resolves minutes later.
            Call::Check(_) => (Updater, Slow),
            // `update.status` falls back to a cached snapshot rather than waiting for the engine,
            // so it answers during an apply — which is wasted if the request is queued behind one.
            Call::Status => (Updater, Prompt),
            Call::Pin(_) | Call::Log(_) | Call::ListInstalled(_) => (Updater, Prompt),
            // Reads a file, like `log`. Bounded by the per-run caps `updater::transcript`
            // enforces at write time, so the answer cannot grow without limit however long
            // a hook talked for.
            Call::Show(_) => (Updater, Prompt),
            // `check` is one HTTP round trip. `install` downloads a policy set and then asks
            // `robotd` to reload, which is the same order of magnitude as a small update — long,
            // but bounded and not a stream.
            Call::PolicyCheck | Call::PolicyInstall(_) => (Updater, Prompt),
            // `fetch` downloads one file and `search` is a single query; both are bounded and
            // neither streams.
            Call::PolicyFetch(_) | Call::PolicySearch(_) => (Updater, Prompt),
            // `login` is one HTTP round trip to Hugging Face and answers with the code to
            // show somebody; the *waiting* happens in a task `updaterd` owns, not on this
            // connection, so it is `Slow` for the round trip rather than `Stream` for the login.
            // That is the whole reason `login` returns a code instead of a token: a client that
            // backgrounds itself while its user reads the code — which is what a phone does when
            // it opens a browser — must be able to come back and ask `status`.
            Call::AccountLogin(_) => (Updater, Slow),
            // Both read or write local state only. `status` is asked on a poll while a login is
            // in flight, so it must not queue behind anything.
            Call::AccountStatus | Call::AccountLogout => (Updater, Prompt),
            // Owns its connection until the peer goes away and never reads another request.
            Call::Subscribe => (Updater, Stream),

            // ── robotd ──────────────────────────────────────────────────────
            Call::RobotSafeToRestart
            | Call::RobotHealth
            | Call::RobotModelApi
            | Call::RobotRemoteSessionActive
            | Call::RobotPolicies
            | Call::RobotMode => (Robot, Prompt),
            // Intents and one-shot skills. All fast: they store a value the control loop reads on
            // its next tick, and none of them waits for the robot to finish anything.
            Call::RobotMove(_)
            | Call::RobotHead(_)
            | Call::RobotLook(_)
            | Call::RobotStop
            | Call::RobotEnable(_)
            | Call::RobotInit
            | Call::RobotRelax
            | Call::RobotDo(_)
            | Call::RobotPose(_)
            | Call::RobotMouth(_)
            | Call::RobotSound(_)
            | Call::RobotTheremin(_)
            | Call::RobotChorale(_)
            | Call::RobotSetMode(_)
            | Call::RobotLoadPolicy(_)
            | Call::RobotReloadPolicies
            | Call::RobotShutdown => (Robot, Prompt),
            Call::RobotSubscribe(_) => (Robot, Stream),
            // `btd` asking what to put on the air. The answering connection carries the beacon
            // stream down and `chorale.heard` notifications up.
            Call::ChoraleSubscribe => (Robot, Stream),

            // ── configd ─────────────────────────────────────────────────────
            Call::NetStatus
            | Call::NetForget(_)
            | Call::SystemInfo
            | Call::SystemServices
            | Call::SystemSetName(_)
            | Call::SystemReboot
            | Call::SystemPairingPin
            | Call::SystemSetPairingPin(_)
            | Call::PadStatus
            | Call::PadForget(_) => (Config, Prompt),

            // **The one place `pad.*` splits across two daemons.** Pairing is about the radio,
            // which `configd` owns. A binding is about what a button does to the robot, and
            // answering it needs two things only `robotd` has: the list of skills this robot
            // actually has, so a name can be refused rather than becoming a dead button, and the
            // config path it is already parsing. Its true sibling is `robot.loadPolicy`, not
            // `pad.pair`.
            //
            // Routing is per method throughout this table — `policy.*` goes to `updaterd` while
            // `robot.loadPolicy` goes to `robotd`, for the same concept — so this costs nothing
            // mechanically. It is only worth a comment because the name suggests otherwise.
            Call::PadBindings | Call::PadBind(_) => (Robot, Prompt),

            // The skill table. `robotd` writes it and reloads itself afterwards, so one call is
            // enough — and it is the daemon that has to accept the result either way.
            Call::RobotSkills | Call::RobotSetSkill(_) | Call::RobotRemoveSkill(_) => (Robot, Slow),
            // Re-sweeps the radio rather than returning the last scan.
            Call::NetScan => (Config, Slow),
            // `configd` polls NetworkManager for up to 45 seconds before calling a join failed,
            // and `pad.pair` waits on a gamepad for its whole timeout. Both hold the connection
            // for that long, which is what `Operation` is for — and why `net.status` must not be
            // queued behind them.
            Call::NetConnect(_) | Call::PadPair(_) => (Config, Operation),

            // ── padd and tofd ───────────────────────────────────────────────
            //
            // Both are streams, and both are named here even though the only transport that
            // exists today reaches neither: what a call *is* does not depend on who may ask it.
            Call::PadInput => (Pad, Stream),
            Call::TofStream => (Tof, Stream),

            // ── answered by no service ──────────────────────────────────────
            //
            // The PIN check belongs to the transport, which is the whole point of it: BLE cannot
            // express a fixed passkey printed on a robot, so the check moved up a layer to where
            // we define the rules (`docs/design/app-path-design.md` §5). A transport that does not
            // gate anything simply never sees this call.
            Call::SystemAuthenticate(_) => return None,
            // These two never dial a service: they travel inside the connection that
            // `chorale.subscribe` opened — the beacon stream down, what the radio heard up.
            Call::ChoraleBeaconSet(_) | Call::ChoraleHeard(_) => return None,
        })
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
            Call::Show(p) => encode(p),
            Call::RobotMove(p) => encode(p),
            Call::RobotHead(p) => encode(p),
            Call::RobotLook(p) => encode(p),
            Call::RobotEnable(p) => encode(p),
            Call::RobotDo(p) => encode(p),
            Call::RobotPose(p) => encode(p),
            Call::RobotMouth(p) => encode(p),
            Call::RobotSetMode(p) => encode(p),
            Call::RobotLoadPolicy(p) => encode(p),
            Call::PolicyInstall(p) => encode(p),
            Call::PolicyFetch(p) => encode(p),
            Call::PolicySearch(p) => encode(p),
            Call::AccountLogin(p) => encode(p),
            Call::RobotSound(p) => encode(p),
            Call::RobotTheremin(p) => encode(p),
            Call::RobotChorale(p) => encode(p),
            Call::ChoraleBeaconSet(p) => encode(p),
            Call::ChoraleHeard(p) => encode(p),
            Call::RobotSubscribe(p) => encode(p),
            Call::NetConnect(p) => encode(p),
            Call::NetForget(p) => encode(p),
            Call::SystemSetName(p) => encode(p),
            Call::SystemSetPairingPin(p) => encode(p),
            Call::SystemAuthenticate(p) => encode(p),
            Call::PadPair(p) => encode(p),
            Call::PadForget(p) => encode(p),
            Call::PadBind(p) => encode(p),
            Call::RobotSetSkill(p) => encode(p),
            Call::RobotRemoveSkill(p) => encode(p),
            Call::Status
            | Call::Subscribe
            | Call::RobotSafeToRestart
            | Call::RobotHealth
            | Call::RobotModelApi
            | Call::RobotRemoteSessionActive
            | Call::RobotStop
            | Call::RobotInit
            | Call::RobotRelax
            | Call::RobotShutdown
            | Call::RobotPolicies
            | Call::RobotReloadPolicies
            | Call::PolicyCheck
            | Call::AccountStatus
            | Call::AccountLogout
            | Call::RobotMode => Value::Object(serde_json::Map::new()),
            Call::NetStatus
            | Call::NetScan
            | Call::SystemInfo
            | Call::SystemServices
            | Call::SystemReboot
            | Call::SystemPairingPin
            | Call::PadStatus
            | Call::PadBindings
            | Call::RobotSkills
            | Call::PadInput
            | Call::TofStream
            | Call::ChoraleSubscribe => Value::Object(serde_json::Map::new()),
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
            method::SHOW => Call::Show(decode(params)?),
            method::SUBSCRIBE => Call::Subscribe,
            method::ROBOT_SAFE_TO_RESTART => Call::RobotSafeToRestart,
            method::ROBOT_HEALTH => Call::RobotHealth,
            method::ROBOT_MODEL_API => Call::RobotModelApi,
            method::ROBOT_SESSION_ACTIVE => Call::RobotRemoteSessionActive,
            method::ROBOT_MOVE => Call::RobotMove(decode(params)?),
            method::ROBOT_HEAD => Call::RobotHead(decode(params)?),
            method::ROBOT_LOOK => Call::RobotLook(decode(params)?),
            method::ROBOT_STOP => Call::RobotStop,
            method::ROBOT_ENABLE => Call::RobotEnable(decode(params)?),
            method::ROBOT_INIT => Call::RobotInit,
            method::ROBOT_RELAX => Call::RobotRelax,
            method::ROBOT_DO => Call::RobotDo(decode(params)?),
            method::ROBOT_POSE => Call::RobotPose(decode(params)?),
            method::ROBOT_MOUTH => Call::RobotMouth(decode(params)?),
            method::ROBOT_SOUND => Call::RobotSound(decode(params)?),
            method::ROBOT_THEREMIN => Call::RobotTheremin(decode(params)?),
            method::ROBOT_CHORALE => Call::RobotChorale(decode(params)?),
            method::CHORALE_SUBSCRIBE => Call::ChoraleSubscribe,
            method::CHORALE_BEACON => Call::ChoraleBeaconSet(decode(params)?),
            method::CHORALE_HEARD => Call::ChoraleHeard(decode(params)?),
            method::ROBOT_SHUTDOWN => Call::RobotShutdown,
            method::ROBOT_MODE => Call::RobotMode,
            method::ROBOT_SET_MODE => Call::RobotSetMode(decode(params)?),
            method::ROBOT_POLICIES => Call::RobotPolicies,
            method::ROBOT_LOAD_POLICY => Call::RobotLoadPolicy(decode(params)?),
            method::ROBOT_RELOAD_POLICIES => Call::RobotReloadPolicies,
            method::POLICY_CHECK => Call::PolicyCheck,
            method::POLICY_INSTALL => Call::PolicyInstall(decode(params)?),
            method::POLICY_FETCH => Call::PolicyFetch(decode(params)?),
            method::POLICY_SEARCH => Call::PolicySearch(decode(params)?),
            method::ACCOUNT_LOGIN => Call::AccountLogin(decode(params)?),
            method::ACCOUNT_STATUS => Call::AccountStatus,
            method::ACCOUNT_LOGOUT => Call::AccountLogout,
            method::ROBOT_SUBSCRIBE => Call::RobotSubscribe(decode(params)?),
            method::NET_STATUS => Call::NetStatus,
            method::NET_SCAN => Call::NetScan,
            method::NET_CONNECT => Call::NetConnect(decode(params)?),
            method::NET_FORGET => Call::NetForget(decode(params)?),
            method::SYSTEM_INFO => Call::SystemInfo,
            method::SYSTEM_SERVICES => Call::SystemServices,
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
            method::PAD_BINDINGS => Call::PadBindings,
            method::PAD_BIND => Call::PadBind(decode(params)?),
            method::ROBOT_SKILLS => Call::RobotSkills,
            method::ROBOT_SET_SKILL => Call::RobotSetSkill(decode(params)?),
            method::ROBOT_REMOVE_SKILL => Call::RobotRemoveSkill(decode(params)?),
            method::PAD_INPUT => Call::PadInput,
            method::TOF_STREAM => Call::TofStream,
            other => {
                return Err(Error::new(
                    code::METHOD_NOT_FOUND,
                    format!("unknown method {other:?}"),
                ));
            }
        })
    }
}

/// Fixtures for tests, in this crate and in its consumers.
///
/// Behind a feature so nothing here reaches a robot, and it adds no dependencies — this crate is
/// on the recovery path and its dependency list is deliberately three crates long.
///
/// It exists because two copies of [`every_call`] had already appeared, one here and one in
/// `btd::route`, and they had already drifted — 115 lines against 82. A third was about to be
/// written for `mediad`.
// `any(test, ...)` so this crate's own tests reach it without the feature being enabled, which is
// the difference between `cargo test -p duck-ipc-proto` working and not.
#[cfg(any(test, feature = "test-support"))]
pub mod test_support {
    use super::*;

    /// One of every [`Call`] variant, so a test cannot silently skip one.
    ///
    /// The exhaustive matches over [`Call`] — `method`, `destination`, and each transport's
    /// permission table — are what force this list to stay complete: a new variant breaks those
    /// builds, and whoever fixes them arrives here next.
    pub fn every_call() -> Vec<Call> {
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
                    from_dir: None,
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
            Call::Show(ShowParams { run: Some(42) }),
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
            Call::RobotLook(LookParams {
                x: 1.0,
                y: 0.25,
                z: -0.1,
                neck_pitch: 0.2,
            }),
            Call::RobotStop,
            Call::RobotEnable(EnableParams {
                on: true,
                toggle: false,
            }),
            Call::RobotInit,
            Call::RobotRelax,
            Call::RobotDo(DoParams {
                skill: "ground_pick".into(),
            }),
            Call::RobotPose(PoseParams {
                z: -0.01,
                roll: 0.05,
                pitch: -0.1,
                active: true,
            }),
            Call::RobotMouth(MouthParams { open: 0.5 }),
            Call::RobotSound(SoundParams {
                tag: SoundTag::Chirp,
                hold: None,
            }),
            Call::RobotShutdown,
            Call::RobotMode,
            Call::RobotPolicies,
            Call::RobotReloadPolicies,
            Call::PolicyCheck,
            Call::PolicyInstall(PolicyInstallParams {
                version: Some("v2".into()),
            }),
            Call::PolicyFetch(PolicyFetchParams {
                repo: "RemiFabre/microduck-flamingo-cycle".into(),
                revision: None,
                file: None,
            }),
            Call::PolicySearch(PolicySearchParams {
                query: "microduck".into(),
            }),
            Call::AccountLogin(AccountLoginParams { force: false }),
            Call::AccountStatus,
            Call::AccountLogout,
            Call::RobotLoadPolicy(LoadPolicyParams {
                slot: Some("walk".into()),
                path: Some("/var/lib/robot/policies/bouncy.onnx".into()),
            }),
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
            Call::SystemServices,
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
            Call::RobotSkills,
            Call::RobotSetSkill(SkillParams::default()),
            Call::RobotRemoveSkill(SkillNameParams {
                name: "polite-bow".into(),
            }),
            Call::PadBindings,
            Call::PadBind(PadBindParams {
                button: "x".into(),
                skill: Some("polite-bow".into()),
            }),
            Call::PadInput,
            Call::TofStream,
        ]
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

    /// A raw-pad notification: no `id`, so no response is expected.
    pub fn notify_pad_report(report: &PadReport) -> Self {
        Self {
            jsonrpc: JSONRPC_VERSION.to_owned(),
            id: None,
            method: method::PAD_REPORT.to_owned(),
            params: Some(serde_json::to_value(report).unwrap_or(Value::Null)),
        }
    }

    /// Read a raw-pad notification back.
    pub fn as_pad_report(&self) -> Option<PadReport> {
        if self.method != method::PAD_REPORT {
            return None;
        }
        serde_json::from_value(self.params.clone()?).ok()
    }

    /// A depth-frame notification: no `id`, so no response is expected.
    pub fn notify_tof_frame(frame: &TofFrame) -> Self {
        Self {
            jsonrpc: JSONRPC_VERSION.to_owned(),
            id: None,
            method: method::TOF_FRAME.to_owned(),
            params: Some(serde_json::to_value(frame).unwrap_or(Value::Null)),
        }
    }

    /// Read a depth-frame notification back.
    pub fn as_tof_frame(&self) -> Option<TofFrame> {
        if self.method != method::TOF_FRAME {
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

/// Every params type below denies unknown fields, and this is the one place that is worth saying
/// why rather than repeating it fourteen times.
///
/// `serde` ignores what it does not know, so without this a client from another release can send a
/// member that changes what a call *means* and be told nothing: v7's `ApplyOptions::from_dir` is the
/// worked example — a daemon that ignores it installs from its configured source while the operator
/// believes they are sideloading a directory. Refusing it names the member, on the one call that
/// cannot be served, which is what [`API_VERSION`]'s handshake gate was standing in for until it was
/// removed for firing on every other call too.
///
/// The robot-intent types (`MoveParams`, `HeadParams`, …) had it from the start; the updater and
/// config ones did not, because the gate was covering for them.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HelloParams {
    pub api_version: u32,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
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

/// A point to look at, trunk frame, metres — see [`method::ROBOT_LOOK`]. The gaze form
/// [`HeadParams`]' doc promised: the daemon solves the IK against its own MJCF model, so a
/// client never has to know which way a positive head_yaw turns.
///
/// `neck_pitch` is posture, not aim — the IK holds it and aims around it. Defaults to 0.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct LookParams {
    /// Forward of the trunk origin.
    pub x: f64,
    /// Left of it.
    pub y: f64,
    /// Above it. NOTE: trunk frame, not floor — the floor is about 0.12 m below.
    pub z: f64,
    pub neck_pitch: f64,
}

/// Answer to [`Call::RobotLook`]: the joints the head was sent to.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct LookResult {
    /// What was handed to the same path `robot.head` feeds — resend these to hold the gaze.
    pub head: HeadParams,
    /// The point is beyond the head's reach (travel limits, or the gimbal geometry near
    /// ±90° yaw); the joints are the closest gaze, not a lock.
    pub clamped: bool,
}

/// A voice-bank tag — what kind of sound, not which file: the robot picks a random
/// variant per play, which is what keeps the duck from sounding like a stuck recording.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SoundTag {
    /// Sharp honk.
    Alarm,
    /// Wake-up quack (sometimes a double "wak-wak").
    Greet,
    /// Rising question.
    Inquire,
    /// Low "tock" — the goodbye before power-off.
    Peck,
    /// The mouth-trigger quack. `robotctl quack` plays this.
    Chirp,
    /// Drowsy, breathy — the petting response.
    Coo,
    /// The held joy ride: start → loop while held → end.
    Wheee,
}

impl SoundTag {
    /// The voice-bank directory name.
    pub fn as_str(&self) -> &'static str {
        match self {
            SoundTag::Alarm => "alarm",
            SoundTag::Greet => "greet",
            SoundTag::Inquire => "inquire",
            SoundTag::Peck => "peck",
            SoundTag::Chirp => "chirp",
            SoundTag::Coo => "coo",
            SoundTag::Wheee => "wheee",
        }
    }
}

/// See [`method::ROBOT_SOUND`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SoundParams {
    pub tag: SoundTag,
    /// Only meaningful for [`SoundTag::Wheee`]: `Some(true)` starts/keeps the ride,
    /// `Some(false)` releases it. The hold decays on its own if the trues stop arriving,
    /// so a client that dies mid-ride does not leave the robot going "wheee" forever.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub hold: Option<bool>,
}

/// See [`method::ROBOT_THEREMIN`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ThereminParams {
    /// True picks the instrument up, false puts it down. Idempotent both ways: a client
    /// that cannot remember whether it already asked may simply ask again.
    pub active: bool,
}

/// Answer to [`Call::RobotTheremin`].
///
/// Refused only for what the robot can know at the door: no voice, no depth frames, the
/// feature switched off. Once accepted it plays immediately — there is no arming step — and
/// [`RobotState::theremin`] carries what it is doing, including what the sensor is saying
/// about each frame.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct ThereminResult {
    pub accepted: bool,
    /// Why not, when `accepted` is false.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
}

impl Default for ThereminResult {
    fn default() -> Self {
        Self {
            accepted: true,
            reason: None,
        }
    }
}

/// What the theremin is doing, in [`RobotState`].
///
/// Absent from the frame entirely while the instrument is down, which is the ordinary state
/// of a duck — so a client that never asks for a theremin never pays for one in its state
/// stream, and one built before this simply does not see the field.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ThereminState {
    /// Distance to the hand being played, metres. `None` when nothing is in the playable
    /// band — which is silence, not an error.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hand_range_m: Option<f64>,
    /// The note being sounded, hertz. `None` when silent.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub note_hz: Option<f64>,
    /// How far open the mouth is being driven, 0..1 — the same number the pitch came from.
    pub mouth: f64,
    /// How many zones the hand covers. Zero while silent. The number that says whether a
    /// dropout was the hand leaving or the sensor blinking.
    pub zones: u32,
    /// This note is the held memory of a frame just gone, bridging a sensor dropout, rather
    /// than something measured now. Reported so a readout can show that rather than implying
    /// it still sees a hand.
    #[serde(default, skip_serializing_if = "not")]
    pub held: bool,
    /// What the sensor said about this frame, as a line: how many zones carry a status the
    /// robot believes, then the count per status code with a `*` on the believed ones — e.g.
    /// `12 usable · 255:40 4*:12 5*:8 1:4`.
    ///
    /// Diagnostic, and the one worth carrying on the wire: a theremin that stops working past
    /// 30 cm and a frame where status 4 covers half the grid are the same fact, but only the
    /// second says what to change. The first version of this feature accepted only ST's two
    /// "valid" codes and died at exactly that distance, invisibly, for want of this line.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sensor: Option<String>,
}

/// The one-shot skills, plus the sit↔stand toggle. See [`method::ROBOT_DO`].
///
/// An enum rather than a free string so a typo is [`code::INVALID_PARAMS`] at the door,
/// not a silently ignored request.
/// Which one-shot skill to run, for [`Call::RobotDo`].
///
/// **A name, not an enumeration.** It was five variants — ground pick, the two kicks, the sit
/// toggle and roulade — each of which needed a `Net`, a config slot and a branch in the control
/// loop before a robot could do it. A robot's skills are now a list in its config, so what a
/// client may ask for is whatever that robot has, and `robot.subscribe`'s acknowledgement is
/// where a client learns the names rather than assuming them.
///
/// The five above are still the names a stock robot answers to. An unknown one is refused with
/// the list it does know, which is the same shape as a bad policy slot.
pub type Skill = String;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DoParams {
    pub skill: Skill,
}

/// Standing body pose offsets. Continuous intent — see [`method::ROBOT_POSE`].
///
/// The trained ranges are small: z −0.025..+0.010 m, roll and pitch ±0.26 rad. The robot
/// clamps nothing here — out-of-distribution values just produce a policy leaning on inputs
/// it never saw, so the client should stay inside them.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct PoseParams {
    /// Height offset, metres. Negative crouches.
    pub z: f64,
    pub roll: f64,
    pub pitch: f64,
    /// While true the pose targets above are glided toward; `false` snaps the body back to
    /// nominal at once, which is the prototype's B-button exit.
    pub active: bool,
}

impl Default for PoseParams {
    fn default() -> Self {
        Self {
            z: 0.0,
            roll: 0.0,
            pitch: 0.0,
            active: true,
        }
    }
}

/// Mouth opening. Continuous intent — see [`method::ROBOT_MOUTH`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct MouthParams {
    /// 0 closed, 1 fully open. Clamped by the robot.
    pub open: f64,
}

/// Which mode to switch to, for [`Call::RobotSetMode`].
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct SetModeParams {
    /// `"walk"` or `"roller"`. A string for the same reason [`ModeResult`] carries one: a mode
    /// this build has never heard of should come back as a refusal naming what it does know,
    /// not as a parse error with no explanation in it.
    pub mode: String,
}

/// Answer to [`Call::RobotMode`].
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct ModeResult {
    /// `"walk"` or `"roller"`. A string rather than an enum so a client older than a new
    /// mode reports it instead of failing to parse the answer.
    pub mode: String,
}

/// Which slot to change, and to what, for [`Call::RobotLoadPolicy`].
///
/// The two `Option`s mirror the config key this ends up writing: `[policy] walk` is an
/// `Option<PathBuf>` where unset means "resolve this slot's default for the current mode", so
/// clearing an override and asking for the default are the same operation on the wire as they
/// are on disk.
///
/// | `slot` | `path` | means |
/// |---|---|---|
/// | `Some` | `Some` | run this file in this slot |
/// | `Some` | `None` | drop this slot's override, back to the default |
/// | `None` | `None` | drop every override — "put it all back" |
/// | `None` | `Some` | refused; there is no such thing as loading one file into every slot |
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct LoadPolicyParams {
    /// `"walk"`, `"stand"`, `"sitstand"`, `"ground_pick"`, `"kick_left"`, `"kick_right"` or
    /// `"roulade"`. A string for the reason [`SetModeParams`] carries one: a slot this build
    /// does not have should be refused with the list of ones it does, not fail to parse.
    pub slot: Option<String>,
    /// The `.onnx` to run. Absolute — the daemon resolves nothing relative, because its working
    /// directory is not the caller's and a path that meant one file to each would be worse than
    /// a refusal.
    pub path: Option<String>,
}

/// Answer to [`Call::RobotPolicies`] — what is actually loaded, not what was configured.
///
/// The distinction is the point. A slot whose override failed to load reports the file it fell
/// back to in `path` and says why in `error`, so "I loaded a policy and the robot did not
/// change" has an answer that does not require reading the journal.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct PoliciesResult {
    /// `"walk"` or `"roller"` — which mode's slot defaults are in play.
    pub mode: String,
    /// Whether policies are loaded at all (`[policy] enabled`). False is a legitimate bench
    /// configuration, not a fault — the slots below then say what *would* load, and nothing is
    /// running. Without this a client cannot tell a deliberately quiet robot from a broken one.
    pub enabled: bool,
    /// One entry per slot this build has, including the empty ones.
    pub slots: Vec<PolicySlot>,
    /// The one-shot skills this robot has, in priority order — the names `robot.do` answers to.
    ///
    /// **A client cannot offer a "bow" button without knowing the robot has a bow.** Which skills
    /// exist is config now, so there is no list to compile in; the other place it is published is
    /// `robot.subscribe`'s acknowledgement, and that is a 50 Hz stream — the wrong thing to open
    /// over BLE to answer a question asked once. Here it rides the read a client already makes to
    /// show what is loaded.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub skills: Vec<String>,
    /// Why the last policy change failed, when it was not a change to one slot.
    ///
    /// **A slot's failure is reported on the slot**; this is for the two that name none — a
    /// reload and a whole-robot reset — where the answer would otherwise be a log line on the
    /// robot and silence on the wire. `robot.setSkill` accepts and then triggers a reload, so
    /// without this a client is told a skill was added and discovers it was not by pressing the
    /// button.
    ///
    /// Cleared by the next change that succeeds. The robot is running the policy it had
    /// throughout — a failed change swaps nothing — so this is *degraded*, never unhealthy.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub change_error: Option<String>,
}

/// One policy slot's state, for [`PoliciesResult`].
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct PolicySlot {
    /// `"walk"`, `"stand"`, …
    pub slot: String,
    /// The file running in it. Absent means the slot is empty, which is a capability this
    /// robot does not have rather than a fault — roller mode has no standing network.
    pub path: Option<String>,
    /// `"official"`, `"community"` or `"local"`. Absent when the slot is empty.
    ///
    /// A string rather than an enum so a client that predates an origin reports it verbatim
    /// instead of failing to parse a robot's answer about what it is running.
    pub origin: Option<String>,
    /// Whether config overrides this slot, as opposed to it resolving to the mode's default.
    pub overridden: bool,
    /// Why this slot is not running what was asked of it. Set when an override failed to load
    /// and the default was used instead; `path` then names what is actually running.
    pub error: Option<String>,
}

/// Which set to install, for [`Call::PolicyInstall`].
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct PolicyInstallParams {
    /// A revision in the policy repo — a tag like `v2`, or a branch. Absent means whatever
    /// [`Call::PolicyCheck`] calls the newest, which is the ordinary case and saves a client
    /// having to make two calls to do the obvious thing.
    pub version: Option<String>,
}

/// Answer to [`Call::PolicyCheck`].
///
/// Deliberately says what is installed *and* what exists, rather than a bare "update available".
/// A robot two versions behind, a robot ahead of its release's pin, and a robot whose Hub is
/// unreachable are three different situations, and a boolean makes them all look alike.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct PolicyCheckResult {
    /// The Hub repo the installed set came from, per its own provenance record.
    pub repo: Option<String>,
    /// The revision installed, e.g. `v1`.
    pub installed: Option<String>,
    /// The newest revision the repo offers, if the Hub could be reached.
    pub available: Option<String>,
    /// Every revision the repo offers, newest first. What makes going *back* possible.
    pub versions: Vec<String>,
    /// Why `available` is absent. An unreachable Hub is a fact to report, not an error to fail
    /// on — the robot is walking either way.
    pub unreachable: Option<String>,
}

/// Answer to [`Call::PolicyInstall`].
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct PolicyInstallResult {
    /// What is now live.
    pub installed: String,
    /// What was live before, when it was something else. Absent on a first install.
    pub previous: Option<String>,
    /// Whether `robotd` picked the new set up. False means the files are in place and the robot
    /// is still running the old ones — a restart away from correct, and worth saying so rather
    /// than reporting a success the robot has not acted on.
    pub reloaded: bool,
}

/// Which policy to fetch, for [`Call::PolicyFetch`].
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct PolicyFetchParams {
    /// A Hub repo, `org/name`.
    pub repo: String,
    /// A branch, tag or commit. Absent means `main`.
    pub revision: Option<String>,
    /// Which file in the repo. Absent means "the only `.onnx` in it", which is what every
    /// microduck policy repo published so far actually has — they all carry one `policy.onnx`.
    /// A repo with several is a refusal naming them, not a guess.
    pub file: Option<String>,
}

/// What a fetched policy turned out to be, for [`Call::PolicyFetch`].
///
/// Most of this comes from the repo's `manifest.json`, which the community convention already
/// carries and which is **untrusted**: it is a stranger's description of a stranger's file. It is
/// worth reading anyway, because a policy that says it is 51-D can be refused before the download
/// rather than after the robot has been asked to run it — and a manifest that lies is caught by
/// the shape gate at load, which is where the real check has always been.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PolicyFetchResult {
    /// Where it landed. This is what goes in `[policy] <slot>`.
    pub path: String,
    pub repo: String,
    pub revision: String,
    /// The commit the revision resolved to, so a moving branch can be noticed later.
    pub commit: Option<String>,
    pub file: String,
    /// `"official"` or `"community"`, by the org the repo belongs to.
    pub origin: String,
    /// The manifest's own name for it, if it had one.
    pub name: Option<String>,
    /// One line about what it does, from the manifest. Untrusted display text.
    pub description: Option<String>,
    /// The manifest's `kind` — `episodic` for a policy that returns itself to a safe pose,
    /// `perpetual` for one that holds until told otherwise. Untrusted, and what decides whether
    /// the daemon has to supply an ending.
    pub kind: Option<String>,
    /// How long the policy runs, from the manifest. Absent for a perpetual one, which has no
    /// length of its own — a caller wanting it as a one-shot has to say how long to hold it.
    pub duration_s: Option<f64>,
    /// The manifest's `action_scale`, if it declared one.
    pub action_scale: Option<f64>,
    /// Seconds a perpetual policy needs to get back to its idle command, from the manifest.
    pub unwind_s: Option<f64>,
    /// The manifest's `command.idle` — the twist that means "stop", and what a skill unwinds to.
    pub idle: Option<[f64; 3]>,
    /// The manifest's `command.encoding`: absent or `"constant"` for the skill family, `"phase"`
    /// for a ground pick, `"posture_flag"` for a sit↔stand. What decides whether the policy can
    /// be a generic one-shot at all — a phase policy fed a constant is a robot moving plausibly
    /// and wrongly.
    pub encoding: Option<String>,
    /// Whether holding the button chains another run, from the manifest.
    pub chain: bool,
    /// The manifest's `schema_version`, so a client can say which convention it read.
    pub schema_version: Option<u32>,
}

/// What to search the Hub for, for [`Call::PolicySearch`].
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct PolicySearchParams {
    pub query: String,
}

/// What to pass [`Call::AccountLogin`].
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AccountLoginParams {
    /// Sign in even though this robot already belongs to an account, or is already waiting for
    /// a code to be approved.
    ///
    /// Without it, a robot that is already signed in refuses with [`code::INVALID_PARAMS`] naming
    /// the account it belongs to, and one with a live code refuses with [`code::BUSY`]. The
    /// second is the same permission as the first in a smaller size — replacing an *attempt* at
    /// belonging to somebody — and it is the only way out of a code nobody is going to approve
    /// that does not mean signing the robot out or waiting five minutes for it to expire.
    ///
    /// A superseded login is inert: it keeps its device code, and if Hugging Face approves it
    /// later the answer is dropped rather than written. That is not ceremony: `account.login` is routed to BLE and to a
    /// datachannel, so on a LAN anybody who can reach the robot can start one — and a login that
    /// silently replaced the owner would convert "was on the wifi once" into remote access that
    /// outlives being there. Taking a robot from its owner should require saying so.
    #[serde(default)]
    pub force: bool,
}

/// Answer to [`Call::AccountLogin`], and the `login` member of [`AccountStatusResult`].
///
/// RFC 8628: show `user_code` to a person, send them to `verification_uri`, and poll
/// [`Call::AccountStatus`] until it says who signed in.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AccountLoginResult {
    /// The short code somebody types. Display it prominently — see the note on
    /// `verification_uri_complete`.
    pub user_code: String,
    /// Where they type it.
    pub verification_uri: String,
    /// RFC 8628's "page that needs no code typed into it", when a server sends one.
    ///
    /// **Hugging Face does not, and today this is `verification_uri` unchanged.** An earlier
    /// version of the robot appended `?user_code=`; HF's device page ignores the parameter and
    /// prefills nothing, so that URL read as a promise the page then broke. The field stays
    /// because it is the protocol's — a server that starts sending a real one is used with no
    /// wire change — which means a client must treat it as "somewhere to send them", not as
    /// "the code is handled".
    ///
    /// So whatever this says, a client **leads with the code and never opens a browser by
    /// itself**. The mini's app learned that the expensive way: auto-switching to Safari hid the
    /// code before the user could read it, and the page asks them to type it anyway.
    pub verification_uri_complete: String,
    /// How long the code is good for. In [`AccountStatusResult`] this is what is *left*.
    pub expires_in: u64,
    /// How often the robot polls Hugging Face, in seconds. Informational — the robot does the
    /// polling. A client polls [`Call::AccountStatus`] at whatever rate suits it.
    pub interval: u64,
}

/// Answer to [`Call::AccountStatus`].
///
/// Three independent facts rather than one state, because they genuinely are: a `force` login can
/// be in flight while the robot still belongs to the previous account, and a login that failed
/// leaves both the previous account and a reason.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct AccountStatusResult {
    /// Who this robot belongs to. `None` means it belongs to nobody and cannot be reached from
    /// outside its LAN.
    pub account: Option<Account>,
    /// A login in flight, with the code still to be approved and the time it has left.
    pub login: Option<AccountLoginResult>,
    /// Why the last login attempt failed, when one did and nothing has superseded it.
    pub last_error: Option<String>,
}

/// The account a robot belongs to, in [`AccountStatusResult`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Account {
    /// The Hugging Face username, as `/oauth/userinfo` reports it. What to show a person.
    pub username: String,
    /// Seconds until the stored access token expires, negative once it has.
    ///
    /// Present so a client can say "signed in, but this robot has not been online in a month"
    /// rather than only discovering it when the robot stops appearing. The robot refreshes on its
    /// own well before this reaches zero; it going negative means the robot could not.
    pub token_expires_in: i64,
    /// Whether a refresh token is stored, so an expiring access token can be renewed without
    /// anybody signing in again.
    pub refreshable: bool,
}

/// Answer to [`Call::AccountLogout`].
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct AccountLogoutResult {
    /// Who it was signed in as, or `None` if it was not signed in at all — so a client can tell
    /// "signed out" from "there was nothing to sign out of" without asking first.
    pub was: Option<String>,
}

/// Answer to [`Call::PolicySearch`].
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct PolicySearchResult {
    pub models: Vec<PolicySearchHit>,
}

/// One Hub model matching a search. Everything here is written by whoever published it.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct PolicySearchHit {
    /// `org/name`.
    pub id: String,
    pub origin: String,
    pub likes: Option<u64>,
    pub downloads: Option<u64>,
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
    /// The skill networks this process loaded, as file names, same reasoning as `walk`.
    /// Absent means that skill is not available on this robot.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sitstand: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ground_pick: Option<String>,
    /// The configurable one-shot skills this robot has, in priority order, and the names
    /// `robot.do` answers to. A list rather than a field per skill, because which skills a robot
    /// has is now config: a client learns them here instead of assuming five.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub skills: Vec<String>,
}

/// Whether the policy should run. Discrete intent — see [`method::ROBOT_ENABLE`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EnableParams {
    pub on: bool,
    /// Flip the robot's current state instead of setting `on` (which is then ignored).
    ///
    /// This is the gamepad's Start button, and it exists because the alternative was the
    /// client keeping its own on/off belief — which drifts from the robot's the moment
    /// anything else moves the state (`robot.relax`, the shutdown sequence, either side
    /// restarting), and a stale belief turns Start into a button that does nothing every
    /// other press. The robot owns the toggle, so a press always means "the other one".
    ///
    /// Toggling OFF returns the robot to its home pose — commanded directly, the
    /// prototype's "returning to default pose" — so the next toggle ON hands the policy a
    /// robot already standing at home. The reply's `reason` names the state the robot
    /// ended in.
    #[serde(default)]
    pub toggle: bool,
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
#[serde(deny_unknown_fields)]
pub struct ApplyOptions {
    /// Run every check (fetch, verify, compatibility, space) and stop before the symlink
    /// swap.
    #[serde(default)]
    pub dry_run: bool,
    /// Skip *only* the "no active remote session" preflight check. Never bypasses
    /// signature, hash or compatibility — those have no override.
    #[serde(default)]
    pub interrupt_sessions: bool,
    /// Take the release from this directory **on the robot** instead of the component's
    /// configured source. The laptop-to-board path: `scripts/dev-push.sh`.
    ///
    /// Changes where the bytes come from, not what is trusted. The directory is read by
    /// the same `LocalDir` source the tests and the offline installer use, so the
    /// manifest signature, the artifact hash and the compatibility checks all still have
    /// to pass — a locally built release installs because the dev key is in the robot's
    /// trusted set, not because anything is skipped.
    ///
    /// A `String` rather than a `PathBuf` because this is a JSON wire type: a path that
    /// is not UTF-8 cannot cross this socket in either form, and the plain type says so
    /// instead of failing at serialisation. It is also interpreted by `updaterd`, whose
    /// working directory is `/` — so a client sends an absolute path, and `robotctl`
    /// resolves one before it gets here.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub from_dir: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ApplyParams {
    pub component: ComponentId,
    pub target: Target,
    #[serde(default)]
    pub options: ApplyOptions,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SelectParams {
    pub component: ComponentId,
    pub version: semver::Version,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PinParams {
    pub component: ComponentId,
    /// `None` unpins.
    pub version: Option<semver::Version>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LogParams {
    pub limit: usize,
}

/// Which run to transcribe.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ShowParams {
    /// `None` means the most recent run, which is what someone debugging an update that has
    /// just happened wants and should not have to look up first.
    pub run: Option<u64>,
}

/// Join a wifi network.
///
/// [`Debug`] is hand-written to redact `psk`, and that is the point of the type. Every other
/// params struct derives it, and a derived one here would put a customer's wifi password into
/// the journal the first time any service logged a request it could not handle — `configd`,
/// `btd` and `robotctl` all log calls, and the credential-carrying one must not be readable
/// afterwards by anyone who can run `journalctl`.
#[derive(Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
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
#[serde(deny_unknown_fields)]
pub struct NetForgetParams {
    pub ssid: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SetNameParams {
    pub name: String,
}

/// Prove knowledge of the pairing PIN.
///
/// [`Debug`] is hand-written to redact the PIN, for the same reason [`NetConnectParams`] is: this
/// is the only thing standing between a paired-but-unauthenticated peer and the robot, and a
/// journal is the wrong place for it.
#[derive(Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
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
#[serde(deny_unknown_fields)]
pub struct SetPairingPinParams {
    pub pin: String,
}

// ── pad.* parameters ─────────────────────────────────────────────────────────

/// Pair the gamepad that is in pairing mode now.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
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
#[serde(deny_unknown_fields)]
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
        /// Units running something other than `version`, which this outcome has scheduled a restart
        /// for. Empty in the ordinary case, where nothing was installed because nothing needed to be.
        ///
        /// It earns a field rather than only a log line because "already current" is otherwise
        /// indistinguishable from "already current, and a daemon is not running it" — the state that
        /// made a recovery command look like a confirmation that there was nothing to recover.
        ///
        /// `default`, so an older `updaterd`'s reply still parses: it reports no stale units because
        /// it did not look, which reads the same as finding none. `ApplyResult` does not
        /// `deny_unknown_fields`, so an older client ignores the field rather than failing to decode
        /// the outcome of an update it just performed.
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        stale: Vec<String>,
    },
    /// Everything verified; stopped before the swap because `dry_run` was set.
    DryRunPassed { candidate: semver::Version },
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
    /// Unix seconds. Stamped when the attempt *finished*, not when it started — the entry is
    /// written on the way out of `apply`. Anyone reading backwards from here for the run's own
    /// logs has to walk backwards; [`RunTranscript`] carries both ends and spares them that.
    pub at: i64,
    pub component: ComponentId,
    pub from: Option<semver::Version>,
    pub to: Option<semver::Version>,
    pub outcome: Outcome,
    /// Which transcript holds what this attempt actually did — [`Call::Show`]'s argument.
    ///
    /// `None` for entries written before transcripts existed, and for an attempt that failed
    /// before one could be opened. `default` and `skip_serializing_if`, so an old log file still
    /// parses and a new one still means something to an older client.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub run: Option<u64>,
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

/// One line of an update's transcript.
///
/// **On disk, not only in the journal**, for the reason the update log itself is
/// (`updater::journal`): the record of what an update did must survive the symlink swap, the
/// rollback, and the power loss the update may itself have provoked. `/var/log` on this board is
/// zram, so the journal survives a clean reboot and not a power cut — and a power cut is one of
/// the likelier endings of exactly the updates someone needs this for.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RunRecord {
    /// Unix seconds.
    pub at: i64,
    /// Flattened, so a line of the file reads as one flat object. This file is meant to be
    /// `cat`ed and `grep`ed on a board where `robotctl` may itself be the thing that is broken,
    /// and `{"at":…,"event":{"event":"phase",…}}` is worse to read than `{"at":…,"event":"phase",…}`.
    #[serde(flatten)]
    pub event: RunEvent,
}

/// What happened, at one moment of one update.
///
/// Typed rather than a free-text line, because the renderer aligns and colours these and a
/// machine reading `--json` should not have to parse prose. [`Self::Note`] is the escape hatch
/// for the genuinely shapeless, and a variant this release does not know decodes as
/// [`Self::Unrecognised`] rather than failing the whole transcript — a newer `updaterd`'s
/// record must stay readable by the `robotctl` that is on the board asking about it, which
/// during a partial update is precisely the pairing that exists.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "event", rename_all = "snake_case")]
pub enum RunEvent {
    /// The run opened: what was asked for, by whom, and what was live at the time.
    Began {
        component: ComponentId,
        /// The target as the caller named it — `latest`, `0.1.4`, `ref my-branch`, `staging`,
        /// `dir /home/pi/push`. Rendered, not structured: it exists to be read back to whoever
        /// is asking "what did I actually run", and `Target` already carries the structure.
        target: String,
        /// The release live when the run began.
        installed: Option<semver::Version>,
        /// Where the manifest was going to be fetched from.
        source: String,
        /// `uid=1000 gid=1000 pid=2317`, from `SO_PEERCRED`. `None` for the unattended path and
        /// for the sideload CLI, where there is no peer.
        requested_by: Option<String>,
    },
    /// A phase boundary. The backbone of the transcript.
    Phase {
        phase: Phase,
        detail: Option<String>,
    },
    /// The manifest that passed its signature check — every fact that decides what follows.
    Manifest {
        version: semver::Version,
        sha256: String,
        bytes: Option<u64>,
        url: Option<String>,
        /// Which trusted key verified it. A set of keys is allowed, so which one matters.
        signed_by: Option<String>,
        source_revision: Option<String>,
    },
    /// A hook ran, with its output verbatim. The richest thing in the file: this is the ONNX
    /// Runtime and GStreamer install talking, and its whole report is the answer to "can this
    /// board encode H.264 at this release".
    Hook {
        hook: String,
        exit_code: Option<i32>,
        output: String,
    },
    /// A unit was restarted, reloaded, or scheduled for restart, and how that went.
    Unit {
        unit: String,
        action: String,
        detail: Option<String>,
    },
    /// The health gate's verdict — the thing that decides commit versus rollback.
    Health {
        passed: bool,
        detail: Option<String>,
    },
    /// Worth writing down, with no shape of its own.
    Note { text: String },
    /// How the run ended. Absent from a transcript whose run was cut short — by the deferred
    /// restart of `updaterd` itself, or by the power going away — and that absence is the point:
    /// a transcript with no `ended` is a run whose verdict is elsewhere.
    Ended {
        /// The update log's verdict, where this run produced one. `None` for the two outcomes the
        /// log deliberately does not keep — a dry run, and an apply that found nothing to do —
        /// which are still perfectly good runs to have a transcript of.
        outcome: Option<Outcome>,
        /// One sentence, always. What `robotctl update show` prints as the last line, and what
        /// spares a reader from having to interpret the tagged outcome above it.
        summary: String,
    },
    /// The per-run caps stopped the writer, and this many events were dropped after it.
    ///
    /// A typed event rather than a flag on the transcript, because it has a *place*: everything
    /// before it was recorded, everything between it and the end was not. A boolean could say
    /// that something was missing but not where the hole is.
    Truncated { dropped: u64 },
    /// A variant from a later release. Kept as a placeholder so one unknown line does not cost
    /// the reader the rest of the run.
    #[serde(other)]
    Unrecognised,
}

/// Answer to [`Call::Show`]: one run, in full.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RunTranscript {
    pub run: u64,
    pub component: ComponentId,
    /// Oldest first. This is a story and it is read forwards.
    pub events: Vec<RunRecord>,
    /// Every run still on disk, newest first — so a client that asked for the latest can say
    /// what else there is without a second call.
    #[serde(default)]
    pub available: Vec<u64>,
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

    /// Accepted, and there was nothing to do — the robot is already in the state asked for.
    ///
    /// Distinct from [`Self::accepted`] only by carrying a reason, which is the whole signal: an
    /// acceptance that queued no work still succeeded, but a caller that then waits for something
    /// to change is waiting for a change that is never coming. `robot.loadPolicy` is the case
    /// this exists for — resetting a slot that was never overridden would otherwise send the
    /// robot home, reload every network and come back to exactly what it was running.
    pub fn already(reason: impl Into<String>) -> Self {
        Self {
            accepted: true,
            reason: Some(reason.into()),
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
    /// Where contact odometry believes the robot is. `default` so a frame from
    /// a `robotd` predating the estimator still parses — zeros, like a robot
    /// that has not moved.
    #[serde(default)]
    pub odom: OdomState,
    /// What the ToF theremin is doing, when one is being played. Absent while the
    /// instrument is down — see [`ThereminState`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub theremin: Option<ThereminState>,
    /// What the chorale is doing, when one is running. Absent otherwise, which is the ordinary
    /// state of a duck — see [`ChoraleState`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub chorale: Option<ChoraleState>,
}

/// What the duck chorale is doing, in [`RobotState`].
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ChoraleState {
    /// Looking for other ducks — on the air and scanning.
    pub listening: bool,
    /// Which part this duck is singing: `bass`, `tenor`, `alto`, `soprano`. `None` while it is
    /// listening, alone, or waiting to be seated by a conductor.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub part: Option<String>,
    /// A conductor has been adopted and the phase lock is still filling — the state between
    /// "waiting for company" and "singing". Carried because a readout that showed both as
    /// "listening" hid a locking failure through three debugging sessions.
    #[serde(default, skip_serializing_if = "not")]
    pub joining: bool,
    /// How far into the piece, in beats. `None` when not singing.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub beats: Option<f64>,
    /// How many ducks are singing, this one included.
    pub voices: u32,
}

/// The contact-odometry estimate: trunk pose in the world frame the IMU chose
/// at boot. There is no magnetometer and no absolute reference — this frame is
/// "wherever the robot was when it came up", which is exactly what relative
/// motion (walked distance, turn angle, a ToF map) needs and all it promises.
#[derive(Debug, Default, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct OdomState {
    /// Trunk position, metres. Z is height above the ground plane.
    pub position: [f64; 3],
    /// Heading, radians.
    pub yaw: f64,
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
    /// Per-device identity: the SoC serial, which the default name is derived from.
    ///
    /// The durable handle a client should key on. It outlives a rename, and it outlives a change of
    /// Bluetooth address — which is not hypothetical, so an app that remembers a robot by its
    /// peripheral identifier alone will lose it (`app-path-design.md` §8.6).
    ///
    /// `None` where there is none to read rather than a fabricated value; the robot then falls back
    /// to its hostname for a name.
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

/// Whether one of the robot's units is running, as systemd sees it.
///
/// Named for the unit rather than for `padd`, which is where it started: the same answers are what
/// [`ServiceUnit`] needs about every daemon. The wire form is unchanged by that rename — these
/// serialise as their own names, not as the type's.
///
/// **`Restarting` and `Failed` are here because folding them into their neighbours made a robot
/// with an unplugged camera report a healthy one.** `mediad` exits when it cannot build a pipeline
/// and `Restart=always` brings it back five seconds later, forever; systemd calls that
/// `ActiveState=activating`, `SubState=auto-restart`. Read as `Active` — which is what happened,
/// for `padd`'s good reason about a daemon still connecting at boot — it printed as a running
/// daemon, and because systemd deletes `RuntimeDirectory=` on every stop there was no
/// [`Identity`] to name a build either. The whole of a crash loop reached the operator as
/// `active · build unknown (old)`: the one explanation that could not be true, since every daemon
/// in this workspace has published its identity since the commit that introduced it.
///
/// `Failed` splits out for the smaller version of the same complaint: it read as `Inactive`, which
/// is also what a deliberate `systemctl stop` reads as, and those are not the same news.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum UnitState {
    /// Running. With a connected pad, the robot is drivable.
    Active,
    /// Between processes: it exited and systemd is starting it again. Once for an ordinary restart,
    /// or every `RestartSec=` for a daemon that cannot start at all — this state cannot tell those
    /// apart, and nothing that reads it should pretend otherwise.
    Restarting,
    /// It could not start, and systemd has stopped trying.
    Failed,
    /// The unit exists and is not running, because something stopped it. A robot whose owner has no
    /// gamepad and disabled `padd` is the ordinary case, which is why this is not a fault.
    Inactive,
    /// No such unit on this board — a release older than the one that added it.
    Absent,
    /// Could not ask: no systemd, or the query failed. Distinct from `Absent`, because "I do not
    /// know" must not read as "it is not installed".
    Unknown,
}

/// One daemon, as systemd and `/proc` describe it. Answer element for [`Call::SystemServices`].
///
/// **This exists to answer "which version is actually running", which nothing else could.**
/// `updaterd`, `robotd` and `configd` report their build over their own sockets, so a running
/// daemon that did not restart into a new release is already visible for those three. `btd` and
/// `padd` cannot be asked, and the reason is not merely that they have no service socket — `padd`
/// does serve one, for [`method::PAD_INPUT`] and nothing else. It is that the process which needs
/// interrogating is by definition the *old* one, which cannot have learned a new way to answer,
/// whatever socket it is holding.
///
/// So it is read from outside the process: systemd knows the PID, and `/proc/<pid>/exe` resolves to
/// the real path the binary was executed from. Since a release installs to `…/releases/<version>/`
/// and the unit points at a `current` symlink, that path names the release the process is running —
/// including when it is not the installed one, which is the whole question.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ServiceUnit {
    /// The systemd unit, e.g. `btd.service`.
    pub unit: String,
    pub state: UnitState,
    /// What the running process published about itself, or `None` when it published nothing —
    /// stopped, or a build too old to publish. [`UnitState`] tells those apart.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub identity: Option<Identity>,
}

/// Answer to [`Call::PadStatus`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PadStatusResult {
    /// Every pad the robot is bonded to, connected first.
    pub pads: Vec<Pad>,
    pub driver: UnitState,
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
    /// BlueZ refused the bond. The classic cause on this board is the `Privacy` setting in
    /// `/etc/bluetooth/main.conf` — see `configd::bluez` for which value and why.
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

/// One entry in the skill table, as a client sees it and as it is written back.
///
/// **The same shape read and written**, so a client can list, change one field and send it back
/// without knowing which fields it is allowed to omit. Every optional one means "the default for
/// this", which is what an absent key in `[[policy.skill]]` means too.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct SkillParams {
    /// What `robot.do` answers to. The key of the table: setting an existing name replaces it.
    pub name: String,
    /// The `.onnx`. Absent means the file this robot's own set ships under `<name>.onnx`, which
    /// is what makes overriding a built-in's *timing* possible without naming its file.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub path: Option<String>,
    /// Seconds it runs. Required for a skill this robot does not already have — a policy that
    /// holds until told otherwise has no length of its own and something must choose one.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub duration: Option<f64>,
    /// The twist fed to the network while it runs. Absent is all zeros, which is what a policy
    /// trained on `zero_command_padding` expects and what most one-shots want.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub command: Option<[f64; 3]>,
    /// The twist driven on the way back, for a policy that does not end itself. A manifest's
    /// `command.idle` is where this comes from when there is one.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unwind: Option<[f64; 3]>,
    /// Seconds spent driving `unwind` before handing back to the gait.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unwind_s: Option<f64>,
    /// Whether a request arriving while it runs starts another when this one finishes.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub chain: Option<bool>,
    /// Scales raw output into a joint offset, when this skill wants its own.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub action_scale: Option<f64>,
    /// Servo gain relative to the gait's, when this skill wants its own.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gain_ratio: Option<f64>,
    /// Read-only: whether config has an opinion about this skill, or it is one the release ships.
    /// Ignored on the way in — a client cannot make a built-in an override by saying so.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub overridden: bool,
}

/// Parameters for [`Call::RobotRemoveSkill`].
#[derive(Debug, Clone, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SkillNameParams {
    pub name: String,
}

/// Answer to [`Call::RobotSkills`].
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct SkillsResult {
    /// Every skill this robot has, in the order `robot.do` tries them.
    pub skills: Vec<SkillParams>,
    /// Names `robot.do` answers to that are **not** in the table above, because the daemon drives
    /// them itself — `ground_pick` writes a scripted phase, `sit_toggle` is latched. A client
    /// listing what a robot can do needs both, and neither can be edited here.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub built_in: Vec<String>,
}

/// Parameters for [`Call::PadBind`].
#[derive(Debug, Clone, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PadBindParams {
    /// `"a"`, `"x"`, `"lb"`, `"rb"` or `"dpad_down"`. A string for the reason a slot is one: a
    /// button this build does not have should be refused with the list of ones it does.
    pub button: String,
    /// Three states in one field, the same shape [`LoadPolicyParams`] uses for a path.
    ///
    /// - a name — run that skill;
    /// - `""` — the button does nothing, switched off on purpose;
    /// - absent — put it back to what this robot ships with.
    ///
    /// "Off" and "back to the default" are genuinely different wishes and a client needs both, so
    /// neither can be spelled the way the other is.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub skill: Option<String>,
}

/// Answer to [`Call::PadBindings`].
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct PadBindingsResult {
    /// One entry per bindable button, in the order a listing should show them.
    pub bindings: Vec<PadBinding>,
}

/// What one pad button runs.
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct PadBinding {
    pub button: String,
    /// Empty means switched off, which is not the same as a button this robot cannot bind — those
    /// are simply absent from the list.
    pub skill: String,
    /// Whether this differs from what the robot ships with, so a client can show what somebody
    /// changed without knowing the defaults itself.
    pub overridden: bool,
    /// Set when the bound name is not a skill this robot has: a button that will do nothing when
    /// pressed. Nothing else reports this, and pressing it is otherwise the only way to find out.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}

/// Answer to [`Call::PadForget`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PadForgetResult {
    /// False when no such pad was bonded — not an error, and a client should not present it as
    /// one. Same contract as [`ForgetResult`].
    pub removed: bool,
}

// ── pad input, as the kernel delivers it ─────────────────────────────────────

/// How the cadence of raw pad reports is judged.
///
/// Shared so the live view in `robotctl monitor` and `scripts/pad-link-test.sh` reach the same
/// verdict about the same link: two tools that disagree about what counts as a stall are two tools
/// nobody can compare. The script keeps its own copies of these numbers deliberately — it is a
/// `/bin/sh` file that has to run on a board with none of this compiled — and names them there.
pub mod pad_link {
    /// Under this, a late report is invisible to whoever is driving. Over it, the robot feels
    /// sticky.
    pub const NOTABLE_MS: u64 = 100;

    /// Where `robotd` zeroes the velocity: the default of its `safety.deadman_ms`. A gap past this
    /// is not a latency complaint, it is the robot stopping.
    pub const DEADMAN_MS: u64 = 500;

    /// Past this, silence is the operator rather than the radio.
    ///
    /// An evdev device sends nothing while nothing moves, so a pad at rest is indistinguishable
    /// from a link that has stopped delivering — except by duration. A link silent this long would
    /// have hit its supervision timeout and dropped, and a drop is visible on its own. Counting
    /// quiet as a stall is how the first real measurement of this robot reported a 75-second
    /// breach of the deadman on a link that never faltered.
    pub const IDLE_MS: u64 = 5000;
}

/// The pad's input device, as the kernel describes it.
///
/// Sent when a subscriber attaches and again whenever the device changes, because every number in
/// a [`PadFrame`] is meaningless without it: `ABS_X = 1583` is a stick position only once you know
/// the axis runs −32768..32767 on this pad and 0..65535 on the next one.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PadInputDevice {
    /// As the pad calls itself: "Xbox Wireless Controller".
    pub name: String,
    /// The event node being read, `/dev/input/event5`.
    ///
    /// Worth reporting because it *changes*: a pad that drops and comes back is often a different
    /// number, and one stale path in a log is how two sessions get read as one.
    pub node: String,
    /// The pad's address, as the kernel has it. This is what ties a report stream to a `btmon`
    /// capture of the same link.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub unique: Option<String>,
    /// `0x0005` for Bluetooth, `0x0003` for USB — [`Self::over_bluetooth`].
    pub bus: u16,
    pub vendor: u16,
    pub product: u16,
    /// Every absolute axis the device declares, with the range its values live in.
    pub axes: Vec<PadAxis>,
    /// Every button it declares. The whole list, not the ones this build has a use for: a pad
    /// whose Start does nothing is a pad someone needs to see the raw code of.
    pub buttons: Vec<PadKey>,
}

impl PadInputDevice {
    /// Bus id of a Bluetooth device, from `linux/input.h`.
    pub const BUS_BLUETOOTH: u16 = 0x0005;
    /// Bus id of a USB device.
    pub const BUS_USB: u16 = 0x0003;

    /// Is this pad on the radio at all?
    ///
    /// A pad on a cable has no link to measure, and saying so beats reporting a flawless one —
    /// `pad-link-test.sh` refuses to run in that case for the same reason.
    pub fn over_bluetooth(&self) -> bool {
        self.bus == Self::BUS_BLUETOOTH
    }
}

/// One absolute axis, with the range and the noise floor the driver claims for it.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PadAxis {
    /// The evdev code: `ABS_X` is 0.
    pub code: u16,
    /// `ABS_X`, as `linux/input-event-codes.h` spells it.
    pub name: String,
    pub min: i32,
    pub max: i32,
    /// The driver's own dead zone for this axis, in axis units.
    ///
    /// A *hardware* claim about where centre stops being centre, and worth having next to the
    /// live value: a stick whose rest position sits outside it is a stick that will creep,
    /// whatever `padd`'s own `--deadzone` then does about it.
    pub flat: i32,
    /// Changes smaller than this the driver considers noise.
    pub fuzz: i32,
    /// Where the axis was when the device was described, so a subscriber starts with the whole
    /// picture rather than with whatever moves first.
    pub value: i32,
}

/// One button, and whether it was held when the device was described.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PadKey {
    /// The evdev code: `BTN_SOUTH` is 0x130.
    pub code: u16,
    pub name: String,
    pub pressed: bool,
}

/// What arrives on a [`method::PAD_INPUT`] subscription.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "report", rename_all = "snake_case")]
pub enum PadReport {
    /// A pad's event node is open and reports are coming. Sent on subscribing if a pad is already
    /// connected, and again each time one appears, so a subscriber has one code path for "already
    /// there" and "turned up later".
    Attached { device: Box<PadInputDevice> },
    /// One report, as the kernel framed it.
    Frame(PadFrame),
    /// The node closed: the pad dropped, was switched off, or `padd` stopped reading it.
    Detached {
        /// Why, in `padd`'s words. Not a code — nothing acts on this, and the honest answer is
        /// usually an errno the operator wants verbatim.
        why: String,
    },
}

/// One report from the pad: everything the kernel delivered between two `SYN_REPORT`s.
///
/// **The report, not the event, is the unit of a radio's cadence.** One flick of a stick is four
/// events in a single report, and counting events instead turns one late report into four.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PadFrame {
    /// Reports since this device attached, counted by `padd`.
    ///
    /// A hole in it means the *socket* fell behind — see [`Self::socket_dropped`]. A report the
    /// pad never sent leaves no hole at all, which is the whole difficulty of measuring this:
    /// absence is not an event, and only the clock can find it.
    pub seq: u64,
    /// The kernel's timestamp on the report, microseconds since the epoch.
    ///
    /// **Not the time `padd` read it.** `input_event.time` is stamped as the kernel takes the
    /// report off the transport, so a cadence measured from it is the pad's own and survives
    /// `padd` being scheduled late. It is also what lines this stream up against a `btmon`
    /// capture of the same seconds.
    pub at_us: u64,
    /// Microseconds since the previous report. Absent for the first one after attaching, where
    /// there is nothing to measure from.
    ///
    /// **Signed, because it can genuinely be negative.** `input_event.time` is `CLOCK_REALTIME`,
    /// which is the price of [`Self::at_us`] lining up with a `btmon` capture: a board with no RTC
    /// gets its clock stepped once its first NTP reply lands, and that step falls between two
    /// reports of a link that never faltered. A negative gap is that step, and reading it as a
    /// 40-year stall — or clamping it to zero and calling it the fastest report ever seen — are
    /// both worse than saying so.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub since_us: Option<i64>,
    /// Every event in the report, in the order the kernel delivered them.
    pub events: Vec<PadEvent>,
    /// `SYN_DROPPED` preceded this report: the kernel discarded events because **this reader**
    /// fell behind.
    ///
    /// It says nothing about the radio, and it makes [`Self::since_us`] here meaningless — the
    /// events that would have filled the gap were thrown away before anyone saw them.
    #[serde(default, skip_serializing_if = "not")]
    pub after_drop: bool,
    /// Reports this subscriber missed because its own socket was behind, since the last frame it
    /// did receive.
    ///
    /// A slow client and a stalled radio produce the same silence, and a debug view that cannot
    /// tell them apart is one that invents faults. `padd` never blocks its reader on a subscriber:
    /// it drops, counts, and says so here.
    #[serde(default, skip_serializing_if = "is_zero")]
    pub socket_dropped: u64,
}

/// One evdev event, as it came off the device.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PadEvent {
    /// The evdev type, numerically: `EV_SYN` 0, `EV_KEY` 1, `EV_ABS` 3.
    pub kind: u16,
    pub code: u16,
    pub value: i32,
    /// `ABS_X`, `BTN_SOUTH`, `SYN_REPORT` — the name `linux/input-event-codes.h` gives this
    /// type/code pair, so a captured line reads without a lookup table beside it. Numeric for a
    /// code the kernel headers this was built against do not name.
    pub name: String,
}

impl PadEvent {
    /// `EV_SYN` — bookkeeping: the report boundary, and the dropped marker.
    pub const SYNCHRONIZATION: u16 = 0x00;
    /// `EV_KEY` — a button changed state.
    pub const KEY: u16 = 0x01;
    /// `EV_ABS` — an absolute axis moved.
    pub const ABSOLUTE: u16 = 0x03;

    /// Did an axis move? The value is then a position in that axis's own range — see
    /// [`PadAxis::min`] and [`PadAxis::max`], which is why it cannot be read without them.
    pub fn is_axis(&self) -> bool {
        self.kind == Self::ABSOLUTE
    }

    /// Did a button change? `value` is 0 for released and non-zero for held: 1 on a press and 2 on
    /// an autorepeat, which a pad does not send but a reader must not mistake for a release.
    pub fn is_button(&self) -> bool {
        self.kind == Self::KEY
    }
}

/// Answer to [`Call::PadInput`].
///
/// `accepted` is not "a pad is connected". A subscription with no pad is normal and is half the
/// point: `padd` runs from boot waiting for one, and watching for it to appear is exactly what
/// this stream is for. The pad, when there is one, arrives as [`PadReport::Attached`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PadInputResult {
    pub accepted: bool,
    /// Why not, when it was refused — a platform with no evdev to read.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
}

/// Answer to [`Call::TofStream`].
///
/// Describes the sensor rather than merely accepting, for the same reason
/// [`SubscribeResult`] names the policy: "subscribed" and "there is a sensor" are
/// different facts, and a viewer that cannot tell them apart shows an empty grid
/// for both a robot with no ToF fitted and one whose frames have not arrived yet.
///
/// Accepted with `sensor: None` is the ordinary state of a duck without the
/// sensor: `tofd` runs, the socket answers, and `unavailable` says why there is
/// nothing to show. The daemon keeps retrying, so a sensor fitted later needs no
/// reconnect — a fresh subscription will name it.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct TofStreamResult {
    pub accepted: bool,
    /// The sensor generation that answered, e.g. `VL53L8CX`. `None` when there is
    /// none — see `unavailable`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sensor: Option<String>,
    /// Why there is no sensor: not fitted, wrong generation, bus unreadable.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unavailable: Option<String>,
    /// Frame geometry, so a viewer can lay out before the first frame lands.
    pub rows: u8,
    pub cols: u8,
    /// Ranging rate the sensor was started at, Hz.
    pub hz: u8,
}

/// One 8×8 depth frame — a [`method::TOF_FRAME`] notification.
///
/// **Millimetres and ST's raw status, not metres.** JSON has no NaN, so a
/// distance-only frame would have to encode "no measurement" as a magic number;
/// carrying the status byte instead keeps the sensor's own three-way answer
/// intact — a range, nothing in range, or a measurement that failed. The `tof`
/// crate's `Frame::zone` is the interpretation, and consumers should use it
/// rather than re-deriving the thresholds.
///
/// `distance_mm` and `status` are parallel and row-major, `rows × cols` long.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct TofFrame {
    /// Frames since this `tofd` started, so a consumer can see a gap it did not
    /// cause. Not a wall clock: `tofd` has no business publishing one.
    pub seq: u64,
    /// Microseconds since `tofd` started — the sender's monotonic clock, like
    /// [`PadFrame::at_us`].
    pub at_us: u64,
    pub rows: u8,
    pub cols: u8,
    pub distance_mm: Vec<i16>,
    pub status: Vec<u8>,
}

/// See [`method::ROBOT_CHORALE`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ChoraleParams {
    /// True starts listening for other ducks, false stops and falls silent. Idempotent both ways.
    pub active: bool,
    /// Pin which piece this robot picks when *it* conducts. `None` lets it choose. A follower
    /// sings what the conductor's beacon names regardless — an ensemble where everyone insists
    /// on their own song is not one — so to guarantee a piece, set it on every duck.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub piece: Option<u8>,
}

/// Answer to [`Call::RobotChorale`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct ChoraleResult {
    pub accepted: bool,
    /// Why not — no voice, no radio, or the robot has not opted in.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
}

impl Default for ChoraleResult {
    fn default() -> Self {
        Self {
            accepted: true,
            reason: None,
        }
    }
}

/// What `robotd` wants on the air — a [`method::CHORALE_BEACON`] notification.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct ChoraleAdvertise {
    /// The beacon to broadcast, or `None` to take the advertisement down.
    ///
    /// Down matters as much as up: a second advertising instance halves the rate of the first, and
    /// `btd`'s front-door interval was tuned against measurements — so the beacon exists only while
    /// a chorale does.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub beacon: Option<ChoraleBeacon>,
    /// Whether to be listening for other ducks' beacons at all. Separate from `beacon` because a
    /// duck that has been asked to stop should stop scanning too, and a duck that is only listening
    /// still scans while advertising an idle beacon.
    pub listening: bool,
}

/// A beacon `btd` heard — a [`method::CHORALE_HEARD`] notification.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ChoraleHeard {
    pub beacon: ChoraleBeacon,
    /// Which radio it came from, as `btd` renders an address. An identity for de-duplication only.
    pub from: String,
    /// How long ago `btd` saw it, in microseconds.
    ///
    /// An age rather than a timestamp, and that is the whole reason this is usable for
    /// synchronisation: the two daemons share a machine but not an epoch, and an age survives the
    /// trip down a socket in a way a timestamp from another process's clock does not. `robotd`
    /// subtracts it from its own clock on arrival.
    pub age_us: u64,
}

/// The duck chorale's beacon: what one duck puts on the air so others can sing with it.
///
/// **A wire contract, and the only one that is not JSON.** Every other message in this crate rides
/// a socket; this one rides a BLE advertisement, because a chorale has to work between two robots
/// with no network between them and no clock in common. It lives here for the reason
/// `btd::adv`'s address layout lives in `btd` — one implementation of the layout, so the half that
/// broadcasts and the half that scans cannot disagree about it.
///
/// ## Why these four bytes and no more
///
/// The controller reports a 251-byte advertising budget, so this is small by choice rather than by
/// necessity: **every field is something several ducks have to agree about**, and the fewer of
/// those there are the fewer ways they can disagree. Notably absent is any kind of timestamp —
/// the arrival of a new [`ChoraleBeacon::beat`] *is* the downbeat, which is what lets a chorale
/// work with no shared clock at all.
///
/// Also absent is the robot's voice seed. Casting consumes a duck's pitch centre and nothing else
/// (`sounds::chorale::cast`), so that is what goes out — quantised to a byte — and the seed, which
/// is the robot's identity, stays at home.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ChoraleBeacon {
    /// Which piece is being sung, so a duck arriving late knows what to join. Zero means "willing,
    /// but nothing is being sung" — which is what a duck with `accept_chorale` on advertises while
    /// it waits for company.
    pub piece: u8,
    /// The beat the conductor is on, wrapping. A byte is about four minutes at a chorale tempo;
    /// the follower unwraps it (`sounds::chorale::beat::Follower`).
    pub beat: u8,
    /// The advertiser's own register — its pitch centre, quantised. All casting needs.
    pub register: u8,
    /// Tie-break and identity, derived from the seed rather than being it. Sixteen bits, and
    /// that width is load-bearing: this id is also how a duck recognises its *own* beacon
    /// reflected back, and how every duck merges sightings of the same peer. With one byte, a
    /// four-duck room collided a pair on the first day — the fourth duck rolled the
    /// conductor's byte, everyone merged the two into one duck, and it dropped the conductor's
    /// beacons as its own reflection and could never join.
    pub id: u16,
    /// Who is singing, in seating order — `(register, id)` per duck. Empty from a duck that is only
    /// listening.
    ///
    /// **This is what stops two ducks singing each other's part.** Seating depends on join order
    /// (`sounds::chorale::seat`), so a duck that seats itself from whatever it happened to hear
    /// disagrees with one that heard a different subset — and both then sing alto. The conductor
    /// keeps the roster and broadcasts it; everyone else replays `seat_all` over it. One source of
    /// truth, which is what a conductor is for.
    ///
    /// There is room: the controller reports a 251-byte advertising budget and a full quartet's
    /// roster is eight bytes.
    pub roster: Vec<(u8, u16)>,
}

impl ChoraleBeacon {
    /// Bottom and top of the range [`ChoraleBeacon::register`] covers.
    ///
    /// Brackets the whole duck population — `sounds::Personality` clamps a pitch centre to
    /// 110–620 Hz — with margin at both ends, so neither the lowest nor the highest duck sits on a
    /// clamp. A byte over that span is ~2 Hz a step, a fortieth of a semitone at the bottom, far
    /// finer than casting looks at.
    pub const REGISTER_LOW_HZ: f64 = 100.0;
    pub const REGISTER_HIGH_HZ: f64 = 625.0;

    /// Hertz per step of [`ChoraleBeacon::register`].
    pub const REGISTER_STEP_HZ: f64 = (Self::REGISTER_HIGH_HZ - Self::REGISTER_LOW_HZ) / 255.0;

    /// The piece number that means "willing, but not singing".
    pub const IDLE: u8 = 0;

    /// Marks the payload as a chorale beacon rather than some other four bytes.
    ///
    /// The advertisement's service UUID already says "this is a duck", so this only has to separate
    /// *this* payload from the address field the other advertising instance carries — a scanner
    /// reading four bytes of IPv4 as a beacon would hear a beat in `192.168.1.42`.
    pub const TAG: u8 = 0xC0;

    /// Quantise a pitch centre into [`ChoraleBeacon::register`].
    pub fn quantise_register(pitch_center_hz: f64) -> u8 {
        ((pitch_center_hz - Self::REGISTER_LOW_HZ) / Self::REGISTER_STEP_HZ)
            .round()
            .clamp(0.0, 255.0) as u8
    }

    /// The pitch centre this register stands for.
    pub fn pitch_center_hz(&self) -> f64 {
        Self::REGISTER_LOW_HZ + f64::from(self.register) * Self::REGISTER_STEP_HZ
    }

    /// Whether this beacon says a piece is under way.
    pub fn singing(&self) -> bool {
        self.piece != Self::IDLE
    }

    /// The most ducks a roster carries, and so the most that can sing together.
    ///
    /// Four, because the piece has four parts. A fifth duck in the room keeps listening rather than
    /// joining: `sounds::chorale::seat` can double a part, but nothing seats a fifth duck over the
    /// radio, and a beacon is not the place to describe a choir. If doubling is ever wanted on
    /// hardware, this and the roster guard in `robotd`'s chorale are the two places that cap it.
    pub const MAX_ROSTER: usize = 4;

    /// The manufacturer-data payload: tag, the fixed fields, then the roster length and its
    /// entries. Ids are big-endian u16 — see [`ChoraleBeacon::id`] for why they grew.
    pub fn to_bytes(&self) -> Vec<u8> {
        let roster = &self.roster[..self.roster.len().min(Self::MAX_ROSTER)];
        let mut bytes = vec![Self::TAG, self.piece, self.beat, self.register];
        bytes.extend(self.id.to_be_bytes());
        bytes.push(roster.len() as u8);
        for (register, id) in roster {
            bytes.push(*register);
            bytes.extend(id.to_be_bytes());
        }
        bytes
    }

    /// Read a beacon out of a manufacturer-data payload.
    ///
    /// `None` for anything that is not exactly this layout, length included. The company id these
    /// ride under is `0xFFFF`, the SIG's testing id that anyone may use, so a payload of the wrong
    /// shape is somebody else's advertisement and not a malformed one of ours — which is also why
    /// a *longer* payload is rejected rather than read leniently: a future beacon with more in it
    /// is not this one.
    pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
        let [
            Self::TAG,
            piece,
            beat,
            register,
            id_hi,
            id_lo,
            count,
            rest @ ..,
        ] = bytes
        else {
            return None;
        };
        let count = usize::from(*count);
        if count > Self::MAX_ROSTER || rest.len() != count * 3 {
            return None;
        }
        Some(Self {
            piece: *piece,
            beat: *beat,
            register: *register,
            id: u16::from_be_bytes([*id_hi, *id_lo]),
            roster: rest
                .as_chunks::<3>()
                .0
                .iter()
                .map(|entry| (entry[0], u16::from_be_bytes([entry[1], entry[2]])))
                .collect(),
        })
    }

    /// This duck's index in the roster, if it is in there — which is how it learns its part.
    pub fn seat_of(&self, register: u8, id: u16) -> Option<usize> {
        self.roster
            .iter()
            .position(|(r, i)| *r == register && *i == id)
    }

    /// The roster's registers as pitch centres, for `sounds::chorale::seat_all`.
    pub fn roster_registers(&self) -> Vec<f64> {
        self.roster
            .iter()
            .map(|(register, _)| {
                Self::REGISTER_LOW_HZ + f64::from(*register) * Self::REGISTER_STEP_HZ
            })
            .collect()
    }
}

/// `skip_serializing_if` for a `bool` that is false by default.
fn not(b: &bool) -> bool {
    !*b
}

/// `skip_serializing_if` for a counter that is zero on every healthy frame.
fn is_zero(n: &u64) -> bool {
    *n == 0
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

/// The release a binary at this path was installed as.
///
/// Matches the layout rather than a configured root: any path with a `releases/<version>/` in it
/// yields that version. The root is `updater.toml`'s to choose, and hardcoding a copy of it here is
/// how the two drift apart. Here rather than in one daemon so that `configd`, `updaterd` and
/// `robotctl` cannot come to disagree about what a release path means.
///
/// A path with no such component is not an error — a hand-built binary run from a developer's home
/// directory is a normal thing on a dev board, and the full path is reported alongside.
pub fn release_from_path(path: &str) -> Option<semver::Version> {
    let path = path.strip_suffix(" (deleted)").unwrap_or(path);
    let mut parts = path.split('/');
    while let Some(part) = parts.next() {
        if part == "releases"
            && let Some(version) = parts.next()
            && let Ok(version) = semver::Version::parse(version)
        {
            return Some(version);
        }
    }
    None
}

/// What a daemon says it is, published by the process itself at startup.
///
/// **Self-published, which is the whole design.** The alternative was reading `/proc/<pid>/exe` from
/// outside, and that was chosen for a bad reason: that an *old* daemon could not have learned to
/// publish anything. Designing around that bought a worse answer. A path names a release directory;
/// a process knows its own version, its own git revision and its own exe — and it can read
/// `/proc/self/exe` with no privilege at all, where reading another user's needs root.
///
/// So the two builds of `0.4.0` a dev channel produces are told apart here by `revision`, the field
/// that actually differs, rather than inferred from a directory name that happens to embed it. A
/// daemon too old to publish has no file and reports as `unknown`, which is honest — an inferred
/// release reads as authoritative.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Identity {
    pub service: String,
    /// Crate version, compiled in. Shared across the workspace, so not enough on its own.
    pub version: String,
    /// Git SHA, compiled in from `DUCK_REVISION`. `None` for a build that did not come from CI.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub revision: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub built_at: Option<String>,
    /// The resolved path this process was launched from, via `/proc/self/exe` — so it names the
    /// release directory even though the unit points at the `current` symlink.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub exe: Option<String>,
    pub pid: u32,
}

impl Identity {
    /// The release this process was installed as, when its path names one.
    pub fn release(&self) -> Option<semver::Version> {
        release_from_path(self.exe.as_deref()?)
    }
}

/// Write this process's identity where anything can read it.
///
/// Never fatal and never allowed to stop a daemon starting: a robot that would not come up because
/// it could not describe itself is a worse failure than one that cannot say what it is. The
/// directory is created if missing so a hand-run binary on a dev board still publishes; on the board
/// it is already there, made by `RuntimeDirectory=`.
pub fn publish_identity(service: &str, build: BuildInfo) -> Result<(), String> {
    let identity = Identity {
        service: service.to_owned(),
        version: build.version.to_owned(),
        revision: build.revision.map(str::to_owned),
        built_at: build.built_at.map(str::to_owned),
        exe: std::env::current_exe()
            .ok()
            .map(|path| path.display().to_string()),
        pid: std::process::id(),
    };

    let path = identity_path(service);
    if let Some(parent) = path.parent() {
        let _ = std::fs::create_dir_all(parent);
    }
    let mut json = serde_json::to_vec(&identity).map_err(|e| e.to_string())?;
    json.push(b'\n');
    std::fs::write(&path, json).map_err(|e| format!("{}: {e}", path.display()))
}

/// What `mediad`'s capture path is doing, published for `robotctl` to read.
///
/// A file rather than a query, for the same reason [`Identity`] is one: it needs no socket, no
/// privilege and no protocol version, and it answers just as well when the daemon has stopped
/// (systemd removes the runtime directory with the unit, so absence means "not running").
///
/// `mediad` is not a request/response service — it routes calls upstream rather than answering
/// them — so adding a served call for this would mean giving it a socket it otherwise has no use
/// for. If a WebRTC peer ever needs these numbers, that is the moment to reconsider.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CameraStats {
    /// Measured delivery rate at the pad before the tee, frames per second.
    ///
    /// Measured *there* on purpose: every other place in that pipeline sits behind something that
    /// drops — a leaky queue, the encoder's queue, a `videorate drop-only` — and reporting one of
    /// those as the capture rate is a mistake this project has already made four times.
    pub fps: f64,
    /// What was asked for, so a reader can judge `fps` without knowing the configuration.
    pub target_fps: u32,
    pub width: u32,
    pub height: u32,
    /// GStreamer format name of what the capture path emits, e.g. `UYVY`.
    pub format: String,
    /// Frames delivered since start.
    pub frames: u64,
    /// Frames the *driver* captured and we never saw, counted from gaps in the sequence number
    /// `v4l2src` leaves in each buffer's offset. Distinct from frames dropped downstream by our
    /// own queues, which is a choice rather than a fault.
    pub dropped: u64,
    /// WebRTC peers currently being encoded for. Zero is normal — nothing encodes until someone
    /// connects.
    pub consumers: u32,
}

/// Where `mediad` publishes [`CameraStats`].
fn camera_stats_path() -> std::path::PathBuf {
    std::path::PathBuf::from("/run/mediad/camera.json")
}

/// Publish [`CameraStats`]. Never fatal: a robot that cannot describe its camera still has one.
pub fn publish_camera_stats(stats: &CameraStats) -> Result<(), String> {
    let path = camera_stats_path();
    if let Some(parent) = path.parent() {
        let _ = std::fs::create_dir_all(parent);
    }
    let mut json = serde_json::to_vec(stats).map_err(|e| e.to_string())?;
    json.push(b'\n');
    std::fs::write(&path, json).map_err(|e| format!("{}: {e}", path.display()))
}

/// What `mediad` published about its camera, or `None` — not running, no camera, or too old.
pub fn read_camera_stats() -> Option<CameraStats> {
    serde_json::from_slice(&std::fs::read(camera_stats_path()).ok()?).ok()
}

/// What one daemon published, or `None` if it published nothing.
///
/// `None` covers both "not running" — systemd removes the directory with the unit — and "too old to
/// publish". Those are told apart by the unit's state, a separate question with a separate answer.
pub fn read_identity(service: &str) -> Option<Identity> {
    let json = std::fs::read(identity_path(service)).ok()?;
    serde_json::from_slice(&json).ok()
}

/// The first line a daemon writes: its own identity.
///
/// At `warn` so it survives `RUST_LOG=warn` on a long-running board (`architecture.md` §8.1) —
/// identifying the running build is not a debug-level concern. `exe` earns its place by naming the
/// release directory the process was actually launched from, which is the difference between "the
/// update worked" and "the symlink moved but systemd is still running the old path".
///
/// **Here rather than copied into each daemon, which is where it started.** Four of the five had an
/// identical private copy and `padd` had none, so `padd` was the one daemon whose journal could not
/// answer which build was running — discovered while chasing exactly that question. A shared
/// definition makes the next daemon's omission a missing call rather than a missing idea.
///
/// A macro, not a function, because [`build_info!`] reads `CARGO_PKG_VERSION` and `DUCK_REVISION`
/// through `env!` at the *call site*: a function here would report this crate's version for
/// everyone. It also keeps `tracing` out of this crate's dependencies, which are deliberately serde
/// and semver and nothing else — the expansion happens where `tracing` already is.
#[macro_export]
macro_rules! log_startup_identity {
    ($service:expr) => {{
        if let Err(e) = $crate::publish_identity($service, $crate::build_info!()) {
            // A warning: `robotctl health` will say `unknown` for this daemon, which is a worse
            // report rather than a broken robot.
            tracing::warn!(error = %e, "could not publish this process's identity");
        }
        tracing::warn!(
            service = $service,
            build = %$crate::build_info!(),
            exe = %std::env::current_exe()
                .map(|path| path.display().to_string())
                .unwrap_or_else(|_| "unknown".to_owned()),
            pid = std::process::id(),
            "starting"
        )
    }};
}

#[cfg(test)]
mod tests {
    use super::test_support::every_call;
    use super::*;

    /// The path a release actually installs to, which is what makes "which version is running"
    /// answerable at all.
    #[test]
    fn a_release_path_names_its_version() {
        assert_eq!(
            release_from_path("/opt/robot/daemon/releases/0.4.0/bin/btd"),
            Some(semver::Version::parse("0.4.0").unwrap())
        );
    }

    /// A branch build, which is the case this was written against: the crate version says `0.4.0`
    /// for both the old binary and the new one, and **only the path tells them apart**.
    #[test]
    fn a_dev_release_keeps_the_suffix_that_distinguishes_it() {
        let version = release_from_path("/opt/robot/daemon/releases/0.4.0-dev.271.7610e6e/bin/btd");
        assert_eq!(
            version,
            Some(semver::Version::parse("0.4.0-dev.271.7610e6e").unwrap())
        );
        // And it must compare as older than the release it precedes, or "running an older build
        // than is installed" cannot be detected.
        assert!(version.unwrap() < semver::Version::parse("0.4.0").unwrap());
    }

    /// The sharpest case: the binary is gone from disk and the process is still running it. The
    /// version has to survive the marker the kernel appends, or the one report that proves a restart
    /// did not happen would say "unknown".
    #[test]
    fn a_deleted_binary_still_names_its_release() {
        assert_eq!(
            release_from_path("/opt/robot/daemon/releases/0.3.0/bin/padd (deleted)"),
            Some(semver::Version::parse("0.3.0").unwrap())
        );
    }

    /// A hand-built binary on a dev board is not a release and must not be forced into looking like
    /// one. The full path is reported instead, which is more use than a wrong version.
    #[test]
    fn a_binary_outside_the_layout_has_no_release() {
        assert_eq!(
            release_from_path("/home/pierre/duck/target/debug/btd"),
            None
        );
        // A `releases` component whose child is not a version is not a release either.
        assert_eq!(release_from_path("/srv/releases/nightly/bin/btd"), None);
    }

    /// The whole mechanism, over a real file: a process publishes what it is, and a reader gets it
    /// back — including the release, derived from the exe path rather than stored twice.
    ///
    /// Round-tripped through `serde` in both directions on purpose. The published file is read by a
    /// *different build* than the one that wrote it, every time an update lands, so a field that
    /// could not round-trip would present as "this daemon published nothing".
    #[test]
    fn an_identity_survives_being_published_and_read_back() {
        let dir = tempfile::tempdir().expect("a temp dir");
        // SAFETY: single-threaded test, and nothing else reads the variable concurrently.
        unsafe { std::env::set_var("DUCK_RUNTIME_DIR", dir.path()) };

        publish_identity("btd", build_info!()).expect("publishing");
        let read = read_identity("btd").expect("reading it back");

        assert_eq!(read.service, "btd");
        assert_eq!(read.version, env!("CARGO_PKG_VERSION"));
        assert_eq!(read.pid, std::process::id());
        // The exe is this test binary, which is not in a release directory — so there is no release,
        // and that is the honest answer rather than an invented one.
        assert!(read.exe.is_some(), "{read:?}");
        assert_eq!(read.release(), None);

        // A service that published nothing reads as nothing, which is how an old daemon reports.
        assert_eq!(read_identity("padd"), None);

        unsafe { std::env::remove_var("DUCK_RUNTIME_DIR") };
    }

    /// `every_call` is a hand-written list, so a new variant is silently untested unless
    /// someone remembers to add it. Pin the count: adding a `Call` without extending the
    /// list fails here, which is the only thing standing between a new method and it never
    /// being round-tripped at all.
    /// `destination` answers for every call but the one the transport answers itself.
    ///
    /// The `None` arm is a single deliberate case — `system.authenticate`, whose check belongs to
    /// the transport rather than to any service. A second `None` appearing here would mean a call
    /// no service owns, which is a call nobody can serve.
    #[test]
    fn only_authenticate_has_no_service() {
        for call in every_call() {
            let has = call.destination().is_some();
            let expected = !matches!(call, Call::SystemAuthenticate(_));
            assert_eq!(
                has,
                expected,
                "{} destination() = {:?}",
                call.method(),
                call.destination()
            );
        }
    }

    /// A stream holds its connection forever, so it must never share a lane with a call that
    /// expects an answer.
    ///
    /// Pinned because the failure is silent and specific: `update.subscribe` owns its connection
    /// and never reads another request, so anything queued behind it is written into a socket
    /// nobody reads — it never runs, never replies and never errors.
    #[test]
    fn subscriptions_are_the_only_thing_on_the_stream_lane() {
        for call in every_call() {
            if let Some((_, Lane::Stream)) = call.destination() {
                assert!(
                    matches!(
                        call,
                        Call::Subscribe
                            | Call::RobotSubscribe(_)
                            | Call::PadInput
                            | Call::TofStream
                    ),
                    "{} is on the Stream lane but is not a subscription",
                    call.method()
                );
            }
        }
    }

    #[test]
    fn every_call_covers_every_variant() {
        assert_eq!(
            every_call().len(),
            61,
            "a Call variant was added or removed — update every_call() and this count"
        );
    }

    /// Every method name in the list is distinct.
    ///
    /// Worth stating what the count above can and cannot do, because it gave false comfort once.
    /// It catches a list edited without the count being bumped — and it cannot catch a variant
    /// added to [`Call`] and to neither, which is how `pad.input` came to be missing from
    /// `every_call` while this test passed at 44.
    ///
    /// What actually caught that is a property test in a consumer: `mediad`'s route table asserts
    /// that some permitted call reaches every service, and `pad.input` is the only one that
    /// reaches `padd`. So the real defence is tests that *use* the list for something, and this
    /// pair only guards against the cheaper mistake.
    #[test]
    fn every_call_has_a_distinct_method() {
        let calls = every_call();
        let names: std::collections::BTreeSet<_> = calls.iter().map(|c| c.method()).collect();
        assert_eq!(
            names.len(),
            calls.len(),
            "every_call lists the same method twice, so a variant is standing in for another"
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
                from_dir: None,
            },
        });
        let line = serde_json::to_string(&Request::call(Id::Number(1), &call)).unwrap();

        assert!(line.contains(r#""jsonrpc":"2.0""#), "{line}");
        assert!(line.contains(r#""method":"update.apply""#), "{line}");
        assert!(line.contains(r#""dry_run":true"#), "{line}");
        assert!(
            !line.contains("from_dir"),
            "an apply that names no directory must not mention one: {line}"
        );
    }

    /// `from_dir` survives the wire, and only appears when it was asked for.
    ///
    /// The absence half is the load-bearing one. Every other client of this type — `btd`
    /// relaying the app, the periodic scheduler — leaves it `None`, and an apply that
    /// carried `"from_dir":null` would be a sideload request as far as a reader of the
    /// journal or a future daemon is concerned.
    #[test]
    fn from_dir_round_trips_and_is_omitted_when_unset() {
        let call = Call::Apply(ApplyParams {
            component: ComponentId::new("daemon"),
            target: Target::Latest,
            options: ApplyOptions {
                from_dir: Some("/var/tmp/duck-sideload".to_owned()),
                ..Default::default()
            },
        });

        let line = serde_json::to_string(&Request::call(Id::Number(1), &call)).unwrap();
        assert!(
            line.contains(r#""from_dir":"/var/tmp/duck-sideload""#),
            "{line}"
        );

        let back: Request = serde_json::from_str(&line).unwrap();
        assert_eq!(back.as_call().unwrap(), call);
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

    /// A `params` member this release does not know is refused, and named, for every method that
    /// takes parameters.
    ///
    /// Pinned across all of them rather than left to fourteen `deny_unknown_fields` attributes being
    /// remembered, because this is where the strictness the `hello` handshake used to supply now
    /// lives — see [`API_VERSION`]. A params type added without the attribute fails here.
    ///
    /// Methods taking *no* parameters are exempt by design: `parse` ignores whatever arrived for
    /// them, so that `{}`, `null` and an absent member all mean the same call.
    #[test]
    fn an_unknown_params_member_is_refused_by_name() {
        for call in every_call() {
            let Value::Object(mut params) = call.params() else {
                panic!(
                    "{} serialised its params as something other than an object",
                    call.method()
                );
            };
            if params.is_empty() {
                continue;
            }
            params.insert("from_a_later_release".to_owned(), Value::Bool(true));

            let error = Call::parse(call.method(), Some(&Value::Object(params)))
                .expect_err(&format!("{} accepted an unknown member", call.method()));
            assert_eq!(error.code, code::INVALID_PARAMS, "{}", call.method());
            assert!(
                error.message.contains("from_a_later_release"),
                "{}: {}",
                call.method(),
                error.message
            );
        }
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
                // Powering the machine off is as consequential as rebooting it.
                method::ROBOT_SHUTDOWN,
                // Replacing the policy set is at least as consequential as bonding a pad: it
                // decides what drives fifteen servos. `policy.check` must stay off this list —
                // asking whether a newer gait exists is inspection, and support has to be able
                // to ask it on a robot it may not change.
                method::POLICY_INSTALL,
                // Putting a stranger's network in a slot is at least as consequential as
                // replacing the official set. `policy.check` and `policy.search` stay off this
                // list: asking what exists is inspection, and support has to be able to ask it
                // on a robot it may not change.
                method::POLICY_FETCH,
                // Binding the robot to an account, and unbinding it. On this list for a reason
                // none of the others share: it decides who can reach the robot from outside the
                // building, and it survives every reboot. `account.status` must stay off it —
                // which account a robot thinks it belongs to is the first thing support asks.
                method::ACCOUNT_LOGIN,
                method::ACCOUNT_LOGOUT,
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

    /// The three shapes a pad subscriber has to tell apart travel under one tag, and a frame
    /// keeps every field a cadence is measured from. A `Frame` that deserialised as `Attached`
    /// would present a dropped link as a fresh device.
    #[test]
    fn pad_reports_round_trip_under_their_tag() {
        let attached = PadReport::Attached {
            device: Box::new(PadInputDevice {
                name: "Xbox Wireless Controller".into(),
                node: "/dev/input/event5".into(),
                unique: Some("78:86:2e:bb:13:28".into()),
                bus: PadInputDevice::BUS_BLUETOOTH,
                vendor: 0x045e,
                product: 0x0b13,
                axes: vec![PadAxis {
                    code: 0,
                    name: "ABS_X".into(),
                    min: -32768,
                    max: 32767,
                    flat: 128,
                    fuzz: 16,
                    value: -3,
                }],
                buttons: vec![PadKey {
                    code: 0x130,
                    name: "BTN_SOUTH".into(),
                    pressed: false,
                }],
            }),
        };
        let frame = PadReport::Frame(PadFrame {
            seq: 412,
            at_us: 1_755_500_000_123_456,
            since_us: Some(7_920),
            events: vec![
                PadEvent {
                    kind: 3,
                    code: 0,
                    value: -1583,
                    name: "ABS_X".into(),
                },
                PadEvent {
                    kind: 0,
                    code: 0,
                    value: 0,
                    name: "SYN_REPORT".into(),
                },
            ],
            after_drop: false,
            socket_dropped: 0,
        });
        let detached = PadReport::Detached {
            why: "read failed: No such device (os error 19)".into(),
        };

        for report in [attached, frame, detached] {
            let line = serde_json::to_string(&Request::notify_pad_report(&report)).unwrap();
            assert!(line.contains(r#""method":"pad.report""#), "{line}");
            let back: Request = serde_json::from_str(&line).unwrap();
            assert!(back.is_notification(), "a report answers nothing");
            assert_eq!(back.as_pad_report().unwrap(), report, "{line}");
        }
    }

    /// The two counters that only ever mean bad news stay off a healthy frame, which is most of
    /// them at 125 reports a second — and a reader must still see `false` and `0` rather than a
    /// missing field it has to interpret.
    #[test]
    fn a_clean_frame_carries_neither_alarm() {
        let clean = PadFrame {
            seq: 1,
            at_us: 10,
            since_us: None,
            events: vec![],
            after_drop: false,
            socket_dropped: 0,
        };
        let line = serde_json::to_string(&clean).unwrap();
        assert!(!line.contains("after_drop"), "{line}");
        assert!(!line.contains("socket_dropped"), "{line}");
        assert!(!line.contains("since_us"), "{line}");
        assert_eq!(serde_json::from_str::<PadFrame>(&line).unwrap(), clean);

        let troubled = PadFrame {
            after_drop: true,
            socket_dropped: 3,
            since_us: Some(640_000),
            ..clean
        };
        let line = serde_json::to_string(&troubled).unwrap();
        assert!(line.contains(r#""after_drop":true"#), "{line}");
        assert!(line.contains(r#""socket_dropped":3"#), "{line}");
        assert_eq!(serde_json::from_str::<PadFrame>(&line).unwrap(), troubled);
    }

    /// A pad on a cable is not a link with nothing wrong with it — it is a link that is not
    /// there. `pad-link-test.sh` refuses to measure one, and the live view has to say the same.
    #[test]
    fn a_usb_pad_is_not_on_the_radio() {
        let usb = PadInputDevice {
            name: "Xbox Wireless Controller".into(),
            node: "/dev/input/event5".into(),
            unique: None,
            bus: PadInputDevice::BUS_USB,
            vendor: 0,
            product: 0,
            axes: vec![],
            buttons: vec![],
        };
        assert!(!usb.over_bluetooth());
        assert!(
            PadInputDevice {
                bus: PadInputDevice::BUS_BLUETOOTH,
                ..usb
            }
            .over_bluetooth()
        );
    }

    /// The beacon's round trip, which the broadcasting half and the scanning half both depend on.
    #[test]
    fn a_beacon_survives_the_advertisement() {
        let beacon = ChoraleBeacon {
            piece: 3,
            beat: 217,
            register: 56,
            id: 0x9F,
            roster: vec![(56, 0x9F), (140, 0x02), (200, 0x71)],
        };
        let bytes = beacon.to_bytes();
        assert_eq!(ChoraleBeacon::from_bytes(&bytes), Some(beacon.clone()));
        assert!(beacon.singing());
        // A trio's beacon is a dozen bytes — nowhere near even a legacy advertisement's budget.
        assert!(bytes.len() <= 20, "{} bytes: {bytes:?}", bytes.len());

        // Idle: willing, but nothing under way, and nobody seated. What a duck waiting for company
        // advertises.
        let idle = ChoraleBeacon {
            piece: ChoraleBeacon::IDLE,
            roster: Vec::new(),
            ..beacon.clone()
        };
        assert!(!idle.singing());
        assert_eq!(
            ChoraleBeacon::from_bytes(&idle.to_bytes()),
            Some(idle),
            "an idle beacon still round-trips"
        );

        // A duck finds its own seat in the roster, which is how it learns its part.
        assert_eq!(beacon.seat_of(56, 0x9F), Some(0));
        assert_eq!(beacon.seat_of(200, 0x71), Some(2));
        assert_eq!(
            beacon.seat_of(56, 0x00),
            None,
            "same register, different duck"
        );
        // And the registers come back as pitch centres for the seating fold.
        let registers = beacon.roster_registers();
        assert_eq!(registers.len(), 3);
        assert!(registers[0] < registers[1] && registers[1] < registers[2]);
    }

    /// The roster's length is part of the layout, so a truncated or overlong payload is not a
    /// beacon — a lenient read would seat a quartet from three ducks' worth of bytes.
    #[test]
    fn a_roster_of_the_wrong_length_is_not_a_beacon() {
        let good = ChoraleBeacon {
            piece: 1,
            beat: 0,
            register: 56,
            id: 1,
            roster: vec![(56, 1), (140, 2)],
        };
        let bytes = good.to_bytes();
        assert!(ChoraleBeacon::from_bytes(&bytes).is_some());
        // One byte short, one byte long, and a count that disagrees with what follows.
        assert_eq!(ChoraleBeacon::from_bytes(&bytes[..bytes.len() - 1]), None);
        let mut long = bytes.clone();
        long.push(0);
        assert_eq!(ChoraleBeacon::from_bytes(&long), None);
        let mut lying = bytes.clone();
        // The count byte sits after tag, piece, beat, register and the two id bytes.
        lying[6] = 4;
        assert_eq!(ChoraleBeacon::from_bytes(&lying), None);
        // More ducks than there are parts is not a chorale.
        let crowd = ChoraleBeacon {
            roster: vec![(1, 1), (2, 2), (3, 3), (4, 4), (5, 5), (6, 6)],
            ..good
        };
        let bytes = crowd.to_bytes();
        let back = ChoraleBeacon::from_bytes(&bytes).expect("truncated to four, not refused");
        assert_eq!(back.roster.len(), ChoraleBeacon::MAX_ROSTER);
    }

    /// The tag is what stops a scanner reading the *other* advertising instance's IPv4 address as a
    /// beat. Four bytes of address are a plausible beacon without it.
    #[test]
    fn an_address_payload_is_not_mistaken_for_a_beacon() {
        // 192.168.1.42, as `btd::adv` would broadcast it.
        assert_eq!(ChoraleBeacon::from_bytes(&[192, 168, 1, 42]), None);
        // Nor is anything else of the wrong length, or the right length with the wrong tag.
        for length in [0usize, 1, 4, 6, 16] {
            assert_eq!(
                ChoraleBeacon::from_bytes(&vec![ChoraleBeacon::TAG; length]),
                None,
                "{length} bytes"
            );
        }
        assert_eq!(ChoraleBeacon::from_bytes(&[0x00, 1, 2, 3, 4, 0]), None);
    }

    /// The register byte has to be fine enough to cast from and wide enough for every duck, or two
    /// robots would seat the ensemble differently and sing each other's parts.
    #[test]
    fn a_quantised_register_is_good_enough_to_cast_from() {
        let round_trip = |hz: f64| {
            ChoraleBeacon {
                piece: 1,
                beat: 0,
                register: ChoraleBeacon::quantise_register(hz),
                id: 0,
                roster: Vec::new(),
            }
            .pitch_center_hz()
        };
        for hz in [110.0, 160.0, 214.4, 389.2, 491.5, 519.0, 620.0] {
            let back = round_trip(hz);
            assert!(
                (back - hz).abs() <= ChoraleBeacon::REGISTER_STEP_HZ / 2.0 + 1e-9,
                "{hz} Hz came back as {back}"
            );
        }
        // Ordering survives, which is the only property casting actually asks of it.
        assert!(ChoraleBeacon::quantise_register(214.4) < ChoraleBeacon::quantise_register(389.2));
        // And the whole population fits without sitting on either clamp — a duck at the floor
        // would be indistinguishable from every other duck at the floor.
        assert!(ChoraleBeacon::quantise_register(110.0) > 0);
        assert!(ChoraleBeacon::quantise_register(620.0) < 255);
        assert_eq!(
            ChoraleBeacon::quantise_register(50.0),
            0,
            "clamps, not wraps"
        );
        assert_eq!(ChoraleBeacon::quantise_register(10_000.0), 255);
    }

    /// The theremin block is absent while the instrument is down, and named when it is up:
    /// a v12 client must see byte-for-byte the frame it saw before, and a v13 one must find
    /// the field where the docs say it is.
    #[test]
    fn the_theremin_block_is_absent_until_there_is_a_theremin() {
        let mut state = RobotState {
            t: 1.5,
            movement: MoveState {
                requested: [0.0; 3],
                applied: [0.0; 3],
                limited_by: Vec::new(),
            },
            head: [0.0; 4],
            policy: "stand".into(),
            safety: SafetyState {
                fallen: false,
                limp: false,
                gravity: [0.0, 0.0, -1.0],
                gain: Some(200),
            },
            control_loop: LoopState {
                hz: 50.0,
                missed: 0,
            },
            joints: vec![0.0; 15],
            targets: vec![0.0; 15],
            odom: OdomState::default(),
            theremin: None,
            chorale: None,
        };
        let down = serde_json::to_string(&state).unwrap();
        assert!(!down.contains("theremin"), "{down}");

        state.theremin = Some(ThereminState {
            hand_range_m: Some(0.31),
            note_hz: Some(412.0),
            mouth: 0.64,
            zones: 9,
            held: false,
            sensor: Some("12 usable · 255:40 4*:12 5*:8 1:4".into()),
        });
        let up = serde_json::to_string(&state).unwrap();
        assert!(up.contains(r#""theremin":{"hand_range_m":0.31"#), "{up}");
        assert!(up.contains(r#""note_hz":412.0"#), "{up}");
        // A silent theremin omits the note rather than sending a zero, which would read as
        // "playing 0 Hz".
        state.theremin = Some(ThereminState::default());
        let silent = serde_json::to_string(&state).unwrap();
        assert!(!silent.contains("note_hz"), "{silent}");
        assert!(!silent.contains("hand_range_m"), "{silent}");

        // And it round-trips: a client reads back what the daemon sent.
        let back: RobotState = serde_json::from_str(&up).unwrap();
        assert_eq!(back.theremin.expect("present").note_hz, Some(412.0));
        let back: RobotState = serde_json::from_str(&down).unwrap();
        assert_eq!(back.theremin, None);
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
            odom: OdomState::default(),
            theremin: None,
            chorale: None,
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

    /// A `robot.state` frame from a `robotd` predating odometry has no `odom`
    /// key; a monitor built after it must read that as a robot at the origin,
    /// not a parse error. Literal JSON because a struct cannot express "this
    /// field does not exist".
    #[test]
    fn a_state_frame_missing_odom_still_parses() {
        let state: RobotState = serde_json::from_str(
            r#"{
                "t": 1.0,
                "move": {"requested": [0,0,0], "applied": [0,0,0]},
                "head": [0,0,0,0],
                "policy": "held",
                "safety": {"fallen": false, "limp": false},
                "loop": {"hz": 50.0, "missed": 0},
                "joints": [],
                "targets": []
            }"#,
        )
        .expect("an old frame must parse");
        assert_eq!(state.odom, OdomState::default());
        assert_eq!(state.odom.position, [0.0; 3]);
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

    /// A transcript line is one flat object, because it is read with `cat` at least as often
    /// as with `robotctl`.
    #[test]
    fn a_run_record_is_one_flat_line() {
        let record = RunRecord {
            at: 1_700_000_000,
            event: RunEvent::Phase {
                phase: Phase::Downloading,
                detail: Some("184.2 MB".into()),
            },
        };
        let line = serde_json::to_string(&record).unwrap();
        assert_eq!(
            line,
            r#"{"at":1700000000,"event":"phase","phase":"downloading","detail":"184.2 MB"}"#
        );
        assert_eq!(serde_json::from_str::<RunRecord>(&line).unwrap(), record);
    }

    /// One line from a later release must not cost the reader the rest of the run.
    ///
    /// The pairing is not hypothetical: during a daemon update the `robotctl` on the board and
    /// the `updaterd` that wrote the file can come from different releases, and that window is
    /// exactly when someone is reading a transcript.
    #[test]
    fn an_unknown_run_event_keeps_the_rest_of_the_transcript() {
        let from_the_future = r#"{"at":1700000001,"event":"quantum_realignment","spin":7}"#;
        let record: RunRecord = serde_json::from_str(from_the_future).unwrap();
        assert_eq!(record.at, 1_700_000_001);
        assert_eq!(record.event, RunEvent::Unrecognised);
    }

    /// An `update.log` entry written before transcripts existed still parses, and still means
    /// what it meant.
    #[test]
    fn a_log_entry_without_a_run_still_parses() {
        let old = r#"{"at":1,"component":"daemon","from":"0.1.3","to":"0.1.4","outcome":{"kind":"success"}}"#;
        let entry: LogEntry = serde_json::from_str(old).unwrap();
        assert_eq!(entry.run, None);
        // And it round-trips back to the same line: an absent run must not become `"run":null`
        // in a file an older `updaterd` may still read.
        assert_eq!(serde_json::to_string(&entry).unwrap(), old);
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
