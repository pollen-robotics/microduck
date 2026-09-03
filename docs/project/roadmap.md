# Roadmap

Status: rewritten · Date: 2026-08-26 (first written 2026-08-05) · Owner: pierre

Companion to [`architecture.md`](../design/architecture.md) (what we're building) and
[`updater-design.md`](../design/updater-design.md) (how it ships). This is *order and sequencing*
— it will change; the design docs shouldn't.

## Where we are

| | |
|---|---|
| `updater/` | engine, verification, store, journal, hooks, preflight, GitHub/HF/local sources, IPC server, systemd unit — **done** |
| `duck-control/` | robot model · bus · IMU · `RobotIo` · observations · ONNX policy · safety. A library: no tokio, no sockets, no systemd |
| `duck-ipc-proto/` | wire contract for every `*.` namespace, at API v14 — serde/serde_json/semver only, so nothing on the recovery path pulls the engine's tree |
| `robotd/` | a 50 Hz loop driving walk/stand/roll through the safety layer, intents, health from deadline adherence and policy state. Since M3: kinematics, contact odometry, gaze IK, the voice, the ToF theremin and the chorale, all hung off the same tick ([`robotd-design.md`](../design/robotd-design.md) §4.4–4.5) |
| `padd/` | gamepad → intents, as an ordinary socket client; ships in the release and runs as its own unit from boot, so pairing a pad is the only step |
| `robotctl/` | the operator CLI — `update`, `health`, `version`, `monitor`, `net`, `system`, `robot`, `pad`, `configure`, `quack`, `chorale`, `theremin`, `completions`; depends on `duck-ipc-proto`, not `updater`, so it stays on the recovery path |
| `configd/` | wifi over NetworkManager, robot name and the identity derived from the SoC serial, pairing PIN, reboot, unit reporting. `--fake-net` serves the whole surface off-board |
| `btd/` | BLE transport adapter — framing, the routed subset, the BlueZ backend, a pairing agent. Works on hardware, unencrypted by default — [`app-path-design.md`](../design/app-path-design.md) §5.5 |
| `duckctl/` | the robot from a laptop. BLE today; named for the robot rather than the radio |
| `mediad/` | camera, mic, encode and the WebRTC gateway, plus the console it serves. **Streaming to a browser on the LAN from a Radxa Zero 3W**, hardware H.264 through `mpph264enc`, `control` datachannel alongside |
| `tof/` | `tofd`: the head's 8×8 ToF matrix on its own socket at 15 Hz. A board with no sensor fitted runs it anyway and says so |
| `xtask/` | package · sign · promote — byte-identical promotion verified |
| `.github/` | ci · release · promote · dev — all four run for real; every release since `0.2.0` reached a board through them |
| bootstrap | `updaterd install` + `scripts/install.sh` — a robot installs its first release through the **ordinary engine**, so there is no bootstrap-only code path to drift |
| recovery | `robot-boot-check.timer` + `robot-rescue` + the `golden` symlink ship and are enabled. **Never exercised on a board** ([`boot-recovery-net.md`](../design/boot-recovery-net.md)) |
| tests | **942 passing** on a Mac with nothing excluded, a few more on Linux — including the health gate, the battery and thermal readout and the policy/safety path against a real `robotd` process, and `configd`'s authorisation over real sockets in `board-test.sh` |
| in flight | `maploc` (#127), the NPU duck detector, the chorale election fix (#151); two design PRs with nothing built — the phone app (#107) and the IPC monitor (#52) |
| missing | the app, the SDK, the policy channel, the autonomous brain, and reaching a robot from outside the LAN |

## The first roadmap reached its target

M1 to M4 were one sequence with one destination: a robot that walks, updates itself over the
air, and cannot be bricked doing it. That arrived. Every release since `0.2.0` reached a board
through the machinery M1 and M2 built, and `0.9.x` is a duck that walks, rolls, sees, talks and
sings.

What follows is not a continuation of that queue. The work left is independent tracks with
different risks and no ordering between them, so **the numbers below are identifiers, not a
sequence** — [the order of work](#the-order-of-work) is its own section.

**The founding claim was half right.** *The hard part is productisation, not capability* judged
the difficulty correctly: the update path, the health gate, the recovery net and the BLE surface
all got built, and all hold. It judged the volume wrong. Nearly everything since `0.5` has been
capability — kinematics, odometry, ToF, gaze, the voice, the chorale, the camera — and the
largest single piece of it, the autonomous brain, is still unported.

## Milestones

Each has a test that says "done", because milestones without one drift.

### M1 — Close the loop · **done**

The updater got something real to gate against, and the team got a shared crate boundary:
a `robotd` skeleton whose state is atomics rather than a mutex (a robot whose loop is wedged
must still be able to answer *I am not healthy*), `duck-ipc-proto` extracted so nothing on the
recovery path links the engine's http/tar/crypto tree, a health gate that is a real socket
probe, one source of truth for the `robotd` socket, an identity line every daemon logs at
`warn` before anything can fail, and first-install bootstrap through the ordinary engine.

**Done:** `robotd/tests/updater_gate.rs` gates an update against a real `robotd` process over a
real socket and commits; `robotd --unhealthy` reverts the content behind `current`.

### M2 — Dev channel · **done**

`sudo robotctl update apply daemon --ref my-branch` installs a branch build, and
`scripts/dev-push.sh` installs a laptop build over ssh with no CI run at all. Two properties
make that safe on every push, both enforced away from the workflow: a dev build **cannot become
`latest`** (the version is a semver prerelease and `version_under` refuses to read a dev tag as
a release version), and it **cannot install on a customer robot** (`allow_dev_keys` is false
there, and a trusted key only counts as a dev key if its filename ends `.dev.pub`).

**Done:** verified against the real repository — `dev.yml` published, `--ref main` installed
over the network, and a customer-robot config refused the same build.

**One thing it left open, now decided:** a private repo's release assets are reachable with a
token and a customer robot has none, so while this repository is private robots in the field
cannot download anything. **The decision is to publish it**, which resolves that and changes no
code — the API path the engine uses works for public repos too
([`updater-design.md`](../design/updater-design.md) §6.1).

### M3 — `robotd` for real · **done**, in two slices

`robotd` **replaced** `microduck_runtime` by extracting its control core into `duck-control`
rather than reimplementing it, so parity arrived as a consequence of the extraction instead of
as a race against a moving target. Slice 1 was a real 50 Hz Dynamixel loop holding its pose —
which is what makes `robot.health` mean *the loop is meeting its deadline*. Slice 2 was one
61-D observation builder, the main-plus-standing policy, the intent surface, and a gamepad
client going through it.

**Safety authority landed here rather than in M6**, holding the only write handle to the bus,
so no policy and no client *can* command a motor around it: joint clamp, fall → limp, and an
intent deadman.

**Hardware first, sim after** — and the board produced the one bug the tests could not: `ort`
*panics* rather than erroring on a runtime below its floor, which killed the control thread and
made health blame the wrong thing.

**Done:** all three on a Radxa Zero 3W. It walks driven through the intent API; an update
applied with `robotctl` restarts it cleanly with the gate passing; a release that comes up
unhealthy reverts on its own.

### M4 — Hardware bring-up · **closing**

A measurement milestone, not a feature one: it exists to turn guesses into numbers on a real
Radxa. Nearly all of it was answered as a side effect of shipping — the loop holds 50.0 Hz on a
non-RT kernel (`missed=3` in 15022 ticks), the bus and the `imu_to_dxl` board answer on
`/dev/ttyS2`, thermals have a real reading across every zone, `systemctl restart` in `on_apply`
works against real systemd, and the gate commits and reverts for real.

**The log-retention question was settled by deciding rather than measuring.** `/var/log` is a
zram device on this image, so `Storage=persistent` gets journald a directory that is itself in
memory: it survives a clean `reboot` and loses recent logs on a power cut. That is the intended
arrangement — the durable record is the update history under `/var/lib`, `fsync`ed per entry
([`deploy/README.md`](../../deploy/README.md)). So the original "logs survive a power cut"
criterion is retired: it asks for a property this project decided not to have.

**Done when:** it walks on hardware, an update applied via `robotctl` restarts `robotd` cleanly
with the gate passing, and `journalctl --list-boots` reports the previous boot after a reboot.
The first two are done; the last is one command on a board and is the only thing outstanding.

**Two numbers deliberately left unmeasured**, because nothing is waiting on them: eMMC write
timing and battery under load. They belong to M7, which is the milestone about knowing what a
board is doing.

### M5 — `mediad`, WebRTC, SDK · **in progress**

**Landed, on hardware.** `mediad` ships in the release and runs as its own unit. The camera
reaches a browser on the LAN through the VPU — `mpph264enc` → `webrtcsink`, constrained
baseline — with a `control` datachannel carrying the same JSON-RPC every other transport
speaks, and the console is served by the robot itself so there is a URL and nothing to install
([`remote-webrtc.md`](../design/remote-webrtc.md) §0,
[`webrtc-console.md`](../design/webrtc-console.md)). Two GStreamer plugins had to be built from
source to get there ([`media-bringup.md`](media-bringup.md)), and the camera has since got 3A
through `rkaiq`.

**Three things keep it open.**

**Outside the LAN.** The same design with a rendezvous service and TURN in front (§7),
deliberately built second. [`remote-access-design.md`](../design/remote-access-design.md) now owns
it: a Hugging Face account reached by the OAuth device flow, and a bridge from the rendezvous Space
the mini's fleet already uses to the signalling server already on the robot.

The **account** is built and is the first half — `robotctl account login` prints a code, somebody
approves it at hf.co/oauth/device, and the robot holds a credential it renews itself; the same
three calls work over BLE, which is the only transport a robot with no wifi has, and over a
datachannel. Nothing consumes the credential yet: the **bridge** is next, and it is where the
rendezvous Space's protocol turns out to be HTTP rather than the WebSocket a LAN client speaks.

One thing to fix before a duck ships with this: the token carries every scope Hugging Face grants,
because the first-party device-code client takes no `scope` parameter. Narrowing it to
`openid profile read-repos` is a public OAuth app in the org and one constant.

**The SDK, and a small Python client.** §5.3 designs it as WebSocket plus snapshot: the same
JSON-RPC, no media stack, `get_frame` returning a JPEG, a few dozen lines — and `mediad`'s
session layer was built so that surface reuses it unchanged (`mediad/src/session.rs`). A Python
client over **WebRTC** instead gets live video and the `control` datachannel from one
connection, at the cost of `aiortc`, an ICE negotiation and a signalling round trip for a caller
who only wants to send an intent and read a frame. **The investigation is whether one client
covers both** — WebSocket for control and snapshots, WebRTC only when the caller asks for a
stream — or whether the WebSocket surface alone is what a script wants and live video stays in
the console. Answer that before writing either, because it decides whether the SDK is fifty
lines or a project.

**Privacy, and it is now two items rather than one.** *Consent* — explicit per-session approval
before a stream starts — is a `mediad` session-layer change and is not blocked on anything. The
*visible indicator* needs an LED under software control, which does not exist on this hardware
([`remote-webrtc.md`](../design/remote-webrtc.md) §11); it is a hardware question and it should
be asked of the hardware rather than parked on a software milestone. `architecture.md` §7 is
right that both are cheap now and expensive later, so consent should not wait for the LED.

**Done when:** telepresence works from outside the LAN, and a server-side script can fetch a
frame and send an intent in a few dozen lines.

### M6 — Ship readiness

What a stranger needs. Preorders are open, so this is ordered by **lead time** rather than by
difficulty — the items with the least code have the longest lead.

- **The pairing PIN.** BLE pairing security rests on a per-robot PIN; the factory default is
  `000000` and public in this repository, so out of the box pairing proves physical presence and
  nothing more. Something has to generate one, print it, and record what was printed — a factory
  process, not a patch. It cannot come from the identity, which is published in an
  advertisement.
- **Calibration**, the other half of provisioning. Identity no longer waits on it: a robot
  derives one from its own SoC serial and names itself `duck-c51b`.
- **The recovery net, exercised on a board.** It is built and shipped and has never run against
  a release whose daemons cannot start, which is the only path that matters. Cheapest item here:
  one board, one deliberately broken release.
- **Consent**, from M5.
- **Manifest staleness reporting** (§8.4.2).
- **Authority arbitration**, finished — including the edge #52 surfaced: `Call::is_mutating()`
  does not cover `robot.enable`, so the call that starts a policy running on a walking robot is
  classified alongside `hello`.
- **The app.** #107 designs it and builds nothing. The blocker is a phone spike — scan, connect,
  `hello`, authenticate, `system.info` with `--require-pairing` on, on a real iPhone and a real
  Android — because §5.5 is currently a fact about CoreBluetooth on a laptop.

One item left M6 by being decided: where a customer robot downloads from. The decision is to
publish this repository, so a shipped robot reaches its releases with no token and no second
host. The follow-up it leaves is a budget rather than a blocker — anonymous GitHub API requests
are capped at 60 an hour per IP, which one duck is nowhere near and a room of twenty on one wifi
is not (§6.1).

**Done when:** a non-developer updates the robot from the phone, and a deliberately bricked
release recovers without a laptop.

### M7 — Knowing what a board is doing

A duck in someone else's hands will develop a fault, and today the answer to *which part* is a
developer reading a journal over ssh. There is no shape for this yet; it is an open
investigation, and the questions are what it owns.

**What exists already** is more than it looks: `robotctl health` reports the bus, the IMU, the
battery, per-motor temperatures and the SoC; `robotctl monitor` draws the loop, the pad stream,
the ToF matrix, the 3D robot and a power row; `scripts/pad-stack-report.sh` is a precedent for
collecting one subsystem's whole story into something a person can paste; `board-test.sh` runs
60 checks, but on emulated aarch64 in CI rather than on the board in front of you.

**What is missing, as questions:**

- **A support bundle.** One command, after a fault, producing a file someone can send. Nothing
  collects health, versions, the journal, the update history and the unit states together.
  `robotctl update show` is the shape of it for one subsystem — a durable per-run record with
  the journal for that window spliced in — and covers updates only.
- **A hardware pass/fail a non-developer can run.** Bus scan and a per-servo answer, IMU sanity,
  ToF present, camera present, NPU present, mic and speaker, battery under load. Closer to
  `board-test.sh` in spirit, aimed at the hardware rather than the release.
- **History, because intermittent faults are invisible in a snapshot.** Bus read failures,
  missed ticks, servo temperature peaks, brownouts. The journal does not survive a power cut by
  design (M4), so anything that matters here needs the update history's durability rather than
  the journal's.
- **The overlap with #52.** The IPC monitor design is the software half of this same question,
  and its crash record — the last events flushed to a fixed-size file under `/var/lib`, outside
  release dirs so it survives update and rollback — is exactly the durability property a
  hardware fault report needs. The two should be decided together rather than growing two
  answers.

M4's two unmeasured numbers land here: eMMC write timing and battery under load.

**Done when:** someone who is not a developer can run one command after a fault and produce
something that names the part at fault, or says the hardware is fine.

### M8 — The policy channel: policies from the Hub

Every policy used to ship **inside the daemon artifact**, so a new gait needed a daemon release
and a policy trained on a laptop reached a duck only through CI or a sideload of the whole daemon.
The point of this milestone is that a policy trained in `microduck_rl` — the training repository,
which is private — can be published, installed and tried on its own version line, and that
someone can try one they did not train and get back.

**Designed**, in [`policy-channel-design.md`](../design/policy-channel-design.md), which owns
the decisions this section used to leave open. The short version: a slot is filled from one of
three origins — official (`pollen-robotics/*`, the reset target), community (any other
Hub repo, unsigned, reported but never auto-applied) or local (a path on the board); `policy
load` writes the config key and `policy reset` removes it, so persistence and undo are the
mechanism that already exists; and the official set ships as one `policies` component rather
than one per slot.

**Most of the engine is already built:** the `hf_hub` source resolves and verifies, a component
brings its own version line, rollback, pin, boot trial and known-bad history, `on_apply =
reload` exists, and `xtask sign` signs any directory of artifacts.

**Three slices, each useful alone:**

1. **The local loop, no network.** `robot.loadPolicy` plus the home-pose reload, and `robotctl
   policy list` / `load` / `reset`. Delivers "try a policy without editing the toml" end to end,
   touches no Hub and no publishing decisions. `API_VERSION` 17 → 18.
2. **Official policies leave the artifact.** *Done.* `robotd` reads
   `/opt/robot/policies/current`, which `scripts/seed-policies.sh` fills by downloading the
   pinned set from `pollen-robotics/microduck-policies` — the arrangement `setup-board.sh`
   already uses for ONNX Runtime. `policies/` and its three `--include` lists are gone, and
   bumping `[workspace.metadata.policies]` is now the whole of shipping a gait.
3. **Moving past the pin.** *Done.* `policy.*` on `updaterd`, `robot.reloadPolicies`, and
   `robotctl policy check` / `update` — a retrained gait now reaches a board with no daemon
   release, which was the milestone's whole point. Each set records the repo it came from, so
   nothing configures it twice.
4. **The community library.** *Done.* `policy load <slot> <org/repo>` fetches one policy from
   any Hub repo into `/var/lib/robot/policies/<org>/<name>/<rev>/`, `origin` reports `community`
   for a stranger's, and `policy search` lists what is out there. The repos published so far
   already carry a `manifest.json` with `obs_len`, `action_len` and `model_api`, so a policy that
   cannot run here is refused before it is downloaded — which is also where `model_api` stopped
   being designed-and-unimplemented.

`reset` means "remove the override and re-resolve" in all three, so slice 1 ships a correct
reset against the in-release copies and slice 2 changes what it resolves *to* without touching
the command.

**What is genuinely missing** is at the two ends, not in the middle:

- **`robotd` cannot reload on demand.** The machinery exists — `robot.setMode` already rebuilds
  every ONNX session at the home pose — but there is no per-slot trigger, and a failed load must
  keep the running controller rather than leaving the robot gaitless.
- **Nothing publishes a bundle.** `xtask package --channel` is close; it checks `--version`
  against the crate version, which a policy set does not have, and the HF repo layout does not
  exist.
- **`model_api`** is designed and unimplemented on both sides. `robot.modelApi` answers, and
  nothing consults it.
- **The training loop.** `microduck_rl` exports ONNX; nothing carries the result to a board.
  Slice 1 is what makes "train it and try it" a minute rather than a CI run.

**Done when:** a policy trained in `microduck_rl` is published to the Hub, installed on a duck,
tried, and undone with `robotctl policy reset` — with the control loop never dropping motor
control through any of it — and someone who did not train it can find it from the robot and try
it.

### M9 — The autonomous brain

The biggest untracked gap: the runtime's `autonomous.rs` exists nowhere in the daemon and no
design doc owns it. [`ideas/autonomous_behavior.md`](../ideas/autonomous_behavior.md) is the
holding pen — a 16-state machine on an energy/mood model, novelty-grid exploration, ToF
avoidance, startle, sound reactions, ball play, a nap cycle, petting.

Every input it needs now exists, and some it never had: ambient sound events, depth frames,
classified trunk-frame obstacle points, voice tags, nearby ducks by stable id, a shared beat,
RSSI as coarse distance, a live synth voice, hand distance from the ToF.

**The shape matters more than the states.** Presence, mood and the beat are *inputs to one
brain*, not modes beside it. The chorale and the theremin grew as explicit modes because there
was no brain to hang them on — the chorale is 55 KB of `robotd` — and that is the pattern this
milestone exists to stop repeating. It gets a design doc before it gets code.

**Done when:** a duck left alone in a room does something worth watching for ten minutes, and
the chorale is something it decides rather than something a command starts.

## The order of work

The numbers above are identifiers. This is the order.

1. **M8, the policy channel.** The next feature. Nothing else is blocked by it, and it unblocks
   the loop that produces the robot's actual behaviour: train, publish, install, try, roll back.
2. **M5's transport investigation.** Cheap, and its answer decides whether the Python client is
   fifty lines or a project.
3. **M6, as shipping approaches**, by lead time: where a customer robot downloads from, then the
   PIN, then the recovery net's hardware test. Consent can land any time and should land early.
4. **M7** — an investigation with no date. It earns priority the first time a duck in someone
   else's hands develops a fault nobody can name, and the cheapest way to be ready is to decide
   it alongside #52 rather than after it.
5. **M9, the brain** — later, deliberately. It is the largest piece of work left and the one
   most likely to grow while being built.

## Decisions that shape work rather than follow it

1. ~~**Signing key custody**~~ — **done** for the daemon. Three encrypted release keys plus an
   unencrypted dev key in `~/.duck-keys`; only `release-1` goes into secrets. Releases are signed
   in CI under `environment: release`, which scopes the secrets but **gates nothing** — no
   required reviewers, no branch policy. Accepted deliberately while no robot is in the field,
   and the declaration is the hook that turns a real gate on with one settings change. See
   [`ci-setup.md`](ci-setup.md). **Reopens with M8**: publishing a policy is a third kind of key.
2. **Safety authority** (§6) — landed in M3.
3. **Provisioning** — identity is done; calibration and the PIN are not, and the PIN is a factory
   process rather than code (M6).
4. **Privacy** — consent is M5 and unblocked; the indicator is a hardware question and should be
   asked as one.
5. ~~**Where a customer robot downloads from**~~ — **decided 2026-08-26:** publish this
   repository, and a robot downloads from it directly. §6.1 keeps the other options and the
   reasoning, because they are the fallback if the source ever has to close again.
6. **Whether policies leave the daemon artifact** — **leaning yes**, not settled. They leave, a
   missing model reports `degraded` rather than unhealthy so the gate commits, and provisioning
   installs the bootstrap set. M8 says what the alternative buys if this turns out fragile.
7. **Curated models or an open set** — open, and it is a trust decision rather than a plumbing
   one (M8).

## Not doing, on purpose

Recorded so they stay decided: A/B image updates, OS/kernel OTA, fleet dashboards/telemetry,
delta updates, staged rollouts, hardware capability matrix, competing model alternatives per
slot (§17), peripheral firmware OTA (§11.1).
