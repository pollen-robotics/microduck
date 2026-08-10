# Robot Daemon — Overall Architecture

Status: draft · Date: 2026-07-22 · Owner: pierre

Sequencing and milestones live in [`roadmap.md`](../project/roadmap.md).

Companion to [`updater-design.md`](updater-design.md), which covers the update
system in detail. This document covers the service split, how services talk to
each other, where state lives, and how the robot is controlled — locally, from
the app, and remotely.

Scope note: this describes where we're going for the **first shipped version**,
not the current prototype (`microduck_runtime`, which is exploratory and will be
rewritten). v1 targets a **single, well-specified hardware configuration**.

## 1. Services

`systemd` is the supervisor: lifecycle, restart-on-crash, ordering, watchdog.

| Service | Owns | Notes |
|---|---|---|
| `robotd` | motor control, kinematics, gait policies, sensor loop, safety | RT-ish core; authoritative on anything that can hurt the robot |
| `mediad` | camera/mic, encode, perception, WebRTC + remote gateway | Heaviest service; also the remote API front door (§5.2) |
| `btd` | BLE GATT server | **Transport adapter only** — owns no state (§4.1). See [`app-path-design.md`](app-path-design.md) |
| `configd` | wifi, robot identity, power, gamepad pairing | Config must be reachable when `robotd` is dead (§3.1), and `btd` must own nothing (§4.1) — so it is neither's business but its own. Gamepad pairing is here rather than in `padd` because bonding a device needs root and BlueZ, and `padd` is deliberately an unprivileged client (§4.1) |
| `updaterd` | update engine | See `updater-design.md` |

Splitting `mediad` from `robotd` is deliberate: a media/perception crash must not
take out motor control.

### 1.1 Invariants

1. **`btd`, `configd` and `updaterd` survive a dead `robotd`.** They are the recovery
   path; they must work in precisely the situation where something is broken. No
   systemd dependency on `robotd`, all IPC optional and timeout-bounded, minimal
   dependency surface (no ML runtime, no media stack). Detailed in
   `updater-design.md` §4.1.

   `configd` is here for a concrete reason rather than symmetry: provisioning wifi is
   exactly what someone needs when the robot is broken, so putting config in `robotd`
   would make it unreachable in the one case that matters.
2. **`robotd` is authoritative on safety.** No remote or local client can bypass
   fall detection, joint/thermal limits, or safe-pose logic. Clients send
   *intents*; `robotd` decides what is executable.
3. **`robotd`'s control loop never blocks on another service.** All cross-service
   reads are last-value-wins caches, never synchronous RPC (§2.4).
4. **Single writer per piece of state.** Every value has exactly one owning
   service; everyone else reads or subscribes.

## 2. Inter-service communication

### 2.1 Control plane vs data plane

Two kinds of traffic with different requirements. Conflating them is the classic
mistake here.

| | Control plane | Data plane |
|---|---|---|
| Content | commands, config, status, perception events | video/audio frames |
| Size/rate | tens of bytes, ≤100 Hz | ~27 MB/s for 640×480 RGB @30 fps |
| Mechanism | unix socket RPC | **never crosses a socket** |

### 2.2 Control plane: JSON-RPC 2.0 over unix sockets

- Each service owns **one unix socket**. Clients connect directly. With N=4 there
  is no case for a broker — a bus is another component that can fail, and it
  fights invariant (1).
- **Wire format: JSON-RPC 2.0, one object per line (NDJSON).** A standard
  protocol rather than a bespoke one: standard request/response correlation,
  standard error objects, and standard **notifications** — which are exactly the
  right shape for pushing progress and event streams. Framing is
  `tokio_util::codec::LinesCodec`; message types are plain `serde` structs.
- **Async with timeouts everywhere, no exceptions.** Any peer may be dead. A
  closed or silent socket is a normal, expected answer.
- Subscriptions are a stream of notifications on an open connection.

**Alternatives measured** (unique dependencies, ARM-Linux target):

| Option | Deps | Why not |
|---|---|---|
| **JSON-RPC/NDJSON + tokio** | **30** | chosen |
| `jsonrpsee-types` only (our transport) | 36 | reasonable; declined — trades frozen-spec code for a `0.x` dependency |
| `varlink` | 24 | close in spirit; less familiar, little gained over JSON-RPC |
| `zbus` (D-Bus, p2p+blocking) | 66 | see below |
| `axum` over UDS | 66 | viable; see "HTTP/WS" below |
| `tarpc` | 71 | ergonomic Rust↔Rust, but not human-readable and server-push is awkward |
| `tonic` (gRPC) | 81 | `.proto` + codegen overhead for a handful of methods |
| `jsonrpsee-server` | 112 | **cannot serve a unix socket** — HTTP/WS transports only; `jsonrpsee-ipc` was never implemented |

Dependency counts are recorded for reference, not as the deciding factor.

**Why a unix socket rather than localhost HTTP/WS.** Functionally near-identical;
the differences are in access control and failure modes:

- **Filesystem permissions are free authorization.** A socket with mode 0660 and a
  dedicated group is reachable only by allowed processes. A TCP port is reachable
  by every process and user on the box, so an auth layer would have to be built to
  get back to parity.
- **`SO_PEERCRED`** yields the caller's uid/gid/pid, used for *both* the audit log
  ("who triggered this rollback" is the first thing support asks) and **enforcement**.
  Two layers, because they answer different questions:
  - the socket's group (mode 0660) decides who may **talk** to the daemon;
  - `allow_uids`/`allow_gids` decide who may **change the robot** — mutating calls
    only. The uid the daemon runs as is always permitted (it could replace the daemon
    regardless); everyone else needs listing, and an unknown peer is denied.
  Read-only calls are deliberately ungated: support must be able to inspect a robot it
  is not authorised to change. Group membership alone saying "may replace the
  firmware" is too coarse for a device where a BLE-facing service is a client.
- **The wrong-interface bug class stops existing.** Binding `0.0.0.0` by typo, by
  config, or by a "make it work from my laptop" patch would expose *firmware
  update control* to the network. Over a unix socket that mistake is
  unrepresentable. Weighted most heavily — not today's threat model, but the
  failure mode.
- Relevant to the planned **SDK**: if third-party or user code ever runs on the
  board, a localhost port is open to it; a group-owned socket is not.

**Why not HTTP/WS as the control plane** (independent of transport — `axum` serves
UDS fine): the protocol needs server→client push for progress. Over HTTP that
means POST for calls *plus* WebSocket/SSE for notifications — two mechanisms, and
`curl` cannot consume the streaming half, so the debuggability win only covers
request/response. Doing everything over a WebSocket restores one mechanism but
adds a handshake to arrive at framed JSON and loses `curl` again. On a persistent
NDJSON connection, calls and notifications are one mechanism with no handshake:
**fewer concepts, which is the actual goal.**

**Future option, if `curl`-ability is wanted for diagnosis:** a small *read-only*
HTTP endpoint on the same unix socket (`GET /status`, `GET /log`), giving
`curl --unix-socket … http://x/status` without moving control operations onto HTTP
or duplicating the streaming path. Diagnostics and control have different
requirements; splitting them serves both.

**Why not D-Bus:** BlueZ (via `bluer`) already pulls it in, so it's on the box
regardless, and `zbus` supports bus-less peer-to-peer. But the same message types
must also travel over **BLE and WebRTC/WebSocket** (§4.1, §5.2), where plain
serde structs work and D-Bus types don't. One definition, several transports is
the goal, and JSON is what makes it free. We use D-Bus only where the OS requires
it (BlueZ, NetworkManager).

**Revisit trigger:** if `btd` ends up deeply invested in D-Bus for BlueZ anyway,
exposing the update interface over `zbus` too would be nearly free *for `btd`*
and would buy `busctl` introspection for debugging. Cheap to change — the types
stay, only framing moves.

### 2.3 Async where it waits, sync where it computes

Async (tokio) is used where a service genuinely waits: serving IPC while a long
operation runs, timing out a peer query or a subprocess, cancelling in-flight work.

CPU-bound and long-running filesystem work stays **synchronous**, and the async
caller hands it to `spawn_blocking`. In the updater that is specifically: SHA-256
over the artifact, minisign stream verification, `zstd`+`tar` extraction, and
recursive deletes of extracted trees. On a Pi these run for seconds; left on an
async worker they would stall the IPC tasks that are supposed to keep answering
`status`/`subscribe` during an update.

Genuinely fast filesystem operations — a symlink `rename`, an fsync, a small
append — are called directly. Dispatching those to a thread pool would cost more
than it saves.

### 2.4 Data plane: features, not frames

`robotd` does not need camera frames — it needs *derived features* ("ball at
(x,y)", "person detected", "loud sound"). Tens of bytes at 10–30 Hz, trivial over
a socket.

**Principle: put perception next to the sensor.** `mediad` owns the camera, runs
inference, and publishes features. Shipping frames to `robotd` so it can run its
own vision would waste most of the board's memory bandwidth.

**In the control loop:** `robotd` subscribes once and reads a locally cached
*latest* snapshot — non-blocking, last-value-wins. A stalled `mediad` then
degrades perception rather than adding jitter to motor control.

**If frames ever must cross a process boundary** (last resort): shared memory
(shm/dmabuf ring buffer) with the socket carrying only "frame N ready at offset
X". libcamera provides dmabuf, so this can be zero-copy. Prefer
features-not-frames and avoid needing this.

## 3. State ownership

Three distinct classes; see `updater-design.md` §5.7 for the update/rollback
implications.

| State | Owner | Mechanism |
|---|---|---|
| Wifi credentials | **NetworkManager** | We never store them. `configd` drives NM over D-Bus; NM persists profiles root-only and reconnects on its own. |
| Robot identity, user prefs, tunables | **config store** (§3.1) | File + `flock` + `rename(2)`, owned by `configd` |
| Calibration, learned state, generated per-device assets | owning service | Outside release dirs; survives update *and* rollback |
| Shipped defaults, binaries, policy bundles | update system | Under `releases/<ver>/`, swapped atomically |

Letting NetworkManager own wifi credentials is less code, better security, and
one less thing to migrate.

**A board does not arrive with NetworkManager**, which this row originally assumed. Armbian's
headless image runs netplan + `systemd-networkd` + `wpa_supplicant`, and netplan is a config
*generator*: it has no scan API, and `netplan apply` reports "config applied" rather than whether
association succeeded. Those are the two things a phone provisioning a robot needs most — "show
me the networks" and "that password was wrong" — so the decision stands and
`scripts/migrate-network.sh` moves a board onto NM once. The reasoning, and what was measured on
the board, is in [`app-path-design.md`](app-path-design.md) §2.

### 3.1 Config store

A plain file plus a small shared crate — **deliberately not a service**:

- `flock` for write serialization; write-to-temp + `rename(2)` for atomicity.
- `inotify` for change notification.
- No single point of failure, readable when any service is down, and the updater
  never touches it (`updater-design.md` §5.7).

Implemented in `configd/src/store.rs`, holding the robot's name and its Bluetooth pairing PIN.
`inotify` is **not** there yet, deliberately: it earns its place when a *second* process reads the
file, and today `configd` is the only one. Watching a file you are the sole writer of is ceremony.

Config **must** be reachable when `robotd` is dead — wifi provisioning is exactly
what a client needs when things are broken — which is why it cannot live in
`robotd`.

**Config is state, not actions.** "Connect to this wifi", "restart", "apply
update", "select model" are actions, dispatched as RPC to the owning service.

## 4. The robot API

### 4.1 One definition, many transports

`btd` **owns nothing**. BLE is one front door among several. If config or
provisioning lived in `btd`, other services would depend on it and an SDK would
absurdly have to go through BLE.

```
        ┌──────── one API definition (shared crate: types + operations)
        │
   ┌────┴─────┬────────────┬──────────────┬────────────────┐
  BLE       unix socket   WebSocket     WebRTC datachannel
 (btd)      robotctl,     server-side   telepresence,
  subset    on-robot SDK  agents/LLM    full fidelity
```

Each transport is a thin adapter over the same API. BLE exposes a **subset**
(provisioning, status, update trigger/progress) — it's too slow and too
constrained for the full surface, and payloads never traverse it.

### 4.2 Cross-cutting rules

- **Per-transport authorization.** BLE implies physical presence + pairing; a
  network transport requires a token. Same API, different authz — decide where
  the check lives from the start.
- **API version handshake.** SDK and daemon versions *will* skew. One integer,
  refuse with a clear message on mismatch (same approach as `model_api`).
- **Intents, not motor writes.** See §6.

## 5. Remote access

### 5.1 Requirement

All robot and media data must be reachable over a **WebRTC** connection, to
support (a) telepresence and (b) a server-side program (e.g. an LLM) that
observes and controls the robot. The latter must be *easy*.

### 5.2 WebRTC session

One PeerConnection carries everything:

```
peer (browser / phone / server)
   ├── video track(s)   ── camera
   ├── audio track(s)   ── mic + speaker (two-way for telepresence)
   ├── datachannel "control"   reliable, ordered      → the robot API (§4)
   └── datachannel "teleop"    unreliable, unordered  → input + high-rate telemetry
```

Two data channels, mirroring §2.1: teleop input and telemetry go **unreliable**
(`maxRetransmits: 0`) because a retransmitted 80 ms-old joystick command is worse
than useless — always take the newest.

**`mediad` owns the PeerConnection.** A PC cannot be split across processes
(tracks and data channels share one DTLS/SCTP association), and it needs the
encoded media, so it lives with `mediad`, which proxies `control` messages to the
owning services over their unix sockets. `mediad` is therefore the **remote
gateway**.

The isolation cost is acceptable: a telepresence session is worthless without
media, so co-locating loses nothing a split would preserve. Local recovery stays
independent via BLE / `robotctl` (invariant 1).

### 5.3 Server-side agents: don't force them through WebRTC

For an LLM-driven controller, WebRTC is the *harder* path. An agent doesn't want a
30 fps H.264 track to decode — it wants a frame every second or two plus a state
blob. Requiring ICE/DTLS/SDP and a decode pipeline first is a poor trade.

| Consumer | Transport | Media |
|---|---|---|
| Telepresence (human) | WebRTC | tracks, low latency |
| Server-side agent / LLM | **WebSocket** | `get_frame` → JPEG on demand, or 1–2 fps push |
| On-robot SDK, `robotctl` | unix socket | snapshot API |
| App | BLE + WebRTC | as needed |

Same API behind all of them. "Run an LLM on a server that controls the robot"
becomes: open a WebSocket, poll a frame, send intents — a few dozen lines, no
media stack. That is what makes it genuinely easy.

Note also that LLM latency (hundreds of ms to seconds) means the agent is a
**high-level** controller: "go to the kitchen", "look at the person". Reactive
control stays local in `robotd`. This is the correct split regardless of
transport.

### 5.4 Infrastructure reality check

"The robot has its own wifi" is **not** the same as "reachable from the
internet". Remote WebRTC needs:

- a **signaling** path (SDP/ICE exchange),
- **STUN**, and **TURN** as a relay fallback for symmetric NAT — real
  infrastructure with real bandwidth cost.

This contradicts the "zero backend" premise of the update design. **LAN-only
telepresence avoids it entirely; internet telepresence does not.** Open question
(§9).

### 5.5 Implementation note

Library choice hinges on hardware encode: GStreamer `webrtcbin` is more pragmatic
if we want V4L2 M2M hardware H.264; `webrtc-rs` is pure Rust and lighter to
reason about but leaves us to build the pipeline. This shapes `mediad`
substantially — decide early.

Latency budget for telepresence driving: aim **<200 ms glass-to-glass**. Implies
low-latency encoder settings, no B-frames, intra-refresh rather than large
keyframes.

## 6. Safety and authority

Remote control of a walking robot over a lossy link. These are much cheaper
designed in than retrofitted.

- **Deadman / heartbeat.** If commands stop arriving or RTT spikes past a
  threshold, `robotd` stops the robot on its own. Non-negotiable: networks
  partition, LLMs stall mid-inference, laptops sleep.
- **Intents, not motor writes.** Remote clients send velocity, gaze target,
  "sit" — never raw joint commands. `robotd` remains authoritative on fall
  detection, joint/thermal limits, and safe poses. A confused agent must not be
  able to command something the robot will execute into a wall.
- **Explicit authority arbitration.** Physical controller, app, remote peer, and
  the autonomous behaviour layer all want control. Defined priority and handoff,
  not last-writer-wins. Local/physical should be able to preempt remote.
- **Session limits.** v1: one media session at a time, plus M control-only
  clients. Multi-peer video (simulcast, encode-once-send-many) is deferred.

## 7. Privacy

It is a camera and microphone in someone's home.

- **Explicit consent** to start a remote session (per-session, or a clear
  persistent opt-in the user can revoke).
- **Visible on-robot indicator** whenever streaming is active.
- DTLS-SRTP keeps media encrypted end-to-end **even through a TURN relay** —
  worth stating plainly to clients.
- BLE provisioning writes carry wifi credentials: that characteristic must be
  paired + encrypted.

## 8. Observability: logs and versions

Cross-cutting, because a robot in someone's home cannot be debugged by attaching a
debugger. What support can ask for has to already be on the robot.

Deployment specifics — the journald drop-in, install steps, verification commands — live in
[`../deploy/README.md`](../../deploy/README.md). This section is the contract every service
must satisfy.

### 8.1 Every service logs to stderr

`tracing` → stderr → journald, level via `RUST_LOG` (`info` in the shipped units). No
service writes its own log files: one mechanism, one retention policy, one place to look.

**The first line each daemon writes is its own identity**, at `warn` so it survives
`RUST_LOG=warn` on a long-running board:

```
WARN starting service="robotd" build=0.2.0 (rev a1b2c3d, built 2026-07-28T13:50:00Z)
     exe=/opt/robot/daemon/releases/0.2.0/bin/robotd pid=814
```

`exe` earns its place: it says which release directory the process was actually launched
from, which is the difference between "the update worked" and "the symlink moved but systemd
is still running the old path".

**Log volume is a retention decision, not a cosmetic one.** `robotd`'s per-tick heartbeat is
at `debug`; at `info` it logs one summary every five minutes carrying the achieved tick rate
as a percentage of target. A per-tick line at `info` would be ~86k entries a day from an idle
robot, and under a journal size cap those entries are what *evict* the logs an incident
needs. The summary also says more: a loop running at 60% of target is alive and passing its
health check, and nothing else would show it.

### 8.2 Two records, deliberately different durability

| | where | survives power loss | capped by |
|---|---|---|---|
| service logs | journald | only if configured, see `deploy/README.md` | `SystemMaxUse` |
| **update history** | `/var/lib/robot/updater/update-log.jsonl` | **yes** | 200 entries |

The update history is not in the journal on purpose. It lives in the engine's `state_dir`
under `/var/lib`, each entry is `fsync`ed on append, and rewrites are atomic
(temp + rename + parent `fsync`). So "what did this robot install, and what happened" is
answerable on a robot whose journal was volatile or wiped — which is the realistic support
case, not the ideal one.

### 8.3 The running version and the installed version are different questions

`updaterd` never restarts itself during an update (`updater-design.md` §4.1), so **after
every update the running binary legitimately lags the installed release until the next
reboot**. Any tool reporting one version number is therefore wrong half the time, and wrong
in the direction that makes a working robot look broken.

`robotctl version` reports both and names the disagreement:

- `updaterd` behind the installed release → expected, resolves at reboot;
- `robotd` behind it → *not* expected, because it is in `on_apply`'s restart set, so the
  restart did not take effect.

Those are different diagnoses and must not share one message. `--json` gives the same
content for a support bundle, and the command works when `updaterd` is **down** — reporting
that as a line rather than exiting, because that is when someone reaches for it.

Four independent places a version is recoverable, so losing one is survivable: the startup
log line; `robotctl version` over IPC; `--version` on every binary; and `version.toml` inside
each release directory (plus `robotctl update list`, which shows the revision each installed
release was built from).

`revision` is compiled in from `DUCK_REVISION` — set by CI, absent locally, where the binary
honestly reports `rev unknown, not a CI build`. Read at compile time, never from git at
runtime: a shipped robot has no repository. It matters more than it looks: once branch
installs land (roadmap M2) several builds share a version number, and the revision is the
only thing separating them.

### 8.4 Health is one question, so it is one command

`robotctl health` reports hardware from `robotd` and software from `updaterd` in a single
answer. That is not a convenience: "what is wrong with this robot" does not divide into
hardware and software until *after* it is answered, and a robot that reverted a release an
hour ago looks exactly like a robot with unpowered servos until both halves are on screen
together. Splitting it would make the caller pick a half before knowing which one is at
fault.

It **exits non-zero when the robot is unhealthy or unreachable**, so it can gate a script —
the contract anything built on top depends on. Nothing else affects the exit code: a flat
pack, a hot motor and a pinned component are reported, not judged. A release must never be
rolled back for the state of the board it landed on.

`--json` carries the same content for a support bundle.

## 9. Open questions

1. **Internet-reachable telepresence in v1, or LAN-only?** The big one: the
   difference between no backend and operating signaling + TURN (§5.4).
2. **Is the SDK team-internal or shipped to end users?** Determines how hard the
   API-compatibility commitment is (§4.2).
3. **Authority priority** when app, remote peer, and autonomous behaviour
   disagree — even a crude fixed order, but decided rather than emergent (§6).
4. **Perception in `mediad` or its own `perceptiond`?** Bundling keeps inference
   next to the camera and is simpler; splitting means a perception crash doesn't
   kill the video stream. Depends how heavy vision gets.
5. **Behaviour/brain layer** (drives, mood, habits): part of `robotd`, or its own
   service and update channel? Its learned state is `updater-design.md` §5.7
   material either way.
6. **Bond revocation over BLE.** Nothing un-pairs a phone; `bluetoothctl untrust` is the
   manual escape. Needs an API and a rule about who may call it
   ([`app-path-design.md`](app-path-design.md) §5).
7. **Per-device provisioning state**, which now has two claimants rather than one: the robot's
   serial (§5.7) and its per-robot pairing PIN. Both need generating, recording and printing at
   manufacture, and defining that once is cheaper than twice.

## 10. Build order

`updaterd` is built **first**, then used to ship every subsequent iteration of
the rest of this architecture. This front-loads update-system risk while failures
are still free (no clients, nothing valuable to break) and means the update path
is exercised hundreds of times before a robot ships.

Consequences to respect while doing it:

- `updaterd` is built against an **interface** to `robotd` (health probe,
  safe-to-restart), not an implementation — stubbed initially, which is also what
  makes it testable.
- Early health probes will be weak ("process alive"). Auto-rollback confidence
  grows as `robotd` matures; the *mechanism* is what's under test first.
- **Keep a manual recovery path (SSH / reflash) throughout early development.**
  The updater is both unproven and rapidly changing, and it ships inside the
  artifact it updates — do not make it the only way back.
- Schema fields that cannot be retrofitted (`min_supported`, `schema_version`,
  `model_api`) are in from the first release even if unused.
