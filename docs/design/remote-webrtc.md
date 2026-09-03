# WebRTC: sessions, signalling, and the control channel

How a phone, a browser or a server-side program drives and observes the robot over WebRTC.
[`architecture.md`](architecture.md) §5 states the requirement; this owns the mechanism.

Scoped to **local signalling**: everything below runs on the robot and works on a LAN with no
backend at all. Reaching a robot from outside the LAN is the same design with a proxy in front of
it (§7) — deliberately not the first thing built, because the local case is the one every other
case is defined in terms of.

## 0. Status: working on hardware

A Radxa Zero 3W streams `videotestsrc` through the hardware encoder to a browser on the LAN, and
the browser gets a `control` datachannel alongside it. Proven end to end on 2026-08-25:

| | |
|---|---|
| signalling | `mediad` runs the server in-process; producer registers, consumer lists and starts a session |
| video | `mpph264enc` → `webrtcsink` → browser, negotiated as `profile-level-id=42e01f` — constrained baseline, which is §2's whole point |
| bundling | `a=group:BUNDLE video0 application1`, `a=sctp-port:5000` — one transport for media and data |
| datachannel | `control` arrives at the peer |

Two things that had never run and both bit on the first attempt, recorded because they are the
shape of bug this design invites rather than one-offs:

- **`tokio::spawn` from a GStreamer signal thread aborts the process.** That thread is not in the
  runtime, and a panic crossing a C closure does not unwind. The journal said `thread caused
  non-unwinding panic` with a backtrace through `g_closure_invoke` and nothing about the cause.
  Nothing in those handlers may panic — see `mediad::pipeline`'s header.
- **The client could not reach the signalling port from a `file://` page.** An opaque origin to a
  private IP is what Chrome's Private Network Access blocks; serving the page over
  `http://localhost` fixes it.
- **A codec that fails negotiation is dropped with a warning, not an error.** Moving from
  pre-encoded H.264 to raw video (§2) puts the encoder inside `webrtcsink`, and its discovery pass
  demands `profile=constrained-baseline` — which `mpph264enc`'s pad template did not list, so H.264
  vanished from the offer, VP8 was negotiated instead, and the session died. Every symptom pointed
  somewhere else: the visible error came from a `videorate` four elements upstream, complaining
  about NV12. This needs plugins release `v3` or later, and it is why `mediad` bridges GStreamer's
  debug log *and* the pipeline bus into the journal — without that, none of the above was visible
  at all.

### The camera, and two independent causes of a 35% frame loss

The head camera streams at **29.3 fps** through the hardware encoder. Getting there took two
unrelated fixes, and the reason it took a while is that neither one alone moves the number —
which made each look ineffective.

**Capture pool depth.** rkisp implements no `V4L2_CID_MIN_BUFFERS_FOR_CAPTURE`, so
`gst_v4l2_object_decide_allocation` computes `own_min` from zero and lands on two buffers. Three
is a cliff, not a slope: `v4l2-ctl --stream-mmap=N` on the main path gives 19.7 fps at two and
29.2 at three or more. Raising it needs both a `GstVideoMeta` in the ALLOCATION query (or
`can_share_own_pool` is false and the branch that reads the query's `min` is never taken) and a
first pool whose `min` is non-zero (any downstream element proposing a pool sets `update` and
forfeits the `+2` bonus — `GstVideoEncoder::propose_allocation` proposes exactly that).

**The pixel format.** rkisp offers a non-contiguous two-plane `NM12` alongside single-plane
formats. Both map to GStreamer `NV12`, `v4l2src` prefers the multi-plane one, and it cannot drive
that at full rate here at any pool depth:

| caps | 2 buffers | 4+ buffers |
|---|---|---|
| `NV12` (selects `NM12`) | 19.5 fps | 19.6 fps |
| `UYVY` (single plane) | 19.7 fps | **29.3 fps** |

`mpph264enc` lists `UYVY` on its sink pad and converts on the RGA, so the 4:2:2 to 4:2:0 step
costs no CPU.

**What was ruled out, by measurement rather than argument** — each of these was a plausible
suspect: `v4l2-ctl` reaches 29.2 fps with either format on the same node; the sensor subdev
reports a 1/30 interval; the driver implements no `S_PARM`; `mpph264enc` encodes 720p at 130 fps
flat out; and the DMABuf caps `v4l2src` prefers when unconstrained are not the reason it is fast
unconstrained.

**The methodological lesson, which cost more than any of the above.** Four different rates were
each taken for the capture rate and none of them was: `rkvenc` interrupts count what the *encoder*
consumed, behind `webrtcsink`'s queue and the `videorate drop-only` in its converter bin;
`v4l2src`'s `lost frames detected` counts gaps in driver sequence numbers and goes silent when the
source is merely slow; a counter on the tee's raw branch sits behind a deliberately leaky
one-buffer queue; and `/dev/video1` is the ISP's self path, not the main path the daemon uses.
`mediad` now meters the pad before the tee, which is the only place with nothing lossy between it
and the driver.

Two things this exposed that are not fixed: `rtpgccbwe` costs about 40% of a core, and GStreamer's
own `INFO`-level log never reaches the journal despite the bridge mapping it to `tracing::info!`,
which is why several of these questions were answered the slow way.

What is still untested: anything at all through a bridge.

### Picking a quality, and why it is one setting rather than four

What the stream is — camera or test pattern, frame size, rate, bitrate — is `[media]` in
`/etc/robot/robotd.toml`, the per-board config file `robotd` already reads. `sudo robotctl
configure` edits it and offers the `systemctl restart mediad` it needs; `mediad` reads it once at
startup, like every other daemon here reads its config.

**One `quality` key naming a rung — `1080p30`, `720p30`, `720p15`, `360p30` — rather than a width,
a height and an fps.** Those three do not vary independently: a combination the capture path
cannot produce is a pipeline that does not start, and that costs the *control* channel along with
the video, because the datachannel is bundled with the video track (§2). Every rung is 16:9, the
sensor's own aspect, so "smaller" never quietly means "cropped". `bitrate` is the one number that
can still be set on its own, and left unset it follows the rung — 2 Mb/s at 720p30, the rate every
measurement above was taken at.

The numbers were `ExecStart` flags in `mediad.service` until the section existed. The release
installer rewrites that unit file, so changing one meant a systemd drop-in — a mechanism for a
board that is *wired* differently, not for someone asking why the picture is soft.

### Congestion control is a CPU setting too

`[media] congestion_control` — `gcc`, `homegrown` or `disabled` — is `webrtcsink`'s own
`congestion-control` property, set rather than inherited. `gcc` is the element's default, so naming
it changes nothing; what it buys is that the day upstream changes that default is not the day every
robot's send rate changes with it.

**It is the largest single CPU consumer in the process.** Per-thread, with one peer connected on
the board:

| thread | %CPU (of one core) |
|---|---|
| `rtpgccbwe1:src` | 7.6 |
| `queue1:src` | 6.0 |
| `mpph264enc2:src` | 5.0 |
| … | |
| `v4l2src0:src` | **0.3** |
| `queue3:src` (the raw tee branch) | **0.3** |

~25% of one core in total, and the shape of it is the point: capture and the raw-branch frame copy
— the only two things that scale with *pixels* — are 0.6% between them. Everything else is packet
work, which scales with bitrate. That is why picking a smaller rung does not lower CPU: on a link
that never saturates, the estimator ramps a 360p stream to about the bitrate the 720p one was
using, and spends it on quality per pixel instead.

`disabled` deletes the `rtpgccbwe` thread. It costs adaptivity, which is the whole reason
`webrtcsink` is handed raw video rather than pre-encoded H.264 (§1) — on a degrading link it is
what keeps a picture instead of a stall. It also makes `bitrate` mean what it says: nothing moves
it, so it is the rate rather than a starting point.

**720p30 is the only measured rung.** The sensor is pinned to a 1920x1080 mode that runs at 30 and
the ISP scales down from it, so 1080p30 asks for no scaling at all; what nobody has measured is
whether the capture path and the encoder hold 30 fps at 2.25x the pixels of the table above. A
rung that does not hold runs slower — it is not a failure to start.

## 1. What this is not

`webrtcbin` is not used. `mediad` uses **`webrtcsink`** from `gst-plugins-rs`, and the difference
is the whole reason this document is short: `webrtcsink` brings a signalling protocol, a session
model, and per-consumer encoder management, so what is left to design is the *control* surface
rather than the media plumbing.

`webrtcbin` would mean writing all three. It is in Debian and `webrtcsink` is not
([`media-bringup.md`](../project/media-bringup.md) covers how that plugin is built and shipped),
which is the one argument for it — and it is outweighed by the protocol coming for free, because
that protocol is what a remote bridge proxies (§7).

## 2. One session, four streams

```
peer (browser / phone / server-side program)
   ├── video track      camera, hardware H.264 (mpph264enc, constrained-baseline)
   ├── audio track      mic; two-way for telepresence
   ├── datachannel "control"   reliable, ordered      → the robot API (§5)
   └── datachannel "teleop"    unreliable, unordered  → input and high-rate telemetry
```

Two data channels rather than one, for the reason `architecture.md` §5.2 gives: a retransmitted
80 ms-old joystick command is worse than useless, so teleop goes `maxRetransmits: 0` and always
takes the newest.

**The first version opens `control` only.** Teleop is not the near-term priority, and leaving it
out is not merely deferral — §6 is about what it removes.

**`webrtcsink` takes pre-encoded H.264 on its sink pad**, so the encoder never reaches
negotiation. Verified on hardware; the four encoder properties that are decisions rather than
defaults are in [`media-bringup.md`](../project/media-bringup.md).

**The pipeline tees raw NV12 before the encoder**, and that placement is deliberate:

```text
                              ┌─ queue ─ mpph264enc ─ h264parse ─ webrtcsink
capture ─ NV12 ─ capsfilter ─ tee
                              └─ queue(leaky, 1) ─ appsink ─ a frame on request
```

§5.3 wants a frame on demand for a server-side program — "it wants a frame every second or two plus
a state blob", not a 30 fps H.264 track to decode — and `architecture.md` §2 wants perception next
to the sensor, deriving features rather than shipping pixels to `robotd`. Both need *pixels*, and
taking them off the encoded branch would mean decoding what was just encoded.

NV12 throughout, because that is what the rkisp path emits and what `mpph264enc` accepts, so
nothing converts anywhere. Each branch has its own `queue` — a `tee` without them runs both from
one thread, so a slow reader would stall the video track — and the raw one is leaky and one buffer
deep, which is the *latest* snapshot `architecture.md` §2 asks for rather than a queue of stale
ones. A stalled reader costs frames, never the encoder.

**A frame is copied out of the appsink only when a reader has asked for one.** The readers on this
branch want two frames a second between them — auto-exposure meters one every 500 ms, the duck
detector looks twice a second when it is enabled at all — and a frame is 1.84 MB at 720p30.
Capturing all thirty was 55 MB/s of memcpy and a 1.8 MB allocation thirty times a second, from
boot, on every robot, for frames nobody read. Every buffer still reaches the appsink and is dropped
there unread; the cost on a frame nobody wants is a relaxed load.

The request is answered by the *next* capture rather than by the last one delivered. Handing back
whatever was lying around would be cheaper again and would put the reader's own polling period into
the measurement — an exposure loop steering on luma from half a second ago is a loop that hunts. A
reader blocks until its frame lands, bounded by a timeout, and a camera that has stopped reaches
the same "no frame" path it always did.

The branch exists from the start rather than being added when something reads it: inserting a tee
into a live pipeline is a materially harder problem than having one that was always there.

## 3. `mediad` runs the signalling server itself

`webrtcsink` has `run-signalling-server`, with `signalling-server-host` and
`signalling-server-port` (gst-plugins-rs 0.15.3). So the server runs **in `mediad`'s own process**
and there is no second binary to build, ship or supervise — which matters, because the plugin we
ship is a `.so` while `gst-webrtc-signalling-server` is a separate Rust binary from the same
upstream crate. Not shipping it is a real simplification, not a shortcut.

`webrtcsink`'s own signaller defaults to `ws://127.0.0.1:8443` and connects to the server it just
started. A LAN client connects to the same server directly — `mediad/webclient/index.html` is one,
in a single file with no build step, which speaks this protocol by hand rather than through
`gst-plugins-rs`'s JS library so that trying it needs nothing installed.
[`webrtc-console.md`](webrtc-console.md) owns what becomes of that page — the robot serving it,
finding the robot, and the control surface it should reach.

**Bind address is a decision, not a default.** Loopback only would mean a LAN peer cannot reach it
at all and every session goes through a bridge, which defeats the point of a local mode. So it
binds on all interfaces, and §4 is what that implies for who may drive.

## 4. Authorisation: none on the robot, and why that holds both locally and remotely

**No gate in the first version.** A peer that reaches the signalling server can start a session,
drive the robot, and see through its camera. That is a decision, not an oversight.

Usability outranks hardening at this stage, and here the trade is not even close. The robot's
pairing PIN is a shared `000000` — a PIN that is the same on every robot authenticates nobody — so
requiring it over WebRTC would add a step to every first connection and buy no safety at all. An
awkward first connect is a real cost; this particular gate is a real cost with no benefit.

What it costs, stated plainly so nobody has to discover it: anyone on the same network has the
robot and its camera. Fine on a bench and in an office. **Not fine in a home**, which is the thing
to revisit before one ships to one.

### Where authorisation actually lives: the bridge, and it is already there

The remote path does not need a gate on the robot either, because it is authenticated **before it
reaches one** — on both sides:

- **The client** authenticates to the rendezvous service with OAuth, and the service shows it only
  the robots its account owns. Reaching the part of the bridge that routes to a given robot *is*
  the proof.
- **The robot** authenticates outward: its relay holds an account token and connects to the service
  with it (§7). So the robot proves it belongs to the account too.

The service is therefore matching two already-authenticated parties, and a session arriving through
it has been authorised twice over. A `system.authenticate` on top would be a second answer to a
question already answered — and a worse one, since the shared `000000` PIN proves less than an
account token does.

**What this means is that the trust moved rather than vanished**, and it is worth naming where it
went: the robot has no independent check, so the binding between a robot and an account is now the
thing that must be right, and it lives in the service rather than here. That is an acceptable place
for it — it is the only component that can know the answer — but it is a dependency, not an absence
of one.

The one arrangement none of this covers is a robot whose signalling port is exposed to the internet
directly, by a port forward rather than through the bridge. Then there is no bridge to have
authenticated anything, and §4's LAN reasoning does not apply either, because the population that
can reach it is no longer the people in the building. That is a deployment mistake rather than a
design decision, and worth saying out loud precisely because nothing in the robot would notice.

### The hook, if it is ever wanted

`system.authenticate` — the method BLE already uses, added in `API_VERSION` v4. A control channel
would serve that one method and refuse the rest by name until it passes, with the PIN read from
`configd` over its unix socket rather than over the channel being authenticated.

It is named here so the answer exists, not because it is planned. The case for it is narrow: it
adds nothing to the bridged path, which is better authenticated already, and on the LAN it costs a
step per connection while proving only that the peer read a number printed on every robot. If it is
ever wanted, it is cheap — §5's routing table already needs a notion of which methods a transport
may reach, and "which methods before authentication" is the same table with a smaller subset rather
than a new mechanism.

## 5. The control channel is a pipe to the existing API

Frames on `control` are **JSON-RPC 2.0, one object per line**, which is what
[`duck-ipc-proto`](../../duck-ipc-proto/src/lib.rs) already defines and what `robotctl` and `btd`
already speak. Nothing new is invented: `mediad` routes a call to the unix socket of the service
that owns it and pumps replies back.

`btd` is the working precedent and should become the shared one:

| `btd` today | what `mediad` needs |
|---|---|
| `route.rs` — which calls may travel, which socket answers, which lane carries them | the same table, with a *different permitted subset* |
| `session.rs` — the `system.authenticate` gate | the same gate |
| `upstream.rs` — dial the sockets, timeout everything | the same |
| `framing.rs` — BLE MTU chunking | **not needed**; SCTP frames itself |

So three of the four files are transport-independent and one is not. **Lift the routing table into
something both transports use**, parameterised by which subset a transport may reach.

The property worth preserving is not the code, it is the exhaustive match. `route.rs` says it
plainly: adding a variant to `proto::Call` makes the file fail to compile, so a new method cannot
reach the radio because somebody forgot this file existed. A `_ => None` wildcard would deny new
methods silently, and the first symptom would be an app that cannot see a feature nobody remembered
to route. That guarantee has to hold **per transport**, or WebRTC becomes the hole in it.

### What WebRTC may reach, and why it is not BLE's subset

BLE's subset is narrow because the radio is slow and anyone within a few metres can talk to it.
Neither applies here, so WebRTC gets more — but "more" is not "everything", and two categories stay
out:

- **`system.pairingPin` and `system.setPairingPin`.** Not because they would compromise *this*
  transport — §4 leaves it open anyway — but because they authorise a **different** one. A LAN peer
  that can rewrite the pairing PIN can lock a phone out of BLE, which is the recovery path. Keeping
  the PIN off every network transport is the same rule that makes it unroutable to BLE itself.
- **`update.*` mutations.** For now only, and for a different reason than the PIN: applying an
  update restarts `mediad` and drops the session. Wanted later; §8 is what it will take.

And one category came *in* that is worth naming here rather than only in the route table:
**`account.*`**, the calls that bind this robot to a Hugging Face account. They are the first
mutating calls this transport carries, and the console is where somebody would sign a robot in.
It is also the only thing here whose effect outlives the session in the way an account does —
[`remote-access-design.md`](remote-access-design.md) §2.6 is the argument and the three
properties that make it hold, and `mediad::route` carries the short version.

### Replies are not correlated, deliberately

`btd` forwards whatever a socket emits without parsing it, and has a test pinning that: a
subscription is a stream of notifications on an open connection, and every one has to reach the
client. Correlating replies to requests would break exactly that. `mediad` inherits the same rule,
which also means **no per-method work in `mediad` when a method is added** — the pipe stays dumb,
and `duck-ipc-proto` stays the only place a method is defined.

The lane concept transfers too, and it is easy to assume it will not. Every daemon serves one
request at a time per connection, so `update.subscribe` followed by anything else on the same
connection hangs — the exact bug `app-path-design.md` §7 records. One datachannel is one ordered
stream with the same hazard, and `btd`'s answer works unchanged: route by method to a per-lane
socket, pump each socket back, never correlate.

## 6. Why `control`-only comes first, and what `teleop` will cost when it lands

`intents.rs` stores each intent in an `ArcSwap` and takes last-writer-wins. That is correct today
because every writer reaches it through a unix socket, where a later message cannot arrive before an
earlier one.

**A reliable, ordered datachannel keeps that true.** SCTP in that mode delivers in order by
definition, so intents arriving over `control` preserve the property `intents.rs` already depends
on. Starting with one channel is therefore not a compromise that stores up work — it means there is
no ordering problem to solve in the first version at all.

### What it costs instead, so nobody is surprised

Head-of-line blocking. On a reliable channel a lost packet stalls everything behind it, including
the control RPCs, so a bad link shows up as *everything* pausing rather than as a stale joystick.
Driving over `control` is fine at a modest rate and gets worse with rate and loss — which is
precisely why `architecture.md` §5.2 specifies a second channel, and why the answer to "the robot
feels laggy over a poor link" is teleop rather than tuning.

### And when teleop lands, it needs sequence numbers

**SCTP with `maxRetransmits: 0` reorders.** A twist from 80 ms ago can land after a fresher one and
win last-writer-wins, and the robot then drives on a stale command with nothing anywhere reporting a
problem. It is not a rare race: it is the normal behaviour of the channel, chosen deliberately.

So teleop frames carry a **monotonic sequence number per stream**, and the writer drops anything not
newer than what it last applied. This is a property of the *transport*, so it belongs in `mediad`
rather than in `robotd` — `robotd` should keep receiving intents whose ordering it can trust, which
is what lets `intents.rs` stay as simple as it is.

Worth writing down before the channel exists rather than after: the failure is silent, it looks like
bad tuning rather than a bug, and the fix is trivial if it is designed in and awkward if a stale
twist has to be diagnosed first.

The deadman needs nothing either way: `safety.gate(command, twist_age)` is already age-based, so a
partition stops the robot with no new code.

## 7. Reaching a robot that is not on your LAN

**The remote path is a bridge to the local signalling server, not a second design.** A relay
process connects outward to a rendezvous service and proxies the same protocol to
`ws://127.0.0.1:8443`. The robot's signalling server, session model, authorisation and control
channel are unchanged; DTLS-SRTP keeps media encrypted end to end even through a relay, which is
worth stating to clients plainly.

Two properties follow, and both are the reason for this shape:

- **Local mode never depends on the bridge.** If the rendezvous service is down, a LAN client still
  connects. Invariant 1 in `architecture.md` — local recovery stays independent — extends to media.
- **The bridge invents no protocol.** The envelopes are the gst signalling protocol's, which is what
  a LAN client already speaks, so the payload — SDP and ICE — passes through opaque. That is the
  concrete payoff for using `webrtcsink` rather than `webrtcbin`.

  It does **not** follow that the bridge is a pure relay, and this bullet used to say it was.
  `reachy_mini`'s rendezvous carries those envelopes over HTTP — SSE inbound, `POST` outbound —
  rather than over a WebSocket, and peer and session ids are per-hop. So the bridge rewrites the
  envelope and keeps a session table both ways: a translator with an opaque payload.
  [`remote-access-design.md`](remote-access-design.md) §3.2 has the two sides side by side.

- **The bridge authenticates, so the robot does not have to.** The relay connects *outward* holding
  an account token, and the service shows a client only the robots its account owns — so a bridged
  session is authorised on both sides before it arrives. §4 covers where that leaves the trust.

  A useful consequence: because the relay is a robot-side process connecting to loopback, the robot
  *can* tell a bridged peer from a LAN one by source address, even though it does not currently act
  on the difference. Nothing is foreclosed if that stops being true.

`reachy_mini` runs exactly this arrangement against a Hugging Face Space, with the robot
registering as a `producer` and the Space keeping a TTL lease refreshed by a heartbeat. Local mode
works now, so the two questions this section left open — whether we adopt that service, and how a
robot is bound to an account — are open no longer for want of asking:
[`remote-access-design.md`](remote-access-design.md) owns both, and the account is a Hugging Face
OAuth **device** flow rather than the redirect flow `reachy_mini` uses (§2.1 for why).

### The signalling protocol, for whoever writes the bridge

From `gst-plugins-rs` 0.15.3, `net/webrtc/protocol` — the wire is JSON with a `type` tag,
camelCase:

| peer → server | server → peer |
|---|---|
| `setPeerStatus` (`roles`, `meta`, `peerId`) | `welcome` (`peerId`) |
| `startSession` (`peerId`, optional `offer`) | `sessionStarted` (`peerId`, `sessionId`) |
| `endSession` (`sessionId`) | `startSession`, `endSession` |
| `peer` (SDP `offer`/`answer`, or `ice`) | `peer`, `error` (`details`) |
| `list`, `listConsumers` | `list` (`producers`), `listConsumers` (`consumers`) |

Roles are `producer`, `listener`, `consumer`. The robot is a `producer`; `meta` is free-form JSON
and is where a robot's identity goes.

## 8. Updating the robot over WebRTC: not yet, and what it will take

Not in the first permitted subset, and **that is a deferral rather than a principle** — a phone
updating a robot over WebRTC is wanted later, so this section is about what has to be true first
rather than why it must not be.

What makes it awkward today: applying an update restarts `mediad`, which drops the session the
client is watching progress on. `update-over-ble.md` records what "start an update and watch it"
failing silently already cost once, and a session that vanishes mid-update is the same shape of
problem. So the first subset leaves `update.*` mutations out and BLE stays the transport that
survives the restart. Read-only `update.*` calls are in from the start — seeing version and history
over a remote session is useful and costs nothing.

Two things have to change for the mutations, and both are small and specific:

- **The client has to survive the restart.** The protocol already supports it: progress is pushed
  as a JSON-RPC *notification*, which `duck-ipc-proto` documents precisely so a client that
  reconnects mid-update can resubscribe and keep receiving them. So the work is a client that
  reconnects and re-subscribes, not a change to the wire format.
- **`RobotRemoteSessionActive` has to get more specific.**
  `updater/src/preflight.rs::check_no_remote_session` refuses an update while a remote session is
  up, which is right when the session is a *bystander* — someone is on a telepresence call and
  should not have the robot restarted under them. It is wrong when the session is the *requester*.
  Nothing sets that flag true yet, so the distinction can be designed in rather than retrofitted:
  the check needs to know whether this update was asked for over the session it is about to drop.

Worth writing down now precisely because nothing sets the flag yet. The moment `mediad` reports
honestly, an update requested over WebRTC would refuse itself, and that would look like a bug in
the update path rather than a missing distinction here.

## 9. Authority: the premise this feature breaks, noted and not acted on

`intents.rs` says its slots are "single-writer in practice, so last-writer-wins means what it
says". That is true with one gamepad. **It stops being true the moment a pad and a remote peer both
drive**, and the failure is not a contest — it is two writers at 50 Hz interleaving into one slot,
producing a robot that obeys neither.

**Deliberately not solved here.** It is recorded so that a confusing robot has an explanation
waiting, and because the flag that eventually resolves it is cheaper to design before there are two
transports than after. `architecture.md` §6 owns the requirement — defined priority and handoff,
local physical able to preempt remote — and the roadmap has it in M6.

When it is time, the cheap answer is a **single-writer token**: one peer holds the right to write
intents, others are observers, and handing it over is explicit. That is much less than §6's full
arbitration and it removes the interleaving, which is the part that produces nonsense rather than
merely the wrong winner. Priority ordering — physical preempting remote without asking — can come
after, on top of the same token.

What this section is *for* until then: knowing that two simultaneous drivers is a known gap rather
than a mystery, and that the first symptom is a robot ignoring both inputs rather than obeying the
wrong one.

## 10. Building `mediad` at all

The `gstreamer-rs` crates are pkg-config crates, so cross-compiling them needs the *target's*
headers, `.pc` files and shared libraries on the developer's machine. `cargo board` cross-builds
from macOS with `cargo-zigbuild`, and the multiarch script that used to supply its one C
dependency said of it that it "is the cost of that one exception, and it is worth reading before
adding another". This is the second, and much larger — and it replaced that script outright,
because Ubuntu multiarch can serve libudev but cannot honestly serve GStreamer: it would give
Ubuntu's while the robot runs Debian trixie's.

**`scripts/cross-sysroot.sh` unpacks the robot's own Debian packages into a sysroot** — proven:
the full workspace cross-builds against it, and `gstreamer`, `gstreamer-app` and
`gstreamer-webrtc` all resolve at 1.26.2, the same version the board runs.

Three things about it worth knowing before touching it:

- **It serves the whole workspace, not just `mediad`.** `PKG_CONFIG_LIBDIR` *replaces*
  pkg-config's search path rather than adding to it, so a sysroot carrying only GStreamer breaks
  `padd` — whose `gilrs` needs libudev — and does it inside `libudev-sys`, nowhere near anything
  about media. Replacing is still right: `PKG_CONFIG_PATH` is additive to the *host's*, which is
  how pkg-config comes to answer with a macOS library and produce a binary that cannot run on the
  robot.
- **The package list is explicit, not resolved.** Walking Debian `Depends` from the obvious roots
  pulls 543 packages, because `libgstreamer-plugins-bad1.0-dev` declares every optional backend's
  dev package and the closure reaches Qt, Vulkan and OpenEXR. Nineteen packages satisfy what is
  actually needed.
- **A `-dev` package alone is not enough for anything actually linked.** It ships `libfoo.so` as a
  symlink onto the `libfoo.so.N` in the runtime package, so `-lfoo` needs both. Libraries that
  only appear in `Requires.private` need just the `-dev`.

The alternative was building `mediad` on an arm64 runner like the plugins in
[`media-bringup.md`](../project/media-bringup.md). Rejected because it splits the daemon build in
two and leaves nobody able to build `mediad` on a laptop — which for the crate that will need the
most iteration against real hardware is the wrong trade.

## 11. Deferred, with reasons

- **A WebSocket surface for server-side programs** (`architecture.md` §5.3). Same JSON-RPC, no
  media stack, `get_frame` returning a JPEG. It is a few dozen lines once §5's routing exists, and
  it is what makes "an LLM drives the robot" easy — but it is a second transport and the first one
  should work.
- **The `teleop` datachannel.** Not the near-term priority; §6 covers what deferring it removes,
  what it costs in the meantime, and the sequence numbers it will need.
- **Multi-peer video.** One media session at a time, plus control-only clients. Simulcast and
  encode-once-send-many are a real project.
- **Consent and the streaming indicator.** `architecture.md` §7 wants explicit per-session consent
  and a visible indicator, and is right that they are cheap now and expensive later. They need
  hardware that exists — an LED under software control — which is not yet established.
- **TURN.** LAN-only needs none. A bridge does, and it costs real bandwidth; that decision belongs
  with the rendezvous service, not here.
