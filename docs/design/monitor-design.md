# Wire Monitoring — seeing what the services say to each other

Status: draft · Date: 2026-08-10 · Owner: pierre

Companion to [`architecture.md`](architecture.md), which owns the IPC contract (§2) and the
observability contract (§8). This document covers one question those two leave open: how
anybody sees the messages the services actually exchange — at the bench while building, and
on a robot in someone's home when support is asked what went wrong.

Nothing here is implemented. This is the design to argue with before any of it is written.

## 1. Two questions, and why they are one mechanism

"Monitor the message exchange" is two different requests wearing one name.

| | **the trace** | **the meter** |
|---|---|---|
| Asked by | a developer at the bench | support, or a script, on a live robot |
| Question | "the phone pressed update and nothing happened — where did it stop?" | "is this robot healthy, and who is talking to it?" |
| Content | individual messages, in order, with cause | counts, rates, latencies, drops |
| Span | one operation across several services | one service over hours |
| Volume | everything, briefly | a fixed-size summary, always |
| Lifetime | while you watch, or in the journal afterwards | continuous, bounded memory |

They are not two systems. They are **one event stream folded two ways**: the trace prints
each event, the meter accumulates them. Building them separately is how a robot ends up with
a log saying one thing and a counter saying another, and no way to tell which lied.

There is a third fold, and §9 argues it is the most valuable one on a robot nobody can attach
a debugger to: the **last** of both, on disk, for whoever arrives after the service died.

`robotctl monitor` already establishes the shape in this codebase — one stream from
`robot.state`, rendered as a repainting frame for a terminal and as one line per tick for a
pipe. Same stream, two renderings, chosen by context. This extends that idea from one
service's telemetry to every service's traffic.

## 2. What is observable today

The message graph, with `mediad` still to come:

```
robotctl ──┐
padd ──────┼──▶ robotd    robot.*, incl. ~50 Hz intent notifications
btd ───────┼──▶ configd   net.*, system.*
           └──▶ updaterd  update.*
updaterd ──────▶ robotd    safeToRestart, health, modelApi, remoteSessionActive
```

What exists:

- **Mutating calls are logged with the caller.** `configd` and `updaterd` both authorise
  against `SO_PEERCRED` and log method + uid/gid (+ pid, in `updaterd`) at `info`, refusals at
  `warn`. This is the seed of the whole design: already the right event, with the right
  identity attached, in the right place — it just covers one call class in two of the four
  services.
- **`robotctl monitor`** renders `robotd`'s state stream, which is the control loop's
  telemetry rather than its traffic.
- **Drop counters exist but are invisible.** `robotd` and `updaterd` both notice a subscriber
  falling behind and log it at `debug`; nothing accumulates it, so "this robot has been
  dropping a third of its state frames all week" is not a question anyone can ask.

What is missing:

- **`robotd` observes nothing.** It has no peer policy and logs no calls, so the busiest
  socket on the robot is the one with the least visibility.
- **Nothing is correlated.** JSON-RPC ids are per-connection. `btd` forwards lines verbatim so
  an id survives that hop, but `updaterd`→`robotd` mints its own starting at `1`. There is no
  way to join a phone's tap to the `robot.safeToRestart` it eventually caused.
- **Read-only calls are entirely dark**, deliberately ungated (§2.2) and therefore unlogged.
  "Support cannot change this robot but must be able to inspect it" is the right rule; it also
  means the inspection traffic is the traffic nobody records.

## 3. Where to observe: the dispatch boundary, not the socket

The tap goes where a service turns a line into a typed `Call` and produces a `Response` —
**not** on the socket, and not in a process between the two.

### 3.1 Why not an interposing proxy

The cheap design is a `robotctl tap` that binds a socket, splices to the real one, and prints
both directions. Zero daemon changes, and every client already takes a socket override
(`robotctl --socket/--robot-socket/--config-socket`, `btd --updater-socket/…`, `padd
--socket`), so it would work today. Two properties disqualify it as the durable answer:

- **It launders authorisation.** `may_mutate` reads `SO_PEERCRED`, which behind a proxy
  reports the *proxy's* uid for every caller. So either mutating calls stop working through
  the tap, or — run as an allowed uid to make them work — the tap becomes a tool that grants
  any peer permission to change the robot. That is not a thing to leave on a board.
- **It is blind to every transport but one.** §4.1 is "one definition, many transports": BLE,
  unix socket, WebSocket, WebRTC datachannel. A unix-socket proxy can see one of the four. It
  cannot see the phone→`btd` leg, and it will not see `mediad`'s remote gateway. A tap at
  dispatch sees all of them by construction, because that is where every transport converges
  on `Call`.

It is also blind in a subtler way: bytes on a wire do not say that a call was *refused by
policy*, that a velocity was *clamped by safety*, or that a state frame was *dropped because a
subscriber lagged*. Those are the events worth having, and none of them exist on the wire.

### 3.2 Alternatives considered

| Option | Why not |
|---|---|
| Interposing proxy (`socat`, or ours) | §3.1: launders `SO_PEERCRED`, sees one transport, cannot see internal outcomes |
| `strace -e trace=read,write -p` | No code, genuinely useful once. Heavy on a Pi, no decoding, no redaction. Keep as a documented escape hatch, not a design |
| `bpftrace` / eBPF uprobes | Same, plus a toolchain a shipped board does not have |
| A message broker everything routes through | Fights invariant 1 outright: another component that can fail, on the recovery path (§2.2) |
| Each service writes its own wire log file | Fights §8.1: one mechanism, one retention policy, one place to look. §9's crash record is a bounded exception, not a log |

## 4. The shared serve loop comes first

The tap is four events, and they are emitted from **one** place: a serve loop shared by every
service, extracted before any monitoring is written.

The alternative — leave the three loops alone and call a tap from each — is cheaper by a
week and wrong by the standard this codebase already sets elsewhere. `route.rs` makes a
forgotten BLE route a compile error rather than a review comment, on the explicit grounds that
a wildcard is "the safe default in the moment and the wrong one over time". A tap that each
new service must remember to call is that wildcard. `mediad` is coming, the SDK is coming, and
the failure mode is silent: a service that is simply absent from the trace looks exactly like
a service that is quiet.

### 4.1 What the loop must absorb

The three loops are near-identical in shape and differ in four ways, each of which is a real
requirement rather than drift:

| | `robotd` | `configd` | `updaterd` |
|---|---|---|---|
| Peer policy | none | `may_mutate` | `may_mutate` |
| Line cap | 64 KiB | 64 KiB | **1 MiB** |
| Push side | `broadcast<RobotState>`, decimated per subscriber | none | `broadcast<Progress>` |
| Connection shape | select over request-or-frame | plain read loop | select over request-or-progress |

So the shared loop is generic over a dispatcher, an optional policy, a line cap, and an
optional push stream. That is a genuine abstraction rather than a copy-paste removal, and
underestimating it is the main risk in this plan. It is also the reason to do it while there
are three implementations and not six.

The extraction must be **behaviour-preserving per service**. `updaterd`'s 1 MiB cap in
particular is not an oversight to be normalised away; whatever it is protecting, a shared
default of 64 KiB would break it silently.

### 4.2 The question this forces

`robotd` passes no policy today. The moment the loop has a policy hook, "does `robotd` pass
`None`?" becomes a decision someone makes on purpose rather than a gap nobody noticed — which
is a gain, but it lands adjacent to something sharper.

`Call::is_mutating()` covers `update.*`, `net.connect/forget`, `system.setName/reboot/
setPairingPin`. It does **not** cover `robot.enable`, `robot.move`, `robot.stop` or
`robot.head`. So the call that starts a policy running on a walking robot is currently
classified alongside `hello`.

That is defensible — intents are rate-limited, safety-clamped and deadman-bounded, and the
whole design says `robotd` is authoritative regardless of what a client asks for — but it has
never been written down as a decision, and the extraction is the moment it stops being
implicit. Note also the consequence if it is ever revisited: `padd` sends `robot.enable`, so
classifying intents as mutating denies the gamepad unless its user is allowlisted.

**Neither question is this document's to settle**, and neither blocks the extraction. They are
recorded here because the refactor surfaces them and they should not be answered by accident
in a diff about serve loops.

## 5. The tap

Four events, emitted by the shared loop about the traffic passing through it:

| Event | Carries |
|---|---|
| `request` | peer identity, transport, method, correlation id, params (§6), arrival time |
| `response` | correlation id, ok/error + code, dispatch duration |
| `notify` | method, subscriber, correlation id |
| `drop` | what was dropped and why — subscriber lagged, queue full, line too large, parse error |

`drop` is not an afterthought. It is the only event that explains a robot that looks fine from
inside and wrong from outside, and it is the one a socket-level observer can never produce.

## 6. Params are opt-in, and redacted even then

`NetConnectParams` and `AuthenticateParams`/`SetPairingPinParams` carry hand-written `Debug`
impls that redact the wifi PSK and the pairing PIN, for a stated reason: services log calls
they could not handle, and a customer's wifi password must not be recoverable by anyone who
can run `journalctl`.

A monitor is exactly the thing that undoes that. It sees the JSON line *before* any `Debug`
impl runs, and if it writes to journald — or to §9's crash record — it undoes it durably. So:

- **The default rendering is method + peer + outcome + timing.** No params. This is enough for
  almost every trace: which call, from whom, what came back, how long.
- **Params are a deliberate opt-in**, per invocation, never the default and never the shipped
  configuration.
- **Even opted in, rendering goes through the typed `Call`**, so the existing redacting `Debug`
  impls apply. A tap that formats the raw line is a tap that routes around the redaction — the
  raw line must not be the thing that gets printed.
- **§9's on-disk record never carries params**, opt-in or not. It outlives the process that
  wrote it and nobody chose for it to be there.

This is the strongest single argument for tapping at dispatch rather than at the socket: at
dispatch the redaction is the natural path, and at the socket it is a thing to remember.

## 7. Correlation, echoed on everything

Neither rendering answers "where did it stop?" without a way to join events across processes.

A correlation id rides on `Request`, on `Response`, and on notifications. Four rules:

1. **A transport adapter mints one if absent.** `btd` today; `mediad`'s gateway later. A call
   arriving from outside the robot gets its identity at the front door.
2. **A service propagates it into any call it makes as a consequence.** `updaterd`→`robotd` is
   the case that exists now, and the one that is currently unjoinable.
3. **Every reply echoes the id of its request**, and every notification carries the id of
   whatever caused it — the `subscribe` for a stream, the `apply` for a progress line.
4. **Nothing parses it.** Opaque string, bounded length. `<service>-<pid>-<counter>` is free,
   unique on one box and readable in a log; a remote peer has no pid, so the rule is strict and
   the format advisory.

Echoing rather than joining locally is the deliberate choice: **a captured line is
self-describing**. Anything that can read the stream — a log excerpt pasted into an issue, a
`journalctl` dump from a robot in someone's home, a future `wire.subscribe` consumer that
attached halfway through — can reconstruct the tree without also knowing which connection the
line came from or which service's memory held the mapping. The alternative keeps the join
inside each process, which is exactly where it is unavailable to whoever is holding the
evidence afterwards.

The cost is a field on every reply, including the rate-bearing ones. It is smaller than it
looks and should be kept that way:

- Omitted entirely when absent (`skip_serializing_if`), so untraced traffic is byte-identical
  to today's.
- A 50 Hz `robot.state` stream whose `subscribe` carried an id pays roughly a kilobyte a
  second — against ~27 MB/s of video the same board is expected to move, and on a socket §2.4
  already calls trivially cheap.

This is a wire change and needs a `PROTOCOL_VERSION` bump. Doing it now is cheap and doing it
later is not — the same category as `min_supported` and `schema_version` in `architecture.md`
§10, fields that went in before they were used because they cannot be retrofitted. The install
consequence is real and worth naming: a bumped version means a stale `robotctl` or a
mismatched `btd` is refused at `hello`, so board and laptop have to move together.

## 8. The trace

One decoded line per event, correlated, with peer identity — the bench rendering.

```
12:04:31.201  btd      → updaterd  update.apply         trace=btd-812-7  peer=uid:0 pid=812
12:04:31.203  updaterd → robotd    robot.safeToRestart  trace=btd-812-7
12:04:31.204  robotd   → updaterd  ok safe=true         trace=btd-812-7  1.2ms
12:04:31.240  updaterd ⇢ btd       progress download 12%  trace=btd-812-7
```

Grouping by correlation id turns that into the tree a developer actually wants, and the
grouping is why §7 exists.

**Off by default**, and enabled without a rebuild — a level on a dedicated `tracing` target, so
`RUST_LOG` turns it on per service on a board that is already misbehaving.

### 8.1 What reaches the journal permanently

Three classes, and the boundary between them is a retention decision rather than a cosmetic
one (§8.1: an entry logged at rate is an entry that *evicts* what an incident needs).

| Class | Level | Why |
|---|---|---|
| Mutating request authorised / refused | `info` / `warn` | Exists today. "Who told this robot to reboot" is the first thing support asks |
| **Any response carrying an error**, mutating or not | `info` | A read-only failure is invisible today, and read-only is most of the surface |
| Everything else — the per-message stream | `trace` | ~86k entries a day from an idle robot at `info`, per §8.1 |

The error class is the addition, and it is the one that can be produced at rate: a client
retrying an unreachable peer, or a stale `robotctl` refused at `hello` in a loop, generates
identical errors indefinitely. Two bounds, both in the loop rather than left to the reader:

- **Identical consecutive errors on one connection collapse**, carrying a repeat count instead
  of a line each — the same shape as journald's own "repeated N times".
- **A per-connection error budget**: past a threshold the connection's errors drop to `trace`
  until it produces a success, and the demotion itself is logged once. A client failing
  10,000 times says nothing that its first failure and a count did not.

Without those two, "every error at `info`" is a remote client's ability to evict a robot's
journal, which is the failure mode §8.1 was written about.

## 9. The meter, and what survives a restart

A fixed-size summary per service, always on, readable over IPC — the field rendering.

Candidate content, all of it foldable from §5's four events:

- calls by method, and errors by code;
- notifications emitted and **dropped**, per stream;
- dispatch duration, p50/p99;
- connections currently open, with peer identity and age;
- process start time, and when the counters were last reset.

Bounded memory is a requirement, not an aspiration: this runs forever on a robot nobody
reboots. Fixed method set, fixed code set, no unbounded keys — a per-peer map keyed by pid
grows without limit on a board where a client reconnects in a loop.

**It lives in each service, and `robotctl` fans out.** No aggregator: invariant 1 says the
recovery path cannot depend on a component, and a stats collector would be one. The consequence
is that `robotctl` renders a partial picture when a service is down — which is correct, and is
itself the most important line in the output.

**Distinct from `robot.health`.** Health answers "is this robot fit to run"; the meter answers
"is this robot's IPC working, and who is using it". `LoopHealth` already reports tick rate and
must not be duplicated here.

### 9.1 The crash record

Counters that live only in memory answer nothing about the most common support case, which is
a service that is *no longer running*. A restart is frequently the event under investigation —
the updater restarts services on purpose (`updater-design.md` §7.2), and a crash loop is the
thing a fresh set of zeroed counters hides most completely.

So a bounded record goes on disk, written atomically, on a timer and on clean shutdown:

- **the last meter snapshot** — the counters as of the last write, so the previous life's
  numbers are readable after the next one starts;
- **the tail of the trace** — the last N events, at the default no-params rendering, kept in an
  in-memory ring at all times and flushed with the snapshot.

The ring is the part worth arguing for. The trace cannot be left on permanently, but the last
few hundred events *before* a service died are exactly what nobody can go back and collect,
and they cost a fixed allocation to keep. It is the flight recorder pattern, and a robot in
someone's home is the case it exists for.

Constraints, following `architecture.md` §8.2 — the same reasoning that keeps the update
history out of the journal:

- Under `/var/lib`, outside release directories, so it survives update *and* rollback
  (`updater-design.md` §5.7) and the updater never touches it.
- **One file per service, fixed maximum size**, overwritten rather than appended. This is a
  crash record, not a log; the moment it grows it becomes the per-service log file §3.2
  rejects.
- Written temp + `rename` + parent `fsync`, so a power cut during a write cannot leave a
  service unable to start.
- **Never carries params** (§6). It outlives the process and nobody opted into it.
- Its absence, its truncation and its staleness are all normal. Anything that reads it treats a
  missing or half-written record as "no information", never as an error.

The meter as served over IPC reports **both lives**: this process's counters, and the last
persisted snapshot with its timestamp. Two numbers, plainly labelled, is the same discipline
`robotctl version` applies to the running versus installed release — one number would be wrong
half the time and wrong in the direction that makes a broken robot look fine.

## 10. How the events get out

Two candidate transports, additive rather than alternative:

- **`tracing` on a dedicated target, first.** Journald already solves persistence, retention
  and offline reading; `RUST_LOG` already solves per-service enablement; no new socket, no new
  authorisation surface. This covers the whole trace use case on day one.
- **A `wire.subscribe` notification stream, later.** What a live TUI wants, and structurally
  the same thing `update.subscribe` and `robot.subscribe` already do. Deferred because it needs
  an authorisation decision of its own — a stream of *everything anyone sends* is strictly more
  sensitive than any single call on it, and it cannot inherit the "read-only calls are ungated"
  rule — and because it must not observe itself into a loop.

Both consume the same internal event, so the second is additive and neither blocks the other.

## 11. What this must never do

Stated as invariants because a monitor is exactly the kind of component that acquires them by
accident:

1. **Nothing may depend on it.** Disabled, absent, or broken, every service behaves
   identically. It is not on the recovery path and must never become a reason a robot cannot be
   recovered (§1.1, invariant 1).
2. **It must not block the control loop.** No allocation-heavy formatting and no I/O on
   `robotd`'s tick path; the disabled tap is a branch not taken, the enabled one hands off, and
   §9.1's flush is not on the tick (§1.1, invariant 3).
3. **It must not become a credential leak.** §6.
4. **It must not widen access.** Reading the meter is a read; nothing about monitoring
   justifies a new way to change the robot.
5. **It must not be able to fill a disk or evict a journal.** §8.1's two bounds and §9.1's
   fixed-size file are what make that true, and they are requirements rather than tuning.

## 12. Open questions

1. **Does `robotd` get a peer policy, and is `robot.enable` mutating?** §4.2. Surfaced by the
   extraction rather than caused by it; wants answering deliberately, not in a serve-loop diff.
2. **What does `mediad` change?** It is the first service that is both a client and a gateway,
   and the first to carry the API over transports with no `SO_PEERCRED` equivalent. Its arrival
   is the moment §7's rule 1 stops being theoretical, and it may be the moment "peer identity"
   needs a definition that is not a uid.
3. **How large is the trace ring, and is it configurable?** §9.1 asserts "a few hundred events"
   with no measurement behind it. On a board where `padd` alone produces 50 events a second,
   a ring sized in events holds a very different amount of *time* depending on what is
   connected — which may argue for sizing it in seconds, or for excluding rate-bearing
   notifications from it entirely.
4. **Does the shared loop become its own crate, or a module of `duck-ipc-proto`?** The latter
   is fewer moving parts; the former keeps `duck-ipc-proto` free of `tokio`, which matters if
   anything ever wants the types without the runtime.

## 13. Build order

Each step is useful alone, and the ordering is forced by which decisions get more expensive
with time:

1. **Extract the shared serve loop** (§4), behaviour-preserving per service, before any
   monitoring exists. Three implementations is the cheapest this ever gets, and every later
   step lands in one place instead of N.
2. **The correlation id and the `PROTOCOL_VERSION` bump** (§7). The only wire change here, and
   its cost grows with every client that exists.
3. **The tap and its four events, plus the `tracing` sink** (§5, §8). At this point the trace
   works and `robotd` stops being dark.
4. **The meter and the crash record** (§9), folded from the same events, with `robotctl`
   fanning out to render both lives.
5. **`wire.subscribe` and a live view** (§10) — only once something wants it, and with the
   authorisation question answered rather than assumed.
