# The App Path — `btd` and `configd`

Status: draft · Date: 2026-08-04 · Owner: pierre

How a phone configures a robot: wifi, name, reboot, version, and triggering an update.

Companion to [`architecture.md`](architecture.md), which owns the service split and the cross-cutting
contract. This covers the two services that landed together, because **they are one feature** and
every decision in one constrains the other: `btd` owns nothing, so `configd` exists; `configd`
serves a PIN, so `btd` can pair; a method routed in `btd` is a method `configd` must answer.

The robot side, that is. The phone at the other end of it is [`mobile-app.md`](mobile-app.md).

Sections marked **measured** were established on a Radxa Zero 3W rather than reasoned about.

**The path works end to end on hardware** (2026-08-05): a Mac discovered the robot, bonded,
read the API version, passed the PIN, and got a real `system.info` back — GATT discovery, chunked
NDJSON both ways, the PIN gate, the routing table and the hop into `configd` over its unix socket.
`configd` answers against a real NetworkManager too, reporting the live SSID and address.

What is **not** yet true: the link carries no encryption (§5.5), `net.connect` has not been driven
over BLE, and nothing has been tested with a phone rather than a laptop.

## 1. The shape

```
        phone ──BLE──▸ btd ──┐
     robotctl ──unix socket──┼──▸ configd ──D-Bus──▸ NetworkManager   (wifi)
    (mediad) ──WebSocket────┘                    └──▸ logind          (reboot)
                             │                    └──▸ config file     (name, PIN)
                             └──▸ updaterd  (update.*)
                             └──▸ robotd    (robot.health)
```

Two rules from `architecture.md` produce this and nothing else was free to vary:

- **§4.1: `btd` owns nothing.** If provisioning or config lived in the BLE service, every other
  service would depend on it, and an SDK would absurdly have to go through Bluetooth to set a
  robot's name.
- **§3.1: config must be reachable when `robotd` is dead.** Provisioning wifi is exactly what
  someone needs when the robot is broken, so config cannot live in the control daemon.

Between them there is no service left to put `net.*` and `system.*` in, hence a fifth one.

**Most of this work was not Bluetooth.** The API surface and the service owning it are needed
identically by the phone app, the SDK, `robotctl` and `mediad`'s remote gateway. `btd` is a thin
pipe over it — and the test of that claim is that adding the seven `net.*`/`system.*` methods cost
`btd` one line each in a routing table.

## 2. Wifi: NetworkManager, and why a board has to be migrated to it  · **measured**

`architecture.md` §3 chose NetworkManager. The board does not have it.

Armbian's headless image runs netplan + `systemd-networkd` + `wpa_supplicant`. Three findings from
the board made the choice again rather than inheriting it:

- **The D-Bus-enabled `wpa_supplicant` holds no interface.** `fi.w1.wpa_supplicant1` is claimed and
  idle (`Interfaces` is an empty array); netplan runs a *second*, `-c`-configured supplicant that
  owns `wlan0` and has no D-Bus at all. So driving `wpa_supplicant` directly would mean displacing
  netplan anyway — with none of NM's failure reporting.
- **netplan cannot report what a phone needs.** It is a config *generator*: no scan API at any
  layer, and `netplan apply` returns "config applied" rather than whether association succeeded.
  "Show me the networks" and "that password was wrong" are the two things a provisioning flow needs
  most, and it answers neither.
- **`RequiredForOnline=no` makes boot worse, not better.** Armbian ships a drop-in turning
  `systemd-networkd-wait-online` into `--any`: succeed when *any* networkd link comes online. Once
  wifi belongs to NM, networkd's only link is a usually-cableless ethernet port, so `--any` can
  never be satisfied. Marking that link not-required removes the only candidate and guarantees the
  failure. Masking the unit is the fix; `NetworkManager-wait-online` is the honest gate.

`scripts/migrate-network.sh` performs the migration once, and refuses to cut over until it has
copied the board's existing credentials into an NM profile — otherwise a headless board goes
offline with no way back. It arms a boot-time backstop that restores netplan and reboots if `wlan0`
has no address after 90s, which is the update system's boot-counter idea applied to a network
change.

### 2.1 `BadKey` is the whole point

`ConnectFailure::BadKey` is why NM was worth a migration. A rejected passphrase is the commonest
provisioning failure there is, and a client that cannot say so leaves the user with nothing to do.
NM reports it as device state reason 7; `configd` maps NM's reasons to `BadKey`, `NotFound`,
`Timeout`, `Unsupported` and `Other`, and an unmapped reason must never become `BadKey` — that
would send someone round a loop retyping a key that was already correct.

`configd` polls NM after `AddAndActivateConnection` rather than returning when activation *starts*,
because "config applied" is the answer netplan gives and the one we rejected.

### 2.2 Provisioning a new network, which is the point of the whole path

The scenario that justifies BLE: a board arrives somewhere new, the wifi it knows is not there, and
nothing on the network can reach it. Three properties make that work, and only the third needed
fixing.

- **The daemons do not wait for a network.** `btd` is `After=dbus.service bluetooth.service` and
  `configd` is `After=NetworkManager.service`, neither with `network-online.target`. A board with no
  reachable AP still comes up serving BLE.
- **A provisioned profile survives a reboot.** `AddAndActivateConnection` leaves a saved profile with
  `autoconnect` defaulting on, so rejoining is NM's business and `configd` stays out of the reconnect
  loop entirely.
- **A scan waits for the scan.**  · **measured** `RequestScan` returns when NM *accepts* the
  request, not when the radio has swept the channels, and NM prunes access points it has not seen
  recently — so while associated, the cached list often holds nothing but the AP the robot is already
  on. Reading it immediately answered with the *previous* scan: one network on the first call, eight
  on an identical second call. `configd` now waits for the `LastScan` property to advance, capped at
  10s, and treats a rate-limited request as "the cache is already fresh" rather than an error. For a
  client whose whole job is choosing a network in an unfamiliar place, "ask twice" was not a contract
  worth shipping.
- **The outcome is read from the activation, not the device.**  · **measured** The worst bug on this
  path. `connect` polled the *device* state, and a device stays `ACTIVATED` on the network it is
  already using while a new activation fails beside it — so `connect("Tehaupoo", psk: "lol")` for a
  network that was not even in range returned `{"outcome":"connected","ssid":"SFR-e994"}`, naming the
  network the robot had been on all along. Reporting success for a join that never happened is the
  worst answer available: a phone concludes the robot is provisioned and moves on.

  Now `AddAndActivateConnection`'s returned active-connection object is polled instead, the requested
  SSID is what gets reported back rather than whatever `status` says, an SSID the radio cannot see is
  refused up front as `NotFound`, and a failed activation deletes the profile NM added — otherwise
  autoconnect retries a known-bad key forever and `net.status` claims the network is `saved`.

  A hidden SSID is refused by that preflight too. Joining one needs `802-11-wireless.hidden` and a
  client that can say "this one is hidden", which the API has no shape for yet.
- **Re-provisioning replaces, it does not accumulate.**  · **measured** `AddAndActivateConnection`
  always adds, and NM tolerates two profiles carrying the same id. So the ordinary path — a
  passphrase mistyped on a phone, `BadKey`, then the right one — left the robot holding both, with no
  guarantee which NM would autoconnect with after the next reboot. `net.forget` made it worse by
  removing one of the two and reporting success. Saved profiles for an SSID are now enumerated as a
  *set* and deleted before a connect, and `net.forget` deletes all of them.

  Deleted before adding rather than after, because the alternative leaves duplicates whenever the add
  succeeds and the cleanup does not. If the add then fails, the SSID is left with no profile, which
  is the honest outcome for a configuration being replaced and is reported to the client.

  Note this disconnects the robot when the profile being replaced is the active one. Unavoidable —
  changing a key means re-associating — and a client on BLE is unaffected, which is the property the
  whole design rests on.

## 3. The GATT surface: a pipe, not an API

One service, **one characteristic**. A client reads it once for the robot's API version, writes
NDJSON request bytes to it, and subscribes to it for answers — the same JSON-RPC lines every other
transport carries. The read is not optional; see §5 for why it exists.

**The version it returns is for saying so, not for refusing.** `API_VERSION` says the two peers were
not built together; it does not say which calls stop working, and on this link it is usually none of
them. Nothing across it checks the number — `configd` gates no `net.*` or `system.*` call on a
version, `updaterd` requires no handshake before `update.status`, and `updaterd`'s `hello` no longer
refuses on it either. So a client that refuses on skew refuses calls the robot would have answered —
and it refuses them on the transport that exists for a robot with no network, where `net.connect` is
the way out of the skew. `duck-btctl` warns and proceeds, and an app should do the same: surface the
mismatch, let the call go, and report the JSON-RPC error if a method whose shape changed is actually
reached. Those errors name themselves — `METHOD_NOT_FOUND` for a route this release does not have,
`INVALID_PARAMS` for a parameter it does not know — which is what makes proceeding safe rather than
merely optimistic.

**No framing header.** The newline that already separates NDJSON messages is the frame delimiter in
both directions. That is safe rather than lucky: `serde_json` escapes a newline inside a string as
`\n`, so a raw `0x0A` never appears inside a serialised object — the same property that makes NDJSON
work on a unix socket. A length prefix would be a BLE-only dialect every client had to implement;
instead a phone does what `robotctl` does: write bytes, read until newline.

Reassembly is capped at 8 KiB, because that buffer is reachable by anyone in radio range.

**Alternatives, and why not:**

| | why not |
|---|---|
| Per-field characteristics (name, ssid, ip, connect…) | Browsable in a generic BLE app, but a second dialect of the same API: every field becomes a UUID plus `btd` code, and `net.scan` (a list) and `update.subscribe` (a stream) fit badly |
| Two characteristics, write and notify | The conventional shape, and written that way first. BlueZ reports a write and a subscription as *separate events*, so two characteristics must be matched across them by device address — guessing at an association that one characteristic gets by construction |

The cost of one characteristic is that it reads oddly in nRF Connect, where the same row is both.

### 3.1 The routed subset is the security boundary

BLE exposes a subset (§4.1). One table in `btd/src/route.rs` decides *whether* a call is
permitted, *which socket* answers it and *which connection to that socket* carries it. The first
two are the same question — a call is allowed exactly when the table names a service for it — and
the third is in the same table so that a new method cannot be added without answering it (§3.5).

**The match over `Call` is exhaustive on purpose.** Adding a protocol method fails `btd`'s build
until someone decides about it. A `_ => None` wildcard would be the safe default in the moment and
wrong over time — it would deny new methods silently, and the first symptom would be a phone app
missing a feature nobody remembered to route. This has already paid for itself once: the seven
`net.*`/`system.*` methods broke the build, as did `updaterd`'s equivalent match.

Refused, each for a reason:

| refused | why |
|---|---|
| `update.pin` | A robot pinned by a mistap refuses every later update and reports itself as up to date — the one failure here that looks like correct behaviour. `robotctl`, and a person who meant it |
| `update.resetToGolden` | Factory reset in all but name. Never over a radio |
| `robot.safeToRestart`, `robot.modelApi`, `robot.remoteSessionActive` | `updaterd`'s private questions to `robotd`; a phone reading them learns nothing it can act on |
| `system.pairingPin`, `system.setPairingPin` | **The load-bearing one.** A passkey an unpaired peer could read — or overwrite — would make pairing theatre. `btd` reads it over the unix socket instead |

**`update.rollback` and `update.select` were on that list and are not now.** They were refused on
the reasoning that the engine reverts a bad release itself, which it does — the one that fails its
health gate. That is not the case an owner reaches for a phone about, which is a release that
installs, passes its gate and then behaves *worse*: a policy that walks unsteadily rather than not
at all, a pad that stops reconnecting. Nothing reverts that but a person, and the person is holding
a phone and has no ssh. Both move to a release that has already run on this board, download
nothing, and are gated and auto-reverted like any other transition. `update.apply` was already
routed and is the more consequential of the three, so this widens what a peer in radio range can do
by close to nothing. `docs/project/update-over-ble.md` §2.4 is the decision and its trade-offs.

## 4. Authorisation: two layers, kept apart

| layer | mechanism | decides |
|---|---|---|
| 1 | socket mode `0660`, group `robot` | who may **connect and talk** |
| 2 | `allow_users` / `--allow-user` | who may make **mutating** calls |

Read-only calls skip layer 2 entirely, so support can inspect a robot it may not change.

Two layers because `btd` must be in the `robot` group to reach the sockets at all, and being in
that group must not amount to "may replace the firmware". Both services therefore grant change
authority to the **named service** — `allow_users = ["btd"]` in `updater.toml`,
`--allow-user btd` in `configd.service` — and both have a test refusing `robot` as a group.

**By name, never by uid.** `systemd-sysusers` allocates dynamically, so a number written into a
shipped config is correct on the board it was written for and wrong on the next one. Names resolve
at startup; an unresolvable name warns rather than aborting, because a robot missing an optional
service must still serve status.

`SO_PEERCRED` reports only a peer's **primary** gid, which is the trap here: `SupplementaryGroups=`
gets a process through the socket mode and no further. Missing that is what made every mutating
call over BLE return `PERMISSION_DENIED` while everything read-only worked — the worst shape for a
bug, because it reads as a mystery rather than a configuration error.

### 4.1 Privilege, and where the parser sits

`btd` is unprivileged; `configd` runs as root. That looks backwards and is not.

`btd` is the process parsing bytes from anyone in radio range. `configd` only ever sees typed JSON
arriving over a peer-credentialled local socket. **Putting the parser on the safe side of that
boundary matters more than hardening the dispatcher.**

`configd` needs root for a narrow reason: NM's connection-modify and logind's `Reboot` are both
polkit-gated, there is **no polkit on this image**, and systemd denies both to a session-less
non-root caller. The alternative was installing a JS policy engine to authorise two calls. Unlike
`robotd` it touches no hardware, so its unit sandboxes it properly — `ProtectSystem=strict`, one
writable path, `AF_UNIX` only, empty `CapabilityBoundingSet`. `CAP_SYS_BOOT` is deliberately absent:
logind performs the reboot and `configd` only asks, so a capability there would permit the unclean
`reboot(2)` this design exists to avoid.

If polkit ever arrives for another reason, `configd` should drop to a dedicated user plus two rules.

### 3.2 One session per subscription, and the bug that decided it  · **measured**

`btd` keeps one session — one reassembly buffer, one outbound queue, one authorisation state — and
the question was how long it lives. The first answer was "as long as the service", because BlueZ's
callback model gives a subscribe no peer identity and only ever holds *one* notify state per
characteristic, so per-peer sessions looked like machinery for a case that cannot arise. A stale
partial line seemed to cost at most one bad request.

It cost the *next* client instead, which is worse, and it took three symptoms on a board to see it:

| symptom | cause |
|---|---|
| a request answered, then the following one timing out | the outbound receiver was taken out of the shared slot by the first pump, so the second subscription had no pump: the reply was written to a channel nobody read |
| `":0,"result":{"authenticated":true}}` — a reply with its beginning missing | those orphaned chunks surfacing through a later notifier |
| `no robot found`, then the same command working | unrelated: a client-side scan taking one snapshot after a fixed sleep, so whether the advertisement fell inside that window was luck |

Only the third was a client bug. The first two are the same defect: **state that outlives the peer it
belonged to.** A disconnect is invisible in this model, so nothing reset it.

So the session is created when a central subscribes and discarded when it goes away — the reassembler
and the queue go with it, and a reconnecting phone starts unauthenticated, which is the behaviour §5.2
already claimed. Two details are load-bearing and both were wrong first:

- The pump waits on `notifier.stopped()` as well as the queue. Learning of a departure only from a
  failed notify needs a reply to send, so a client that disconnects while idle would hold the slot
  until a request arrived for nobody.
- Teardown clears the slot only if it still holds *its own* sender. A notify to a vanished central
  takes as long as BlueZ takes to give up, by which time a reconnecting central may have installed a
  newer session that a blind clear would kill.

The write path still refuses a write with no live subscription. Accepting one would be a lie: there
is nowhere to send the answer.

### 3.3 One thing the mobile app will hit: do not scan with a service filter  · **measured**

`duck-btctl` is a test tool and deliberately not much more — the real client is a phone app. But one of its
bugs is a property of CoreBluetooth rather than of the tool, so it is worth writing down before
someone rediscovers it on iOS.

`btd` advertises the service UUID and the hostname. Scanning with a service filter still finds the
robot only *sometimes*: **CoreBluetooth honours the filter strictly, and a bonded peripheral
frequently advertises with an empty service list.** Filtered, it is then never reported at all — not
"reported without services", absent. That presented as `no robot found` on one run and success on the
next, with nothing changed in between, and it survived a first fix because the name-based fallback
could only match peripherals the filtered scan had already returned.

The app should scan unfiltered and discriminate its own candidates, strongest evidence first:
advertised UUID, then a known name or a stored peripheral identifier, and treat "serves our
characteristic" as the only authoritative identity test — it is knowable solely after connecting. An
iOS app has a better third tier than `duck-btctl` does, `retrievePeripherals(withIdentifiers:)`, which
`btleplug` does not expose; storing the identifier after a first successful connection is the right
move there and removes the guesswork entirely.

Also worth knowing: a single snapshot taken after a fixed sleep is not enough. Advertising is
periodic and the adapter's view of a bonded peripheral comes and goes, so poll until a candidate
appears.

### 3.4 The robot has to advertise often enough to be caught  · **measured**

`duck-btctl` reported `no robot found` on roughly half its runs, and for a while that was read as flakiness
in the tool — or as the gamepad, whose LE link shares the radio. It was neither. `btd` registered its
advertisement without an interval, so BlueZ used the kernel's default of **1.28 s**, and a central
scanning at a low duty cycle does not catch a 1.28 s advertiser reliably.

Measured from a Mac scanning continuously for two minutes, counting arrivals per device:

| device | signal | arrivals in 120 s |
|---|---|---|
| smart plug | −66 dBm | 130 |
| beacon | −91 dBm | 159 |
| the robot | **−36 dBm** | **16** |

The robot was the strongest signal in the room and was heard an order of magnitude less often than
anything else in it, so range, interference and the client were all ruled out rather than argued
about. It arrived once every 7.5 s on average, with silences of 9 s, 14 s, 17 s and once 31 s; the
large gaps came out as near-integer multiples of 1.28 s, which is what identified the interval from
the arrivals. An eight-second scan landing in one of those silences finds nothing.

Two things follow for the app.

**A phone does not escape this by being a phone.** First connection is a scan, so onboarding hits the
same silences — and that is the moment a robot has to be findable. What §3.3's
`retrievePeripherals(withIdentifiers:)` buys is narrower than it looks: a stored identifier lets iOS
connect *without* a fresh sighting, so reconnection degrades to latency instead of failure. It does
nothing for the first connection, which is the one that matters most.

**The advertising interval is a property of the robot, so it is `btd`'s to get right**: 100-150 ms,
8-12× the default, which is what ordinary peripherals use. Not the 20 ms the spec allows — one antenna
carries this, the gamepad's LE link and wifi, and airtime spent shouting comes out of what the robot
is for.

`btd/examples/advwatch.rs` is the measurement, kept because the claim is only checkable by re-running
it: arrivals per device with signal strength, then the robot's silences.

**Confirmed on the board.** With 100-150 ms installed, the same two-minute watch from the same Mac:

| | arrivals in 120 s | mean spacing | worst silence | silences ≥ 8 s |
|---|---|---|---|---|
| 1.28 s, the default | 16 | 7.5 s | 30.8 s | 7 |
| 100-150 ms | 151 | 0.8 s | 3.8 s | **0** |

Nine times as often, and — the part that matters — nothing left within a factor of two of `duck-btctl`'s
eight-second window, so the failure it was diagnosed from cannot occur. The robot went from 34th of
106 devices heard to 7th of 74.

### 3.5 One connection per lane, because a daemon serves one request at a time

`btd` opens sockets to `updaterd`, `robotd` and `configd` on demand and keeps them for the session
(`btd/src/upstream.rs`). It held **one per service**, and both daemons behind it serve a connection
one request at a time: read a line, await the whole call, then read the next
(`updater/src/ipc.rs::handle_connection`, `configd/src/main.rs::handle`). So every call on a session
queued behind the slowest thing on that queue, and the two orderings an app reaches for first were
broken by it:

| the client does | what happened |
|---|---|
| `update.apply`, then `update.status` while it runs | the status line waited in a socket `updaterd` would not read for minutes. The client timed out having heard nothing, while the robot was fine and updating |
| `update.subscribe`, then `update.apply` | worse. `stream_progress` owns its connection until the peer goes away and never reads another request, so the apply was written into a socket nobody read: it never ran, never replied and never errored |

The second is the one to remember. An owner taps "update", the robot does nothing at all, and there
is no error anywhere to find — the request is sitting in a socket buffer.

So calls are grouped by **how long they hold a connection**, and each group gets its own connection.
The lane is decided in `route.rs` next to the permission and the service (§3.1), which is what makes
the exhaustive match cover it too: a new long-running method cannot be added without someone
choosing.

| lane | holds its connection for | calls |
|---|---|---|
| `Prompt` | as long as a lookup | `hello`, `update.status`, `update.log`, `update.listInstalled`, `robot.health`, `net.status`, `net.forget`, `system.*`, `pad.status`, `pad.forget` |
| `Slow` | seconds — the network, or a radio sweep | `update.check`, `net.scan` |
| `Operation` | as long as it takes, and changes the robot | `update.apply`, `update.rollback`, `update.select`, `net.connect`, `pad.pair` |
| `Stream` | forever, and answers nothing | `update.subscribe` |

Sharing a lane is queueing, and each grouping is one where that is the right answer: `updaterd`
single-flights mutations behind a file lock and answers `BUSY` for a second one, and two radio
operations at once on `configd` is not a thing to want. `update.check` is deliberately *not* on the
`Operation` lane — asked during an update it has an immediate answer, and queueing it would turn
`BUSY` into a spinner that resolves minutes later.

At most four sockets per service per session, and in practice two. A connection per call would be
the tidier model and needs `btd` to know when a call ended, which needs it to parse replies — it
never does (§3), and that property is what keeps the routed subset a transport rather than a second
implementation of the API.

## 5. Pairing: just-works, and a PIN the transport checks

A six-digit PIN, stored by `configd`, checked by `btd` before it serves anything. **Not** by the
Bluetooth bond — and that is forced by the spec rather than chosen.

### 5.1 Why BLE cannot carry a printed PIN  · **measured**

The first design had the robot answer BlueZ's passkey request with its stored PIN. On hardware, macOS
displayed *its own* random six-digit code and waited for someone to type it into the robot.

In LE passkey entry one side **displays** a passkey and the other **inputs** it, and the roles follow
from the IO capabilities each side declares. Implementing `request_passkey` declares "this device can
input", so macOS took the display role. A robot with no keyboard cannot fill that role.

The reverse is no better. With `DisplayPasskey` the robot takes the display role, but the **spec has
the displaying side generate the passkey at random** — BlueZ chooses it and hands it to the agent.
There is no way to make it present a value we stored, and a headless robot has nothing to display it
on anyway.

So a fixed, printed-on-the-robot PIN is not expressible in BLE passkey entry. Three options remained:

| | |
|---|---|
| Just-works only | Encrypted, unauthenticated, no PIN. Security is physical presence. What most headless BLE devices do |
| Out-of-band (QR) | Genuinely authenticated and genuinely per-robot. BlueZ's OOB support is thin and no phone app exists to drive it. A large lift for v1 |
| **Just-works plus an app-layer PIN** | **Chosen.** Pair for encryption; check the PIN in the session, where we define the rules |

### 5.2 How it works

Pairing is just-works: every agent handler is `None`, which `bluer` publishes as `NoInputNoOutput`.
The read on the RPC characteristic requires `encrypt_read`, which is what makes a central bond at
all — plain encryption, not `encrypt_authenticated_*`, because a just-works bond can never satisfy
the authenticated variants and demanding them would refuse every client.

Then `btd` serves nothing until the client sends `system.authenticate`. That call is answered by the
transport rather than forwarded, which is why the routing table has a third outcome (`Route::Local`)
alongside "forward" and "refuse". `hello` is the one other call allowed through unauthenticated,
because it reports only versions — the same thing the GATT read already tells an unauthenticated
client — and refusing it would leave a mismatched client unable to learn why nothing works.

Three details that are load-bearing rather than incidental:

- **The PIN is fetched from `configd` per attempt**, not cached, so `robotctl system set-pin` takes
  effect on the next try rather than the next reboot. A `configd` that cannot answer means the
  session is refused rather than admitted.
- **Compared as a string.** `042042` and `42042` are different secrets; a numeric parse would make
  them the same. There is a test for exactly that.
- **Three attempts, then the session closes.** A six-digit PIN is a million guesses over a link that
  is encrypted but not authenticated, so rationing is the only thing making brute force expensive:
  reconnecting costs a full BLE connect and bond. `attempts_remaining` comes back to the client so it
  can say "two left" rather than silently losing its connection.

### 5.3 What this is and is not worth

**The PIN crosses an encrypted-but-unauthenticated link**, so an attacker present *at the moment of
pairing* could capture it. That is the price of the trade, and it is the reason to prefer OOB later
if the threat model ever justifies it. What it buys over just-works alone is that a device which
merely bonds — trivial for anyone in range — still cannot do anything.

**The factory PIN is `000000` and is public in this repository.** Out of the box, therefore, this
proves physical presence and nothing more. `btd` logs a warning on every authentication with the
default, and `robotctl system pin` says so too. Security rests entirely on the PIN being per-robot,
which makes it a **provisioning obligation**: something must generate it, print it, and record what
was printed. That is `updater-design.md` §5.7's per-device state, the same slot that owes us a serial
number.

**No pairing window, and that is decided rather than deferred.** The robot is pairable whenever it
advertises. A per-robot PIN already carries what a window would add: knowing a printed PIN requires
physical access, and anyone who can read the sticker can pick the robot up. A button would add a
visible consent moment, a recovery path for a lost PIN, and defence in depth if a sticker is
photographed — none needed for v1, each additive later, since an enclosure with a button can gate
`set_pairable` without changing this design.

### 5.5 Encryption is currently off, and that is not settled  · **measured**

`encrypt_read` on the characteristic makes the read **hang** on macOS: CoreBluetooth issues the Read
Request, BlueZ refuses it for insufficient encryption, and nothing resolves it — no prompt, no
error, no retry. The client waits out its timeout against a working robot. With the flag off the read
answers instantly, so the requirement is the cause.

So `btd` currently runs with `--insecure-no-pairing` on the test board, and **the PIN crosses an
unencrypted link**. That is worse than §5.3 describes: it is not "encrypted but unauthenticated", it
is neither. Anyone in radio range during the exchange can read the PIN, and thereafter do anything a
client may do.

Unresolved, and the next thing to establish is whether a bond exists at all — `bluetoothctl info
<mac>` reporting `Paired: no` would mean no encryption can ever be established and the flag is a
symptom rather than the cause. Until that is known, moving the requirement to the write is a guess:
it would fail identically if there is no bond to encrypt with.

**The default is insecure, on purpose, for now.** The flag is `--require-pairing` and it is **off**.
A board installed from a release therefore serves an unencrypted link and works out of the box.

The alternative was tried first and rejected: with pairing required by default, a fresh install is
secure and **unusable** — every client hangs on the version read, because that is precisely the
configuration that breaks CoreBluetooth. Nothing is protected by a robot nobody can talk to, and the
project is far from shipping; between a default that cannot be used and one that can, development
tooling takes the usable one.

The cost is stated rather than hedged: **every robot running this has wifi credentials and a PIN
readable by a bystander.** `btd` logs a warning naming that at every start, so the choice stays
visible instead of becoming the thing nobody remembers. The old `--insecure-no-pairing` flag is
accepted and ignored, purely so a board carrying it in a drop-in does not fail to start on the update
that removed it.

This must be closed — the flag flipped, and defaulted on — before anything is handed to anyone. A robot whose provisioning secret is readable by a
bystander is not a robot you can hand to a stranger.

### 5.6 Open

- **Bond revocation.** Nothing un-pairs a phone; `bluetoothctl untrust` is the manual escape. Needs
  an API and a rule about who may call it — plausibly not BLE itself.
- **Rate limiting survives only within a session.** Three wrong PINs close the session, but nothing
  counts across reconnects, so a determined peer can retry indefinitely at the cost of a bond per
  three guesses. A per-address backoff in `btd` is the obvious next step and needs somewhere to keep
  that state across sessions.

### 2.3 The fake is the specification, and nothing checks NM against it

Worth stating plainly, because it now describes three bugs rather than one. In every wifi bug found on
the board, **`FakeNet` already had the correct behaviour and the NetworkManager implementation had
drifted from it**:

| behaviour | `FakeNet` | NM path, as shipped |
|---|---|---|
| an SSID the radio cannot see | `NotFound` | reported `connected`, naming a different network |
| a failed attempt | saves nothing | left a saved profile with the bad key |
| re-provisioning an SSID | replaces | stacked a second profile |

So the trait is not merely a testing seam; it is the only written form of the contract. The problem is
the direction of the check — the suite verifies the fake against the contract, and nothing verifies NM
against either. Both implement `Net`, so the same assertions *could* run against both; what stops it
is that the NM side needs a real NetworkManager and a real radio, which means a board rather than CI —
see `install-path-gap.md` §"What would close it", where a board is on-demand by design and deliberately
not a CI runner.

Until that exists, the honest summary is: `configd`'s wifi behaviour is tested, and the code that runs
on the robot is not. Every bug above was found by hand, on hardware, in the space of one session.

### 2.4 What has actually run on a board

Recorded because "built" and "works" were the same word in this document for too long, and four
"fixes" were verified against binaries that were never running.

Proven end to end, on a Radxa Zero 3 with a Mac as the client: discovery, connection, the version
read, PIN authentication, `system.info`, `net.status`, `net.scan`, the refusal boundary
(`robot.move`, `system.pairingPin`, `update.select` all refused with code 14 by `btd` itself),
`update.check` routed through to `updaterd`, an SSID the radio cannot see refused as `NotFound`, and
**a network the robot had never seen provisioned over BLE, joined, and rejoined by itself after a
reboot** — which is the scenario the whole path exists for. A rejected passphrase comes back as
`BadKey` carrying NM's reason 7, which is the answer a phone acts on: re-prompt for the password
rather than show a generic failure.

`net.forget` clears every profile for an SSID: five duplicate `kek` profiles, left behind by the
pre-fix binaries, went in one call. The whole `net.*` surface has now run against a real
NetworkManager.

Still false on a board: the link is unencrypted (§5.5). That is the only thing left between this path
and something that can be handed to someone.

## 6. Testing without a radio  · **measured**

The suite runs on a laptop with no hardware, no network, no D-Bus and no Docker, and that had to
stay true. Two seams make it so:

- **`configd`'s wifi is a trait** with an in-memory fake, as `duck-control` has `RobotIo`.
  `--fake-net` serves the whole `net.*` surface including a wrong-key failure on demand, which is
  awkward to provoke against a real access point.
- **`btd`'s radio is two channels, not a trait.** A `GattLink` trait would need an async `recv` and
  an async `send`, and the session loop waits on both at once — meaning associated types or a fight
  with the borrow checker inside a `select!`. A plain struct holding two `mpsc` channels says the
  same thing, and a test constructs one instead of implementing anything.

So the session tests drive a complete BLE conversation over real unix sockets: a refused call never
reaching the daemon, `robot.*` routing to `robotd` rather than `updaterd`, and every notification of
a subscription stream arriving through a 23-byte MTU.

`board-test.sh` covers what only appears on Linux: the socket modes, `--allow-user` resolving a
name, a group member reading, a non-member blocked by the socket mode, an unnamed member denied a
mutating call **and the refused change not having taken effect**, a rejected passphrase exiting 5
rather than 1, a PIN keeping its leading zero, and no passphrase in the log. Plus `btd --version`,
which is a real cross-link check: `btd` is the only binary pulling C beyond `zstd`, because `bluer`
links libdbus built from vendored source by `zig cc`.

`duck-btctl` (`cargo run -p btd --example duck-btctl`) is the phone's stand-in and the only way to exercise
the radio. An **example, not a binary**, so `btleplug` never reaches the robot; `btleplug` rather
than `bluer` because it must run on a developer's Mac. It reuses `btd::framing`, so the chunking is
genuinely the client half of the robot's own code rather than a reimplementation free to agree with
itself.

### 6.1 What is not tested

- Neither service has met a **real radio** or a **real NetworkManager**. Both type-check for
  aarch64; that is all.
- The **cutover** in `migrate-network.sh` runs only on a freshly flashed board. It was performed by
  hand once, step by step; the script was then re-run over the result to confirm the idempotent
  path. The first person to flash a board is the real test.
- **~73s before BLE answers.** `hci0` does not exist until `aic-bluetooth.service` attaches the
  AIC8800's UART, and `bluetooth.service` spends 26s blocked behind `dbus`. `btd` waits and retries
  rather than exiting — the same lesson as `robotd` waiting for the motor bus — but a phone app
  designed around instant discovery will be disappointed.

## 7. Costs accepted

- **Two D-Bus stacks in the artifact.** `btd` links libdbus through `bluer`; `configd` uses `zbus`.
  A few MB. `bluer` was chosen because a GATT server, advertising and a pairing agent are exactly
  what it exists for, against roughly 700 lines of hand-written `org.bluez` object plumbing. Worth
  revisiting if `bluer` grows a `zbus` backend.
- **A vendored libdbus** is ours to keep current rather than the distro's. Acceptable for a library
  reached only over a local socket by a daemon we wrote.
- **`btd` is deliberately absent from `on_apply`'s restart set.** It may be the *transport the update
  was requested over*: restarting it drops the connection carrying `update.subscribe`, and the phone
  that started the update never learns the outcome. Same reason `updaterd` does not restart itself
  mid-update. The cost is bounded rather than open-ended — the exclusion expires once the reply is on
  the wire, so the engine restarts `btd` 5 s later and the next `updaterd` start verifies that it
  happened (`restart-order.md` §1 and §5).

## 8. Next

Ordered by what blocks what, not by size.

### 8.1 Encryption — the blocker

§5.5 in full. The link is unencrypted **by default** — `--require-pairing` exists and is off, because
requiring it makes every client hang — so the PIN and every wifi passphrase cross in clear. Closing
this means making the secure configuration work *and* flipping the default; doing only the first
leaves every board insecure. One fact decides the fix and is not yet known: whether a bond exists at all (`bluetoothctl info <mac>` on
the robot). *Bonded but not encrypting* and *never bonded* need opposite repairs, and shipping the
wrong one leaves the problem in place while looking solved.

### 8.2 Telling robots apart — three people, three robots, one room  · **built**

Three friends unbox three robots. Each has to reach *theirs*, and until this landed none of them
could: every board flashed from one image advertised `radxa-zero3`.

This is a usability problem first. It has a security tail — a phone that picks the wrong robot can
authenticate to it and write its owner's wifi credentials into it, since the PIN is `000000` on all
of them — but the tail is not what makes it worth fixing. A robot you cannot pick out of a list is
one nobody can use, and that stands on its own. §8.1 is tracked separately and gates none of this.

#### What a robot is called

**Identity comes from the SoC serial**, read from `/proc/device-tree/serial-number` and reported by
`system.info` (`configd::identity`). Fused into the chip and handed over by the bootloader, so it
survives a reflash, survives swapping the radio module, and needs no provisioning step to exist —
which is what keeps a hand-flashed board working. It is also a plain file read: no root, no D-Bus,
and available immediately rather than after the ~73 seconds `hci0` takes to appear.

This is not a new idea in this repository, which is worth saying: `updater-design.md` §5.6 already
picked the same value as the update log's stable device ID, for the same reason — it works and needs
no provisioning step to obtain. Reusing it means the robot has one identity rather than two.

The Bluetooth adapter address was the better-looking candidate, because a peer already sees it at
the link layer, so a name derived from it leaks nothing new. It was rejected on evidence — see §8.6.

**The default name is `duck-` plus four hex characters of a SHA-256 of the serial**: `duck-c51b`.
Hashed rather than sliced, because nothing guarantees which part of a chip id varies between chips.
SHA-256 rather than `std`'s hasher, whose output is not stable across Rust releases — a toolchain
bump would rename every robot in the field, and nobody would ever connect the two events.

Four hex characters is 65 536 possibilities, so three robots in a room collide about once in 22 000
times. This is a default meant to be *distinguishable*, not a unique key.

The name travels in the **scan response**, not the advertisement, and now always will: flags (3),
the 128-bit service UUID (18) and the address field (8, see below) spend 29 of the 31 bytes a legacy
advertisement holds. Before the address it was 21, and a name of 8 characters or fewer could have
fitted alongside — `duck-c51b` is 9, one over, so in practice it never did. A scan response is a
second exchange a central can miss on its own, which is why a device reported with no name and no
services is a plausible robot rather than something to filter out.

#### The advertised name is the one the robot was given

`btd` asks `configd` what the robot is called and reconciles its advertisement against that answer
every few seconds. Before this it advertised `/etc/hostname` while `system.setName` wrote to a file
nothing ever read: renaming a robot changed nothing a phone could see, not even after a restart, and
three comments in the tree described the intended behaviour as though it existed.

The reconcile runs alongside the adapter watch inside one bring-up, so losing the radio ends both —
and the next bring-up asks again, which is what picks up a rename made while Bluetooth was down.

**A robot has two names, and both are set.** The advertisement carries a Local Name; the adapter
separately serves a GAP Device Name (`0x2A00`) that BlueZ takes from `Adapter.Alias` and defaults to
the hostname. Setting only the first meant a renamed robot advertised `duck-c51b` and answered
`radxa-zero3` to anyone who read the characteristic — and reading it is what a central does on
connecting. BlueZ then caches the answer over the advertised name, so on Linux a robot was
`duck-c51b` until first contact and `radxa-zero3` after it, and `--name duck-c51b` stopped finding
it; CoreBluetooth keeps both and reports `radxa-zero3 [duck-c51b]`. A phone's own Bluetooth settings
shows the GAP name, which is the case that matters most here and the one no tool in this tree could
see. `advertise` sets the alias alongside the advertisement, so every path that publishes a name
publishes both. A client that cached the old name keeps it until it is forgotten, which is a client
problem with a client fix.

Reconciled rather than event-driven, deliberately. `btd` forwards `system.setName` without reading
the reply — interpreting replies is what this daemon avoids — and re-asking the moment it forwards
one races the write it just forwarded. Polling is fewer moving parts and covers renames made through
`robotctl`, which never cross `btd` at all. `--name` pins the *name* for bench work — the reconcile
loop still runs, because the address below moves whether or not the name does. An unreachable
`configd` falls back to the hostname: `btd` is on the recovery path and has to come up when the rest
of the robot has not.

#### The advertisement carries the robot's IPv4 address

`duck-btctl scan` connects to nothing, which is what makes it the command to reach for when a robot
is unreachable — and it is why a listing can only report what an advertisement carries. So the
question a listing is most often read to answer, *where do I ssh?*, had no answer in it: the address
is in `net.status`, and reading that costs a connection, a bond and the PIN, per robot.

Four bytes of IPv4 go in a manufacturer-data field under company id `0xFFFF` — the id the Bluetooth
SIG reserves for internal and interoperability testing, which is the right one for a project that
has not been assigned one. Anyone may use that id, so **the field is not an identity check**: it is
read only from a device that also advertised the service UUID, which is the discriminator.

**The SSID is not in it and cannot be.** An SSID is up to 32 bytes on its own, against the 6 bytes
of payload the budget above leaves. It stays a `wifi status` question, and that is the one to ask
when the address in a listing says something surprising.

A robot with no address advertises `0.0.0.0` rather than dropping the field, so a listing can tell
three states apart: an address, a robot with no network, and a robot on a release from before this
existed, which broadcasts no field at all. Collapsing the last two would send the reader to check
wifi on a robot that needs an update.

The address is reconciled on the same tick as the name, every few seconds — far faster than a DHCP
lease moves, and the point is the other case: whoever just ran `wifi connect` is waiting to read the
address it produced. `configd` failing to answer keeps the last known address rather than clearing
it, or a `configd` restart would deregister and re-register the advertisement every tick with a
client watching the address blink. And if BlueZ ever refuses an advertisement carrying the field,
`btd` retries without it: an advertisement with no address is a robot someone can still reach, and a
refused one is a robot gone dark.

#### Provisioning can name a board, and does not have to

`provision.sh --name`, which `provision-board.sh --name` passes through, calls
`robotctl system set-name` at the end. Optional on purpose:
the derived default already distinguishes a board, so provisioning *improves* on a working name
rather than being what supplies one. That is what makes this survive a board flashed by hand, which
was the objection that ruled out assigning identity at provisioning time.

#### What the app should do

- **Store the serial as the key.** It is the only handle that survives both a rename and a change of
  Bluetooth address, and it is what a "favourite" or a user-chosen label should hang off.
- **Store the peripheral identifier too**, as the fast path — `retrievePeripherals(withIdentifiers:)`
  on iOS skips scanning entirely. It is a cache, not the identity: it dies with the address (§8.6),
  and the serial is what re-establishes which robot a favourite meant.
- **RSSI is a sort key, never identity.** Fine for putting the nearest robot first. Not evidence:
  signal through a body or a table reorders robots freely.

Once a robot has been connected to once, this is solved — the app goes straight back to it and shows
whatever the owner called it.

#### Still open

- **Which of three is mine, the very first time.** Three distinct names in a list is not the same as
  knowing which one is the duck in your hands. Bridging that needs a physical signal: something
  printed on the robot, an `identify` action, or proximity. Cheapest mitigation costs nothing and is
  not code — power one robot on at a time, and have the app say so.
- **`identify`** — make *this* robot nod, blink or chirp. Nothing in the tree drives an LED, a
  speaker or a screen, and motor control is refused over BLE by design (§3.1), so this is a missing
  device path rather than the policy question it looks like. When it exists, two decisions come with
  it: what it may actuate, and that it has to work *before* authentication, since requiring the PIN
  first is circular when aiming the PIN at the right robot is the problem.
- **A sticker, and a per-robot PIN to print on it.** §5.3 wants the PIN; both wait on hardware
  nobody has settled. Note that the PIN cannot be derived from the identity: the identity is
  published in an advertisement, so anything computed from it is public.
- **What a factory reset does.** Nothing clears `configd`'s config today — it survives updates and
  rollbacks by design, and `update.resetToGolden` reverts releases rather than config — so a
  provisioned name and a user rename are indistinguishable. A separate "factory name" slot only
  earns its keep once something can clear one without clearing the other.

#### What the tests do not cover

The identity and the derived name are ordinary unit tests. The scenario this exists for — three
robots advertising at once and a client choosing correctly — is not testable off a board, and wants
either three of them or a fake peripheral. Worth saying plainly rather than leaving the green suite
to imply otherwise.

### 8.4 PIN attempts across reconnects

§5.6. Three wrong PINs close the session; nothing counts across reconnects, so a peer retries
indefinitely at the cost of a bond per three guesses. Needs somewhere to keep per-address state.

### 8.6 The adapter address does not hold still — **not investigated**

Recorded so it is not rediscovered from scratch. Nothing here has been chased down, and it is
deliberately parked rather than open work.

One board reported two different Bluetooth addresses across sixteen boots, from `btd`'s own startup
line (`serving BLE ... address=`):

- `50:37:CD:16:2B:EC` — two `btd` starts within one boot, 10:04 and 10:05
- `50:37:CD:16:1B:92` — every boot after, 10:18 onward, fifteen of them

Between them: no reflash. Nothing done to the board but BLE connections through `duck-btctl` and gamepad
pairing. The top four bytes held; only the low two moved, which fits a driver that generates an
address once behind a vendor prefix and caches it rather than reading one out of the module. `configd`
never power-cycles the adapter — it only powers one on that is off — so pad pairing has no obvious
mechanism, which makes it the thing to test rather than the thing to blame.

Why it matters beyond a puzzling log line:

- **Every bond lives under `/var/lib/bluetooth/<address>/`**, so an address change orphans all of
  them. A paired gamepad and a bonded phone both stop working, and re-pairing looks like a fix for
  an unrelated fault.
- **iOS derives the peripheral identifier from the device address**, so an app's saved "my robot"
  breaks with it. This is why §8.2 has the app key favourites on the *serial*.

It is not an identity problem any more — §8.2 no longer depends on this value for anything — which is
why this can wait.

Cheapest next steps, when someone picks it up. `sudo ls -la --time-style=long-iso /var/lib/bluetooth/`
dates the moment the second address first appeared, since BlueZ names a directory after each adapter
it has seen. Then `bluetoothctl show | head -3` before and after `robotctl pad pair` settles the pad
hypothesis without a reboot. If the address turns out to come from a file, it can be pinned
deliberately — which would also make it survive a reflash.
