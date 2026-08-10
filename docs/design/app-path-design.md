# The App Path — `btd` and `configd`

Status: draft · Date: 2026-08-04 · Owner: pierre

How a phone configures a robot: wifi, name, reboot, version, and triggering an update.

Companion to [`architecture.md`](architecture.md), which owns the service split and the cross-cutting
contract. This covers the two services that landed together, because **they are one feature** and
every decision in one constrains the other: `btd` owns nothing, so `configd` exists; `configd`
serves a PIN, so `btd` can pair; a method routed in `btd` is a method `configd` must answer.

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

BLE exposes a subset (§4.1). One table in `btd/src/route.rs` decides both *whether* a call is
permitted and *which socket* answers it, because those are the same question: a call is allowed
exactly when the table names a service for it.

**The match over `Call` is exhaustive on purpose.** Adding a protocol method fails `btd`'s build
until someone decides about it. A `_ => None` wildcard would be the safe default in the moment and
wrong over time — it would deny new methods silently, and the first symptom would be a phone app
missing a feature nobody remembered to route. This has already paid for itself once: the seven
`net.*`/`system.*` methods broke the build, as did `updaterd`'s equivalent match.

Refused, each for a reason:

| refused | why |
|---|---|
| `update.select`, `update.pin` | Operator surgery, made with `robotctl` and a record of who did it — not a mistap in a phone UI |
| `update.rollback` | The engine reverts a bad release itself, so the phone needs no button for the ordinary case. Recovery mode (§8.2) should reopen this deliberately |
| `update.resetToGolden` | Factory reset in all but name. Never over a radio |
| `robot.safeToRestart`, `robot.modelApi`, `robot.remoteSessionActive` | `updaterd`'s private questions to `robotd`; a phone reading them learns nothing it can act on |
| `system.pairingPin`, `system.setPairingPin` | **The load-bearing one.** A passkey an unpaired peer could read — or overwrite — would make pairing theatre. `btd` reads it over the unix socket instead |

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

`btctl` is a test tool and deliberately not much more — the real client is a phone app. But one of its
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
iOS app has a better third tier than `btctl` does, `retrievePeripherals(withIdentifiers:)`, which
`btleplug` does not expose; storing the identifier after a first successful connection is the right
move there and removes the guesswork entirely.

Also worth knowing: a single snapshot taken after a fixed sleep is not enough. Advertising is
periodic and the adapter's view of a bonded peripheral comes and goes, so poll until a candidate
appears.

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
is that the NM side needs a real NetworkManager and a real radio, which means a board
(`install-path-gap.md`, option D) rather than CI.

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

`btctl` (`cargo run -p btd --example btctl`) is the phone's stand-in and the only way to exercise
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
- **`btd` is deliberately absent from `on_apply`'s restart set**, so it runs the old binary until
  the next reboot. It may be the *transport the update was requested over*: restarting it drops the
  connection carrying `update.subscribe`, and the phone that started the update never learns the
  outcome. Same reason `updaterd` does not restart itself (§8.3).

## 8. Next

Ordered by what blocks what, not by size.

### 8.1 Encryption — the blocker

§5.5 in full. The link is unencrypted **by default** — `--require-pairing` exists and is off, because
requiring it makes every client hang — so the PIN and every wifi passphrase cross in clear. Closing
this means making the secure configuration work *and* flipping the default; doing only the first
leaves every board insecure. One fact decides the fix and is not yet known: whether a bond exists at all (`bluetoothctl info <mac>` on
the robot). *Bonded but not encrypting* and *never bonded* need opposite repairs, and shipping the
wrong one leaves the problem in place while looking solved.

### 8.2 Telling robots apart — three people, three robots, one room

Not a refinement of the above; a second gap that happens to be discovered by the same scenario. Three
friends with three robots must each reach *theirs*, and today they cannot.

What the radio actually offers a phone right now:

- **The advertised name is the hostname.** `configd`'s `Store` falls back to `hostname()` when no name
  has been set, and `btd` advertises that as `local_name`. Every board flashed from one image
  advertises `radxa-zero3`.
- **There is no serial.** `system.info` returns `serial: null` — no per-device identity exists
  (`updater-design.md` §5.7 owns that gap).
- **The PIN is `000000` on all of them** (§5.3).

The three compound into something worse than a bad UX: a phone cannot merely *pick* the wrong robot,
it can **authenticate to it and reconfigure it**. Choosing your friend's robot from a list is a
mistake; being able to put your wifi credentials into it is a security failure. So this is a
prerequisite for shipping alongside encryption, not a nicety after it.

Directions, roughly in dependency order:

- **A per-device identity, assigned at provisioning.** Everything else hangs off this. The default
  advertised name should derive from it — `duck-7f3a` rather than `radxa-zero3` — so robots are
  distinguishable straight out of the box, before anyone has renamed anything.
- **Print it on the robot**, with the PIN. A sticker carrying name and PIN is what makes "connect to
  the one in front of me" a *check* rather than a guess, and it composes with §5.3's requirement that
  a shipped robot have a per-robot PIN rather than `000000`.
- **An `identify` action** — make *this* robot nod, blink or chirp — so a human confirms the right one
  before configuring it. Two decisions this needs, neither of them plumbing: motor control is refused
  over BLE by design (§3.1), so `identify` cannot simply be a move and needs its own narrowly-scoped
  action; and it has to work **before** authentication, because requiring the PIN first is circular
  when aiming the PIN at the right robot is the whole problem. Allowing an unauthenticated stranger in
  BLE range to make robots chirp is a real cost, and probably an acceptable one.
- **RSSI as a sort key, never as identity.** Useful for putting the nearest robot first in a list.
  Not evidence: signal strength through a body or a table reorders robots freely.

### 8.3 `API_VERSION` skew

`hello` should refuse only when the client is *newer* than the daemon —
`install-path-gap.md` covers it. Small, self-contained, and it has already cost an hour twice.

### 8.4 Derive the restart set from the release

`on_apply`'s unit list lives in the board's own `updater.toml`, so a release that adds a daemon never
restarts it and reports success anyway. `install-path-gap.md` §4 has the full account and the fix.

### 8.5 PIN attempts across reconnects

§5.6. Three wrong PINs close the session; nothing counts across reconnects, so a peer retries
indefinitely at the cost of a bond per three guesses. Needs somewhere to keep per-address state.
