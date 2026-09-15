# The phone app — what it is built from

Status: draft · Date: 2026-09-14 · Owner: pierre

The owner's app: put the robot on a wifi network, update it, sign it in, see whether it is well,
and ask it to do the thing it knows. What it is written in, what it borrows from the app we already
ship for another robot, and what the robot still owes it.

**Nothing here is built.** This is the approach, written down before the first line so the
decisions are arguable rather than implied by a repo.

Companion to [`app-path-design.md`](app-path-design.md), which owns the robot side — the GATT
surface, the routing table, the lanes, pairing, identity, and every open question about them. Where
the two touch, that page is the owner and this one points at it.
[`remote-webrtc.md`](remote-webrtc.md) owns the other transport, and §1.2 below is the line between
them.

## 1. The API is finished; the app is a client

`btd/src/route.rs` is the specification, and [`duckctl.md`](../robot/duckctl.md) is the closest
thing to a functional spec the app has — every command in it is a call the app can make, over the
same radio, with the same reply.

| screen | calls |
|---|---|
| pick a robot | the advertisement — name and IPv4 — then `hello` and `system.authenticate` |
| wifi | `net.status`, `net.scan`, `net.connect`, `net.forget` |
| health | `robot.health`, `system.services`, `system.info`, `system.logs` |
| update | `update.check`, `update.apply`, `update.subscribe`, `update.status`, `update.listInstalled`, `update.log`, `update.show`, `update.rollback`, `update.select` |
| account | `account.login`, `account.status`, `account.logout` |
| what it walks with | `robot.policies`, `robot.loadPolicy`, `robot.reloadPolicies`, `policy.check`, `policy.search`, `policy.fetch`, `policy.install`, `detector.check`, `detector.install` |
| skills and the pad | `robot.skills`, `robot.setSkill`, `robot.removeSkill`, `robot.do`, `pad.status`, `pad.pair`, `pad.forget`, `pad.bindings`, `pad.bind` |
| the robot itself | `system.setName`, `system.reboot` |

Anything it turns out to need that is not routed is a one-line change to `btd/src/route.rs` and a
decision about whether it belongs on a radio — which is the property §3.1 exists to give us, and the
app is the first client that will test it.

### 1.1 This is no longer four screens, and that is recent

When this page was first written the routed subset was wifi, update, health and a name, and the
sentence here was "that is the whole app". It is not any more. Over the last month `route.rs`
gained the account device flow, the Hub policy catalogue with an install, the skill table, the pad
bindings, `robot.do`, `system.logs`, and `update.rollback` / `update.select`. Each arrived with its
own argument on its own arm in that file, and the arguments rhyme: **ten metres of radio range means
whoever tapped it is looking at the robot, and the bond is PIN-checked** — so BLE is the transport
that best answers *who is watching*, which is what most of the refusals in that file actually turn
on.

The consequence for this page is not a longer table. It is that the app stopped being a settings
utility and became **the robot's own interface**, and everything below that reads as over-engineering
for four screens is not sized for four screens any more. It also means §7's list of what the robot
owes the app is shorter than it was, and §4's session layer matters more than it did.

What stays off the radio is the same shape it always was: teleop and anything streaming
(`robot.move`, `robot.subscribe`, `tof.stream`, `pad.input`), the pairing PIN, `update.pin` and
`update.resetToGolden`, and the joints (`robot.init`, `robot.relax`, `robot.stop`). The first group
belongs to the other transport; the rest belongs to a person at a terminal on the robot.

### 1.2 BLE, now that there is an alternative

The original argument here was that the app is Bluetooth-only because there is nothing else —
`mediad` was not built. That is no longer true. `mediad` ships, WebRTC is the default transport for
video and control, the robot serves its own console at `http://<robot>:8080/`, and
[`remote-access-design.md`](remote-access-design.md) reaches a duck from outside the LAN through a
Hugging Face account. So the app's transport is now a choice, and it should be made deliberately.

**BLE stays the app's channel, for the reason §2.2 of the robot-side page exists.** Settings that
work with no network is the whole point: a board arrives somewhere new, the wifi it knows is not
there, and nothing on the network can reach it. An app that needs the LAN to change a setting cannot
fix the setting that is keeping the robot off the LAN. That case does not go away because a second
transport exists — it is precisely the case the second transport cannot serve.

**The console is not a competitor; it is the other half.** It needs an address, which means it needs
a network, which means it needs whatever the app is for. And it is where video and driving belong:
`remote-webrtc.md` §5 says plainly why teleop is not BLE's, and a 20-byte notification budget is the
short version. A reasonable end state is an app that owns the robot over BLE and opens a WebRTC
session when there is a network to open it on — which is a second transport in the same app, not a
second app, because `route.rs` and `mediad::route` already read the same `Call` enum and the same
destination table.

Not now, though. **Carrying only BLE is what lets this be built in parallel with everything else**,
and M6's "a non-developer updates the robot from the phone" waits on nothing.

## 2. `reachy_mini_mobile_app` is a reference, not a base

Pollen ships a Tauri 2 app for Reachy Mini — React 19, MUI, `tauri-plugin-blec`, TestFlight and
Play Internal. It provisions a robot over BLE and then does everything else over the LAN and
Hugging Face.

We are not starting from it, and one of the two original reasons has weakened enough to be worth
restating rather than repeating.

**What has weakened.** The objection used to be that its state machine's terminal state is *a robot
appeared on the Hugging Face central listing*, and duck had no account link, no central signalling
and no Hub catalogue. Duck now has all three: `account.login` runs the device flow over the radio,
a Space is the rendezvous, and `policy.search` / `policy.install` are a Hub catalogue. So the shape
of their wizard is now a shape duck could have, and pretending otherwise would be reading a
month-old note as a fact.

**What has not.** Its BLE layer speaks a different dialect — four characteristics, commands as
strings, replies parsed by substring (`ERROR:`, `OK:`, and `ECHO:` for "your firmware is too old to
know this command"), which its own source calls a throwaway test surface. And its Bluetooth exists
to run **once**, which §4 is the reason we cannot inherit. Those two are structural, and the second
one decides the app's shape rather than merely its transport.

What is worth taking is the plumbing, which is where the expensive knowledge is:

| take as code | take as a rule | leave |
|---|---|---|
| `tauri-plugin-blec` wiring — but **not** their vendored `btleplug` patch, and not their npm bindings. The CoreBluetooth crash on connect it exists for (deviceplug/btleplug#397) is fixed upstream in `btleplug` 0.13, which `blec` 0.14 is built on; their 0.8 predates it. And the crate-to-npm lock-step they have to maintain never reaches a client driven from Rust, because the webview does not call BLE. `duckctl` is still on a `btleplug` that has the crash | scan unfiltered and discriminate your own candidates — their scan core and §3.3 arrived at this separately, which is the evidence it is real | TanStack Query: it caches server state over HTTP, and this app has none |
| the Android BLE runtime-permission handling, which is the reason to use `blec` over `btleplug` bare | poll for a candidate rather than taking one snapshot after a sleep (§3.4) | the setup-wizard state machine — §4 |
| edge-to-edge, `viewport-fit=cover`, safe-area insets, the portrait lock | every error carries the step it recovers to, so "try again" does not restart the flow | the string-matched error taxonomy: `configd` returns `BadKey` and `NotFound` as types, and a version skew names itself `METHOD_NOT_FOUND` or `INVALID_PARAMS` (§3) |
| the release workflow's shape — unsigned simulator `.app` and debug `.apk` on the release, signed TestFlight and Play Internal alongside | a BLE drop while the app is backgrounded is expected, not a fault: iOS tears the GATT link down | |
| the `x25519-hkdf-sha256-aesgcm` sealed-password scheme, if §8.1 lands on sealing rather than on the link layer — the wire format is documented on their side and there is a working client to test an implementation against | | |
| their App Store compliance and review notes, which are written once and cost a rejection to learn | a failed connect can look like a success and then hang at subscribe; force a fresh discovery by disconnecting and reconnecting | |

## 3. The protocol lives in Rust

`tauri-plugin-blec` exposes a Rust handler as well as a JS one, and it is the *same* handler — a
link opened from either side is drivable from both. So the app can depend on `duck-ipc-proto` for
the wire types and `btd::framing` for the chunking, drive the radio from Rust, and hand the webview
a few typed commands. React never parses a robot reply; it calls something that returns a
`NetStatus`.

This is the reuse that matters, and it is the one Reachy Mini could not have: **the app cannot
drift from the daemon.** A protocol change fails the app's build the same way it fails `btd`'s
routing table.

It is also the argument for Tauri over the alternatives, and not the usual one. Every cross-platform
toolkit gives you one codebase; only this one lets the client speak the server's own types.

**`duckctl` is the proof that it works, and the caveat that comes with it.** It is its own crate
now, and it depends on `btd` for exactly this reason — `framing` there is the *client* half of the
module the robot chunks with, so an asymmetry shows up as the tool not working rather than as two
implementations agreeing with each other. It runs on macOS, Linux and Windows because `btd` keeps
`bluer` behind `cfg(target_os = "linux")`, which is the same reason it would build for `ios` and
`android`.

The caveat: depending on `btd` for `framing` also pulls `tokio`, `clap` and `tracing-subscriber`
into the app for the sake of one module that needs none of them. Acceptable for a laptop tool, worth
a look before a phone binary. Splitting `framing` into its own crate — or putting it behind a
default-off feature — is a small change on the daemon side and the kind that is much cheaper before
there is a second consumer than after. §5 is where that decision lands.

**What that deletes, concretely.** Reachy Mini's transport writes a command, synchronously reads the
response characteristic, and — if the reply is the `OK: working` ack — waits for a notification
carrying the real payload. A fast reply can arrive *before* that read, which leaves the payload
orphaned in a backlog for the next command's wait to consume: that is how a wifi scan came to return
nothing while being handed the previous command's key-exchange object. The fix is a stale-backlog
purge at the top of every command.

None of that is reachable here. JSON-RPC ids match replies to requests by construction, one
characteristic means there is no write-to-notify association to guess at, and §3.2 already made the
robot discard a session with the peer it belonged to. Worth writing down so nobody helpfully ports
the workaround along with the transport.

**And two hazards the robot has already absorbed**, so the app does not re-derive them. Calls are
grouped into lanes by how long they hold a connection (§3.5), so `update.subscribe` followed by
`update.apply` is no longer an update that silently never runs — the client may issue them in any
order. And a reply larger than one burst no longer tears the notification session down (§3.6): `btd`
learns the MTU from the peer's own writes and sizes chunks from it, which is why `system.logs` over
a radio is a thing an app may offer. [`update-over-ble.md`](../project/update-over-ble.md) is the
record of both being found by driving the path, which is the argument for the spike in §6.

## 4. Bluetooth is the permanent channel, not a setup step

The one structural difference, and it decides the app's shape.

Reachy Mini's BLE code runs **once** — a wizard with a terminal state, after which the LAN takes
over and Bluetooth is never used again. Duck's BLE is where the robot's interface lives, for as long
as the robot exists (§1.1).

So this is not a wizard with a settings tab bolted on; it is an app with a first-run path.
Concretely, the app needs a session layer that reconnects and re-authenticates without saying so,
because §3.2 has `btd` discard the session when the central goes away and a reconnecting phone
starts unauthenticated. Every screen has to tolerate the link dropping underneath it and coming
back — including during an update, where `btd` is restarted five seconds after the reply goes out
([`restart-order.md`](restart-order.md) §1), and including while the user is off in a browser
approving a device code, which is why `account.login` answers with a code rather than a token.

The robot meets that layer halfway and it is worth knowing which half: `updaterd` keeps the latest
progress per component and replays it to a new subscriber, and `update.status` answers from a cached
snapshot during an update. So a phone that reconnects mid-update is not lost — it re-authenticates,
re-subscribes, and is told where things got to.

That layer is the app's actual core. It has no counterpart to copy.

## 5. Its own repo  · **done**

The daemon workspace co-versions because everything in it ships in one artifact. The app ships to
app stores on a different cadence and does not belong to that version line, so it got its own:
[`microduck-app`](https://github.com/pollen-robotics/microduck-app), private.

**The cost this section used to warn about is not real.** It said `duck-ipc-proto` and whatever
carries `framing` would be git dependencies on a private repo, so the app's CI would need a token.
`microduck` is public, so they are ordinary git dependencies and nothing needs a credential.
Publishing `duck-ipc-proto` to crates.io stays the other way and stays unnecessary.

`hf-robot-account` is the precedent for the other direction: it was extracted from this workspace to
crates.io because a second consumer wanted it. That is the shape to reach for if `framing` turns out
to want the same, and §3's caveat is the reason it might — the app now being that second consumer is
what makes the extraction answerable against a real list rather than a guess.

## 6. The spike, which has now run  · **measured** (2026-09-15)

One throwaway build: scan, connect, subscribe, `hello`, `system.authenticate`, `system.info`,
against the link the robot actually serves — `--require-pairing` **off** (§5.5). It lives in
[`microduck-app`](https://github.com/pollen-robotics/microduck-app) as its first commit.

**It ran green on an iPhone 17, iOS 26.6.2, against olducky.** Every step: the unfiltered scan found
the robot, the advertisement's name and IPv4 reached the first screen without connecting to
anything, and `system.info` answered — which is the step that matters, because it is refused before
`system.authenticate` and therefore proves the session gate opened rather than merely that the link
came up.

It also proves the thing this whole page is an argument for: the protocol ran in Rust on a phone,
through `duck-ipc-proto` and `btd::framing`, which are the daemon's own. `cargo check` for
`aarch64-apple-ios` is the static half of that and a green `system.info` is the other.

Two open questions went with it, one did not, and one was never this run's to answer:

- **Is iOS happy with one characteristic that reads, writes and notifies?** **Yes.** It reads oddly
  in nRF Connect (§3), and that stays a cosmetic cost rather than the real one it would have been if
  a phone stack had refused it.
- **What MTU does a phone actually negotiate?** **515**, which is the answer this run existed for.
  CoreBluetooth reports a `maximumWriteValueLength` of 512 and `btleplug` adds the three-byte header
  to get an ATT MTU; `btd` subtracts the same three, so a notification carries **512 bytes rather
  than 20**. §3.6 has what that does to the two replies it measured from a Mac. Short version:
  `system.logs` over a radio is a call an app may offer.
- **Does iOS hang on `encrypt_read` the way macOS does?** Still open, and deliberately not this
  run's question — §5.5 has where the separate attempt got to, and §8.1 why it no longer blocks
  building.
- **Does a stored peripheral identifier survive as §3.3 hopes?** Still open. The spike scans every
  time, so it never exercised the fast path. Cheap to answer whenever the session layer (§4) needs
  it, which is the first thing that will.

Thrown away now, as intended. What it bought is that the two questions that could have forced a
protocol change before any screen exists did not.

## 7. What the robot still owes the app

Each of these is owned by [`app-path-design.md`](app-path-design.md); the point of the list is that
they are all app-facing. It is shorter than it was — three rows were settled in the last month.

| | |
|---|---|
| **Encryption** — §5.5, §8.1 | Still the thing that has to close before a robot goes to anyone: an app whose job is writing a wifi passphrase cannot *ship* over a link that carries it in clear. It no longer blocks *building*, which is the change. §8.1 now has three candidate fixes and two of them need no bond, so the app is written against the open link either way and the choice can be made while it is being written |
| **`identify`** — §8.2 | Make *this* robot do something, so a list of three names becomes the duck in your hands. Two thirds solved already: `robot.do` is routed, and §8.2's claim that nothing in the tree drives a speaker is stale — there is a `sounds` crate, and `robot.sound` is refused rather than missing. What is left is §8.2's second requirement, that it work *before* authentication, since requiring the PIN first is circular when aiming the PIN at the right robot is the problem. Reachy Mini's answer is the shape to copy: `PLAY_SOUND` is an explicitly public, no-PIN command played straight from the scan list |
| **A per-robot PIN** — §5.3, §8.2 | Nothing generates, prints or records one, and it cannot be derived from the identity because the identity is advertised. Waits on hardware. Until then, see §8 |
| **Bond revocation** — §5.6 | "Forget this phone" is a settings-app staple and there is no API for it. `bluetoothctl untrust` is the manual escape |
| **Factory reset** — §8.2 | Nothing clears `configd`'s config, so a provisioned name and a user rename are indistinguishable |

Settled since this page was written, and named because the app inherits the answer rather than a
question:

| | |
|---|---|
| **Which of three robots is mine** — §8.2, **built** | The name comes from the SoC serial, so three boards from one image no longer all answer to `radxa-zero3`, and the advertisement carries the robot's IPv4 as well. The first screen can list names and addresses without connecting to anything. The store-the-serial / store-the-identifier / RSSI-is-a-sort-key rules in §8.2 are the app's, verbatim |
| **Version skew** — §3 | Decided and closed: the version read reports, it does not gate. `updaterd`'s `hello` no longer refuses on it either, and a method the peer does not have names itself `METHOD_NOT_FOUND`. The app warns and proceeds, like `duckctl` |
| **Going back from a bad release** — §3.1 | `update.rollback` and `update.select` were refused and are routed now. A release that installs, passes its gate and behaves *worse* is reverted by a person, and the person is holding a phone and has no ssh |

## 8. Open

- **The PIN screen, for v1.** The factory PIN is `000000` and public in this repository, so a PIN
  step today asks for a secret that protects nothing and adds a screen to the one flow where
  friction costs most. Sending it silently and adding the screen when there is a printed per-robot
  PIN to type is the honest reading of what §5.3 currently buys. Against that: a flow that never
  asked for a code is a flow people have to learn later, and the screen is where "wrong PIN, two
  attempts left" would live. Not decided.
- **Whether one app eventually serves both robots.** Not now — it would mean a second dialect in a
  shipping codebase, and the flows share only a transport. If it ever becomes a goal, the
  precondition is the *protocol* converging, not the UI: one client is worth having only if there
  is one API under it.
- **Where `framing` lives.** §3's caveat. A decision to make before the phone binary exists, not
  after.
- **The UI kit.** MUI is what the other app uses and it is heavy for four screens — and §1.1 is the
  reason to re-ask, because it is no longer four screens. Keeping it avoids designing a component
  system, which is the wrong place to spend. Worth one look before it is load-bearing, and not worth
  a second.
