# The phone app — what it is built from

Status: draft · Date: 2026-08-18 · Owner: pierre

The settings app: update the robot, put it on a wifi network, see whether it is well. What it is
written in, what it borrows from the app we already ship for another robot, and what the robot
still owes it.

**Nothing here is built.** This is the approach, written down before the first line so the
decisions are arguable rather than implied by a repo.

Companion to [`app-path-design.md`](app-path-design.md), which owns the robot side — the GATT
surface, the routing table, pairing, identity, and every open question about them. Where the two
touch, that page is the owner and this one points at it.

## 1. The API is finished; the app is a client

`duck-btctl`'s command list is already the specification, and
[`duck-btctl.md`](../robot/duck-btctl.md) is the closest thing to a functional spec the app has:

| screen | calls |
|---|---|
| pick a robot | the advertisement, then `hello` and `system.authenticate` |
| wifi | `net.status`, `net.scan`, `net.connect`, `net.forget` |
| health | `robot.health`, `system.services`, `system.info` |
| update | `update.check`, `update.apply`, `update.subscribe`, `update.listInstalled`, `update.log` |
| the robot itself | `system.setName`, `system.reboot`, `pad.status`, `pad.pair`, `pad.forget` |

That is the whole app. Anything it turns out to need that is not in that list is a one-line change
to `btd/src/route.rs` and a decision about whether it belongs on a radio — which is the property
§3.1 exists to give us, and the app is the first client that will test it.

### 1.1 BLE only, and that is a product statement

There is no LAN surface for a phone today: `mediad` is not built, and `robotd` speaks over a unix
socket. So the app is Bluetooth-only because it has no alternative.

It should stay that way after `mediad` lands. **Settings that work with no network** is the whole
reason §2.2 exists — a board arrives somewhere new, the wifi it knows is not there, and nothing on
the network can reach it. An app that needs the LAN to change a setting cannot fix the setting that
is keeping the robot off the LAN. Telepresence and apps belong on the network path when there is
one; wifi, update and health belong on the one that is always there.

It also means this can be built now, in parallel with `mediad`, and M6's "a non-developer updates
the robot from the phone" does not wait on M5.

## 2. `reachy_mini_mobile_app` is a reference, not a base

Pollen ships a Tauri 2 app for Reachy Mini — React 19, MUI, `tauri-plugin-blec`, TestFlight and
Play Internal. It provisions a robot over BLE and then does everything else over the LAN and
Hugging Face.

We are not starting from it, for reasons that are structural rather than about quality. Its BLE
layer speaks a different dialect — four characteristics, commands as strings, replies parsed by
substring (`ERROR:`, `OK:`, and `ECHO:` for "your firmware is too old to know this command"), which
its own source calls a throwaway test surface. Its state machine's terminal state is *a robot
appeared on the Hugging Face central listing*, and duck has no account link, no central signaling
and no Hub catalog. And its Bluetooth exists to run once, which §4 below is the reason we cannot
inherit.

What is worth taking is the plumbing, which is where the expensive knowledge is:

| take as code | take as a rule | leave |
|---|---|---|
| `tauri-plugin-blec` wiring, including the vendored `btleplug` patch and keeping the crate and the npm bindings in lock-step | scan unfiltered and discriminate your own candidates — their scan core and §3.3 arrived at this separately, which is the evidence it is real | TanStack Query: it caches server state over HTTP, and this app has none |
| the Android BLE runtime-permission handling, which is the reason to use `blec` over `btleplug` bare | poll for a candidate rather than taking one snapshot after a sleep (§3.4) | the setup-wizard state machine — §4 |
| edge-to-edge, `viewport-fit=cover`, safe-area insets, the portrait lock | every error carries the step it recovers to, so "try again" does not restart the flow | the string-matched error taxonomy: `configd` returns `BadKey` and `NotFound` as types |
| the release workflow's shape — unsigned simulator `.app` and debug `.apk` on the release, signed TestFlight and Play Internal alongside | a BLE drop while the app is backgrounded is expected, not a fault: iOS tears the GATT link down | |
| their App Store compliance and review notes, which are written once and cost a rejection to learn | a failed connect can look like a success and then hang at subscribe; force a fresh discovery by disconnecting and reconnecting | |

## 3. The protocol lives in Rust

`tauri-plugin-blec` exposes a Rust handler as well as a JS one, and it is the *same* handler — a
link opened from either side is drivable from both. So the app can depend on `duck-ipc-proto` for
the wire types and `btd::framing` for the chunking, drive the radio from Rust, and hand the webview
a few typed commands. React never parses a robot reply; it calls something that returns a
`NetStatus`.

This is the reuse that matters, and it is the one Reachy Mini could not have: **the app cannot
drift from the daemon.** `duck-btctl` already proves the shape — it reuses `btd::framing`, so its
chunking is genuinely the client half of the robot's own code rather than a reimplementation free
to agree with itself (§6). A protocol change fails the app's build the same way it fails `btd`'s
routing table.

It is also the argument for Tauri over the alternatives, and not the usual one. Every cross-platform
toolkit gives you one codebase; only this one lets the client speak the server's own types.

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

## 4. Bluetooth is the permanent channel, not a setup step

The one structural difference, and it decides the app's shape.

Reachy Mini's BLE code runs **once** — a wizard with a terminal state, after which the LAN takes
over and Bluetooth is never used again. Duck's BLE is where the settings live, for as long as the
robot exists (§1.1).

So this is not a wizard with a settings tab bolted on; it is a settings app with a first-run path.
Concretely, the app needs a session layer that reconnects and re-authenticates without saying so,
because §3.2 has `btd` discard the session when the central goes away and a reconnecting phone
starts unauthenticated. Every screen has to tolerate the link dropping underneath it and coming
back — including during an update, where `btd` is restarted five seconds after the reply goes out
(§7).

That layer is the app's actual core. It has no counterpart to copy.

## 5. Its own repo, and what that costs

The daemon workspace co-versions because everything in it ships in one artifact. The app ships to
app stores on a different cadence and does not belong to that version line, so it gets its own
repository.

The consequence to accept up front: `duck-ipc-proto` and `btd::framing` become a git dependency on
a private repo, so the app's CI needs a token. Publishing `duck-ipc-proto` is the other way and can
wait — a git dependency is reversible and needs no decision about what we are willing to support
in public.

## 6. The spike that comes first

One throwaway build, on a real iPhone and a real Android: scan, connect, subscribe, `hello`,
`system.authenticate`, `system.info` — with `--require-pairing` **on**.

Four open questions have the same answer:

- **Does iOS hang on `encrypt_read` the way macOS does?** §5.5 is the blocker and it is currently
  a fact about CoreBluetooth on a laptop. Nobody has asked a phone, and a phone is the client this
  is for. The answer decides whether the fix is moving the requirement to the write or something
  else entirely.
- **Are both platforms happy with one characteristic that reads, writes and notifies?** It reads
  oddly in nRF Connect (§3), which is a cosmetic cost; a phone stack refusing it would not be.
- **What MTU do we actually get?** Settings payloads are small, but `net.scan` returning a dozen
  networks is the one call that could feel slow, and it is the call someone stares at.
- **Does a stored peripheral identifier survive as §3.3 hopes**, through `blec`'s abstraction rather
  than `retrievePeripherals(withIdentifiers:)` directly?

Two days, thrown away afterwards. Either it de-risks the app or it says the protocol needs a change
before any screen is designed — and the second answer is far cheaper now than after a UI exists.

## 7. What the robot still owes the app

Each of these is owned by [`app-path-design.md`](app-path-design.md); the point of the list is that
they are all app-facing, and three of them are the app's first screen.

| | |
|---|---|
| **Encryption** — §5.5, §8.1 | The blocker. An app whose job is writing a wifi passphrase cannot ship over a link that carries it in clear. §6 above is how we learn what the fix is |
| **Which of three robots is mine** — §8.2 | The first screen of a settings app is a list, and a list does not say which one is in your hands. The zero-code mitigation — power one on at a time — belongs in the app's copy rather than in someone's head |
| **`identify`** — §8.2 | The real answer to the above, and a missing device path rather than a policy question. Two decisions come with it: what it may actuate, and that it must work before authentication |
| **A per-robot PIN** — §5.3, §8.2 | Nothing generates, prints or records one. Until something does, see §8 below |
| **Bond revocation** — §5.6 | "Forget this phone" is a settings-app staple and there is no API for it |
| **Factory reset** — §8.2 | Nothing clears `configd`'s config, so a provisioned name and a user rename are indistinguishable |
| **Version skew** — §3 | Already decided: the version read reports, it does not gate. The app inherits the rule rather than reinventing it |

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
- **The UI kit.** MUI is what the other app uses and it is heavy for four screens. Keeping it
  avoids designing a component system for a tool app, which is the wrong place to spend. Worth one
  look before it is load-bearing, and not worth a second.
