# The phone app — what it is built from

Status: built · Date: 2026-09-17 · Owner: pierre

The owner's app: stand the robot up, put it on a wifi network, update it, sign it in, drive it, see
whether it is well, and ask it to do the thing it knows. What it is written in, what it borrows from
the app we already ship for another robot, and what the robot still owes it.

It lives in **[`pollen-robotics/microduck-app`](https://github.com/pollen-robotics/microduck-app)**,
which is where the code and the operational half — building it, signing it, getting it onto a phone
— are. This page is the reasoning: why it is shaped the way it is, and which of those decisions cost
something.

Companion to [`app-path-design.md`](app-path-design.md), which owns the robot side — the GATT
surface, the routing table, the lanes, pairing, identity, and every open question about them. Where
the two touch, that page is the owner and this one points at it.
[`remote-webrtc.md`](remote-webrtc.md) owns the other transport, and §1.2 is the line between them.

## 1. The API is the specification; the app is a client

`btd/src/route.rs` decides what a phone may ask for, and [`duckctl.md`](../robot/duckctl.md) is the
closest thing to a functional spec the app has — every command in it is a call the app can make,
over the same radio, with the same reply.

| screen | calls |
|---|---|
| pick a robot | the advertisement — name and IPv4 — then `hello` and `system.authenticate` |
| the robot | `system.info`, `robot.health`, `system.services`, `system.setName`, `system.reboot`, `robot.init`, `robot.enable`, `robot.rebootMotors` |
| drive | WebRTC: video, `robot.move`, `robot.subscribe` |
| moves | `robot.skills`, `robot.do`, `robot.policies`, `robot.loadPolicy`, `policy.check`, `policy.search`, `policy.fetch`, `policy.install` |
| wifi | `net.status`, `net.scan`, `net.connect`, `net.forget` |
| update | `update.check`, `update.apply`, `update.subscribe`, `update.status`, `update.listInstalled`, `update.log`, `update.rollback`, `update.select` |
| logs | `system.logs` |
| account | `account.login`, `account.status`, `account.logout` |

Anything it turns out to need that is not routed is a one-line change to `route.rs` and a decision
about whether it belongs on a radio — which is the property §3.1 exists to give us. The app has now
tested that property five times: `update.rollback` and `update.select`, then `robot.init`, then
`robot.enable`, then `robot.rebootMotors`, each opened because using the app found the refusal
costing something the refusal's own reasoning had not accounted for.

### 1.1 It is the robot's interface, not a settings utility

The routed subset was once wifi, update, health and a name. It is now most of what a person does
with a duck, and the arguments on each arm of `route.rs` rhyme: **ten metres of radio range means
whoever tapped it is looking at the robot, and the bond is PIN-checked** — so BLE is the transport
that best answers *who is watching*, which is what most of the refusals in that file turn on.

That is why the machinery below is not over-engineering. A settings utility could get away with a
call and a spinner; an interface cannot.

What stays off the radio: continuous intent and anything streaming (`robot.move`, `robot.subscribe`,
`tof.stream`, `pad.input`), the pairing PIN, `update.pin` and `update.resetToGolden`, and the one
call whose only outcome is the floor — `robot.relax`. The first group belongs to the other
transport; the rest belongs to a person at a terminal on the robot.

`robot.rebootMotors` was in that last group and is not any more, and the pair is worth keeping
straight: relaxing a robot that is holding itself up *produces* the fall, while rebooting a servo
that has latched an overload recovers a robot which stopped holding itself some time ago — and
until this was routed, the app could name the tripped joint on `robot.health` and offer nothing but
ssh. The arm in `route.rs` carries the reasoning.

### 1.2 Two transports, and they are not rivals

Everything above except `drive` goes over BLE. **BLE is the app's channel for the reason §2.2 of the
robot-side page exists**: settings that work with no network. A board arrives somewhere new, the
wifi it knows is not there, and nothing on the network can reach it — an app that needed the LAN to
change a setting could not fix the setting keeping the robot off the LAN.

`drive` needs a network by definition, and gets the address to use from `net.status` **over the
radio**. That is the two working as one thing rather than two: the transport that always works tells
you how to reach the one that is fast. The robot's own console is not a competitor either — it needs
an address, which means it needs a network, which means it needs whatever the app is for.

## 2. `reachy_mini_mobile_app` is a reference, not a base

Pollen ships a Tauri 2 app for Reachy Mini — React 19, MUI, `tauri-plugin-blec`, TestFlight and
Play Internal. It provisions a robot over BLE and then does everything else over the LAN and
Hugging Face.

Its BLE layer speaks a different dialect — four characteristics, commands as strings, replies parsed
by substring (`ERROR:`, `OK:`, and `ECHO:` for "your firmware is too old to know this command"),
which its own source calls a throwaway test surface. And its Bluetooth exists to run **once**, which
§4 is the reason we cannot inherit. The second decides this app's shape rather than merely its
transport.

What is worth taking is the plumbing, which is where the expensive knowledge is:

| take as code | take as a rule | leave |
|---|---|---|
| `tauri-plugin-blec` wiring — but **not** their vendored `btleplug` patch and not their npm bindings. The CoreBluetooth crash on connect the patch exists for (deviceplug/btleplug#397) is fixed upstream in `btleplug` 0.13, which `blec` 0.14 is built on; their 0.8 predates it. And the crate-to-npm lock-step they maintain never reaches a client driven from Rust, because the webview does not call BLE | scan unfiltered and discriminate your own candidates — their scan core and §3.3 arrived at this separately, which is the evidence it is real | TanStack Query: it caches server state over HTTP, and this app has none |
| the Android BLE runtime-permission handling, which is the reason to use `blec` over `btleplug` bare | poll for a candidate rather than taking one snapshot after a sleep (§3.4) | the setup-wizard state machine — §4 |
| edge-to-edge, `viewport-fit=cover`, safe-area insets, the portrait lock | every error carries the step it recovers to, so "try again" does not restart the flow | the string-matched error taxonomy: `configd` returns `BadKey` and `NotFound` as types, and a missing method names itself `METHOD_NOT_FOUND` |
| their App Store compliance and review notes, which are written once and cost a rejection to learn | a BLE drop while the app is backgrounded is expected, not a fault: iOS tears the GATT link down | |
| the `x25519-hkdf-sha256-aesgcm` sealed-password scheme, if §8.1 of the robot page lands on sealing rather than on the link layer — the wire format is documented on their side and there is a working client to test an implementation against | a failed connect can look like a success and then hang at subscribe; force a fresh discovery by disconnecting and reconnecting | |

## 3. The protocol lives in Rust

`tauri-plugin-blec` exposes a Rust handler as well as a JS one, and it is the *same* handler — a
link opened from either side is drivable from both. So the app depends on `duck-ipc-proto` for the
wire types and `duck-ble` for the chunking and the GATT UUIDs, drives the radio from Rust, and hands
the webview typed commands. React never parses a robot reply; it calls something that returns a
`NetStatus`.

This is the reuse that matters, and it is the one Reachy Mini could not have: **the app cannot drift
from the daemon.** A protocol change fails the app's build the same way it fails `btd`'s routing
table — which has happened twice in anger, both times catching a field the app would otherwise have
learned about from a robot at a bad moment.

It is also the argument for Tauri over the alternatives, and not the usual one. Every cross-platform
toolkit gives you one codebase; only this one lets the client speak the server's own types.

**`duck-ble` exists because of this app.** `gatt`, `framing` and `adv` were `btd`'s, and each was
already marked as wire contract where it lived — `adv` says outright that a decoder written
separately in the client would agree only with itself, which this app proved by hand-rolling one
before the crate existed. Extracting them stopped a phone binary carrying `clap` and
`tracing-subscriber` for the sake of one module, and stopped `duckctl` compiling a Linux daemon — and
on Linux, `bluer` and a vendored libdbus — to reach three platform-independent files.

### 3.1 Where the media is, and why that is not a hole in this

The one place the protocol is not in Rust is the WebRTC datachannel, and only halfway. A peer
connection and a `<video>` are the webview's by nature; there is no Rust equivalent that would get
frames onto a phone screen. But the datachannel carries the *same* JSON-RPC, so hand-writing those
envelopes in TypeScript would reintroduce exactly the drift this section is arranged to prevent.

So the webview moves the bytes and Rust decides what they are: every line is built by
`proto::Request` from a `proto::Call`, and the app's signalling module contains no method name at
all. A `{vx, vy, vyaw}` object literal in a `.ts` file would keep compiling and start steering
wrong.

### 3.2 What that deletes, concretely

Reachy Mini's transport writes a command, synchronously reads the response characteristic, and — if
the reply is the `OK: working` ack — waits for a notification carrying the real payload. A fast
reply can arrive *before* that read, which leaves the payload orphaned in a backlog for the next
command's wait to consume: that is how a wifi scan came to return nothing while being handed the
previous command's key-exchange object. The fix is a stale-backlog purge at the top of every
command.

None of that is reachable here. JSON-RPC ids match replies to requests by construction, one
characteristic means there is no write-to-notify association to guess at, and §3.2 of the robot page
already made the robot discard a session with the peer it belonged to. Worth writing down so nobody
helpfully ports the workaround along with the transport.

## 4. Bluetooth is the permanent channel, not a setup step

The one structural difference, and it decides the app's shape.

Reachy Mini's BLE code runs **once** — a wizard with a terminal state, after which the LAN takes over
and Bluetooth is never used again. Duck's BLE is where the robot's interface lives, for as long as
the robot exists.

So this is not a wizard with a settings tab bolted on. The app has a session layer that reconnects
and re-authenticates without saying so, because §3.2 has `btd` discard the session when the central
goes away and a reconnecting phone starts unauthenticated. Every screen tolerates the link dropping
underneath it and coming back — including during an update, where `btd` restarts five seconds after
the reply goes out ([`restart-order.md`](restart-order.md) §1), and while the user is off in a
browser approving a device code, which is why `account.login` answers with a code rather than a
token.

The robot meets that layer halfway: `updaterd` keeps the latest progress per component and replays
it to a new subscriber, and `update.status` answers from a cached snapshot during an update. So a
phone that reconnects mid-update re-authenticates, re-subscribes, and is told where things got to.

**That layer is the app's actual core, and it turned out to be testable.** §8.2 of the robot page
says the scenario worth testing is not testable off a board — true of three robots in a room, and
not true of a link that drops. `tauri-plugin-blec` ships an in-process mock that drops links and
hangs operations on demand, so the reconnect, the re-authentication and the re-subscription are
ordinary tests. They have caught three real defects: a subscription sent twice on the call that
opened the link, a reconnect that could not find its own robot because the discovery cache had
emptied, and a `Down` event for a link that was never up.

## 5. Its own repo

The daemon workspace co-versions because everything in it ships in one artifact. The app ships to
app stores on a different cadence and does not belong to that version line, so it has its own.

`microduck` is public, so `duck-ipc-proto` and `duck-ble` are ordinary git dependencies and nothing
needs a credential. The dependency deliberately tracks `main` rather than a pinned revision: a
protocol change that breaks the app's build is the signal this arrangement exists to give.

## 6. The version floor, which is the opposite of the daemon's rule

**The app refuses a robot below `API_VERSION` 31 — robot 0.14 — at the handshake.**

This is deliberately unlike the rule the daemon follows. `app-path-design.md` §3 is emphatic that a
version difference reports and never refuses, and that is right for the daemon: it is the transport
that exists for a robot with no network, where `net.connect` is the way out of the skew.

The app made a different trade, and the trade is what it buys every screen. One floor, checked once,
means a screen can treat a field as present, a method as routed and a reply as the shape the daemon
defines — instead of each one carrying its own guess about what an older robot might have sent.
Finding out at the handshake also beats finding out four screens later from a method that does not
exist.

**The cost is real and is in the refusal's own words**: a robot below the floor cannot be updated
*from the phone*, because the phone will not talk to it. `duckctl update apply` reaches it over the
same radio, and the message says so.

A robot *newer* than the app is served. The protocol only gains fields, and a client that refused
what it did not recognise would need updating in step with every robot.

## 7. What the robot still owes the app

Each of these is owned by [`app-path-design.md`](app-path-design.md); the point of the list is that
they are all app-facing.

| | |
|---|---|
| **Encryption** — §5.5, §8.1 | The thing that has to close before a robot goes to anyone: an app whose job is writing a wifi passphrase cannot *ship* over a link that carries it in clear. It does not block building — §8.1's three candidate fixes include two that need no bond, so the app is written against the open link either way |
| **`identify`** — §8.2 | Make *this* robot do something, so a list of three names becomes the duck in your hands. Two thirds solved: `robot.do` is routed, and §8.2's claim that nothing drives a speaker is stale — there is a `sounds` crate and `robot.sound` is refused rather than missing. What is left is that it must work *before* authentication, since requiring the PIN first is circular when aiming the PIN at the right robot is the problem. Reachy Mini's `PLAY_SOUND` — explicitly public, no PIN, played from the scan list — is the shape to copy |
| **A per-robot PIN** — §5.3, §8.2 | Nothing generates, prints or records one, and it cannot be derived from the identity because the identity is advertised. Waits on hardware |
| **Bond revocation** — §5.6 | "Forget this phone" is a settings-app staple and there is no API for it |
| **Factory reset** — §8.2 | Nothing clears `configd`'s config, so a provisioned name and a user rename are indistinguishable |
| **A pad that reconnects while a phone is connected** — [`pad-minimal-pairing.md`](../project/pad-minimal-pairing.md) fault 3 | The controller will not initiate as central while `btd` holds a peripheral link, so a bonded pad switched on during a session simply sits there. No daemon fix: the obvious one is the call that fails. The app could yield its link for twenty seconds on request; until then the answer is to turn the pad on first |

Settled by using the app, and named because each was a refusal whose own reasoning did not survive
contact with the thing it was refusing:

| | |
|---|---|
| **Standing the robot up** — #301, #303 | `robot.init` was refused because it "wants the person doing it to be looking at the robot rather than at a screen", which is the argument `route.rs` makes *for* routing `robot.do` and `policy.install`. `robot.enable` was refused as teleop, which it is not — one request, not fifty a second. Without both, the app could show a robot everything about itself and not make it move, and the way out was to go and find the gamepad |
| **A notification carries 512 bytes** — §3.6, #292 | Found by this app and reproduced with `duckctl`: `btd` sized replies at `mtu - 3`, which against a CoreBluetooth MTU of 517 is two bytes more than a characteristic value may hold. The central kept the first 512 and dropped the rest, silently, so `{"security":…}` arrived as `{"serity":…}` — or, when the lost pair contained the newline, as a call that never answered |
| **Which of three robots is mine** — §8.2 | The name comes from the SoC serial and the advertisement carries the IPv4, so the first screen lists names and addresses without connecting to anything. The app keys on the serial and treats the address as a cache, which is §8.2's rule verbatim: a remembered address reused by another board is refused by name rather than silently adopted |

## 8. Open

- **The PIN screen.** The factory PIN is `000000` and public in this repository, so a PIN step today
  asks for a secret that protects nothing and adds a screen to the one flow where friction costs
  most. The app sends it silently. The screen appears when there is a printed per-robot PIN to type,
  and that is also where "wrong PIN, two attempts left" would live.
- **The gamepad, from the phone.** `pad.pair`, `pad.forget`, `pad.bind` and the skill-table edits
  are routed and unreached. `route.rs` says BLE is what `pad.bind` exists for — whoever is holding
  the robot is holding the pad — and fault 3 above is why pairing from the phone is harder than the
  routing suggests. Since v38 `pad.pair` answers at once and the robot runs the session, retries
  and quacks by itself; the app follows `pad.status` by session number. On a `weird-ble` board the
  session stops `btd`, so the phone loses its link a second after asking and reads the outcome when
  it reconnects.
- **Whether one app eventually serves both robots.** Not now — it would mean a second dialect in a
  shipping codebase, and the flows share only a transport. If it becomes a goal, the precondition is
  the *protocol* converging, not the UI.
- **The UI kit.** There is not one: the app is about three hundred lines of CSS. MUI was the
  obvious candidate and was not taken, and nothing so far has wanted it. Worth one look if a screen
  ever needs a component a stylesheet cannot give it, and not worth a second.
