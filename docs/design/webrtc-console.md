# The WebRTC client: from a test page to the robot's console

Status: landed · Date: 2026-08-25 · Owner: pierre

`mediad/webclient/index.html` proved the transport. This is what turns it into something a person
uses: served by the robot rather than by `python3`, found through the address `btd` already
broadcasts, and exercising the control surface `route.rs` actually permits.

Companion to [`remote-webrtc.md`](remote-webrtc.md), which owns the transport — signalling,
the session model, authorisation, and what a peer may call. Where the two touch, that page is the
owner and this one points at it.

**All four changes are in.** This page was written before the first line of them, so the decisions
are arguable rather than implied by a diff; it is kept because the alternatives are the part worth
being able to re-read. Where the code and this page could drift, the code is the answer:
`mediad/src/web.rs` serves the page, `mediad/src/producer.rs` fills in `meta`, `duckctl`'s `ip` and
`open` find the robot, and `mediad/webclient/index.html` is the console. §7 says what is left.

## 0. What was wrong with it

Not much, for what it was for. It answered "does a browser get video and a datachannel", and the
answer was yes. Six things are wrong with it as *a client*:

| | |
|---|---|
| it needs `python3 -m http.server` | a second tool and a second terminal, and the page is served from the laptop to talk to the robot |
| the URL is typed by hand | `ws://radxa-zero3.local:8443` — and `radxa-zero3` is the hostname on **every** board flashed from one image (`configd/src/main.rs` says so where it falls back to it), so two robots on one network collide |
| its comment block is mostly warnings | `file://` and Private Network Access, http not https — a header that had to be corrected twice already (`7f52a34`) |
| it exercises the protocol, not the robot | `route.rs` permits move, head, look, pose, mouth, do, sound, enable, init, relax, stop, subscribe, `tof.stream`, `pad.input`. The page offers `robot.health` and a JSON textbox |
| it is shipped nowhere | no `--include` names it, so a robot in the field has no client |
| it cannot say which robot it reached | the producer's `meta` is unset, so `list` returns an id and nothing else |

Every one of those is a step between a person and a working robot, and none of them is about
WebRTC.

## 1. `mediad` serves the page

**Landed** — `mediad/src/web.rs`, `--web-port`, default 8080.

An HTTP listener in `mediad`, one route, returning the page. `http://<robot>:8080/` and there is
nothing else to run.

This deletes four problems at once rather than one:

- **No `python3`.** The instruction becomes an address.
- **No URL to type.** The page defaults its signalling target to `ws://${location.hostname}:8443`
  — it was served by the robot, so it knows which robot it is talking to.
- **Private Network Access stops applying.** Chrome blocks a request from a *public or opaque*
  origin to a private address. A page served from `192.168.x` has a private origin, so the check
  that broke `file://` is not reached at all. The warning block in the header goes away because
  the failure it warns about cannot happen.
- **One version.** The page and the binary ship together (§1.2), so a client from a checkout can
  no longer be pointed at a robot from a release.

### 1.1 Which HTTP server

**`axum`. Decided.** It is already in `Cargo.lock` — `updater` uses it as a dev-dependency for its
test mirror — so the crate is known-good against this toolchain and the cross build.

The alternative was a hand-rolled HTTP/1.1 responder: this serves one file over one method, so it
is perhaps sixty lines. Sixty lines of hand-written request parsing bound to `0.0.0.0` is a parser
exposed to everyone on the network, written to avoid a dependency the build already resolves —
and `hyper` underneath `axum` is the most-read implementation of that parser in the language.

`RestrictAddressFamilies=AF_UNIX AF_INET AF_INET6` in `mediad.service` already permits the
listener; nothing in the unit changes.

### 1.2 The page is embedded, not installed

`include_str!("../webclient/index.html")`, served from memory.

The alternative — install it under `/opt/robot/daemon/current/webclient/` and read it at
request time — costs an `--include` line in *three* places (`_build-release.yml`, `dev.yml`,
`scripts/dev-push.sh`), which `xtask`'s packaging tripwires exist to keep in step and which is
exactly the list that has drifted before. It also puts a filesystem read behind a network request
in a unit running `ProtectSystem=strict`, and it makes "which page is this robot serving" a
question with two answers.

Embedding costs a rebuild to change a stylesheet. That is the right trade for a page that is part
of the daemon's interface.

### 1.3 Two ports, and nobody has to know there are two

`webrtcsink` owns the listener on 8443 (`run-signalling-server`, with only `-host` and `-port` to
say about it), so the page cannot be a route on it. Two ports: 8080 serves the page, 8443 stays
the signalling server. Nothing about the media path changes, which is the whole argument for doing
it this way first.

**Two ports is a fact about the implementation, and it must not become a step for a person.**
Three things keep it there, and they are requirements on the change rather than hopes:

- **One address is typed.** `http://<robot>:8080`. The 8443 is reached by the page's own
  JavaScript and never by a human. `duckctl open` (§2) removes even that one.
- **`mediad` fills the signalling URL in as it serves the page**, rather than the page carrying a
  constant. It knows the host the request arrived on and it knows its own `--port`, so the page
  gets the real answer — one `str::replace` over the embedded string at startup. This is what
  makes `--port` safe to change: there is no second place holding a stale copy of it.
- **The one failure two ports can produce is named in words.** If 8080 answers and 8443 does not
  — a firewall between, most plausibly — the page must say *the page came from this robot, but its
  signalling port did not answer*, not `websocket error`. That is the only route by which the
  second port can ever reach a person, and it should reach them as a diagnosis.

The one-port variant is to run the signalling server ourselves — upstream publishes it as a
library alongside the plugin — and point `webrtcsink`'s signaller at `ws://127.0.0.1:8080/ws`.
Then one `axum` server serves the page and the protocol on one origin. More work, and a second
copy of the protocol version to keep in step with the `.so` we ship, so not in the first change.

**But it is where this ends up, and here is why.** A browser gives a page served over plain http
to a private address no microphone, and depending on the browser no gamepad either — those are
secure-context APIs, and `http://192.168.1.42` is not a secure context (`http://localhost` is,
which is precisely why nobody has hit this yet). Two-way audio is in `remote-webrtc.md` §2. The
robot will need to serve TLS, and `webrtcsink`'s built-in server offers no way to configure a
certificate, while an `axum` server offers the ordinary one. So the sequence is: two ports now,
our own signalling server when audio or a browser gamepad is wanted, TLS on the same day.

Worth writing down now rather than discovering it while wiring a microphone.

## 2. Finding the robot: two commands, and one of them is already hand-rolled

**Landed** — `duckctl ip` and `duckctl open`, advertisement first and `net.status` behind it.

`btd` files the robot's IPv4 in its advertisement under company id `0xFFFF`, and `duckctl`
already parses it — `Address::At`, `Unassigned`, `Unsaid`, three answers rather than two, and
`scan` prints it today. So the work is a command, not a mechanism.

### 2.1 `duckctl ip`

The robot's address on stdout and nothing else, so `ssh microduck@$(duckctl ip)` works — the split
the tool already keeps, diagnostics on stderr and data on stdout.

**This is not a new idea in this repo; it is one that has already been written badly once.**
`scripts/dev-push.sh` needs exactly this and hand-rolls it: `resolve_board()` calls
`duckctl wifi status` and pipes the JSON through a six-line Python program embedded in the
shell script to pull `result.ip4`. That is the command, minus a home.

Reading the advertisement rather than calling `net.status` is better on three counts, all of them
visible in what `dev-push.sh` had to write around:

- **No connection, so no bond and no PIN.** `resolve_board` carries a whole failure branch for
  "the robot refused `net.status` — a wrong PIN, most often". An advertisement read cannot be
  refused, so that branch stops existing.
- **Seconds rather than tens of seconds.** `dev-push.sh` caches the address per robot precisely
  because "BLE discovery costs ten to twenty seconds"; a scan that stops at the first matching
  advertisement is about a second, because `duckctl` already polls until something appears
  rather than sleeping out `SCAN_TIME`.
- **It is not stale.** `btd`'s `reconcile_advertisement` re-reads `net.status` every `ADV_POLL`
  (5s) and re-advertises when the answer moves, so the advertisement *is* `net.status` with a
  five-second lag — well inside the window in which a new lease has already broken ssh.

**With one fallback, which is not optional.** A robot bonded to this Mac often stops advertising
the service to it — `duckctl`'s own scan tiers exist for that — so when no advertisement is
seen, `ip` connects and asks `net.status`, which is what `dev-push.sh` does today. Cheap read
first, call second. Without the fallback this command would fail on exactly the laptops that use
it most.

The three-way `Address` already carries the right failure text and this is where it pays:

- `Unassigned` — the robot has no network. The fix is `duckctl wifi connect`, and it has to be
  over BLE, because `net.connect` is refused over WebRTC by design ("a robot that has never seen a
  network cannot be configured over that network").
- `Unsaid` — a release from before `btd` advertised an address. The fallback answers anyway;
  updating makes it fast.

Its third caller is neither of the two above: `install-dev.md` opens by asking for "the board's
**IP address**", with the note that mDNS on this image is unreliable, and offers no way to get it.

### 2.2 `duckctl open`

Resolve, then open `http://<address>:8080/` in the browser. `--print` prints the URL instead, for
a machine with no browser or a script; `--port` for a robot started with a non-default
`--web-port`.

A command rather than a documented shell substitution, for one reason: **the port default should
live in exactly one place that a person never has to read.** `open "http://$(duckctl ip):8080"`
works, and it is the kind of line someone writes once and then looks up forever.

### 2.3 Not these

- **`duckctl url`.** That is `open --print`. A third command whose entire content is a port
  number.
- **A URL column on `scan`.** `scan` lists earbuds too, and the robot lines already carry the
  address. One note under the list pointing at `open` is enough.
- **Anything that has to connect.** Both commands are advertisement reads with a fallback. A
  command here that *required* a bond would be a different kind of command, and it would belong
  with `wifi` and `update` rather than with finding a robot.

### 2.4 The loop this closes, and the one follow-on

BLE provisions the network and then hands you the URL for it. The two transports stop being
alternatives and become a sequence — which is worth saying because `route.rs` refuses `net.connect`
over WebRTC deliberately, and this is the other half of that refusal.

The follow-on, in its own change rather than these four: `dev-push.sh`'s `resolve_board` becomes
`client --name "$1" ip`, deleting the embedded Python and the wrong-PIN branch. Separate because it
touches the push path, and because it should land after `ip` has been used by hand a few times.

`duckctl` is a stopgap for the phone app, so this stays at two commands and no new mechanism.
The durable halves are the ones that outlive it: `btd` broadcasting the address, and `mediad`
serving on a known port. The app will do the same two steps natively.

## 3. The tool is named after a transport it is about to stop being

**Decided and landed** — `duck-btctl` is `duckctl`, in its own crate. The rest of this section is
the reasoning, kept because the alternatives are the part worth being able to re-read.

`open` launches a browser at an http URL, and the name of the tool that does it said `bt`. Worth
pulling on, because it turned out to point at something larger than one command.

**`open` itself is not misplaced.** Its substance *is* Bluetooth: it scans for an advertisement to
learn where the robot is, and the `xdg-open` on the end is one line. `duckctl open` reads as
"use the radio to find the robot, then show me its console", which is exactly what it does — the
same way `wifi connect` is a wifi command whose whole mechanism is BLE.

**The name is still wrong, for a reason that arrives with `mediad` rather than with `open`.** There
is about to be a second transport to this robot, and the two reach *different* method sets by
design: `robot.move` is refused over BLE and permitted over WebRTC; `net.connect` is the other way
round. So a person will need both, and the thing they actually want is not a choice of binary —
it is a tool that knows which transport can serve a call and says so when neither can:

> `wifi connect` is Bluetooth-only, and this robot is not advertising.

That tool cannot be called `btctl`. And a tool called `btctl` quietly teaches everyone that
Bluetooth is the way to reach a robot, at exactly the moment it stops being the only one.

### 3.1 It is a move, not a rename, and that fixes two existing warts

A transport-agnostic client cannot stay `btd/examples/duck-btctl.rs`. It would need `btleplug`
*and* a WebSocket client, and `btd` is the BLE daemon — an example of it is the wrong home for
half of that. So the honest shape is its own workspace member, and two things that are awkward
today stop being awkward on the way:

- **The install line.** `cargo install --path btd --example duck-btctl` is odd enough that
  `dev-push.sh` carries a fallback for clones that never ran it — it shells out to
  `cargo run -q -p btd --example duck-btctl` instead. `cargo install --path duckctl` needs no
  fallback.
- **The example-that-is-really-a-product.** The reason it is an example is to keep `btleplug` off
  the robot, since an example's dev-dependencies never reach the shipped artifact. Its own crate
  keeps that for free, by not being a dependency of any daemon — the same guarantee, stated
  directly rather than as a side effect of where the file sits.

### 3.2 `duckctl`

It pairs with `robotctl` the way the two are actually used: `robotctl` on the robot, `duckctl` at
it. It keeps the `duck-` family the repo is already named for, and it is short enough to type
without an alias.

`duck` alone is better to type and is taken: Cyberduck ships a CLI by that name, which is
plausible on a Mac. `microduck` is the repository and too long for a command someone runs twenty
times an hour.

### 3.3 What was not rewritten

196 references across about twenty files, most of them prose. **The dated records keep the old
name.** `update-over-ble.md` and `install-path-gap.md` describe a moment — an update session, four
install-path bugs and what closed them — and rewriting a tool name into an account of something
that happened before the tool had it makes the record slightly false. Their *links* are repointed
so they still resolve; their prose is not.

`roadmap.md` sits in the same directory and is the exception, because it is not a record of a
moment: it describes the repo as it is now, down to a crate-by-crate layout table. A layout table
naming a directory that does not exist is simply wrong.

**No compatibility shim, no alias.** One user and one robot: a `duck-btctl` that still works is a
second name to keep in step and a reason for the old one to survive in someone's shell history for
a year.

### 3.4 What keeps it off the board

`cargo board --bins` — in `dev-push.sh` and in the release workflow — builds every default member
for aarch64. As an example it was excluded for free; as a crate it would be cross-compiled, which
means building a Bluetooth stack for a board that must never see one, on the release path.

**`default-members` in the workspace root, everything except `duckctl`.** One list, rather than
naming binaries at each of the two `--bins` call sites and keeping the two in step by hand — which
is the failure this repo keeps writing down. `--workspace` is unaffected, so CI lints and tests it
exactly as before.

## 4. The page becomes a console

**Landed** — `mediad/webclient/index.html` and the embedded `controller.js`, with no build step.

The permitted subset is large and almost none of it is reachable from the page. Reorganised
around what a person came to do:

| | |
|---|---|
| **header** | robot name, release, API version — from `hello` and `system.info`, sent automatically when the channel opens, not clicked |
| **video** | plus link quality from `getStats()`: bitrate, fps, loss, RTT. Today a stream that degrades is a picture that looks worse and a log that says nothing |
| **drive** | keys and an on-screen stick → `robot.move` at a fixed rate; drag on the video → `robot.look` |
| **controller** | a separate tab for stick sources, direction, sensitivity, deadzone, travel/speed limits and button skills; edits are staged until Save |
| **posture** | `robot.enable`, `init`, `relax`, `stop`, `shutdown` — confirm on the last two |
| **do / sound** | the `Do` and `Sound` enums as menus |
| **telemetry** | `robot.subscribe` at 2 Hz into a live panel: mode, health |
| **console** | the raw JSON box, the log, and the two refusal buttons — collapsed, because they prove the route table rather than drive the robot |

Three constraints on it:

- **The stop button must not read as an e-stop.** `route.rs` permits `robot.stop` on the grounds
  that this channel is reliable and the deadman already stops the robot when intents stop
  arriving, and then says in as many words that the UI should not imply it is a physical e-stop.
  A label, not a big red circle.
- **A version difference is a banner, not a locked door.** `hello` reporting skew says so and the
  page keeps working — same rule `duck-btctl` settled on in #102.
- **No build step.** The HTML and controller script are embedded in `mediad`, and the Space
  publisher copies both. There is no frontend framework or package installation.

### 4.1 Controller configuration

Open the console with `duckctl open`, connect, then select **Controller**. Select the stick
mode to edit its mappings; change a button menu to assign one of this robot's skills or
**No action** to disable it. **Save** writes the batch to `/etc/robot/robotd.toml`;
`padd` reloads it within a second. **Cancel** discards local edits. **Restore defaults**
stages removal of controller overrides and needs Save before it changes the robot.
Keyboard driving is suspended while this tab is visible. Existing unsaved edits survive
a lost connection, and inputs are disabled until the control channel returns.

API v42 adds `pad.config` (typed settings, effective values, defaults and choices from the
robot's parameter registry) and `pad.setConfig` (`changes` keyed by parameter path, with
JSON null meaning remove the override). The latter permits only controller parameters,
validates the whole batch and uses the terminal editor's atomic writer, preserving other
settings and comments. An invalid batch leaves the file untouched. Both methods use the
existing control-channel authorization and BLE PIN gate. The runtime must support v42;
older robots report the unsupported method in the editor. The parameter ownership and
mapping semantics live in [robotd-design.md](robotd-design.md#42-params).

The draft tests run with `node --test mediad/webclient/controller.test.cjs` (Node 22+,
no npm dependencies). Rust tests exercise typed edits, rejection of an invalid batch without
partial writes, default restoration, comment preservation and the embedded script's HTTP route.

Driving from the page is also the first real test of two claims `remote-webrtc.md` makes and
nothing has exercised: that the deadman stops the robot when a session drops (§6), and that
ordering over `control` is enough to keep `intents.rs` honest (§6 again). Both are cheap to
believe and expensive to be wrong about.

## 5. The robot names itself before the session starts

**Landed** — `mediad/src/producer.rs`.

`webrtcsink` takes a `meta` structure and the signalling server hands it to every peer in `list`
— the page already logs it and it is empty. Fill it from `configd`: name, serial, release,
`API_VERSION`.

Small, and it pays three times: the page can name the robot before starting a session, a client
that finds two producers can say which is which, and the rendezvous service in §7 needs exactly
this field to route on. Cheapest item here.

## 6. What it opens, but is not

A surface for server-side programs. The console's `axum` server makes a WebSocket beside the page
look like one more route, but a program that drives a duck is off the LAN by definition, and it
already has a transport: the rendezvous control lane
([`remote-access-design.md`](remote-access-design.md) §3.8). This listener serves `/frame` for a
still on the LAN and nothing more for programs.

## 7. Order, and what is left

Four changes, each of which stood alone and landed separately, in this order:

1. **Serve the page, with the signalling URL filled in as it is served.** Deleted the python
   instruction and the warning block. Small, and everything else was nicer on top of it. The API
   version is substituted the same way, for §4's banner — a literal in the page would be a second
   copy of `API_VERSION`, wrong on the day it is bumped and wrong in the direction that reports
   agreement.
2. **Producer `meta`.** Smaller still, and independent.
3. **`duckctl` gains `ip` and `open`.** Client-side only, touched no daemon.
4. **The console.** The large one, done last, on a page that was already reachable.

**Not done, and deliberately separate:** `dev-push.sh`'s `resolve_board` still hand-rolls this with
`duckctl wifi status` and six lines of embedded Python (§2.4). It becomes `duckctl ip`, which deletes
the Python and the wrong-PIN branch — separate because it touches the push path, and because it
should land after `ip` has been used by hand a few times.

## 8. Not doing

- **A gate on the console.** §4 of `remote-webrtc.md` owns that decision and nothing here changes
  its terms. A `--no-web` flag to turn the page off is worth having for the home case that section
  flags — one flag, not a mechanism.
- **A JS framework, a bundler, or `gstwebrtc-api`.** The page speaks the protocol by hand because
  a client that needs npm is a client nobody runs. Still true at four times the size.
- **Serving over TLS.** §1.3 says when, and why it was not now.
- **Teaching the advertisement a port.** Four bytes of IPv4 is what it carries; a robot on a
  non-default `--web-port` is a `--port` on `duckctl open`, not a wire-format change.
