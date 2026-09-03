# Remote access — an account, and a rendezvous behind it

Status: draft · Date: 2026-09-02 · Owner: pierre

How a duck is reached from outside the LAN. [`remote-webrtc.md`](remote-webrtc.md) §7 states the
shape — a bridge from a rendezvous service to the signalling server already running on the robot —
and it is right about the shape. This page owns the two things that shape needs and does not have:
a **credential that names an account**, and a **service to present it to**.

**Built so far** (2026-09-02): §2 — the account. `account.login`, `account.status` and
`account.logout` are served by `updaterd` and reachable locally, over BLE and over a WebRTC
datachannel; `robotctl account login` prints a code and waits, `duckctl account login` prints one
and hangs up. The credential lands in `/etc/robot/hf-token` and renews itself. Nothing consumes it
yet — that is §3, and it is the next slice.

What is **established** about the services this depends on, by probing them rather than by reading
about them:

- **Hugging Face implements the OAuth device grant**, and `huggingface_hub` ships a **first-party
  public client** for it (`DEVICE_CODE_OAUTH_CLIENT_ID`), so this needs no OAuth app registered
  anywhere. §2.3.
- **A token lasts 30 days**, comes with a refresh token, and **that refresh token rotates on every
  refresh**. §2.7 is what that costs.
- **The token carries every scope Hugging Face grants** — `write-repos`, `manage-repos`, `jobs`,
  `read-billing` — because the first-party client takes no `scope` parameter. §2.4, and it is the
  one thing here that should change before a duck ships.
- **`/oauth/userinfo` answers with the identity in one round trip**, so nothing decodes a JWT.
- **The rendezvous is ours**, and it is `pollen-robotics/reachy_mini_central` — a FastAPI app in
  a Space, readable and changeable by us. Everything §3 and §4 say about it below is read off that
  server rather than inferred from a client.
- **Its wire is not the gst signalling protocol on a WebSocket.** It is the same JSON envelopes
  over **HTTP** — SSE inbound, `POST` outbound, `Authorization: Bearer` — with per-hop peer and
  session ids. This corrects a claim in `remote-webrtc.md` §7; §3.2 says what it costs.

## 1. What has to be true for a duck to be reachable

Four things, and the robot has none of them:

1. a credential that names an account the robot belongs to (§2);
2. a service the robot reaches **outward**, which shows a robot only to its owner (§4);
3. a bridge from that service to `ws://127.0.0.1:8443` (§3);
4. a client that speaks the service's wire, served from somewhere the client can reach (§5).

Plus NAT traversal, which is a property of the media path rather than of any of the above (§6).

One invariant constrains all four, and it is not negotiable here: **local mode must not come to
depend on any of it.** `architecture.md`'s first invariant is that local recovery stays independent,
and `remote-webrtc.md` §7 extends it to media — if the service is down, a LAN client still connects.
Every choice below that could have been made more simply by routing local sessions through the
service too was made the other way for this reason, and §3.1 is where it costs something.

## 2. The account is an OAuth device flow against Hugging Face

### 2.1 Why the device grant, which is also where `reachy_mini` ended up

`reachy_mini` has **both**. It started with authorization code + PKCE, pointing the redirect URI
back at the robot's own HTTP server — `http://reachy-mini.local:8000/api/hf-auth/oauth/callback`,
or `localhost` for the tethered variant — and added a device-code flow later, described in its own
source as "refresh-capable, redirect-free login", which is what its mobile app's setup wizard uses
now. Its reasons are the two below. Worth knowing that this is not a difference of opinion: it is
the same conclusion reached twice, from different directions.

Three costs come with the redirect flow:

- **A registered redirect URI per hostname.** The app has exactly one, which is why the mobile app
  carries a loopback HTTP bridge that catches HF's callback on `127.0.0.1:8000` and rewrites it as a
  302 onto a custom `reachymini://` scheme — a whole component whose stated purpose is avoiding an
  HF-side config change (`features/auth/oauthLoopback.ts`).
- **The browser must be able to resolve and reach the robot.** The callback is a URL *on the robot*.
  So logging in requires being on the robot's network, with mDNS working, which is the same class of
  problem `webrtc-console.md` §2 spends a section on for a page.
- **It is a browser flow on a device with no browser.** The robot is not the party that authenticates;
  it merely hosts the landing pad.

The device grant inverts that: the robot asks HF for a code, says *"open hf.co/oauth/device and type
M8HJ-FMGN"*, and polls until somebody has. No redirect URI, no hostname, no requirement that the
authorising device can reach the robot at all — a phone on cellular is fine. It is the flow specified
for a device with no browser and no keyboard, which is what a duck is. It is also the only one of the
two that yields a **refresh token**, which is what keeps a robot reachable past its first month
(§2.7).

The cost, stated plainly: somebody types eight characters. That is the whole of it, and it is smaller
than the mDNS dependency it removes.

**Three invariants inherited from `reachy_mini`'s wizard**, none of which is about Python and all of
which were learned the expensive way:

- **Lead with the code. Whether to open a browser is a property of the surface, not a rule.** The
  mini's app learned that auto-switching to Safari hid the code before anyone could read it — but
  that is a *phone*: the browser replaces the only screen and backgrounds the app, so the code is
  gone. A terminal keeps it in the scrollback, and the same reasoning gives three different
  answers here:

  - `robotctl account login` opens nothing, because it runs **on the robot**, which has no
    display. It prints the code and waits.
  - `duckctl account login` prints the code and *then* opens the page, because it runs on your
    own machine — where `duckctl open` already launches a browser. `--no-open` suppresses it, and
    so does stderr not being a terminal, because a script that opens a browser window on whoever
    runs it is a surprise. A browser that will not launch is a warning appended to the code, never
    an error replacing it.
  - A phone app keeps the mini's rule as written, for the mini's reason.

  **What opening buys is the navigation, not the typing.** An earlier version of this section
  claimed the robot could hand over a URL with the code in it, on the strength of a note in the
  mini's setup docs saying `huggingface_hub` synthesises a `?user_code=` form. It does not — it
  falls back to `verification_uri` unchanged — and HF's device page ignores the parameter, which a
  browser confirmed after this shipped. `verification_uri_complete` is therefore the plain page
  today, and stays in the reply only because a server that starts sending a real one is then used
  without a wire change.
- **The client going away mid-flow is expected, not an error.** Opening the HF page backgrounds a
  phone app, and iOS then tears the GATT link down. By that point the transport has done its job:
  the daemon is polling and the client comes back to `status`. This is why `login` answers with a
  code rather than a token, and it is the property `robotctl`'s wait loop is careful to not be
  load-bearing for.
- **Appearing on the rendezvous is the only real success signal.** A stored token means the *login*
  worked, not that the robot is reachable — their wizard treats "a robot with my hardware id is in
  the listing" as done and everything else as recoverable. §3 has to make that check available
  here, and `account.status` reporting relay state is where it will go.

### 2.2 The flow, and the transcript it was built from

```
POST https://huggingface.co/oauth/device
     client_id=26be6b09-91c5-47da-9861-d2d2bb7a7e36

  → {"device_code":"41ad39ae-…","user_code":"A6MY-0314",
     "verification_uri":"https://hf.co/oauth/device","expires_in":300}

POST https://huggingface.co/oauth/token
     grant_type=urn:ietf:params:oauth:grant-type:device_code
     &client_id=26be6b09-…&device_code=41ad39ae-…

  → HTTP 400 {"error":"authorization_pending"}      until somebody approves it
  → {"access_token":"…","refresh_token":"…","expires_in":2591999,
     "token_type":"bearer","id_token":"…","scope":"manage-repos write-repos …"}

GET  https://huggingface.co/oauth/userinfo   Authorization: Bearer <access token>
  → {"name":"Rouanet","preferred_username":"PierreRouanet","orgs":[…], …}
```

No `scope` is sent, because the first-party client does not take one (§2.4). Five things this
answers, each of which is a decision nobody has to make now:

- **No `verification_uri_complete`, and no way to synthesise one.** There is no URL that carries
  the code — HF's device page ignores `?user_code=` — so the code has to be *read by a person*,
  which makes displaying it the **client's** job rather than the daemon's. The field is passed
  through as the plain `verification_uri` so that a server which later sends a real one needs no
  change here.
- **No `interval`.** RFC 8628's five seconds applies, and `slow_down` adds five more.
- **`expires_in: 300`** on the *code*. Five minutes: long enough not to hurry, short enough that a
  client should show what is left, which is why `account.status` counts it down rather than
  repeating the original number.
- **`expires_in: 2591999`** on the *token*, with a `refresh_token`. §2.7.
- **`/oauth/userinfo` gives the identity in one round trip** — `preferred_username` is the handle
  and `name` is a display name that can be anything, so the handle is what is stored and shown.
  The alternative was decoding the `id_token`, which is a JWT parser and a JWKS fetch for the same
  string.

  **It is the last call and the least important one, so it cannot be allowed to fail the login.**
  By the time it runs, somebody has typed a code into a phone; discarding the token because a
  proxy answered 502 would make them do the whole flow again for a label. So the record is stored
  with no name, `account.status` answers `unknown` rather than "signed out", and the next
  `maintain` pass fills it in — the network that failed is the one this board is expected to have.

**One login at a time, and the guard covers the round trip rather than the check before it.** Two
callers arriving together — a console page and a phone, which is a normal thing to happen during
setup — would otherwise both be handed a code, and the store would keep whichever approval landed
last while somebody read the other code and watched it do nothing. The second caller gets
`BUSY` while the first is in flight — with its own message, because "another update is already in
progress" is what `BUSY` says elsewhere and it would send that person looking in the wrong place.
`account.status` carries the code, so a client that lost track of one rejoins it rather than
starting another.

**A refusal has to name a way through it, and `force` is that way here too.** A code nobody is
going to approve — the usual case, somebody started a login and walked off — otherwise holds the
robot for five minutes, and the only remedy was `logout`, which destroys a working credential to
clear a pending one. `force` already means "replace what this robot belongs to"; replacing an
*attempt* at it is the same permission in a smaller size.

**What that costs is that an abandoned flow is still holding a live device code**, and Hugging
Face will approve it if somebody gets round to it. The flow lives in a task that outlives the call
which started it, so this is equally true of `logout`: sign a robot out with a code in the air and
an approval a minute later would sign it back in on its own. Each flow therefore carries the
generation it was started with and checks that it is still the current one — before the store is
touched and again after, because the write awaits — so a superseded login keeps its code and
drops its answer. That is also what makes `logout` able to promise what it says.

### 2.3 The OAuth client is Hugging Face's own — **closed**

`huggingface_hub` ships a **first-party public device-code client**, `DEVICE_CODE_OAUTH_CLIENT_ID`
= `26be6b09-91c5-47da-9861-d2d2bb7a7e36`, which is what `hf auth login` uses. It is public — no
secret, so nothing needs baking into a release beyond a public identifier — and it needs no OAuth
app registered anywhere. `updater::account::CLIENT_ID` is that constant, and it is the whole of
what this decision came to.

Two alternatives, recorded because the first one looks obvious and is blocked:

- **`reachy_mini`'s own app** (`71146982-…`) is a **confidential** client, so the device endpoint
  refuses it — *"if you want to use the device code flow without client secret authentication,
  delete the secret from the oauth app to make it public"*. Baking that secret into every duck
  makes it not a secret, and making the app public would change the mini's posture to suit us.
- **Dynamic registration.** `POST /oauth/register` is unauthenticated and honours
  `token_endpoint_auth_method: "none"`, so a robot could mint its own public client at first login.
  It works. Pointless now, and a client per robot would be a fleet of identities nobody can
  enumerate or revoke.

### 2.4 The scopes are not ours to choose, and that is the thing to fix before shipping

The first-party client takes **no `scope` parameter**, and the token it issues carries everything
Hugging Face grants:

```
manage-repos write-repos read-repos gated-repos contribute-repos write-collections
read-collections openid write-discussions inference-api jobs webhooks read-billing read-mcp
```

A duck therefore holds a credential that can **push to its owner's repositories, start Jobs and
read their billing** — for something whose entire purpose is proving an identity to a rendezvous
service. Every Reachy Mini in the field is in the same position; that is context, not a defence.

What the account actually needs is `openid profile`, plus `read-repos` for one real thing:
`policy.install` reaching a **private** policy repo from the same token file, with no new
mechanism (`policy-channel-design.md` §7).

**So the narrow version is a Pollen-owned public device-code client with
`openid profile read-repos`** — one constant in `account.rs` and one click by somebody with HF org
admin. It is not blocking: the flow works today and a scope change is a re-login. It is worth
doing before a duck goes home with anybody, because the failure mode is asymmetric — a robot that
has been able to write all along cannot be un-done, while a robot that needs a wider scope later
just asks for one.

### 2.5 Where the token lives, and who writes it

`/etc/robot/hf-token`, `root:robot`, `0640`. JSON: the access token, the refresh token, an
absolute `expires_at` (the response gives a duration, and a duration means nothing after a reboot)
and the username, so `account.status` answers with no network at all — a robot that is offline
still knows who it belongs to.

**Written `0600` and relaxed to `0640` after the group is set**, rather than through
`fsutil::write_atomic` like everything else this daemon writes. That helper does not set a mode,
and a token that lands `0644` and is chmodded a moment later is world-readable for that moment —
the kind of window that is invisible in testing and permanent in a `ps`-and-`cat` afterwards.
`account::write_private` is the same rename dance with the temp file opened `0600` from the start,
and a test asserts the landed file gives "others" nothing. On a board with no `robot` group — a
developer's laptop, a half-provisioned board — it stays root-only and says so once, rather than
guessing.

**Not in `robotd.toml`.** Every mechanism that exists for that file is wrong for a secret:
`robotctl configure --list` prints what a robot changes, `policy-channel-design.md`'s full-screen
editor shows the file, and "what has been changed on this robot" is a report we now generate. A
bearer token would be in all three outputs. Its own file, with its own mode, is the whole of the
protection it gets — and §7 says why that is enough.

**`updaterd` owns it.** It already has the HTTP client, already reaches the network on the robot's
behalf, already runs as `root`, and already has a namespace of calls that write system state
(`policy.*`). `configd` owns *config*, has no HTTP client, and adding outward network egress to the
daemon that answers `system.info` would be a new kind of thing for it. `mediad` must not own it: it
runs as `User=mediad` under `ProtectHome=yes`, and it is the process a remote peer talks to.

**`mediad` will read it** — on each connect attempt, plus a slow poll (30 s) while it has none,
which is also its `waiting for token` state. Not built: nothing consumes the credential until §3
exists. No cross-daemon notification: `reachy_mini` has a
`notify_token_change` call from its auth router into its relay, and re-reading the file on the
reconnect that is going to happen anyway makes it unnecessary. The cost is that a fresh login takes
up to a poll interval to become a live producer, which nobody can perceive.

### 2.6 The calls, and which transports may reach them

`account.login`, `account.status` and `account.logout`, on `updaterd` — for `policy.*`'s reasons
exactly: it is the daemon with a network stack, `robotctl` must not link one (it is on the
recovery path), and the credential it stores is also what would reach a private Hub repo, which is
already this daemon's job. `configd` owns *config*, not credentials, and has no HTTP client.
`mediad` must not own it: it runs unprivileged under `ProtectHome=yes`, and it is the process a
remote peer talks to.

**`login` answers with a code and hands the waiting to the daemon.** There is no progress
notification and no long-held connection; a client polls `status`. That is not a simplification,
it is the requirement — see §2.1's second invariant.

**All three are routed to all three transports**: local, BLE and a WebRTC datachannel. BLE matters
most and is the easy call — it is the only transport that reaches a robot fresh out of a box,
which has no network, hence no console and no LAN to open one from, and it is where a setup wizard
already lives. Locally it is `robotctl`, which is how a developer does anything.

**WebRTC is the one worth arguing about, and it is worth writing down rather than assuming.** The
console is the obvious place to put a "sign in" button — a page with the robot already on screen
— and the alternative is ssh or a phone. Against that: this is the one call on that transport
whose effect is **durable in a way nothing else there is**. Everything else a LAN peer may do is
bounded by the session; `account.login` converts *having been on the wifi once* into remote access
that outlives being there. §4 of `remote-webrtc.md` accepts that anyone on the network has the
robot and its camera. It did not consider anyone on the network having them from another continent
next month.

Three things make that acceptable rather than merely permitted:

- **A robot that already belongs to somebody refuses.** `account.login` without `force` answers
  `INVALID_PARAMS` naming the account, so a LAN peer cannot silently take a robot from its owner —
  it has to say so, and a well-behaved client has to ask a person first.
- **It is visible.** `account.status` names the account, from any transport, with no
  authorisation at all. "Which account does this robot belong to" is a question anybody can ask.
- **It is revocable** — `account.logout` from anywhere, and revoking the grant on Hugging Face,
  which no robot-side gate could offer.

  Worth being exact about the first one, because it is weaker than it sounds: **`logout` deletes
  the robot's copy and revokes nothing.** The robot stops being a producer, which is the effect
  somebody signing out is after — but the access token it held stays valid at Hugging Face until
  it expires, up to thirty days, for anything that already read the file. The credential is
  `0640 root:robot`, so "anything" means root or `mediad` on that board; a stolen board is the
  case that matters, and for that the answer is the account's connected-apps page on hf.co, not a
  call here. Sending a revocation from the robot is a candidate for §9 and deliberately not in
  this slice: it needs the endpoint checked against the first-party client rather than assumed,
  and a `logout` that failed because the network was down must still forget the token locally.

**One consequence that lives in another file, and it found a bug.** `account.login` and
`account.logout` are `Call::is_mutating`, which is what `updaterd` authorises against a peer's
uid, so `mediad` had to be added to `allow_users` in `deploy/updater.toml` alongside `btd`.

**Be exact about what that grants, because it is more than the account.** `allow_users` is a
gate on the *caller*, not on the method: `updaterd` now performs any mutating call `mediad` makes,
`update.apply` included. What stops a remote peer applying an update is `mediad`'s own route
table, which refuses to relay it — so that table is not one narrowing among several, it is the
only one, on the process most exposed to a stranger's traffic. `btd` has had exactly this shape
since BLE could apply an update, and the same answer: the boundary is a named list with a test,
`mediad::route`'s `only_these_mutating_calls_are_reachable_over_webrtc`, so routing a new mutating
method has to change that list on purpose and say why. A per-method gate in `updaterd` is what
would make it two layers rather than one; it is not built, and this is the note that says the
choice was made rather than missed.

Writing it down immediately turned up two methods nobody had noticed were broken:
**`policy.install` and `policy.fetch` were routed to WebRTC while `mediad` was not in
`allow_users`**, so `updaterd` answered them `PERMISSION_DENIED` — the console could offer a Hub
browser whose install button could not work. The `allow_users` line added here for the account
fixes them too. That is the argument for a named list over a counted one: the list is where a
transport's authority and a config file's grants are forced to agree out loud.

### 2.7 The token expires in 30 days, and the refresh token rotates — **closed**

A device-code token comes back as `expires_in: 2591999` — thirty days — with a `refresh_token`,
and refreshing **rotates** it: the answer carries a *new* refresh token and the old one is spent.
So the store is two strings plus a clock, and there are three consequences worth naming.

**A robot that is simply left on must renew itself.** `updater::account::maintain` wakes every six
hours and refreshes anything with under a week left — three-quarters of the way through the
token's life, leaving a week of retries for a board whose network is marginal. It is spawned
unconditionally, unlike the update scheduler, because a robot with update checks switched off
still has an account that stops working after a month.

**Rotation leaves one window that cannot be closed.** Between "Hugging Face issued a new pair" and
"the new pair is on disk", the old refresh token is already dead. A power cut in that window
leaves a robot holding a credential HF will not renew. No write ordering fixes it — the rotation
happened on their side — so it is handled rather than prevented: the write is atomic (a reader
sees the old pair or the new one, never half of one), the failure surfaces in
`account.status`'s `last_error`, and the fix is signing in again. Renewing a week early is what
makes that a nuisance rather than an outage.

**A robot switched off for more than thirty days comes back needing a login**, and no margin can
save it. `account.token_expires_in` goes negative, which is how a client says so rather than
leaving somebody to discover it when the robot fails to appear.

## 3. The bridge

### 3.1 A listener on loopback, not a signaller inside `webrtcsink`

`webrtcsink` takes a custom signaller (the `Signallable` interface), so the temptation is to implement
one that speaks the rendezvous wire directly: one hop fewer, no id translation, no local WebSocket
client. **Rejected**, for a reason that is structural rather than aesthetic: one `webrtcsink` has one
signaller. Pointing it at the service means local sessions go through the service too — which breaks
§1's invariant — and keeping both means a second `webrtcsink` off the tee, which means encoding the
same frames twice on a board where the encoder is the budget.

So the bridge is what `remote-webrtc.md` §7 describes, and what `reachy_mini` runs:

```
  rendezvous  ──SSE──►  relay task  ──ws──►  127.0.0.1:8443  ◄──ws──  webrtcsink
  (HTTP)      ◄─POST──  (in mediad)  ◄──ws──  signalling server      (the producer)
```

The relay registers with the service as a **`producer`** and with the local server as a
**`listener`** — the roles are inverted on the two sides, because to the service it *is* the robot and
to the pipeline it is a peer asking for a session.

### 3.2 What it translates, and the correction to §7

§7 says "the bridge parses nothing. It proxies the gst signalling protocol, which is the same protocol
a LAN client speaks." The payloads — SDP and ICE — are indeed opaque and stay that way. The envelope
is not, in three ways:

| | local side | rendezvous side |
|---|---|---|
| transport | WebSocket | SSE inbound, `POST /send` outbound |
| auth | none (§4 of `remote-webrtc.md`) | `Authorization: Bearer <hf token>` |
| ids | its own `peerId`, its own `sessionId` | different ones, per hop |
| our role | `listener` | `producer` |

So the bridge keeps a session table both ways and rewrites `sessionId` on every `peer` message. That
is where `reachy_mini`'s relay has needed most of its scar tissue (§3.4), and it is the honest
description: **a translator with an opaque payload**, not a relay. The payoff §7 claims for
`webrtcsink` over `webrtcbin` survives — the protocol still exists rather than being invented, and the
translation is a table rather than a parser — but "proxies, parses nothing" should stop being said.

### 3.3 The lease, and why the heartbeat is not optional

The service evicts a producer that has sent nothing for `LEASE_SECONDS`, whatever its socket looks
like. That is not defensive over-engineering on its part: a half-open TCP connection — wifi yanked,
NAT rebinding, a sleeping captive portal — absorbs server-pushed keepalives silently for minutes,
during which the robot believes it is reachable and is not.

The numbers, from the server rather than from a guess: `PRODUCER_LEASE_SECONDS` is **30**, and
the SSE welcome advertises `recommended_heartbeat_interval_seconds: 10.0`. The lease is keyed
*only* on inbound `POST /send` — a healthy-looking SSE stream refreshes nothing.

So the relay re-emits `setPeerStatus` at the cadence the welcome names, falling back to 5 s and
clamped to [1 s, 60 s] so a misconfigured service can neither ask for a request storm nor talk us
into a cadence slower than our own eviction. `reachy_mini`'s ladder has a middle rung —
`lease_seconds / 3` — for a server that publishes the lease but not the cadence. **This server
publishes no `lease_seconds`**, so that rung is unreachable here; it is not worth reproducing a
negotiation step for a field nothing sends.

**The SSE side has its own keepalive to size against.** After 30 s with nothing to deliver the
server emits an `event: ping`, whose only job is to stop the HTTP/2 proxy in front of the Space
from killing an idle connection. A read timeout on our side therefore has to be comfortably more
than 30 s — `reachy_mini` uses 60, which is two missed pings, and that is the number to take.

### 3.4 The failure modes are already known, which is the main reason to read their relay

Four, each cheap to build in now and expensive to rediscover:

- **Split-brain.** The SSE stream is healthy and the service no longer lists us — a `setPeerStatus`
  round trip cancelled mid-flight leaves exactly this. Nothing in the connection notices. Their
  answer: poll `/api/robot-status` every 30 s, and force a reconnect after two consecutive misses.
- **Concurrent sessions.** The server does gate this — `handle_start_session` answers
  `sessionRejected` with `reason: "robot_busy"` and the `activeApp` that holds it, and pushes a
  `sessionStateChanged` to the owner's other devices so their UI flips inside the round trip. So
  the robot-side gate is belt-and-braces rather than a workaround, and it should stay: a second
  peer driving the same robot is `remote-webrtc.md` §9's interleaving bug with two remote writers
  instead of a pad and a peer, and that is a bad enough outcome to check twice.
- **Ordering at registration.** Register as a producer *before* reporting `connected`, or every
  observer — a status call, a page, a person — sees "remote access enabled" while the service does not
  yet know the robot exists.
- **Backoff with jitter, capped.** 5 s growing to 60 s, plus ~10%. A fleet reconnecting in lockstep
  after a service restart is a self-inflicted outage.

What we do **not** copy is their `RobotAppLock`: it arbitrates a local *app* against a remote session,
and a duck has no app. `remote-webrtc.md` §9 owns the equivalent question here (a pad and a peer both
writing intents) and defers it deliberately; a second remote peer is the same gap, not a new one.

### 3.5 It is a task in `mediad`, in a module with no GStreamer in it

Not a new unit. A `relayd` would need its own copy of the producer identity, its own config, its own
restart story, and it would still be useless without `mediad` running — three new moving parts to
isolate a task that is a websocket, an HTTP client and a hash map.

It goes where `session.rs` went, and for the same reason: **transport-agnostic on purpose**, so it is
testable on a laptop against a fake service and a fake local server. That is what made the control
channel testable without a board and it is worth twice as much here, because every failure in §3.4 is
a timing failure that no manual test on hardware will reproduce on demand.

One inherited rule: nothing in a GStreamer signal handler may panic (`pipeline.rs`'s header, and the
process abort that taught it). The relay never touches one — but it will want to *reach* the pipeline
eventually, and that is the boundary to keep clean.

### 3.6 What a bridged peer may call: exactly what a LAN peer may

The session is the same `webrtcsink` session and `route.rs` is the same table. This is deliberate and
it is also the part worth re-examining once it works: §4 argues the robot needs no gate because the
service authenticated both ends, and after this page that argument gets *stronger* — a bridged peer
has proved account ownership, where a LAN peer has proved only that it is on the wifi. The robot can
tell them apart by source address (§7 notes it), and nothing yet acts on the difference. Keep it true.

### 3.7 What a duck has to call itself, and it is not free-form after all

`meta` is free-form to the *protocol*, but the server reads two keys out of it, so a duck that
fills it in arbitrarily gets subtly wrong behaviour rather than a clear failure:

- **`hardware_id`** (or `install_id`) is the **stable-identity key**. On `setPeerStatus` the server
  looks for another producer of the *same user* carrying the same value and evicts the older one —
  ending its session if it had one. It exists because a re-flashed daemon, a duplicated SD card or
  a stale tray process would otherwise show up as a second robot forever.

  So a duck **must** put something stable there, and the obvious candidate already exists:
  `producer.rs` reads the SoC serial for the local `meta`, which is exactly "stable per physical
  robot across reinstalls and renames". Leaving the key out means a robot that reconnects with a
  fresh token is listed twice; putting the *name* there means renaming a robot forks its identity.
- **`name`** is what the listing shows a person, and the consumer's `name` is what the server
  reports back as `activeApp` to the owner's other devices. `transport` (`"wifi"` / `"usb"`) is a
  mini-ism a duck can leave alone; a `kind` of `microduck` is what lets one client list both
  families without opening a session.

**And a hazard that belongs in the provisioning path, not here: peers are keyed by token.**
`get_or_create_peer` is a `token -> peer_id` map, and a second SSE connection on the same token
supersedes the first. Two robots sharing one token therefore take turns being reachable, and
neither is broken in a way that looks like a bug. Each duck runs its own device flow, so each gets
its own token — *unless* an image is cloned with `/etc/robot/hf-token` in it, which is exactly what
this project's flashing path does with everything else in `/etc/robot`. Whatever produces a golden
image has to exclude that file, and this is the note that says why.

## 4. The rendezvous is the one `reachy_mini` uses — **decided**

`pollen-robotics-reachy-mini-central.hf.space`, the Space the mini's fleet already registers
with. Decided rather than derived: it costs no backend work, it is proven under real robots, and
the whole of this page becomes a client-side project.

What a robot needs from it is small: `GET /events` (SSE, `Authorization: Bearer <hf token>`),
`POST /send` (same), and `GET /api/robot-status` to ask whether it is still listed. Its `meta` is
free-form, so a duck registers with whatever identifies it — and `producer.rs` already assembles
exactly the fields a listing wants (name, serial, release, `api_version`), which is what
`webrtc-console.md` §5 predicted the rendezvous would need. A `kind` of `microduck` goes in the
same structure, so a client that lists a user's robots can tell a duck from a mini without
opening a session.

**We own it**, which is what makes this reuse rather than a dependency: `pollen-robotics/reachy_mini_central`
is a FastAPI app in a Space, and it can be read and changed on this side. Two things follow:

- **The protocol is a fact, not a guess.** Every number in §3 — the 30 s lease keyed on `POST /send`,
  the 10 s advertised cadence, the 30 s SSE ping, the `sessionRejected` gate, the `hardware_id`
  eviction — is read off `app.py`. An earlier version of this page said the repository was private;
  that was a wrong-name 401 mistaken for a permissions error, and the wire was reverse-read from
  the client for no reason.
- **A duck-shaped need is a pull request, not a fork.** If ducks want a different lease, a `kind`
  filter on the listing, or an eviction rule that does not assume one robot per token, those are
  changes to a server we maintain. Which also means the reverse: a change made there for the mini
  can break ducks, and after this page there is a second family of robots on it.

The one thing that does not transfer is its **lock model** — `RobotAppLock`, local app versus
remote session, which the mini's relay gates incoming sessions on. A duck has no app, so §3.4
takes the reconnect behaviour and leaves that part.

## 5. The client, and where it is served — **open**

The console is `include_str!`'d into `mediad` and served by the robot (`webrtc-console.md` §1), which
works because the client is on the LAN. **A remote client cannot fetch a page from a robot it cannot
reach**, so remote needs the page hosted off-robot *and* a second signalling transport in it.

Three ways:

- **The same file, published by CI to GitHub Pages**, with the transport chosen by the URL it is given.
  One page, two hosts, still no build step — the constraint `webrtc-console.md` §8 defends survives.
  The service stays a rendezvous and nothing about the client lives in it.
- **The service serves the page.** One host and one deploy, at the cost of putting the client inside a
  service we may not own; a page in a private repo is a page nobody here can edit.
- **No client yet.** Prove login and the relay with the service's own dashboard, `/api/robot-status`
  and `duckctl` — which needs no page at all, and is the whole of slices 1 and 2 in §8.

**Recommendation: the third, then the first.** The proof that the token path and the lease work needs
no UI, and deferring the hosting decision by a slice costs nothing.

One thing to know before that page is written, and it is settled rather than a preference:
`EventSource` **cannot set headers**, so a browser speaking the SSE wire must either put the token
in the query string or read the stream with `fetch` and split SSE by hand. `reachy-mini-js` does
the former. **The server is removing it** — `_resolve_hf_token` accepts `?token=` only as a
transitional fallback, logs a deprecation warning per client IP, and says in its own docstring that
the query form goes once the known clients ship the header. A bearer token in a query string is
also a bearer token in the Space's access log and in every proxy in between.

So the page is `fetch` plus a few lines of line-splitting, not one browser API — and it should be
written that way first rather than written twice.

## 6. NAT: decide the STUN server, defer TURN

`webrtcsink` defaults its `stun-server` to a public Google address. LAN sessions need none, so nothing
has exercised it — and the moment remote works, **a duck's reachability quietly depends on a third
party we do not run**. Set the property rather than inherit it, for the same reason
`remote-webrtc.md` §0 sets `congestion-control` to the value that is already the default: the day
upstream changes it should not be the day every robot's connectivity changes with it.

TURN is what makes symmetric NAT and CGNAT work at all, and it relays the *whole* session's media at
somebody's expense. Not in the first slice, and `remote-webrtc.md` §11 is right that the decision
belongs with whoever runs the rendezvous rather than with the daemon.

## 7. Authorisation, restated now that there is an account

§4 of `remote-webrtc.md` argues the robot needs no gate of its own because a bridged session was
authenticated twice before it arrived — the client to the service, the robot outward with a token —
and that the trust therefore *moved* into the service rather than vanishing. This page does not change
that argument; it adds the one thing §4 could not name, which is **when the binding happens and who
performs it**:

- Before `account.login`, a duck is unreachable from outside the LAN. There is nothing to attack.
- After it, one account owns it, and `account.login`/`account.logout` are the calls that can move
  that ownership. They are routed to every transport, including WebRTC, and §2.6 is the argument
  for that plus the three properties that make it hold — a robot already signed in refuses, the
  binding is readable by anybody, and it is revocable from more places than the robot.
- **A remote peer re-binding the robot is a narrower risk than it looks**, and it is worth being
  precise about why: only clients of the account the robot *currently* belongs to can reach it
  remotely at all, so a remote `account.login` is the owner's own client. The exposure that is
  real is the LAN one, and that is what `force` exists for.
- A robot that changes hands must be logged out. That is the same list as the pairing PIN and the
  calibration — a hand-over process, in M6 — and this is one more item on it, worth adding while the
  list is still being written rather than after a second-hand duck streams to a stranger.
- The token is a bearer credential in a file, so a stolen board yields it. The answer is §2.4's
  read-only scopes, not encryption: a robot has to read this file unattended at boot, so anything
  it can decrypt without a human is something the thief can decrypt too. Which is the sharpest
  argument for narrowing the scopes — as it stands, a stolen duck yields a token that can write to
  its owner's repositories.

## 8. Order of work

Five slices, and the first two are independently useful and need no client:

1. **`account login`** — the device flow, the token file, `account status`. `updaterd`. **Done**:
   three calls, three transports, two CLIs, and a token that renews itself. Verifiable on its own,
   which is what made it the first slice: it prints the Hugging Face username.
2. **The relay, registering only** — producer registration, the negotiated heartbeat, reconnect and
   backoff, the split-brain poll. `mediad`. Verifiable with no client at all: the service's dashboard
   counts a producer and `/api/robot-status` lists the duck.
3. **Session translation** — a remote consumer gets video and the `control` channel. The first slice
   that needs something to connect *with*.
4. **The client, hosted.** §5.
5. **STUN decided; TURN if a real network needs it.** §6.

## 9. What is open, and who can close it

| | needs |
|---|---|
| §2.4 the scope breadth | one public device-code client in the `pollen-robotics` HF org with `openid profile read-repos`, created by somebody with org admin. Not blocking — a scope change is a re-login — and it should not ship without it |
| §5 where the client is served | follows the shape of §3, and is the decision that actually couples us to a service |
| §2.6 `logout` revokes nothing | whether Hugging Face accepts a revocation for the first-party device-code client, checked rather than assumed. Not blocking — signing out stops the robot being reachable, and a stolen board is answered on hf.co — but it is the difference between "forgotten" and "revoked" |

Closed since this page was written: the OAuth client (§2.3 — Hugging Face ships one), whether the
token expires (§2.7 — thirty days, with a rotating refresh token), which rendezvous to use (§4 —
the mini's), and whether we can read it (§4 — we maintain it; the "private repo" in an earlier
draft was a wrong-name 401).

One item this page created rather than closed: **a golden image must not carry
`/etc/robot/hf-token`**, because peers are keyed by token and two robots sharing one take turns
being reachable. §3.7. That belongs to whoever owns the flashing path.

## 10. Not doing

- **The `teleop` datachannel.** `remote-webrtc.md` §6 owns it; a remote session makes head-of-line
  blocking more visible, not more urgent.
- **`update.*` mutations over a remote session.** §8 of `remote-webrtc.md` says what it will take, and
  the answer is a client that survives the restart rather than anything here.
- **Multi-peer.** One media session at a time, as before.
- **Per-session consent.** An M5 item, orthogonal to this page and made more pointed by it: a stream
  that can be started from another continent is the case `architecture.md` §7 was written for.
- **A duck-specific mobile app.** #107 designs one and M6 owns the phone spike. This page's client
  question (§5) is deliberately answerable without it.
