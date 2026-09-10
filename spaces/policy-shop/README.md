---
title: microduck policy shop
emoji: 🦆
colorFrom: yellow
colorTo: pink
sdk: gradio
app_file: app.py
pinned: false
hf_oauth: true
short_description: Pick a policy off the Hub, put it on your duck, run it.
---

# microduck policy shop

Everything published to the Hub as `microduck-…`, read the way the robot reads it, with a button
that downloads one onto your duck and runs it.

**Do not edit this Space directly.** The source is `spaces/policy-shop/` in
`pollen-robotics/microduck`, and `scripts/publish-space.sh policy-shop` is what puts it here.

## The one click

Four calls over the session's `control` channel, in the order `robotctl policy add` makes them:

| | |
| --- | --- |
| `policy.fetch {repo, file}` | `updaterd` downloads it and reads the `manifest.json` beside it |
| `robot.setSkill {name, path, duration, …}` | `robotd` writes the entry and re-reads its skills |
| `robot.policies` | did the reload take? `change_error` is where it says no |
| `robot.do {skill}` | run it |

Nothing is invented on either side: those are `duck-ipc-proto` method names, already in
`mediad/src/route.rs`'s permitted set, and `mediad` knows none of them — the control channel is a
pipe to the API the robot already serves.

**The robot's reading of a manifest wins over this page's.** The catalogue is read here so a row
can say what a policy claims to be before anybody clicks it, but the skill that gets written comes
from `policy.fetch`'s answer, which is about the bytes that were actually downloaded.

## The picture

The session carries the camera as well as the channel, so the newest frame is shown beside the
buttons — decoded from H.264 by `aiortc`, on whichever transport is connected, and repainted ten
times a second. Watching the thing you installed do something is most of the point of installing
it from here rather than from a terminal.

It is put the right way up by asking. The camera is mounted a quarter turn off and nothing on the
robot rotates the pixels; `media.video` carries `rotate` and the control channel sends it once
when it opens, so there is no angle written down anywhere in this Space. `vision-demo` has to
compile one in, because it has no control channel to ask on.

## What it refuses, and why it is this side's job

A policy whose command the daemon generates — a phase for a ground pick, a flag for a sit↔stand —
cannot be a one-shot skill. `robot.setSkill` would accept the entry, and the robot would then feed
a constant to a network trained on a phase: it moves plausibly and wrongly, which is worse than a
refusal. `robotctl`'s `skill_encoding_refusal` is the rule, `robotctl` is not in the path of a
click, so `catalogue.refusal` is the same rule again.

The shape claims — `obs_len`, `action_len`, `model_api`, `robot.model` — are deliberately *not*
repeated here. `policy.fetch` checks them itself, before the download, so "this policy is 51-D and
this robot is 61-D" comes back in a second and comes back from the robot.

A `perpetual` policy has no length of its own, so a one-shot made out of one is a hold and an
unwind. `robotctl policy add` refuses without `--hold` rather than picking a number; this page asks
instead, and that is the seconds box above the rows.

## Two ways in, and right now only one of them works

**From anywhere** is the rendezvous: the robot registers as a producer holding its account token,
this signs in as the visitor, and the service pairs them. It is the path that reaches a duck behind
its owner's router, and it needs a candidate pair that works — which today it may not have.
`turn.fastrtc.org` has no A record and its zone has no NS records at all
(`remote-access-design.md` §6), so neither side offers a `relay` and a session between a home
router and a data centre falls back to host and srflx. Often that punches a hole. Often it does
not, and because the control channel is SCTP over the same candidate pair, when it does not punch
nothing here works at all. The status line names the stage it reached instead of saying
"connecting…", so this failure is legible rather than mysterious.

**On this network** needs none of that. `mediad` is already serving `webrtcsink`'s signalling
server on `ws://<robot>:8443` — it is what the robot's own console talks to — and it carries the
same gst envelopes the rendezvous carries over SSE and `POST /send`. So `lan.py` is one hop
swapped and nothing above it changed: same `control` channel, same JSON-RPC, same buttons. No
account, no lease, no rendezvous, no relay, and on one network both sides offer host candidates.

That makes it the useful thing to reach for when a click does not work and nobody knows which
layer to blame. **If it works on the LAN and not through the rendezvous, the transport is the
problem** and nothing about the policy, the manifest or the robot is.

A Space in a data centre cannot reach a LAN, so that tab is for running this yourself. Once:

```bash
uv venv && uv pip install -r requirements.txt
```

Then, and every time after:

```bash
uv run app.py
```

`uv run` picks up the `.venv` beside it, so nothing has to be activated and no flags are needed.
There is deliberately no `pyproject.toml`: `requirements.txt` is what Hugging Face installs, and a
second copy of one dependency list is a second copy to get wrong — which is the whole reason the
install step is a separate line rather than something inferred from a manifest.

`DUCK_HOST` pre-fills the address box. `HF_TOKEN` stands in for the sign-in, and so does whatever
`hf auth login` stored — see **Identity** for why a local run needs one of those rather than the
login button.

**One consumer at a time.** The rendezvous's rule, not a simplification: while this holds a
session, the robot's own console cannot open one, and neither can the vision demo. The LAN
transport is subject to the same thing for a different reason — one `webrtcsink` session per
consumer slot — so *disconnect* before opening the console.

## Identity

**A visitor's token by preference, and never the robot's.** The rendezvous maps a token to one
peer, so a consumer authenticating as the robot would take the robot off its owner's listing.
`hf_oauth: true` plus Gradio's login button gives each visitor their own, which reaches their own
robots and nobody else's — and is what makes a public Space defensible.

**Outside a Space, that button is a mock, and its token is a placeholder.** Gradio mocks its own
login when `SPACE_ID` is unset: the buttons behave and the profile is real, but `access_token` is
the literal string `mock-oauth-token-for-local-dev`. Sent to a service that resolves tokens
through `whoami-v2`, it is a `401` — which is what "the rendezvous refused this token" meant the
first time this page was run locally, while `microduck-console` listed the same duck perfectly.
So a mocked token is dropped rather than preferred, and a local run uses `HF_TOKEN` or whatever
`hf auth login` stored. The log line says which, on every press.

Ducks are told from minis on `meta.kind`, and the chosen robot is pinned by `peerId` rather than by
name: their consumer's auto-pick falls back to the only visible producer whatever it is called, so
an account with one duck and one mini could otherwise hand this page a mini to drive with method
names it does not serve. §5.1 has the other half, which is theirs.

## When it does not work

**The log says which of four things failed**, because they present identically as "the button did
nothing" and have nothing in common. It is at the bottom of the page and on the terminal at once —
a Space has logs nobody has open and a browser has no stderr — and it carries every HTTP status,
every signalling frame, every JSON-RPC line and every refusal, with the token's *source* named and
the token itself never written down.

```bash
DUCK_LOG=DEBUG uv run app.py
```

adds the streaming notifications and each ICE candidate — which is what to reach for when
signalling crossed and media did not, and nothing else.

Each layer is also checkable on its own, in the order the failures happen:

```bash
uv run rendezvous.py
```

lists the account's robots with their `kind` and their `busy` state. A `401` here is the token and
nothing else. This needs no Gradio and no robot.

```bash
uv run catalogue.py
```

prints the whole Hub catalogue with what each policy claims and which are refused. Needs no token.

```bash
uv run lan.py
```

drives a real session against a producer on loopback. If this passes and a duck does not, the
problem is between here and the duck rather than in this code.

## What the checks above actually cover

`catalogue.py` reads the Hub the way `updater/src/policy.rs` reads it — `?search=microduck` and a
`manifest.json` per hit — so the gallery and `policy.search` cannot disagree about what exists.

`rendezvous.py` is one `GET /api/robot-status`, read off `reachy_mini_central`'s own `app.py`:
`Depends(_resolve_hf_token)` then one `whoami-v2` call, with no scope check and no requirement
that the caller hold an event stream. Its docstring says it is for exactly this — "a passive
status indicator without consuming a session slot" — which also makes it safe to press
mid-session, where the console's `list` route would open a second stream on the same token and
evict the peer the session is riding on (§3.7).

`lan.py` stands up a producer on loopback that speaks what `webrtcsink`'s signaller speaks
and drives a real session against it: welcome, list, startSession, an offer answered, DTLS, SCTP,
the `control` channel, a call matched to its reply, and a video track decoded to check that frames
arrive at all and arrive as RGB. Two aiortc peers on `127.0.0.1` are not a duck — they are the
same protocol, and a dozen hand-written envelope shapes are exactly the thing that fails silently
rather than loudly. The producer there sends VP8 where a duck sends H.264, because the codec is
aiortc's to pick and the plumbing under test is the same one either way.
