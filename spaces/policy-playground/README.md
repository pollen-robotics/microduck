---
title: microduck policy playground
emoji: 🦆
colorFrom: yellow
colorTo: pink
sdk: docker
app_port: 7860
pinned: false
hf_oauth: true
hf_oauth_expiration_minutes: 1440
short_description: Pick a trick, watch your duck do it.
tags:
 - microduck
---

# microduck policy playground

Sign in, wake your duck up, press a trick. The trick is downloaded from the Hub onto the duck and
run, and the card you pressed says which of those it is doing.

**Do not edit this Space directly.** The source is `spaces/policy-playground/` in
`pollen-robotics/microduck`, and `scripts/publish-space.sh` is what puts it here.

## Working on it

```bash
cd spaces/policy-playground/web && npm install
```

```bash
npm run dev
```

That serves the real page against the real Hub and the real rendezvous. Signing in needs an OAuth
app whose redirect URI is exactly the dev URL, so pass one:

```bash
open "http://localhost:5173/?client_id=<an app registered for localhost:5173>"
```

Building writes one self-contained `index.html` beside the `Dockerfile`:

```bash
npm run build
```

That file is committed, because `publish-space.sh` publishes the *top level* of a space directory —
files, not trees — and a bundler emits `assets/`. Inlining everything keeps the published Space the
same four files the console has: a page, a Dockerfile, an entrypoint and this card. Nothing is
built on Hugging Face's side, so nothing there can fail for a reason nobody can see.

## Why it is a browser and not a Python container

The Gradio version of this page is still published and works, and every hard problem it had came
from being a client that runs in a data centre:

- **The rendezvous refused it.** `requests` signs its calls `python-requests/2.x`, which Hugging
  Face's edge reads as a bot: from a Space container the very first `GET /api/robot-status` came
  back `429` with an HTML page and `server=awselb/2.0`, and the service never saw the request. From
  a browser the call carries the visitor's own address and their own browser's signature.
- **Every visitor shared one robot.** The session lived in a module-level object, because module
  globals are per-process and a Space is one process. Here each visitor is their own browser, and
  per-visitor sessions cost nothing to arrange.
- **`aiortc`, `av`, a DTLS cipher patch and `PyGObject`** existed to give Python a WebRTC stack.
  A browser has one.
- **Server-side rendering**, which put a Node proxy in front of the page and stopped ten seconds
  after it started, with no traceback.

None of those exist here. `microduck-console` is the same shape and has never had any of them.

## Who it is for

A ten-year-old with a duck. That is a constraint on the whole page and not a coat of paint:
nothing on it names a method, a transport, a socket or a schema. A trick has a name, a sentence
about what it does, and one button.

What the four calls are — `policy.fetch`, `robot.setSkill`, `robot.policies`, `robot.do` — which
lane they take, and why a refusal happened, all live in **What just happened?** at the bottom. That
is where somebody goes when it breaks, not when they want to see a duck bow.

## The parts

`src/rendezvous.ts` is a port of `sdk/duck/wire.py`: `GET /events` for the stream, `POST /send`
for everything else, and a `peer` envelope carrying `rpc` is a control call. Four things that
protocol punishes a reader for not knowing are in its header, all of them learned in the Python
version and all still true.

`src/hub.ts` makes the same two requests `updater/src/policy.rs` makes — `?search=microduck` and a
`manifest.json` per hit — so the gallery and `policy.search` cannot disagree about what exists.
Every field below the repo is the publisher's claim, displayed and never acted on: what gets
installed comes from the robot's own reading of the manifest it downloaded.

`src/auth.ts` is PKCE, with the client id substituted into the page by `entrypoint.sh` rather than
injected by the platform — the console's Dockerfile says what the documented injection did instead,
which was nothing, for a day.

## What a press does

    getting it → putting it on your duck → waiting for your duck → doing it

**"Waiting for your duck" is not padding.** Putting a trick on a duck makes it reload, a reloading
duck goes back to its standing pose, and it refuses to do anything until it gets there — so without
the wait, the press that installs is the press that gets refused, and the trick silently never
runs. `homed` on `robot.policies` (`API_VERSION` 30) is the flag to wait on. A duck too old to
publish it sends nothing, and then there is nothing to wait for.

The rest of the page is inert while a press is in flight. A duck does one thing at a time, so a
page that accepted a second press would be promising something it cannot keep.
