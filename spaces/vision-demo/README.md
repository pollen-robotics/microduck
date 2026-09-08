---
title: microduck vision demo
emoji: 🦆
colorFrom: yellow
colorTo: indigo
sdk: docker
app_port: 7860
pinned: false
short_description: A duck's camera, processed on Hugging Face hardware.
---

# microduck vision demo

The robot streams H.264 to `pollen-robotics/reachy_mini_central`; this Space is the consumer,
decodes it with `aiortc`, and runs a few pixels of OpenCV over each frame — edges, motion, sparse
optical flow, and an overlay of the camera's geometry.

**Do not edit this Space directly.** The source is `spaces/vision-demo/` in
`pollen-robotics/microduck`, and `scripts/publish-space.sh` is what puts it here.

**Docker rather than `sdk: gradio`, and not by preference.** A Gradio Space installs
`gradio[oauth,mcp]==6.26.0`, which needs `starlette>=1.0.1`, where `reachy-mini` needs
`starlette<1.0.0` — every version of it. `ResolutionImpossible`, correctly. Gradio 5.23 resolves
with the same consumer, so the version is the thing to control, and only a Docker Space can.

## Two things it is for

It shows the path end to end: a camera on a duck, a container in a data centre, a processed
picture. That is the demo.

**What it measured, on its first run:** signalling crosses and ICE does not. The rendezvous
welcomes this consumer, lists the duck, starts a session, and the offer and answer are exchanged —
then `peer connection failed (ice failed)`, because no candidate pair works between a data centre
and a robot behind a home router and neither side can offer a relay. `remote-access-design.md` §6
has the log and what follows from it: TURN is a requirement for cloud consumers, not a fallback for
awkward networks. **This Space will connect the day the robot can offer a relay candidate, with no
change here.**

And it is **the only thing that tests the transport from a data centre**. Every session before it
came from a browser on the robot's own network, or a phone on 4G. A Space is neither, and the robot
cannot offer a `relay` candidate while the TURN credentials endpoint has no DNS
(`remote-access-design.md` §6) — so the status panel reports signalling, the peer connection and
the frame count separately. "Signalling worked and media did not" is a specific, expected outcome
here, and it is not a fault in this Space.

## Identity, and why this Space must stay private

It authenticates with an **`HF_TOKEN` secret** — Settings → Variables and secrets — belonging to
the account the robot belongs to. Not the robot's own token: the rendezvous maps a token to one
peer, so a consumer sharing the robot's credential takes the robot off its owner's listing (§3.7).

A visitor's own OAuth token would be better and would make this safe to publish. Two attempts at
one failed on platform behaviour rather than on code: a static Space never injected the client id
(`remote-access-design.md` §5.0), and a Docker Space does not put `OAUTH_CLIENT_ID` in the
environment either — Gradio then decides it is not in a Space, mocks OAuth, and refuses to start
without a local login. A demo that will not start is worse than a demo with one credential, so the
button is gone and the Space stays private.

## What it cannot do

**Drive the robot.** `ReachyCentralConsumer` looks for the data channel label the mini's daemon
opens and logs `ignoring unexpected data channel: 'control'` against ours, so no JSON-RPC reaches
this process. One consequence is visible in the demo: the camera geometry overlay *derives* its
numbers from the sensor's optics instead of reading the `intrinsics` the robot publishes in
`media.video`. §5.1 has the fix, and it belongs on their side of the SDK.
