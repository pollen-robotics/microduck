---
title: microduck vision demo
emoji: 🦆
colorFrom: yellow
colorTo: indigo
sdk: gradio
app_file: app.py
pinned: false
hf_oauth: true
short_description: A duck's camera, processed on Hugging Face hardware.
---

# microduck vision demo

The robot streams H.264 to `pollen-robotics/reachy_mini_central`; this Space is the consumer,
decodes it with `aiortc`, and runs a few pixels of OpenCV over each frame — edges, motion, sparse
optical flow, and an overlay of the camera's geometry.

**Do not edit this Space directly.** The source is `spaces/vision-demo/` in
`pollen-robotics/microduck`, and `scripts/publish-space.sh` is what puts it here.

## Two things it is for

It shows the path end to end: a camera on a duck, a container in a data centre, a processed
picture. That is the demo.

And it is **the only thing that tests the transport from a data centre**. Every session before it
came from a browser on the robot's own network, or a phone on 4G. A Space is neither, and the robot
cannot offer a `relay` candidate while the TURN credentials endpoint has no DNS
(`remote-access-design.md` §6) — so the status panel reports signalling, the peer connection and
the frame count separately. "Signalling worked and media did not" is a specific, expected outcome
here, and it is not a fault in this Space.

## Identity

**A visitor's token by preference, never the robot's.** The rendezvous maps a token to one peer, so
a consumer authenticating as the robot takes the robot off its owner's listing. `hf_oauth: true`
plus Gradio's login button gives each visitor their own token, which reaches their own robots and
nobody else's — which is also what makes a public Space defensible. An `HF_TOKEN` secret is the
fallback for a private Space with one owner.

## What it cannot do

**Drive the robot.** `ReachyCentralConsumer` looks for the data channel label the mini's daemon
opens and logs `ignoring unexpected data channel: 'control'` against ours, so no JSON-RPC reaches
this process. One consequence is visible in the demo: the camera geometry overlay *derives* its
numbers from the sensor's optics instead of reading the `intrinsics` the robot publishes in
`media.video`. §5.1 has the fix, and it belongs on their side of the SDK.
