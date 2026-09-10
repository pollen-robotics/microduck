---
title: microduck vision demo
emoji: 🦆
colorFrom: yellow
colorTo: indigo
sdk: docker
app_port: 7860
pinned: false
hf_oauth: true
short_description: A duck's camera, processed on Hugging Face hardware.
---

# microduck vision demo

A robot on somebody's home network streams its camera to this Space as H.264, and OpenCV runs
over each decoded frame here. Nothing is on the robot's network and no relay is involved.

**Do not edit this Space directly.** The source is `spaces/vision-demo/` in
`pollen-robotics/microduck`, and `scripts/publish-space.sh vision-demo` is what puts it here.

## The robot dials us, and that is the whole design

Pulling the camera through the rendezvous means WebRTC, and WebRTC between a robot behind a home
router and a container in a data centre needs a **relay candidate**. `turn.fastrtc.org` has no A
record and `fastrtc.org` no NS records at all (`remote-access-design.md` §6), so there is none:
signalling crosses and media never does. That was this Space's whole history — "signalling worked
and media did not" was its documented expected outcome.

So the direction is inverted:

```
this Space ──media.stream {url: "wss://…/frames"}──► rendezvous ──► the duck
the duck   ═════════ H.264, outbound wss, direct ═════════════════►  this Space
```

An outbound WebSocket is the one thing that always works — the robot is already holding one to a
Space every second it is reachable, which is how it appears in your robot list at all. NAT stops
being a participant, no credentials proxy has to exist, and **the rendezvous carries an instruction
rather than pixels**, which is what makes this scale where relaying payload through a service the
mini fleet depends on would not.

`media.stream` is the call, answered by `mediad` itself (`mediad/src/stream.rs`), and it reads
three ways off one key: a `url` starts a stream, `url: null` stops it, and no `url` asks what is
streaming.

## What that costs

**No return media path.** This cannot drive the robot or watch it in real time; a viewer wants
WebRTC and §6 is still what that needs. This is for the case where the consumer is a program.

**A few frames a second, not thirty.** Five at 640 px is plenty for a model and a fraction of the
encode the video track is already doing. The rate is imposed on the robot by a `videorate` in the
branch rather than by the receiver asking politely.

**H.264, with JPEG still reachable.** The board has a hardware encoder, so H.264 costs the VPU
rather than a core, and inter-frame prediction is worth a lot of bytes: on synthetic frames it
measured 0.5 KB against JPEG's 6.5 KB, and real camera footage will be less dramatic than that but
the same direction. What it costs is that a receiver cannot decode anything until a keyframe
arrives, and a dropped unit corrupts every unit after it until the next one. Both are handled
rather than hoped about — `mediad` asks its encoder for a keyframe the moment the valve opens,
repeats SPS/PPS in front of every keyframe, and abandons the stream to the next keyframe rather
than sending across a gap. `media.stream {"encoding": "jpeg"}` is the other option, for a receiver
that reconnects often enough to care more about starting instantly than about bandwidth.

**Frames arrive upright.** The pipeline's turn happens before the tee, so both the video track and
this branch carry an upright picture and the hello says `rotate: 0` with the mount angle beside it
for information. `filters.upright` is therefore unused here, and kept because it is still right
for a WebRTC consumer, which is told the angle and turns the picture itself.

## Whose camera this is

The frame endpoint is public — a Space's URL is a Space's URL — so the robot presents **its own
account token** on the handshake and this resolves it through `whoami-v2`, exactly as the
rendezvous does. Frames are filed under the username it answers with, and a visitor is shown only
their own account's robots. Without that, anybody could push frames into the demo and, worse, a
visitor could be shown a stranger's camera. The token is resolved once per connection and never
stored.

## Running it yourself

Once:

```bash
uv venv && uv pip install -r requirements.txt
```

Then:

```bash
DUCK_RECEIVER=ws://192.168.1.50:7860/frames uv run app.py
```

`DUCK_RECEIVER` has to be an address **the robot can reach** — your machine's address on the
robot's network, not `localhost`. On a Space it is derived from `SPACE_HOST` and needs no setting.
`HF_TOKEN` or what `hf auth login` stored stands in for the sign-in button, which is mocked outside
a Space.

## Why this is a Docker Space

FastAPI owns the server and Gradio is mounted into it, because the frames arrive on a WebSocket
route of our own. That is the documented direction — the reverse, adding routes to Gradio's app,
has known WebSocket breakage — and running `uvicorn` ourselves removes any question about whether
the platform will carry a custom route. `microduck-console` is a Docker Space for an unrelated
reason (§5.0), so this is the second.

## There is no WebRTC stack in here any more

It used to pull frames with `reachy_mini[central-consumer]`, which meant `aiortc`, `av`, and a DTLS
cipher shim over one of their private methods — and which could never connect from a data centre.
The robot dials us now, so what is left is `gradio`, `fastapi`, `opencv`, `requests` and `av`. That
last one is back for H.264, and it is a decoder rather than a transport: no ICE, no DTLS, no
signalling, nothing that a NAT gets a vote on.
