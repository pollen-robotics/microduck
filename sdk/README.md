# Driving a duck from a script

```python
from duck import Duck

with Duck() as duck:
    print(duck.health()["healthy"])
    duck.move(vx=0.1)
```

`pip install -e sdk`, and sign in once with `hf auth login` (or set `HF_TOKEN`). `Duck()` finds the
one duck online on that account; `Duck("duck-c51b")` picks one by name when there are several.

## What it is

The rendezvous control lane: the same JSON-RPC every other transport speaks, carried as HTTP to
the rendezvous and answered on the robot by `mediad::relay`
([`remote-access-design.md`](../docs/design/remote-access-design.md) §3.8). No WebRTC and no
address to know, so the same script works on the duck's own wifi and from a data centre.

Every method is one call, and what a script may call is what `mediad::route` permits a remote
peer, decided on the robot. `duck.call(method, params)` reaches anything without a wrapper.

The Spaces under `spaces/` use these same modules (`wire.py`, `rendezvous.py`, `control.py`)
rather than a copy of them.

## What it costs

- **A signed-in account, even on the LAN.** The robot is found and reached through its owner's
  Hugging Face account. The token must be the script's own, not the duck's: the rendezvous keys a
  peer by token, so sharing one supersedes the robot.
- **A behaviour's rate, not a joystick's.** The rendezvous allows 1200 requests a minute per
  token, about 20 a second, and going over earns a `429` on everything that token does. The robot
  sends at most 400 notifications a minute on a lane. Fine for a loop deciding a few times a
  second, as an LLM does; not for teleop, which is what WebRTC is for.

## A behaviour is a loop

`watch` merges state and depth and hands over the latest of each, once per tick:

```python
with Duck() as duck:
    for view in duck.watch(fps=4):
        if view.nearest and view.nearest < 0.4:
            duck.move(vyaw=0.8)
        else:
            duck.move(vx=0.12)
```

It asks the robot for each stream at the tick rate, held under what the lane carries, so at
`fps=4` a tick may repeat the reading before it. `view.nearest` is the closest thing the depth
sensor sees in metres, or `None`, read the way `tof::Frame::zone` reads it: a negative distance is
a failed convergence, not the nearest thing in the room.

`examples/wander_without_bumping_into_things.py` is a duck walking around a room.

## Frames

The lane carries no pixels. To watch a duck, use WebRTC, which the console already speaks.

For a program taking frames on a long-running stream, `media.stream` is the fallback
([`faq.md`](../docs/faq.md)): the robot dials a WebSocket you serve and pushes frames to it. It only
works where the robot can reach you, a Space or a server with an address of its own.
`stream_frames` starts it and `receive` serves the far end, decoding JPEG or H.264 by what the
robot's hello says (`pip install -e "sdk[frames]"`):

```python
threading.Thread(target=receive, args=(8099, on_picture), daemon=True).start()
with Duck() as duck:
    duck.stream_frames("wss://this-server.example/frames", fps=1)
```

`examples/fetch_a_frame_and_send_an_intent.py` is both halves.
