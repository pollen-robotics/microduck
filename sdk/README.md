# Driving a duck from a script

```python
from duck import Duck

with Duck("robot.local") as duck:
    print(duck.health()["healthy"])
    duck.move(vx=0.1)
```

`pip install -e .`, and the robot needs `mediad` running — the socket is `/agent` on the port the
console is served from, 8080 by default.

## What it is

One WebSocket, the same JSON-RPC every other transport speaks, and no media stack.
`architecture.md` §5.3 is the argument for why a program should not have to negotiate ICE and
decode H.264 to send an intent.

It is deliberately small. Every method is one call, and the ones that exist are the ones
`mediad::route` permits — so what a script may do is what a browser on the LAN may do, decided in
one place on the robot rather than twice.

## A behaviour is a loop

What the robot can see arrives on three streams at three different rates — state at the control
rate, depth at 15 Hz, frames at whatever you asked for. `watch` merges them and hands you the
latest of each, once per tick:

```python
with Duck("robot.local") as duck:
    for view in duck.watch(fps=4, camera=False):
        if view.nearest and view.nearest < 0.4:
            duck.move(vyaw=0.8)
        else:
            duck.move(vx=0.12)
```

`view.nearest` is the closest thing the depth sensor can see, in metres, or `None` when it sees
nothing usable. The interpretation is `tof::Frame::zone`'s, including the part that bites: the
sensor returns negative distances on a failed convergence and still flags them valid, so taken at
face value they are the nearest thing in the room.

`examples/wander_without_bumping_into_things.py` is a duck that walks around a room, in about
twenty lines of behaviour.

Nothing here looks at a picture. `view.frame` is the JPEG as it arrived and what is in it is your
model to run — an SDK that shipped one would be a much bigger dependency and a worse guess than
whatever you already have.

## Frames

The robot does not serve frames, it **sends** them to a socket you open:

```python
from duck import Duck, receive
import threading

threading.Thread(target=receive, args=(8099, print_frame), daemon=True).start()
with Duck("robot.local") as duck:
    duck.frames(url="ws://192.168.1.20:8099", fps=1)
```

That direction is the point. A robot behind a home router and a script anywhere else cannot pair
without a relay candidate, and the robot dialling out means NAT is not a participant.
`mediad/src/stream.rs` has the whole argument.

`examples/fetch_a_frame_and_send_an_intent.py` is both halves in about thirty lines.

## What it does not do

Live video. A viewer wants WebRTC and the console already is one. This is for when the consumer
is a program.
