"""Drive a duck from a script.

`architecture.md` §5.3 argues a server-side program should not be pushed through WebRTC: an agent
does not want a 30 fps H.264 track to decode, it wants a frame every second or two and a state
blob, and ICE and DTLS in front of that is a poor trade. `mediad`'s `/agent` WebSocket is that
surface. This is the client for it, and it is the "few dozen lines" M5 asks for:

    from duck import Duck

    with Duck("robot.local") as duck:
        print(duck.health()["healthy"])
        duck.move(vx=0.1)

Every method here is one JSON-RPC call against the same `route` table the console page uses, so
what a script may do is what a browser on the LAN may do, decided in one place on the robot. There
is deliberately no method for anything the robot refuses that transport.
"""

from __future__ import annotations

from .client import Duck, receive
from .rpc import RpcError

__all__ = ["Duck", "RpcError", "receive"]
