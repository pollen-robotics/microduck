"""Drive a duck from a script, wherever its owner's account reaches it.

    from duck import Duck

    with Duck() as duck:
        duck.enable(True)
        for view in duck.watch(fps=2):
            if view.position and view.position[0] > 1.0:
                duck.stop()
                break
            duck.move(vx=0.12)

The Spaces under `spaces/` use these same modules rather than a copy.
"""

from __future__ import annotations

from .client import Duck, receive
from .control import RpcError
from .rendezvous import RendezvousError
from .view import View
from .wire import WireError

__all__ = ["Duck", "RendezvousError", "RpcError", "View", "WireError", "receive"]
