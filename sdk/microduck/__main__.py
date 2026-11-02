"""`python -m microduck [name]`: the account's ducks, then the lane to one of them, read only.

No Gradio, no browser, and nothing asked to move. It lists the ducks the rendezvous has for the
account, opens the lane to the one named (or the first), and asks what it is running and what it
can do. Which makes it the thing to run first when a script gets no answer, because it checks the
transport and nothing else.

A timeout here has one likely cause and the message says so: the envelope crossed the rendezvous
and the robot did not answer it, which is a `mediad` without the control lane.

Here and not under `wire.py` or `rendezvous.py`, because the package imports both, and
`python -m` on a module the package has already imported runs a second copy of it and warns.
"""

from __future__ import annotations

import logging
import sys

from . import rendezvous
from .client import _token
from .control import Rpc, RpcError
from .wire import WireError, WsConsumer


def main(argv: list[str]) -> int | str:
    logging.basicConfig(level=logging.INFO, format="%(name)-10s %(message)s")

    credential = _token()
    if not credential:
        return "no token: run `hf auth login`, or set HF_TOKEN"

    try:
        found, others = rendezvous.ducks(credential)
    except rendezvous.RendezvousError as e:
        return str(e)
    for listed in found:
        print(f"  {listed.peer_id}  {listed.label()}")
    print(f"\n{len(found)} duck(s)" + (f"; also listed: {', '.join(others)}" if others else ""))
    if not found:
        return 1

    wanted = argv[0] if argv else None
    duck = next((d for d in found if wanted in (d.name, d.peer_id)), None) if wanted else found[0]
    if duck is None:
        return f"no duck called {wanted!r}"
    print(f"\n{duck.label()}\n  peer {duck.peer_id}\n")
    if duck.busy:
        return "it is busy: one consumer at a time, and the robot's own console counts"

    # Shorter than a script's, because a person is watching this one.
    rpc = Rpc(timeout=15)
    consumer = WsConsumer(credential, duck.peer_id, rpc, label="microduck-sdk/wire-check")
    try:
        consumer.start()
    except WireError as e:
        return f"\ncould not open the lane: {e}"

    try:
        for method in ("robot.policies", "robot.skills"):
            try:
                print(f"\n{method}:\n  {rpc.call(method)}")
            except RpcError as e:
                if "no answer in" in e.message:
                    print(
                        f"\n{method}: no answer.\n\n"
                        "  The envelope crossed the rendezvous (the session opened, so the robot\n"
                        "  is listening) and nothing answered it. That is a `mediad` without the\n"
                        "  control lane: it reads a `peer` envelope carrying `rpc` as a step of a\n"
                        "  negotiation it has no session for, and drops it. Install this branch\n"
                        "  on the board:\n\n"
                        f"    scripts/dev-push.sh --name {duck.name}\n"
                    )
                    return 1
                print(f"\n{method}: refused, {e.message}")
    finally:
        consumer.stop()
        print("\ndisconnected")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
