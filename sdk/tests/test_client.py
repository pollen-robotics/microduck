"""What a caller can rely on, without a robot.

The wire is checked against a fake socket rather than a duck: these are about the shape of what
goes out and what comes back, and the half that needs a real `mediad` is `README.md`'s example.
"""

import json
import threading

import pytest

from duck.rpc import Rpc, RpcError


def bound() -> tuple[Rpc, list]:
    """An `Rpc` writing into a list instead of a socket."""
    rpc = Rpc(timeout=2.0)
    sent: list = []
    rpc.bound_to(lambda message: (sent.append(message), True)[1])
    return rpc, sent


def test_a_call_goes_out_as_jsonrpc_and_its_answer_comes_back():
    rpc, sent = bound()

    def answer():
        while not sent:
            pass
        rpc.on_message(json.dumps({"jsonrpc": "2.0", "id": sent[0]["id"], "result": {"ok": True}}))

    threading.Thread(target=answer, daemon=True).start()
    assert rpc.call("robot.health") == {"ok": True}
    assert sent[0]["method"] == "robot.health"
    assert sent[0]["jsonrpc"] == "2.0"


def test_a_refusal_names_the_method_it_refused():
    """A reply carries an id and nothing else, so the method has to be remembered here — and
    "robot.setMode: not available over this transport" is a better sentence than "call:"."""
    rpc, sent = bound()

    def refuse():
        while not sent:
            pass
        rpc.on_message(
            json.dumps(
                {
                    "jsonrpc": "2.0",
                    "id": sent[0]["id"],
                    "error": {"code": -32601, "message": "not available over this transport"},
                }
            )
        )

    threading.Thread(target=refuse, daemon=True).start()
    with pytest.raises(RpcError) as refused:
        rpc.call("robot.setMode", {"mode": "roller"})
    assert "robot.setMode" in str(refused.value)


def test_notifications_are_kept_by_method_and_never_answer_a_call():
    """`robot.state` streams. A caller reads the last one; nothing here waits for it."""
    rpc, _ = bound()
    rpc.on_message(json.dumps({"jsonrpc": "2.0", "method": "robot.state", "params": {"t": 1.0}}))
    rpc.on_message(json.dumps({"jsonrpc": "2.0", "method": "robot.state", "params": {"t": 2.0}}))
    assert rpc.notifications["robot.state"] == {"t": 2.0}


def test_a_closed_socket_fails_everything_in_flight():
    """A promise nobody settles is a leak, so a dropped connection fails the callers rather than
    leaving each to time out on its own."""
    rpc, sent = bound()
    failed: list = []

    def call():
        try:
            rpc.call("robot.health")
        except RpcError as e:
            failed.append(e)

    caller = threading.Thread(target=call, daemon=True)
    caller.start()
    while not sent:
        pass
    rpc.abandon("the robot closed the connection")
    caller.join(timeout=3)
    assert failed and "closed" in str(failed[0])
