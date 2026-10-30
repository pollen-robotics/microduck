"""Checks for the frame receiver's bookkeeping: `python test_receiver.py`, or pytest.

No robot, no network: the route is the real one, driven by FastAPI's test client (which needs
`httpx`), and `whoami` is replaced so a made-up token resolves to an account. This is the only
test file that imports `receiver`, so the replacement is left in place.
"""

import contextlib
import json
import time

import cv2
import numpy as np
from fastapi import FastAPI
from fastapi.testclient import TestClient

import receiver

TOKEN = {"authorization": "Bearer robot-token"}
JPEG = cv2.imencode(".jpg", np.full((48, 64, 3), 128, np.uint8))[1].tobytes()


def hello(name: str) -> str:
    return json.dumps({"robot": {"name": name}, "frames": {"encoding": "jpeg"}})


def client() -> TestClient:
    receiver.FRAMES = receiver.Frames()
    receiver.whoami = lambda token: "eric" if token == "robot-token" else None
    app = FastAPI()
    receiver.mount(app)
    return TestClient(app)


def until(condition, timeout: float = 2.0) -> bool:
    """Whether `condition()` became true within `timeout`: the route runs on another thread."""
    end = time.monotonic() + timeout
    while not condition():
        if time.monotonic() > end:
            return False
        time.sleep(0.01)
    return True


def frames() -> dict[str, int]:
    return {s.robot: s.frames for s in receiver.FRAMES.of("eric")}


def test_a_closed_stream_is_forgotten():
    c = client()
    with c.websocket_connect("/frames", headers=TOKEN) as robot:
        robot.send_text(hello("duck-a"))
        robot.send_bytes(JPEG)
        assert until(lambda: frames() == {"duck-a": 1})
    assert until(lambda: frames() == {})


def test_a_late_close_of_the_previous_connection_keeps_the_new_one():
    # A robot redials before this end has seen its previous connection go: a second
    # `media.stream`, whose old socket closes only when its encoder thread next looks, or a link
    # that dropped without a close. The old connection's teardown must not take the new one's
    # stream with it.
    c = client()
    with contextlib.ExitStack() as later:
        with c.websocket_connect("/frames", headers=TOKEN) as old:
            old.send_text(hello("duck-a"))
            old.send_bytes(JPEG)
            assert until(lambda: frames() == {"duck-a": 1})
            first = receiver.FRAMES.newest("eric")

            new = later.enter_context(c.websocket_connect("/frames", headers=TOKEN))
            new.send_text(hello("duck-a"))
            assert until(lambda: receiver.FRAMES.newest("eric") is not first)
        # The old connection is closed now, and its teardown has had time to run.
        time.sleep(0.2)
        new.send_bytes(JPEG)
        assert until(lambda: frames() == {"duck-a": 1}), (
            f"the old connection's close dropped the live stream: {frames()}"
        )


def test_a_connection_that_never_said_hello_leaves_the_live_stream_alone():
    # Units with no hello are filed under "a duck" (and decoded as H.264, so a JPEG here is only
    # counted, never shown), which is also the name a closing connection fell back to when it had
    # registered nothing.
    c = client()
    with c.websocket_connect("/frames", headers=TOKEN) as robot:
        robot.send_bytes(JPEG)
        assert until(lambda: receiver.FRAMES.newest("eric") is not None)
        live = receiver.FRAMES.newest("eric")
        assert live.robot == "a duck"
        with c.websocket_connect("/frames", headers=TOKEN):
            pass
        time.sleep(0.2)
        assert receiver.FRAMES.newest("eric") is live, "a connection that said nothing removed it"


def test_closing_one_duck_leaves_the_other():
    c = client()
    with c.websocket_connect("/frames", headers=TOKEN) as b:
        b.send_text(hello("duck-b"))
        b.send_bytes(JPEG)
        with c.websocket_connect("/frames", headers=TOKEN) as a:
            a.send_text(hello("duck-a"))
            a.send_bytes(JPEG)
            assert until(lambda: frames() == {"duck-a": 1, "duck-b": 1})
        assert until(lambda: frames() == {"duck-b": 1})


if __name__ == "__main__":
    for name, test in list(globals().items()):
        if name.startswith("test_"):
            test()
            print("ok", name)
