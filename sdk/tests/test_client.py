"""Against a fake rendezvous whose robot echoes every call back."""

from __future__ import annotations

import json
import queue
import re
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest

from duck import Duck, RendezvousError, RpcError
from duck.client import LANE_NOTIFICATIONS_PER_MINUTE, _choose, _lane_rate
from duck.control import Rpc
from duck.rendezvous import Robot

REPO = Path(__file__).resolve().parents[2]


class FakeRendezvous:
    def __init__(self, robots: list[dict]):
        self.robots = robots
        self.events: queue.Queue = queue.Queue()
        self.posts: list[dict] = []
        self.bearers: list[str] = []
        self.calls: list[dict] = []
        fake = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, *_):
                pass

            def do_GET(self):
                fake.bearers.append(self.headers.get("authorization", ""))
                if self.path == "/api/robot-status":
                    self._json({"robots": fake.robots})
                elif self.path == "/events":
                    self.send_response(200)
                    self.send_header("content-type", "text/event-stream")
                    self.send_header("transfer-encoding", "chunked")
                    self.end_headers()
                    self._event({"type": "welcome", "peerId": "consumer-1", "username": "u"})
                    while True:
                        message = fake.events.get()
                        if message is None:
                            break
                        try:
                            self._event(message)
                        except OSError:
                            break

            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers["content-length"])))
                fake.posts.append(body)
                if body["type"] == "startSession":
                    self._json({"type": "sessionStarted", "sessionId": "s-1"})
                    return
                self._json({"status": "ok"})
                if body["type"] == "peer" and "rpc" in body:
                    call = body["rpc"]
                    fake.calls.append(call)
                    if "id" not in call:
                        return
                    answer = {"method": call["method"], "params": call.get("params")}
                    fake.events.put(
                        {
                            "type": "peer",
                            "sessionId": "s-1",
                            "rpc": {"jsonrpc": "2.0", "id": call["id"], "result": answer},
                        }
                    )

            def _json(self, value):
                raw = json.dumps(value).encode()
                self.send_response(200)
                self.send_header("content-type", "application/json")
                self.send_header("content-length", str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)

            def _event(self, value):
                raw = f"data: {json.dumps(value)}\n\n".encode()
                self.wfile.write(f"{len(raw):x}\r\n".encode() + raw + b"\r\n")
                self.wfile.flush()

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.url = f"http://127.0.0.1:{self.server.server_address[1]}"
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def notify(self, method: str, params: dict) -> None:
        self.events.put(
            {
                "type": "peer",
                "sessionId": "s-1",
                "rpc": {"jsonrpc": "2.0", "method": method, "params": params},
            }
        )

    def close(self) -> None:
        self.events.put(None)
        self.server.shutdown()


def listed(name: str, peer: str, busy: bool = False, kind: str = "microduck") -> dict:
    return {"peerId": peer, "busy": busy, "meta": {"name": name, "kind": kind}}


@pytest.fixture
def rendezvous():
    fake = FakeRendezvous([listed("duck-a", "peer-a"), listed("mini", "peer-m", kind="reachy")])
    yield fake
    fake.close()


def test_a_duck_found_by_name_answers_over_the_lane(rendezvous):
    with Duck("duck-a", token="hf_test", base=rendezvous.url, timeout=5) as duck:
        assert duck.health() == {"method": "robot.health", "params": {}}

    started = [p for p in rendezvous.posts if p["type"] == "startSession"]
    assert started == [{"type": "startSession", "peerId": "peer-a"}]
    assert set(rendezvous.bearers) == {"Bearer hf_test"}
    assert any(p["type"] == "endSession" for p in rendezvous.posts)


def test_closing_does_not_wait_for_the_services_next_ping(rendezvous):
    """The real service pings every 30 s, and the reader is blocked in a read until then."""
    duck = Duck("duck-a", token="hf_test", base=rendezvous.url, timeout=5)
    closer = threading.Thread(target=duck.close, daemon=True)
    closer.start()
    closer.join(timeout=3)
    assert not closer.is_alive(), "close() is still waiting on the event stream"


def test_the_only_duck_on_an_account_needs_no_name(rendezvous):
    with Duck(token="hf_test", base=rendezvous.url, timeout=5) as duck:
        assert duck.robot.name == "duck-a"


def test_intents_carry_the_fields_robotd_accepts(rendezvous):
    """Read off the Rust: these structs are `deny_unknown_fields`."""
    proto = (REPO / "duck-ipc-proto" / "src" / "lib.rs").read_text()

    def fields(struct: str) -> set[str]:
        body = re.search(rf"pub struct {struct} \{{(.*?)\n\}}", proto, re.S)
        assert body, struct
        return set(re.findall(r"pub (\w+):", body.group(1)))

    with Duck("duck-a", token="hf_test", base=rendezvous.url, timeout=5) as duck:
        duck.head(head_yaw=0.2)
        duck.move(vx=0.1)
        duck.look(0.5, 0.0, 0.1)

    sent = {c["method"]: set(c["params"]) for c in rendezvous.calls}
    notified = {c["method"] for c in rendezvous.calls if "id" not in c}
    assert notified == {"robot.head", "robot.move"}, "continuous intents are notifications"
    assert sent["robot.head"] == fields("HeadParams")
    assert sent["robot.move"] == fields("MoveParams")
    assert sent["robot.look"] <= fields("LookParams")


def test_watch_leaves_depth_off_unless_asked(rendezvous):
    with Duck("duck-a", token="hf_test", base=rendezvous.url, timeout=5) as duck:
        duck.watch(fps=2)
    assert "tof.stream" not in {c["method"] for c in rendezvous.calls}


def test_watch_asks_for_rates_the_lane_can_carry(rendezvous):
    with Duck("duck-a", token="hf_test", base=rendezvous.url, timeout=5) as duck:
        ticks = duck.watch(fps=20, depth=True)
        asked = {c["method"]: c["params"] for c in rendezvous.calls}
        per_minute = (asked["robot.subscribe"]["hz"] + asked["tof.stream"]["hz"]) * 60
        assert per_minute <= LANE_NOTIFICATIONS_PER_MINUTE

        rendezvous.notify("robot.state", {"safety": {"fallen": False}, "odom": {"yaw": 0.3}})
        rendezvous.notify("tof.frame", {"distance_mm": [500, 250], "status": [5, 5]})
        for view in ticks:
            if view.state and view.depth:
                break
        assert view.yaw == 0.3
        assert view.nearest == 0.25


def test_the_lane_rate_never_asks_for_more_than_the_robot_posts():
    for fps in (0.5, 1, 2, 4, 10, 50):
        for streams in (1, 2):
            rate = _lane_rate(fps, streams)
            assert rate >= 1
            assert rate * streams * 60 <= LANE_NOTIFICATIONS_PER_MINUTE


def test_choosing_a_duck_says_why_it_could_not():
    a = Robot(listed("duck-a", "peer-a"))
    b = Robot(listed("duck-b", "peer-b"))
    busy = Robot(listed("duck-c", "peer-c", busy=True) | {"activeApp": "the console"})

    with pytest.raises(RendezvousError, match="no ducks online"):
        _choose([], None)
    with pytest.raises(RendezvousError, match="duck-a, duck-b"):
        _choose([a, b], None)
    with pytest.raises(RendezvousError, match="no duck called 'duck-z'"):
        _choose([a, b], "duck-z")
    with pytest.raises(RendezvousError, match="busy with the console"):
        _choose([busy], None)
    assert _choose([a, b], "peer-b") is b


def test_no_token_is_said_before_anything_is_asked(monkeypatch):
    monkeypatch.delenv("HF_TOKEN", raising=False)
    monkeypatch.setattr("duck.client._token", lambda: None)
    with pytest.raises(RendezvousError, match="no Hugging Face token"):
        Duck()


def bound() -> tuple[Rpc, list]:
    rpc = Rpc(timeout=2.0)
    sent: list = []
    rpc.bound_to(lambda message: (sent.append(message), True)[1])
    return rpc, sent


def test_a_refusal_names_the_method_it_refused():
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


def test_a_closed_lane_fails_everything_in_flight():
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
    rpc.abandon("the session ended")
    caller.join(timeout=3)
    assert failed and "ended" in str(failed[0])
