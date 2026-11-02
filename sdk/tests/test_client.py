"""Against a fake rendezvous whose robot echoes every call back."""

from __future__ import annotations

import json
import os
import queue
import re
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest

from microduck import Duck, RendezvousError, RpcError, WireError
from microduck.client import LANE_NOTIFICATIONS_PER_MINUTE, _choose, _lane_rate
from microduck.control import Rpc
from microduck.rendezvous import Robot

REPO = Path(__file__).resolve().parents[2]


class FakeRendezvous:
    def __init__(self, robots: list[dict]):
        self.robots = robots
        self.events: queue.Queue = queue.Queue()
        self.posts: list[dict] = []
        self.bearers: list[str] = []
        self.calls: list[dict] = []
        self.answers: dict[str, dict] = {}
        self.silent: set[str] = set()
        self.reject = False
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
                    if fake.reject:
                        self._json({"type": "sessionRejected", "activeApp": "the console"})
                    else:
                        self._json({"type": "sessionStarted", "sessionId": "s-1"})
                    return
                self._json({"status": "ok"})
                if body["type"] == "peer" and "rpc" in body:
                    call = body["rpc"]
                    fake.calls.append(call)
                    if "id" not in call or call["method"] in fake.silent:
                        return
                    answer = fake.answers.get(
                        call["method"], {"method": call["method"], "params": call.get("params")}
                    )
                    if callable(answer):
                        answer = answer(call)
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
        duck.stop()

    sent = {c["method"]: set(c["params"]) for c in rendezvous.calls}
    notified = {c["method"] for c in rendezvous.calls if "id" not in c}
    assert notified == {"robot.head", "robot.move"}, "continuous intents are notifications"
    assert sent["robot.head"] == fields("HeadParams")
    assert sent["robot.move"] == fields("MoveParams")
    assert sent["robot.look"] <= fields("LookParams")


def moves(fake) -> int:
    return sum(c["method"] == "robot.move" for c in list(fake.calls))


def test_a_move_is_resent_until_stop_so_the_deadman_stays_fed(rendezvous):
    """`robotd` zeroes the twist 500 ms after the last intent, less than a round trip through the
    rendezvous, so one `move()` has to keep arriving."""
    with Duck("duck-a", token="hf_test", base=rendezvous.url, timeout=5) as duck:
        duck.move(vx=0.1)
        time.sleep(0.6)
        assert moves(rendezvous) >= 3
        duck.stop()
        after = moves(rendezvous)
        time.sleep(0.5)
        assert moves(rendezvous) == after, "nothing is resent once stopped"


def test_a_script_that_stops_calling_move_stops_the_robot(rendezvous, monkeypatch):
    monkeypatch.setattr("microduck.client.HOLD", 0.3)
    with Duck("duck-a", token="hf_test", base=rendezvous.url, timeout=5) as duck:
        duck.move(vx=0.1)
        time.sleep(0.6)
        held = moves(rendezvous)
        time.sleep(0.5)
        assert moves(rendezvous) == held


def test_a_refusal_raises_with_the_robots_reason(rendezvous):
    """`robot.enable` answers `accepted: false` rather than an error, and a script that only
    checks for exceptions would carry on as if the robot were driving."""
    rendezvous.answers["robot.enable"] = {"accepted": False, "reason": "the robot is limp"}
    with Duck("duck-a", token="hf_test", base=rendezvous.url, timeout=5) as duck:
        with pytest.raises(RpcError, match="the robot is limp"):
            duck.enable(True)


class Seat:
    """`robot.policies` and `robot.do` for a duck in its seat, as `robotd` answers them: a toggle
    is accepted whenever the policy drives, and one that lands mid-sit is dropped."""

    def __init__(self, sitting: bool, still_settling: int = 0, rises_by_itself_after=None):
        self.sitting = sitting
        self.since = time.monotonic()
        # How many toggles land while the sit is still settling, and so do nothing.
        self.still_settling = still_settling
        self.rises_by_itself_after = rises_by_itself_after
        self.toggles = 0
        self.homed = True
        self.refuse: str | None = None

    def enable(self, _call):
        # Times run from here, where the robot's own would: connecting first takes a while.
        self.since = time.monotonic()
        return {"accepted": True, "reason": "enabled, driving"}

    def policies(self, _call):
        elapsed = time.monotonic() - self.since
        if self.rises_by_itself_after is not None and elapsed >= self.rises_by_itself_after:
            self.sitting = False
        return {"enabled": True, "slots": [], "sitting": self.sitting, "homed": self.homed}

    def do(self, call):
        assert call["params"] == {"skill": "sit_toggle"}
        self.toggles += 1
        if self.refuse:
            reason, self.refuse = self.refuse, None
            return {"accepted": False, "reason": reason}
        if self.toggles > self.still_settling:
            self.sitting = False
        return {"accepted": True}

    def install(self, fake) -> None:
        fake.answers["robot.policies"] = self.policies
        fake.answers["robot.do"] = self.do
        fake.answers["robot.enable"] = self.enable


@pytest.fixture
def quick_seat(monkeypatch):
    monkeypatch.setattr("microduck.client.SEAT_SETTLE", 0.5)


def test_stand_toggles_a_duck_that_is_on_and_sitting(rendezvous, quick_seat):
    """`enable(True)` leaves a seated duck seated, so the toggle is what stands it."""
    seat = Seat(sitting=True)
    seat.install(rendezvous)
    with Duck("duck-a", token="hf_test", base=rendezvous.url, timeout=5) as duck:
        duck.stand()
    assert seat.toggles == 1 and not seat.sitting
    assert [c["method"] for c in rendezvous.calls][0] == "robot.enable"


def test_stand_sends_the_toggle_again_when_the_sit_was_still_settling(rendezvous, quick_seat):
    """The robot accepts a toggle mid-sit and then ignores it, so an accepted toggle is not
    proof the duck got up."""
    seat = Seat(sitting=True, still_settling=1)
    seat.install(rendezvous)
    with Duck("duck-a", token="hf_test", base=rendezvous.url, timeout=5) as duck:
        duck.stand()
    assert seat.toggles == 2 and not seat.sitting


def test_stand_leaves_a_duck_standing_itself_up_alone(rendezvous, quick_seat):
    """A duck turned on in its seat settles and rises by itself. A toggle on top would be a
    second stand-up, or a sit once it is up."""
    seat = Seat(sitting=True, rises_by_itself_after=0.2)
    seat.install(rendezvous)
    with Duck("duck-a", token="hf_test", base=rendezvous.url, timeout=5) as duck:
        duck.stand()
    assert seat.toggles == 0


def test_stand_never_toggles_a_standing_duck(rendezvous, quick_seat):
    seat = Seat(sitting=False)
    seat.install(rendezvous)
    with Duck("duck-a", token="hf_test", base=rendezvous.url, timeout=5) as duck:
        duck.stand()
    assert seat.toggles == 0


def test_stand_gives_up_on_a_duck_that_stays_seated(rendezvous, quick_seat):
    seat = Seat(sitting=True, still_settling=100)
    seat.install(rendezvous)
    with Duck("duck-a", token="hf_test", base=rendezvous.url, timeout=5) as duck:
        with pytest.raises(RpcError, match="still sitting"):
            duck.stand(timeout=1.2)


def test_stand_waits_out_a_refusal_that_clears_by_itself(rendezvous, quick_seat):
    seat = Seat(sitting=True)
    seat.refuse = "the robot is still going to its home pose; try again in a moment"
    seat.install(rendezvous)
    with Duck("duck-a", token="hf_test", base=rendezvous.url, timeout=5) as duck:
        duck.stand()
    assert seat.toggles == 2 and not seat.sitting


def test_stand_does_not_toggle_before_the_duck_has_homed(rendezvous, quick_seat):
    """`robot.do` refuses every skill until then, and `robot.policies` says when it is."""
    seat = Seat(sitting=True)
    seat.homed = False
    seat.install(rendezvous)
    with Duck("duck-a", token="hf_test", base=rendezvous.url, timeout=5) as duck:
        with pytest.raises(RpcError, match="still sitting"):
            duck.stand(timeout=1.2)
    assert seat.toggles == 0


def test_stand_on_a_robot_that_does_not_say_whether_it_sits_only_enables(rendezvous, caplog):
    rendezvous.answers["robot.policies"] = {"enabled": True, "slots": []}
    with Duck("duck-a", token="hf_test", base=rendezvous.url, timeout=5) as duck:
        duck.stand()
    methods = [c["method"] for c in rendezvous.calls]
    assert "robot.enable" in methods and "robot.do" not in methods
    assert "does not say whether it is sitting" in caplog.text


def test_python_m_microduck_runs_without_a_second_copy_of_a_module(rendezvous):
    """`python -m microduck.wire` warned, because the package had imported `wire` already."""
    rendezvous.answers["robot.policies"] = {"enabled": True, "slots": []}
    env = {
        **os.environ,
        "HF_TOKEN": "hf_test",
        "REACHY_CENTRAL_URL": rendezvous.url,
        "PYTHONWARNINGS": "error::RuntimeWarning",
    }
    run = subprocess.run(
        [sys.executable, "-m", "microduck", "duck-a"],
        env=env,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert run.returncode == 0, run.stderr
    assert "RuntimeWarning" not in run.stderr
    assert "duck-a" in run.stdout and "robot.policies:" in run.stdout
    assert {c["method"] for c in rendezvous.calls} == {"robot.policies", "robot.skills"}


def test_closing_fails_a_call_still_waiting(rendezvous):
    rendezvous.silent.add("robot.health")
    duck = Duck("duck-a", token="hf_test", base=rendezvous.url, timeout=30)
    failed: list = []

    def ask():
        try:
            duck.health()
        except RpcError as e:
            failed.append(e)

    caller = threading.Thread(target=ask, daemon=True)
    caller.start()
    time.sleep(0.3)
    duck.close()
    caller.join(timeout=3)
    assert failed, "the call was left to wait out its 30 s"


def test_a_duck_that_cannot_start_leaves_nothing_running(rendezvous):
    rendezvous.reject = True
    before = sum(t.name == "duck-wire" for t in threading.enumerate())
    with pytest.raises(WireError, match="busy"):
        Duck("duck-a", token="hf_test", base=rendezvous.url, timeout=5)
    time.sleep(0.5)
    assert sum(t.name == "duck-wire" for t in threading.enumerate()) == before


def test_watch_needs_a_rate_the_lane_can_take(rendezvous):
    with Duck("duck-a", token="hf_test", base=rendezvous.url, timeout=5) as duck:
        for fps in (0, -1, 50):
            with pytest.raises(ValueError):
                duck.watch(fps=fps)


def test_a_stream_is_asked_for_once(rendezvous):
    """`tofd` and `robotd` never answer a second request on a streaming connection."""
    with Duck("duck-a", token="hf_test", base=rendezvous.url, timeout=5) as duck:
        duck.watch(fps=2, depth=True)
        duck.watch(fps=2, depth=True)
    asked = [c["method"] for c in rendezvous.calls]
    assert asked.count("robot.subscribe") == 1
    assert asked.count("tof.stream") == 1


def test_watch_leaves_depth_off_unless_asked(rendezvous):
    with Duck("duck-a", token="hf_test", base=rendezvous.url, timeout=5) as duck:
        duck.watch(fps=2)
    assert "tof.stream" not in {c["method"] for c in rendezvous.calls}


def test_watch_asks_for_rates_the_lane_can_carry(rendezvous):
    with Duck("duck-a", token="hf_test", base=rendezvous.url, timeout=5) as duck:
        ticks = duck.watch(fps=10, depth=True)
        asked = {c["method"]: c["params"] for c in rendezvous.calls}
        assert asked["robot.subscribe"]["hz"] * 2 * 60 <= LANE_NOTIFICATIONS_PER_MINUTE
        assert asked["tof.stream"] == {}, "tof.stream takes no parameters"

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
    monkeypatch.setattr("microduck.client._token", lambda: None)
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
