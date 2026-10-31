"""One duck, over the rendezvous control lane (`remote-access-design.md` §3.8)."""

from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
from typing import Any, Callable, Iterator

from . import rendezvous
from .control import Rpc, RpcError
from .rendezvous import DEFAULT_CENTRAL_URL, RendezvousError, Robot
from .view import View
from .wire import WireError, WsConsumer

__all__ = ["Duck", "receive"]

logger = logging.getLogger(__name__)

# `NOTIFICATIONS_PER_WINDOW` in `mediad/src/relay.rs`.
LANE_NOTIFICATIONS_PER_MINUTE = 400

# `robotd` zeroes the twist 500 ms after the last intent (`SafetyConfig::deadman`), less than a
# round trip through the rendezvous, so the client resends it until it is this old here.
HOLD = 1.0
RESEND = 0.15

# A tick per call is a request per tick, and the rendezvous allows about 20 a second per token.
MAX_FPS = 10.0


class Duck:
    """A connected duck, found by name on the account the token belongs to.

    With no name, the one duck online on that account. Blocking: answers and notifications are
    read on the lane's own thread.
    """

    def __init__(
        self,
        name: str | None = None,
        token: str | None = None,
        timeout: float = 30.0,
        label: str = "microduck-sdk",
        base: str = DEFAULT_CENTRAL_URL,
    ):
        token = token or _token()
        if not token:
            raise RendezvousError(
                "no Hugging Face token: pass token=, set HF_TOKEN, or run `hf auth login`"
            )
        found, _ = rendezvous.ducks(token, base=base)
        self.robot: Robot = _choose(found, name)
        self._rpc = Rpc(timeout=timeout)
        self._lane = WsConsumer(token, self.robot.peer_id, self._rpc, base=base, label=label)
        try:
            self._lane.start()
        except BaseException:
            self._lane.stop()
            raise

        self._streams: set[str] = set()
        self._twist = (0.0, 0.0, 0.0)
        self._held_until = 0.0
        self._closing = False
        self._intent_error: WireError | None = None
        self._wake = threading.Condition()
        self._sending = threading.Lock()
        self._sender = threading.Thread(target=self._keep_moving, name="duck-intents", daemon=True)
        self._sender.start()

    def close(self) -> None:
        with self._wake:
            self._closing = True
            self._wake.notify()
        self._sender.join(timeout=5)
        self._lane.stop()
        self._rpc.abandon("the duck was closed")

    def __enter__(self) -> "Duck":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def call(self, method: str, params: dict[str, Any] | None = None) -> Any:
        """Any method the robot routes to a remote peer."""
        return self._rpc.call(method, params)

    def health(self) -> dict[str, Any]:
        return self._rpc.call("robot.health")

    def state(self) -> dict[str, Any] | None:
        """The last `robot.state` that arrived after `subscribe`, or `None`."""
        return self._rpc.notifications.get("robot.state")

    def subscribe(self, hz: int = 2) -> Any:
        """Start `robot.state`, once per duck. Keep it low: past the lane's budget the robot drops
        them."""
        return self._stream("robot.subscribe", {"hz": hz})

    def video(self) -> dict[str, Any]:
        return self._rpc.call("media.video")

    def move(self, vx: float = 0.0, vy: float = 0.0, vyaw: float = 0.0) -> None:
        """Walk, m/s and rad/s, until `stop()` or for `HOLD` seconds after the last call.

        The twist is resent from a thread of its own, so the robot's deadman stays fed across the
        rendezvous. A script that stops calling stops the robot about a second and a half later.
        """
        with self._wake:
            failed, self._intent_error = self._intent_error, None
            if failed is not None:
                raise RpcError("robot.move", {"message": str(failed)})
            self._twist = (vx, vy, vyaw)
            self._held_until = time.monotonic() + HOLD
            self._wake.notify()

    def stop(self) -> Any:
        with self._wake:
            self._held_until = 0.0
        # An intent already on its way lands before the stop, not after it.
        with self._sending:
            pass
        return _accepted("robot.stop", self._rpc.call("robot.stop"))

    def look(self, x: float, y: float, z: float) -> Any:
        """Point the camera at a trunk-frame point, metres: x forward, y left, z up."""
        return self._rpc.call("robot.look", {"x": x, "y": y, "z": z})

    def head(
        self,
        head_yaw: float = 0.0,
        head_pitch: float = 0.0,
        head_roll: float = 0.0,
        neck_pitch: float = 0.0,
    ) -> None:
        """Pose the head by joint angle, radians. A notification, as continuous intents are."""
        self._rpc.notify(
            "robot.head",
            {
                "head_yaw": head_yaw,
                "head_pitch": head_pitch,
                "head_roll": head_roll,
                "neck_pitch": neck_pitch,
            },
        )

    def enable(self, on: bool = True) -> Any:
        """Hand the robot to its policy, or take it back. Raises `RpcError` when refused."""
        return _accepted("robot.enable", self._rpc.call("robot.enable", {"on": on}))

    def do(self, skill: str) -> Any:
        """Run a one-shot skill by name. Raises `RpcError` when refused."""
        return _accepted("robot.do", self._rpc.call("robot.do", {"skill": skill}))

    def stream_frames(
        self,
        url: str,
        fps: float | None = None,
        longest: int | None = None,
        encoding: str | None = None,
    ) -> Any:
        """`media.stream`: the robot dials `url` and pushes frames there. `receive` is the far end.

        Only where the robot can reach `url`, a Space or a server with its own address. The robot
        sends its owner's Hugging Face token in that connection's `Authorization` header, so a
        receiver can tell whose camera it is: point it only at a server you trust with it.
        """
        params: dict[str, Any] = {"url": url}
        if fps is not None:
            params["fps"] = fps
        if longest is not None:
            params["longest"] = longest
        if encoding is not None:
            params["encoding"] = encoding
        return self._rpc.call("media.stream", params)

    def stop_frames(self) -> Any:
        return self._rpc.call("media.stream", {"url": None})

    def watch(self, fps: float = 2.0, depth: bool = False) -> Iterator[View]:
        """One `View` per tick with the latest state, and depth if asked, `fps` ticks a second.

        State is asked for at the tick rate held under the lane's budget, so a tick may repeat the
        reading before it. Depth is off by default: `tofd` sends every frame, 15 a second, and the
        robot posts lane messages one at a time, so today it holds every answer behind it.
        """
        if not 0 < fps <= MAX_FPS:
            raise ValueError(f"fps must be above 0 and at most {MAX_FPS:g}, not {fps}")
        self.subscribe(_lane_rate(fps, 2 if depth else 1))
        if depth:
            self._stream("tof.stream")
        return _ticks(self._rpc, fps)

    def _stream(self, method: str, params: dict[str, Any] | None = None) -> Any:
        """Start a stream once. A second request on its connection is never answered."""
        if method in self._streams:
            return None
        answer = self._rpc.call(method, params)
        self._streams.add(method)
        return answer

    def _keep_moving(self) -> None:
        while True:
            with self._wake:
                while not self._closing and time.monotonic() >= self._held_until:
                    self._wake.wait()
                if self._closing:
                    return
            started = time.monotonic()
            with self._sending:
                # Read again under the lock `stop` waits on, so a stop is never followed by a move.
                with self._wake:
                    if self._closing or time.monotonic() >= self._held_until:
                        continue
                    vx, vy, vyaw = self._twist
                try:
                    self._lane.send_intent(
                        {
                            "jsonrpc": "2.0",
                            "method": "robot.move",
                            "params": {"vx": vx, "vy": vy, "vyaw": vyaw},
                        }
                    )
                except WireError as e:
                    logger.warning("an intent could not be posted: %s", e)
                    with self._wake:
                        self._intent_error = e
                        self._held_until = 0.0
                    continue
            time.sleep(max(0.0, RESEND - (time.monotonic() - started)))


def _accepted(method: str, answer: Any) -> Any:
    if isinstance(answer, dict) and answer.get("accepted") is False:
        raise RpcError(method, {"message": answer.get("reason") or "refused"})
    return answer


def _token() -> str | None:
    token = os.environ.get("HF_TOKEN")
    if token:
        return token
    try:
        from huggingface_hub import get_token
    except ImportError:
        return None
    return get_token()


def _choose(found: list[Robot], name: str | None) -> Robot:
    if name:
        robot = next((d for d in found if name in (d.name, d.peer_id)), None)
        if robot is None:
            listed = ", ".join(d.name for d in found) or "none"
            raise RendezvousError(f"no duck called {name!r} on this account; listed: {listed}")
    elif len(found) == 1:
        robot = found[0]
    elif not found:
        raise RendezvousError("no ducks online for this account")
    else:
        names = ", ".join(d.name for d in found)
        raise RendezvousError(f"this account has several ducks online, name one: {names}")
    if robot.busy:
        raise RendezvousError(
            f"{robot.name} is busy with {robot.active_app or 'something else'}. One consumer at a "
            "time, and the robot's own console counts."
        )
    return robot


def _lane_rate(fps: float, streams: int) -> int:
    """Per-stream rate: the caller's, held under three quarters of the lane's budget."""
    ceiling = max(1, int(LANE_NOTIFICATIONS_PER_MINUTE * 0.75 / 60 / streams))
    return max(1, min(math.ceil(fps), ceiling))


def _ticks(rpc: Rpc, fps: float) -> Iterator[View]:
    started = time.monotonic()
    period = 1.0 / fps
    tick = 0
    while True:
        tick += 1
        yield View(
            state=rpc.notifications.get("robot.state"),
            depth=rpc.notifications.get("tof.frame"),
            elapsed=time.monotonic() - started,
            tick=tick,
        )
        time.sleep(period)


def receive(port: int, on_picture: Callable[[Any], None], host: str = "0.0.0.0") -> None:
    """Serve the socket `stream_frames` points a robot at, handing each picture to `on_picture`.

    The decoder is chosen from the robot's hello, JPEG or H.264. Pictures are RGB arrays. A frame
    that fails to decode, or that `on_picture` raises on, is logged and skipped. Needs the `frames`
    extra, and blocks.
    """
    from websockets.sync.server import serve

    from .frames import decoder_for

    def handler(connection: Any) -> None:
        decode: Callable[[bytes], list[Any]] | None = None
        for message in connection:
            if isinstance(message, str):
                try:
                    decode = decoder_for(json.loads(message))
                except ValueError as e:
                    logger.warning("a hello this cannot use, so its frames are dropped: %s", e)
                    decode = None
                continue
            if decode is None:
                continue
            try:
                for picture in decode(message):
                    on_picture(picture)
            except Exception as e:  # noqa: BLE001 - one bad frame is not a bad stream
                logger.warning("a frame was skipped: %r", e)

    with serve(handler, host, port) as server:
        server.serve_forever()
