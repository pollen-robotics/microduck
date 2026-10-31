"""One duck, over the rendezvous control lane (`remote-access-design.md` §3.8)."""

from __future__ import annotations

import json
import math
import os
import time
from typing import Any, Callable, Iterator

from . import rendezvous
from .control import Rpc, RpcError
from .rendezvous import DEFAULT_CENTRAL_URL, RendezvousError, Robot
from .view import View
from .wire import WsConsumer

__all__ = ["Duck", "receive"]

# `NOTIFICATIONS_PER_WINDOW` in `mediad/src/relay.rs`.
LANE_NOTIFICATIONS_PER_MINUTE = 400


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
        self._lane.start()

    def close(self) -> None:
        self._lane.stop()

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
        """Start `robot.state`. Keep it low: past the lane's budget the robot drops them."""
        return self._rpc.call("robot.subscribe", {"hz": hz})

    def video(self) -> dict[str, Any]:
        return self._rpc.call("media.video")

    def move(self, vx: float = 0.0, vy: float = 0.0, vyaw: float = 0.0) -> bool:
        """Walk, m/s and rad/s. A notification, so it waits for nothing; the deadman stops the
        robot when they stop arriving."""
        return self._rpc.notify("robot.move", {"vx": vx, "vy": vy, "vyaw": vyaw})

    def stop(self) -> Any:
        return self._rpc.call("robot.stop")

    def look(self, x: float, y: float, z: float) -> Any:
        """Point the camera at a trunk-frame point, metres: x forward, y left, z up."""
        return self._rpc.call("robot.look", {"x": x, "y": y, "z": z})

    def head(
        self,
        head_yaw: float = 0.0,
        head_pitch: float = 0.0,
        head_roll: float = 0.0,
        neck_pitch: float = 0.0,
    ) -> bool:
        """Pose the head by joint angle, radians. A notification, like `move`."""
        return self._rpc.notify(
            "robot.head",
            {
                "head_yaw": head_yaw,
                "head_pitch": head_pitch,
                "head_roll": head_roll,
                "neck_pitch": neck_pitch,
            },
        )

    def enable(self, on: bool = True) -> Any:
        return self._rpc.call("robot.enable", {"on": on})

    def do(self, skill: str) -> Any:
        return self._rpc.call("robot.do", {"skill": skill})

    def stream_frames(
        self,
        url: str,
        fps: float | None = None,
        longest: int | None = None,
        encoding: str | None = None,
    ) -> Any:
        """`media.stream`: the robot dials `url` and pushes frames there. `receive` is the far end.

        Only where the robot can reach `url`, a Space or a server with its own address.
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

        Each stream is asked for at the tick rate held under the lane's budget, so a tick may
        repeat the reading before it. Depth is off by default: `tofd` sends every frame, 15 a
        second, and the robot posts lane messages one at a time, so today it holds every answer
        behind it.
        """
        hz = _lane_rate(fps, 2 if depth else 1)
        self.subscribe(hz)
        if depth:
            try:
                self._rpc.call("tof.stream", {"hz": hz})
            except RpcError:
                pass  # no ToF fitted
        return _ticks(self._rpc, fps)


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
    period = 1.0 / fps if fps > 0 else 0.0
    tick = 0
    while True:
        tick += 1
        yield View(
            state=rpc.notifications.get("robot.state"),
            depth=rpc.notifications.get("tof.frame"),
            elapsed=time.monotonic() - started,
            tick=tick,
        )
        if period:
            time.sleep(period)


def receive(port: int, on_picture: Callable[[Any], None], host: str = "0.0.0.0") -> None:
    """Serve the socket `stream_frames` points a robot at, handing each picture to `on_picture`.

    The decoder is chosen from the robot's hello, JPEG or H.264. Pictures are RGB arrays. Needs
    the `frames` extra, and blocks.
    """
    from websockets.sync.server import serve

    from .frames import decoder_for

    def handler(connection: Any) -> None:
        decode: Callable[[bytes], list[Any]] | None = None
        for message in connection:
            if isinstance(message, str):
                try:
                    decode = decoder_for(json.loads(message))
                except ValueError:
                    continue
            elif decode is not None:
                for picture in decode(message):
                    on_picture(picture)

    with serve(handler, host, port) as server:
        server.serve_forever()
