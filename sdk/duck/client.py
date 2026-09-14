"""One duck, over the agent WebSocket.

The transport is the thin part: a background thread owns the socket, `Rpc` owns the id space and
the pending table, and everything below is one call each. What is *not* thin is which calls exist
— those are the ones `mediad::route` permits, and a method missing here is a method the robot
would refuse anyway.
"""

from __future__ import annotations

import json
import threading
import time
from typing import Any, Callable, Iterator

from websockets.sync.client import connect

from .rpc import Rpc, RpcError
from .view import View

__all__ = ["Duck", "RpcError", "View"]

#: Where `mediad` serves the console, and the agent socket beside it.
DEFAULT_PORT = 8080


class Duck:
    """A connected duck.

    Blocking, on purpose: a script that fetches a frame and sends an intent has nothing else to do
    while it waits, and `asyncio` in front of that is a tax on the simplest possible caller. The
    socket is read on its own thread so notifications — `robot.state`, `media.detections` — arrive
    while a call is in flight rather than behind it.
    """

    def __init__(self, host: str, port: int = DEFAULT_PORT, timeout: float = 30.0):
        self.url = f"ws://{host}:{port}/agent"
        self._rpc = Rpc(timeout=timeout)
        self._socket = connect(self.url)
        self._closing = threading.Event()
        self._rpc.bound_to(self._send)
        self._reader = threading.Thread(target=self._read, name="duck-agent", daemon=True)
        self._reader.start()

    # ── the transport ────────────────────────────────────────────────────────

    def _send(self, message: dict[str, Any]) -> bool:
        try:
            self._socket.send(json.dumps(message))
            return True
        except Exception:
            return False

    def _read(self) -> None:
        try:
            for message in self._socket:
                self._rpc.on_message(message)
        except Exception as e:
            # A closed socket during `close()` is the ordinary way this ends, and failing every
            # call in flight over it would be a lie.
            if not self._closing.is_set():
                self._rpc.abandon(str(e))
            return
        if not self._closing.is_set():
            self._rpc.abandon("the robot closed the connection")

    def close(self) -> None:
        self._closing.set()
        self._socket.close()

    def __enter__(self) -> "Duck":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    # ── asking ───────────────────────────────────────────────────────────────

    def call(self, method: str, **params: Any) -> Any:
        """Any method the robot routes to this transport, for the ones without a wrapper below."""
        return self._rpc.call(method, params or None)

    def health(self) -> dict[str, Any]:
        """Is the robot alright — the loop's rate, the bus, the battery, the motors."""
        return self._rpc.call("robot.health")

    def state(self) -> dict[str, Any] | None:
        """The last `robot.state` that arrived, or `None` before the first.

        A read of what has already been pushed rather than a call: `subscribe` starts the stream
        and this is where it lands.
        """
        return self._rpc.notifications.get("robot.state")

    def subscribe(self, hz: int | None = None) -> Any:
        """Start the `robot.state` stream, so `state()` has something to answer with."""
        return self._rpc.call("robot.subscribe", {"hz": hz} if hz else {})

    def video(self) -> dict[str, Any]:
        """What the camera is sending: the frame size, and how far it is mounted from upright."""
        return self._rpc.call("media.video")

    # ── telling ──────────────────────────────────────────────────────────────

    def move(self, vx: float = 0.0, vy: float = 0.0, vyaw: float = 0.0) -> Any:
        """Walk. Metres per second forward and left, radians per second about up.

        One call is one intent, and `robotd`'s deadman zeroes the twist when they stop arriving —
        so a script that wants the robot to keep walking has to keep saying so, which is the
        property that makes a crashed script a robot that stops.
        """
        return self._rpc.call("robot.move", {"vx": vx, "vy": vy, "vyaw": vyaw})

    def stop(self) -> Any:
        """Zero the twist now rather than waiting for the deadman."""
        return self._rpc.call("robot.stop")

    def look(self, x: float, y: float, z: float) -> Any:
        """Point the camera at a trunk-frame point: x forward, y left, z up, metres."""
        return self._rpc.call("robot.look", {"x": x, "y": y, "z": z})

    def head(self, yaw: float = 0.0, pitch: float = 0.0, roll: float = 0.0) -> Any:
        """Pose the head directly, radians."""
        return self._rpc.call("robot.head", {"yaw": yaw, "pitch": pitch, "roll": roll})

    def enable(self, on: bool = True) -> Any:
        """Hand the robot to its policy, or take it back."""
        return self._rpc.call("robot.enable", {"on": on})

    def do(self, skill: str) -> Any:
        """Run a one-shot skill by name — whatever `robot.health` says this robot has."""
        return self._rpc.call("robot.do", {"skill": skill})

    # ── frames ───────────────────────────────────────────────────────────────

    def frames(self, url: str, fps: float | None = None, longest: int | None = None) -> Any:
        """Tell the robot to send JPEG frames to a WebSocket it dials.

        **Outbound, and that is the point.** The robot is behind somebody's router and the script
        may be anywhere; a robot that dials out needs no relay candidate and no NAT traversal.
        `stream.rs` has the argument. `receive` below is the other end of it.
        """
        params: dict[str, Any] = {"url": url}
        if fps is not None:
            params["fps"] = fps
        if longest is not None:
            params["longest"] = longest
        return self._rpc.call("media.stream", params)

    def frames_stop(self) -> Any:
        """Stop streaming."""
        return self._rpc.call("media.stream", {"url": None})

    def frames_status(self) -> Any:
        """What is streaming, if anything."""
        return self._rpc.call("media.stream")

    # ── the loop ─────────────────────────────────────────────────────────────

    def watch(
        self,
        fps: float = 2.0,
        camera: bool = True,
        depth: bool = True,
        frame_port: int = 8099,
    ) -> "Iterator[View]":
        """Yield one [`View`] per tick, with whatever each stream sent most recently.

        This is the loop a robot program is:

            for view in duck.watch():
                if view.nearest and view.nearest < 0.3:
                    duck.stop()

        It subscribes to `robot.state`, starts the depth stream, and tells the robot to send
        frames to a socket it opens here — then merges the three and hands over the latest of
        each. Everything is turned off again when the loop ends, including when it ends because
        the caller raised.

        `camera=False` skips the frames, which is what a behaviour that only needs state and
        depth wants: the robot stops encoding JPEGs for nobody, and no inbound port is opened.

        **The rate is the behaviour's, not the sensors'.** `robot.state` arrives at the control
        rate and depth at 15 Hz; ticking at those would make the loop the fastest thing rather
        than the one deciding. `fps` is how often the caller wants to think.
        """
        return _watch(self, fps=fps, camera=camera, depth=depth, frame_port=frame_port)


def receive(port: int, on_frame: Callable[[bytes], None], host: str = "0.0.0.0") -> None:
    """Listen for the frames a duck was told to send, and hand each JPEG to `on_frame`.

    The half a script would otherwise have to write itself, and the reason `frames()` is usable
    without a Space: `mediad` sends one text message describing what is coming and then one binary
    message per frame, so everything here is "ignore the first kind, pass on the second".

    Blocks. Runs until the socket closes or the callback raises.
    """
    from websockets.sync.server import serve

    def handler(connection: Any) -> None:
        for message in connection:
            # The opening text frame says the size and the rotation; the rest are JPEGs.
            if isinstance(message, bytes):
                on_frame(message)

    with serve(handler, host, port) as server:
        server.serve_forever()


def _watch(
    duck: Duck, fps: float, camera: bool, depth: bool, frame_port: int
) -> Iterator[View]:
    """[`Duck.watch`]'s body, as a generator so its `finally` runs when the caller stops."""
    latest: dict[str, Any] = {"frame": None}
    started = time.monotonic()

    # A notification lands on the reader thread. Nothing here locks: each of these is one
    # assignment of one reference, and a tick reading a field mid-swap gets the old value or the
    # new one, never half of either.
    def on_frame(jpeg: bytes) -> None:
        latest["frame"] = jpeg

    server = None
    if camera:
        server = _FrameServer(frame_port, on_frame)
        server.start()
        duck.frames(url=f"ws://{_address_the_robot_can_reach(duck.url)}:{frame_port}", fps=fps)

    duck.subscribe()
    if depth:
        # `tofd` streams to whoever asked; the notifications land in `Rpc.notifications` beside
        # `robot.state`, so there is nothing further to wire up.
        try:
            duck.call("tof.stream")
        except RpcError:
            # A duck with no ToF fitted refuses this, and that is not a reason to stop: a
            # behaviour that wanted depth gets `None` and can say so itself.
            pass

    tick = 0
    period = 1.0 / fps if fps > 0 else 0.0
    try:
        while True:
            tick += 1
            yield View(
                frame=latest["frame"],
                state=duck._rpc.notifications.get("robot.state"),
                depth=duck._rpc.notifications.get("tof.frame"),
                elapsed=time.monotonic() - started,
                tick=tick,
            )
            if period:
                time.sleep(period)
    finally:
        # Whatever ended the loop — a `break`, an exception, the caller simply stopping — the
        # robot should not be left encoding frames for a socket that has gone.
        if camera:
            try:
                duck.frames_stop()
            except RpcError:
                pass
        if server is not None:
            server.stop()


class _FrameServer:
    """The socket the robot dials, run on a thread so the loop above stays in charge."""

    def __init__(self, port: int, on_frame: Callable[[bytes], None]):
        self._port = port
        self._on_frame = on_frame
        self._server: Any = None
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        from websockets.sync.server import serve

        ready = threading.Event()

        def run() -> None:
            def handler(connection: Any) -> None:
                for message in connection:
                    if isinstance(message, bytes):
                        self._on_frame(message)

            with serve(handler, "0.0.0.0", self._port) as server:
                self._server = server
                ready.set()
                server.serve_forever()

        self._thread = threading.Thread(target=run, name="duck-frames", daemon=True)
        self._thread.start()
        # Bound before telling the robot where to dial, or the first connection is refused and
        # `stream.rs` backs off before anybody is listening.
        ready.wait(timeout=5)

    def stop(self) -> None:
        if self._server is not None:
            self._server.shutdown()


def _address_the_robot_can_reach(url: str) -> str:
    """Our address on the route to the robot.

    Not `localhost`: the robot dials this, so it has to be the address *it* would use, which on
    any machine with more than one interface is not something a caller should have to work out.
    """
    import socket
    from urllib.parse import urlparse

    host = urlparse(url).hostname or "127.0.0.1"
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect((host, 80))
        return str(probe.getsockname()[0])
    finally:
        probe.close()
