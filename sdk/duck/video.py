"""The duck's camera over WebRTC, on the session the control lane already holds.

The robot offers video as soon as a session starts, and this answers it. Pictures come back as RGB
arrays, turned upright by the mount `media.video` reports. Needs the `video` extra.
"""

from __future__ import annotations

import asyncio
import logging
import threading
from typing import Any, Callable

logger = logging.getLogger(__name__)

# What `webrtcsink` on the robot asks, so both ends use one server.
STUN = "stun:stun.l.google.com:19302"


class Camera:
    """Answers the robot's offer and keeps the newest picture.

    aiortc runs on an event loop of its own thread. The lane's reader hands it the robot's messages
    in the order they came, and callers read `latest` or wait on `picture` from any thread.
    """

    def __init__(self, send: Callable[[dict[str, Any]], None]):
        self._send = send
        self.rotate = 0
        self.latest: Any = None
        self.frames = 0
        self.state = "waiting for the robot's offer"
        self._arrived = threading.Condition()
        self._pc: Any = None
        self._loop = asyncio.new_event_loop()
        self._inbox: asyncio.Queue[dict[str, Any]] = asyncio.Queue()
        self._thread = threading.Thread(target=self._run, name="duck-video", daemon=True)
        self._thread.start()

    def on_media(self, message: dict[str, Any]) -> None:
        self._loop.call_soon_threadsafe(self._inbox.put_nowait, message)

    def picture(self, timeout: float | None = None) -> Any:
        """The newest picture, waiting up to `timeout` seconds for the first one."""
        with self._arrived:
            self._arrived.wait_for(lambda: self.latest is not None, timeout=timeout)
            return self.latest

    def close(self) -> None:
        if self._loop.is_closed():
            return
        future = asyncio.run_coroutine_threadsafe(self._close(), self._loop)
        try:
            future.result(timeout=5)
        except Exception as e:  # noqa: BLE001 - closing is best effort
            logger.debug("the peer connection did not close cleanly: %r", e)
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=5)

    def _run(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._serving = self._loop.create_task(self._serve())
        self._loop.run_forever()
        self._loop.close()

    async def _serve(self) -> None:
        # One at a time, so an ICE candidate is never added before the offer it belongs to.
        while True:
            message = await self._inbox.get()
            try:
                if message.get("sdp"):
                    await self._answer(message["sdp"])
                elif message.get("ice"):
                    await self._candidate(message["ice"])
            except Exception as e:  # noqa: BLE001 - one bad message is not a dead camera
                logger.warning("the robot's media message could not be used: %r", e)

    async def _answer(self, offer: dict[str, Any]) -> None:
        from aiortc import RTCConfiguration, RTCIceServer, RTCPeerConnection, RTCSessionDescription

        if self._pc is not None:
            await self._pc.close()
        pc = RTCPeerConnection(RTCConfiguration(iceServers=[RTCIceServer(urls=STUN)]))
        self._pc = pc

        @pc.on("track")
        def _on_track(track: Any) -> None:
            if track.kind == "video":
                asyncio.ensure_future(self._read(track))

        @pc.on("connectionstatechange")
        def _on_state() -> None:
            self.state = pc.connectionState
            logger.info("video: %s", self.state)

        await pc.setRemoteDescription(RTCSessionDescription(sdp=offer["sdp"], type=offer["type"]))
        await pc.setLocalDescription(await pc.createAnswer())
        # aiortc gathers before it answers, so its candidates ride in the answer itself.
        answer = {"type": "answer", "sdp": pc.localDescription.sdp}
        await asyncio.to_thread(self._send, {"sdp": answer})

    async def _candidate(self, ice: dict[str, Any]) -> None:
        from aiortc.sdp import candidate_from_sdp

        line = ice.get("candidate") or ""
        if self._pc is None or not line:
            return
        if line.startswith("candidate:"):
            line = line[len("candidate:") :]
        candidate = candidate_from_sdp(line)
        candidate.sdpMLineIndex = ice.get("sdpMLineIndex")
        await self._pc.addIceCandidate(candidate)

    async def _read(self, track: Any) -> None:
        import numpy as np

        while True:
            try:
                frame = await track.recv()
            except Exception:  # noqa: BLE001 - the track ends with the session
                return
            picture = frame.to_ndarray(format="rgb24")
            if self.rotate % 360:
                picture = np.rot90(picture, k=-(self.rotate // 90) % 4)
            with self._arrived:
                self.latest = picture
                self.frames += 1
                self._arrived.notify_all()

    async def _close(self) -> None:
        self._serving.cancel()
        if self._pc is not None:
            await self._pc.close()
            self._pc = None
        await asyncio.gather(self._serving, return_exceptions=True)
