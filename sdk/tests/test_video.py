"""`Camera` against an aiortc peer standing in for the robot. Skipped without the `video` extra."""

from __future__ import annotations

import asyncio
import threading
import time

import pytest

pytest.importorskip("aiortc")
np = pytest.importorskip("numpy")

from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack  # noqa: E402
from av import VideoFrame  # noqa: E402

from duck.video import Camera  # noqa: E402


class LeftEdgeWhite(VideoStreamTrack):
    """A landscape picture whose left column is white, so a turn shows where it went."""

    async def recv(self) -> VideoFrame:
        pts, time_base = await self.next_timestamp()
        picture = np.zeros((48, 64, 3), dtype=np.uint8)
        picture[:, :8] = 255
        frame = VideoFrame.from_ndarray(picture, format="rgb24")
        frame.pts, frame.time_base = pts, time_base
        return frame


def test_the_offer_is_answered_and_pictures_come_back_upright():
    loop = asyncio.new_event_loop()
    threading.Thread(target=loop.run_forever, daemon=True).start()

    def on_robot(coroutine):
        return asyncio.run_coroutine_threadsafe(coroutine, loop).result(timeout=15)

    robot = on_robot(_peer())
    answers: list = []
    camera = Camera(send=answers.append)
    camera.rotate = 90

    async def offer() -> dict:
        robot.addTrack(LeftEdgeWhite())
        await robot.setLocalDescription(await robot.createOffer())
        return {"type": "offer", "sdp": robot.localDescription.sdp}

    camera.on_media({"type": "peer", "sdp": on_robot(offer())})
    deadline = time.monotonic() + 15
    while not answers and time.monotonic() < deadline:
        time.sleep(0.05)
    assert answers, "the offer was never answered"
    assert answers[0]["sdp"]["type"] == "answer"
    on_robot(robot.setRemoteDescription(RTCSessionDescription(**answers[0]["sdp"])))

    picture = camera.picture(timeout=15)
    try:
        assert picture is not None, "no picture arrived"
        assert picture.shape[:2] == (64, 48), "a quarter turn makes it portrait"
        assert picture[0].mean() > 200, "turned clockwise, the white left edge is the top row"
        assert picture[-1].mean() < 60
    finally:
        camera.close()
        on_robot(robot.close())
        loop.call_soon_threadsafe(loop.stop)


async def _peer() -> RTCPeerConnection:
    return RTCPeerConnection()
