"""`Camera` against an aiortc peer standing in for the robot. Skipped without the `video` extra."""

from __future__ import annotations

import asyncio
import logging
import threading
import time

import pytest

pytest.importorskip("aiortc")
np = pytest.importorskip("numpy")

from aiortc import (  # noqa: E402
    RTCConfiguration,
    RTCPeerConnection,
    RTCSessionDescription,
    VideoStreamTrack,
)
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


@pytest.fixture
def robot():
    loop = asyncio.new_event_loop()
    threading.Thread(target=loop.run_forever, daemon=True).start()

    def run(coroutine):
        return asyncio.run_coroutine_threadsafe(coroutine, loop).result(timeout=15)

    async def peer() -> RTCPeerConnection:
        return RTCPeerConnection(RTCConfiguration(iceServers=[]))

    pc = run(peer())
    yield pc, run
    run(pc.close())
    loop.call_soon_threadsafe(loop.stop)


def test_the_offer_is_answered_and_pictures_come_back_upright(robot, caplog):
    pc, run = robot
    answers: list = []
    camera = Camera(send=answers.append, stun=None)
    camera.rotate = 90
    try:

        async def offer() -> dict:
            pc.addTrack(LeftEdgeWhite())
            await pc.setLocalDescription(await pc.createOffer())
            return {"type": "offer", "sdp": pc.localDescription.sdp}

        sdp = run(offer())
        camera.on_media({"type": "peer", "sdp": sdp})
        # The robot trickles its candidates too, by line index and nothing else.
        line = next(x for x in sdp["sdp"].splitlines() if x.startswith("a=candidate:"))
        with caplog.at_level(logging.WARNING, logger="duck.video"):
            camera.on_media({"type": "peer", "ice": {"candidate": line[2:], "sdpMLineIndex": 0}})

            deadline = time.monotonic() + 10
            while not answers and time.monotonic() < deadline:
                time.sleep(0.02)
            assert answers, "the offer was never answered"
            assert answers[0]["sdp"]["type"] == "answer"
            run(pc.setRemoteDescription(RTCSessionDescription(**answers[0]["sdp"])))

            picture = camera.picture(timeout=10)
        assert not caplog.records, [r.getMessage() for r in caplog.records]
        assert picture is not None, "no picture arrived"
        assert picture.shape[:2] == (64, 48), "a quarter turn makes it portrait"
        assert picture[0].mean() > 200, "turned clockwise, the white left edge is the top row"
        assert picture[-1].mean() < 60
        assert picture.flags["C_CONTIGUOUS"], "torch.from_numpy takes it as it is"
    finally:
        camera.close()


def test_closing_wakes_a_caller_waiting_for_a_picture():
    camera = Camera(send=lambda _: None, stun=None)
    got: list = []
    waiter = threading.Thread(target=lambda: got.append(camera.picture(timeout=10)), daemon=True)
    waiter.start()
    time.sleep(0.1)
    camera.close()
    waiter.join(timeout=2)
    assert not waiter.is_alive(), "picture() waited out its timeout after close()"
    assert got == [None]
