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

from microduck.video import Camera, VideoError  # noqa: E402


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
        with caplog.at_level(logging.WARNING, logger="microduck.video"):
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


def waiting(camera: Camera, timeout: float = 10) -> tuple[threading.Thread, list]:
    got: list = []

    def wait() -> None:
        try:
            got.append(camera.picture(timeout=timeout))
        except VideoError as e:
            got.append(e)

    waiter = threading.Thread(target=wait, daemon=True)
    waiter.start()
    time.sleep(0.1)
    return waiter, got


def test_closing_wakes_a_caller_waiting_for_a_picture():
    camera = Camera(send=lambda _: None, stun=None)
    waiter, got = waiting(camera)
    camera.close()
    waiter.join(timeout=2)
    assert not waiter.is_alive(), "picture() waited out its timeout after close()"
    assert [str(e) for e in got] == ["the duck was closed"]


def test_no_picture_yet_is_told_apart_from_a_video_that_stopped():
    camera = Camera(send=lambda _: None, stun=None)
    try:
        with pytest.raises(VideoError, match="no picture arrived in 0.2 s"):
            camera.picture(timeout=0.2)

        with camera._arrived:
            camera.latest = np.zeros((2, 2, 3), dtype=np.uint8)
            camera.frames = 40
        assert camera.picture(timeout=0.2) is not None

        # What a failed connection or an ended track does.
        camera.state = "failed"
        camera._drop(camera._pc)
        started = time.monotonic()
        with pytest.raises(VideoError, match=r"the video stopped after 40 pictures \(failed\)"):
            camera.picture(timeout=10)
        assert time.monotonic() - started < 1, "said at once, not after the timeout"
    finally:
        camera.close()


def test_a_replaced_connection_ending_late_is_not_the_video_stopping():
    camera = Camera(send=lambda _: None, stun=None)
    try:
        with camera._arrived:
            camera.latest = np.zeros((2, 2, 3), dtype=np.uint8)
            camera.frames = 3
        camera._drop(object())
        assert camera.picture(timeout=0.2) is not None
    finally:
        camera.close()


def test_a_video_that_ends_before_its_first_picture_wakes_the_caller_waiting_for_it():
    camera = Camera(send=lambda _: None, stun=None)
    try:
        waiter, got = waiting(camera)
        camera.state = "failed"
        camera._drop(camera._pc)
        waiter.join(timeout=2)
        assert not waiter.is_alive(), "picture() waited out its timeout after the video ended"
        assert [str(e) for e in got] == ["the video ended before any picture arrived (failed)"]
    finally:
        camera.close()
