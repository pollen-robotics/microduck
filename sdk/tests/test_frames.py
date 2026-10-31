"""`receive` against what `mediad::stream` sends. Skipped without the `frames` extra."""

from __future__ import annotations

import json
import socket
import threading
import time
from fractions import Fraction

import pytest

av = pytest.importorskip("av")
np = pytest.importorskip("numpy")
pytest.importorskip("websockets")

from websockets.sync.client import connect  # noqa: E402

from duck import receive  # noqa: E402
from duck.frames import decoder_for  # noqa: E402

WIDTH, HEIGHT = 64, 48


def pictures(count: int) -> list:
    return [
        av.VideoFrame.from_ndarray(
            np.full((HEIGHT, WIDTH, 3), 20 * i, dtype=np.uint8), format="rgb24"
        )
        for i in range(count)
    ]


def encoded(codec_name: str, pix_fmt: str, count: int) -> list[bytes]:
    try:
        codec = av.CodecContext.create(codec_name, "w")
    except (av.FFmpegError, ValueError):
        pytest.skip(f"this FFmpeg has no {codec_name} encoder")
    codec.width, codec.height, codec.pix_fmt = WIDTH, HEIGHT, pix_fmt
    codec.time_base = Fraction(1, 5)
    units = []
    for i, frame in enumerate(pictures(count)):
        frame.pts = i
        units += [bytes(p) for p in codec.encode(frame.reformat(format=pix_fmt))]
    units += [bytes(p) for p in codec.encode(None)]
    return units


def hello(encoding: str) -> dict:
    return {"type": "hello", "frames": {"encoding": encoding, "fps": 5}}


def test_a_jpeg_is_one_picture_as_soon_as_it_arrives():
    decode = decoder_for(hello("jpeg"))
    [unit] = encoded("mjpeg", "yuvj420p", 1)
    [picture] = decode(unit)
    assert picture.shape == (HEIGHT, WIDTH, 3)


def test_h264_is_decoded_as_a_stream_not_a_unit_at_a_time():
    decode = decoder_for(hello("h264"))
    units = encoded("libx264", "yuv420p", 5)
    out = [p for unit in units for p in decode(unit)]
    assert out and out[0].shape == (HEIGHT, WIDTH, 3)


def test_an_encoding_nobody_decodes_is_refused_by_name():
    with pytest.raises(ValueError, match="'vp9'"):
        decoder_for(hello("vp9"))


def test_receive_branches_on_the_hello_before_the_first_frame():
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]

    seen: list = []
    threading.Thread(
        target=receive, args=(port, seen.append), kwargs={"host": "127.0.0.1"}, daemon=True
    ).start()

    deadline = time.monotonic() + 5
    while True:
        try:
            robot = connect(f"ws://127.0.0.1:{port}")
            break
        except OSError:
            assert time.monotonic() < deadline, "receive never listened"
            time.sleep(0.05)

    with robot:
        robot.send(json.dumps(hello("jpeg")))
        for unit in encoded("mjpeg", "yuvj420p", 2):
            robot.send(unit)
        deadline = time.monotonic() + 5
        while len(seen) < 2 and time.monotonic() < deadline:
            time.sleep(0.05)

    assert len(seen) == 2
    assert seen[0].shape == (HEIGHT, WIDTH, 3)
