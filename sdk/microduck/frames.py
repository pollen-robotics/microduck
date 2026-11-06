"""Frames from `media.stream` turned into pictures. Needs the `frames` extra."""

from __future__ import annotations

from typing import Any, Callable


def decoder_for(hello: dict[str, Any]) -> Callable[[bytes], list[Any]]:
    """A callable turning one message into zero or more RGB arrays, chosen by `frames.encoding`.

    H.264 keeps one decoder for the connection and gives nothing until the first keyframe.
    """
    import av

    encoding = ((hello.get("frames") or {}).get("encoding") or "h264").lower()
    if encoding in ("jpeg", "jpg", "mjpeg"):
        jpeg = av.CodecContext.create("mjpeg", "r")

        def decode_jpeg(unit: bytes) -> list[Any]:
            return [f.to_ndarray(format="rgb24") for f in jpeg.decode(av.Packet(unit))]

        return decode_jpeg
    if encoding in ("h264", "avc"):
        h264 = av.CodecContext.create("h264", "r")

        def decode_h264(unit: bytes) -> list[Any]:
            pictures = []
            for packet in h264.parse(unit):
                for frame in h264.decode(packet):
                    pictures.append(frame.to_ndarray(format="rgb24"))
            return pictures

        return decode_h264
    raise ValueError(f"the robot offered frames as {encoding!r}, which this cannot decode")
