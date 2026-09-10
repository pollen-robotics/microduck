"""The endpoint a duck dials, and the frames it leaves behind.

**The robot connects to us, which is the whole reason this exists.** Pulling the camera through
the rendezvous means WebRTC, and WebRTC between a robot behind a home router and a container in a
data centre needs a relay candidate — `turn.fastrtc.org` has no A record and its zone no NS
records (`remote-access-design.md` §6), so it negotiates and carries nothing. An outbound
WebSocket has no such problem: the robot already holds one to a Space every second it is
reachable. So the direction is inverted, NAT stops being a participant, and this is the socket at
the far end.

`mediad`'s `stream.rs` is the other half. It sends one text frame describing what is coming, then
one binary message per JPEG, upright, at the rate it was asked for.

## Whose camera this is, which is not optional

This endpoint is public — a Space's URL is a Space's URL — so anything that can reach the internet
can open it. Two things would go wrong without a check: anybody could push frames into somebody
else's demo, and worse, a visitor could be shown a stranger's camera. So the robot presents its
own account token on the handshake and this resolves it through `whoami-v2`, exactly as the
rendezvous does, and frames are filed under the username it answers with. A visitor sees the
robots on their own account and nothing else.

The token never leaves this process and is never stored: it is resolved once per connection and
what is kept is the username.
"""

from __future__ import annotations

import json
import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Any

import numpy as np
import requests
from fastapi import WebSocket, WebSocketDisconnect

logger = logging.getLogger(__name__)

WHOAMI = "https://huggingface.co/api/whoami-v2"

# A frame older than this is not what the robot is looking at any more. Shown as stale rather than
# hidden, because "the stream stopped" and "there was never a stream" are different problems.
STALE_AFTER = 5.0


@dataclass
class Stream:
    """The newest picture from one robot, and how the stream has been going.

    **A decoded picture rather than the bytes, because H.264 is stateful.** A JPEG can be kept
    and decoded whenever somebody looks; an H.264 access unit means nothing without every unit
    before it, so units are decoded in arrival order as they land and what is kept is the result.
    JPEG goes the same way rather than keeping two rules — at a few frames a second the decode is
    free either way.
    """

    username: str
    robot: str
    hello: dict[str, Any] = field(default_factory=dict)
    picture: np.ndarray | None = None
    frames: int = 0
    first_at: float = 0.0
    last_at: float = 0.0
    bytes_in: int = 0
    #: What turns a unit into pictures, chosen from the hello. Stateful for H.264, which is why it
    #: belongs to the stream rather than being made per call.
    decoder: Any = None
    #: Units that arrived and produced no picture yet — every H.264 stream starts with some,
    #: because the decoder has nothing to predict from until a keyframe and its parameter sets
    #: have both arrived.
    undecoded: int = 0
    last_error: str | None = None

    @property
    def fps(self) -> float:
        span = self.last_at - self.first_at
        return (self.frames - 1) / span if self.frames > 1 and span > 0 else 0.0

    @property
    def stale(self) -> bool:
        return time.monotonic() - self.last_at > STALE_AFTER


class Frames:
    """Every live stream, by account.

    A dict rather than one slot, because a Space serves whoever opens it and two people's robots
    must not land in the same place. Keyed by username and then by robot name: one account can have
    more than one duck, and a demo that silently showed the last one to connect would be a demo
    that lies about which robot it is looking at.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._streams: dict[tuple[str, str], Stream] = {}

    def opened(self, username: str, robot: str, hello: dict[str, Any]) -> Stream:
        with self._lock:
            stream = Stream(username=username, robot=robot, hello=hello)
            stream.decoder = decoder_for(hello)
            stream.first_at = stream.last_at = time.monotonic()
            self._streams[(username, robot)] = stream
            return stream

    def arrived(self, stream: Stream, unit: bytes) -> None:
        """Decode one unit and keep what came out, if anything.

        No lock: one writer per stream, and a reader takes the ndarray by reference. A reader
        either sees the previous picture or this one, never half of either — which is why the
        assignment is the last thing that happens.
        """
        stream.bytes_in += len(unit)
        stream.last_at = time.monotonic()
        try:
            pictures = stream.decoder(unit)
        except Exception as e:  # noqa: BLE001 - a bad unit is not a bad stream
            stream.last_error = f"{type(e).__name__}: {e}"
            logger.warning("a unit would not decode: %s", stream.last_error)
            return
        if not pictures:
            # Ordinary at the start of an H.264 stream, and worth counting rather than hiding: a
            # stream that only ever does this is a stream whose keyframe never arrived.
            stream.undecoded += 1
            return
        for picture in pictures:
            stream.frames += 1
            stream.picture = picture

    def closed(self, username: str, robot: str) -> None:
        with self._lock:
            self._streams.pop((username, robot), None)

    def of(self, username: str) -> list[Stream]:
        with self._lock:
            return [s for (user, _), s in self._streams.items() if user == username]

    def newest(self, username: str) -> Stream | None:
        streams = sorted(self.of(username), key=lambda s: s.last_at, reverse=True)
        return streams[0] if streams else None


FRAMES = Frames()


def whoami(token: str) -> str | None:
    """The account a token belongs to, or `None`.

    The same one call the rendezvous makes (`validate_hf_token`), and deliberately no more: no
    scope check and no token-type check, because a robot's device-flow token and a visitor's OAuth
    token are both legitimate here and neither carries anything this needs to inspect.
    """
    try:
        answer = requests.get(
            WHOAMI, headers={"Authorization": f"Bearer {token}"}, timeout=10
        )
    except requests.RequestException as e:
        logger.warning("whoami-v2 unreachable: %s", e)
        return None
    if answer.status_code != 200:
        logger.info("whoami-v2 refused a token: HTTP %s", answer.status_code)
        return None
    try:
        return answer.json().get("name")
    except ValueError:
        return None


def decoder_for(hello: dict[str, Any]) -> Any:
    """A callable turning one unit into zero or more RGB pictures.

    Chosen from `frames.encoding` in the hello rather than sniffed from the bytes, which is why
    `mediad` sends that field: a JPEG's magic and an Annex-B start code are both recognisable, but
    a receiver that guessed would be a receiver that guesses wrong on the day a third encoding
    appears.
    """
    encoding = ((hello.get("frames") or {}).get("encoding") or "h264").lower()
    if encoding in ("jpeg", "jpg", "mjpeg"):
        return jpeg_decoder()
    if encoding in ("h264", "avc"):
        return h264_decoder()
    raise ValueError(f"a robot offered frames as {encoding!r}, which this Space cannot decode")


def jpeg_decoder() -> Any:
    """One JPEG in, one picture out. Stateless, which is the point of it.

    OpenCV rather than PIL because the filters are OpenCV anyway. `imdecode` gives BGR, flipped
    once here so everything downstream — the filters, Gradio — deals in RGB like the rest of this
    project.
    """
    import cv2

    def decode(unit: bytes) -> list[np.ndarray]:
        picture = cv2.imdecode(np.frombuffer(unit, dtype=np.uint8), cv2.IMREAD_COLOR)
        return [] if picture is None else [picture[:, :, ::-1]]

    return decode


def h264_decoder() -> Any:
    """Annex-B access units in, pictures out — with the first few producing nothing.

    **Stateful, and the state is the whole difference from JPEG.** A predicted frame means nothing
    without the frames it refers to, so this holds one decoder for the life of a connection and
    feeds it in arrival order. It produces nothing until a keyframe and its parameter sets have
    both arrived, which `mediad` makes certain of by asking its encoder for a keyframe when the
    valve opens and by carrying SPS/PPS in front of every one (`h264parse config-interval=-1`).

    `parse` before `decode` even though `mediad` sends whole access units: the parser is what
    strips start codes and splits a unit that arrived with its parameter sets glued in front,
    which is exactly what a keyframe looks like here.
    """
    import av

    codec = av.CodecContext.create("h264", "r")

    def decode(unit: bytes) -> list[np.ndarray]:
        pictures: list[np.ndarray] = []
        for packet in codec.parse(unit):
            for frame in codec.decode(packet):
                pictures.append(frame.to_ndarray(format="rgb24"))
        return pictures

    return decode


def mount(app: Any) -> None:
    """Add `GET /frames` (WebSocket) to a FastAPI app.

    A route on a FastAPI app that *owns* the server, with Gradio mounted into it — not a route
    added to Gradio's own app. Mounting Gradio inside FastAPI is the documented direction; the
    reverse has known WebSocket breakage, which would be an unusually annoying thing to discover
    from inside a Space.

    **`WebSocket` is imported at module level and must stay there.** This file has
    `from __future__ import annotations`, so every annotation is a string that FastAPI resolves
    with `get_type_hints` against the *module's* globals. Imported inside this function instead,
    the name is a local: FastAPI cannot resolve `socket: WebSocket`, stops recognising the
    parameter as the connection, fails to build the route's dependencies, and closes every
    incoming socket before the handler runs — which the robot is told as an HTTP 403 and reads as
    a wrong url. It took a header probe to find, because nothing logs it at either end.
    """
    @app.websocket("/frames")
    async def frames(socket: WebSocket) -> None:  # pyright: ignore[reportUnusedFunction]
        # **Resolved before the upgrade is accepted.** A socket accepted and then closed looks to
        # the robot exactly like a receiver that hung up, and it would redial forever.
        header = socket.headers.get("authorization") or ""
        token = header[len("Bearer ") :].strip() if header.lower().startswith("bearer ") else ""
        if not token:
            # **Said out loud**, because a refusal is invisible from the robot's side: closing
            # before `accept` is reported to it as an HTTP 403, which reads exactly like a wrong
            # url. This line is the difference between "the Space refused my token" and an
            # afternoon of guessing.
            logger.info(
                "a frame stream was refused: no bearer token (authorization header %s)",
                "absent" if not header else f"present, {len(header)} chars, not Bearer",
            )
            await socket.close(code=1008, reason="no bearer token")
            return
        username = whoami(token)
        if not username:
            logger.info("a frame stream was refused: whoami-v2 did not know that token")
            await socket.close(code=1008, reason="Hugging Face did not know that token")
            return

        await socket.accept()
        logger.info("a robot on %s's account opened a frame stream", username)

        stream: Stream | None = None
        robot = "a duck"
        try:
            while True:
                message = await socket.receive()
                if "text" in message and message["text"] is not None:
                    # The hello. One per connection, and a redial sends a new one — a Space
                    # restarts on every push, so the robot cannot assume we remember anything.
                    try:
                        hello = json.loads(message["text"])
                    except ValueError:
                        continue
                    robot = (hello.get("robot") or {}).get("name") or robot
                    stream = FRAMES.opened(username, robot, hello)
                    logger.info("hello from %s: %s", robot, message["text"][:200])
                elif "bytes" in message and message["bytes"] is not None:
                    if stream is None:
                        # Frames before a hello: acceptable rather than fatal, since what is
                        # missing is a label rather than a picture.
                        stream = FRAMES.opened(username, robot, {})
                    FRAMES.arrived(stream, message["bytes"])
                elif message.get("type") == "websocket.disconnect":
                    break
        except WebSocketDisconnect:
            pass
        except Exception as e:  # noqa: BLE001 - one bad socket is not a bad Space
            logger.warning("a frame stream failed: %r", e)
        finally:
            FRAMES.closed(username, robot)
            logger.info("the frame stream from %s ended", robot)
