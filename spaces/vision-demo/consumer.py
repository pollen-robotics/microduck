"""A consumer of one duck, over the rendezvous service.

The other end of `mediad::relay`. The robot registers with `reachy_mini_central` as a producer;
this asks that service for a session, answers the robot's offer, and decodes the video into numpy
arrays. `docs/design/remote-access-design.md` §3 is the protocol and §3.2's table is the wire.

# Why this exists rather than `reachy_mini[central-consumer]`

Their consumer works — it was the first non-browser client to get frames out of a duck, which is
how we know the transport is sound. As a *dependency* it brought three problems that are all the
same problem: it is the robot's own package rather than a client library.

- `reachy_mini/__init__.py` imports `ReachyMini` and `ReachyMiniApp`, so reaching the consumer
  drags in the daemon's world: **PyGObject**, which is a source build wanting a C compiler and
  cairo headers for GStreamer bindings a consumer never touches.
- It pins `starlette<1.0.0`, which cannot coexist with any current Gradio.
- It looks for the mini's data channel label and ignores ours, so no JSON-RPC reaches the process
  — which means the camera geometry cannot be *asked* for and has to be re-derived.

We wrote the robot half of this protocol. The client half is smaller than the workarounds, and it
gets the third point for free: this one speaks `control`, so the intrinsics come from the robot
rather than from arithmetic repeated here.

# What it does not do

One robot, one session, video and the control channel. No audio, no producer re-selection, no
reconnect: a demo that drops its session should say so and let somebody press connect, and
`mediad::relay`'s reconnect logic exists on the robot's side where it matters.
"""

from __future__ import annotations

import asyncio
import json
import logging
import threading
from typing import Any, Callable

import httpx
import numpy as np
from aiortc import (
    RTCConfiguration,
    RTCIceServer,
    RTCPeerConnection,
    RTCSessionDescription,
)
from aiortc.sdp import candidate_from_sdp

logger = logging.getLogger(__name__)

DEFAULT_RENDEZVOUS = "https://pollen-robotics-reachy-mini-central.hf.space"

# The same STUN server `webrtcsink` defaults to on the robot, so both ends ask one service rather
# than two. A relay candidate would come from the *robot* — only one side needs one, and
# `remote-access-design.md` §6 has why that side is not this one.
STUN = "stun:stun.l.google.com:19302"

# `meta.kind`, so a consumer written for a duck cannot pick up a mini. Their clients match on
# `meta.name` and skip this, which is §5.1's open conversation.
KIND = "microduck"

# The robot creates this channel as each consumer arrives, and speaks JSON-RPC over it.
CONTROL_CHANNEL = "control"

# How long the whole handshake gets: SSE open, welcome, list, startSession, offer, answer.
HANDSHAKE_TIMEOUT = 30.0


def _patch_dtls_ciphers() -> None:
    """Add the one cipher GStreamer's `webrtcsink` and aiortc do not otherwise share.

    Without this DTLS never completes and the peer connection sits in `connecting` forever, with
    both sides behaving correctly and neither logging anything wrong. Diagnosed and fixed by
    `reachy_mini`'s `central_consumer`, which is the only reason this is fifteen lines here rather
    than a week; upstream is aiortc PR #1392, and this goes when a release negotiates it.
    """
    from aiortc.rtcdtlstransport import RTCCertificate

    if getattr(RTCCertificate, "_duck_cipher_patched", False):
        return
    original = RTCCertificate._create_ssl_context
    ciphers = (
        b"ECDHE-ECDSA-AES128-GCM-SHA256:ECDHE-ECDSA-CHACHA20-POLY1305:"
        b"ECDHE-ECDSA-AES128-SHA:ECDHE-ECDSA-AES256-SHA:"
        b"ECDHE-RSA-AES128-GCM-SHA256"
    )

    def patched(self: Any, srtp_profiles: Any) -> Any:
        context = original(self, srtp_profiles)
        try:
            context.set_cipher_list(ciphers)
        except Exception as e:  # noqa: BLE001 - a refused list is worth a line, not a crash
            logger.warning("could not extend the DTLS cipher list: %r", e)
        return context

    RTCCertificate._create_ssl_context = patched  # type: ignore[method-assign]
    RTCCertificate._duck_cipher_patched = True  # type: ignore[attr-defined]
    logger.info("DTLS cipher list extended for GStreamer's webrtcsink")


_patch_dtls_ciphers()


def sse_events(chunk: str, buffer: str) -> tuple[list[str], str]:
    """Split an SSE byte-stream fragment into complete `data:` payloads, and what is left over.

    Pure, and separate from the reading, so the framing is testable without a server. Two things
    it has to get right, both of which cost the console page an hour: **CRLF**, because the
    service's line endings are not ours to assume, and **a payload split across reads**, which is
    the normal case for an SDP offer larger than a TCP segment.
    """
    buffer = (buffer + chunk).replace("\r\n", "\n")
    payloads: list[str] = []
    while "\n\n" in buffer:
        frame, buffer = buffer.split("\n\n", 1)
        data = "".join(
            line[5:].strip() for line in frame.split("\n") if line.startswith("data:")
        )
        if data:
            payloads.append(data)
    return payloads, buffer


def pick_producer(producers: list[dict[str, Any]], name: str | None) -> dict[str, Any] | None:
    """Choose which robot to drive: a duck, by name when one is asked for.

    Filtering on `meta.kind` first is what keeps this from picking up a mini, which would answer
    every method name with `METHOD_NOT_FOUND` and look like a broken robot. If a name is given and
    matches nothing, that is **not** a reason to take whatever is there instead: a consumer aimed
    at one robot silently driving another is worse than one that reports it found nothing.
    """
    ducks = [p for p in producers if (p.get("meta") or {}).get("kind") == KIND]
    if name:
        wanted = name.strip().lower()
        return next(
            (p for p in ducks if ((p.get("meta") or {}).get("name") or "").lower() == wanted),
            None,
        )
    return ducks[0] if ducks else None


class DuckConsumer:
    """One session with one duck. Not thread-safe; drive it from its own event loop."""

    def __init__(
        self,
        token: str,
        robot_name: str | None = None,
        label: str = "duck-consumer",
        rendezvous: str = DEFAULT_RENDEZVOUS,
    ) -> None:
        self._token = token
        self._robot_name = robot_name
        self._label = label
        self._base = rendezvous.rstrip("/")

        self._client: httpx.AsyncClient | None = None
        self._pc: RTCPeerConnection | None = None
        self._task: asyncio.Task[None] | None = None
        self._control: Any = None

        self._peer_id: str | None = None
        self._robot_peer_id: str | None = None
        self._session_id: str | None = None
        self._frames = 0
        self._latest: tuple[int, np.ndarray] | None = None
        self._video: dict[str, Any] | None = None
        self._error: str | None = None
        # The frame is read by whatever renders and written by the track reader.
        self._lock = threading.Lock()

    # ── what a caller sees ──────────────────────────────────────────────────

    def latest_frame(self) -> tuple[int, np.ndarray] | None:
        with self._lock:
            return self._latest

    def video_info(self) -> dict[str, Any] | None:
        """What `media.video` answered: size, mount rotation, and the camera's intrinsics.

        `None` until the control channel has opened and answered — which is a second or two after
        the first frame, so a caller should treat its absence as "not yet" rather than "never".
        """
        return self._video

    def status(self) -> dict[str, Any]:
        return {
            "session_id": self._session_id,
            "robot_peer_id": self._robot_peer_id,
            "pc_state": self._pc.connectionState if self._pc is not None else None,
            "ice_state": self._pc.iceConnectionState if self._pc is not None else None,
            "frames": self._frames,
            "control": self._control is not None,
            "error": self._error,
        }

    async def start(self) -> None:
        if self._task is not None and not self._task.done():
            return
        self._error = None
        self._client = httpx.AsyncClient(
            timeout=httpx.Timeout(10.0, read=None),
            headers={"authorization": f"Bearer {self._token}"},
        )
        self._task = asyncio.create_task(self._run(), name="duck-consumer")

    async def stop(self) -> None:
        """End the session, in the order the service requires.

        **The farewell goes first, and that ordering is the whole of this method.** `POST /send`
        is refused with a 400 — "Connect to /events first" — once the event stream has gone, so a
        teardown that cancels the reader before saying goodbye cannot say goodbye at all. The
        robot then waits out a timeout instead of freeing its side immediately, and the shutdown
        raises on the way out. `reachy_mini`'s consumer has this bug; noticing it there did not
        stop me writing it here, which is the argument for the ordering being written down rather
        than remembered.

        And nothing here raises. A teardown that throws leaves a UI holding a consumer it thinks
        is still alive, which is worse than a session the service reaps on its own.
        """
        if self._session_id and self._client is not None:
            try:
                await self._send({"type": "endSession", "sessionId": self._session_id})
            except Exception as e:  # noqa: BLE001 - a farewell is a courtesy, not a requirement
                logger.debug("could not tell the service the session ended: %r", e)

        task, self._task = self._task, None
        if task is not None:
            task.cancel()
            try:
                await task
            except BaseException:  # noqa: BLE001 - includes the CancelledError we just caused
                pass
        if self._pc is not None:
            await self._pc.close()
        if self._client is not None:
            await self._client.aclose()
        self._pc, self._client, self._control = None, None, None
        self._session_id = None

    def call(self, method: str, params: dict[str, Any] | None = None, request_id: int = 1) -> bool:
        """Send one JSON-RPC request over the robot's control channel."""
        if self._control is None:
            return False
        self._control.send(
            json.dumps(
                {"jsonrpc": "2.0", "id": request_id, "method": method, "params": params or {}}
            )
        )
        return True

    # ── the session ─────────────────────────────────────────────────────────

    async def _send(self, message: dict[str, Any]) -> dict[str, Any] | None:
        """`POST /send`, returning the body when there is one.

        **The reply can be a message**, which is the row of §3.2's table that cost the console an
        afternoon: `list` and `startSession` are answered in the HTTP response, everything else
        arrives on the event stream.
        """
        assert self._client is not None
        response = await self._client.post(f"{self._base}/send", json=message)
        if response.status_code >= 400:
            raise RuntimeError(f"POST /send {message['type']}: HTTP {response.status_code}")
        try:
            body = response.json()
        except ValueError:
            return None
        return body if isinstance(body, dict) and body.get("type") else None

    async def _run(self) -> None:
        assert self._client is not None
        try:
            async with self._client.stream("GET", f"{self._base}/events") as stream:
                if stream.status_code == 401:
                    raise RuntimeError("Hugging Face refused this token")
                stream.raise_for_status()

                buffer = ""
                async for chunk in stream.aiter_text():
                    payloads, buffer = sse_events(chunk, buffer)
                    for payload in payloads:
                        await self._on_message(json.loads(payload))
        except asyncio.CancelledError:
            raise
        except Exception as e:  # noqa: BLE001 - surfaced through `status`, not raised at a UI
            self._error = f"{type(e).__name__}: {e}"
            logger.warning("the consumer's session ended: %s", self._error)

    async def _on_message(self, message: dict[str, Any]) -> None:
        kind = message.get("type")

        if kind == "welcome":
            self._peer_id = message.get("peerId")
            # The peer exists only once the stream does — `POST /send` before `GET /events` is a
            # 400 — so everything else starts here rather than in `start`.
            listing = await self._send({"type": "list"})
            await self._on_message(listing or {"type": "list", "producers": []})

        elif kind == "list":
            if self._session_id:
                return
            producer = pick_producer(message.get("producers") or [], self._robot_name)
            if producer is None:
                self._error = (
                    f"no duck named {self._robot_name!r} is registered"
                    if self._robot_name
                    else "no duck is registered with this account"
                )
                return
            self._robot_peer_id = producer.get("id")
            meta = producer.get("meta") or {}
            logger.info("found %s (%s)", meta.get("name"), self._robot_peer_id)
            started = await self._send(
                {"type": "startSession", "peerId": self._robot_peer_id}
            )
            if started and started.get("type") == "sessionStarted":
                self._session_id = started.get("sessionId")
                self._open_peer_connection()

        elif kind == "peer":
            await self._on_peer(message)

        elif kind == "endSession":
            self._error = message.get("reason") or "the robot ended the session"
            self._session_id = None

        elif kind == "sessionRejected":
            self._error = (
                f"the robot refused a session ({message.get('reason')})"
                f"{' — ' + message['activeApp'] if message.get('activeApp') else ''}"
            )

    def _open_peer_connection(self) -> None:
        """Build the peer connection before the robot's offer arrives."""
        self._pc = RTCPeerConnection(
            RTCConfiguration(iceServers=[RTCIceServer(urls=STUN)])
        )

        @self._pc.on("track")
        def _on_track(track: Any) -> None:
            if track.kind != "video":
                return
            asyncio.create_task(self._read_track(track))

        @self._pc.on("datachannel")
        def _on_datachannel(channel: Any) -> None:
            if channel.label != CONTROL_CHANNEL:
                logger.info("ignoring a data channel labelled %r", channel.label)
                return
            self._control = channel

            @channel.on("message")
            def _on_line(line: str) -> None:
                self._on_control_line(line)

            # The one question worth asking immediately: what the picture is, geometrically.
            # A client that derives this instead is guessing at a sensor mode it cannot see.
            self.call("media.video", request_id=1)

    def _on_control_line(self, line: str) -> None:
        try:
            message = json.loads(line)
        except ValueError:
            return
        # `media.video` arrives twice over: pushed as a notification when the channel opens, and
        # as the reply to the call above. Either is welcome; whichever lands first wins.
        if message.get("method") == "media.video" and isinstance(message.get("params"), dict):
            self._video = message["params"]
        elif message.get("id") == 1 and isinstance(message.get("result"), dict):
            self._video = message["result"]

    async def _read_track(self, track: Any) -> None:
        while True:
            try:
                frame = await track.recv()
            except Exception:  # noqa: BLE001 - the track ending is how a session ends
                return
            image = frame.to_ndarray(format="rgb24")
            with self._lock:
                self._frames += 1
                self._latest = (self._frames, image)

    async def _on_peer(self, message: dict[str, Any]) -> None:
        if self._pc is None:
            return
        if sdp := message.get("sdp"):
            await self._pc.setRemoteDescription(
                RTCSessionDescription(sdp=sdp["sdp"], type=sdp["type"])
            )
            # The robot offers, because `webrtcsink` knows what it is sending. Answering waits for
            # ICE gathering to finish, so the answer carries this end's candidates and nothing has
            # to be trickled back.
            answer = await self._pc.createAnswer()
            await self._pc.setLocalDescription(answer)
            await self._send(
                {
                    "type": "peer",
                    "sessionId": self._session_id,
                    "sdp": {"type": "answer", "sdp": self._pc.localDescription.sdp},
                }
            )
        elif ice := message.get("ice"):
            if not ice.get("candidate"):
                return  # end-of-candidates, which aiortc infers for itself
            candidate = candidate_from_sdp(ice["candidate"].split(":", 1)[-1])
            candidate.sdpMLineIndex = ice.get("sdpMLineIndex", 0)
            await self._pc.addIceCandidate(candidate)


def frames_from(
    consumer: DuckConsumer,
) -> Callable[[], tuple[int, np.ndarray] | None]:
    """A getter, for a UI that should not hold a consumer."""
    return consumer.latest_frame
