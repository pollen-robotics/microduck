"""The same session over the robot's own signalling server, for while TURN is down.

The rendezvous path is the one that reaches a duck from anywhere, and it needs a relay candidate
to do it: between a home router and a data centre, srflx-to-srflx punches often enough to be worth
trying and not often enough to rely on. `turn.fastrtc.org` has no A record and its zone has no NS
records at all (`docs/design/remote-access-design.md` §6), so today the honest answer for a
consumer that wants to *drive* a duck is to be on the duck's network — where ICE needs no relay
because both sides offer host candidates.

**This is a transport, not a second design.** `webrtcsink`'s signalling server is already running
on the robot at `ws://<robot>:8443`, and it carries the same gst envelopes the rendezvous carries
over SSE and `POST /send` — §3.2 has the two sides side by side. So everything above the session is
untouched: the same `control` channel, the same JSON-RPC, the same `Rpc` object. What changes is
one hop, and it changes in the direction of *fewer* moving parts: no account, no lease, no
rendezvous, no relay.

Which makes it the thing to reach for when a click does not work over the rendezvous and nobody
knows which layer to blame. If it works here and not there, the transport is the problem and
nothing about the policy, the manifest or the robot is.

What is deliberately absent is a reconnect loop. Their consumer has one because a robot on the far
side of the internet goes away for reasons nobody can see; a robot on this desk that stops
answering is worth a sentence on the page rather than a silent retry every five seconds.
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import logging
import re
from typing import Any

import aiohttp
import numpy as np
import numpy.typing as npt
from aiortc import RTCDataChannel, RTCPeerConnection, RTCSessionDescription
from aiortc.mediastreams import MediaStreamError
from aiortc.sdp import candidate_from_sdp

# **Their DTLS shim, called out loud.** aiortc's default cipher list shares no cipher with the
# GStreamer `webrtcsink` a duck runs, so DTLS never completes and the peer connection never
# reaches `connected` — a failure that looks exactly like a NAT problem and is not one. Their
# module applies it at import; calling it here means this file does not depend on somebody else's
# import having happened first, and it is idempotent by design.
from reachy_mini.media.central_consumer import _patch_aiortc_dtls_ciphers

from control import CONTROL_LABEL, Rpc

logger = logging.getLogger(__name__)

_patch_aiortc_dtls_ciphers()

# What `mediad --port` defaults to, which is `webrtcsink`'s own signaller's default.
DEFAULT_SIGNALLING_PORT = 8443


def _log_candidates(side: str, sdp: str) -> None:
    """How many candidates of each type one side offered.

    The single most useful line when signalling crossed and media did not: `host` only on both
    sides means nothing can work across networks, and no `relay` anywhere is §6.
    """
    kinds: dict[str, int] = {}
    for kind in re.findall(r"candidate:[^\r\n]* typ (\w+)", sdp or ""):
        kinds[kind] = kinds.get(kind, 0) + 1
    logger.info("%s ICE candidates: %s", side, kinds or "none")


class LanConsumer:
    """One session with one robot on this network.

    The surface is `ReachyCentralConsumer`'s — `start`, `stop`, `status`, `latest_frame`,
    `send_command` — so `app.py` holds either without knowing which. That is not politeness: the
    point of having two transports is that everything above them is the same code, and a page that
    branched on which one it had would be two pages.
    """

    def __init__(self, host: str, rpc: Rpc, port: int = DEFAULT_SIGNALLING_PORT):
        self.url = f"ws://{host}:{port}"
        self.host = host
        self._rpc = rpc

        self._session: aiohttp.ClientSession | None = None
        self._ws: aiohttp.ClientWebSocketResponse | None = None
        self._task: asyncio.Task[None] | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._pc: RTCPeerConnection | None = None

        self._self_peer_id: str | None = None
        self._robot_peer_id: str | None = None
        self._session_id: str | None = None
        self._remote_desc_set = False
        self._pending_ice: list[dict[str, Any]] = []

        self.meta: dict[str, Any] = {}
        self.error: str | None = None

        self._cmd_channel: RTCDataChannel | None = None
        self._cmd_channel_open = False
        self._latest: tuple[int, npt.NDArray[np.uint8]] | None = None
        self._frames = 0

    # ── the surface `app.py` uses ────────────────────────────────────────────

    async def start(self) -> None:
        """Open the socket, then pump it.

        The connect is awaited here rather than inside the task on purpose: a hostname that does
        not resolve, a robot that is off, and a port nothing is listening on are all worth
        reporting to whoever just typed the address, and a task that swallows them turns every one
        of them into "connecting…".
        """
        # The budget goes on the *connect* and not on the socket's life: a WebSocket that is
        # working is idle most of the time, and a `total` here would end a healthy session. Eight
        # seconds is enough for mDNS plus a TCP handshake on a LAN, and short enough that a
        # mistyped address comes back while somebody is still looking at it.
        session = aiohttp.ClientSession(
            timeout=aiohttp.ClientTimeout(total=None, sock_connect=8)
        )
        try:
            self._ws = await session.ws_connect(self.url, heartbeat=20)
        except Exception:
            await session.close()
            raise
        logger.info("signalling socket open: %s", self.url)
        self._session = session
        self._loop = asyncio.get_running_loop()
        self._task = asyncio.create_task(self._pump(), name="lan-signalling")

    async def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._task
            self._task = None
        if self._session_id and self._ws is not None and not self._ws.closed:
            with contextlib.suppress(Exception):
                await self._ws.send_json({"type": "endSession", "sessionId": self._session_id})
        if self._pc is not None:
            with contextlib.suppress(Exception):
                await self._pc.close()
            self._pc = None
        if self._ws is not None:
            with contextlib.suppress(Exception):
                await self._ws.close()
            self._ws = None
        if self._session is not None:
            with contextlib.suppress(Exception):
                await self._session.close()
            self._session = None
        self._cmd_channel, self._cmd_channel_open = None, False

    def status(self) -> dict[str, Any]:
        return {
            "connected": self._pc is not None and self._pc.connectionState == "connected",
            "self_peer_id": self._self_peer_id,
            "robot_peer_id": self._robot_peer_id,
            "session_id": self._session_id,
            "pc_state": self._pc.connectionState if self._pc is not None else None,
            "frames": self._frames,
            # Not in their status, and the whole reason this transport exists — a page that says
            # "over the LAN" is saying which failure modes do not apply.
            "transport": f"ws {self.url}",
        }

    def latest_frame(self) -> tuple[int, npt.NDArray[np.uint8]] | None:
        return self._latest

    def send_command(self, envelope: dict[str, Any]) -> bool:
        """Write one JSON line to the control channel, from whatever thread asks.

        `RTCDataChannel.send` is not thread-safe and a Gradio callback runs in a worker thread, so
        the send is marshalled onto the loop that owns the peer connection — the same arrangement,
        and for the same reason, as the one their consumer makes.
        """
        channel, loop = self._cmd_channel, self._loop
        if channel is None or not self._cmd_channel_open or loop is None:
            return False
        payload = json.dumps(envelope)

        def do_send() -> None:
            try:
                channel.send(payload)
            except Exception as e:  # noqa: BLE001 - a failed send is a closed channel, reported
                logger.warning("send failed: %r", e)

        try:
            loop.call_soon_threadsafe(do_send)
            return True
        except RuntimeError:
            return False

    # ── the signalling protocol, read off net/webrtc/protocol ────────────────

    async def _send(self, message: dict[str, Any]) -> None:
        if self._ws is None or self._ws.closed:
            raise RuntimeError("the signalling socket is closed")
        logger.info("→ %s", json.dumps(message)[:240])
        await self._ws.send_json(message)

    async def _pump(self) -> None:
        assert self._ws is not None
        try:
            async for raw in self._ws:
                if raw.type is not aiohttp.WSMsgType.TEXT:
                    continue
                try:
                    message = json.loads(raw.data)
                except ValueError:
                    logger.warning("unparseable signalling frame: %r", raw.data[:120])
                    continue
                await self._handle(message)
        except asyncio.CancelledError:
            raise
        except Exception as e:  # noqa: BLE001 - reported on the page, not raised into a callback
            self.error = f"{type(e).__name__}: {e}"
            logger.warning("signalling ended: %r", e)
        else:
            self.error = self.error or "the robot closed the signalling socket"

    async def _handle(self, message: dict[str, Any]) -> None:
        kind = message.get("type")
        logger.info("← %s", json.dumps(message)[:240])

        if kind == "welcome":
            # The server names us, then we ask who is producing. `list` takes no fields.
            self._self_peer_id = message.get("peerId")
            await self._send({"type": "list"})

        elif kind == "list":
            producers = message.get("producers") or []
            if not producers:
                self.error = (
                    "the robot's signalling server has no producer. `mediad` registers as one "
                    "when its pipeline reaches PLAYING — `journalctl -u mediad -b` is where a "
                    "pipeline that would not start says why."
                )
                return
            producer = producers[0]
            self._robot_peer_id = producer.get("id")
            self.meta = producer.get("meta") or {}
            # No `offer` of our own: the producer offers and we answer, which is the direction
            # `webrtcsink` wants — it is the side that knows what it is sending.
            await self._send({"type": "startSession", "peerId": self._robot_peer_id})

        elif kind == "sessionStarted":
            self._session_id = message.get("sessionId")
            await self._build_pc()

        elif kind == "peer":
            if message.get("sdp"):
                await self._on_offer(message["sdp"])
            elif message.get("ice"):
                await self._on_ice(message["ice"])

        elif kind == "endSession":
            self.error = "the robot ended the session"
            self._session_id = None

        elif kind == "error":
            self.error = str(message.get("details") or "the signalling server refused something")

    async def _on_offer(self, sdp: dict[str, Any]) -> None:
        if self._pc is None or sdp.get("type") != "offer":
            return
        await self._pc.setRemoteDescription(
            RTCSessionDescription(sdp=sdp["sdp"], type="offer")
        )
        self._remote_desc_set = True
        pending, self._pending_ice = self._pending_ice, []
        for candidate in pending:
            await self._add_ice(candidate)

        _log_candidates("remote", sdp["sdp"])
        answer = await self._pc.createAnswer()
        await self._pc.setLocalDescription(answer)
        _log_candidates("local", self._pc.localDescription.sdp)
        # aiortc has finished gathering by the time `setLocalDescription` returns, so the answer
        # carries every candidate we have and nothing is trickled outbound. Their consumer relies
        # on the same thing.
        await self._send(
            {
                "type": "peer",
                "sessionId": self._session_id,
                "sdp": {
                    "type": self._pc.localDescription.type,
                    "sdp": self._pc.localDescription.sdp,
                },
            }
        )

    async def _on_ice(self, ice: dict[str, Any]) -> None:
        if not self._remote_desc_set or self._pc is None:
            # Candidates before the offer are ordinary trickle, not an error.
            self._pending_ice.append(ice)
            return
        await self._add_ice(ice)

    async def _add_ice(self, ice: dict[str, Any]) -> None:
        text = (ice.get("candidate") or "").strip()
        if not text:
            return  # end-of-candidates; aiortc wants nothing for it
        try:
            candidate = candidate_from_sdp(text.removeprefix("candidate:"))
            candidate.sdpMid = ice.get("sdpMid")
            if ice.get("sdpMLineIndex") is not None:
                candidate.sdpMLineIndex = int(ice["sdpMLineIndex"])
            await self._pc.addIceCandidate(candidate)
            logger.debug("+remote candidate %s", text[:90])
        except Exception as e:  # noqa: BLE001 - one bad candidate is not a dead session
            logger.warning("addIceCandidate failed: %r (%r)", e, text[:80])

    async def _build_pc(self) -> None:
        # **No ICE servers.** This is a client on the robot's own network, which is the case that
        # needs neither STUN nor a relay — and asking for STUN here would make a LAN session wait
        # on a Google address to answer before it could offer anything.
        pc = RTCPeerConnection()
        self._pc = pc
        self._remote_desc_set = False

        # The three states that fail for unrelated reasons and look identical from outside.
        @pc.on("connectionstatechange")
        async def on_state() -> None:
            logger.info("peer connection: %s", pc.connectionState)

        @pc.on("iceconnectionstatechange")
        async def on_ice_state() -> None:
            logger.info("ice: %s", pc.iceConnectionState)

        @pc.on("icegatheringstatechange")
        async def on_gathering() -> None:
            logger.info("ice gathering: %s", pc.iceGatheringState)

        @pc.on("track")
        def on_track(track: Any) -> None:
            logger.info("track: %s", track.kind)
            if track.kind == "video":
                asyncio.ensure_future(self._consume_video(track))

        @pc.on("datachannel")
        def on_datachannel(channel: RTCDataChannel) -> None:
            if channel.label != CONTROL_LABEL:
                logger.info("ignoring data channel %r", channel.label)
                return
            logger.info("control channel attached (state=%s)", channel.readyState)
            self._cmd_channel = channel

            def opened() -> None:
                self._cmd_channel_open = True
                self._rpc.bound_to(self.send_command)
                logger.info("control channel open (%s)", self.url)

            def closed() -> None:
                self._cmd_channel_open = False
                self._rpc.abandon("the control channel closed")

            if channel.readyState == "open":
                opened()
            channel.on("open")(opened)
            channel.on("close")(closed)
            channel.on("message")(self._rpc.on_message)

    async def _consume_video(self, track: Any) -> None:
        """Decode frames so the page can show what the policy did.

        The camera is mounted a quarter turn off and nothing on the robot rotates the picture —
        `media.video` carries `rotate` for exactly this, and the control channel sends it once when
        it opens, so the page can put the duck upright without a constant anywhere.
        """
        try:
            while True:
                frame = await track.recv()
                self._frames += 1
                # `rgb24` because that is what their consumer promises `latest_frame`'s callers,
                # and the page holds either transport without asking which.
                # Every frame is decoded and only the newest is kept: the page polls, and a queue
                # of frames nobody looked at is a memory leak with extra steps.
                self._latest = (self._frames, frame.to_ndarray(format="rgb24"))
        except (MediaStreamError, asyncio.CancelledError):
            return
        except Exception as e:  # noqa: BLE001 - a dead decoder is not a dead session
            logger.warning("video ended: %r", e)


# ── checking it without a duck ────────────────────────────────────────────────
#
# `uv run lan.py` stands up a producer on loopback that speaks what `webrtcsink`'s signaller
# speaks, and drives a real session against it: welcome, list, startSession, an offer answered,
# DTLS, SCTP, the `control` channel, and a JSON-RPC call matched to its reply.
#
# **Which is the only way this file gets checked at all.** Every other part of the page can be
# read against a fake dict; the signalling protocol is a dozen envelope shapes written from
# `net/webrtc/protocol` and the console page, and getting one of them wrong produces silence
# rather than an error. Two aiortc peers on 127.0.0.1 is not a duck — it is the same protocol,
# and it catches the mistakes that are about the protocol.


async def _selfcheck(port: int = 8443) -> None:  # pragma: no cover - a script, not a test suite
    import json as _json

    import av
    from aiohttp import WSMsgType, web
    from aiortc import VideoStreamTrack

    class Stripes(VideoStreamTrack):
        """A picture with a top and a bottom, and wider than it is tall.

        Both on purpose: an upside-down frame and an unrotated one have the same shape, so the
        asymmetry is what makes `rotate` checkable at all — 180×320 turned a quarter is 320×180,
        and the red half moves to a side.
        """

        async def recv(self) -> av.VideoFrame:
            pts, time_base = await self.next_timestamp()
            picture = np.zeros((180, 320, 3), dtype=np.uint8)
            picture[:90] = (255, 0, 0)
            frame = av.VideoFrame.from_ndarray(picture, format="rgb24")
            frame.pts, frame.time_base = pts, time_base
            return frame

    answers = {"robot.policies": {"mode": "walk", "enabled": True, "slots": []}}
    calls: list[str] = []

    async def producer(request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        pc: RTCPeerConnection | None = None
        await ws.send_json({"type": "welcome", "peerId": "consumer-1"})
        async for raw in ws:
            if raw.type is not WSMsgType.TEXT:
                continue
            message = _json.loads(raw.data)
            if message["type"] == "list":
                await ws.send_json(
                    {
                        "type": "list",
                        "producers": [
                            {"id": "robot-1", "meta": {"name": "fakeducky", "kind": "microduck"}}
                        ],
                    }
                )
            elif message["type"] == "startSession":
                await ws.send_json(
                    {"type": "sessionStarted", "peerId": "robot-1", "sessionId": "session-1"}
                )
                pc = RTCPeerConnection()
                # The robot opens the channel, which is why a consumer that opens its own gets an
                # unrouted one: `mediad` calls create-data-channel per consumer.
                channel = pc.createDataChannel(CONTROL_LABEL)
                # And it sends pixels, which is the half of "run it" that a table cannot show.
                # VP8 here where a duck sends H.264 — the codec is aiortc's to pick and the
                # plumbing under test is the same: a track offered, answered, decoded, and the
                # newest frame kept.
                pc.addTrack(Stripes())

                @channel.on("message")
                def _on_call(text: str) -> None:
                    call = _json.loads(text)
                    calls.append(call["method"])
                    answer = answers.get(call["method"])
                    body = (
                        {"result": answer}
                        if answer is not None
                        else {"error": {"code": -32601, "message": "no such method here"}}
                    )
                    channel.send(_json.dumps({"jsonrpc": "2.0", "id": call["id"], **body}))

                await pc.setLocalDescription(await pc.createOffer())
                await ws.send_json(
                    {
                        "type": "peer",
                        "sessionId": "session-1",
                        "sdp": {"type": "offer", "sdp": pc.localDescription.sdp},
                    }
                )
            elif message["type"] == "peer" and message.get("sdp"):
                assert pc is not None
                await pc.setRemoteDescription(
                    RTCSessionDescription(sdp=message["sdp"]["sdp"], type=message["sdp"]["type"])
                )
        return ws

    application = web.Application()
    application.router.add_get("/", producer)
    runner = web.AppRunner(application)
    await runner.setup()
    await web.TCPSite(runner, "127.0.0.1", port).start()

    rpc = Rpc(timeout=15)
    consumer = LanConsumer("127.0.0.1", rpc, port)
    await consumer.start()
    try:
        await _exercise(consumer, rpc, calls)
    finally:
        # Torn down whatever happened: a check that fails should print why, not print why and
        # then bury it under "Unclosed client session".
        await consumer.stop()
        await runner.cleanup()
    print("\nok — the protocol this file writes is the protocol a duck answers")


async def _exercise(  # pragma: no cover - the body of the script above
    consumer: LanConsumer, rpc: Rpc, calls: list[str]
) -> None:
    for _ in range(120):
        if rpc.is_open():
            break
        await asyncio.sleep(0.25)

    assert rpc.is_open(), f"no control channel: {consumer.error}"

    for _ in range(80):
        if consumer.latest_frame() is not None:
            break
        await asyncio.sleep(0.25)
    frame = consumer.latest_frame()
    assert frame is not None, "the control channel opened and no frame ever arrived"
    identifier, picture = frame
    assert picture.shape == (180, 320, 3), picture.shape
    # `rgb24`, which is what their consumer promises `latest_frame`'s callers — so the red half
    # has to come back red rather than blue, or the page shows a duck in the wrong colours.
    # Dominance rather than equality: the codec is lossy, and pure red arrives as (251, 1, 0).
    red, green, blue = (int(channel) for channel in picture[0, 0])
    assert red > 200 and green < 40 and blue < 40, (red, green, blue)

    print("session:", consumer.status())
    print("meta:   ", consumer.meta)
    print(f"frame:   #{identifier} {picture.shape} rgb, top-left {tuple(picture[0, 0])}")
    print("call:   ", await asyncio.to_thread(rpc.call, "robot.policies"))
    try:
        await asyncio.to_thread(rpc.call, "robot.nonsense")
    except Exception as e:  # noqa: BLE001 - the refusal is the thing being shown
        print("refusal:", e)
    assert calls == ["robot.policies", "robot.nonsense"], calls


async def _ask(host: str) -> None:  # pragma: no cover - a script, not a test suite
    """Open a lane to a real duck on this network and ask it two read-only questions.

    Nothing here asks the robot to move: `robot.policies` and `robot.skills` are what it is
    running and what it could be asked to do. It does take the robot's one session slot for a few
    seconds, so its console cannot connect while this runs.
    """
    rpc = Rpc(timeout=20)
    consumer = LanConsumer(host, rpc)
    await consumer.start()
    for _ in range(160):
        if rpc.is_open():
            break
        await asyncio.sleep(0.25)
    if not rpc.is_open():
        print(f"\nno control channel: {consumer.error or consumer.status()}")
        await consumer.stop()
        return
    print(f"\nrobot:  {consumer.meta}")
    print(f"session: {consumer.status()}")
    try:
        for method in ("robot.policies", "robot.skills", "media.video"):
            print(f"\n{method}:\n  {await asyncio.to_thread(rpc.call, method)}")
    finally:
        await consumer.stop()
    print("\ndisconnected")


if __name__ == "__main__":
    import sys

    logging.basicConfig(level=logging.INFO, format="%(name)-6s %(message)s")
    if len(sys.argv) > 1:
        # A real duck on this network: `uv run lan.py olducky.local`, or its IP.
        asyncio.run(_ask(sys.argv[1]))
    else:
        # No argument: the loopback producer, which needs no robot at all.
        logging.getLogger().setLevel(logging.WARNING)
        asyncio.run(_selfcheck())
