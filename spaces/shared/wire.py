"""JSON-RPC to a duck over the rendezvous, with no WebRTC in the path at all.

The other two transports negotiate: they exchange SDP and ICE so the two ends can find a route
between them, and then speak WebRTC over it. That works on one network and it works across NATs
*when a relay candidate exists* — which right now it does not, because `turn.fastrtc.org` has no A
record and its zone has no NS records (`remote-access-design.md` §6). Signalling crosses, media
does not, and the control channel goes with it because SCTP rides the same candidate pair.

This needs none of it. The rendezvous already relays what is wanted: `handle_peer_message` in its
`app.py` forwards **every key of a `peer` envelope except `type` and `sessionId`** verbatim to the
session partner, without looking at `sdp` or `ice`. So a `peer` envelope carrying an `rpc` key is a
control call, and `mediad::relay`'s control lane answers it out of the same routing table and the
same per-lane sockets the datachannel uses.

    POST /send  {"type": "peer", "sessionId": S, "rpc": {"jsonrpc": "2.0", "id": 1, …}}
    SSE         {"type": "peer", "sessionId": S, "rpc": {"jsonrpc": "2.0", "id": 1, "result": …}}

Two HTTP verbs, one stream, and nothing a NAT can refuse. What it costs:

**No video.** Pixels are RTP on the media path. Over this they would be base64 inside JSON at the
rate limit below — a snapshot, not a stream — so this transport shows none, and the page says so.

**It is not a teleop lane.** The rendezvous allows 1200 requests per 60 s per peer, and going over
earns a `429` on everything that token does. The robot's lane budgets its own notifications for
that reason (`NOTIFICATIONS_PER_WINDOW`); this side simply does not stream, and a click is four
calls.

**The peer is bound by the stream.** `POST /send` before `GET /events` is a 400 — identity comes
from the bearer token and the stream is what binds it — so the stream opens first and everything
else follows the `welcome`. Which is the same order `mediad::relay` uses on the robot's side, for
the same reason.

**And `startSession`'s answer is in the POST body**, not on the stream, which is the one shape a
reader of this protocol gets wrong once. `list` is the same. Everything else arrives on the stream.
"""

from __future__ import annotations

import contextlib
import json
import logging
import queue
import threading
from typing import Any

import requests

from rendezvous import DEFAULT_CENTRAL_URL

logger = logging.getLogger(__name__)

# Long enough for a Space's egress and a cold Space on the other end.
CONNECT_TIMEOUT = 20
# The SSE stream sends a comment ping every 30 s to hold itself open through its own proxy, so
# silence for appreciably longer than that is a dead stream rather than a quiet one.
READ_TIMEOUT = 90


class WireError(Exception):
    """Something the person pressing the button can act on."""


class WsConsumer:
    """One control-only session with one duck.

    The surface is the other two consumers' — `start`, `stop`, `status`, `latest_frame`,
    `send_command` — so `app.py` holds any of the three without knowing which. `latest_frame`
    always answers `None` here, which is not a stub: there is no video on this transport, and a
    page that showed the last frame of a *previous* session would be lying about what it is
    looking at.
    """

    def __init__(self, token: str, peer_id: str, rpc: Any, base: str = DEFAULT_CENTRAL_URL,
                 label: str = "microduck-policy-shop"):
        self._token = token
        self._peer_id = peer_id
        self._rpc = rpc
        self._base = base.rstrip("/")
        self._label = label

        # **Two sessions, because `requests.Session` is not thread-safe.** The pump thread holds
        # a streaming `GET /events` open for the life of the connection while every call POSTs
        # from whichever Gradio worker thread the click landed on. One shared connection pool
        # across those two is a stream that loses messages — a reply that never arrives, or a
        # `startSession` the far end never sees.
        self._streaming = requests.Session()
        self._posting = requests.Session()
        self._stream: requests.Response | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

        self._self_peer_id: str | None = None
        self._session_id: str | None = None
        self._welcome: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=1)
        self.meta: dict[str, Any] = {}
        self.error: str | None = None

    # ── the surface `app.py` uses ────────────────────────────────────────────

    def start(self) -> None:
        """Open the stream, register, and ask for a session.

        Synchronous and blocking, because every step has to happen before the next one can and
        because a caller with nothing to do while it waits is not made better by a coroutine.
        """
        self._open_stream()
        self._thread = threading.Thread(target=self._pump, name="duck-wire", daemon=True)
        self._thread.start()

        try:
            welcome = self._welcome.get(timeout=CONNECT_TIMEOUT)
        except queue.Empty:
            raise WireError(
                "the rendezvous accepted the stream and never said hello"
            ) from None
        self._self_peer_id = welcome.get("peerId")
        logger.info(
            "welcome: peer %s, account %s",
            (self._self_peer_id or "?")[:8],
            welcome.get("username") or "unknown",
        )

        # A name in the listing, so the owner's other devices can see who holds the robot — the
        # service reports a consumer's `meta.name` back as `activeApp`.
        self._post({"type": "setPeerStatus", "roles": ["listener"], "meta": {"name": self._label}})

        # **The session is what makes a `peer` envelope routable**: the service drops one naming a
        # session it does not know. Nothing about it commits either end to WebRTC — no offer is
        # sent, and the robot's control lane does not need one.
        answer = self._post({"type": "startSession", "peerId": self._peer_id})
        kind = (answer or {}).get("type")
        if kind == "sessionRejected":
            raise WireError(
                "this duck is busy with "
                f"{(answer or {}).get('activeApp') or 'something else'} — one consumer at a time, "
                "and the robot's own console counts."
            )
        if kind == "error":
            raise WireError(str((answer or {}).get("details") or "the rendezvous refused"))
        if kind != "sessionStarted":
            raise WireError(f"`startSession` answered {answer!r} rather than a session")
        self._session_id = (answer or {}).get("sessionId")
        logger.info("session %s open, control only", (self._session_id or "?")[:8])
        self._rpc.bound_to(self.send_command)

    def stop(self) -> None:
        # Set first, so the pump reads it as deliberate rather than reporting a failed stream.
        self._stop.set()
        if self._session_id:
            try:
                self._post({"type": "endSession", "sessionId": self._session_id})
            except WireError:
                # A teardown that cannot be delivered still ends this side of the session, and the
                # robot's lane is dropped by the service's own endSession broadcast either way.
                pass
        self._session_id = None
        if self._stream is not None:
            self._stream.close()
            self._stream = None
        self._streaming.close()
        self._posting.close()

    def status(self) -> dict[str, Any]:
        return {
            "connected": self._session_id is not None,
            "self_peer_id": self._self_peer_id,
            "robot_peer_id": self._peer_id,
            "session_id": self._session_id,
            # There is no peer connection, which is the whole point — so it is reported as the
            # state it is in rather than left blank for a panel to interpret as a failure.
            "pc_state": "connected" if self._session_id else None,
            "frames": 0,
            "transport": "the rendezvous, control only",
        }

    @staticmethod
    def latest_frame() -> None:
        return None

    def send_command(self, envelope: dict[str, Any]) -> bool:
        """One JSON-RPC object out, as a `peer` envelope. Thread-safe by having no shared state."""
        if not self._session_id:
            return False
        try:
            self._post({"type": "peer", "sessionId": self._session_id, "rpc": envelope})
        except WireError as e:
            logger.warning("a call could not be posted: %s", e)
            return False
        return True

    # ── the wire ────────────────────────────────────────────────────────────

    def _open_stream(self) -> None:
        logger.info("GET %s/events", self._base)
        try:
            stream = self._streaming.get(
                f"{self._base}/events",
                headers={
                    "authorization": f"Bearer {self._token}",
                    "accept": "text/event-stream",
                },
                stream=True,
                timeout=(CONNECT_TIMEOUT, READ_TIMEOUT),
            )
        except requests.RequestException as e:
            raise WireError(f"the rendezvous could not be reached: {e}") from None
        logger.info("GET %s/events -> HTTP %s", self._base, stream.status_code)
        if stream.status_code == 401:
            raise WireError("the rendezvous refused this token — `whoami-v2` did not know it.")
        if stream.status_code != 200:
            raise WireError(f"the event stream answered HTTP {stream.status_code}")
        self._stream = stream

    def _post(self, message: dict[str, Any]) -> dict[str, Any] | None:
        """`POST /send`, returning the body when the service answers in it.

        `startSession` and `list` are answered in the response body; everything else comes back on
        the stream and answers `{"status": "ok"}` here. A reader that discards this body loses the
        session id and then fails on the robot's offer for a session it was never told about —
        which is a mistake the console page made first and documented.
        """
        redacted = {k: v for k, v in message.items() if k != "rpc"}
        logger.info("→ %s %s", message.get("type"), json.dumps(redacted.get("rpc") or redacted)[:180])
        try:
            answer = self._posting.post(
                f"{self._base}/send",
                headers={"authorization": f"Bearer {self._token}"},
                json=message,
                timeout=CONNECT_TIMEOUT,
            )
        except requests.RequestException as e:
            raise WireError(f"POST /send: {e}") from None
        if answer.status_code == 429:
            raise WireError(
                "the rendezvous is rate-limiting this token (1200 requests a minute). "
                "Something is streaming over a lane that is not for streaming."
            )
        if answer.status_code == 400:
            raise WireError(
                "the rendezvous says this peer does not exist — its event stream is gone. "
                "Disconnect and connect again."
            )
        if answer.status_code != 200:
            raise WireError(f"POST /send answered HTTP {answer.status_code}: {answer.text[:200]}")
        try:
            body = answer.json()
        except ValueError:
            return None
        if isinstance(body, dict) and body.get("type"):
            logger.info("← (body) %s", json.dumps(body)[:200])
            return body
        return None

    def _pump(self) -> None:
        """Read the stream, hand `rpc` payloads to `Rpc`, and remember the welcome.

        **CRLF normalised on arrival, because the framing is not ours.** SSE permits `\\r\\n` and a
        proxy is free to rewrite them; splitting on `\\n\\n` alone then matches nothing and every
        message is swallowed in silence — a stream that looks connected and says nothing.
        """
        assert self._stream is not None
        buffer = ""
        try:
            for chunk in self._stream.iter_content(chunk_size=None):
                if self._stop.is_set():
                    return
                buffer += chunk.decode("utf-8", "replace").replace("\r\n", "\n")
                while "\n\n" in buffer:
                    frame, _, buffer = buffer.partition("\n\n")
                    data = "".join(
                        line[len("data:") :].strip()
                        for line in frame.split("\n")
                        if line.startswith("data:")
                    )
                    if data:
                        self._handle(data)
        except Exception as e:  # noqa: BLE001 - see below; a reader cannot pick its exception
            # **Broad on purpose, and the breadth is the point.** `stop` closes the response while
            # this thread is blocked inside `iter_content`, and what comes out of that is not a
            # `RequestException`: it is whatever urllib3 was in the middle of — a `ProtocolError`,
            # a closed-socket `ValueError`, an `AttributeError` on a released connection. Catching
            # only the polite one printed `Exception in thread duck-wire` after every clean
            # disconnect, which is a traceback that says a bug where there is none, next to the
            # word "disconnected".
            if not self._stop.is_set():
                self.error = f"the event stream failed: {type(e).__name__}: {e}"
                logger.warning("%s", self.error)
            else:
                logger.debug("the event stream closed on teardown: %r", e)
        finally:
            if not self._stop.is_set():
                self.error = self.error or "the rendezvous closed the event stream"
                self._rpc.abandon(self.error)

    def _handle(self, data: str) -> None:
        try:
            message = json.loads(data)
        except ValueError:
            logger.warning("← unparseable frame: %s", data[:160])
            return

        kind = message.get("type")
        if kind == "welcome":
            # A second welcome supersedes nothing here: the queue holds the first, which is the
            # one `start` is waiting on.
            with contextlib.suppress(queue.Full):
                self._welcome.put_nowait(message)
            return

        if kind == "peer":
            payload = message.get("rpc")
            if payload is None:
                # An `sdp` or an `ice`: the robot's media path answering a negotiation this
                # transport never started. Nothing here can use it, and it is not an error.
                logger.debug("← a peer envelope with no rpc; ignoring it")
                return
            self._rpc.on_message(json.dumps(payload))
            return

        if kind in ("endSession", "sessionRejected"):
            self.error = (
                f"the session ended: {message.get('reason') or 'no reason given'}"
            )
            logger.info("← %s", json.dumps(message)[:200])
            self._session_id = None
            self._rpc.abandon(self.error)
            return

        logger.debug("← %s", json.dumps(message)[:200])


def connect(token: str, peer_id: str, rpc: Any, label: str) -> WsConsumer:
    consumer = WsConsumer(token, peer_id, rpc, label=label)
    consumer.start()
    return consumer


if __name__ == "__main__":
    # ── checking the lane against the real rendezvous and a real duck ─────────
    #
    # `uv run wire.py` — no Gradio, no browser, and read-only calls: it asks the robot what it is
    # running and what it can do, and asks for nothing to move. Which makes it the thing to run
    # first when a click does not work, because it isolates the transport from everything else on
    # the page.
    #
    # A timeout here has exactly one likely cause and the message says so: the envelope crossed
    # the rendezvous and the robot did not answer it, which is a `mediad` without the control
    # lane. That was every duck until this branch is installed on one.
    import logging as _logging
    import os
    import sys

    import rendezvous
    from control import Rpc, RpcError
    from huggingface_hub import get_token

    _logging.basicConfig(level=_logging.INFO, format="%(name)-10s %(message)s")

    credential = os.environ.get("HF_TOKEN") or get_token()
    if not credential:
        sys.exit("no token: run `hf auth login`, or set HF_TOKEN")

    wanted = sys.argv[1] if len(sys.argv) > 1 else None
    try:
        found, others = rendezvous.ducks(credential)
    except rendezvous.RendezvousError as e:
        sys.exit(str(e))
    if not found:
        sys.exit(f"no ducks online for this account{f'; also listed: {others}' if others else ''}")

    duck = next((d for d in found if wanted in (d.name, d.peer_id)), None) if wanted else found[0]
    if duck is None:
        sys.exit(f"no duck called {wanted!r}; found {[d.name for d in found]}")
    print(f"\n{duck.label()}\n  peer {duck.peer_id}\n")
    if duck.busy:
        sys.exit("it is busy — one consumer at a time, and the robot's own console counts.")

    # Shorter than the page's, because a person is watching this one.
    rpc = Rpc(timeout=15)
    consumer = WsConsumer(credential, duck.peer_id, rpc, label="microduck-policy-shop/wire-check")
    try:
        consumer.start()
    except WireError as e:
        sys.exit(f"\ncould not open the lane: {e}")

    try:
        for method in ("robot.policies", "robot.skills"):
            try:
                print(f"\n{method}:\n  {rpc.call(method)}")
            except RpcError as e:
                if "no answer in" in e.message:
                    print(
                        f"\n{method}: no answer.\n\n"
                        "  The envelope crossed the rendezvous — the session opened, so the robot "
                        "is\n  listening — and nothing answered it. That is a `mediad` without the "
                        "control\n  lane: it reads a `peer` envelope carrying `rpc` as a step of a "
                        "negotiation it\n  has no session for, and drops it. Install this branch on "
                        "the board:\n\n    scripts/dev-push.sh --name "
                        f"{duck.name}\n"
                    )
                    break
                print(f"\n{method}: refused — {e.message}")
    finally:
        consumer.stop()
        print("\ndisconnected")
