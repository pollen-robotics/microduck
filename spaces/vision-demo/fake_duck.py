"""A duck that is a laptop: registers with the rendezvous, answers `media.stream`, and pushes H.264.

For testing a **deployed** Space from outside its network without a board. It plays the robot's
whole side of `remote-access-design.md` §5.3, using the same wire `mediad` uses:

1. `GET /events` with a bearer token, then `setPeerStatus {roles:["producer"]}` — which is what
   puts it in the account's robot list, so *find my ducks* finds it. `mediad::relay` does this.
2. Answers `peer` envelopes carrying `rpc` — the control lane (§5.2). Enough of the API to be
   driven: `media.stream`, and read-only answers for what a client asks while looking around.
3. On `media.stream {url}`, dials that url and pushes H.264 access units, Annex-B, one per
   message. `mediad::stream` does this.

Frames come from the webcam if there is one, and from a moving test pattern if there is not.

## Two tokens, and this is the thing that will waste an afternoon otherwise

**The rendezvous keys a peer by token string.** A second `/events` on the same token supersedes the
first (§3.7), so if this duck and the Space sign in with the *same* token they take turns
existing: the Space connects, the duck vanishes from the listing, and nothing works in a way that
looks like the duck crashed.

So give this one its own token — a second access token on the **same Hugging Face account**, from
`huggingface.co/settings/tokens`. Same account matters: the rendezvous only lets a consumer open a
session with a producer whose `username` matches, so a duck on another account is invisible rather
than shared.

    DUCK_HF_TOKEN=hf_the_second_one uv run fake_duck.py

Everything else is `HF_TOKEN` or `hf auth login` as usual, and the Space keeps using that one.

The variable is `DUCK_HF_TOKEN` and not `DUCK_TOKEN` for a boring reason worth writing down:
`DUCK_TOKEN` was already set in the environment this was first run in, to a GitHub token, and the
rendezvous spent ten seconds failing to resolve it before answering `401 Invalid token`. Which
read exactly like a broken script. Hence the name, and hence the two log lines below that say
which credential is in use and whether it is even shaped like a Hugging Face one.

**It is a prototype, not a simulator.** No config, no policies, no motion — it answers the handful
of calls a client asks while finding its way to the camera, and refuses the rest by name.
"""

from __future__ import annotations

import fractions
import json
import logging
import os
import sys
import socket as socket_module
import threading
import time
from typing import Any

import av
import numpy as np
import requests
import websockets.sync.client as wsc

RENDEZVOUS = os.environ.get(
    "REACHY_CENTRAL_URL", "https://pollen-robotics-reachy-mini-central.hf.space"
).rstrip("/")
NAME = os.environ.get("DUCK_NAME", "a-laptop-pretending")
FPS = int(os.environ.get("DUCK_FPS", "5"))
WIDTH, HEIGHT = 640, 360

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)-6s %(message)s",
                    datefmt="%H:%M:%S")
logger = logging.getLogger("duck")


# ── the camera ───────────────────────────────────────────────────────────────


def pictures():
    """RGB frames from the webcam, or a moving pattern when there is no camera."""
    import cv2

    camera = cv2.VideoCapture(0)
    if camera.isOpened():
        logger.info("using the webcam")
        try:
            while True:
                ok, frame = camera.read()
                if not ok:
                    break
                yield cv2.resize(frame, (WIDTH, HEIGHT))[:, :, ::-1]
        finally:
            camera.release()
        return

    logger.info("no webcam; sending a test pattern")
    step = 0
    while True:
        picture = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        picture[: HEIGHT // 2] = (200, 40, 40)
        picture[:, step % (WIDTH - 40) : step % (WIDTH - 40) + 40] = 255
        step += 12
        yield picture
        time.sleep(1.0 / FPS)


# ── pushing frames at whatever asked for them ────────────────────────────────


class Pusher:
    """One outbound frame stream, started and stopped by `media.stream`."""

    def __init__(self) -> None:
        self.url: str | None = None
        self.sent = 0
        self.bytes_out = 0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self, url: str, token: str) -> None:
        self.stop()
        self._stop.clear()
        self.url, self.sent, self.bytes_out = url, 0, 0
        self._thread = threading.Thread(
            target=self._run, args=(url, token), name="pusher", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self.url = None

    def status(self) -> dict[str, Any]:
        return {
            "streaming": self.url is not None,
            "url": self.url,
            "sent": self.sent,
            "dropped": 0,
            "connected": self.url is not None,
            "fps": FPS,
            "longest": WIDTH,
            "encoding": "h264",
        }

    def _run(self, url: str, token: str) -> None:
        container = av.open("/dev/null", "w", format="h264")
        stream = container.add_stream("libx264", rate=FPS)
        stream.width, stream.height, stream.pix_fmt = WIDTH, HEIGHT, "yuv420p"
        # `repeat-headers` is `h264parse config-interval=-1`'s job on the robot: parameter sets in
        # front of every keyframe, so a receiver that connects mid-stream needs nothing it missed.
        stream.options = {"tune": "zerolatency", "g": str(FPS), "x264-params": "repeat-headers=1"}

        try:
            logger.info("dialling %s", url)
            with wsc.connect(url, additional_headers={"authorization": f"Bearer {token}"}) as sock:
                sock.send(
                    json.dumps(
                        {
                            "type": "hello",
                            "robot": {"name": NAME, "kind": "microduck", "release": "fake"},
                            "frames": {
                                "encoding": "h264",
                                "annexb": True,
                                "fps": FPS,
                                "longest": WIDTH,
                                "rotate": 0,
                            },
                        }
                    )
                )
                logger.info("streaming to %s", url)
                interval = 1.0 / FPS
                for index, picture in enumerate(pictures()):
                    if self._stop.is_set():
                        break
                    began = time.monotonic()
                    frame = av.VideoFrame.from_ndarray(
                        np.ascontiguousarray(picture), format="rgb24"
                    )
                    frame.pts = index
                    frame.time_base = fractions.Fraction(1, FPS)
                    for packet in stream.encode(frame):
                        unit = bytes(packet)
                        sock.send(unit)
                        self.sent += 1
                        self.bytes_out += len(unit)
                    if self.sent and self.sent % (FPS * 5) == 0:
                        logger.info(
                            "%d units, %.0f KB, %.0f B/unit",
                            self.sent,
                            self.bytes_out / 1024,
                            self.bytes_out / self.sent,
                        )
                    rest = interval - (time.monotonic() - began)
                    if rest > 0:
                        time.sleep(rest)
        except Exception as e:  # noqa: BLE001 - a prototype says why and stops
            logger.warning("the frame stream ended: %r", e)
        finally:
            self.url = None


# ── the control lane, from the robot's side ──────────────────────────────────


ANSWERS: dict[str, Any] = {
    # Enough for a client to look around and find the camera. A real duck answers these out of
    # `robotd`; this one is a prototype and says so by having no policies at all.
    "robot.policies": {"mode": "walk", "enabled": False, "slots": [], "skills": []},
    "robot.skills": {"skills": [], "built_in": []},
    "media.video": {"width": WIDTH, "height": HEIGHT, "rotate": 0},
}


class FakeDuck:
    def __init__(self, token: str) -> None:
        self.token = token
        # **Two sessions, and this is not tidiness.** `requests.Session` is not thread-safe, and
        # this holds a streaming `GET /events` open on one thread while the heartbeat and every
        # reply POST from another. Sharing one connection pool across that is a corrupted stream:
        # messages arrive mangled or not at all, which showed up as a `startSession` this duck
        # never saw followed by two `endSession`s it could not explain.
        self.streaming = requests.Session()
        self.posting = requests.Session()
        self.peer_id: str | None = None
        self.session_id: str | None = None
        self.pusher = Pusher()

    def headers(self) -> dict[str, str]:
        return {"authorization": f"Bearer {self.token}"}

    def post(self, message: dict[str, Any]) -> dict[str, Any] | None:
        answer = self.posting.post(
            f"{RENDEZVOUS}/send", headers=self.headers(), json=message, timeout=20
        )
        if answer.status_code != 200:
            logger.warning("POST /send -> HTTP %s %s", answer.status_code, answer.text[:120])
            return None
        try:
            body = answer.json()
        except ValueError:
            return None
        return body if isinstance(body, dict) else None

    def register(self) -> None:
        """`setPeerStatus`, which is what puts this in the account's robot list.

        `hardware_id` matters: the rendezvous only sweeps producers that carry one, so a fake duck
        without it would haunt the listing after this process exits (§3.7).
        """
        self.post(
            {
                "type": "setPeerStatus",
                "roles": ["producer"],
                "meta": {
                    # **Stable across runs, not per process.** The rendezvous evicts an older
                    # producer of the same user carrying the same `hardware_id`, which is how a
                    # restarted robot replaces itself instead of appearing twice (§3.7). With the
                    # pid in here every run of this script left another duck in the listing —
                    # which is exactly the ghost that field exists to prevent. `mediad` uses the
                    # SoC serial for the same reason.
                    "hardware_id": f"fake-{socket_module.gethostname()}",
                    "name": NAME,
                    "kind": "microduck",
                    "release": "fake",
                    "api_version": 23,
                },
            }
        )

    def heartbeat(self) -> None:
        """The lease is refreshed by inbound POSTs, not by a socket that looks healthy (§3.3)."""
        while True:
            time.sleep(10)
            self.register()

    def rpc(self, call: dict[str, Any]) -> dict[str, Any]:
        method, params = call.get("method"), call.get("params") or {}
        logger.info("← %s %s", method, json.dumps(params)[:160])

        if method == "media.stream":
            if "url" not in params:
                return {"result": self.pusher.status()}
            if params["url"] is None:
                self.pusher.stop()
                return {"result": {"streaming": False, "was": None}}
            self.pusher.start(params["url"], self.token)
            return {"result": self.pusher.status()}

        if method in ANSWERS:
            return {"result": ANSWERS[method]}
        return {"error": {"code": -32601, "message": f"{method} is not something a fake duck does"}}

    def handle(self, message: dict[str, Any]) -> None:
        kind = message.get("type")
        if kind == "welcome":
            self.peer_id = message.get("peerId")
            logger.info(
                "registered as %s for %s", (self.peer_id or "?")[:8], message.get("username")
            )
            self.register()
            threading.Thread(target=self.heartbeat, daemon=True).start()
        elif kind == "startSession":
            self.session_id = message.get("sessionId")
            logger.info("a consumer opened session %s", (self.session_id or "?")[:8])
        elif kind == "endSession":
            logger.info("the session ended")
            self.session_id = None
            self.pusher.stop()
        elif kind == "peer" and message.get("rpc") is not None:
            call = message["rpc"]
            body = self.rpc(call)
            self.post(
                {
                    "type": "peer",
                    "sessionId": message.get("sessionId"),
                    "rpc": {"jsonrpc": "2.0", "id": call.get("id"), **body},
                }
            )

    def run(self) -> None:
        logger.info("GET %s/events", RENDEZVOUS)
        stream = self.streaming.get(
            f"{RENDEZVOUS}/events",
            headers={**self.headers(), "accept": "text/event-stream"},
            stream=True,
            timeout=(20, 120),
        )
        logger.info("GET /events -> HTTP %s", stream.status_code)
        if stream.status_code != 200:
            # The status *and* the body: "refused this token" was the message here first, and it
            # is a guess dressed as a diagnosis — the same mistake as the Space's 403.
            sys.exit(
                f"the rendezvous answered HTTP {stream.status_code}: {stream.text[:300]}"
            )

        buffer = ""
        for chunk in stream.iter_content(chunk_size=None):
            buffer += chunk.decode("utf-8", "replace").replace("\r\n", "\n")
            while "\n\n" in buffer:
                frame, _, buffer = buffer.partition("\n\n")
                data = "".join(
                    line[len("data:") :].strip()
                    for line in frame.split("\n")
                    if line.startswith("data:")
                )
                if data:
                    try:
                        self.handle(json.loads(data))
                    except Exception as e:  # noqa: BLE001 - a prototype logs and carries on
                        logger.warning("a message went wrong: %r", e)


def main() -> None:
    from huggingface_hub import get_token

    token = os.environ.get("DUCK_HF_TOKEN")
    if not token:
        token = os.environ.get("HF_TOKEN") or get_token()
        logger.warning(
            "no DUCK_HF_TOKEN, so this duck is using the same token the Space will. The "
            "rendezvous keys a peer by token, so whichever connects second evicts the first — see "
            "this file's header. Fine for checking the listing; not for the whole loop."
        )
    if not token:
        sys.exit("no token: set DUCK_TOKEN, or run `hf auth login`")
    # The shape, never the token: which credential this is, is the thing that explains a refusal,
    # and it is the half that is safe to write down.
    logger.info(
        "token: %d chars, starts %r, from %s",
        len(token),
        token[:8],
        "DUCK_HF_TOKEN"
        if os.environ.get("DUCK_HF_TOKEN")
        else "HF_TOKEN"
        if os.environ.get("HF_TOKEN")
        else "`hf auth login`",
    )
    if not token.startswith("hf_"):
        # Cheap, and it is the check that would have saved the afternoon this file's header
        # describes: only Hugging Face tokens mean anything to `whoami-v2`, and everything else
        # comes back as a slow, unexplained 401.
        logger.warning(
            "that token does not start with `hf_`, so it is probably not a Hugging Face one — the "
            "rendezvous will answer 401 after about ten seconds of trying to resolve it"
        )
    FakeDuck(token).run()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logger.info("stopped")
