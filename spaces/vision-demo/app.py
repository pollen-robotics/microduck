"""A duck's camera, processed on Hugging Face hardware — with the robot doing the dialling.

The robot is behind somebody's router and this runs in a container in a data centre. The obvious
way to join them is the one this Space used to take: a WebRTC consumer pulls the stream through
`reachy_mini_central`. That ran into the one thing WebRTC cannot do unaided — a relay candidate —
and `turn.fastrtc.org` has no A record and its zone no NS records at all
(`docs/design/remote-access-design.md` §6). Signalling crossed and media never did, from every
Space, every time.

**So the direction is inverted and the problem disappears.** The rendezvous is used for one small
thing — telling the robot where to send frames — and the frames come **outbound from the robot** to
a WebSocket on this Space. A robot dialling out is the one thing that always works; it is doing it
right now to stay reachable at all. NAT stops being a participant, no relay is needed, and the
rendezvous carries an instruction rather than pixels, which is also why this scales where relaying
payload through a shared service would not.

    this Space ──media.stream {url: "wss://…/frames"}──► rendezvous ──► the duck
    the duck   ═════════ JPEG frames, outbound wss, direct ═════════►  this Space

Two consequences worth knowing. There is **no return media path**, so this cannot drive the robot
or speak to it except through the control lane above — which is enough to start and stop a stream
and to ask what the robot is doing. And the frames are **upright already**: `mediad` turns them
while converting, because that conversion is a per-pixel loop either way and what receives them is
a model that wants the picture the way up it was trained on.
"""

from __future__ import annotations

import logging
import os
import sys
import time
from collections import deque
from typing import Any

import gradio as gr
import numpy as np
from fastapi import FastAPI

# `spaces/shared`, flattened beside this file at publish time by `scripts/publish-space.sh`.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import receiver
import rendezvous
from control import Rpc, RpcError
from filters import FILTERS
from rendezvous import RendezvousError
from wire import WireError, WsConsumer

ON_A_SPACE = bool(os.environ.get("SPACE_ID"))

# **Whether to ask Gradio for a sign-in button at all**, and it is not a style choice.
#
# A `gr.LoginButton` sets `blocks.expects_oauth`, which makes Gradio call `attach_oauth` while the
# Blocks is being *built* — at import, before anything serves. That then does one of two things,
# and both raise in a container:
#
# - `SPACE_ID` absent → the *mocked* routes, whose `_get_mocked_oauth_info()` needs a local
#   `hf auth login`. A container has none: `ValueError: Your machine must be logged in to HF`.
# - `SPACE_ID` present but `OAUTH_CLIENT_ID` absent → the real routes, which raise
#   `OAUTH_CLIENT_ID environment variable is not set`.
#
# Either way uvicorn never binds, and Hugging Face reports an exiting container as "App process
# crashed" — with the UI apparently visible for a second, because the *previous* container was
# still serving it. That cost four rounds of blaming the port.
#
# The condition is therefore the variable Gradio actually needs, read from inside the container
# (`/env` on the hello Space is how this was established rather than guessed): OAuth is available
# when the platform provisioned it, which needs `hf_oauth: true` in the README *and* a Space that
# has rebuilt since. When it has not, a token box takes the button's place and the page still
# works — which is the property worth having, because this is exactly the state a freshly
# recreated Space is in.
OAUTH_AVAILABLE = bool(os.environ.get("OAUTH_CLIENT_ID"))

LEVEL = os.environ.get("DUCK_LOG", "INFO").upper()


class Ring(logging.Handler):
    """The last few hundred log lines, for a panel on the page.

    **A Space's logs are on a page only its owner can open**, and reading them needs write access
    to the Space — so "click the button and tell me what it said" was a round trip through
    somebody's screenshot. The panel shows what the container's stderr shows, to whoever is
    already looking at the thing that failed. `policy-shop` has the same handler for the same
    reason.
    """

    def __init__(self, size: int = 400):
        super().__init__()
        self.lines: deque[str] = deque(maxlen=size)

    def emit(self, record: logging.LogRecord) -> None:
        try:
            self.lines.append(self.format(record))
        except Exception:  # noqa: BLE001 - a logger that raises is worse than a line lost
            pass


RING = Ring()
RING.setFormatter(logging.Formatter("%(asctime)s %(name)-10s %(message)s", datefmt="%H:%M:%S"))
logging.basicConfig(
    level=LEVEL, format="%(asctime)s %(levelname)-5s %(name)-10s %(message)s", datefmt="%H:%M:%S"
)
logging.getLogger().addHandler(RING)
for chatty in ("urllib3", "httpx", "requests"):
    logging.getLogger(chatty).setLevel(logging.WARNING)
logger = logging.getLogger("vision")

# Which of the platform's variables actually arrived. The absence of `SPACE_ID` is what decides
# whether a sign-in button can exist at all, so it is worth one line at startup rather than a
# traceback later.
logger.info(
    "environment: SPACE_ID=%s SPACE_HOST=%s OAUTH_CLIENT_ID=%s -> sign-in %s",
    "set" if os.environ.get("SPACE_ID") else "absent",
    os.environ.get("SPACE_HOST") or "absent",
    "set" if os.environ.get("OAUTH_CLIENT_ID") else "absent",
    "available" if os.environ.get("OAUTH_CLIENT_ID") else "unavailable; using the token box",
)

PORT = int(os.environ.get("PORT", 7860))

# Where the robot is told to send frames. Derived rather than configured, because a wrong url here
# is a robot streaming into nowhere with nothing anywhere to say so.
#
# On a Space, `SPACE_HOST` is the Space's own hostname and `wss://` is the only thing that will
# work through the platform's proxy. Locally the default is loopback, because the common local case
# is `fake_duck.py` on this same machine — and loopback is then exactly right. A *real* robot is on
# another machine, so it needs this host's address on the robot's network instead, which is what
# the box on the page is for.
def default_receiver() -> str:
    if os.environ.get("DUCK_RECEIVER"):
        return os.environ["DUCK_RECEIVER"]
    if os.environ.get("SPACE_HOST"):
        return f"wss://{os.environ['SPACE_HOST']}/frames"
    return f"ws://127.0.0.1:{PORT}/frames"


RECEIVER_URL = default_receiver()

# What to ask the robot for. Five a second is plenty for a model and cheap on a board that is also
# running a control loop; 640 keeps a frame around 40 KB at quality 70.
FPS = float(os.environ.get("DUCK_FPS", "5"))
LONGEST = int(os.environ.get("DUCK_LONGEST", "640"))
QUALITY = int(os.environ.get("DUCK_QUALITY", "70"))


class Link:
    """The control lane to one duck: enough to start a stream and to stop it.

    No media rides this — that is the point — so it is two HTTP verbs and an SSE stream, and it
    survives every NAT the WebRTC version did not.
    """

    def __init__(self) -> None:
        self.consumer: WsConsumer | None = None
        self.rpc: Rpc | None = None
        self.robot: str | None = None

    def connect(self, token: str, peer_id: str, name: str) -> str:
        self.disconnect()
        rpc = Rpc(timeout=20)
        consumer = WsConsumer(
            token, peer_id, rpc, label=f"microduck-vision-demo/{os.environ.get('SPACE_ID', 'local')}"
        )
        try:
            consumer.start()
        except WireError as e:
            return f"**could not reach {name}:** {e}"
        self.consumer, self.rpc, self.robot = consumer, rpc, name
        return f"**connected to {name}.** Now tell it where to send frames."

    def disconnect(self) -> str:
        consumer, self.consumer, self.rpc, self.robot = self.consumer, None, None, None
        if consumer is None:
            return "not connected"
        try:
            consumer.stop()
        except Exception as e:  # noqa: BLE001 - a teardown that fails still ends this side
            return f"disconnected, with a complaint: {e}"
        return "disconnected"

    def call(self, method: str, params: dict[str, Any] | None = None) -> Any:
        if self.rpc is None:
            raise RpcError(method, {"message": "not connected to a duck"})
        return self.rpc.call(method, params)


LINK = Link()

# The frame id last handed to the browser. **Re-sending an unchanged picture is what made Safari
# complain**: the stream is five frames a second and the timer polled ten times a second, so half
# the responses re-encoded a picture the page already had — and every one of those is a PNG plus a
# markdown blob for Safari's client to parse. `gr.skip()` leaves a component untouched, so a tick
# with no new frame sends nothing at all.
LAST_SENT: dict[str, int] = {}


def token_of(typed: str | None, oauth: gr.OAuthToken | None) -> str:
    """A visitor's token, and never a mocked one.

    Outside a Space, Gradio mocks its own login and hands the app the literal string
    `mock-oauth-token-for-local-dev`, which every service correctly refuses. `HF_TOKEN` or what
    `hf auth login` stored is what a local run uses instead.
    """
    if typed and typed.strip():
        logger.info("token: the one typed into the page")
        return typed.strip()
    if oauth is not None and oauth.token and oauth.token != "mock-oauth-token-for-local-dev":
        logger.info("token: the visitor's Hugging Face sign-in")
        return oauth.token
    from huggingface_hub import get_token

    stored = (os.environ.get("HF_TOKEN") or get_token() or "").strip()
    logger.info("token: %s", "a locally stored one" if stored else "none")
    return stored


def find(typed: str, oauth: gr.OAuthToken | None) -> tuple[Any, str]:
    token = token_of(typed, oauth)
    if not token:
        return gr.update(choices=[]), (
            "sign in with Hugging Face, or paste a token above."
            if OAUTH_AVAILABLE
            else "paste a Hugging Face token above — this Space has no sign-in available."
        )
    try:
        ducks, others = rendezvous.ducks(token)
    except RendezvousError as e:
        return gr.update(choices=[]), f"**{e}**"
    if not ducks:
        extra = f" Also listed: {', '.join(others)}." if others else ""
        return gr.update(choices=[]), (
            "no ducks online for this account. A duck registers once it has been signed in "
            f"(`robotctl account login`) and has a network.{extra}"
        )
    return (
        gr.update(choices=[(d.label(), d.peer_id) for d in ducks], value=ducks[0].peer_id),
        f"{len(ducks)} duck(s). Connect, then start the stream.",
    )


def connect(peer_id: str | None, typed: str, oauth: gr.OAuthToken | None) -> str:
    if not peer_id:
        return "pick a duck first — press *find my ducks*."
    token = token_of(typed, oauth)
    if not token:
        return "sign in with Hugging Face, or set `HF_TOKEN`."
    return LINK.connect(token, peer_id, peer_id[:8])


def start(url: str, fps: float, longest: int) -> str:
    """`media.stream` — the one call this whole Space is arranged around."""
    url = (url or "").strip()
    if not url:
        return "**no receiver url.** The box beside the button is where the robot is told to send."
    if not url.startswith(("ws://", "wss://")):
        return f"**`{url}` is not a ws:// or wss:// url**, so the robot will refuse it."
    if "127.0.0.1" in url or "localhost" in url:
        # Right for `fake_duck.py` on this machine, and wrong for anything else — a real robot's
        # loopback is its own, so it would dial itself and find nothing.
        logger.info("the receiver is loopback, which only a duck on this machine can reach")

    try:
        answer = LINK.call(
            "media.stream",
            # `quality` is JPEG's, and H.264 is the default — the bitrate is the pipeline's, so
            # there is no knob here worth putting on the page.
            {"url": url, "fps": fps, "longest": int(longest), "quality": QUALITY},
        )
    except RpcError as e:
        return f"**`media.stream` refused:** {e.message}"
    logger.info("media.stream accepted: %s", answer)
    note = (
        f"**streaming to `{url}`** at {fps:g} fps, {int(longest)} px. Frames should appear within "
        "a second or two."
    )
    if ("127.0.0.1" in url or "localhost" in url) and not ON_A_SPACE:
        note += (
            " That address is loopback, so it only works for a duck running on this machine — "
            "`fake_duck.py`. A robot elsewhere needs this host's address on *its* network."
        )
    return note


def stop() -> str:
    try:
        LINK.call("media.stream", {"url": None})
    except RpcError as e:
        return f"**could not stop it:** {e.message}"
    return "**stopped.** The robot is no longer sending frames."


def render(filter_name: str, oauth: gr.OAuthProfile | None) -> tuple[Any, str]:
    """The newest frame from this visitor's own robot, processed.

    **Filed by account, so a visitor sees their own duck.** The receiver resolves the token the
    robot connected with, and this reads only that account's streams — which is what makes a
    public Space carrying somebody's camera defensible at all.
    """
    who = getattr(oauth, "username", None) or os.environ.get("DUCK_ACCOUNT", "")
    if not who:
        return None, "sign in to see your robot's frames."

    stream = receiver.FRAMES.newest(who)
    if stream is None:
        LAST_SENT.pop(who, None)
        return None, (
            "**no stream yet.** Connect to a duck and press *start streaming*. If it was already "
            "streaming, the robot may still be redialling — a Space restarts on every push, and "
            "the robot backs off before trying again."
        )
    if stream.picture is None:
        # Ordinary for the first moment of an H.264 stream: the decoder has nothing to predict
        # from until a keyframe arrives. Saying which of the two states this is saves somebody
        # deciding the Space is broken.
        waiting = (
            f"{stream.undecoded} unit(s) arrived and produced no picture yet — normal for the "
            "first moment of an H.264 stream, which cannot decode anything before a keyframe."
            if stream.undecoded
            else "the stream opened and nothing has arrived on it."
        )
        note = f"**{stream.robot}:** {waiting}"
        if stream.last_error:
            note += f" Last decode error: {stream.last_error}"
        return None, note

    # Nothing new since the last tick: say so rather than re-encoding the same frame. The filter
    # is part of the identity — switching filters has to redraw even when the frame has not moved.
    fingerprint = stream.frames * 1000 + (hash(filter_name) % 1000)
    if LAST_SENT.get(who) == fingerprint:
        return gr.skip(), gr.skip()
    LAST_SENT[who] = fingerprint

    picture = stream.picture
    processed = FILTERS.get(filter_name, FILTERS["raw"])(picture, None)
    age = time.monotonic() - stream.last_at
    encoding = (stream.hello.get("frames") or {}).get("encoding", "?")
    per_frame = stream.bytes_in / stream.frames if stream.frames else 0
    note = (
        f"**{stream.robot}** — {stream.frames} frames, {stream.fps:.1f}/s, "
        f"{picture.shape[1]}×{picture.shape[0]}, {encoding}, "
        f"{per_frame / 1024:.0f} KB/frame, {stream.bytes_in / 1e6:.1f} MB in"
    )
    if stream.stale:
        note += f". **Nothing for {age:.0f}s** — the robot stopped sending, or its wifi went."
    return processed, note


with gr.Blocks(title="duck vision demo") as demo:
    gr.Markdown(
        f"""
        # A duck's camera, processed in a data centre

        The robot **dials this Space** and pushes JPEG frames; OpenCV runs over them here. Nothing
        is on the robot's network, and no relay candidate is involved — which is why this works
        where pulling the stream over WebRTC does not, while `turn.fastrtc.org` has no DNS.

        Frames arrive at `{RECEIVER_URL}` — change it below if the robot cannot reach that.
        """
    )

    with gr.Row():
        if OAUTH_AVAILABLE:
            gr.LoginButton()
        # **Always offered, not only when the button is missing.** The sign-in stores its session
        # in a cookie, and a Space runs in an iframe on `huggingface.co` — so Safari's tracking
        # prevention drops it as third-party whatever `SameSite` says, and signing in appears to
        # do nothing. A pasted token needs no cookie, so it works in every browser and on a phone.
        # Opening the Space's own `…hf.space` URL directly also fixes it, by making the cookie
        # first-party; this is the path that does not require knowing that.
        typed_token = gr.Textbox(
            value="",
            label="…or paste a Hugging Face token",
            placeholder="hf_…",
            info=(
                "Needed if the sign-in button does nothing — Safari and some phone browsers drop "
                "the session cookie because this page is in an iframe. Opening "
                f"https://{os.environ.get('SPACE_HOST', 'this-space.hf.space')} directly works too."
                if OAUTH_AVAILABLE
                else "This Space has no sign-in button, so a token is how it knows who you are."
            ),
            type="password",
            scale=2,
        )
        finding = gr.Button("find my ducks")
        chosen = gr.Dropdown(choices=[], label="your ducks", scale=2)
        connecting = gr.Button("connect", variant="primary")
        disconnecting = gr.Button("disconnect")

    status = gr.Markdown("**not connected.**")

    with gr.Row():
        receiver_url = gr.Textbox(
            value=RECEIVER_URL,
            label="where the robot should send frames",
            info=(
                "This Space's own address, as the robot sees it."
                if ON_A_SPACE
                else "Loopback works for `fake_duck.py` on this machine; a real robot needs this "
                "host's address on its own network."
            ),
            scale=3,
        )
        fps = gr.Number(value=FPS, label="frames a second", precision=1)
        longest = gr.Number(value=LONGEST, label="longest side (px)", precision=0)
        starting = gr.Button("start streaming", variant="primary")
        stopping = gr.Button("stop")

    chosen_filter = gr.Dropdown(
        choices=list(FILTERS), value="edges (Canny)", label="what to run on each frame"
    )
    picture = gr.Image(label="from the robot", height=520)
    frame_note = gr.Markdown("")

    # Defined here rather than beside its `tick`, because the buttons below switch it on and off
    # and Gradio needs the component to exist before an event can name it.
    ticker = gr.Timer(0.5, active=False)

    with gr.Accordion("the log — every call, every frame stream, every refusal", open=False):
        gr.Markdown(
            "The same lines the container's stderr gets. `DUCK_LOG=DEBUG` adds more. Paste this "
            "rather than describing a symptom — four things meet in one button and they all fail "
            "as nothing happening."
        )
        wire_log = gr.Markdown("")

    finding.click(find, inputs=typed_token, outputs=[chosen, status])
    connecting.click(connect, inputs=[chosen, typed_token], outputs=status)
    disconnecting.click(lambda: LINK.disconnect(), outputs=status)
    # The frame poll follows the stream: on when one is asked for, off when it is stopped or the
    # session ends.
    starting.click(start, inputs=[receiver_url, fps, longest], outputs=status).then(
        lambda: gr.Timer(active=True), outputs=ticker
    )
    stopping.click(stop, outputs=status).then(
        lambda: gr.Timer(active=False), outputs=ticker
    )
    disconnecting.click(lambda: gr.Timer(active=False), outputs=ticker)

    # **Twice a second, and only while something is streaming.**
    #
    # Every tick is a request to the Space whether or not there is a new frame, and an idle page
    # polling forever is what earns a `429` from the platform's edge — which looks exactly like a
    # broken Space and is nobody's bug. So the timer starts inactive and the buttons turn it on
    # and off: a page that nobody has connected costs nothing at all.
    #
    # Two a second for a five-frame-a-second stream shows every other frame, which for a
    # perception demo is the difference between "smooth" and "polite". `render` still skips an
    # unchanged frame on top of that.
    ticker.tick(
        render, inputs=chosen_filter, outputs=[picture, frame_note], show_progress="hidden"
    )

    # The log, every three seconds and only its tail. It used to ship all four hundred lines every
    # second, which is a large payload arriving constantly for a panel that is usually closed.
    def tail() -> str:
        lines = list(RING.lines)[-40:]
        return ("```\n" + "\n".join(lines) + "\n```") if lines else ""

    # Ten seconds: it is a diagnostic, read after something went wrong rather than watched.
    gr.Timer(10.0).tick(tail, outputs=wire_log, show_progress="hidden")


# **FastAPI owns the server and Gradio is mounted into it**, not the other way round: the robot's
# frames arrive on a WebSocket route of our own, and mounting Gradio inside FastAPI is the
# documented direction. The reverse — adding routes to Gradio's app — has known WebSocket breakage,
# which is a poor thing to discover from inside a Space.
app = FastAPI()


@app.get("/env")
def env() -> dict[str, str]:
    """Which of the platform's variables arrived, and what this app concluded from them.

    An endpoint rather than a log line: reading a Space's logs needs write access to the Space, and
    every question about this has otherwise cost a round trip through a pasted log. Presence only,
    never a value — `OAUTH_CLIENT_SECRET` is in this environment too.
    """
    shown = ("SPACE_ID", "SPACE_HOST", "PORT")
    watched = shown + ("OAUTH_CLIENT_ID", "OAUTH_SCOPES", "HF_TOKEN", "GRADIO_SSR_MODE")
    answer = {
        name: (os.environ[name] if name in shown else "set") if name in os.environ else "absent"
        for name in watched
    }
    answer["sign_in"] = "available" if OAUTH_AVAILABLE else "token box"
    answer["receiver"] = RECEIVER_URL
    return answer


receiver.mount(app)
# `ssr_mode=False` for the reason the Dockerfile sets `GRADIO_SSR_MODE=false`: server-side
# rendering starts a Node server, this image has no Node, and a page behind a sign-in has no use
# for SEO. Passed here as well as set there so the answer does not depend on the environment.
app = gr.mount_gradio_app(app, demo, path="/", ssr_mode=False)


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=int(os.environ.get("PORT", 7860)))
