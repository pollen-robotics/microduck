"""Simple image processing on a duck's camera, running on Hugging Face hardware.

The robot is behind somebody's router; this runs in a container in a data centre. What joins them
is the rendezvous service the robot registers with (`docs/design/remote-access-design.md` §3): the
robot is a *producer*, this is a *consumer*, and the WebRTC session between them carries H.264
which `aiortc` decodes into numpy arrays here.

## What it is for

Two things, and the second is the one that earns it a place in the repository.

**It shows the pipeline end to end** — camera to cloud to a processed picture — which is the demo.

**And it is the only thing that tests the transport from a data centre.** Every session before this
came from a browser on the robot's own network or a phone on 4G. A Space is neither: its egress NAT
is somebody else's, and the robot cannot offer a `relay` candidate while `turn.fastrtc.org` has no
DNS (§6). So the interesting outcome here is not the edge detector — it is whether `pc_state`
reaches `connected` at all, which is why the status panel reports each stage separately rather than
saying "connecting…" until somebody gives up.

## What it deliberately does not do

**It does not drive the robot.** `ReachyCentralConsumer` looks for the data channel label the
mini's daemon opens and logs `ignoring unexpected data channel: 'control'` against ours, so no
JSON-RPC reaches this process — which also means the camera geometry cannot be *asked* for. The
overlay below therefore derives it, and says so. §5.1 has the fix, which belongs on their side.
"""

from __future__ import annotations

import asyncio
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Any

import gradio as gr
import numpy as np

from reachy_mini.media.central_consumer import ReachyCentralConsumer

from filters import FILTERS, upright

# Which robot to look for. The rendezvous lists every robot an account owns, and the consumer picks
# by `meta.name` — the name `robotctl system set-name` sets.
ROBOT = os.environ.get("DUCK_NAME", "olducky")


@dataclass
class Link:
    """The one session this Space holds, and the thread that owns its event loop.

    A dedicated loop in a background thread rather than Gradio's: the connection outlives any one
    request, `aiortc` wants a single loop for the life of a peer connection, and `send_command`
    keeps a reference to whichever loop started it.

    **One session at a time, and that is the service's rule rather than a simplification here.**
    The rendezvous allows one consumer per robot; a second is answered `sessionRejected: robot
    busy`. So while this Space is connected, nobody can watch the console — which is worth knowing
    before wondering why the page went quiet.
    """

    consumer: ReachyCentralConsumer | None = None
    loop: asyncio.AbstractEventLoop | None = None
    thread: threading.Thread | None = None
    error: str | None = None
    started_at: float | None = None
    # The previous frame, for the filters that need one.
    previous: np.ndarray | None = None
    lock: threading.Lock = field(default_factory=threading.Lock)

    def start(self, token: str) -> str:
        with self.lock:
            if self.consumer is not None:
                return "already connected"
            if not token:
                return "no Hugging Face token: sign in, or set HF_TOKEN on this Space"

            self.error = None
            self.previous = None
            loop = asyncio.new_event_loop()
            thread = threading.Thread(target=loop.run_forever, name="duck-consumer", daemon=True)
            thread.start()

            consumer = ReachyCentralConsumer(
                hf_token=token,
                robot_name=ROBOT,
                consumer_label=f"microduck-vision-demo/{os.environ.get('SPACE_ID', 'local')}",
            )
            try:
                asyncio.run_coroutine_threadsafe(consumer.start(), loop).result(timeout=30)
            except Exception as e:  # noqa: BLE001 - reported, not raised into a UI callback
                self.error = f"{type(e).__name__}: {e}"
                loop.call_soon_threadsafe(loop.stop)
                return f"could not start: {self.error}"

            self.consumer, self.loop, self.thread = consumer, loop, thread
            self.started_at = time.monotonic()
            return f"connecting to {ROBOT}…"

    def stop(self) -> str:
        with self.lock:
            consumer, loop = self.consumer, self.loop
            self.consumer, self.loop, self.thread = None, None, None
            self.started_at = None
            self.previous = None
        if consumer is None or loop is None:
            return "not connected"
        try:
            asyncio.run_coroutine_threadsafe(consumer.stop(), loop).result(timeout=10)
        except Exception as e:  # noqa: BLE001 - a teardown that fails still ends the session
            return f"stopped, with a complaint: {type(e).__name__}: {e}"
        finally:
            loop.call_soon_threadsafe(loop.stop)
        return "disconnected"

    def snapshot(self) -> tuple[np.ndarray | None, dict[str, Any]]:
        consumer = self.consumer
        if consumer is None:
            return None, {}
        frame = consumer.latest_frame()
        return (frame[1] if frame is not None else None), consumer.status()


LINK = Link()


def describe(status: dict[str, Any]) -> str:
    """Say which stage the connection reached, because the stages fail differently.

    A blank "connecting…" is the worst thing this panel can say: signalling failing, ICE failing
    and a decoder producing nothing look identical from the outside and have nothing in common.
    """
    if not status:
        if LINK.error:
            return f"**not connected** — {LINK.error}"
        return "**not connected.** Press connect."

    session = status.get("session_id")
    state = status.get("pc_state")
    frames = status.get("frames") or 0

    if not session:
        return (
            "**no session yet.** The rendezvous has not paired this consumer with the robot: "
            f"either `{ROBOT}` is not registered — it needs `robotctl account login` and a "
            "network — or another consumer holds it (one at a time)."
        )
    if state != "connected":
        waited = time.monotonic() - (LINK.started_at or time.monotonic())
        return (
            f"**signalling worked and media has not** — session `{session[:8]}`, peer connection "
            f"`{state}` after {waited:.0f}s.\n\n"
            "This is the case a data centre is expected to hit: no candidate pair works. The "
            "robot offers host and srflx candidates, this container offers its own, and neither "
            "side offers a `relay` — because the TURN credentials endpoint has no DNS "
            "(`remote-access-design.md` §6). Signalling crossing while media does not is exactly "
            "that failure, and it is not a fault in this Space."
        )
    return (
        f"**connected** — session `{session[:8]}`, {frames} frames decoded. "
        f"Robot peer `{(status.get('robot_peer_id') or '?')[:8]}`."
    )


def render(filter_name: str) -> tuple[np.ndarray | None, str]:
    frame, status = LINK.snapshot()
    if frame is None:
        return None, describe(status)

    with LINK.lock:
        previous, LINK.previous = LINK.previous, frame.copy()

    picture = upright(frame)
    was = upright(previous) if previous is not None else None
    processed = FILTERS.get(filter_name, FILTERS["raw"])(picture, was)
    return processed, describe(status)


def connect(profile: gr.OAuthProfile | None, token: gr.OAuthToken | None) -> str:
    """Open the session with whichever credential this Space has.

    **A visitor's token by preference, and never the robot's.** The rendezvous maps a token to one
    peer, so a consumer authenticating as the robot would take the robot off its owner's listing —
    the same fact §3.7 records about two things sharing a credential. A visitor's OAuth token is
    also what makes a *public* Space defensible: it reaches the visitor's own robots and nobody
    else's. `HF_TOKEN` is the fallback for a private Space with one owner.
    """
    if token is not None:
        return LINK.start(token.token)
    fallback = os.environ.get("HF_TOKEN", "").strip()
    if fallback:
        return LINK.start(fallback)
    return "sign in with Hugging Face, or set an HF_TOKEN secret on this Space"


with gr.Blocks(title="duck vision demo") as demo:
    gr.Markdown(
        f"""
        # A duck's camera, processed in a data centre

        `{ROBOT}` streams H.264 to the rendezvous service; this Space is the consumer, decodes it
        with `aiortc`, and runs a few pixels of OpenCV over it. Nothing here is on the robot's
        network.

        One consumer at a time — while this is connected, the robot's console cannot open a
        session.
        """
    )

    with gr.Row():
        gr.LoginButton()
        connect_button = gr.Button("connect", variant="primary")
        disconnect_button = gr.Button("disconnect")
        chosen = gr.Dropdown(
            choices=list(FILTERS), value="edges (Canny)", label="what to run on each frame"
        )

    status = gr.Markdown("**not connected.** Press connect.")
    picture = gr.Image(label="from the robot", height=520)

    connect_button.click(connect, outputs=status)
    disconnect_button.click(lambda: LINK.stop(), outputs=status)

    # Ten a second: the stream is 30 fps and a browser will not notice the difference, while the
    # decode and the filter both cost real CPU on the smallest hardware tier.
    gr.Timer(0.1).tick(render, inputs=chosen, outputs=[picture, status])


if __name__ == "__main__":
    demo.launch(server_name="0.0.0.0", server_port=int(os.environ.get("PORT", 7860)))
