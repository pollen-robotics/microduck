"""Pick a policy off the Hub, put it on your duck, and watch it run — from anywhere.

The robot is behind somebody's router and this is a container in a data centre; what joins them is
the rendezvous service the duck registers with (`docs/design/remote-access-design.md` §3). The
robot is a *producer*, this is a *consumer*, and the WebRTC session between them carries the
camera and one data channel called `control` — JSON-RPC 2.0, `duck-ipc-proto`'s own wire, the same
lines `robotctl` sends over a unix socket.

## What the one click is

Four calls, in the order `robotctl policy add` makes them, and nothing invented:

    policy.fetch    {repo, file}   → updaterd downloads it and reads the manifest beside it
    robot.setSkill  {name, path…}  → robotd writes the entry and re-reads its skills
    robot.policies                 → did the reload take? `change_error` is where it says no
    robot.do        {skill}        → run it

Every one of them is in `mediad/src/route.rs`'s permitted set already, and the pipe is dumb: no
method here is known to `mediad`, which is what `remote-webrtc.md` §5 means by the control channel
being a pipe to the existing API.

## Two things that will bite before the robot does

**Media may not connect from a data centre, and it is not this Space's fault.** A relay candidate
needs `turn.fastrtc.org`, which has no DNS at all right now (§6), so the session falls back to
host and srflx — often enough to punch a hole, and often enough not. The control channel is SCTP
over that same candidate pair, so when it does not punch, nothing here works. Running this file on
a laptop on the robot's own network is the way through — `uv run app.py`, and the README's
local-run section is the two lines that get there.

**One consumer at a time.** That is the rendezvous's rule, not a simplification here: while this
Space holds a session, the robot's own console cannot open one, and the vision demo cannot either.
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
import os
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any

import gradio as gr
import numpy as np

import catalogue
import rendezvous
from catalogue import Policy
from control import Rpc, RpcError
from lan import DEFAULT_SIGNALLING_PORT, LanConsumer
from rendezvous import RendezvousError
from wire import WireError, WsConsumer


# ── the log ──────────────────────────────────────────────────────────────────
#
# **Every request, every frame and every refusal, to the terminal and to the page.** Not a
# convenience: three protocols meet here and none of them fails loudly. A rendezvous that answers
# 401, a candidate pair that never forms, a data channel with the wrong label and a policy the
# robot refuses on its shape all present as "the button did nothing", and the difference between
# them is one line each. `DUCK_LOG=DEBUG` adds the streaming notifications and the individual ICE
# candidates.


class Ring(logging.Handler):
    """The last few hundred log lines, for the panel at the bottom of the page.

    The page shows what the terminal shows, because whoever is looking at one is usually not
    looking at the other — a Space has logs nobody has open, and a browser has no stderr.
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

LEVEL = os.environ.get("DUCK_LOG", "INFO").upper()
logging.basicConfig(
    level=LEVEL, format="%(asctime)s %(levelname)-5s %(name)-10s %(message)s", datefmt="%H:%M:%S"
)
logging.getLogger().addHandler(RING)
# `aioice` logs a line per candidate pair check, which buries everything else. It is exactly what
# is wanted when ICE is the suspect, and nothing but noise until then.
for chatty in ("aioice", "aiortc", "aiohttp", "httpx", "urllib3"):
    logging.getLogger(chatty).setLevel(logging.DEBUG if LEVEL == "DEBUG" else logging.WARNING)

logger = logging.getLogger("shop")

# How long to hold a policy that declares no length of its own. Perpetual means "until told
# otherwise", so something has to choose, and `robotctl policy add` refuses rather than guessing —
# a page can ask instead, which is what the number beside the button is.
DEFAULT_HOLD = 3.0

# Rows drawn for the catalogue. Gradio wants its components at build time, so this is a ceiling
# rather than a count: 23 policies were published when this was written.
MAX_ROWS = 40

# What Gradio's local login mock puts in `access_token`. A constant rather than a `SPACE_ID`
# check, because it is the exact thing to refuse: a real token that happened to arrive outside a
# Space should still be used, and this string should never be sent anywhere whatever set it.
MOCK_TOKEN = "mock-oauth-token-for-local-dev"

# Whether this process *is* a Space, which changes what two of the tabs can honestly promise. Set
# by the platform, and the same variable Gradio checks to decide whether to mock its login.
ON_A_SPACE = bool(os.environ.get("SPACE_ID"))

# The download happens on the robot, over the robot's wifi. Everything else here is a question
# about state and answers in milliseconds.
FETCH_TIMEOUT = 180.0

CATALOGUE: list[Policy] = []
CATALOGUE_ERROR: str | None = None
CATALOGUE_READ_AT: float = 0.0
# How long a peer id stays worth a name in the status line. Filled by the listing.
NAMES: dict[str, str] = {}


@dataclass
class Link:
    """The one session this Space holds, and the thread that owns its event loop.

    A dedicated loop in a background thread rather than Gradio's: the connection outlives any one
    request, `aiortc` wants a single loop for the life of a peer connection, and every send is
    marshalled back onto it.
    """

    consumer: WsConsumer | LanConsumer | None = None
    loop: asyncio.AbstractEventLoop | None = None
    rpc: Rpc | None = None
    robot: str | None = None
    error: str | None = None
    started_at: float | None = None
    # Which hop got us here. The two transports fail for entirely different reasons — one can be
    # defeated by a NAT and the other cannot — so a status line that does not say which one it is
    # describing is describing nothing.
    over_lan: bool = False

    def __post_init__(self) -> None:
        self.lock = threading.Lock()

    # ── the listing ──────────────────────────────────────────────────────────

    @staticmethod
    def robots(token: str) -> tuple[list[tuple[str, str]], str]:
        """The ducks this token can reach, and a line about what else was there.

        Safe to press mid-session: `/api/robot-status` is one GET and opens no event stream, so it
        cannot supersede the peer the session is riding on. That is why it is this endpoint rather
        than the console's `list`.
        """
        try:
            found, others = rendezvous.ducks(token)
        except RendezvousError as e:
            return [], str(e)

        ducks = [(duck.label(), duck.peer_id) for duck in found]

        if not ducks and others:
            return [], (
                f"{len(others)} robot(s) on this account and none of them a duck: "
                + ", ".join(others)
                + ". A duck registers with `kind: microduck`."
            )
        if not ducks:
            return [], (
                "no robots online for this account. A duck registers when it has been signed in "
                "(`robotctl account login`) and has a network — `duckctl account status` says "
                "which of the two is missing."
            )
        note = f"{len(ducks)} duck(s)"
        if others:
            note += f", and {len(others)} other robot(s) not listed here"
        return ducks, note

    # ── the session ──────────────────────────────────────────────────────────

    def through_rendezvous(self, token: str, peer_id: str, label: str) -> str:
        """From anywhere, over the control lane — no candidate pair, so nothing §6 can defeat.

        **This used to be a WebRTC consumer and is not any more.** It negotiated a session,
        exchanged ICE, and then sat in `connecting` forever from anywhere that needed a relay,
        which is anywhere interesting. The rendezvous relays an `rpc` key on a `peer` envelope
        verbatim, and `mediad`'s control lane answers it out of the same routing table the
        datachannel uses — so the calls are identical and the transport underneath them is two
        HTTP verbs. The cost is that there is no video on it; the LAN transport is where pixels
        live.
        """
        return self._open(
            label,
            over_lan=False,
            build=lambda rpc: WsConsumer(
                token,
                # **Pinned, not matched by name.** The listing already knows the id, and a name
                # match would be the place a mini could be handed to a duck's client.
                peer_id,
                rpc,
                label=f"microduck-policy-shop/{os.environ.get('SPACE_ID', 'local')}",
            ),
        )

    def over_the_lan(self, host: str) -> str:
        """From the robot's own network, needing no account and no relay.

        The address is all this takes: `mediad` is already serving the signalling server the
        console talks to, and on one network ICE has host candidates on both sides.
        """
        typed = (host or "").strip()
        if not typed:
            return "type the robot's address — a hostname like `olducky.local`, or its IP."

        # **A pasted URL keeps its host and loses its port.** What is in somebody's clipboard is
        # `http://olducky.local:8080/` from opening the console, and 8080 is the page's port, not
        # the signaller's — dialling it gets an HTTP server that will not upgrade, which is a
        # confusing way to learn you pasted the wrong thing. A bare `host:port` is somebody being
        # deliberate, so that one is honoured.
        pasted = typed.startswith(("http://", "https://", "ws://", "wss://"))
        address = typed.split("//")[-1].split("/")[0]
        port = DEFAULT_SIGNALLING_PORT
        if ":" in address:
            address, _, given = address.partition(":")
            if given.isdigit() and not pasted:
                port = int(given)
        return self._open(
            address, over_lan=True, build=lambda rpc: LanConsumer(address, rpc, port)
        )

    def _open(self, label: str, over_lan: bool, build: Any) -> str:
        with self.lock:
            if self.consumer is not None:
                return f"already connected to {self.robot} — disconnect first"

            self.error = None
            rpc = Rpc()
            loop = asyncio.new_event_loop()
            thread = threading.Thread(target=loop.run_forever, name="duck-consumer", daemon=True)
            thread.start()

            consumer = build(rpc)
            # The LAN consumer is `aiortc` and wants the loop; the wire is `requests` and wants
            # none. Rather than give one of them a fake shape, the two are started differently and
            # everything after this line treats them the same.
            starting = (
                asyncio.run_coroutine_threadsafe(consumer.start(), loop)
                if over_lan
                else None
            )
            try:
                if starting is not None:
                    starting.result(timeout=30)
                else:
                    consumer.start()
            except (WireError, RendezvousError) as e:
                self.error = str(e)
                loop.call_soon_threadsafe(loop.stop)
                return f"could not reach {label}: {self.error}"
            except Exception as e:  # noqa: BLE001 - reported, never raised into a UI callback
                # `TimeoutError` arrives with an empty message, which would print as a colon and
                # nothing — the type name is the whole of what it has to say.
                self.error = f"{type(e).__name__}: {e}" if str(e) else type(e).__name__
                # Unwind before stopping the loop: a half-open connect leaves an aiohttp session
                # behind, and stopping the loop under it is how "Unclosed client session" ends up
                # on somebody's terminal instead of an explanation.
                if starting is not None:
                    starting.cancel()
                    with contextlib.suppress(Exception):
                        asyncio.run_coroutine_threadsafe(consumer.stop(), loop).result(timeout=5)
                else:
                    with contextlib.suppress(Exception):
                        consumer.stop()
                loop.call_soon_threadsafe(loop.stop)
                return f"could not reach {label}: {self.error}"

            self.consumer, self.loop, self.rpc = consumer, loop, rpc
            self.robot, self.started_at, self.over_lan = label, time.monotonic(), over_lan
            logger.info("session opening: %s over %s", label, "the LAN" if over_lan else "the rendezvous")

        # Blocking here rather than leaving the page to poll: nothing below the buttons can be
        # answered until the channel is open, and a page that says "connected" while every call
        # fails is the worst of the three states.
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            if rpc.is_open():
                logger.info("control channel usable after %.1fs", time.monotonic() - (self.started_at or 0))
                # **Asked, not waited for.** `mediad` pushes `media.video` the moment it hands
                # over the channel, which races a consumer whose channel is not open yet — the
                # console page carries the same note, and a dropped push is a sideways picture
                # with nothing anywhere to say why. A question asked when we are ready cannot
                # arrive too early. Best-effort: a transport with no video refuses it, which is
                # the correct answer and not a failure to connect.
                if over_lan:
                    try:
                        rpc.notifications["media.video"] = rpc.call("media.video", timeout=10)
                    except RpcError as e:
                        logger.info("no camera geometry: %s", e.message)
                named = (self.consumer.meta or {}).get("name") if over_lan else None
                self.robot = named or label
                return f"connected to {self.robot}, control channel open"
            if getattr(consumer, "error", None):
                return f"**{consumer.error}**"
            time.sleep(0.5)
        return self.describe()

    def disconnect(self) -> str:
        logger.info("disconnecting")
        with self.lock:
            consumer, loop, rpc = self.consumer, self.loop, self.rpc
            self.consumer, self.loop, self.rpc = None, None, None
            self.robot, self.started_at, self.over_lan = None, None, False
        if rpc is not None:
            rpc.abandon("disconnected")
        if consumer is None or loop is None:
            return "not connected"
        try:
            if isinstance(consumer, LanConsumer):
                asyncio.run_coroutine_threadsafe(consumer.stop(), loop).result(timeout=10)
            else:
                consumer.stop()
        except Exception as e:  # noqa: BLE001 - a teardown that fails still ends the session
            return f"disconnected, with a complaint: {type(e).__name__}: {e}"
        finally:
            loop.call_soon_threadsafe(loop.stop)
        return "disconnected"

    def call(self, method: str, params: dict[str, Any] | None = None, timeout: float | None = None) -> Any:
        rpc = self.rpc
        if rpc is None:
            raise RpcError(method, {"message": "not connected"})
        return rpc.call(method, params, timeout)

    def describe(self) -> str:
        """Which stage the connection reached, because the stages fail differently.

        A blank "connecting…" is the worst thing this panel can say: signalling failing, ICE
        failing and a data channel that never opened look identical from the outside and have
        nothing in common.
        """
        consumer, rpc = self.consumer, self.rpc
        if consumer is None:
            return f"**not connected** — {self.error}" if self.error else "**not connected.**"

        status = consumer.status()
        session, state = status.get("session_id"), status.get("pc_state")
        waited = time.monotonic() - (self.started_at or time.monotonic())

        if not session:
            return (
                f"**no session yet** after {waited:.0f}s. The rendezvous has not paired this "
                "consumer with the robot: either it went offline, or another consumer holds it — "
                "one at a time, and the robot's own console counts."
            )
        if state != "connected":
            if not self.over_lan:
                # The control lane has no peer connection to be in a state about: it reports
                # `connected` for as long as it holds a session. So this is a session that went
                # away underneath us, which is the service ending it or the stream dropping.
                why = self.error or "no reason given"
                return (
                    f"**the session ended** after {waited:.0f}s — {why}. Connect again."
                )
            if self.over_lan:
                return (
                    f"**signalling worked and the peer connection has not** — session "
                    f"`{session[:8]}`, state `{state}` after {waited:.0f}s. On one network that is "
                    "not a NAT problem: it is a firewall between here and the robot's UDP ports, "
                    "or DTLS failing to agree a cipher — `journalctl -u mediad -b` is where the "
                    "robot's half of it is."
                )
            return (
                f"**signalling worked and the peer connection has not** — session "
                f"`{session[:8]}`, state `{state}` after {waited:.0f}s. This is the case a data "
                "centre is expected to hit: neither side offers a `relay` candidate, because the "
                "TURN credentials endpoint has no DNS (§6). *connect over the LAN* is the way "
                "through until that is fixed."
            )
        if rpc is None or not rpc.is_open():
            return (
                f"**connected, and no control channel** — session `{session[:8]}`, "
                f"{status.get('frames') or 0} frames. Media crossed and `control` did not open, "
                "which is a duck running a daemon that opens no data channel."
            )
        where = status.get("transport") or "the rendezvous"
        return (
            f"**connected to {self.robot}** over {where} — session `{session[:8]}`, "
            f"{status.get('frames') or 0} frames, control channel open."
        )

    def picture(self) -> Any:
        """The newest frame, turned the way the robot is looking.

        The camera is mounted a quarter turn off and nothing on the robot rotates the pixels —
        `media.video` carries `rotate` for exactly that, and the control channel sends it once
        when it opens. So the angle is asked for rather than compiled in, and a robot whose
        `--rotate` changes needs no edit here.
        """
        consumer, rpc = self.consumer, self.rpc
        if consumer is None:
            return None
        frame = consumer.latest_frame()
        if frame is None:
            return None
        picture = frame[1]
        video = (rpc.notifications.get("media.video") if rpc else None) or {}
        degrees = int(video.get("rotate") or 0) % 360
        if degrees:
            # `np.rot90` counts anticlockwise and the mount is measured clockwise.
            picture = np.ascontiguousarray(np.rot90(picture, k=-(degrees // 90)))
        return picture

    def shows_video(self) -> bool:
        """Whether this transport has pixels at all."""
        return self.consumer is not None and self.over_lan

    def picture_note(self) -> str:
        """Why the picture is missing, when it is missing for a reason rather than by failure.

        **An empty box reads as broken.** The control lane carries JSON-RPC relayed as HTTP and
        no media at all — which is the whole reason it works where WebRTC does not, since pixels
        are exactly what a media path is for. So on that transport there is nothing to show, and
        the panel has to say which of the two it is rather than leave a hole.
        """
        if self.consumer is None or self.over_lan:
            return ""
        return (
            "**No video on this transport, by design.** The rendezvous relays calls, not RTP: "
            "this tab works precisely because there is no media path to negotiate, and pixels "
            "are what a media path is for. `robot.policies` says what the robot did; the "
            "*on this network* tab shows it doing it."
        )

    @staticmethod
    def log() -> str:
        """What the terminal has been saying, for whoever has no terminal."""
        if not RING.lines:
            return ""
        return "```\n" + "\n".join(RING.lines) + "\n```"


LINK = Link()


# ── what the robot already has ───────────────────────────────────────────────


def robot_state() -> tuple[str, list[str]]:
    """What is loaded and what it can be asked to do, as one read.

    `robot.policies` carries the skills as well as the slots — deliberately, because a client
    cannot offer a "bow" button without knowing the robot has a bow, and the other place that
    list is published is a 50 Hz stream.
    """
    try:
        policies = LINK.call("robot.policies") or {}
        skills = LINK.call("robot.skills") or {}
    except RpcError as e:
        return f"could not read this robot: {e.message}", []

    lines = [f"**mode** `{policies.get('mode') or '?'}`"]
    if not policies.get("enabled"):
        lines.append(
            "**policies are switched off on this robot** (`[policy] enabled`), so nothing below "
            "will move it. A legitimate bench configuration, not a fault."
        )
    if policies.get("change_error"):
        lines.append(f"**last change failed:** {policies['change_error']}")

    slots = policies.get("slots") or []
    if slots:
        lines.append("")
        lines.append("| slot | running | origin |")
        lines.append("| --- | --- | --- |")
        for slot in slots:
            # `path` is what is *actually* loaded, which is the point of the call: a slot
            # whose override failed reports the file it fell back to and says why.
            path = slot.get("path") or "*empty*"
            warn = f" — ⚠️ {slot['error']}" if slot.get("error") else ""
            lines.append(
                f"| `{slot.get('slot')}` | {path}{warn} | {slot.get('origin') or '–'} |"
            )

    table = skills.get("skills") or []
    names = [s.get("name") for s in table if s.get("name")]
    built_in = skills.get("built_in") or []
    if table:
        lines.append("")
        lines.append("| skill | seconds | from config |")
        lines.append("| --- | --- | --- |")
        for skill in table:
            seconds = skill.get("duration")
            lines.append(
                f"| `{skill.get('name')}` | {seconds if seconds is not None else '–'} | "
                f"{'yes' if skill.get('overridden') else 'shipped'} |"
            )
    if built_in:
        lines.append("")
        lines.append(
            "The daemon drives these itself, so they are not editable here: "
            + ", ".join(f"`{name}`" for name in built_in)
        )
    return "\n".join(lines), names + [n for n in built_in if n not in names]


def not_accepted(result: Any) -> str | None:
    """Why a discrete intent said no, or `None`.

    **`accepted: false` is a normal answer and not a JSON-RPC error**, which `IntentResult` says
    outright: safety may refuse to run a policy on a fallen robot, and the caller needs the reason
    rather than something that reads as "the call broke". A page that only caught errors would
    report every one of those as a success and leave a motionless robot unexplained.

    An `accepted: true` carrying a reason is `IntentResult::already` — it succeeded and queued no
    work — so it is not a refusal and is worth saying anyway, which is `already_note`.
    """
    if not isinstance(result, dict) or "accepted" not in result:
        return None
    if result.get("accepted"):
        return None
    return result.get("reason") or "refused, with nothing said about why"


def already_note(result: Any) -> str:
    """An acceptance's reason, which means the robot was already in the state asked for."""
    if isinstance(result, dict) and result.get("accepted") and result.get("reason"):
        return f" — {result['reason']}"
    return ""


# ── the one click ────────────────────────────────────────────────────────────


def install_and_run(index: int, hold: float) -> str:
    """`policy.fetch`, `robot.setSkill`, `robot.policies`, `robot.do` — and stop at the first no.

    Every refusal is worth showing verbatim. `policy.fetch` is the one that checks the claims that
    matter — `obs_len`, `action_len`, `model_api`, `robot.model` — and it makes them *before* the
    download, so "this policy is 51-D and this robot is 61-D" arrives in a second rather than
    after 800 KB and a load failure.
    """
    if index >= len(CATALOGUE):
        return "that row is stale — reload the catalogue."
    policy = CATALOGUE[index]
    logger.info("click: %s (hold %s)", policy.key, hold)

    blocked = catalogue.refusal(policy)
    if blocked:
        return f"**not installed.** {blocked}"

    params: dict[str, Any] = {"repo": policy.repo}
    if policy.file:
        params["file"] = policy.file

    try:
        fetched = LINK.call("policy.fetch", params, timeout=FETCH_TIMEOUT) or {}
    except RpcError as e:
        return f"**`policy.fetch` refused it.** {e.message}"

    # The robot's own reading of the manifest wins over this Space's: it downloaded the file and
    # parsed the manifest beside it, so its answer is about the bytes that are going to run.
    if not fetched.get("duration_s") and not hold:
        return (
            f"**{policy.name} holds until it is told otherwise**, so it has no length of its own. "
            "Say how many seconds to hold it, beside the button."
        )
    late_refusal = catalogue.refusal(
        Policy(repo=policy.repo, name=policy.name, encoding=fetched.get("encoding"))
    )
    if late_refusal:
        return f"**downloaded, and not installed.** {late_refusal}"

    skill = catalogue.skill_for(fetched, hold)
    logger.info("installing %r from the fetch answer", skill["name"])
    try:
        added = LINK.call("robot.setSkill", skill)
    except RpcError as e:
        return f"**downloaded to `{fetched.get('path')}`, and `robot.setSkill` refused.** {e.message}"
    said_no = not_accepted(added)
    if said_no:
        return f"**downloaded to `{fetched.get('path')}`, and not added:** {said_no}"

    # A skill accepted is not a skill the robot has: `robot.setSkill` triggers a reload, and a
    # reload that failed says so here and nowhere else. Without this check the page would report
    # success and the robot would do nothing.
    try:
        after = LINK.call("robot.policies") or {}
    except RpcError as e:
        after = {}
        unconfirmed = f" (could not confirm the reload: {e.message})"
    else:
        unconfirmed = ""
    if after.get("change_error"):
        return f"**added, and the robot could not re-read it:** {after['change_error']}"

    name = skill["name"]
    try:
        ran = LINK.call("robot.do", {"skill": name})
    except RpcError as e:
        return (
            f"**`{name}` is installed** ({skill['duration']:g}s) **and it would not run:** "
            f"{e.message}"
        )
    said_no = not_accepted(ran)
    if said_no:
        return (
            f"**`{name}` is installed** ({skill['duration']:g}s) **and the robot would not run "
            f"it:** {said_no}"
        )
    return (
        f"**`{name}` installed and running** — {skill['duration']:g}s from "
        f"`{policy.key}`{already_note(ran)}{unconfirmed}"
    )


def run_installed(name: str) -> str:
    if not name:
        return "pick a skill first."
    try:
        ran = LINK.call("robot.do", {"skill": name})
    except RpcError as e:
        return f"**`{name}` would not run:** {e.message}"
    said_no = not_accepted(ran)
    if said_no:
        return f"**`{name}` would not run:** {said_no}"
    return f"**`{name}` running.**{already_note(ran)}"


def plain(method: str) -> str:
    """The three buttons that take no parameters: init, stop, relax."""
    try:
        result = LINK.call(method)
    except RpcError as e:
        return f"**`{method}` refused:** {e.message}"
    said_no = not_accepted(result)
    if said_no:
        return f"**`{method}` refused:** {said_no}"
    return f"**`{method}` done.**{already_note(result)}"


# ── the page ─────────────────────────────────────────────────────────────────


def row_text(policy: Policy) -> str:
    bits = [f"**{policy.name}** · {policy.headline()} · `{policy.origin}`"]
    if policy.description:
        bits.append(policy.description)
    blocked = catalogue.refusal(policy)
    warn = catalogue.caution(policy)
    if blocked:
        bits.append(f"⛔ {blocked}")
    elif warn:
        bits.append(f"⚠️ {warn}")
    if catalogue.needs_a_length(policy) and not blocked:
        bits.append("_declares no length — the seconds above are what it will be held for._")
    bits.append(f"<sub>`{policy.key}`</sub>")
    return "  \n".join(bits)


def load_catalogue(force: bool = False) -> list[Any]:
    """Render the rows, reading the Hub only when there is nothing to render.

    `demo.load` fires per page view and the read is twenty-odd HTTPS requests, so a Space
    with two visitors would spend its first four seconds fetching manifests it already has.
    *reload the catalogue* is the force, and a restart is the other one.
    """
    global CATALOGUE, CATALOGUE_ERROR, CATALOGUE_READ_AT
    if force or not CATALOGUE:
        CATALOGUE, CATALOGUE_ERROR = catalogue.read_hub()
        CATALOGUE_READ_AT = time.time()

    heading = (
        CATALOGUE_ERROR
        if CATALOGUE_ERROR
        else (
            f"{len(CATALOGUE)} policies on the Hub, official first — read "
            f"{time.strftime('%H:%M UTC', time.gmtime(CATALOGUE_READ_AT))}."
            # Gradio wants its components at build time, so the ceiling is real. Saying so is
            # the difference between a short list and a list that quietly lost its tail.
            + (
                f" Only the first {MAX_ROWS} are shown — raise `MAX_ROWS`."
                if len(CATALOGUE) > MAX_ROWS
                else ""
            )
        )
    )
    updates: list[Any] = [heading]
    for index in range(MAX_ROWS):
        if index < len(CATALOGUE):
            policy = CATALOGUE[index]
            updates.append(gr.update(visible=True))
            updates.append(gr.update(value=row_text(policy)))
            updates.append(
                gr.update(interactive=catalogue.refusal(policy) is None)
            )
        else:
            updates.append(gr.update(visible=False))
            updates.append(gr.update(value=""))
            updates.append(gr.update(interactive=False))
    return updates


def token_of(oauth: gr.OAuthToken | None) -> str:
    """A visitor's token by preference, and never the robot's — unless it is a mock.

    The rendezvous maps a token to one peer, so a consumer authenticating *as* the robot takes the
    robot off its owner's listing — the same fact §3.7 records about two robots sharing a
    credential. A visitor's OAuth token reaches their own robots and nobody else's, which is what
    makes a public Space defensible.

    **And outside a Space that token is a placeholder**, which cost an afternoon of blaming the
    rendezvous. Gradio mocks its own login when `SPACE_ID` is unset — the buttons behave, the
    profile is real, and `_get_mocked_oauth_info` sets `access_token` to the literal string
    `mock-oauth-token-for-local-dev`. Sent to a service that resolves tokens through `whoami-v2`,
    that is a `401`, correctly, and the page's own error message said "sign in again" — the one
    thing that could not help. So a mocked token is recognised and dropped rather than preferred,
    and what a local run uses is `HF_TOKEN` or whatever `hf auth login` stored, which is what
    `huggingface_hub.get_token` reads.
    """
    if oauth is not None and oauth.token and oauth.token != MOCK_TOKEN:
        # Never the token itself, here or anywhere — which *source* it came from is the half that
        # explains a refusal, and the half that is safe to write down.
        logger.info("token: the visitor's Hugging Face sign-in")
        return oauth.token

    if oauth is not None and oauth.token == MOCK_TOKEN:
        logger.info("token: Gradio's mocked sign-in, which no service accepts — using a real one")

    from huggingface_hub import get_token

    stored = (os.environ.get("HF_TOKEN") or get_token() or "").strip()
    logger.info(
        "token: %s",
        "HF_TOKEN"
        if os.environ.get("HF_TOKEN")
        else "the one `hf auth login` stored"
        if stored
        else "none — sign in, set HF_TOKEN, or run `hf auth login`",
    )
    return stored


def find_robots(oauth: gr.OAuthToken | None) -> tuple[Any, str]:
    token = token_of(oauth)
    if not token:
        return gr.update(choices=[], value=None), (
            "sign in with Hugging Face, or set an `HF_TOKEN` secret on this Space."
        )
    ducks, note = LINK.robots(token)
    NAMES.update({peer: label for label, peer in ducks})
    return gr.update(choices=ducks, value=ducks[0][1] if ducks else None), note


def open_session(peer_id: str | None, oauth: gr.OAuthToken | None) -> str:
    if not peer_id:
        return "no duck chosen — press *find my ducks* first."
    token = token_of(oauth)
    if not token:
        return "sign in with Hugging Face, or set an `HF_TOKEN` secret on this Space."
    return LINK.through_rendezvous(token, peer_id, NAMES.get(peer_id, peer_id))


with gr.Blocks(title="microduck policy shop") as demo:
    gr.Markdown(
        """
        # Put a policy on your duck

        Everything published to the Hub as `microduck-…`, read the way the robot reads it, with a
        button that downloads one onto your duck and runs it. Four calls per click:
        `policy.fetch`, `robot.setSkill`, `robot.policies`, `robot.do`.

        One consumer at a time — while this holds a session, the robot's own console cannot open
        one.
        """
    )

    with gr.Tab("from anywhere"):
        gr.Markdown(
            "Through the rendezvous, which reaches a duck behind its owner's router. **Nothing "
            "is configured on the robot for this** — it connects outward to "
            "`reachy_mini_central` holding its own account token and registers as a producer, "
            "this page signs in as you, and the service introduces the two.\n\n"
            "**And there is no WebRTC in this path at all.** The calls are relayed as HTTP: the "
            "rendezvous forwards an `rpc` key on a `peer` envelope verbatim, and `mediad`'s "
            "control lane answers it out of the same routing table the datachannel uses. No ICE, "
            "no DTLS, no relay candidate — so nothing here can be defeated by a NAT, which the "
            "WebRTC version was, every time, while `turn.fastrtc.org` has no DNS (§6).\n\n"
            "The cost is pixels: video is RTP on the media path, so there is none on this tab. "
            "`robot.policies` says what the robot did; the tab beside this one shows it."
        )
        with gr.Row():
            gr.LoginButton()
            find = gr.Button("find my ducks")
            chosen = gr.Dropdown(choices=[], label="your ducks", scale=2)
            connect = gr.Button("connect", variant="primary")

    with gr.Tab("on this network"):
        gr.Markdown(
            "Straight to the signalling server `mediad` is already running — no account, no "
            "rendezvous, no relay, and nothing between here and the robot. `duckctl ip` prints "
            "the address; `.local` works wherever mDNS does."
            + (
                # The confusion this exists to prevent: a duck is reached by the *rendezvous*
                # introducing two peers, not by anything being pointed at this page's URL — so
                # there is nothing to type here that would make a data centre reach a LAN.
                "\n\n**This tab cannot work from a Space.** This page is running in a data "
                "centre and your robot is behind your router; no address typed here is "
                "reachable from here. It is for running this same file on the robot's own "
                "network — the README's local-run section is two commands. Use *from "
                "anywhere* instead, which is what the rendezvous is for."
                if ON_A_SPACE
                else ""
            )
        )
        with gr.Row():
            host = gr.Textbox(
                value=os.environ.get("DUCK_HOST", ""),
                placeholder="olducky.local",
                label="the robot's address on this network",
                scale=2,
            )
            connect_lan = gr.Button("connect over the LAN", variant="primary")

    with gr.Row():
        disconnect = gr.Button("disconnect")

    link_state = gr.Markdown("**not connected.**")
    # What the last click did. Separate from the line above because that one repaints on a
    # timer, and a refusal worth reading would be gone a second after it arrived.
    status = gr.Markdown("")

    with gr.Row():
        read = gr.Button("read this robot")
        init = gr.Button("init (power the joints, stand)")
        stop = gr.Button("stop")
        relax = gr.Button("relax (cut torque — it will collapse)")

    # Small, and beside the buttons rather than above them: the point of a picture here is
    # seeing that the thing which was just installed did something, not watching a camera.
    view = gr.Image(label="what the duck sees", height=260, visible=False)
    # Why the box above is missing, when it is missing on purpose. Kept out of the image's own
    # label because a label cannot say a paragraph and this is the paragraph somebody needs.
    view_note = gr.Markdown("")

    with gr.Accordion("what this duck has now", open=True):
        robot_panel = gr.Markdown("Connect, then *read this robot*.")
        with gr.Row():
            installed = gr.Dropdown(choices=[], label="a skill it already has", scale=2)
            run = gr.Button("run it")

    gr.Markdown("## From the Hub")
    with gr.Row():
        hold = gr.Number(
            value=DEFAULT_HOLD,
            label="seconds to hold a policy that declares no length",
            precision=1,
            scale=2,
        )
        reload_catalogue = gr.Button("reload the catalogue")
    catalogue_note = gr.Markdown("Loading…")

    rows: list[Any] = []
    for index in range(MAX_ROWS):
        with gr.Row(visible=False) as row:
            text = gr.Markdown("")
            button = gr.Button("put it on the duck and run it", scale=0)
        button.click(
            lambda hold_s, index=index: install_and_run(index, hold_s),
            inputs=hold,
            outputs=status,
        ).then(lambda: robot_state()[0], outputs=robot_panel)
        rows.extend([row, text, button])

    with gr.Accordion("the log — every request, every frame, every refusal", open=False):
        gr.Markdown(
            "The same lines the terminal gets. `DUCK_LOG=DEBUG` adds the streaming "
            "notifications and each ICE candidate."
        )
        wire = gr.Markdown("")

    find.click(find_robots, outputs=[chosen, status])
    connect.click(open_session, inputs=chosen, outputs=status).then(
        lambda: robot_state(), outputs=[robot_panel, installed]
    )
    connect_lan.click(lambda where: LINK.over_the_lan(where), inputs=host, outputs=status).then(
        lambda: robot_state(), outputs=[robot_panel, installed]
    )
    disconnect.click(lambda: LINK.disconnect(), outputs=status)
    read.click(lambda: robot_state(), outputs=[robot_panel, installed])
    init.click(lambda: plain("robot.init"), outputs=status)
    stop.click(lambda: plain("robot.stop"), outputs=status)
    relax.click(lambda: plain("robot.relax"), outputs=status)
    run.click(run_installed, inputs=installed, outputs=status)
    reload_catalogue.click(
        lambda: load_catalogue(force=True), outputs=[catalogue_note, *rows]
    )

    demo.load(load_catalogue, outputs=[catalogue_note, *rows])
    # Once a second: it repaints a status line and a transcript, and the session it describes
    # changes state on its own — an ICE failure two minutes in should not need a click to appear.
    # Ten a second for the picture and once a second for the words: the stream is 30 fps and a
    # browser will not notice the difference, while a status line that repainted at 10 Hz would
    # be unreadable.
    gr.Timer(0.1).tick(
        lambda: (
            gr.update(value=LINK.picture(), visible=LINK.shows_video()),
            LINK.picture_note(),
        ),
        outputs=[view, view_note],
        show_progress="hidden",
    )
    gr.Timer(1.0).tick(
        lambda: (LINK.describe(), LINK.log()),
        outputs=[link_state, wire],
        show_progress="hidden",
    )


if __name__ == "__main__":
    demo.launch(server_name="0.0.0.0", server_port=int(os.environ.get("PORT", 7860)))
