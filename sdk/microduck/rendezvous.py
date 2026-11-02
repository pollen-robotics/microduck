"""Which robots an account can reach, and whether each one is busy.

`GET /api/robot-status` with `Authorization: Bearer <hf token>`. Read off
`pollen-robotics/reachy_mini_central`'s `app.py` rather than guessed, because a `401` from it was
first mistaken here for something structural: the endpoint is `Depends(_resolve_hf_token)` and then
`validate_hf_token`, which is one `whoami-v2` call and **no scope check, no token-type check and no
requirement that the caller hold an event stream**. Its own docstring says what it is for — "render
a passive status indicator without consuming a session slot" — filtered to `p.username ==
username`, so an account sees its own robots and nobody else's.

Which makes it the right call for a listing, and better than the console's route for one:
`mediad/webclient/index.html` lists by opening `GET /events` and asking `list`, because a browser
that is about to start a session needs the stream anyway. Peers are keyed by token, so a second
`/events` on the same one supersedes the first (`remote-access-design.md` §3.7) — a page that
listed that way could not refresh its list without dropping the session it was holding. This one
opens nothing.

**A `429` here is not theirs.** `robot_status` is `validate_hf_token` and then a loop over the
producers — it never calls `check_rate_limit`, whose bucket is 1200 requests a minute keyed on a
hash of the token, and the only status it raises is `401`. So a 429 on this route was written by
something in front of the application, and `_who_answered` exists to say which something.

**A `401` here means the token, and nothing else.** Worth stating plainly because the first thing
that produced one was not a robot problem at all: run outside a Space, Gradio mocks its login and
hands the app the literal string `mock-oauth-token-for-local-dev`, which `whoami-v2` refuses
exactly as it should. `app.py`'s `token_of` is where that is dealt with.
"""

from __future__ import annotations

import logging
import os
from typing import Any

import requests

# The Space the mini's fleet registers with, and now ours: `mediad::relay::DEFAULT_RENDEZVOUS`.
#
# Written here rather than imported from `reachy_mini.media.central_consumer`, which is where it
# was read from originally. One constant is not worth a dependency on `aiortc`, `av` and a WebRTC
# stack in a Space that speaks only HTTP — and it made this module unimportable in the vision Space
# for no reason. `REACHY_CENTRAL_URL` overrides it, the same name their own `from_env` reads.
DEFAULT_CENTRAL_URL = os.environ.get(
    "REACHY_CENTRAL_URL", "https://pollen-robotics-reachy-mini-central.hf.space"
).rstrip("/")

# `meta.kind`, which is on the wire so one client can list two families of robot without opening a
# session to ask what it found. §5.1: their clients do not read it, this one does — an account with
# a duck and a mini on it would otherwise offer either as a duck.
DUCK = "microduck"

TIMEOUT = 20

# **What we call ourselves, and it is not cosmetic.** `requests` signs every call
# `python-requests/2.x`, which Hugging Face's edge treats as a bot: from a Space's container the
# very first `GET /api/robot-status` came back `429` with an HTML page and `server=awselb/2.0`,
# so the rendezvous never saw it. The reference client — `reachy_mini.media.central_consumer`,
# which `rf-detr-realtime-webcam` runs server-side from its own Space against this same host and
# route — sends no `User-Agent` of its own either, but it is built on `aiohttp` and inherits that
# library's signature instead. The difference between the two calls was the client, and nothing
# else: same URL, same `Authorization`, same everything.
#
# So this names the client honestly rather than imitating a browser. A caller that says who it is
# and carries a link is what a rate limiter is meant to let through, and it is the half of this
# that stays true if the rule ever changes.
USER_AGENT = "microduck-sdk/0.1 (+https://github.com/pollen-robotics/microduck/tree/main/sdk)"

# Headers that say *who* answered. Asked only when the answer is one the application could not
# have written: an edge or a proxy names itself and carries an id worth quoting, where FastAPI
# would have sent `application/json` and a `detail`.
TELLTALE = (
    "retry-after",
    "server",
    "via",
    "x-request-id",
    "x-amzn-requestid",
    "cf-ray",
    "x-ratelimit-limit",
    "x-ratelimit-remaining",
    "x-ratelimit-reset",
    "content-type",
)


logger = logging.getLogger(__name__)


class RendezvousError(Exception):
    """Something the person who pressed the button can act on."""


class Robot:
    """One producer as the rendezvous describes it."""

    def __init__(self, entry: dict[str, Any]):
        meta = entry.get("meta") or {}
        self.peer_id: str = entry.get("peerId") or entry.get("id") or ""
        self.name: str = meta.get("name") or entry.get("robotName") or "a robot with no name"
        self.kind: str | None = meta.get("kind")
        self.release: str = meta.get("release") or "release unknown"
        # **A duck in MuJoCo, not one on a desk.** `mediad` sets this from `configd --simulated`;
        # `docs/design/simulation.md` §8 is why it is one declared fact rather than something each
        # client works out. Accepted as a bool or as the string a `GstStructure` turns it into,
        # because those are the two ways it reaches a listing and neither is this Space's choice.
        flag = meta.get("simulated")
        self.simulated: bool = flag is True or str(flag).lower() == "true"
        self.busy: bool = bool(entry.get("busy"))
        # Who has it, when it is busy. The rendezvous reports the *consumer's* name here, which
        # is why this Space sends a `consumer_label` worth reading.
        self.active_app: str | None = entry.get("activeApp")
        self.age: float | None = entry.get("last_seen_age_seconds")

    def label(self) -> str:
        """What the dropdown shows: enough to choose by, and the reason not to.

        `busy` matters more than it looks. One consumer at a time is the rendezvous's rule, so a
        robot somebody's console is already watching will refuse a session — and a dropdown that
        did not say so would make that look like a fault here.

        `simulated` matters for the opposite reason: nothing here goes wrong, and somebody drives a
        duck in MuJoCo believing it is the one on the shelf.
        """
        bits = [self.name]
        # Second, before anything about availability: which robot this is matters more than whether
        # it happens to be busy, and somebody scanning a dropdown of real ducks should not have to
        # read to the end of the line to find the one that is not real.
        if self.simulated:
            bits.append("simulated")
        bits.append(self.release)
        if self.busy:
            bits.append(f"busy with {self.active_app or 'something'}")
        if self.age is not None and self.age > 60:
            bits.append(f"last heard from {self.age / 60:.0f} min ago")
        return " — ".join(bits)


def _who_answered(answer: requests.Response) -> str:
    """Which hop produced this status, in the terms the answer itself offers.

    **A status the application cannot produce came from something in front of it**, and the only
    evidence of which something is what came back: a `server` that names itself, a request id to
    quote at whoever runs it, a `retry-after` that says whether this is a burst or a wall, and a
    body that is HTML where FastAPI would have written `{"detail": ...}`.

    The token is never part of this. What is quoted is the answer, which the caller already has.
    """
    bits = [f"{name}={answer.headers[name]}" for name in TELLTALE if name in answer.headers]
    body = " ".join((answer.text or "").split())[:200]
    if body:
        bits.append(f"body={body!r}")
    return ", ".join(bits) or "nothing — no telltale headers and an empty body"


def ducks(token: str, base: str = DEFAULT_CENTRAL_URL) -> tuple[list[Robot], list[str]]:
    """This account's ducks, and the names of whatever else was listed.

    Blocking: every caller is a Gradio callback with nothing else to do.
    """
    if not token:
        raise RendezvousError("no token to ask with")

    logger.info("GET %s/api/robot-status", base)
    try:
        answer = requests.get(
            f"{base}/api/robot-status",
            headers={"Authorization": f"Bearer {token}", "User-Agent": USER_AGENT},
            timeout=TIMEOUT,
        )
    except requests.RequestException as e:
        raise RendezvousError(f"the rendezvous could not be reached: {e}") from None

    logger.info("GET %s/api/robot-status -> HTTP %s", base, answer.status_code)
    # Logged for every refusal rather than only for the one that raises here, because which hop
    # answered is the question in all of them and the branches below each throw a different half
    # of it away.
    if answer.status_code != 200:
        logger.info("  answered by: %s", _who_answered(answer))
    if answer.status_code == 401:
        raise RendezvousError(
            "the rendezvous refused this token — `whoami-v2` did not recognise it. On a Space, "
            "sign in again. Running locally, Gradio's login button is a mock that hands the app "
            "a placeholder string, so the token comes from `HF_TOKEN` or `hf auth login` "
            "instead — the log line above says which one was used."
        )
    if answer.status_code == 429:
        raise RendezvousError(
            "something answered 429 before the rendezvous did — `/api/robot-status` raises 401 "
            "and nothing else, and its rate limiter is not on that route. So this is an edge in "
            f"front of the Space, and here is what it said: {_who_answered(answer)}"
        )
    if answer.status_code != 200:
        raise RendezvousError(
            f"the rendezvous answered HTTP {answer.status_code}: {answer.text[:200]}"
        )
    try:
        listed = answer.json().get("robots") or []
    except ValueError:
        raise RendezvousError("the rendezvous answered something that is not JSON") from None

    logger.info("%d producer(s) listed", len(listed))
    ours, theirs = [], []
    for entry in listed:
        robot = Robot(entry)
        logger.info(
            "  %s kind=%s busy=%s peer=%s", robot.name, robot.kind, robot.busy, robot.peer_id[:8]
        )
        if not robot.peer_id:
            continue
        if robot.kind == DUCK:
            ours.append(robot)
        else:
            theirs.append(f"{robot.name} ({robot.kind or 'no kind declared'})")
    return ours, theirs


if __name__ == "__main__":
    import os
    import sys

    from huggingface_hub import get_token

    logging.basicConfig(level=logging.INFO, format="%(name)s: %(message)s")
    credential = os.environ.get("HF_TOKEN") or get_token()
    if not credential:
        sys.exit("no token: run `hf auth login`, or set HF_TOKEN")
    try:
        found, others = ducks(credential)
    except RendezvousError as e:
        sys.exit(str(e))
    for duck in found:
        print(f"  {duck.peer_id}  {duck.label()}")
    print(f"\n{len(found)} duck(s)" + (f"; also listed: {', '.join(others)}" if others else ""))
