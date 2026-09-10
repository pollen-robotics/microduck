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
        """
        bits = [self.name, self.release]
        if self.busy:
            bits.append(f"busy with {self.active_app or 'something'}")
        if self.age is not None and self.age > 60:
            bits.append(f"last heard from {self.age / 60:.0f} min ago")
        return " — ".join(bits)


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
            headers={"Authorization": f"Bearer {token}"},
            timeout=TIMEOUT,
        )
    except requests.RequestException as e:
        raise RendezvousError(f"the rendezvous could not be reached: {e}") from None

    logger.info("GET %s/api/robot-status -> HTTP %s", base, answer.status_code)
    if answer.status_code == 401:
        raise RendezvousError(
            "the rendezvous refused this token — `whoami-v2` did not recognise it. On a Space, "
            "sign in again. Running locally, Gradio's login button is a mock that hands the app "
            "a placeholder string, so the token comes from `HF_TOKEN` or `hf auth login` "
            "instead — the log line above says which one was used."
        )
    if answer.status_code == 429:
        raise RendezvousError("the rendezvous is rate-limiting this token; wait a minute.")
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
