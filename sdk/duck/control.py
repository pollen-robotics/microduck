"""JSON-RPC 2.0 over whatever carries it: ids out, answers and notifications back.

`duck-ipc-proto`'s own wire, one object per line — the same lines `robotctl` sends over a unix
socket, the console page sends over a datachannel, and `mediad`'s control lane relays inside a
`peer` envelope. Ids are handed out here and answers matched to them, which is what lets a Gradio
callback block on one.

**Transport-agnostic, and that is now load-bearing rather than tidy.** Three of them have been
tried: a datachannel over the rendezvous, a datachannel on the LAN, and JSON-RPC relayed as HTTP
with no WebRTC at all. Every one of them hands lines to this object and takes lines back, so none
of the page above it changed when the first was replaced by the third.

It once also held a shim over `ReachyCentralConsumer`, whose `pc.on("datachannel")` handler drops
any label but `"data"` while `mediad` opens `"control"` — `remote-access-design.md` §5.1 records
it, and it is still true of their client. It is gone because the transport that needed it is gone:
a rendezvous session that has to negotiate ICE cannot connect from a data centre while §6 stands,
so `wire.py` replaced it, and with it the only dependency this file had on another package's
private method.
"""

from __future__ import annotations

import asyncio
import itertools
import json
import logging
import threading
from concurrent.futures import Future
from concurrent.futures import TimeoutError as FutureTimeout
from typing import Any, Callable

logger = logging.getLogger(__name__)

# What `mediad` calls the channel it opens on every consumer's peer connection.
CONTROL_LABEL = "control"


class RpcError(Exception):
    """A refusal from the robot, carrying the code it refused with.

    Distinct from a transport failure on purpose: `policy.fetch` answering "obs_len 51, this
    robot is 61" is the robot working correctly and is the most useful thing this Space can
    show, while "no answer in 30s" means something else entirely.
    """

    def __init__(self, method: str, error: dict[str, Any]):
        self.method = method
        self.code = error.get("code")
        self.message = error.get("message") or "refused, with nothing said about why"
        super().__init__(f"{method}: {self.message}")


class Rpc:
    """Requests out, answers and notifications in, over one data channel.

    Not a session and not a connection — those are the consumer's. This owns the id space and
    the pending table, and it survives a channel closing: the futures are failed rather than
    left for a caller to time out on one at a time.
    """

    def __init__(self, timeout: float = 30.0):
        self.timeout = timeout
        self._ids = itertools.count(1)
        # The method rides along with the future so a refusal can name what was refused: a
        # reply carries an id and nothing else, and "call: no such method" is a worse sentence
        # than "robot.setSkill: no such method" for the sake of one tuple.
        self._pending: dict[int, tuple[str, Future]] = {}
        self._lock = threading.Lock()
        self._send: Callable[[dict[str, Any]], bool] | None = None
        # The last one of each notification. A duck streams `robot.state` at whatever rate it was
        # asked for, so keeping them all is a leak and keeping the last is what a panel shows.
        self.notifications: dict[str, Any] = {}

    def bound_to(self, send: Callable[[dict[str, Any]], bool]) -> None:
        self._send = send

    def is_open(self) -> bool:
        return self._send is not None

    def call(
        self, method: str, params: dict[str, Any] | None = None, timeout: float | None = None
    ) -> Any:
        """Send one request and wait for its answer.

        Blocking, because every caller here is a Gradio callback with nothing else to do. The
        timeout is not a budget on the robot's behalf: a promise nobody settles is a leak, and a
        method that never answers is worth seeing rather than a panel that quietly stopped.
        `policy.fetch` is the one call that wants its own — it is a download over the robot's
        wifi, not a question about state.
        """
        budget = self.timeout if timeout is None else timeout
        send = self._send
        if send is None:
            raise RpcError(method, {"message": "no control channel — connect first"})

        call_id = next(self._ids)
        future: Future = Future()
        with self._lock:
            self._pending[call_id] = (method, future)
        logger.info("→ %s %s", method, json.dumps(params or {}))

        if not send({"jsonrpc": "2.0", "id": call_id, "method": method, "params": params or {}}):
            with self._lock:
                self._pending.pop(call_id, None)
            raise RpcError(method, {"message": "the control channel would not take the request"})

        try:
            return future.result(timeout=budget)
        except FutureTimeout:
            with self._lock:
                self._pending.pop(call_id, None)
            raise RpcError(
                method, {"message": f"no answer in {budget:.0f}s"}
            ) from None

    def on_message(self, raw: Any) -> None:
        """One line off the channel. Runs on the consumer's event loop, so it does not block."""
        if isinstance(raw, bytes):
            raw = raw.decode("utf-8", "replace")
        try:
            message = json.loads(raw)
        except (TypeError, ValueError):
            logger.warning("← unparseable: %s", str(raw)[:160])
            return

        call_id = message.get("id")
        if call_id is None:
            # A notification. `robot.state` streams, `media.video` arrives once when the channel
            # opens, and `media.detections` a couple of times a second — none of them answer
            # anything anybody asked for.
            method = message.get("method")
            if method:
                self.notifications[method] = message.get("params")
                # DEBUG, not INFO: `media.detections` arrives a couple of times a second and
                # `robot.state` at whatever rate it was asked for, and a log that scrolls is a
                # log nobody reads the top of.
                logger.debug("← %s %s", method, json.dumps(message.get("params"))[:200])
            return

        with self._lock:
            waiting = self._pending.pop(call_id, None)
        if waiting is None:
            # A duck does not correlate replies (`remote-webrtc.md` §5), so this is ordinary:
            # a fire-and-forget intent's answer, or one that arrived after its timeout.
            return
        method, future = waiting
        if "error" in message:
            error = message["error"] or {}
            logger.warning("← %s refused: %s", method, error.get("message"))
            future.set_exception(RpcError(method, error))
        else:
            result = message.get("result")
            logger.info("← %s %s", method, json.dumps(result)[:240])
            future.set_result(result)

    def abandon(self, why: str) -> None:
        """Fail everything in flight. A closed channel answers nothing, ever."""
        logger.info("control channel gone: %s", why)
        with self._lock:
            pending, self._pending = self._pending, {}
            self._send = None
        for method, future in pending.values():
            if not future.done():
                future.set_exception(RpcError(method, {"message": why}))
