"""Does this consumer get frames out of a real duck?

Run it from this directory, which is what makes the import work anywhere:

    HF_TOKEN=hf_... uv run --with httpx --with numpy --with aiortc --with av \
        python check_consumer.py

It uses **your** token, not the robot's: the rendezvous maps a token to one peer, so a consumer
sharing the robot's credential would take the robot off its owner's listing. And it takes the one
session the service allows, so close the console page first.

What it proves, in order: the event stream opens, the service pairs this consumer with a duck,
DTLS completes against GStreamer's `webrtcsink` (the cipher shim in `consumer.py`), frames decode,
and the robot answers `media.video` on the control channel — which is the thing their consumer
structurally cannot do, and the reason the geometry in this demo is the robot's number rather than
a repeat of the arithmetic.
"""

from __future__ import annotations

import asyncio
import os
import sys

from consumer import DuckConsumer

ROBOT = os.environ.get("DUCK_NAME", "olducky")
TOKEN = os.environ.get("HF_TOKEN", "").strip()


async def main() -> int:
    if not TOKEN:
        print("set HF_TOKEN to a token of the account the robot belongs to", file=sys.stderr)
        return 2

    consumer = DuckConsumer(token=TOKEN, robot_name=ROBOT, label="check-consumer")
    print(f"connecting to {ROBOT}…")
    await consumer.start()
    try:
        for _ in range(150):
            await asyncio.sleep(0.2)
            frame = consumer.latest_frame()
            if frame is not None:
                print(f"frame {frame[0]}: {frame[1].shape} {frame[1].dtype}")
                break
            if reported := consumer.status().get("error"):
                print(f"failed: {reported}", file=sys.stderr)
                print(f"status: {consumer.status()}", file=sys.stderr)
                return 1
        else:
            print(f"no frame in 30s. status: {consumer.status()}", file=sys.stderr)
            return 1

        # The control channel opens a moment after the video track, so this waits for it rather
        # than reporting its absence as a fault.
        for _ in range(25):
            await asyncio.sleep(0.2)
            if consumer.video_info() is not None:
                break
        print(f"media.video: {consumer.video_info()}")
        print(f"status: {consumer.status()}")
        return 0
    finally:
        await consumer.stop()


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
