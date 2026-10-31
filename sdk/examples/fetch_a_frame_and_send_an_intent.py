"""M5's sentence as a program: a server-side script fetches a frame and sends an intent.

    pip install -e "sdk[frames]" pillow
    python examples/fetch_a_frame_and_send_an_intent.py wss://this-server.example/frames

Control goes over the rendezvous, so the duck can be anywhere its owner's account reaches. The
frame comes over `media.stream`, which means the robot dials the url given here: run this where
that url reaches port 8099 on this machine, a server with an address of its own. The robot
sends its owner's Hugging Face token to that url, so it has to be a server you trust with it.
"""

import sys
import threading
import time

from PIL import Image

from duck import Duck, receive

PORT = 8099


def main(url: str, name: str | None) -> None:
    seen = threading.Event()

    def on_picture(picture) -> None:
        if not seen.is_set():
            Image.fromarray(picture).save("frame.jpg")
            print(f"got a {picture.shape[1]}x{picture.shape[0]} frame, written to frame.jpg")
            seen.set()

    threading.Thread(target=receive, args=(PORT, on_picture), daemon=True).start()

    with Duck(name) as duck:
        print(f"{duck.robot.name}: healthy {duck.health()['healthy']}")

        duck.stream_frames(url, fps=1, encoding="jpeg")
        seen.wait(timeout=15)
        duck.stop_frames()

        duck.enable(True)
        duck.move(vx=0.1)
        time.sleep(2)
        duck.stop()
        print("walked for two seconds")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
