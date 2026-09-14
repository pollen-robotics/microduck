"""M5's sentence, as a program: a server-side script fetches a frame and sends an intent.

    python examples/fetch_a_frame_and_send_an_intent.py robot.local

Frames come *outbound* from the robot to a socket this script opens, so the robot dials us: no
relay candidate, no NAT traversal, and it works from anywhere the robot can reach.
"""

import sys
import threading

from duck import Duck, receive

FRAME_PORT = 8099


def main(host: str) -> None:
    seen = threading.Event()

    def on_frame(jpeg: bytes) -> None:
        if not seen.is_set():
            with open("frame.jpg", "wb") as f:
                f.write(jpeg)
            print(f"got a frame, {len(jpeg)} bytes, written to frame.jpg")
            seen.set()

    threading.Thread(target=receive, args=(FRAME_PORT, on_frame), daemon=True).start()

    with Duck(host) as duck:
        print("healthy:", duck.health()["healthy"])

        # Where to send them. The robot dials this, so it has to be our address as the robot
        # sees it rather than a loopback one.
        duck.frames(url=f"ws://{local_address(host)}:{FRAME_PORT}", fps=1)
        seen.wait(timeout=10)
        duck.frames_stop()

        # An intent. One call is one intent, and the deadman stops the robot when they stop
        # arriving — so this walks for about a second and then the robot stops on its own.
        duck.move(vx=0.1)
        print("told it to walk")


def local_address(host: str) -> str:
    """Our address on the route to the robot, which is what the robot should dial back."""
    import socket

    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect((host, 80))
        return probe.getsockname()[0]
    finally:
        probe.close()


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "robot.local")
