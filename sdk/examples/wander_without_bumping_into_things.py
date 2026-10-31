"""A duck that walks around a room and does not walk into it.

    python examples/wander_without_bumping_into_things.py [duck-name]

The whole behaviour is the loop below. Depth and state arrive on two streams at two rates, and
`watch` is what makes them one object per tick. Depth over the rendezvous waits on the robot
posting lane messages faster than `tofd` sends frames; see the README.
"""

import sys

from duck import Duck

# Metres. The ToF sees about 2 m.
TOO_CLOSE = 0.4

# m/s and rad/s.
WALK = 0.12
TURN = 0.8


def main(name: str | None) -> None:
    with Duck(name) as duck:
        duck.enable(True)

        for view in duck.watch(fps=2, depth=True):
            if view.fallen:
                print("down, stopping and letting somebody pick it up")
                duck.stop()
                break

            near = view.nearest
            if near is None:
                # Nothing in front, or no sensor: turn rather than walk blind.
                duck.move(vyaw=TURN)
            elif near < TOO_CLOSE:
                print(f"{near:.2f} m ahead, turning")
                duck.move(vyaw=TURN)
            else:
                duck.move(vx=WALK)

            if view.elapsed > 60:
                print("that is enough for now")
                duck.stop()
                break


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else None)
