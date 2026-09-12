"""A duck that walks around a room and does not walk into it.

    python examples/wander_without_bumping_into_things.py robot.local

The whole behaviour is the loop below. Everything it reads — depth, state, whether the robot has
fallen — arrives on a different stream at a different rate, and `watch` is what makes that one
object per tick instead of three subscriptions to merge by hand.
"""

import sys

from duck import Duck

#: How close something has to be before turning away from it, metres. The ToF sees about 2 m, so
#: this is "in the way" rather than "visible".
TOO_CLOSE = 0.4

#: Metres per second, and radians per second. A duck's top speed is not the interesting part of a
#: wander; being able to stop is.
WALK = 0.12
TURN = 0.8


def main(host: str) -> None:
    with Duck(host) as duck:
        duck.enable(True)

        for view in duck.watch(fps=4, camera=False):
            if view.fallen:
                print("down — stopping and letting somebody pick it up")
                duck.stop()
                break

            near = view.nearest
            if near is None:
                # No usable range: either nothing in front or no sensor. Walking on a reading
                # that does not exist is how a duck finds a wall with its face, so it turns.
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
    main(sys.argv[1] if len(sys.argv) > 1 else "robot.local")
