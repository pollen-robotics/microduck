"""What a behaviour reads off a tick.

The depth rules are `tof::Frame::zone`'s, and these hold the Python to them: the protocol says a
consumer should use that interpretation rather than re-deriving the thresholds, which only means
anything if somebody checks the copy.
"""

from duck.view import STATUS_NO_TARGET, View


def depth(distances, statuses):
    return {"rows": 1, "cols": len(distances), "distance_mm": distances, "status": statuses}


def test_a_tick_with_nothing_in_it_is_readable():
    """The first tick of any loop, and every tick on a duck with no sensors. A behaviour that
    has to guard every field would be a behaviour nobody writes correctly."""
    view = View()
    assert view.frame is None
    assert view.ranges == []
    assert view.nearest is None
    assert view.position is None
    assert view.fallen is False


def test_only_the_two_valid_statuses_are_a_range():
    """5 and 9 are what ST documents as usable — valid, and valid with a large pulse. Everything
    else is the sensor saying it did not measure, and reporting it as a distance would put a
    number a behaviour acts on where there is no measurement at all."""
    view = View(depth=depth([400, 800, 1200, 300], [5, 9, 255, 4]))
    assert view.ranges == [0.4, 0.8, None, None]
    assert view.nearest == 0.4


def test_a_negative_distance_is_not_a_range_whatever_the_status_says():
    """The non-obvious half of `Frame::zone`: the sensor returns negative distances on a failed
    convergence and still flags them valid. Taken at face value they are the nearest thing in
    the room, so an avoidance behaviour would stop for something that is not there."""
    view = View(depth=depth([-120, 600], [5, 5]))
    assert view.ranges == [None, 0.6]
    assert view.nearest == 0.6


def test_an_empty_room_and_an_absent_sensor_both_read_as_nothing_seen():
    """Both are `nearest is None`, deliberately — a behaviour does the same thing either way.
    `depth` is still there for a caller that needs to tell them apart."""
    empty = View(depth=depth([0, 0], [STATUS_NO_TARGET, STATUS_NO_TARGET]))
    assert empty.nearest is None and empty.depth is not None
    absent = View()
    assert absent.nearest is None and absent.depth is None


def test_state_is_unwrapped_where_a_behaviour_would_reach_for_it():
    view = View(
        state={
            "safety": {"fallen": True},
            "odom": {"position": [1.0, 2.0, 0.1], "yaw": 0.5},
        }
    )
    assert view.fallen is True
    assert view.position == (1.0, 2.0, 0.1)
    assert view.yaw == 0.5


def test_a_robot_that_has_not_said_it_fell_has_not_fallen():
    """`False` rather than `None`, because a behaviour should not act on a fall it has no
    evidence for — and `if view.fallen` is what everybody will write."""
    assert View(state={}).fallen is False
    assert View().fallen is False
