import math

import pytest

from uav_control.guidance.rolling_intercept_guidance import (
    RollingReferenceFilter,
)


def make_filter():
    return RollingReferenceFilter(
        position_gain=1.0,
        max_horizontal_speed=7.0,
        max_vertical_speed=2.0,
        max_horizontal_acceleration=4.0,
        max_vertical_acceleration=1.5,
    )


def test_abrupt_prediction_jump_has_bounded_reference_acceleration():
    reference_filter = make_filter()
    reference_filter.initialize((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    state = reference_filter.update(
        (100.0, -100.0, 10.0),
        (10.0, -10.0, 5.0),
        0.05,
    )

    assert math.hypot(state.ax, state.ay) == pytest.approx(4.0)
    assert abs(state.az) == pytest.approx(1.5)
    assert math.hypot(state.vx, state.vy) == pytest.approx(0.2)
    assert abs(state.vz) == pytest.approx(0.075)
    assert math.hypot(state.x, state.y) == pytest.approx(0.005)


def test_matching_moving_prediction_advances_without_artificial_lag():
    reference_filter = make_filter()
    reference_filter.initialize((10.0, 2.0, 0.0), (5.0, 0.0, 0.0))

    state = reference_filter.update(
        (10.25, 2.0, 0.0),
        (5.0, 0.0, 0.0),
        0.05,
    )

    assert state.x == pytest.approx(10.25)
    assert state.y == pytest.approx(2.0)
    assert state.vx == pytest.approx(5.0)
    assert state.ax == pytest.approx(0.0)


def test_reset_reinitializes_from_next_prediction():
    reference_filter = make_filter()
    reference_filter.initialize((1.0, 2.0, 3.0), (0.0, 0.0, 0.0))
    reference_filter.reset()

    state = reference_filter.update(
        (20.0, 30.0, 1.0),
        (3.0, 4.0, 0.2),
        0.05,
    )

    assert (state.x, state.y, state.z) == (20.0, 30.0, 1.0)
    assert (state.vx, state.vy, state.vz) == (3.0, 4.0, 0.2)
    assert (state.ax, state.ay, state.az) == (0.0, 0.0, 0.0)
