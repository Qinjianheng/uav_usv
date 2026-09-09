import math

import pytest

from uav_control.guidance.intercept_solver import (
    earliest_reachable_intercept,
)


def test_finds_analytic_constant_velocity_intercept_time():
    def predict_target(horizon):
        return 20.0 + 5.0 * horizon, 0.0, 0.0, 5.0, 0.0, 0.0

    state, t_go = earliest_reachable_intercept(
        predict_target,
        (0.0, 0.0, 0.0),
        horizontal_speed=8.0,
        vertical_speed=2.0,
        max_prediction_time=8.0,
        search_step=0.05,
    )

    assert t_go == pytest.approx(20.0 / 3.0, abs=1e-5)
    assert state[:3] == pytest.approx(
        (20.0 + 5.0 * t_go, 0.0, 0.0),
    )


def test_respects_simultaneous_vertical_reachability():
    def stationary_target(_horizon):
        return 3.0, 4.0, 6.0, 0.0, 0.0, 0.0

    _, t_go = earliest_reachable_intercept(
        stationary_target,
        (0.0, 0.0, 0.0),
        horizontal_speed=10.0,
        vertical_speed=2.0,
        max_prediction_time=5.0,
        search_step=0.05,
    )

    assert t_go == pytest.approx(3.0, abs=1e-5)


def test_curved_prediction_returns_a_reachable_intercept_point():
    radius = 10.0
    speed = 5.0
    turn_rate = speed / radius

    def circular_target(horizon):
        heading = turn_rate * horizon
        return (
            radius * math.sin(heading),
            radius * (1.0 - math.cos(heading)),
            0.0,
            speed * math.cos(heading),
            speed * math.sin(heading),
            0.0,
        )

    state, t_go = earliest_reachable_intercept(
        circular_target,
        (-15.0, 0.0, 0.0),
        horizontal_speed=8.0,
        vertical_speed=2.0,
        max_prediction_time=8.0,
        search_step=0.05,
    )
    required_time = math.hypot(
        state[0] + 15.0,
        state[1],
    ) / 8.0

    assert t_go < 8.0
    assert required_time == pytest.approx(t_go, abs=1e-5)


def test_uses_prediction_limit_when_no_point_is_reachable():
    def faster_target(horizon):
        return 10.0 + 20.0 * horizon, 0.0, 0.0

    state, t_go = earliest_reachable_intercept(
        faster_target,
        (0.0, 0.0, 0.0),
        horizontal_speed=8.0,
        vertical_speed=2.0,
        max_prediction_time=3.0,
        search_step=0.05,
    )

    assert t_go == 3.0
    assert state[:3] == (70.0, 0.0, 0.0)
