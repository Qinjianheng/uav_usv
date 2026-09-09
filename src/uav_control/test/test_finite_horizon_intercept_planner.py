import math

import pytest

from uav_control.guidance.finite_horizon_intercept_planner import (
    FiniteHorizonInterceptPlanner,
    QuinticAxis,
)


def make_planner(**overrides):
    parameters = {
        'minimum_duration': 1.0,
        'maximum_duration': 2.0,
        'duration_step': 0.1,
        'sample_step': 0.05,
        'maximum_horizontal_speed': 7.4,
        'maximum_vertical_speed': 2.0,
        'maximum_horizontal_acceleration': 4.5,
        'maximum_vertical_acceleration': 2.0,
        'desired_closing_speed': 3.0,
        'minimum_closing_speed': 0.5,
        'closing_speed_step': 0.5,
        'capture_radius': 0.25,
        'sea_surface_z': 0.0,
        'contact_clearance': 0.05,
    }
    parameters.update(overrides)
    return FiniteHorizonInterceptPlanner(**parameters)


def test_quintic_axis_satisfies_complete_boundary_states():
    axis = QuinticAxis.from_boundary(
        1.0,
        -0.5,
        0.3,
        7.0,
        2.0,
        -0.2,
        1.7,
    )

    assert axis.sample(0.0)[:3] == pytest.approx((1.0, -0.5, 0.3))
    assert axis.sample(1.7)[:3] == pytest.approx((7.0, 2.0, -0.2))


def test_planner_finds_feasible_moving_target_trajectory():
    planner = make_planner()

    def target_state(horizon):
        return (
            (4.0 + 2.0 * horizon, 0.0, -0.1),
            (2.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        )

    plan = planner.plan(
        (0.0, 0.0, -1.0),
        (4.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        target_state,
    )

    assert plan is not None
    assert 1.0 <= plan.duration <= 2.0
    assert plan.maximum_horizontal_speed <= 7.4
    assert plan.maximum_horizontal_acceleration <= 4.5
    assert plan.maximum_vertical_speed <= 2.0
    assert plan.maximum_vertical_acceleration <= 2.0
    terminal = plan.sample(plan.duration)
    assert terminal.position == pytest.approx(plan.target_position)
    assert terminal.position[2] < 0.0


def test_planner_relaxes_terminal_closing_speed_when_needed():
    planner = make_planner()

    def target_state(horizon):
        return (
            (1.0 + 5.0 * horizon, 0.0, 0.0),
            (5.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        )

    plan = planner.plan(
        (0.0, 0.0, -1.0),
        (7.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        target_state,
    )

    assert plan is not None
    assert 0.5 <= plan.closing_speed < 3.0


def test_planner_rejects_target_outside_two_second_reachable_set():
    planner = make_planner()

    def distant_target(_horizon):
        return (
            (50.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        )

    plan = planner.plan(
        (0.0, 0.0, -1.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        distant_target,
    )

    assert plan is None


def test_contact_point_stays_above_sea_and_inside_capture_radius():
    planner = make_planner()

    def surface_target(_horizon):
        return (
            (2.0, 0.0, 0.15),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        )

    plan = planner.plan(
        (0.0, 0.0, -1.0),
        (2.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        surface_target,
    )

    assert plan is not None
    assert plan.target_position[2] == pytest.approx(-0.05)
    assert abs(plan.target_position[2] - 0.15) <= 0.25
    samples = [
        plan.sample(index * plan.duration / 40.0)
        for index in range(41)
    ]
    assert all(sample.position[2] < 0.0 for sample in samples)
    assert all(
        math.hypot(sample.velocity[0], sample.velocity[1]) <= 7.4
        for sample in samples
    )
