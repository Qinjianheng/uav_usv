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


def test_preferred_clearance_and_terminal_velocity_are_sea_safe():
    planner = make_planner(
        preferred_clearance=0.1,
        maximum_vertical_speed=4.0,
        maximum_vertical_acceleration=4.0,
    )

    def descending_surface_target(_horizon):
        return (
            (2.0, 0.0, 0.15),
            (0.0, 0.0, 0.1),
            (0.0, 0.0, 0.0),
        )

    plan = planner.plan(
        (0.0, 0.0, -1.0),
        (2.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        descending_surface_target,
    )

    assert plan is not None
    assert plan.target_position[2] == pytest.approx(-0.1)
    terminal = plan.sample(plan.duration)
    assert terminal.velocity[2] <= 0.0


def test_minco_planner_tracks_curved_target_prediction_with_three_pieces():
    planner = make_planner(
        minco_piece_count=3,
        minco_target_curve_weight=0.7,
        maximum_horizontal_acceleration=8.0,
        maximum_vertical_acceleration=4.0,
    )

    def turning_target(horizon):
        angular_rate = 0.35
        speed = 2.0
        angle = angular_rate * horizon
        return (
            (
                3.0 + speed * math.sin(angle) / angular_rate,
                speed * (1.0 - math.cos(angle)) / angular_rate,
                -0.1,
            ),
            (speed * math.cos(angle), speed * math.sin(angle), 0.0),
            (
                -speed * angular_rate * math.sin(angle),
                speed * angular_rate * math.cos(angle),
                0.0,
            ),
        )

    plan = planner.plan(
        (0.0, 0.0, -1.0),
        (4.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        turning_target,
    )

    assert plan is not None
    assert plan.planner_type == 'MINCO_T3'
    assert plan.minco_trajectory.piece_count == 3
    terminal = plan.sample(plan.duration)
    assert terminal.position == pytest.approx(plan.target_position, abs=1e-7)
    assert terminal.velocity[1] > 0.0


def test_minco_relaxes_target_curve_weight_to_remain_dynamically_feasible():
    planner = make_planner(
        minco_piece_count=3,
        minco_target_curve_weight=0.7,
        maximum_horizontal_speed=6.5,
        maximum_horizontal_acceleration=3.5,
        desired_closing_speed=1.5,
        minimum_closing_speed=0.3,
        closing_speed_step=0.3,
    )

    def fast_turning_target(horizon):
        angular_rate = 0.3
        speed = 5.0
        angle = angular_rate * horizon
        return (
            (
                2.0 + speed * math.sin(angle) / angular_rate,
                speed * (1.0 - math.cos(angle)) / angular_rate,
                -0.1,
            ),
            (speed * math.cos(angle), speed * math.sin(angle), 0.0),
            (
                -speed * angular_rate * math.sin(angle),
                speed * angular_rate * math.cos(angle),
                0.0,
            ),
        )

    plan = planner.plan(
        (0.0, 0.0, -1.0),
        (6.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        fast_turning_target,
    )

    assert plan is not None
    assert plan.planner_type == 'MINCO_T3'
    assert 0.0 < plan.target_curve_weight < 0.7
    assert plan.maximum_horizontal_acceleration <= 3.5
