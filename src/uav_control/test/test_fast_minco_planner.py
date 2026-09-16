"""Behavior tests for the bounded realtime MINCO search."""

import pytest

from uav_control.guidance.fast_minco_planner import FastMincoPlanner
from uav_control.guidance.fast_minco_planner import FastPlanningFailure


def make_planner(**overrides):
    """Create the realtime planner with the project dynamic envelope."""
    parameters = {
        'minimum_duration': 1.0,
        'maximum_duration': 3.0,
        'duration_margin': 0.35,
        'sample_step': 0.05,
        'maximum_horizontal_speed': 7.0,
        'maximum_vertical_speed': 4.0,
        'maximum_horizontal_acceleration': 3.5,
        'maximum_vertical_acceleration': 3.0,
        'preferred_closing_speed': 1.5,
        'conservative_closing_speed': 0.3,
        'capture_radius': 0.25,
        'sea_surface_z': 0.0,
        'contact_clearance': 0.05,
        'preferred_clearance': 0.1,
        'piece_count': 3,
        'target_curve_weight': 0.7,
        'deadline_seconds': 0.08,
    }
    parameters.update(overrides)
    return FastMincoPlanner(**parameters)


def moving_target(horizon):
    """Return a simple target forecast without route-specific knowledge."""
    return (
        (3.0 + 2.0 * horizon, 0.0, -0.1),
        (2.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
    )


def test_realtime_search_checks_at_most_six_candidates():
    planner = make_planner(maximum_horizontal_acceleration=0.3)

    outcome = planner.plan(
        initial_position=(0.0, 0.0, -1.0),
        initial_velocity=(0.0, 0.0, 0.0),
        initial_acceleration=(0.0, 0.0, 0.0),
        target_state_at_time=moving_target,
    )

    assert outcome.diagnostics.candidates_checked <= 6


def test_fast_feasible_plan_does_not_wait_for_geometric_optimization():
    planner = make_planner(maximum_horizontal_acceleration=8.0)

    outcome = planner.plan(
        initial_position=(0.0, 0.0, -1.0),
        initial_velocity=(4.0, 0.0, 0.0),
        initial_acceleration=(0.0, 0.0, 0.0),
        target_state_at_time=moving_target,
    )

    assert outcome.plan is not None
    assert outcome.plan.planner_type == 'MINCO_T3_FAST'
    assert not outcome.diagnostics.optimization_attempted


def test_deadline_discards_a_result_even_if_a_candidate_would_be_feasible():
    clock_values = iter((0.0, 0.09, 0.10, 0.11, 0.12, 0.13))
    planner = make_planner(clock=lambda: next(clock_values, 0.20))

    outcome = planner.plan(
        initial_position=(0.0, 0.0, -1.0),
        initial_velocity=(4.0, 0.0, 0.0),
        initial_acceleration=(0.0, 0.0, 0.0),
        target_state_at_time=moving_target,
    )

    assert outcome.plan is None
    assert outcome.failure == FastPlanningFailure.DEADLINE_EXCEEDED


def test_contact_below_reachable_capture_sphere_is_capture_geometry_failure():
    planner = make_planner()

    outcome = planner.plan(
        initial_position=(0.0, 0.0, -1.0),
        initial_velocity=(2.0, 0.0, 0.0),
        initial_acceleration=(0.0, 0.0, 0.0),
        target_state_at_time=lambda _horizon: (
            (2.0, 0.0, 1.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        ),
    )

    assert outcome.plan is None
    assert outcome.failure == FastPlanningFailure.CAPTURE_GEOMETRY


def test_preferred_contact_duration_is_tried_before_rolling_horizon_defaults():
    """Catch a feasible locked contact time being replaced every planning tick."""
    planner = make_planner(maximum_horizontal_acceleration=8.0)

    outcome = planner.plan(
        initial_position=(0.0, 0.0, -1.0),
        initial_velocity=(4.0, 0.0, 0.0),
        initial_acceleration=(0.0, 0.0, 0.0),
        target_state_at_time=moving_target,
        preferred_duration=2.2,
    )

    assert outcome.plan is not None
    assert outcome.plan.duration == pytest.approx(2.2)


def test_terminal_minimum_duration_allows_locked_subsecond_plan():
    """Catch terminal planning being blocked by the normal one-second floor."""
    planner = make_planner(
        maximum_horizontal_acceleration=20.0,
        maximum_vertical_acceleration=20.0,
    )

    outcome = planner.plan(
        initial_position=(0.0, 0.0, -0.1),
        initial_velocity=(2.0, 0.0, 0.0),
        initial_acceleration=(0.0, 0.0, 0.0),
        target_state_at_time=lambda horizon: (
            (0.2 + 2.0 * horizon, 0.0, -0.1),
            (2.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        ),
        preferred_duration=0.8,
        minimum_duration_override=0.30,
    )

    assert outcome.plan is not None
    assert outcome.plan.duration == pytest.approx(0.8)


def test_terminal_bounded_search_fails_instead_of_selecting_late_plan():
    """Catch terminal fallback searching past the 0.30 s contact window."""
    planner = make_planner(maximum_duration=4.0)

    def target(_horizon):
        return (
            (2.0, 0.0, -0.1),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        )

    unbounded = planner.plan(
        initial_position=(0.0, 0.0, -1.0),
        initial_velocity=(0.0, 0.0, 0.0),
        initial_acceleration=(0.0, 0.0, 0.0),
        target_state_at_time=target,
        preferred_duration=0.8,
        minimum_duration_override=0.8,
    )
    bounded = planner.plan(
        initial_position=(0.0, 0.0, -1.0),
        initial_velocity=(0.0, 0.0, 0.0),
        initial_acceleration=(0.0, 0.0, 0.0),
        target_state_at_time=target,
        preferred_duration=0.8,
        minimum_duration_override=0.8,
        maximum_duration_override=1.1,
    )

    assert unbounded.plan is not None
    assert unbounded.plan.duration > 1.1
    assert bounded.plan is None
    assert bounded.failure == FastPlanningFailure.HORIZON_INSUFFICIENT


def test_horizontal_and_vertical_dynamic_failures_are_distinct():
    horizontal = make_planner(
        maximum_horizontal_speed=2.1,
        maximum_horizontal_acceleration=0.5,
        maximum_vertical_acceleration=8.0,
    ).plan(
        initial_position=(0.0, 0.0, -0.1),
        initial_velocity=(2.0, 0.0, 0.0),
        initial_acceleration=(0.0, 0.0, 0.0),
        target_state_at_time=lambda _horizon: (
            (5.0, 0.0, -0.1),
            (2.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        ),
    )
    vertical = make_planner(
        maximum_horizontal_acceleration=8.0,
        maximum_vertical_speed=0.5,
        maximum_vertical_acceleration=0.5,
    ).plan(
        initial_position=(0.0, 0.0, -3.0),
        initial_velocity=(2.0, 0.0, 0.0),
        initial_acceleration=(0.0, 0.0, 0.0),
        target_state_at_time=lambda _horizon: (
            (3.0, 0.0, -0.1),
            (2.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        ),
    )

    assert horizontal.failure == FastPlanningFailure.DYNAMIC_LIMIT_HORIZONTAL
    assert vertical.failure in (
        FastPlanningFailure.DYNAMIC_LIMIT_VERTICAL,
        FastPlanningFailure.HORIZON_INSUFFICIENT,
    )
