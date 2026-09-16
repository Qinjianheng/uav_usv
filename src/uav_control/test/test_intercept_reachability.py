import math

import pytest

from uav_control.guidance.finite_horizon_intercept_planner import (
    FiniteHorizonInterceptPlanner,
    PlanningFailureReason,
)
from uav_control.guidance.intercept_reachability import (
    SeaSafetyState,
    _velocity_envelope_integral,
    apply_sea_safety_guard,
    estimate_reachability,
)


def test_vertical_reachability_dominates_high_dive_case():
    estimate = estimate_reachability(
        initial_position=(0.0, 0.0, -5.0),
        initial_velocity=(4.0, 0.0, 0.0),
        target_position=(4.0, 0.0, -0.1),
        target_velocity=(4.0, 0.0, 0.0),
        maximum_horizontal_speed=6.5,
        maximum_vertical_speed=4.0,
        maximum_horizontal_acceleration=3.0,
        maximum_vertical_acceleration=3.0,
        vertical_braking_acceleration=2.5,
        response_delay=0.15,
        search_limit=4.0,
    )

    assert math.isfinite(estimate.vertical_min_time)
    assert estimate.vertical_min_time > estimate.horizontal_min_time
    assert estimate.required_time == pytest.approx(
        estimate.vertical_min_time
    )


def test_low_altitude_fast_descent_enters_emergency_braking():
    result = apply_sea_safety_guard(
        current_z=-0.3,
        current_vz=2.0,
        proposed_vz=2.2,
        sea_surface_z=0.0,
        reserve_clearance=0.07,
        response_delay=0.15,
        effective_braking_acceleration=2.5,
        control_dt=0.05,
        warning_margin=0.15,
        maximum_vertical_speed=4.0,
    )

    assert result.state in (
        SeaSafetyState.BRAKE,
        SeaSafetyState.UNRECOVERABLE,
    )
    assert result.command_vz < 2.0


def test_safe_altitude_keeps_a_safe_command_unchanged():
    result = apply_sea_safety_guard(
        current_z=-5.0,
        current_vz=0.2,
        proposed_vz=0.3,
        sea_surface_z=0.0,
        reserve_clearance=0.07,
        response_delay=0.15,
        effective_braking_acceleration=2.5,
        control_dt=0.05,
        warning_margin=0.15,
        maximum_vertical_speed=4.0,
    )

    assert result.state == SeaSafetyState.SAFE
    assert result.command_vz == pytest.approx(0.3)


def test_velocity_envelope_integral_is_exact_at_off_grid_switch():
    upper = _velocity_envelope_integral(
        duration=1.0,
        initial_velocity=0.0,
        final_velocity=0.2,
        maximum_speed=10.0,
        positive_acceleration=1.0,
        negative_acceleration=2.0,
        maximize=True,
    )
    lower = _velocity_envelope_integral(
        duration=1.0,
        initial_velocity=0.0,
        final_velocity=-0.2,
        maximum_speed=10.0,
        positive_acceleration=2.0,
        negative_acceleration=1.0,
        maximize=False,
    )

    assert upper == pytest.approx(0.3933333333333333, abs=1e-12)
    assert lower == pytest.approx(-0.3933333333333333, abs=1e-12)


def test_dynamic_horizon_reports_insufficient_absolute_horizon():
    planner = FiniteHorizonInterceptPlanner(
        minimum_duration=1.0,
        maximum_duration=2.0,
        absolute_maximum_duration=2.2,
        horizon_extra_margin=0.5,
        duration_step=0.1,
        sample_step=0.05,
        maximum_horizontal_speed=6.5,
        maximum_vertical_speed=2.0,
        maximum_horizontal_acceleration=3.0,
        maximum_vertical_acceleration=1.0,
        effective_vertical_braking_acceleration=1.0,
        desired_closing_speed=1.0,
        minimum_closing_speed=0.3,
        closing_speed_step=0.3,
        capture_radius=0.25,
        sea_surface_z=0.0,
        contact_clearance=0.05,
    )

    plan = planner.plan(
        (0.0, 0.0, -5.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        lambda _horizon: (
            (0.0, 0.0, -0.1),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        ),
    )

    assert plan is None
    assert planner.last_diagnostics.failure_reason == (
        PlanningFailureReason.HORIZON_INSUFFICIENT
    )
    assert planner.last_diagnostics.vertical_min_time > 2.2
