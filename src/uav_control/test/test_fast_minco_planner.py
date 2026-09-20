"""Behavior tests for the bounded realtime MINCO search."""

import math

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


def test_staged_duration_search_sweeps_the_horizon_after_dynamic_failures():
    """
    A dynamic-limit failure must explore the whole horizon, not one step.

    The previous form advanced by a single duration_margin step and then
    stopped, which left the long-duration end -- exactly where the required
    acceleration becomes feasible -- unprobed and rejected every terminal plan
    of the 2026-09-20 runs while the vehicle hovered near the sea.
    """
    planner = make_planner(maximum_duration=4.0)
    planner.last_success_duration = 3.5

    durations = iter(planner._duration_candidates(1.2, 4.0))
    assert next(durations) == pytest.approx(1.2)
    planner._candidate_failures.append(
        FastPlanningFailure.DYNAMIC_LIMIT_VERTICAL
    )
    assert next(durations) == pytest.approx(3.5)
    planner._candidate_failures.append(FastPlanningFailure.SEA_CLEARANCE)

    assert list(durations) == pytest.approx([1.9, 2.6, 3.3, 4.0])


def test_candidate_diagnostics_preserve_all_constraint_violations():
    planner = make_planner(
        maximum_horizontal_speed=2.1,
        maximum_horizontal_acceleration=0.5,
        maximum_vertical_speed=4.0,
        maximum_vertical_acceleration=8.0,
    )

    outcome = planner.plan(
        initial_position=(0.0, 0.0, -0.1),
        initial_velocity=(2.0, 0.0, 0.0),
        initial_acceleration=(0.0, 0.0, 0.0),
        target_state_at_time=lambda _horizon: (
            (5.0, 0.0, -0.1),
            (2.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        ),
    )

    diagnostics = outcome.diagnostics
    assert diagnostics.reachability_compute_time >= 0.0
    assert diagnostics.validation_compute_time >= 0.0
    assert 1 <= len(diagnostics.candidate_diagnostics) <= 6
    candidate = diagnostics.candidate_diagnostics[0]
    assert candidate.duration > 0.0
    assert candidate.closing_speed > 0.0
    assert candidate.generation_time >= 0.0
    assert candidate.validation_time >= 0.0
    assert candidate.maximum_horizontal_speed >= 0.0
    assert candidate.maximum_vertical_speed >= 0.0
    assert candidate.maximum_horizontal_acceleration >= 0.0
    assert candidate.maximum_vertical_acceleration >= 0.0
    assert candidate.maximum_sea_clearance_violation >= 0.0
    assert candidate.maximum_constraint_violation > 0.0
    assert 0.0 <= candidate.maximum_violation_time <= candidate.duration
    assert candidate.maximum_violation_phase in ('START', 'MIDDLE', 'END')
    assert candidate.failure != FastPlanningFailure.NONE.value
    assert candidate.violations


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


def test_sea_state_heave_must_not_pull_waypoints_into_the_clearance_zone():
    """
    A heaving surface target must not drag MINCO waypoints toward the sea.

    The target curve weight used to be applied to all three axes, so the USV's
    vertical heave was blended into the intermediate waypoints.  Because the
    USV sits at the sea surface, that pushed the waypoints below the clearance
    ceiling and rejected otherwise flyable terminal dives with SEA_CLEARANCE.
    """
    planner = make_planner(
        maximum_duration=4.0,
        sample_step=0.1,
        # Production envelope from baseline.yaml: relaxing these would let a
        # shallower dive pass and hide the geometry defect.
        maximum_horizontal_acceleration=3.0,
        maximum_vertical_acceleration=3.0,
        effective_vertical_braking_acceleration=2.5,
        capture_radius=0.35,
        preferred_clearance=0.1,
        quadrature_intervals_per_piece=10,
        # Keep this geometry regression independent of host timing.
        deadline_seconds=0.5,
    )

    amplitude = 0.15
    angular_frequency = 2.0 * math.pi * 0.25
    # Phase chosen so the heave pushes the target below the sea surface while
    # the waypoints are being placed.
    heave_phase = 3.0

    def heaving_surface_target(horizon):
        angle = angular_frequency * horizon + heave_phase
        return (
            (
                -3.0 + 4.0 * horizon,
                0.8,
                amplitude * math.sin(angle),
            ),
            (
                4.0,
                0.0,
                amplitude * angular_frequency * math.cos(angle),
            ),
            (
                0.0,
                0.0,
                -amplitude * angular_frequency**2 * math.sin(angle),
            ),
        )

    outcome = planner.plan(
        initial_position=(0.0, 0.0, -0.35),
        initial_velocity=(0.35, 0.0, 0.10),
        initial_acceleration=(0.0, 0.0, 0.0),
        target_state_at_time=heaving_surface_target,
    )

    assert outcome.failure == FastPlanningFailure.NONE
    assert outcome.plan is not None
    samples = [
        outcome.plan.sample(outcome.plan.duration * index / 200.0)
        for index in range(201)
    ]
    assert all(
        sample.position[2] <= outcome.plan.sea_clearance_ceiling_z + 1e-6
        for sample in samples
    )


# Recorded target truth for the first-descent stall of run
# modular_intercept_20260920_113839: the figure-eight target turns hard while
# the UAV hovers 0.66 m above the sea two metres behind it.
_TURNING_TARGET_TRACK = (
    (0.00, (70.923, -18.785, 0.043), (3.690, -1.545, 0.039)),
    (0.49, (72.770, -19.450, 0.048), (3.830, -1.140, 0.000)),
    (1.04, (74.920, -19.910, 0.022), (3.970, -0.500, 0.000)),
    (1.63, (77.280, -19.950, -0.023), (3.980, 0.420, 0.000)),
    (2.15, (79.290, -19.490, -0.048), (3.760, 1.350, 0.000)),
    (2.63, (81.020, -18.640, -0.044), (3.360, 2.160, 0.000)),
    (3.15, (82.620, -17.340, -0.014), (2.830, 2.820, 0.000)),
    (3.65, (83.910, -15.810, 0.024), (2.340, 3.250, 0.000)),
    (4.15, (84.970, -14.110, 0.048), (1.900, 3.520, 0.000)),
)


def _turning_target_state(horizon):
    horizon = max(float(horizon), 0.0)
    for index in range(1, len(_TURNING_TARGET_TRACK)):
        start_time, start_position, start_velocity = (
            _TURNING_TARGET_TRACK[index - 1]
        )
        end_time, end_position, end_velocity = _TURNING_TARGET_TRACK[index]
        if horizon <= end_time or index == len(_TURNING_TARGET_TRACK) - 1:
            span = end_time - start_time
            fraction = 0.0 if span <= 0.0 else min(
                max((horizon - start_time) / span, 0.0),
                1.0,
            )
            return (
                tuple(
                    start + fraction * (end - start)
                    for start, end in zip(start_position, end_position)
                ),
                tuple(
                    start + fraction * (end - start)
                    for start, end in zip(start_velocity, end_velocity)
                ),
                (0.0, 0.0, 0.0),
            )
    raise AssertionError('target track exhausted')


def test_curve_weight_must_keep_a_turning_terminal_path_in_budget():
    """
    Guard the 2026-09-20 curve-guidance regression.

    Blending the MINCO intermediate waypoints toward the target's curved path
    made the terminal trajectory demand 4-8 m/s^2 of lateral acceleration
    against the 3.0 m/s^2 planning limit, so every terminal plan was rejected
    with DYNAMIC_LIMIT_HORIZONTAL while the vehicle hovered near the sea.  The
    production weight is therefore 0.0.
    """
    initial_position = (68.835, -19.082, -0.662)
    initial_velocity = (3.521, -1.903, -0.276)

    def plan(curve_weight):
        planner = make_planner(
            maximum_duration=4.0,
            maximum_horizontal_acceleration=3.0,
            maximum_vertical_acceleration=3.0,
            capture_radius=0.50,
            contact_clearance=0.27,
            preferred_clearance=0.29,
            target_curve_weight=curve_weight,
        )
        return planner.plan(
            initial_position=initial_position,
            initial_velocity=initial_velocity,
            initial_acceleration=(0.0, 0.0, 0.0),
            target_state_at_time=_turning_target_state,
            preferred_duration=1.96,
            minimum_duration_override=1.96,
            maximum_duration_override=3.92,
        )

    straight = plan(0.0)
    assert straight.plan is not None
    assert straight.plan.maximum_horizontal_acceleration <= 3.0
    # Characterize why the production weight cannot stay at the old 0.7.
    assert plan(0.7).plan is None
