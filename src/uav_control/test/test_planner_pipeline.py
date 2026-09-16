"""Tests for planner request timing and latest-only scheduling."""

import pytest

from uav_control.guidance.fast_minco_planner import FastPlanningFailure
from uav_control.guidance.planner_pipeline import LatestRequestSlot
from uav_control.guidance.planner_pipeline import PlannerRequest
from uav_control.guidance.planner_pipeline import PredictionSample
from uav_control.guidance.planner_pipeline import PredictionSeries
from uav_control.guidance.planner_pipeline import UavKinematicState
from uav_control.guidance.planner_pipeline import validate_input
from uav_control.guidance.planner_pipeline import validate_plan_arrival
from uav_control.guidance.planner_pipeline import validate_target_shift
from uav_control.guidance.planner_pipeline import validate_total_deadline
from uav_control.guidance import planner_pipeline


def make_request(sequence_id=1, prediction_stamp=10.0, uav_stamp=10.02):
    """Return one internally consistent planner request."""
    prediction = PredictionSeries(
        mission_id=5,
        sequence_id=sequence_id,
        source_stamp=prediction_stamp,
        valid_until=prediction_stamp + 0.125,
        samples=(
            PredictionSample(0.0, (1.0, 0.0, -0.1), (2.0, 0.0, 0.0),
                             (0.0, 0.0, 0.0)),
            PredictionSample(1.0, (3.0, 0.0, -0.1), (2.0, 0.0, 0.0),
                             (0.0, 0.0, 0.0)),
        ),
        source='simulation_truth',
    )
    uav = UavKinematicState(
        stamp=uav_stamp,
        position=(0.0, 0.0, -1.0),
        velocity=(0.0, 0.0, 0.0),
        acceleration=(0.0, 0.0, 0.0),
    )
    return PlannerRequest(
        mission_id=5,
        prediction=prediction,
        uav=uav,
        trajectory_start_stamp=uav.stamp,
    )


def test_prediction_series_interpolates_without_future_truth_access():
    state = make_request().prediction.state_at(0.25)

    assert state[0] == pytest.approx((1.5, 0.0, -0.1))
    assert state[1] == pytest.approx((2.0, 0.0, 0.0))
    assert state[2] == pytest.approx((0.0, 0.0, 0.0))


def test_latest_request_replaces_pending_request_instead_of_queueing():
    slot = LatestRequestSlot()
    first = make_request(sequence_id=1)
    latest = make_request(sequence_id=3)

    slot.submit(first)
    slot.submit(make_request(sequence_id=2))
    slot.submit(latest)

    assert slot.replaced_request_count == 2
    assert slot.take() is latest
    assert slot.take() is None


def test_stale_prediction_and_uav_state_are_reported_separately():
    prediction_stale = make_request(prediction_stamp=9.8, uav_stamp=10.0)
    state_stale = make_request(prediction_stamp=10.0, uav_stamp=9.8)

    assert validate_input(prediction_stale, now=10.01, maximum_age=0.125) == (
        FastPlanningFailure.PREDICTION_STALE
    )
    assert validate_input(state_stale, now=10.01, maximum_age=0.125) == (
        FastPlanningFailure.STATE_STALE
    )


def test_prediction_staleness_remains_independent_of_minco_start_stamp():
    """A fresh UAV start must not make an older prediction acceptable."""
    request = make_request(prediction_stamp=9.90, uav_stamp=10.10)
    request = PlannerRequest(
        mission_id=request.mission_id,
        prediction=request.prediction,
        uav=request.uav,
        trajectory_start_stamp=10.10,
    )

    assert validate_input(request, now=10.15, maximum_age=0.125) == (
        FastPlanningFailure.PREDICTION_STALE
    )


def test_plan_age_is_measured_from_oldest_input_not_completion_time():
    request = make_request(prediction_stamp=10.0, uav_stamp=10.02)

    failure = validate_plan_arrival(
        request,
        generated_stamp=10.126,
        maximum_age=0.125,
    )

    assert failure == FastPlanningFailure.PLAN_STALE_ON_ARRIVAL


def test_changed_prediction_endpoint_rejects_completed_plan():
    request = make_request(sequence_id=1)
    shifted = make_request(sequence_id=2).prediction
    shifted_samples = list(shifted.samples)
    shifted_samples[-1] = PredictionSample(
        1.0,
        (4.0, 0.0, -0.1),
        (2.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
    )
    shifted = PredictionSeries(
        mission_id=shifted.mission_id,
        sequence_id=shifted.sequence_id,
        source_stamp=shifted.source_stamp,
        valid_until=shifted.valid_until,
        samples=tuple(shifted_samples),
        source=shifted.source,
    )

    failure = validate_target_shift(
        request=request,
        latest_prediction=shifted,
        intercept_time=1.0,
        tolerance=0.5,
    )

    assert failure == FastPlanningFailure.PLAN_STALE_ON_ARRIVAL


def test_target_shift_uses_absolute_contact_time_when_sources_differ():
    def series(source_stamp, sequence_id):
        return PredictionSeries(
            mission_id=5,
            sequence_id=sequence_id,
            source_stamp=source_stamp,
            valid_until=source_stamp + 4.0,
            samples=tuple(
                PredictionSample(
                    relative_time=float(index),
                    position=(4.0 * (source_stamp + index), 0.0, -0.1),
                    velocity=(4.0, 0.0, 0.0),
                    acceleration=(0.0, 0.0, 0.0),
                )
                for index in range(5)
            ),
            source='simulation_truth',
        )

    request = PlannerRequest(
        mission_id=5,
        prediction=series(10.0, 100),
        uav=make_request().uav,
        trajectory_start_stamp=make_request().uav.stamp,
    )

    failure = validate_target_shift(
        request=request,
        latest_prediction=series(10.2, 102),
        intercept_time=3.0,
        tolerance=0.05,
    )

    assert failure == FastPlanningFailure.NONE


def test_target_shift_rejects_latest_prediction_beyond_available_horizon():
    request = make_request()
    short_latest = PredictionSeries(
        mission_id=5,
        sequence_id=2,
        source_stamp=10.2,
        valid_until=11.2,
        samples=(
            PredictionSample(
                0.0, (1.0, 0.0, -0.1), (2.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            ),
            PredictionSample(
                1.0, (3.0, 0.0, -0.1), (2.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            ),
        ),
        source='simulation_truth',
    )

    failure = validate_target_shift(
        request=request,
        latest_prediction=short_latest,
        intercept_time=3.0,
        tolerance=0.5,
    )

    assert failure == FastPlanningFailure.PLAN_STALE_ON_ARRIVAL


def test_total_planner_deadline_includes_diagnostics_overhead():
    assert validate_total_deadline(0.079, 0.08) == FastPlanningFailure.NONE
    assert validate_total_deadline(0.080, 0.08) == (
        FastPlanningFailure.DEADLINE_EXCEEDED
    )


def test_contact_schedule_counts_down_absolute_contact_time():
    """Catch replanning that silently moves contact three seconds ahead."""
    schedule = planner_pipeline.ContactTimeSchedule(
        terminal_threshold=1.0,
    )

    schedule.accept_plan(source_stamp=10.0, selected_t_go=3.2)

    assert schedule.contact_stamp == pytest.approx(13.2)
    assert schedule.remaining_t_go(10.2) == pytest.approx(3.0)
    assert schedule.remaining_t_go(10.4) == pytest.approx(2.8)
    assert not schedule.is_terminal(12.1)
    assert schedule.is_terminal(12.3)


def test_contact_schedule_resets_between_missions():
    schedule = planner_pipeline.ContactTimeSchedule(terminal_threshold=1.0)
    schedule.accept_plan(source_stamp=10.0, selected_t_go=3.0)

    schedule.reset()

    assert schedule.contact_stamp is None
    assert schedule.preferred_t_go(10.2) is None


def test_terminal_contact_stamp_counts_down_without_drifting():
    """Catch terminal replans replacing the locked absolute contact time."""
    schedule = planner_pipeline.ContactTimeSchedule(
        terminal_threshold=1.0,
        freeze_time=0.30,
    )
    schedule.accept_plan(source_stamp=10.0, selected_t_go=1.0)

    remaining = [
        schedule.remaining_t_go(stamp)
        for stamp in (10.2, 10.4, 10.6)
    ]

    assert schedule.contact_stamp == pytest.approx(11.0)
    assert remaining == pytest.approx((0.8, 0.6, 0.4))


def test_contact_schedule_accepts_only_forward_terminal_reschedules():
    """Allow necessary delay without ever moving committed contact backward."""
    schedule = planner_pipeline.ContactTimeSchedule(
        terminal_threshold=1.0,
        freeze_time=0.30,
    )
    schedule.accept_plan(source_stamp=10.0, selected_t_go=1.0)

    assert schedule.accept_plan(
        source_stamp=10.0,
        selected_t_go=1.8,
        rescheduled=True,
    ) == pytest.approx(11.8)

    # A later replan may not move the committed contact backward.
    assert schedule.accept_plan(
        source_stamp=10.0,
        selected_t_go=1.4,
        rescheduled=True,
    ) == pytest.approx(11.8)

    # A normal, non-rescheduled plan may not move it either.
    assert schedule.accept_plan(
        source_stamp=10.0,
        selected_t_go=2.0,
        rescheduled=False,
    ) == pytest.approx(11.8)


@pytest.mark.parametrize(
    ('remaining', 'expected'),
    ((0.31, True), (0.30, False), (0.20, False)),
)
def test_terminal_replanning_stops_at_freeze_boundary(remaining, expected):
    """Catch a new terminal plan replacing the final committed trajectory."""
    schedule = planner_pipeline.ContactTimeSchedule(
        terminal_threshold=1.0,
        freeze_time=0.30,
    )
    schedule.accept_plan(source_stamp=10.0, selected_t_go=1.0)

    assert schedule.should_replan(11.0 - remaining) is expected


def test_terminal_mission_with_point_eight_seconds_allows_planner_request():
    policy = planner_pipeline.PlanningRequestPolicy(
        terminal_minimum_duration=0.30,
    )

    decision = policy.decide(
        mission_state=7,
        remaining_t_go=0.8,
    )

    assert decision.submit
    assert decision.minimum_duration == pytest.approx(0.30)
    assert decision.terminal_mode


@pytest.mark.parametrize(
    ('mission_state', 'remaining_t_go'),
    ((7, 2.0), (6, 0.8)),
)
def test_planner_marks_terminal_from_mission_or_remaining_time(
    mission_state,
    remaining_t_go,
):
    """Catch planner trajectories disagreeing with terminal mission state."""
    assert planner_pipeline.terminal_mode_for_plan(
        mission_state=mission_state,
        request_terminal_mode=False,
        remaining_t_go=remaining_t_go,
        terminal_time_threshold=1.0,
        terminal_state=7,
    )
