"""Contract tests for the modular interception ROS interfaces."""

import importlib

import pytest
from rclpy.serialization import deserialize_message, serialize_message


def message_class(name):
    """Load one generated message class and report a useful test failure."""
    module_name = ''.join(
        f'_{character.lower()}' if character.isupper() else character
        for character in name
    ).lstrip('_')
    try:
        module = importlib.import_module(
            f'uav_usv_interfaces.msg._{module_name}'
        )
    except ModuleNotFoundError:
        pytest.fail(f'modular message {name} has not been generated')
    return getattr(module, name)


def round_trip(message, message_type):
    """Serialize and deserialize through the real ROS 2 type support."""
    return deserialize_message(serialize_message(message), message_type)


def test_target_prediction_preserves_identity_and_source_timestamps():
    target_prediction = message_class('TargetPrediction')
    message = target_prediction()
    message.mission_id = 41
    message.sequence_id = 17
    message.source_stamp.sec = 100
    message.generated_stamp.sec = 101
    message.valid_until.sec = 102
    message.source = 'simulation_truth'
    message.model = 'BCTRA_BOUNDED_Z'
    message.prediction_horizon = 3.0
    message.valid = True

    restored = round_trip(message, target_prediction)

    assert restored.mission_id == 41
    assert restored.sequence_id == 17
    assert restored.source_stamp.sec == 100
    assert restored.generated_stamp.sec == 101
    assert restored.valid_until.sec == 102
    assert restored.source == 'simulation_truth'


def test_intercept_trajectory_carries_complete_piecewise_polynomial():
    intercept_trajectory = message_class('InterceptTrajectory')
    polynomial_segment = message_class('PolynomialSegment')
    message = intercept_trajectory()
    message.mission_id = 8
    message.plan_id = 12
    message.prediction_sequence_id = 33
    message.source_stamp.sec = 50
    message.planning_started_stamp.sec = 51
    message.generated_stamp.sec = 52
    message.published_stamp.sec = 53
    message.valid_until.sec = 54
    message.piece_count = 1
    message.trajectory_duration = 2.0
    message.contact_stamp.sec = 54
    message.selected_t_go = 2.0
    message.remaining_t_go = 1.9
    message.terminal_mode = False
    message.planned_capture_margin = 0.2
    message.valid = True
    segment = polynomial_segment()
    segment.duration.sec = 2
    segment.coefficients = [float(value) for value in range(18)]
    message.segments = [segment]

    restored = round_trip(message, intercept_trajectory)

    assert restored.mission_id == 8
    assert restored.plan_id == 12
    assert restored.prediction_sequence_id == 33
    assert restored.source_stamp.sec == 50
    assert restored.planning_started_stamp.sec == 51
    assert restored.generated_stamp.sec == 52
    assert restored.published_stamp.sec == 53
    assert restored.valid_until.sec == 54
    assert restored.contact_stamp.sec == 54
    assert restored.selected_t_go == pytest.approx(2.0)
    assert restored.remaining_t_go == pytest.approx(1.9)
    assert not restored.terminal_mode
    assert restored.planned_capture_margin == pytest.approx(0.2)
    assert restored.piece_count == len(restored.segments) == 1
    assert restored.segments[0].duration.sec == 2
    assert list(restored.segments[0].coefficients) == [
        float(value) for value in range(18)
    ]


def test_planner_diagnostic_failure_reasons_are_distinct_constants():
    diagnostic = message_class('PlannerDiagnostic')
    reasons = {
        diagnostic.STATE_STALE,
        diagnostic.PREDICTION_STALE,
        diagnostic.HORIZON_INSUFFICIENT,
        diagnostic.CAPTURE_GEOMETRY,
        diagnostic.DYNAMIC_LIMIT_HORIZONTAL,
        diagnostic.DYNAMIC_LIMIT_VERTICAL,
        diagnostic.SEA_CLEARANCE,
        diagnostic.MINCO_CONSTRUCTION_FAIL,
        diagnostic.OPTIMIZATION_FAIL,
        diagnostic.DEADLINE_EXCEEDED,
        diagnostic.PLAN_STALE_ON_ARRIVAL,
    }

    assert diagnostic.NONE not in reasons
    assert len(reasons) == 11


def test_planner_diagnostic_carries_publish_age_and_completion_delay():
    diagnostic = message_class('PlannerDiagnostic')
    message = diagnostic()
    message.input_age_at_finish = 0.058
    message.input_age_at_publish = 0.064
    message.completion_to_publish_delay = 0.006
    message.selected_t_go = 2.8
    message.contact_stamp.sec = 103
    message.remaining_t_go = 2.7
    message.terminal_mode = False
    message.planned_capture_margin = 0.18
    message.required_time = 1.42
    message.horizontal_min_time = 1.10
    message.vertical_min_time = 1.42
    message.sea_safe_min_time = 0.65
    message.search_min_time = 1.42
    message.search_max_time = 1.20
    message.available_prediction_duration = 3.85
    message.locked_remaining_t_go = 0.90

    restored = round_trip(message, diagnostic)

    assert restored.input_age_at_finish == pytest.approx(0.058)
    assert restored.input_age_at_publish == pytest.approx(0.064)
    assert restored.completion_to_publish_delay == pytest.approx(0.006)
    assert restored.selected_t_go == pytest.approx(2.8)
    assert restored.contact_stamp.sec == 103
    assert restored.remaining_t_go == pytest.approx(2.7)
    assert restored.planned_capture_margin == pytest.approx(0.18)
    assert restored.required_time == pytest.approx(1.42)
    assert restored.horizontal_min_time == pytest.approx(1.10)
    assert restored.vertical_min_time == pytest.approx(1.42)
    assert restored.sea_safe_min_time == pytest.approx(0.65)
    assert restored.search_min_time == pytest.approx(1.42)
    assert restored.search_max_time == pytest.approx(1.20)
    assert restored.available_prediction_duration == pytest.approx(3.85)
    assert restored.locked_remaining_t_go == pytest.approx(0.90)


def test_mission_state_constants_are_machine_parseable():
    mission_state = message_class('MissionState')

    assert mission_state.INIT == 0
    assert mission_state.MINCO_READY != mission_state.MINCO_TRACKING
    assert mission_state.SAFE_WAIT != mission_state.FAILURE


def test_controller_diagnostic_carries_plan_identity_and_timing():
    controller_diagnostic = message_class('ControllerDiagnostic')
    message = controller_diagnostic()
    message.mission_id = 2
    message.plan_id = 9
    message.source_age = 0.08
    message.prediction_age = 0.03
    message.trajectory_age = 0.08
    message.callback_compute_time = 0.002
    message.status = 'TRACKING'
    message.target_distance = 1.8
    message.remaining_t_go = 0.8
    message.terminal_mode = True

    restored = round_trip(message, controller_diagnostic)

    assert restored.mission_id == 2
    assert restored.plan_id == 9
    assert restored.source_age == pytest.approx(0.08)
    assert restored.prediction_age == pytest.approx(0.03)
    assert restored.trajectory_age == pytest.approx(0.08)
    assert restored.callback_compute_time == pytest.approx(0.002)
    assert restored.target_distance == pytest.approx(1.8)
    assert restored.remaining_t_go == pytest.approx(0.8)
    assert restored.terminal_mode


def test_intercept_result_is_scoped_to_one_mission():
    intercept_result = message_class('InterceptResult')
    message = intercept_result()
    message.mission_id = 15
    message.success = False
    message.outcome = 'FAILURE'
    message.reason = 'TIMEOUT'

    restored = round_trip(message, intercept_result)

    assert restored.mission_id == 15
