"""Message conversion tests for the independent interception planner."""

import ast
import inspect
from types import SimpleNamespace

import pytest
from px4_msgs.msg import VehicleLocalPosition
from uav_usv_interfaces.msg import PredictedTargetPoint, TargetPrediction

from uav_control.guidance.fast_minco_planner import FastMincoPlanner
from uav_control.guidance.fast_minco_planner import FastPlanningFailure
from uav_control.guidance.fast_minco_planner import FastPlanningOutcome
from uav_control.guidance.finite_horizon_intercept_planner import PlannerDiagnostics
from uav_control.guidance.intercept_planner_node import plan_to_message
from uav_control.guidance.intercept_planner_node import prediction_from_message
from uav_control.guidance.intercept_planner_node import uav_state_from_message
from uav_control.guidance.planner_pipeline import PlannerRequest
from uav_control.guidance.planner_pipeline import PredictionSample
from uav_control.guidance.planner_pipeline import PredictionSeries
from uav_control.guidance.planner_pipeline import UavKinematicState
import uav_control.guidance.intercept_planner_node as planner_node_module


def test_planner_node_does_not_shadow_rclpy_executor_property():
    tree = ast.parse(inspect.getsource(planner_node_module.InterceptPlannerNode))
    assignments = [
        target.attr
        for node in ast.walk(tree)
        if isinstance(node, (ast.Assign, ast.AnnAssign))
        for target in (
            node.targets if isinstance(node, ast.Assign) else [node.target]
        )
        if isinstance(target, ast.Attribute)
        and isinstance(target.value, ast.Name)
        and target.value.id == 'self'
    ]

    assert 'executor' not in assignments


def test_planner_submission_and_completion_use_independent_timers():
    tree = ast.parse(inspect.getsource(planner_node_module.InterceptPlannerNode))
    timer_callbacks = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Attribute):
            continue
        if node.func.attr != 'create_timer' or len(node.args) < 2:
            continue
        callback = node.args[1]
        if isinstance(callback, ast.Attribute):
            timer_callbacks.append(callback.attr)

    assert 'planning_timer_callback' in timer_callbacks
    assert 'terminal_planning_timer_callback' in timer_callbacks
    assert 'completion_timer_callback' in timer_callbacks


def test_planner_input_conversion_preserves_prediction_source_stamp():
    message = TargetPrediction()
    message.mission_id = 3
    message.sequence_id = 5
    message.source_stamp.sec = 12
    message.source_stamp.nanosec = 100_000_000
    message.valid_until.sec = 12
    message.valid_until.nanosec = 225_000_000
    message.source = 'simulation_truth'
    sample = PredictedTargetPoint()
    sample.relative_time.sec = 1
    sample.position.x = 4.0
    sample.velocity.x = 2.0
    message.samples = [sample]
    message.valid = True

    prediction = prediction_from_message(message)

    assert prediction.source_stamp == pytest.approx(12.1)
    assert prediction.valid_until == pytest.approx(12.225)
    assert prediction.samples[0].relative_time == pytest.approx(1.0)
    assert prediction.samples[0].position == pytest.approx((4.0, 0.0, 0.0))


def test_uav_input_uses_ros_receive_time_for_comparable_clock_domain():
    message = VehicleLocalPosition()
    message.x = 1.0
    message.y = 2.0
    message.z = -3.0
    message.vx = 4.0
    message.vy = 0.5
    message.vz = -0.2

    state = uav_state_from_message(message, received_stamp=8.25)

    assert state.stamp == pytest.approx(8.25)
    assert state.position == pytest.approx((1.0, 2.0, -3.0))
    assert state.velocity == pytest.approx((4.0, 0.5, -0.2))


def test_planner_request_uses_latest_uav_stamp_as_minco_start():
    prediction = PredictionSeries(
        mission_id=2,
        sequence_id=4,
        source_stamp=10.00,
        valid_until=11.00,
        samples=(
            PredictionSample(
                0.0,
                (10.0, 0.0, -0.1),
                (1.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            ),
            PredictionSample(
                1.0,
                (11.0, 0.0, -0.1),
                (1.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            ),
        ),
        source='simulation_truth',
    )
    uav = UavKinematicState(
        stamp=10.10,
        position=(0.0, 0.0, -1.0),
        velocity=(0.0, 0.0, 0.0),
        acceleration=(0.0, 0.0, 0.0),
    )
    node = SimpleNamespace(
        mission_id=2,
        latest_prediction=prediction,
        latest_uav=uav,
        contact_schedule=SimpleNamespace(contact_stamp=None),
    )
    decision = SimpleNamespace(
        terminal_mode=False,
        minimum_duration=1.0,
    )

    request = planner_node_module.InterceptPlannerNode._current_request(
        node,
        decision,
    )

    assert request.trajectory_start_stamp == pytest.approx(10.10)


def test_planner_queries_prediction_at_absolute_minco_contact_time():
    class CapturingPlanner:
        minimum_duration = 0.10

        def plan(self, **kwargs):
            self.target_at_quarter_second = kwargs['target_state_at_time'](
                0.25
            )
            return FastPlanningOutcome(
                plan=None,
                failure=FastPlanningFailure.CAPTURE_GEOMETRY,
                diagnostics=PlannerDiagnostics(),
            )

    prediction = PredictionSeries(
        mission_id=2,
        sequence_id=4,
        source_stamp=10.00,
        valid_until=11.00,
        samples=(
            PredictionSample(
                0.0,
                (10.0, 0.0, -0.1),
                (1.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            ),
            PredictionSample(
                1.0,
                (11.0, 0.0, -0.1),
                (1.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            ),
        ),
        source='simulation_truth',
    )
    request = PlannerRequest(
        mission_id=2,
        prediction=prediction,
        uav=UavKinematicState(
            stamp=10.10,
            position=(0.0, 0.0, -1.0),
            velocity=(0.0, 0.0, 0.0),
            acceleration=(0.0, 0.0, 0.0),
        ),
        trajectory_start_stamp=10.10,
    )
    planner = CapturingPlanner()
    node = SimpleNamespace(
        planner=planner,
        maximum_input_age=0.20,
        hard_deadline_seconds=1.0,
        _ros_seconds=lambda: 10.15,
    )

    planner_node_module.InterceptPlannerNode._run_request(node, request)

    assert planner.target_at_quarter_second[0] == pytest.approx(
        (10.35, 0.0, -0.1)
    )


def test_minco_plan_message_contains_reconstructable_coefficients():
    planner = FastMincoPlanner(
        minimum_duration=1.0,
        maximum_duration=3.0,
        duration_margin=0.35,
        sample_step=0.05,
        maximum_horizontal_speed=7.0,
        maximum_vertical_speed=4.0,
        maximum_horizontal_acceleration=8.0,
        maximum_vertical_acceleration=4.0,
        preferred_closing_speed=1.5,
        conservative_closing_speed=0.3,
        capture_radius=0.25,
        sea_surface_z=0.0,
        contact_clearance=0.05,
        preferred_clearance=0.1,
    )
    outcome = planner.plan(
        initial_position=(0.0, 0.0, -1.0),
        initial_velocity=(4.0, 0.0, 0.0),
        initial_acceleration=(0.0, 0.0, 0.0),
        target_state_at_time=lambda horizon: (
            (3.0 + 2.0 * horizon, 0.0, -0.1),
            (2.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        ),
    )
    assert outcome.plan is not None

    message = plan_to_message(
        outcome.plan,
        mission_id=2,
        plan_id=6,
        prediction_sequence_id=4,
        trajectory_start_stamp=10.10,
        planning_started_stamp=10.02,
        generated_stamp=10.04,
        target_state_source='simulation_truth',
        contact_stamp=12.0,
        remaining_t_go=1.96,
        terminal_mode=False,
        planned_capture_margin=0.20,
    )

    assert message.mission_id == 2
    assert message.plan_id == 6
    assert message.prediction_sequence_id == 4
    assert (
        message.source_stamp.sec
        + message.source_stamp.nanosec * 1e-9
    ) == pytest.approx(10.10)
    assert message.planner_type == 'MINCO_T3_FAST'
    assert message.piece_count == len(message.segments) == 3
    assert sum(
        segment.duration.sec + segment.duration.nanosec * 1e-9
        for segment in message.segments
    ) == pytest.approx(message.trajectory_duration)
    assert all(len(segment.coefficients) == 18 for segment in message.segments)
    assert message.contact_stamp.sec == 12
    assert message.selected_t_go == pytest.approx(outcome.plan.duration)
    assert message.remaining_t_go == pytest.approx(1.96)
    assert not message.terminal_mode
    assert message.planned_capture_margin == pytest.approx(0.20)
