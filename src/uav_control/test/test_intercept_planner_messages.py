"""Message conversion tests for the independent interception planner."""

import ast
import inspect
from collections import deque
from types import SimpleNamespace

import pytest
from px4_msgs.msg import VehicleLocalPosition
from uav_usv_interfaces.msg import PlannerDiagnostic
from uav_usv_interfaces.msg import PredictedTargetPoint, TargetPrediction

from uav_control.guidance.fast_minco_planner import FastMincoPlanner
from uav_control.guidance.fast_minco_planner import FastPlanningFailure
from uav_control.guidance.fast_minco_planner import FastPlanningOutcome
from uav_control.guidance.finite_horizon_intercept_planner import InterceptPlan
from uav_control.guidance.finite_horizon_intercept_planner import PlannerDiagnostics
from uav_control.guidance.intercept_planner_node import PlannerJobResult
from uav_control.guidance.intercept_planner_node import plan_to_message
from uav_control.guidance.intercept_planner_node import prediction_from_message
from uav_control.guidance.intercept_planner_node import uav_state_from_message
from uav_control.guidance.minco_trajectory import MincoS3Trajectory
from uav_control.guidance.planner_pipeline import ContactTimeSchedule
from uav_control.guidance.planner_pipeline import PlannerRequest
from uav_control.guidance.planner_pipeline import PredictionSample
from uav_control.guidance.planner_pipeline import PredictionSeries
from uav_control.guidance.planner_pipeline import UavKinematicState
import uav_control.guidance.intercept_planner_node as planner_node_module


class RecordingPublisher:
    """Collect messages at the ROS publisher boundary."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def make_publish_test_plan(duration=1.2):
    """Return one complete MINCO plan for publication behavior tests."""
    trajectory = MincoS3Trajectory(
        start_position=(0.0, 0.0, -1.0),
        start_velocity=(0.0, 0.0, 0.0),
        start_acceleration=(0.0, 0.0, 0.0),
        end_position=(duration, 0.0, -0.1),
        end_velocity=(1.0, 0.0, 0.0),
        end_acceleration=(0.0, 0.0, 0.0),
        durations=(duration,),
    )
    return InterceptPlan(
        axes=(),
        duration=duration,
        closing_speed=0.0,
        target_position=(duration, 0.0, -0.1),
        target_velocity=(1.0, 0.0, 0.0),
        target_acceleration=(0.0, 0.0, 0.0),
        maximum_horizontal_speed=1.0,
        maximum_vertical_speed=1.0,
        maximum_horizontal_acceleration=1.0,
        maximum_vertical_acceleration=1.0,
        cost=0.0,
        minco_trajectory=trajectory,
        planner_type='MINCO_T3_FAST',
        piece_durations=(duration,),
    )


def make_publish_test_prediction(sequence_id, shifted_at_candidate=False):
    """Return a prediction equal at 11.0 and optionally shifted at 11.2."""
    candidate_x = 2.2 if shifted_at_candidate else 1.2
    final_x = 3.0 if shifted_at_candidate else 2.0
    return PredictionSeries(
        mission_id=2,
        sequence_id=sequence_id,
        source_stamp=10.0,
        valid_until=12.0,
        samples=(
            PredictionSample(
                0.0, (0.0, 0.0, -0.1), (1.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            ),
            PredictionSample(
                1.0, (1.0, 0.0, -0.1), (1.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            ),
            PredictionSample(
                1.2, (candidate_x, 0.0, -0.1), (1.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            ),
            PredictionSample(
                2.0, (final_x, 0.0, -0.1), (1.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            ),
        ),
        source='simulation_truth',
    )


def run_publish_test(terminal_mode, latest_prediction):
    """Run one completed planner job through the real publication gate."""
    prediction = make_publish_test_prediction(sequence_id=4)
    schedule = ContactTimeSchedule(terminal_max_reschedule_delay=0.30)
    schedule.accept_plan(source_stamp=10.0, selected_t_go=1.0)
    request = PlannerRequest(
        mission_id=2,
        prediction=prediction,
        uav=UavKinematicState(
            stamp=10.0,
            position=(0.0, 0.0, -1.0),
            velocity=(0.0, 0.0, 0.0),
            acceleration=(0.0, 0.0, 0.0),
        ),
        trajectory_start_stamp=10.0,
        contact_stamp=11.0,
        terminal_mode=terminal_mode,
        minimum_duration=0.30 if terminal_mode else 1.0,
    )
    trajectory_pub = RecordingPublisher()
    diagnostic_pub = RecordingPublisher()
    node = SimpleNamespace(
        _ros_seconds=lambda: 10.05,
        maximum_input_age=0.125,
        endpoint_tolerance=0.5,
        latest_prediction=latest_prediction,
        contact_schedule=schedule,
        mission_state=0,
        terminal_time_threshold=1.0,
        planned_capture_radius=0.35,
        completed_plan_count=0,
        completion_times=deque(maxlen=100),
        _completion_frequency=lambda: 0.0,
        plan_id=0,
        request_slot=SimpleNamespace(replaced_request_count=0),
        diagnostic_pub=diagnostic_pub,
        trajectory_pub=trajectory_pub,
        frame_id='local_ned',
    )
    job = PlannerJobResult(
        request=request,
        outcome=FastPlanningOutcome(
            plan=make_publish_test_plan(),
            failure=FastPlanningFailure.NONE,
            diagnostics=PlannerDiagnostics(),
        ),
        planning_started_stamp=10.01,
        generated_stamp=10.03,
        compute_time=0.02,
    )

    planner_node_module.InterceptPlannerNode._publish_job(node, job)

    return node, schedule, trajectory_pub, diagnostic_pub


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
        maximum_duration = 4.0

        def plan(self, **kwargs):
            self.maximum_duration_override = kwargs[
                'maximum_duration_override'
            ]
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
        contact_stamp=10.90,
        terminal_mode=True,
        minimum_duration=0.30,
    )
    planner = CapturingPlanner()
    node = SimpleNamespace(
        planner=planner,
        maximum_input_age=0.20,
        hard_deadline_seconds=1.0,
        terminal_max_reschedule_delay=0.30,
        _ros_seconds=lambda: 10.15,
    )

    planner_node_module.InterceptPlannerNode._run_request(node, request)

    assert planner.target_at_quarter_second[0] == pytest.approx(
        (10.35, 0.0, -0.1)
    )
    assert planner.maximum_duration_override == pytest.approx(1.1)


def test_terminal_reschedule_validates_shift_at_candidate_contact():
    """Catch stale-target validation using the old locked contact time."""
    node, schedule, trajectory_pub, diagnostic_pub = run_publish_test(
        terminal_mode=True,
        latest_prediction=make_publish_test_prediction(
            sequence_id=5,
            shifted_at_candidate=True,
        ),
    )

    assert trajectory_pub.messages == []
    assert schedule.contact_stamp == pytest.approx(11.0)
    assert diagnostic_pub.messages[-1].result == (
        PlannerDiagnostic.RESULT_FAILURE
    )
    assert node.plan_id == 1


def test_normal_plan_cannot_publish_endpoint_after_locked_contact():
    """Catch a normal replacement contradicting its locked contact metadata."""
    _, schedule, trajectory_pub, diagnostic_pub = run_publish_test(
        terminal_mode=False,
        latest_prediction=make_publish_test_prediction(sequence_id=5),
    )

    assert trajectory_pub.messages == []
    assert schedule.contact_stamp == pytest.approx(11.0)
    assert diagnostic_pub.messages[-1].result == (
        PlannerDiagnostic.RESULT_FAILURE
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
