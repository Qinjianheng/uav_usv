"""Message conversion tests for the independent interception planner."""

import ast
import inspect
from collections import deque
from types import SimpleNamespace

import pytest
from px4_msgs.msg import VehicleLocalPosition
from uav_usv_interfaces.msg import ControllerDiagnostic, MissionState
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
from uav_control.guidance.planner_pipeline import LatestRequestSlot
from uav_control.guidance.planner_pipeline import PlannerRequest
from uav_control.guidance.planner_pipeline import PredictionSample
from uav_control.guidance.planner_pipeline import PredictionSeries
from uav_control.guidance.planner_pipeline import UavKinematicState
from uav_control.control.trajectory_tracker_node import trajectory_from_message
from uav_control.control.trajectory_tracking import TrackerKinematicState
from uav_control.control.trajectory_tracking import TrajectoryRejectReason
from uav_control.control.trajectory_tracking import TrajectoryTrackerCore
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


def run_publish_test(
    terminal_mode,
    latest_prediction,
    plan_duration=1.2,
    locked_contact_unreachable=False,
    mission_state=0,
    committed_contact=True,
    diagnostics=None,
):
    """Run one completed planner job through the real publication gate."""
    prediction = make_publish_test_prediction(sequence_id=4)
    schedule = ContactTimeSchedule()
    if committed_contact:
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
        contact_stamp=11.0 if committed_contact else None,
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
        mission_state=mission_state,
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
        approach_time_sync_tolerance=0.35,
        approach_reserve_clearance=0.20,
        planner=SimpleNamespace(
            sea_surface_z=0.0,
            response_delay=0.15,
            effective_vertical_braking_acceleration=2.5,
            maximum_vertical_speed=4.0,
        ),
    )
    job = PlannerJobResult(
        request=request,
        outcome=FastPlanningOutcome(
            plan=make_publish_test_plan(duration=plan_duration),
            failure=FastPlanningFailure.NONE,
            diagnostics=diagnostics or PlannerDiagnostics(),
        ),
        planning_started_stamp=10.01,
        generated_stamp=10.03,
        compute_time=0.02,
    )
    object.__setattr__(
        job,
        'locked_contact_unreachable',
        bool(locked_contact_unreachable),
    )

    planner_node_module.InterceptPlannerNode._publish_job(node, job)

    return node, schedule, trajectory_pub, diagnostic_pub


def test_far_guidance_publishes_only_after_terminal_admission_is_ready():
    waiting = PlannerDiagnostics(
        horizontal_min_time=1.0,
        vertical_min_time=0.5,
    )
    _, _, waiting_trajectories, waiting_diagnostics = run_publish_test(
        terminal_mode=False,
        latest_prediction=make_publish_test_prediction(sequence_id=5),
        mission_state=MissionState.FAR_GUIDANCE,
        committed_contact=False,
        diagnostics=waiting,
    )

    assert waiting_trajectories.messages == []
    assert waiting_diagnostics.messages[-1].result == (
        PlannerDiagnostic.RESULT_IDLE
    )
    assert not waiting_diagnostics.messages[-1].terminal_admission
    assert waiting_diagnostics.messages[-1].terminal_admission_reason == (
        'HORIZONTAL_NOT_READY'
    )

    ready = PlannerDiagnostics(
        horizontal_min_time=0.7,
        vertical_min_time=0.5,
    )
    _, _, ready_trajectories, ready_diagnostics = run_publish_test(
        terminal_mode=False,
        latest_prediction=make_publish_test_prediction(sequence_id=5),
        mission_state=MissionState.FAR_GUIDANCE,
        committed_contact=False,
        diagnostics=ready,
    )

    assert len(ready_trajectories.messages) == 1
    assert ready_diagnostics.messages[-1].result == (
        PlannerDiagnostic.RESULT_SUCCESS
    )
    assert ready_diagnostics.messages[-1].terminal_admission


def test_recovery_to_far_guidance_starts_new_contact_cycle_same_mission():
    node = object.__new__(planner_node_module.InterceptPlannerNode)
    node.mission_id = 2
    node.mission_state = MissionState.TERMINAL_MINCO
    node.intercept_requested = True
    node.planning_cycle_id = 4
    node.request_slot = LatestRequestSlot()
    node.last_submitted_key = ('old',)
    node.contact_schedule = ContactTimeSchedule()
    node.contact_schedule.accept_plan(10.0, 1.0)
    node.request_policy = planner_node_module.PlanningRequestPolicy(
        terminal_state=MissionState.TERMINAL_MINCO,
    )
    node.latest_prediction = make_publish_test_prediction(sequence_id=6)
    node.latest_uav = UavKinematicState(
        stamp=10.5,
        position=(0.0, 0.0, -1.0),
        velocity=(0.0, 0.0, 0.0),
        acceleration=(0.0, 0.0, 0.0),
    )
    node.request_slot.submit(SimpleNamespace(planning_cycle_id=4))

    def transition(state, completed=False):
        message = MissionState()
        message.mission_id = 2
        message.state = state
        message.state_name = str(state)
        message.intercept_requested = True
        message.completed = completed
        node.mission_callback(message)

    transition(MissionState.PLAN_RECOVERY)
    assert node.contact_schedule.contact_stamp == pytest.approx(11.0)
    recovery_cycle = node.planning_cycle_id
    transition(MissionState.SAFE_WAIT)
    assert node.contact_schedule.contact_stamp == pytest.approx(11.0)
    transition(MissionState.FAR_GUIDANCE)

    assert node.planning_cycle_id > recovery_cycle
    assert node.contact_schedule.contact_stamp is None
    assert node.request_slot.take() is None
    assert node.last_submitted_key is None
    decision = node.request_policy.decide(
        MissionState.FAR_GUIDANCE,
        remaining_t_go=None,
    )
    assert decision.submit and not decision.terminal_mode
    request = node._current_request(decision)
    assert request.contact_stamp is None
    assert not request.terminal_mode
    assert request.planning_cycle_id == node.planning_cycle_id


def test_tracker_ack_commits_only_matching_pending_candidate():
    node = object.__new__(planner_node_module.InterceptPlannerNode)
    node.mission_id = 2
    node.planning_cycle_id = 5
    node.contact_schedule = ContactTimeSchedule()
    node.contact_schedule.accept_plan(10.0, 1.0)
    node.contact_schedule.propose(8, 11.8, 5, 10.05)

    stale = ControllerDiagnostic()
    stale.mission_id = 2
    stale.attempted_plan_id = 7
    stale.status = 'PLAN_ACCEPTED'
    node.controller_diagnostic_callback(stale)
    assert node.contact_schedule.contact_stamp == pytest.approx(11.0)

    accepted = ControllerDiagnostic()
    accepted.mission_id = 2
    accepted.attempted_plan_id = 8
    accepted.status = 'PLAN_ACCEPTED'
    node.controller_diagnostic_callback(accepted)
    assert node.contact_schedule.contact_stamp == pytest.approx(11.8)
    assert node.contact_schedule.committed_plan_id == 8


def test_candidate_reject_then_accept_commits_only_real_tracker_acceptance():
    schedule = ContactTimeSchedule()
    schedule.accept_plan(10.0, 1.0)
    planner_node = object.__new__(planner_node_module.InterceptPlannerNode)
    planner_node.mission_id = 2
    planner_node.planning_cycle_id = 5
    planner_node.contact_schedule = schedule
    tracker = TrajectoryTrackerCore()

    def candidate(plan_id):
        return trajectory_from_message(plan_to_message(
            make_publish_test_plan(duration=1.2),
            mission_id=2,
            plan_id=plan_id,
            prediction_sequence_id=4,
            trajectory_start_stamp=10.0,
            planning_started_stamp=10.01,
            generated_stamp=10.03,
            target_state_source='simulation_truth',
            contact_stamp=11.2,
            remaining_t_go=1.2,
        ))

    schedule.propose(8, 11.2, 5, 10.03)
    rejected = tracker.accept(
        candidate(8),
        TrackerKinematicState(
            stamp=10.05,
            position=(10.0, 0.0, -1.0),
            velocity=(0.0, 0.0, 0.0),
        ),
        mission_id=2,
        target_endpoint=(1.2, 0.0, -0.1),
    )
    assert rejected == TrajectoryRejectReason.STATE_POSITION_MISMATCH
    rejection = ControllerDiagnostic()
    rejection.mission_id = 2
    rejection.attempted_plan_id = 8
    rejection.status = 'PLAN_REJECTED'
    planner_node.controller_diagnostic_callback(rejection)
    assert schedule.contact_stamp == pytest.approx(11.0)

    accepted_candidate = candidate(9)
    sample = accepted_candidate.sample_at_ros_time(10.05)
    schedule.propose(9, 11.2, 5, 10.04)
    accepted = tracker.accept(
        accepted_candidate,
        TrackerKinematicState(
            stamp=10.05,
            position=sample.position,
            velocity=sample.velocity,
        ),
        mission_id=2,
        target_endpoint=(1.2, 0.0, -0.1),
    )
    assert accepted == TrajectoryRejectReason.NONE
    confirmation = ControllerDiagnostic()
    confirmation.mission_id = 2
    confirmation.attempted_plan_id = 9
    confirmation.status = 'PLAN_ACCEPTED'
    planner_node.controller_diagnostic_callback(confirmation)
    assert schedule.contact_stamp == pytest.approx(11.2)
    assert schedule.committed_plan_id == 9


def test_old_async_cycle_result_cannot_publish_after_recovery():
    node, schedule, trajectory_pub, diagnostic_pub = run_publish_test(
        terminal_mode=True,
        latest_prediction=make_publish_test_prediction(sequence_id=5),
    )
    # Re-run one equivalent worker result after a recovery cycle invalidation.
    request = PlannerRequest(
        mission_id=2,
        prediction=make_publish_test_prediction(sequence_id=4),
        uav=UavKinematicState(
            stamp=10.0,
            position=(0.0, 0.0, -1.0),
            velocity=(0.0, 0.0, 0.0),
            acceleration=(0.0, 0.0, 0.0),
        ),
        trajectory_start_stamp=10.0,
        contact_stamp=11.0,
        terminal_mode=True,
        minimum_duration=0.30,
        planning_cycle_id=4,
    )
    node.planning_cycle_id = 5
    node.mission_id = 2
    node.intercept_requested = True
    node.mission_state = MissionState.FAR_GUIDANCE
    before = len(trajectory_pub.messages)
    job = PlannerJobResult(
        request=request,
        outcome=FastPlanningOutcome(
            plan=make_publish_test_plan(duration=1.2),
            failure=FastPlanningFailure.NONE,
            diagnostics=PlannerDiagnostics(),
        ),
        planning_started_stamp=10.01,
        generated_stamp=10.03,
        compute_time=0.02,
    )

    planner_node_module.InterceptPlannerNode._publish_job(node, job)

    assert len(trajectory_pub.messages) == before
    assert schedule.contact_stamp == pytest.approx(11.0)
    assert diagnostic_pub.messages[-1].rejection_stage == (
        'MISSION_OR_REQUEST_INVALIDATED'
    )


def test_terminal_freeze_is_not_unlocked_before_recovery_to_far_guidance():
    node = object.__new__(planner_node_module.InterceptPlannerNode)
    node.mission_id = 2
    node.mission_state = MissionState.TERMINAL_MINCO
    node.intercept_requested = True
    node.planning_cycle_id = 4
    node.request_slot = LatestRequestSlot()
    node.last_submitted_key = None
    node.contact_schedule = ContactTimeSchedule(freeze_time=0.30)
    node.contact_schedule.accept_plan(10.0, 1.0)
    node._ros_seconds = lambda: 10.75
    node.request_policy = planner_node_module.PlanningRequestPolicy(
        terminal_freeze_time=0.30,
        terminal_state=MissionState.TERMINAL_MINCO,
    )

    message = MissionState()
    message.mission_id = 2
    message.state = MissionState.TERMINAL_MINCO
    message.intercept_requested = True
    message.completed = False
    node.mission_callback(message)

    decision = node.request_policy.decide(
        node.mission_state,
        node.contact_schedule.remaining_t_go(node._ros_seconds()),
    )
    assert not decision.submit
    assert node.contact_schedule.contact_stamp == pytest.approx(11.0)


@pytest.mark.parametrize(
    'state',
    (
        MissionState.PLAN_RECOVERY,
        MissionState.SAFE_WAIT,
        MissionState.CAPTURE,
        MissionState.FAILURE,
        MissionState.ABORTED,
    ),
)
def test_recovery_and_terminal_states_never_submit_planning(state):
    node = object.__new__(planner_node_module.InterceptPlannerNode)
    node.intercept_requested = True
    node.mission_state = state
    node._current_request = lambda _decision: pytest.fail(
        'planning request must not be generated in this state'
    )

    node._planning_tick(terminal_tick=False)
    node._planning_tick(terminal_tick=True)


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
    assert prediction.samples[0].position == pytest.approx(
        (4.0, 0.0, 0.0)
    )


def test_terminal_freeze_uses_contact_stamp_minus_current_ros_time():
    """Catch freeze decisions that subtract the prediction source time."""
    observed = []

    class RecordingPolicy:
        def decide(self, mission_state, remaining_t_go=None):
            observed.append(remaining_t_go)
            return SimpleNamespace(
                terminal_mode=True,
                submit=False,
            )

    node = object.__new__(
        planner_node_module.InterceptPlannerNode
    )
    node.intercept_requested = True
    node.latest_prediction = SimpleNamespace(source_stamp=10.0)
    node.contact_schedule = ContactTimeSchedule(
        freeze_time=0.30
    )
    node.contact_schedule.accept_plan(10.0, 1.0)
    node.request_policy = RecordingPolicy()
    node.mission_state = 7
    node._ros_seconds = lambda: 10.75

    node._planning_tick(terminal_tick=True)

    assert observed == pytest.approx([0.25])


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
        _ros_seconds=lambda: 10.15,
    )

    planner_node_module.InterceptPlannerNode._run_request(node, request)

    assert planner.target_at_quarter_second[0] == pytest.approx(
        (10.35, 0.0, -0.1)
    )
    # Prediction ends at 11.0, so only 0.9 s remains from trajectory start 10.1.
    assert planner.maximum_duration_override == pytest.approx(0.9)


def test_terminal_locked_contact_does_not_cap_available_search_horizon():
    """Keep old contact preferred while permitting a later feasible contact."""
    class CapturingPlanner:
        minimum_duration = 0.10
        maximum_duration = 4.0

        def plan(self, **kwargs):
            self.preferred_duration = kwargs['preferred_duration']
            self.minimum_duration_override = kwargs[
                'minimum_duration_override'
            ]
            self.maximum_duration_override = kwargs[
                'maximum_duration_override'
            ]
            return FastPlanningOutcome(
                plan=None,
                failure=FastPlanningFailure.CAPTURE_GEOMETRY,
                diagnostics=PlannerDiagnostics(),
            )

    prediction = PredictionSeries(
        mission_id=2,
        sequence_id=4,
        source_stamp=10.0,
        valid_until=10.125,
        samples=(
            PredictionSample(
                0.0,
                (0.0, 0.0, -0.1),
                (1.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            ),
            PredictionSample(
                4.0,
                (4.0, 0.0, -0.1),
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
        _ros_seconds=lambda: 10.15,
    )

    planner_node_module.InterceptPlannerNode._run_request(node, request)

    assert planner.preferred_duration == pytest.approx(0.80)
    assert planner.minimum_duration_override == pytest.approx(0.80)

    # Prediction extends to 14.0; MINCO starts at 10.1.
    # Full usable horizon is therefore 3.9 s, not 0.8 + 0.3.
    assert planner.maximum_duration_override == pytest.approx(3.90)


def test_terminal_reschedule_beyond_old_point_three_limit_can_publish():
    """Permit a later feasible terminal contact validated at its new time."""
    node, schedule, trajectory_pub, diagnostic_pub = run_publish_test(
        terminal_mode=True,
        latest_prediction=make_publish_test_prediction(sequence_id=5),
        plan_duration=1.8,
    )

    assert len(trajectory_pub.messages) == 1
    assert schedule.contact_stamp == pytest.approx(11.0)
    assert schedule.pending_contact_stamp == pytest.approx(11.8)
    assert diagnostic_pub.messages[-1].result == (
        PlannerDiagnostic.RESULT_SUCCESS
    )
    assert node.plan_id == 1


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
    assert diagnostic_pub.messages[-1].rejection_stage == (
        'TARGET_PREDICTION_SHIFT'
    )
    assert diagnostic_pub.messages[-1].target_prediction_shift > 0.5
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


def test_nonterminal_unreachable_contact_can_publish_forward_candidate():
    """Reachability proof opens one pending recovery, not contact drift."""
    node, schedule, trajectory_pub, diagnostic_pub = run_publish_test(
        terminal_mode=False,
        latest_prediction=make_publish_test_prediction(sequence_id=5),
        locked_contact_unreachable=True,
    )

    assert len(trajectory_pub.messages) == 1
    assert schedule.contact_stamp == pytest.approx(11.0)
    assert schedule.pending_contact_stamp == pytest.approx(11.2)
    assert diagnostic_pub.messages[-1].candidate_published
    assert diagnostic_pub.messages[-1].contact_delay == pytest.approx(0.2)
    assert diagnostic_pub.messages[-1].contact_recovery_reason == (
        'CONTACT_UNREACHABLE_RECOVERY'
    )
    assert node.plan_id == 1


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
