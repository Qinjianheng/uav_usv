"""Exercise acquisition-driven planning with bounded worker start rates."""

from types import SimpleNamespace

import pytest
from uav_usv_interfaces.msg import MissionState, PredictedTargetPoint
from uav_usv_interfaces.msg import TargetPrediction

from uav_control.guidance.intercept_planner_node import InterceptPlannerNode
from uav_control.guidance.intercept_planner_node import seconds_to_time
from uav_control.guidance.planner_pipeline import ContactTimeSchedule
from uav_control.guidance.planner_pipeline import LatestRequestSlot
from uav_control.guidance.planner_pipeline import PlanningRequestPolicy
from uav_control.guidance.planner_pipeline import UavKinematicState


class WorkerRecorder:
    """Record worker boundary requests without invoking ROS or a solver."""

    def __init__(self, clock):
        """Keep the request submission clock."""
        self.clock = clock
        self.starts = []

    def submit(self, callback, request):
        """Capture the request actually dispatched to the worker."""
        self.starts.append((self.clock[0], request))
        return SimpleNamespace(done=lambda: False)


def make_node(clock):
    """Configure a real planner wrapper around a recording worker boundary."""
    node = object.__new__(InterceptPlannerNode)
    node._ros_seconds = lambda: clock[0]
    node.intercept_requested = True
    node.mission_state = MissionState.FAR_GUIDANCE
    node.mission_id = 1
    node.planning_cycle_id = 1
    node.frame_id = 'local_ned'
    node.maximum_input_age = 0.125
    node.planning_period = 0.2
    node.terminal_planning_period = 0.1
    node.last_worker_started_stamp = None
    node.future = None
    node.ready_job = None
    node.latest_prediction = None
    node.latest_uav = UavKinematicState(
        clock[0], (0, 0, -5), (0, 0, 0), (0, 0, 0),
    )
    node.active_plan_reference = None
    node.last_submitted_key = None
    node.request_slot = LatestRequestSlot()
    node.contact_schedule = ContactTimeSchedule()
    node.request_policy = PlanningRequestPolicy()
    node.worker_executor = WorkerRecorder(clock)
    return node


def prediction(now, sequence):
    """Build a prediction with 20 ms acquisition age and an explicit epoch."""
    message = TargetPrediction()
    message.valid = True
    message.frame_id = 'local_ned'
    message.source = 'tracking'
    message.mission_id = 1
    message.sequence_id = sequence
    message.observation_stamp = seconds_to_time(now - 0.02)
    message.source_stamp = seconds_to_time(now - 0.01)
    message.valid_until = seconds_to_time(now + 0.105)
    message.samples = [PredictedTargetPoint()]
    message.samples[0].velocity.x = 4.0
    return message


def test_prediction_arrival_starts_without_waiting_for_bad_timer_phase():
    """Start from the new input instead of leaving it idle for a timer."""
    clock = [10.113]
    node = make_node(clock)
    node.prediction_callback(prediction(clock[0], 1))
    assert len(node.worker_executor.starts) == 1
    started, request = node.worker_executor.starts[0]
    assert started == clock[0]
    assert request.observation_stamp == pytest.approx(10.093)
    assert request.prediction.sequence_id == 1


def test_normal_prediction_callbacks_obey_five_hertz_worker_budget():
    """Do not turn a 20 Hz prediction stream into a 20 Hz solver stream."""
    clock = [10.0]
    node = make_node(clock)
    for index in range(20):
        clock[0] = 10.0 + 0.05 * index
        node.future = None
        node.prediction_callback(prediction(clock[0], index + 1))
    starts = [stamp for stamp, request in node.worker_executor.starts]
    assert len(starts) == 5
    assert all(b - a >= 0.2 - 1e-8 for a, b in zip(starts, starts[1:]))


def test_busy_worker_does_not_queue_a_stale_handover_boundary():
    """Build the eventual request from current UAV and newest prediction."""
    clock = [10.0]
    node = make_node(clock)
    node.prediction_callback(prediction(clock[0], 1))
    clock[0] = 10.25
    node.prediction_callback(prediction(clock[0], 2))
    assert len(node.worker_executor.starts) == 1
    node.future = None
    clock[0] = 10.30
    node.latest_uav = UavKinematicState(
        10.299, (2, 0, -5), (4, 0, 0), (0, 0, 0),
    )
    node.prediction_callback(prediction(clock[0], 3))
    assert len(node.worker_executor.starts) == 2
    request = node.worker_executor.starts[-1][1]
    assert request.prediction.sequence_id == 3
    assert request.trajectory_start_stamp == 10.299
    assert request.uav.position == (2, 0, -5)


def test_timers_do_not_resubmit_cached_prediction_after_completion():
    """Wait for new predictor input rather than recycling a cached snapshot."""
    clock = [10.0]
    node = make_node(clock)
    node.prediction_callback(prediction(clock[0], 1))
    node.future = None
    clock[0] = 10.25
    node.planning_timer_callback()
    node.terminal_planning_timer_callback()
    assert len(node.worker_executor.starts) == 1


def test_missed_contact_does_not_freeze_reacquired_far_guidance_forever():
    """Retry from a new contact after recovery, lock, and FAR_GUIDANCE."""
    clock = [12.0]
    node = make_node(clock)
    node.contact_schedule.accept_plan(10.0, 1.0)
    node.published_plan_references = {7: object()}
    node.prediction_callback(prediction(clock[0], 1))
    assert len(node.worker_executor.starts) == 1
    request = node.worker_executor.starts[0][1]
    assert request.contact_stamp is None
    assert request.planning_cycle_id == 2
    assert node.contact_schedule.contact_stamp is None
    assert not node.published_plan_references


def test_expired_contact_stays_frozen_until_execution_enters_recovery():
    """Do not replace a terminal trajectory inside the final freeze window."""
    clock = [11.01]
    node = make_node(clock)
    node.mission_state = MissionState.TERMINAL_MINCO
    node.contact_schedule.accept_plan(10.0, 1.0)
    node.prediction_callback(prediction(clock[0], 1))
    assert not node.worker_executor.starts
    assert node.contact_schedule.contact_stamp == 11.0


def test_terminal_callbacks_obey_ten_hertz_and_freeze_window():
    """Keep terminal rate limits and stop submissions inside 300 ms."""
    clock = [10.0]
    node = make_node(clock)
    node.mission_state = MissionState.TERMINAL_MINCO
    node.contact_schedule.accept_plan(10.0, 1.0)
    for index in range(17):
        clock[0] = 10.0 + 0.05 * index
        node.future = None
        node.prediction_callback(prediction(clock[0], index + 1))
    starts = [stamp for stamp, request in node.worker_executor.starts]
    assert len(starts) == 7
    assert all(b - a >= 0.1 - 1e-8 for a, b in zip(starts, starts[1:]))
    assert starts[-1] <= 10.6 + 1e-8
