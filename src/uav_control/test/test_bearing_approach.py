"""Far RGB approach moves without depth but never overrides loss or recovery."""

from dataclasses import replace
import math
from types import SimpleNamespace

import pytest

from test_follow_wait import runnable_follow_node
from test_strict_visual_control import stamp, target
from uav_control.control.flight_guidance import FlightGuidanceCore, FlightKinematicState
from uav_control.mission.target_visibility import TargetVisibilityState
from uav_control.evaluation.run_metrics import RunMetricAccumulator
from uav_control.control.trajectory_tracker_node import TrajectoryTrackerNode
from uav_control.control.trajectory_tracking import TrajectoryRejectReason
from test_run_logging import node_fixture, phase, csv_rows
from uav_usv_interfaces.msg import MissionState


def approach_node(bearing=.05):
    node, state = runnable_follow_node()
    node.mission_state = MissionState.TARGET_ACQUIRE
    node.mission_state_name = 'TARGET_ACQUIRE'
    node.latest_kf_message = None
    node.intercept_requested = False
    node.bearing_approach_enabled = True
    node.bearing_approach_speed = 6.
    node.bearing_approach_active = False
    node.visibility = TargetVisibilityState()
    for value in (10., 10.1, 10.2):
        node.visibility.observe(value, bearing, True, value)
    node.offboard_modes = []
    node._publish_offboard_mode = lambda timestamp, velocity_control: (
        node.offboard_modes.append(velocity_control))
    return node, state


def test_rgb_alone_approaches_and_logs_without_claiming_position_lock():
    node, _ = approach_node()
    node.timer_callback()
    command, status = node.diagnostics[-1]
    assert status == 'BEARING_APPROACH'
    assert command.mode == 'VELOCITY'
    assert command.velocity == pytest.approx((.149812539, .007496875, 0.))
    assert math.hypot(*command.velocity[:2]) == pytest.approx(.15)  # 3m/s² * 50ms
    assert node.offboard_modes[-1]
    assert all(math.isnan(v) for v in node.setpoint_pub.messages[-1].position)
    assert node.yaw_owner == 'VISION'
    assert not node.visibility_decision.locked
    assert node.latest_target_state is None


@pytest.mark.parametrize('condition', [
    'stale', 'invalid', 'behind_camera', 'two_frames', 'disabled', 'terminal',
    'intercept', 'recovery', 'low_height', 'takeoff', 'safe_wait',
])
def test_rgb_approach_inhibited_on_loss_or_other_control_authority(condition):
    node, state = approach_node()
    if condition == 'stale':
        node.latest_state = replace(state, stamp=10.326)
        node._ros_seconds = lambda: 10.326
    elif condition in ('invalid', 'behind_camera'):
        node.visibility.observe(10.201, math.pi/2. if condition == 'behind_camera' else 0.,
                                condition != 'invalid', 10.201)
        node.latest_state = replace(state, stamp=10.201)
        node._ros_seconds = lambda: 10.201
    elif condition == 'two_frames':
        node.visibility.reset()
        for value in (10.1, 10.2):
            node.visibility.observe(value, .05, True, value)
    elif condition == 'disabled':
        node.bearing_approach_enabled = False
    elif condition == 'terminal':
        node.terminal_mode_latched = True
    elif condition == 'intercept':
        node.intercept_requested = True
    elif condition == 'recovery':
        node.safe_recovery_latched = True
        node.latest_state = replace(state, position=(1., 2., -.2))
    elif condition == 'low_height':
        node.latest_state = replace(state, position=(1., 2., -2.))
    elif condition == 'takeoff':
        node.mission_state = MissionState.TAKEOFF
        node.mission_state_name = 'TAKEOFF'
    elif condition == 'safe_wait':
        node.mission_state = MissionState.SAFE_WAIT
    node.timer_callback()
    assert node.diagnostics[-1][1] != 'BEARING_APPROACH'
    assert not node.bearing_approach_active


def test_approach_stop_brakes_then_resumes_without_position_mode():
    node, state = approach_node()
    node.timer_callback()
    node.visibility.observe(10.21, 0., False, 10.21)
    node._ros_seconds = lambda: 10.21
    node.latest_state = replace(state, stamp=10.21, position=(4., 2., -5.))
    node.timer_callback()
    assert node.diagnostics[-1][0].position == (4., 2., -5.)
    assert node.diagnostics[-1][0].velocity == (0., 0., 0.)
    assert node.diagnostics[-1][0].mode == 'VELOCITY'
    assert node.search_hold_position is None
    for value in (10.22, 10.27, 10.32):
        node.visibility.observe(value, .05, True, value)
    node._ros_seconds = lambda: 10.32
    node.latest_state = replace(state, stamp=10.32, velocity=(.4, 0., 0.))
    node.timer_callback()
    assert node.diagnostics[-1][1] == 'BEARING_APPROACH'
    assert node.diagnostics[-1][0].velocity[0] <= .4 + 3.*.1 + 1e-9


def test_stable_depth_lock_transfers_to_existing_follow():
    node, _ = approach_node()
    node.timer_callback()
    node.latest_kf_message = target(now=10.2, source=10.2)
    node.visibility.mark_observation(10.2, True, 10.2)
    node.mission_state = MissionState.FOLLOW
    node.mission_state_name = 'FOLLOW'
    node.timer_callback()
    assert node.diagnostics[-1][1] == 'FOLLOW'
    assert not node.bearing_approach_active
    assert node.visibility_decision.locked


def test_direction_and_speed_are_bounded_from_uav_heading_only():
    core = FlightGuidanceCore()
    state = FlightKinematicState((0., 0., -5.), (0., 0., 0.))
    for _ in range(80):
        command = core.bearing_approach(state, math.pi/2., 6., .05)
    assert command.velocity[0] == pytest.approx(0., abs=1e-12)
    assert command.velocity[1] == pytest.approx(6.)
    assert command.velocity[2] == 0.
    assert not command.far_guidance_available
    assert not command.takeoff_complete


def test_duplicate_bearing_frames_cannot_enable_approach():
    node, _ = approach_node()
    node.visibility.reset()
    for _ in range(3):
        node.bearing_callback(SimpleNamespace(
            stamp=stamp(10.2), raw_stamp=stamp(5.), valid=True, bearing=.05))
    node.timer_callback()
    assert node.diagnostics[-1][1] != 'BEARING_APPROACH'


def test_intentional_approach_is_not_reported_as_search_drift():
    metrics = RunMetricAccumulator(1.)
    metrics.observe(1., 'TARGET_ACQUIRE', position=(0., 0., -5.), control_status='SEARCH')
    metrics.observe(2., 'TARGET_ACQUIRE', position=(0., 0., -5.),
                    control_status='BEARING_APPROACH')
    metrics.observe(3., 'TARGET_ACQUIRE', position=(6., 0., -5.),
                    control_status='BEARING_APPROACH')
    metrics.observe(4., 'REACQUIRE', position=(12., 0., -5.), control_status='REACQUIRE')
    metrics.observe(5., 'REACQUIRE', position=(12.2, 0., -5.), control_status='REACQUIRE')
    summary = metrics.summary(6.)
    assert summary['bearing_approach_duration'] == 2.
    assert summary['search_xy_drift_max'] == pytest.approx(.2)


def test_approach_log_preserves_rgb_age_without_inventing_kf_age(tmp_path):
    node, _ = approach_node()
    node._ros_seconds = lambda: 10.22
    node.latest_state = replace(node.latest_state, stamp=10.22)
    node.timer_callback()
    command, status = node.diagnostics[-1]
    diagnostics = []
    node.diagnostic_pub = SimpleNamespace(publish=diagnostics.append)
    node.last_rejection = TrajectoryRejectReason.NONE
    TrajectoryTrackerNode._publish_diagnostic(node, 10.22, command, status, .001)
    message = diagnostics[-1]
    assert message.source_age == pytest.approx(.02)
    assert math.isinf(message.kf_state_age)
    assert math.isinf(message.last_valid_observation_age)
    evaluator = node_fixture(tmp_path)
    phase(evaluator, 'TAKEOFF', 10.)
    phase(evaluator, 'TARGET_ACQUIRE', 10.1)
    evaluator.clock_value = 10.22
    evaluator.controller_callback(message)
    evaluator.timer_callback()
    paths = evaluator.writer.paths
    evaluator._finalize_run('NODE_SHUTDOWN', 10.23)
    row = csv_rows(paths.csv_path)[-1]
    assert row['controller_status'] == 'BEARING_APPROACH'
    assert row['bearing_approach_active'] == 'True'
    assert float(row['bearing_age_at_control']) == pytest.approx(.02)
