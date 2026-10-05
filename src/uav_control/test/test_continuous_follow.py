"""One FOLLOW phase refines RGB direction into position following."""

from dataclasses import replace
import math

import pytest
from uav_usv_interfaces.msg import MissionState

from test_bearing_approach import approach_node
from test_follow_wait import runnable_follow_node
from uav_control.mission.mission_manager import MissionManagerCore, MissionPhase
from uav_control.mission.target_visibility import TargetVisibilityState


def flying_follow():
    core = MissionManagerCore()
    core.set_flight_ready(True)
    assert core.handle_command('X', 1.)
    assert core.mark_takeoff_complete(2.)
    return core


def test_takeoff_enters_follow_without_depth_or_centered_lock():
    core = flying_follow()
    assert core.phase == MissionPhase.FOLLOW
    assert not core.target_locked


def test_follow_loss_does_not_cycle_acquire_lock_and_follow():
    core = flying_follow()
    for state, locked in [('REACQUIRE', False), ('TARGET_LOCK', True),
                          ('SAFE_WAIT', False), ('TARGET_LOCK', True)]:
        core.observe_visibility(core.mission_id, state, locked, 3.)
        assert core.phase == MissionPhase.FOLLOW


def test_early_y_is_queued_while_follow_continues_until_precise_lock():
    core = flying_follow()
    assert core.handle_command('Y', 2.1)
    assert core.intercept_requested and core.phase == MissionPhase.FOLLOW
    core.observe_visibility(core.mission_id, 'REACQUIRE', False, 2.2)
    assert core.phase == MissionPhase.FOLLOW
    core.observe_visibility(core.mission_id, 'TARGET_LOCK', True, 2.3)
    assert core.phase == MissionPhase.FAR_GUIDANCE


def test_fresh_kf_follow_does_not_require_centered_rgb_lock():
    node, _ = runnable_follow_node()
    node.visibility = TargetVisibilityState()
    for stamp in (10.1, 10.15, 10.2):
        node.visibility.observe(stamp, .4, True, stamp, position_valid=True)
    node.timer_callback()
    command, status = node.diagnostics[-1]
    assert not node.visibility_decision.locked
    assert status == 'FOLLOW' and command.mode == 'VELOCITY'
    assert not node.bearing_approach_active
    assert node.latest_target_state is not None


def test_rgb_follow_and_refined_kf_share_continuous_velocity_owner():
    node, state = approach_node()
    node.mission_state = MissionState.FOLLOW
    node.mission_state_name = 'FOLLOW'
    node.intercept_requested = True  # pending Y must not stop coarse FOLLOW
    node.timer_callback()
    first, status = node.diagnostics[-1]
    assert status == 'FOLLOW' and node.bearing_approach_active
    from test_strict_visual_control import target
    node.latest_state = replace(state, stamp=10.25)
    node._ros_seconds = lambda: 10.25
    node.latest_kf_message = target(now=10.25, source=10.25)
    node.latest_kf_message.position.x = -20.
    node.latest_kf_message.position.y = 20.
    node.visibility.observe(10.25, .3, True, 10.25, position_valid=True)
    node.timer_callback()
    second, status = node.diagnostics[-1]
    assert status == 'FOLLOW' and not node.bearing_approach_active
    assert math.hypot(second.velocity[0] - first.velocity[0],
                      second.velocity[1] - first.velocity[1]) <= (
        node.flight_guidance.maximum_horizontal_acceleration * .05 + 1e-8
    )
    assert first.velocity[2] == pytest.approx(second.velocity[2])
