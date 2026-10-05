"""Low-altitude intercept retains fresh visual yaw; search/recovery still holds."""

from types import SimpleNamespace

import pytest

from test_strict_visual_control import control_node
from uav_control.mission.target_visibility import VisibilityDecision
from uav_usv_interfaces.msg import MissionState


@pytest.mark.parametrize('phase', [
    MissionState.MINCO_READY, MissionState.MINCO_TRACKING, MissionState.TERMINAL_MINCO,
])
def test_locked_intercept_continues_visual_yaw_below_search_height(phase):
    node, state = control_node(height=.7)
    node.mission_state = phase
    node.intercept_requested = True
    node.visibility_decision = VisibilityDecision('TRACKING', True, True, .4, 0)
    node.visibility.last_valid_image_bearing = .4
    node.latest_kf_message = None  # Isolate the visual gate's feedback fallback.
    command = SimpleNamespace()
    node._final_yaw(command, state, .05)
    assert command.yawspeed == pytest.approx(.4)
    assert node.yaw_owner == 'VISION'


@pytest.mark.parametrize('phase,requested,locked,visible,recovering', [
    (MissionState.FOLLOW, False, True, True, False),
    (MissionState.TERMINAL_MINCO, False, True, True, False),
    (MissionState.TERMINAL_MINCO, True, False, True, False),
    (MissionState.TERMINAL_MINCO, True, True, False, False),
    (MissionState.TERMINAL_MINCO, True, True, True, True),
    (MissionState.CAPTURE, True, True, True, False),
])
def test_low_altitude_yaw_stays_inhibited_without_active_visual_intercept(
        phase, requested, locked, visible, recovering):
    node, state = control_node(height=.7)
    node.mission_state = phase
    node.intercept_requested = requested
    node.safe_recovery_latched = recovering
    node.visibility_decision = VisibilityDecision('TRACKING', locked, visible, .4, 0)
    node.visibility.last_valid_image_bearing = .4
    command = SimpleNamespace()
    node._final_yaw(command, state, .05)
    assert command.yawspeed == 0.
    assert node.yaw_owner == 'HOLD'
