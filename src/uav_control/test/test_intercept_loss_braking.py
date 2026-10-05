"""A Y request must not bypass bounded braking when fresh vision expires."""

from dataclasses import replace

import pytest

from test_strict_visual_control import control_node
from uav_usv_interfaces.msg import MissionState


@pytest.mark.parametrize('requested', [False, True])
def test_high_altitude_visual_loss_brakes_before_anchoring(requested):
    node, state = control_node()
    node.intercept_requested = requested
    node.mission_state = MissionState.REACQUIRE
    state = replace(state, velocity=(4., 0., 0.))
    node.flight_guidance.previous_velocity = state.velocity

    command = node._search_or_recovery_command(state, .05)

    assert command.mode == 'VELOCITY'
    assert command.velocity == pytest.approx((3.85, 0., 0.))
    assert node.search_hold_position is None
    assert not command.far_guidance_available
    assert node.tracker.active_trajectory is None


def test_requested_intercept_still_recovers_upward_near_sea():
    node, state = control_node(height=.7)
    node.intercept_requested = True
    node.mission_state = MissionState.REACQUIRE
    state = replace(state, velocity=(4., 0., .2))

    command = node._search_or_recovery_command(state, .05)

    assert node.safe_recovery_latched
    assert node.search_state == 'SAFE_RECOVERY'
    assert command.position[2] < state.position[2]
