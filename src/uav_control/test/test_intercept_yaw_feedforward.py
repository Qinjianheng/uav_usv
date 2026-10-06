"""Keep a rotating target visible using fresh KF line-of-sight feedforward."""

from dataclasses import replace
from types import SimpleNamespace

import pytest
from uav_usv_interfaces.msg import MissionState

from test_strict_visual_control import control_node, target
from uav_control.mission.target_visibility import VisibilityDecision


def intercept_node():
    """Set a centred target moving sideways at 2 m range."""
    node, state = control_node(now=10.2, height=.7)
    node.intercept_requested = True
    node.mission_state = MissionState.TERMINAL_MINCO
    node.visibility_decision = VisibilityDecision('TRACKING', True, True, 0., 0)
    node.visibility.last_valid_image_bearing = 0.
    node.latest_kf_message = target(now=10.2, source=10.18)
    node.latest_kf_message.position = SimpleNamespace(x=2., y=0., z=0.)
    node.latest_kf_message.velocity = SimpleNamespace(x=0., y=1., z=0.)
    state = replace(state, position=(0., 0., -.7), velocity=(0., 0., 0.))
    return node, state


def test_centred_moving_line_of_sight_does_not_wait_for_pixel_error():
    """Request 1 m/s divided by 2 m of yaw feedforward at zero image error."""
    node, state = intercept_node()
    command = SimpleNamespace()
    node._final_yaw(command, state, .05)
    assert command.yawspeed == pytest.approx(.5)
    assert command.yaw == pytest.approx(.025)
    assert node.yaw_owner == 'VISION'


def test_y_preparation_also_centres_a_rotating_line_of_sight():
    """FAR_GUIDANCE must not wait for angular lag before turning the camera."""
    node, state = intercept_node()
    node.mission_state = MissionState.FAR_GUIDANCE
    state = replace(state, position=(0., 0., -1.5))
    command = SimpleNamespace()
    node._final_yaw(command, state, .05)
    assert command.yawspeed == pytest.approx(.5)


def test_partial_mask_centroid_does_not_steer_away_from_fitted_target_center():
    """Clipped pixels at +0.4 rad must not turn a camera with a centred sphere."""
    from uav_usv_interfaces.msg import TargetObservation
    from test_strict_visual_control import stamp

    node, state = intercept_node()
    state = replace(state, velocity=(0., 1., 0.))  # No LOS rotation.
    node.visibility.last_valid_image_bearing = .4
    observation = TargetObservation()
    observation.stamp.sec = stamp(10.2).sec
    observation.stamp.nanosec = stamp(10.2).nanosec
    observation.frame_id = 'local_ned'
    observation.valid = True
    observation.center_camera_x = 1.
    observation.center_camera_z = -.2
    node.observation_callback(observation)
    command = SimpleNamespace()
    node._final_yaw(command, state, .05)
    assert command.yawspeed == 0.


@pytest.mark.parametrize('source', [9., 10.21])
def test_center_feedback_cannot_replace_fresh_rgb_with_old_or_future_geometry(source):
    from uav_usv_interfaces.msg import TargetObservation
    from test_strict_visual_control import stamp

    node, state = intercept_node()
    state = replace(state, velocity=(0., 1., 0.))
    node.visibility.last_valid_image_bearing = .1
    observation = TargetObservation()
    observation.stamp.sec = stamp(source).sec
    observation.stamp.nanosec = stamp(source).nanosec
    observation.frame_id = 'local_ned'
    observation.valid = True
    observation.center_camera_x = 1.
    node.observation_callback(observation)
    command = SimpleNamespace()
    node._final_yaw(command, state, .05)
    assert command.yawspeed == pytest.approx(.1)


def test_feedforward_uses_relative_velocity_including_uav_motion():
    """Cancel angular motion when UAV and target translate together."""
    node, state = intercept_node()
    state = replace(state, velocity=(0., 1., 0.))
    node.visibility.last_valid_image_bearing = .1
    command = SimpleNamespace()
    node._final_yaw(command, state, .05)
    assert command.yawspeed == pytest.approx(.1)


@pytest.mark.parametrize('source', [9.0, 10.21, 0.0])
def test_stale_future_or_missing_acquisition_cannot_supply_feedforward(source):
    """Retain image feedback without adding an untrusted velocity estimate."""
    node, state = intercept_node()
    node.latest_kf_message = target(now=10.2, source=source)
    node.visibility.last_valid_image_bearing = .1
    command = SimpleNamespace()
    node._final_yaw(command, state, .05)
    assert command.yawspeed == pytest.approx(.1)


def test_negative_feedforward_and_feedback_obey_existing_yaw_limit():
    """Keep the original 1 rad/s final-command envelope."""
    node, state = intercept_node()
    node.latest_kf_message.velocity.y = -10.
    node.visibility.last_valid_image_bearing = -.2
    command = SimpleNamespace()
    node._final_yaw(command, state, .05)
    assert command.yawspeed == -node.max_observation_yaw_rate


def test_follow_does_not_gain_intercept_feedforward():
    """Apply this change only after Y enters an active visual MINCO phase."""
    node, state = intercept_node()
    state = replace(state, position=(0., 0., -5.))
    node.mission_state = MissionState.FOLLOW
    node.intercept_requested = False
    node.visibility.last_valid_image_bearing = .1
    command = SimpleNamespace()
    node._final_yaw(command, state, .05)
    assert command.yawspeed == pytest.approx(.1)


def test_zero_horizontal_range_has_finite_yaw_feedback():
    """Avoid an undefined angular velocity at zero horizontal range."""
    node, state = intercept_node()
    node.latest_kf_message.position.x = 0.
    node.visibility.last_valid_image_bearing = .1
    command = SimpleNamespace()
    node._final_yaw(command, state, .05)
    assert command.yawspeed == pytest.approx(.1)
