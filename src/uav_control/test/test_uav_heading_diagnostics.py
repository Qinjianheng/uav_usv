"""Independent evaluator pose diagnostics preserve acquisition brackets."""

import math
from types import SimpleNamespace

import pytest

from uav_control.evaluation.gazebo_entity_pose import TimedPoseQuery
from uav_control.evaluation.uav_heading_diagnostics import (
    GazeboUavPoseTracker, heading_diagnostics,
)


def tracker():
    result = GazeboUavPoseTracker()
    result.add_clock_anchor(10., 1000., 1000.01, 50.)
    result.add_clock_anchor(10.1, 1000.1, 1000.11, 50.1)
    return result


def observation(yaw=0.1):
    return SimpleNamespace(
        valid=True, geometry_diagnostics_enabled=True,
        interpolated_attitude_w=math.cos(yaw / 2),
        interpolated_attitude_x=0., interpolated_attitude_y=0.,
        interpolated_attitude_z=math.sin(yaw / 2),
        interpolated_uav_x=1., interpolated_uav_y=2.,
        interpolated_uav_z=-3., target_range=10.,
        position=SimpleNamespace(x=11., y=2., z=-3.),
    )


def test_model_pose_is_interpolated_at_acquisition_with_raw_bracket():
    result = tracker()
    assert result.add_pose(10., (2., 1., 3.), (1., 0., 0., 0.), 1000.11)
    assert result.add_pose(10.1, (4., 3., 5.),
                           (math.cos(.1), 0., 0., math.sin(.1)), 1000.11)
    query = result.query_at(1000.05)
    assert query.status == 'INTERPOLATED'
    assert query.value[:3] == pytest.approx((3., 2., 4.))
    assert query.left_sim_stamp == 10.
    assert query.right_sim_stamp == 10.1
    assert query.fraction == pytest.approx(.5)
    assert query.value[3:] == pytest.approx(
        (math.cos(.05), 0., 0., math.sin(.05)))
    assert result.query_at(1000.2).status == 'AFTER_HISTORY'


def test_clock_bracket_wait_does_not_use_receipt_stamp():
    result = tracker()
    assert not result.add_pose(10.15, (2., 1., 3.),
                               (1., 0., 0., 0.), 1000.11)
    assert result.query_at(1000.15).value is None
    result.add_clock_anchor(10.2, 1000.2, 1000.21, 50.2)
    assert result.query_at(1000.15).value[:3] == (2., 1., 3.)
    assert result.last_mapped_stamp == pytest.approx(1000.15)


def test_pending_pose_expires_without_extrapolation():
    result = tracker()
    result.add_pose(10.15, (2., 1., 3.), (1., 0., 0., 0.), 1000.11)
    result.add_clock_anchor(10.3, 1000.3, 1000.31, 50.3)
    assert result.query_at(1000.15).value is None
    assert result.pending_pose_timeout_count == 1


def test_clock_reset_clears_pose_history_and_pending_queue():
    result = tracker()
    result.add_pose(10.05, (2., 1., 3.), (1., 0., 0., 0.), 1000.11)
    result.add_pose(10.15, (2., 1., 3.), (1., 0., 0., 0.), 1000.11)
    assert not result.add_clock_anchor(1., 1000.2, 1000.21, 50.2)
    assert result.query_at(1000.05).status == 'EMPTY'
    assert result.reset_count == 1
    assert not result.pending_poses


def test_bad_quaternion_rejected_and_exact_model_name_only():
    result = tracker()
    assert not result.add_pose(10.05, (2., 1., 3.), (0., 0., 0., 0.), 1000.11)
    assert result.last_status == 'UAV_POSE_INVALID'
    assert result.matches_model('x500_mono_cam_0')
    assert not result.matches_model('x500_mono_cam_01')
    assert not result.matches_model('x500_mono_cam_0::base_link')


def test_heading_uses_uav_model_flu_enu_reference_not_target_direction():
    # ENU heading pi/2 maps to NED heading zero.
    query = TimedPoseQuery(
        (2., 1., 3., math.sqrt(.5), 0., 0., math.sqrt(.5)),
        'EXACT', 1000., 10., 10., 1000., 1000., 0., 0.)
    row = heading_diagnostics(observation(), query, error_ned=(.05, 1., 0.))
    assert row['reference_heading'] == pytest.approx(0., abs=1e-12)
    assert row['px4_heading'] == pytest.approx(.1)
    assert row['delta_yaw'] == pytest.approx(.1)
    assert row['predicted_lateral_error'] == pytest.approx(10 * math.sin(.1))
    assert row['actual_lateral_error'] == pytest.approx(1.)
    assert row['heading_reference_source'] == 'gazebo_uav_model_pose'
    assert row['heading_diagnostics_status'] == 'VALID'
    assert row['uav_reference_position_x'] == 1.
    assert row['uav_reference_position_y'] == 2.
    assert row['uav_reference_position_z'] == -3.


@pytest.mark.parametrize('geometry', (False, True))
def test_unbracketed_reference_never_creates_heading(geometry):
    message = observation()
    message.geometry_diagnostics_enabled = geometry
    row = heading_diagnostics(
        message, TimedPoseQuery(None, 'AFTER_HISTORY', 1000.))
    assert row['heading_diagnostics_status'] != 'VALID'
    assert math.isnan(row['reference_heading'])
    assert row['uav_pose_status'] == 'AFTER_HISTORY'


def test_geometry_disabled_does_not_use_zero_default_quaternion():
    message = observation()
    message.geometry_diagnostics_enabled = False
    query = TimedPoseQuery((2., 1., 3., 1., 0., 0., 0.), 'EXACT', 1000.)
    row = heading_diagnostics(message, query)
    assert row['heading_diagnostics_status'] == 'GEOMETRY_DISABLED'
    assert math.isnan(row['px4_heading'])


def test_attitude_counterfactual_holds_px4_origin_instead_of_model_origin():
    message = observation()
    for prefix, value in (
        ('center_camera', (10., 0., 0.)),
        ('camera_translation', (0., 0., 0.)),
    ):
        for axis, number in zip(('x', 'y', 'z'), value):
            setattr(message, prefix + '_' + axis, number)
    message.camera_pitch_down = 0.
    message.target_reference_z_offset = 0.
    query = TimedPoseQuery(
        (500., 600., 700., math.sqrt(.5), 0., 0., math.sqrt(.5)),
        'EXACT', 1000.)
    row = heading_diagnostics(
        message, query, entity_center_ned=(11., 2., -3.))
    assert row['attitude_counterfactual_status'] == 'PX4_POSITION_HELD_FIXED'
    assert row['attitude_counterfactual_error_x'] == pytest.approx(0.)
    assert row['attitude_counterfactual_error_y'] == pytest.approx(0.)
    assert row['attitude_counterfactual_error_z'] == pytest.approx(0.)
    assert row['uav_position_reference_status'] == (
        'MODEL_ORIGIN_FRAME_NOT_ESTABLISHED')


def test_disabled_reference_is_explicit_and_has_no_heading():
    row = heading_diagnostics(observation(), None)
    assert not row['gazebo_uav_diagnostics_enabled']
    assert row['heading_diagnostics_status'] == 'DISABLED'
    assert row['heading_reference_source'] == 'none'
    assert math.isnan(row['reference_heading'])


def test_dedicated_yaml_changes_only_evaluator_and_geometry_flags():
    from pathlib import Path
    import yaml

    config = (Path(__file__).resolve().parents[2]
              / 'uav_usv_bringup/config')
    baseline = yaml.safe_load((config / 'baseline.yaml').read_text())
    dedicated = yaml.safe_load(
        (config / 'visual_geometry_diagnostics.yaml').read_text())
    dedicated['rgbd_target_localizer']['ros__parameters'][
        'geometry_diagnostics_enabled'] = False
    params = dedicated['intercept_evaluator_node']['ros__parameters']
    params['gazebo_entity_diagnostics_enabled'] = False
    assert params.pop('gazebo_uav_diagnostics_enabled') is True
    assert params.pop('gazebo_uav_entity_name') == 'x500_mono_cam_0'
    assert dedicated == baseline
