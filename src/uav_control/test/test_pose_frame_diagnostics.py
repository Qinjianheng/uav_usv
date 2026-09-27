"""Independent checks for Gazebo ENU/FLU and PX4 NED/FRD orientation."""

import importlib
import math

import numpy as np
import pytest

from uav_control.evaluation.gazebo_entity_pose import TimedPoseHistory


def _quaternion_from_gazebo_euler(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def test_spawn_yaw_maps_gazebo_body_to_px4_identity():
    diagnostics = importlib.import_module(
        'uav_control.evaluation.pose_frame_diagnostics'
    )
    gazebo_q = _quaternion_from_gazebo_euler(0.0, 0.0, math.pi / 2)
    rotation = diagnostics.gazebo_model_rotation_ned_frd(gazebo_q)

    assert rotation == pytest.approx(np.eye(3), abs=1e-12)
    assert diagnostics.rotation_residual_rpy(
        rotation, np.eye(3),
    ) == pytest.approx(
        (0.0, 0.0, 0.0), abs=1e-12,
    )


def test_full_gazebo_orientation_maps_roll_pitch_and_yaw_signs():
    diagnostics = importlib.import_module(
        'uav_control.evaluation.pose_frame_diagnostics'
    )
    gazebo_q = _quaternion_from_gazebo_euler(0.12, 0.08, math.pi / 2 + 0.2)
    rotation = diagnostics.gazebo_model_rotation_ned_frd(gazebo_q)

    # Gazebo roll stays positive; Gazebo pitch/yaw reverse in NED/FRD.
    assert diagnostics.rotation_residual_rpy(
        rotation, np.eye(3),
    ) == pytest.approx(
        (0.12, -0.08, -0.2), abs=1e-10,
    )


def test_model_pose_interpolation_uses_slerp_and_own_sample_bracket():
    diagnostics = importlib.import_module(
        'uav_control.evaluation.pose_frame_diagnostics'
    )
    history = TimedPoseHistory(
        maximum_age=2.0, interpolator=diagnostics.interpolate_model_pose,
    )
    left = (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
    right = (2.0, 0.0, 0.0, *(_quaternion_from_gazebo_euler(
        0.0, 0.0, math.pi,
    )))
    assert history.add(10.0, 1000.0, left)
    assert history.add(10.2, 1000.2, right)
    assert history.add(10.4, 1000.4, right)
    query = history.query_at(1000.1)

    assert query.left_sim_stamp == pytest.approx(10.0)
    assert query.right_sim_stamp == pytest.approx(10.2)
    assert query.value[:3] == pytest.approx((1.0, 0.0, 0.0))
    rotated = diagnostics.quaternion_rotation(query.value[3:])
    assert rotated @ np.array((1.0, 0.0, 0.0)) == pytest.approx((
        0.0, 1.0, 0.0,
    ), abs=1e-10)


def test_query_metadata_uses_query_bracket_and_reports_failure():
    diagnostics = importlib.import_module(
        'uav_control.evaluation.pose_frame_diagnostics'
    )
    history = TimedPoseHistory(maximum_age=2.0)
    history.add(10.0, 1000.0, (0.0,))
    history.add(10.2, 1000.2, (2.0,))
    history.add(10.4, 1000.4, (4.0,))

    metadata = diagnostics.query_metadata('entity', history.query_at(1000.1))
    assert metadata['entity_left_sim_stamp'] == pytest.approx(10.0)
    assert metadata['entity_right_sim_stamp'] == pytest.approx(10.2)
    assert metadata['entity_fraction'] == pytest.approx(0.5)
    assert metadata['entity_status'] == 'INTERPOLATED'
    failed = diagnostics.query_metadata('entity', history.query_at(1000.5))
    assert failed['entity_status'] == 'AFTER_HISTORY'
    assert math.isnan(failed['entity_left_sim_stamp'])


def test_px4_attitude_interpolation_preserves_reset_evidence():
    diagnostics = importlib.import_module(
        'uav_control.evaluation.pose_frame_diagnostics'
    )
    history = TimedPoseHistory(
        maximum_age=2.0, interpolator=diagnostics.interpolate_px4_attitude,
    )
    history.add(10.0, 1000.0, (1.0, 0.0, 0.0, 0.0, 1))
    history.add(10.2, 1000.2, (0.0, 0.0, 0.0, 1.0, 2))
    query = history.query_at(1000.1)
    assert query.value[:4] == pytest.approx((
        math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5),
    ))
    assert diagnostics.query_crosses_reset(query, 4)


def test_invalid_online_attitude_has_no_rotation_comparison():
    diagnostics = importlib.import_module(
        'uav_control.evaluation.pose_frame_diagnostics'
    )
    assert math.isnan(diagnostics.rotation_angle_between_quaternions(
        (0.0, 0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0),
    ))
    assert diagnostics.rotation_angle_between_quaternions(
        (1.0, 0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0),
    ) == pytest.approx(0.0)


def test_reconstructed_model_orientation_preserves_full_rpy_residual():
    diagnostics = importlib.import_module(
        'uav_control.evaluation.pose_frame_diagnostics'
    )
    px4_quaternion = _quaternion_from_gazebo_euler(0.04, -0.03, 0.1)
    residual = (0.07, -0.05, 0.2)

    model_quaternion = diagnostics.model_quaternion_from_residual(
        px4_quaternion, residual,
    )
    px4_rotation = diagnostics.quaternion_rotation(px4_quaternion)
    model_rotation = diagnostics.quaternion_rotation(model_quaternion)
    assert diagnostics.rotation_residual_rpy(
        model_rotation, px4_rotation,
    ) == pytest.approx(residual, abs=1e-12)


def test_four_pose_counterfactuals_use_online_camera_transform():
    diagnostics = importlib.import_module(
        'uav_control.evaluation.pose_frame_diagnostics'
    )
    px4_quaternion = (1.0, 0.0, 0.0, 0.0)
    model_quaternion = _quaternion_from_gazebo_euler(
        0.0, 0.0, math.pi / 2,
    )
    camera_vector = (10.0, 0.0, 0.0)
    px4_position = np.array((0.0, 0.0, 0.0))
    model_position = np.array((1.0, 2.0, 0.0))
    positions = diagnostics.four_pose_counterfactuals(
        camera_vector, px4_position, model_position,
        px4_quaternion, model_quaternion,
        (0.35, 0.0, 0.19), 0.20944, 0.42,
    )
    for key, position, attitude in (
        ('A', px4_position, px4_quaternion),
        ('B', model_position, model_quaternion),
        ('C', model_position, px4_quaternion),
        ('D', px4_position, model_quaternion),
    ):
        assert positions[key] == pytest.approx(
            diagnostics.camera_target_to_local_ned(
                camera_vector, position, attitude,
                (0.35, 0.0, 0.19), 0.20944, 0.42,
            )
        )
    assert positions['C'] - positions['A'] == pytest.approx(model_position)
