"""Same-frame target attribution and capture-field regression checks."""

from pathlib import Path
import math
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))
from vision_p6_target_frame_analysis import (  # noqa: E402
    frame_error_vectors, frame_mapping,
)
from vision_static_capture import pose_history_diagnostics  # noqa: E402
from vision_p6_representative_ray_analysis import (  # noqa: E402
    pixel_centers, ray_radius_center,
)
from uav_control.perception.rgbd_target_localizer import (  # noqa: E402
    camera_target_to_local_ned, local_ned_target_to_camera_flu,
)


def test_same_frame_truth_separates_local_yaw_from_camera_error():
    model_position = np.array((1.0, 2.0, -5.0))
    px4_position = np.array((1.02, 2.03, -5.01))
    yaw = 0.13
    px4_rotation = np.array(((math.cos(yaw), -math.sin(yaw), 0.0),
                             (math.sin(yaw), math.cos(yaw), 0.0),
                             (0.0, 0.0, 1.0)))
    model_rotation = np.eye(3)
    sphere = np.array((9.0, 3.0, -5.0))
    translation = (0.35, 0.0, 0.19)
    pitch = 0.20944
    offset = 0.42
    model_quaternion = (1.0, 0.0, 0.0, 0.0)
    px4_quaternion = (math.cos(yaw / 2), 0.0, 0.0,
                      math.sin(yaw / 2))
    camera = local_ned_target_to_camera_flu(
        sphere, model_position, model_quaternion, translation, pitch,
    )
    online = camera_target_to_local_ned(
        camera, px4_position, px4_quaternion, translation, pitch, offset,
    )
    result = frame_error_vectors(
        camera, camera, online, sphere, model_position, model_rotation,
        px4_position, px4_rotation, translation, pitch, offset,
    )
    assert np.linalg.norm(result['camera_error']) < 1e-10
    assert np.linalg.norm(result['gazebo_reconstruction_error']) < 1e-10
    assert np.linalg.norm(result['same_frame_error']) < 1e-10
    assert np.linalg.norm(result['cross_frame_error']) > 1.0
    np.testing.assert_allclose(
        result['cross_frame_error'],
        result['same_frame_error'] + result['frame_rotation_term']
        + result['frame_position_term'], atol=1e-10,
    )
    rotation, shift = frame_mapping(
        model_position, model_rotation, px4_position, px4_rotation,
    )
    np.testing.assert_allclose(rotation @ model_position + shift,
                               px4_position, atol=1e-10)


def test_capture_preserves_pose_history_diagnostic_fields():
    fields = {
        name: float(index) / 10
        for index, name in enumerate((
            'pose_history_start_stamp', 'pose_history_end_stamp',
            'position_history_start_stamp', 'position_history_end_stamp',
            'attitude_history_start_stamp', 'attitude_history_end_stamp',
            'position_source_stamp', 'position_mapped_stamp',
            'attitude_source_stamp', 'attitude_mapped_stamp',
        ), start=1)
    }
    message = SimpleNamespace(**fields)
    assert pose_history_diagnostics(message) == pytest.approx(fields)


def test_representative_rays_use_distinct_valid_and_red_centers():
    mask = np.array(((True, True, True), (False, True, False)))
    valid = np.array(((False, False, True), (False, True, False)))
    centers = pixel_centers(mask, valid, (10, 20, 13, 22))
    assert centers['B0_valid_depth_median'] == (11.5, 20.5)
    assert centers['B1_red_mask_median'] == (11.0, 20.0)
    assert centers['B1_red_mask_centroid'] == (11.0, 20.25)
    assert centers['B2_bbox_center'] == (11.0, 20.5)
    result = ray_radius_center(5.0, (12.0, 20.5),
                               (200.0, 200.0, 10.0, 20.0), 0.25)
    ray = np.array((1.0, -0.01, -0.0025))
    np.testing.assert_allclose(result, (5.0 + 0.25 / np.linalg.norm(ray))
                               * ray)
