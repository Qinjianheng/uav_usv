import math

import numpy as np
import pytest

from uav_control.perception.rgbd_target_localizer import (
    body_frd_to_ned_rotation,
    camera_target_to_local_ned,
    target_vector_from_rgbd,
)


def test_rgbd_center_pixel_recovers_forward_target_center():
    mask = np.zeros((4, 6), dtype=bool)
    mask[2, 3] = True
    depth = np.full((4, 6), math.inf, dtype=float)
    depth[2, 3] = 9.75

    vector = target_vector_from_rgbd(
        mask,
        depth,
        horizontal_fov=math.pi / 2.0,
        minimum_depth=0.2,
        maximum_depth=25.0,
        target_radius=0.25,
    )

    assert vector == pytest.approx((10.0, 0.0, 0.0))


def test_rgbd_projection_uses_camera_flu_image_signs():
    mask = np.zeros((3, 5), dtype=bool)
    mask[0, 4] = True
    depth = np.full((3, 5), math.inf, dtype=float)
    depth[0, 4] = 5.0

    vector = target_vector_from_rgbd(
        mask,
        depth,
        horizontal_fov=math.pi / 2.0,
        minimum_depth=0.2,
        maximum_depth=25.0,
    )

    assert vector[0] == pytest.approx(5.0)
    assert vector[1] < 0.0
    assert vector[2] > 0.0


def test_camera_pose_transform_compensates_mount_pitch_and_translation():
    pitch = math.radians(12.0)
    desired_body_flu = np.array((10.0, 2.0, -3.0))
    translation = np.array((0.35, 0.0, -0.05))
    relative_body = desired_body_flu - translation
    camera_vector = np.array((
        math.cos(pitch) * relative_body[0]
        - math.sin(pitch) * relative_body[2],
        relative_body[1],
        math.sin(pitch) * relative_body[0]
        + math.cos(pitch) * relative_body[2],
    ))

    position = camera_target_to_local_ned(
        camera_vector,
        uav_position_ned=(1.0, 4.0, -5.0),
        attitude_quaternion=(1.0, 0.0, 0.0, 0.0),
        camera_translation_flu=translation,
        camera_pitch_down=pitch,
        target_reference_z_offset=0.42,
    )

    assert position == pytest.approx((11.0, 2.0, -1.58))


def test_body_to_ned_rotation_applies_yaw():
    half_angle = math.pi / 4.0
    rotation = body_frd_to_ned_rotation((
        math.cos(half_angle),
        0.0,
        0.0,
        math.sin(half_angle),
    ))

    assert rotation @ np.array((10.0, 0.0, 0.0)) == pytest.approx(
        (0.0, 10.0, 0.0),
        abs=1.0e-9,
    )


def test_rgbd_localizer_rejects_missing_valid_depth():
    mask = np.ones((2, 2), dtype=bool)
    depth = np.full((2, 2), math.inf, dtype=float)

    assert target_vector_from_rgbd(
        mask,
        depth,
        horizontal_fov=1.74,
        minimum_depth=0.2,
        maximum_depth=25.0,
    ) is None
