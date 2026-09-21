import math
from types import SimpleNamespace

import numpy as np
import pytest
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition

from uav_control.perception import rgbd_target_localizer

from uav_control.perception.rgbd_target_localizer import (
    Px4RosClockMapper,
    TimestampedVectorHistory,
    body_frd_to_ned_rotation,
    camera_target_to_local_ned,
    due_data_timeout,
    synchronized_measurement_time,
    target_vector_from_rgbd,
    validated_sensor_stamp,
)


def test_missing_image_timeout_is_reported_at_a_bounded_rate():
    assert due_data_timeout(10.6, 10.0, -math.inf, 0.5)
    assert not due_data_timeout(10.7, 10.0, 10.6, 0.5)
    assert due_data_timeout(11.2, 10.0, 10.6, 0.5)


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


def test_image_callbacks_only_replace_latest_frames(monkeypatch):
    """Catch image decoding and red segmentation returning to DDS callbacks."""
    node = object.__new__(rgbd_target_localizer.RgbdTargetLocalizer)
    node.latest_color_message = None
    node.latest_depth_message = None
    node.color_time = -1.0
    node.depth_time = -1.0
    node.color_measurement_time = None
    node.depth_measurement_time = None
    node.data_timeout = 0.5
    node.image_clock_mapper = Px4RosClockMapper(0.05)
    node._ros_seconds = lambda: 10.0
    monkeypatch.setattr(
        rgbd_target_localizer,
        'red_pixel_mask',
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError('heavy color work ran in callback')
        ),
    )
    monkeypatch.setattr(
        rgbd_target_localizer,
        'decode_float32_depth',
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError('heavy depth work ran in callback')
        ),
    )
    stamp = SimpleNamespace(sec=2, nanosec=0)
    header = SimpleNamespace(stamp=stamp)
    color = SimpleNamespace(header=header)
    depth = SimpleNamespace(header=header)

    node.color_callback(color)
    node.depth_callback(depth)

    assert node.latest_color_message is color
    assert node.latest_depth_message is depth


def test_sensor_stamp_must_be_in_the_ros_clock_domain():
    assert validated_sensor_stamp(
        measurement_stamp=10.0,
        receipt_stamp=10.04,
        maximum_age=0.5,
    ) == pytest.approx(10.0)
    assert validated_sensor_stamp(
        measurement_stamp=2.0,
        receipt_stamp=10.0,
        maximum_age=0.5,
    ) is None
    assert validated_sensor_stamp(
        measurement_stamp=10.2,
        receipt_stamp=10.0,
        maximum_age=0.5,
    ) is None


def test_rgb_depth_pair_uses_acquisition_time_and_rejects_mismatch():
    assert synchronized_measurement_time(10.00, 10.04, 0.05) == (
        pytest.approx(10.02)
    )
    assert synchronized_measurement_time(10.00, 10.06, 0.05) is None


def test_pose_history_interpolates_at_image_acquisition_time():
    history = TimestampedVectorHistory(maximum_age=1.0)
    assert history.add(10.0, (0.0, 0.0, -5.0))
    assert history.add(10.2, (2.0, 4.0, -5.0))

    assert history.value_at(10.1) == pytest.approx((1.0, 2.0, -5.0))
    assert history.value_at(9.9) is None
    assert history.value_at(10.3) is None
    assert not history.add(10.1, (99.0, 99.0, 99.0))


def test_px4_clock_mapper_rejects_a_time_jump_instead_of_retiming_pose():
    mapper = Px4RosClockMapper(maximum_offset_jump=0.05)

    assert mapper.to_ros_time(
        2.0, receipt_ros_time=10.0
    ) == pytest.approx(10.0)
    assert mapper.to_ros_time(
        2.1, receipt_ros_time=10.1
    ) == pytest.approx(10.1)
    assert mapper.to_ros_time(1.0, receipt_ros_time=10.2) is None


def test_px4_callbacks_cache_pose_in_mapped_measurement_time():
    node = object.__new__(rgbd_target_localizer.RgbdTargetLocalizer)
    now = [10.0]
    node._ros_seconds = lambda: now[0]
    node.px4_clock_mapper = Px4RosClockMapper(maximum_offset_jump=0.05)
    node.position_history = TimestampedVectorHistory(maximum_age=1.0)
    node.attitude_history = TimestampedVectorHistory(maximum_age=1.0)
    node.uav_position = None
    node.uav_attitude = None
    node.uav_position_time = -math.inf
    node.uav_attitude_time = -math.inf

    first_position = VehicleLocalPosition()
    first_position.timestamp = 2_000_000
    first_position.x = 0.0
    first_position.y = 0.0
    first_position.z = -5.0
    node.position_callback(first_position)
    first_attitude = VehicleAttitude()
    first_attitude.timestamp = 2_000_000
    first_attitude.q = [1.0, 0.0, 0.0, 0.0]
    now[0] = 10.01
    node.attitude_callback(first_attitude)

    now[0] = 10.2
    second_position = VehicleLocalPosition()
    second_position.timestamp = 2_200_000
    second_position.x = 2.0
    second_position.y = 4.0
    second_position.z = -5.0
    node.position_callback(second_position)
    second_attitude = VehicleAttitude()
    second_attitude.timestamp = 2_200_000
    second_attitude.q = [1.0, 0.0, 0.0, 0.0]
    node.attitude_callback(second_attitude)

    assert node.position_history.value_at(10.1) == pytest.approx(
        (1.0, 2.0, -5.0)
    )
    assert node.attitude_history.value_at(10.1) == pytest.approx(
        (1.0, 0.0, 0.0, 0.0)
    )


def test_attitude_cache_normalizes_antipodal_quaternions():
    node = object.__new__(rgbd_target_localizer.RgbdTargetLocalizer)
    now = [10.0]
    node._ros_seconds = lambda: now[0]
    node.px4_clock_mapper = Px4RosClockMapper(maximum_offset_jump=0.05)
    node.attitude_history = TimestampedVectorHistory(maximum_age=1.0)
    node.uav_attitude = None
    node.uav_attitude_time = -math.inf

    first = VehicleAttitude()
    first.timestamp = 2_000_000
    first.q = [1.0, 0.0, 0.0, 0.0]
    node.attitude_callback(first)
    now[0] = 10.2
    second = VehicleAttitude()
    second.timestamp = 2_200_000
    second.q = [-1.0, 0.0, 0.0, 0.0]
    node.attitude_callback(second)

    assert node.attitude_history.value_at(10.1) == pytest.approx(
        (1.0, 0.0, 0.0, 0.0)
    )
