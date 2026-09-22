import math
from types import SimpleNamespace

import numpy as np
import pytest
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from sensor_msgs.msg import Image

from uav_control.perception import rgbd_target_localizer

from uav_control.perception.rgbd_target_localizer import (
    GazeboImageClockMapper,
    Px4RosClockMapper,
    RgbDepthPairBuffer,
    TimestampedVectorHistory,
    body_frd_to_ned_rotation,
    camera_target_to_local_ned,
    due_data_timeout,
    pose_history_rejection_reason,
    quaternion_slerp,
    synchronized_measurement_time,
    target_vector_from_rgbd,
    validated_sensor_stamp,
)


def test_gazebo_clock_maps_sim_time_from_server_side_dual_clock_anchors():
    mapper = GazeboImageClockMapper()
    assert mapper.add_anchor(10.0, 1000.0, 1000.04, 50.0)
    assert mapper.add_anchor(10.1, 1000.2, 1000.25, 50.2)

    mapped = mapper.to_ros_time(10.05, now_ros_time=1000.25)

    assert mapped == pytest.approx(1000.1)
    assert mapper.last_status == 'MAPPED_INTERPOLATED'
    assert mapper.mapping_mode == 'GAZEBO_CLOCK_SYSTEM_INTERPOLATION'


def test_gazebo_mapping_does_not_follow_image_bridge_delay_variation():
    mapper = GazeboImageClockMapper()
    mapper.add_anchor(20.0, 2000.0, 2000.01, 60.0)
    mapper.add_anchor(20.2, 2000.2, 2000.22, 60.2)

    early_receipt_mapping = mapper.to_ros_time(20.1, 2000.22)
    late_receipt_mapping = mapper.to_ros_time(20.1, 2000.60)

    assert early_receipt_mapping == pytest.approx(2000.1)
    assert late_receipt_mapping == pytest.approx(2000.1)


def test_gazebo_clock_system_jump_invalidates_old_mapping():
    mapper = GazeboImageClockMapper(maximum_system_clock_step=0.1)
    mapper.add_anchor(5.0, 100.0, 100.01, 10.0)
    mapper.add_anchor(5.1, 100.1, 100.11, 10.1)
    assert mapper.to_ros_time(5.05, 100.11) == pytest.approx(100.05)

    accepted = mapper.add_anchor(5.2, 101.2, 101.21, 10.2)

    assert not accepted
    assert mapper.reset_count == 1
    assert mapper.to_ros_time(5.15, 101.21) is None
    assert mapper.last_status == 'SYSTEM_CLOCK_RESET'


def test_gazebo_clock_pause_resume_and_rewind_are_bounded():
    mapper = GazeboImageClockMapper()
    mapper.add_anchor(1.0, 50.0, 50.01, 1.0)
    mapper.add_anchor(1.1, 50.1, 50.11, 1.1)

    assert not mapper.add_anchor(1.1, 50.3, 50.31, 1.3)
    assert mapper.last_status == 'SIM_TIME_PAUSED'
    assert mapper.add_anchor(1.2, 50.4, 50.41, 1.4)
    assert mapper.to_ros_time(1.15, 50.41) == pytest.approx(50.25)

    assert not mapper.add_anchor(0.1, 50.5, 50.51, 1.5)
    assert mapper.reset_count == 1
    assert mapper.last_status == 'SIM_TIME_RESET'
    assert mapper.to_ros_time(1.15, 50.51) is None


def test_gazebo_image_time_without_trusted_clock_pair_is_rejected():
    mapper = GazeboImageClockMapper()

    assert mapper.to_ros_time(12.0, 100.0) is None
    assert mapper.last_status == 'CLOCK_REFERENCE_UNAVAILABLE'


def test_gazebo_clock_rejects_stale_and_future_mapped_images_separately():
    mapper = GazeboImageClockMapper(maximum_reference_age=0.5)
    mapper.add_anchor(10.0, 100.0, 100.01, 1.0)
    mapper.add_anchor(10.2, 100.2, 100.21, 1.2)

    assert mapper.to_ros_time(10.1, 101.0) is None
    assert mapper.last_status == 'CLOCK_REFERENCE_STALE'
    assert mapper.to_ros_time(10.3, 100.21) is None
    assert mapper.last_status == 'IMAGE_AFTER_CLOCK_REFERENCE'


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
    node.image_pair_buffer = RgbDepthPairBuffer(0.05)
    node.pending_image_pairs = __import__('collections').deque(maxlen=8)
    node.last_pair_rejection = ''
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
        pytest.approx(10.00)
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


def test_quaternion_history_uses_shortest_path_slerp():
    history = TimestampedVectorHistory(
        maximum_age=1.0,
        interpolator=quaternion_slerp,
    )
    assert history.add(10.0, (1.0, 0.0, 0.0, 0.0))
    assert history.add(10.2, (0.0, 0.0, 0.0, 1.0))

    halfway = history.value_at(10.1)

    assert halfway == pytest.approx(
        (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))
    )
    assert np.linalg.norm(halfway) == pytest.approx(1.0)
    assert quaternion_slerp(
        (1.0, 0.0, 0.0, 0.0),
        (-1.0, 0.0, 0.0, 0.0),
        0.5,
    ) == pytest.approx((1.0, 0.0, 0.0, 0.0))


@pytest.mark.parametrize(
    'stamp, position_range, attitude_range, expected',
    (
        (10.1, None, (10.0, 10.2), 'POSITION_HISTORY_EMPTY'),
        (10.1, (10.0, 10.2), None, 'ATTITUDE_HISTORY_EMPTY'),
        (
            9.9, (10.0, 10.2), (9.8, 10.2),
            'POSITION_TIMESTAMP_BEFORE_HISTORY',
        ),
        (
            9.9, (9.8, 10.2), (10.0, 10.2),
            'ATTITUDE_TIMESTAMP_BEFORE_HISTORY',
        ),
        (
            10.3, (10.0, 10.2), (10.0, 10.4),
            'POSITION_TIMESTAMP_AFTER_HISTORY',
        ),
        (
            10.3, (10.0, 10.4), (10.0, 10.2),
            'ATTITUDE_TIMESTAMP_AFTER_HISTORY',
        ),
    ),
)
def test_pose_history_rejection_reason_is_stream_specific(
    stamp,
    position_range,
    attitude_range,
    expected,
):
    assert pose_history_rejection_reason(
        stamp,
        position_range,
        attitude_range,
    ) == expected


def test_px4_clock_mapper_rejects_a_time_jump_instead_of_retiming_pose():
    mapper = Px4RosClockMapper(maximum_offset_jump=0.05)

    assert mapper.to_ros_time(2.0, receipt_ros_time=10.0) is None
    assert mapper.to_ros_time(2.02, receipt_ros_time=10.02) is None
    assert mapper.to_ros_time(2.05, receipt_ros_time=10.05) is None
    assert mapper.to_ros_time(
        2.1, receipt_ros_time=10.1
    ) == pytest.approx(10.1)
    assert mapper.to_ros_time(1.0, receipt_ros_time=10.2) is None


def test_shared_px4_mapper_tolerates_cross_topic_interleaving():
    mapper = Px4RosClockMapper(
        maximum_offset_jump=0.05,
        calibration_samples=4,
    )
    samples = (
        ('position', 10.050, 100.070),
        ('attitude', 10.040, 100.075),
        ('position', 10.100, 100.120),
        ('attitude', 10.090, 100.125),
    )

    mapped = [
        mapper.to_ros_time(source, receipt, stream_name=stream)
        for stream, source, receipt in samples
    ]

    assert mapper.reset_count == 0
    assert mapped[-1] == pytest.approx(100.110)


def test_px4_reset_requires_consistent_regression_from_both_streams():
    mapper = Px4RosClockMapper(
        maximum_offset_jump=0.05,
        calibration_samples=4,
    )
    for stream, source in (
        ('position', 10.00),
        ('attitude', 10.00),
        ('position', 10.05),
        ('attitude', 10.05),
    ):
        mapper.to_ros_time(source, source + 90.0, stream_name=stream)
    before = mapper.reset_count

    assert mapper.to_ros_time(
        1.00, 100.10, stream_name='position'
    ) is None
    assert mapper.reset_count == before
    assert mapper.to_ros_time(
        0.99, 100.11, stream_name='attitude'
    ) is None
    assert mapper.reset_count == before + 1


def test_ros_clock_rollback_resets_shared_mapping_immediately():
    mapper = Px4RosClockMapper(calibration_samples=1)
    assert mapper.to_ros_time(
        10.0, 100.0, stream_name='position'
    ) == pytest.approx(100.0)

    assert mapper.to_ros_time(
        10.1, 99.0, stream_name='attitude'
    ) is None
    assert mapper.reset_count == 1
    assert mapper.last_status == 'ROS_CLOCK_RESET'


def test_single_delayed_px4_message_does_not_reset_shared_mapping():
    mapper = Px4RosClockMapper(
        maximum_offset_jump=0.05,
        calibration_samples=2,
    )
    assert mapper.to_ros_time(
        10.0, 100.0, stream_name='position'
    ) is None
    assert mapper.to_ros_time(
        10.0, 100.01, stream_name='attitude'
    ) == pytest.approx(100.0)
    before = mapper.reset_count

    assert mapper.to_ros_time(
        10.05, 100.5, stream_name='position'
    ) == pytest.approx(100.05)
    assert mapper.reset_count == before


def test_single_forward_timestamp_outlier_does_not_reset_shared_mapping():
    mapper = Px4RosClockMapper(
        maximum_offset_jump=0.05,
        calibration_samples=2,
    )
    assert mapper.to_ros_time(
        10.0, 100.0, stream_name='position'
    ) is None
    assert mapper.to_ros_time(
        10.0, 100.01, stream_name='attitude'
    ) == pytest.approx(100.0)
    before = mapper.reset_count

    assert mapper.to_ros_time(
        15.0, 100.05, stream_name='position'
    ) is None
    assert mapper.last_status == 'OFFSET_OUTLIER'
    assert mapper.reset_count == before
    assert mapper.to_ros_time(
        10.05, 100.06, stream_name='position'
    ) == pytest.approx(100.05)


@pytest.mark.parametrize('old_offset', (0.507, 0.191))
def test_sustained_two_stream_offset_change_recovers_mapping(old_offset):
    mapper = Px4RosClockMapper(
        maximum_offset_jump=0.05,
        calibration_samples=4,
        offset_recovery_samples=3,
    )
    for stream, source in (
        ('position', 10.00),
        ('attitude', 10.00),
        ('position', 10.05),
        ('attitude', 10.05),
    ):
        mapper.to_ros_time(
            source,
            source + old_offset,
            stream_name=stream,
        )
    assert mapper.offset == pytest.approx(old_offset)

    mapped = []
    for index in range(6):
        source = 20.0 + 0.05 * index
        mapped.append(mapper.to_ros_time(
            source,
            source + 0.004,
            stream_name='position',
        ))
        mapped.append(mapper.to_ros_time(
            source + 0.002,
            source + 0.008,
            stream_name='attitude',
        ))

    assert mapper.offset_recalibration_count == 1
    assert mapper.reset_count == 1
    assert mapper.calibration_count == 2
    assert mapper.offset == pytest.approx(0.004)
    assert mapped[-1] == pytest.approx(20.256)
    assert mapper.last_status == 'MAPPED'


def test_one_stream_offset_outlier_does_not_trigger_recalibration():
    mapper = Px4RosClockMapper(
        maximum_offset_jump=0.05,
        calibration_samples=4,
        offset_recovery_samples=3,
    )
    for stream, source in (
        ('position', 10.00),
        ('attitude', 10.00),
        ('position', 10.05),
        ('attitude', 10.05),
    ):
        mapper.to_ros_time(
            source,
            source + 0.507,
            stream_name=stream,
        )

    assert mapper.to_ros_time(
        20.10, 20.105, stream_name='position'
    ) is None
    assert mapper.to_ros_time(
        20.10, 20.607, stream_name='attitude'
    ) == pytest.approx(20.607)
    assert mapper.reset_count == 0
    assert mapper.offset_recalibration_count == 0


def test_offset_recalibration_replaces_old_pose_history():
    node = object.__new__(rgbd_target_localizer.RgbdTargetLocalizer)
    now = [10.0]
    node._ros_seconds = lambda: now[0]
    node.px4_clock_mapper = Px4RosClockMapper(
        maximum_offset_jump=0.05,
        calibration_samples=4,
        offset_recovery_samples=3,
    )
    node.position_history = TimestampedVectorHistory(1.0)
    node.attitude_history = TimestampedVectorHistory(1.0)
    node.uav_position = None
    node.uav_attitude = None
    node.uav_position_time = -math.inf
    node.uav_attitude_time = -math.inf
    node.position_mapped_stamp = math.nan
    node.attitude_mapped_stamp = math.nan
    node.waiting_image_pair = None

    def send(stream, source, receipt):
        now[0] = receipt
        if stream == 'position':
            message = VehicleLocalPosition()
            message.x, message.y, message.z = 0.0, 0.0, -5.0
            callback = node.position_callback
        else:
            message = VehicleAttitude()
            message.q = [1.0, 0.0, 0.0, 0.0]
            callback = node.attitude_callback
        message.timestamp = round(source * 1_000_000)
        callback(message)

    for stream, source in (
        ('position', 10.00),
        ('attitude', 10.00),
        ('position', 10.05),
        ('attitude', 10.05),
    ):
        send(stream, source, source + 0.507)
    assert node.position_history.time_range is not None
    assert node.attitude_history.time_range is not None

    for index in range(6):
        source = 20.0 + 0.05 * index
        send('position', source, source + 0.004)
        send('attitude', source + 0.002, source + 0.008)

    assert node.px4_clock_mapper.offset_recalibration_count == 1
    assert node.position_history.time_range[0] >= 20.0
    assert node.attitude_history.time_range[0] >= 20.0


def test_px4_ros_domain_mode_keeps_true_source_sample_time():
    mapper = Px4RosClockMapper(
        source_is_ros_time=True,
        calibration_samples=4,
    )

    mapped = mapper.to_ros_time(
        100.125,
        receipt_ros_time=100.500,
        stream_name='position',
    )

    assert mapped == pytest.approx(100.125)
    assert mapper.offset == pytest.approx(0.0)
    assert mapper.last_status == 'DIRECT_TIMESTAMP'


def test_duplicate_and_small_out_of_order_sample_are_bounded_per_stream():
    mapper = Px4RosClockMapper(calibration_samples=1)
    mapper.to_ros_time(10.0, 100.0, stream_name='position')
    before = mapper.reset_count

    assert mapper.to_ros_time(
        10.0, 100.01, stream_name='position'
    ) is None
    assert mapper.last_status == 'DUPLICATE_SOURCE_TIMESTAMP'
    assert mapper.to_ros_time(
        9.99, 100.02, stream_name='position'
    ) is None
    assert mapper.last_status == 'OUT_OF_ORDER_SOURCE_TIMESTAMP'
    assert mapper.reset_count == before


def test_cross_topic_interleaving_does_not_clear_pose_histories():
    node = object.__new__(rgbd_target_localizer.RgbdTargetLocalizer)
    now = [100.070]
    node._ros_seconds = lambda: now[0]
    node.px4_clock_mapper = Px4RosClockMapper(
        maximum_offset_jump=0.05,
        calibration_samples=4,
    )
    node.position_history = TimestampedVectorHistory(1.0)
    node.attitude_history = TimestampedVectorHistory(1.0)
    node.uav_position = None
    node.uav_attitude = None
    node.uav_position_time = -math.inf
    node.uav_attitude_time = -math.inf

    def position(source, receipt):
        now[0] = receipt
        message = VehicleLocalPosition()
        message.timestamp = round(source * 1_000_000)
        message.x, message.y, message.z = 1.0, 2.0, -5.0
        node.position_callback(message)

    def attitude(source, receipt):
        now[0] = receipt
        message = VehicleAttitude()
        message.timestamp = round(source * 1_000_000)
        message.q = [1.0, 0.0, 0.0, 0.0]
        node.attitude_callback(message)

    position(10.050, 100.070)
    attitude(10.040, 100.075)
    position(10.100, 100.120)
    attitude(10.090, 100.125)

    assert node.px4_clock_mapper.reset_count == 0
    assert node.position_history.time_range is not None
    assert node.attitude_history.time_range is not None


def test_one_bad_position_timestamp_does_not_clear_attitude_history():
    node = object.__new__(rgbd_target_localizer.RgbdTargetLocalizer)
    now = [100.0]
    node._ros_seconds = lambda: now[0]
    node.px4_clock_mapper = Px4RosClockMapper(
        maximum_offset_jump=0.05,
        calibration_samples=4,
    )
    node.position_history = TimestampedVectorHistory(1.0)
    node.attitude_history = TimestampedVectorHistory(1.0)
    node.uav_position = None
    node.uav_attitude = None
    node.uav_position_time = -math.inf
    node.uav_attitude_time = -math.inf
    for index, source in enumerate((10.0, 10.05)):
        now[0] = 100.0 + 0.05 * index
        position = VehicleLocalPosition()
        position.timestamp = round(source * 1_000_000)
        position.x, position.y, position.z = 0.0, 0.0, -5.0
        node.position_callback(position)
        attitude = VehicleAttitude()
        attitude.timestamp = round(source * 1_000_000)
        attitude.q = [1.0, 0.0, 0.0, 0.0]
        node.attitude_callback(attitude)
    attitude_range = node.attitude_history.time_range
    before = node.px4_clock_mapper.reset_count

    now[0] = 100.10
    bad_position = VehicleLocalPosition()
    bad_position.timestamp = 1_000_000
    bad_position.x, bad_position.y, bad_position.z = 0.0, 0.0, -5.0
    node.position_callback(bad_position)

    assert node.px4_clock_mapper.reset_count == before
    assert node.attitude_history.time_range == attitude_range


def test_confirmed_px4_restart_clears_both_old_pose_histories():
    node = object.__new__(rgbd_target_localizer.RgbdTargetLocalizer)
    now = [100.0]
    node._ros_seconds = lambda: now[0]
    node.px4_clock_mapper = Px4RosClockMapper(calibration_samples=2)
    node.position_history = TimestampedVectorHistory(1.0)
    node.attitude_history = TimestampedVectorHistory(1.0)
    node.uav_position = None
    node.uav_attitude = None
    node.uav_position_time = -math.inf
    node.uav_attitude_time = -math.inf
    node.position_mapped_stamp = math.nan
    node.attitude_mapped_stamp = math.nan
    node.waiting_image_pair = ('old-clock-color', 'old-clock-depth')

    for kind, source, receipt in (
        ('position', 10.00, 100.00),
        ('attitude', 10.00, 100.01),
        ('position', 10.05, 100.05),
        ('attitude', 10.05, 100.06),
    ):
        now[0] = receipt
        if kind == 'position':
            message = VehicleLocalPosition()
            message.x, message.y, message.z = 0.0, 0.0, -5.0
            callback = node.position_callback
        else:
            message = VehicleAttitude()
            message.q = [1.0, 0.0, 0.0, 0.0]
            callback = node.attitude_callback
        message.timestamp = round(source * 1_000_000)
        callback(message)
    assert node.position_history.time_range is not None
    assert node.attitude_history.time_range is not None

    now[0] = 100.10
    position = VehicleLocalPosition()
    position.timestamp = 1_000_000
    position.x, position.y, position.z = 0.0, 0.0, -5.0
    node.position_callback(position)
    now[0] = 100.11
    attitude = VehicleAttitude()
    attitude.timestamp = 990_000
    attitude.q = [1.0, 0.0, 0.0, 0.0]
    node.attitude_callback(attitude)

    assert node.px4_clock_mapper.reset_count == 1
    assert node.position_history.time_range is None
    assert node.attitude_history.time_range is None
    assert node.waiting_image_pair is None

    now[0] = 100.16
    new_position = VehicleLocalPosition()
    new_position.timestamp = 1_050_000
    new_position.x, new_position.y, new_position.z = 0.0, 0.0, -5.0
    node.position_callback(new_position)

    assert node.position_history.time_range is not None
    assert node.attitude_history.time_range is not None
    assert node.position_history.time_range[0] > 100.0
    assert node.attitude_history.time_range[0] > 100.0


def test_rgb_depth_pairs_by_acquisition_time_despite_transport_delay():
    pairs = RgbDepthPairBuffer(maximum_skew=0.02, capacity=4)
    color = object()
    depth = object()

    assert pairs.add('color', color, 5.0, 10.01) == ''
    assert pairs.add('depth', depth, 5.0, 10.08) == ''
    paired_color, paired_depth, reason = pairs.pop_pair()

    assert reason == ''
    assert paired_color.message is color
    assert paired_depth.message is depth
    assert paired_color.raw_stamp == paired_depth.raw_stamp == 5.0


def test_rgb_depth_pairing_selects_closest_acquisition_times():
    pairs = RgbDepthPairBuffer(maximum_skew=0.1, capacity=4)
    early_color = object()
    nearest_color = object()
    depth = object()
    pairs.add('color', early_color, 5.00, 10.01)
    pairs.add('color', nearest_color, 5.09, 10.10)
    pairs.add('depth', depth, 5.08, 10.20)

    paired_color, paired_depth, reason = pairs.pop_pair()

    assert reason == ''
    assert paired_color.message is nearest_color
    assert paired_depth.message is depth
    assert paired_color.raw_stamp - paired_depth.raw_stamp == pytest.approx(
        0.01
    )
    assert len(pairs.color) == 1
    assert pairs.color[0].message is early_color


def test_rgb_depth_pairs_are_consumed_once_and_keep_real_skew():
    pairs = RgbDepthPairBuffer(maximum_skew=0.1, capacity=4)
    pairs.add('color', object(), 8.00, 20.02)
    pairs.add('depth', object(), 8.04, 20.11)

    color, depth, reason = pairs.pop_pair()

    assert reason == ''
    assert depth.raw_stamp - color.raw_stamp == pytest.approx(0.04)
    assert pairs.pop_pair() == (None, None, '')


def test_rgb_depth_buffer_rejects_duplicates_mismatch_and_resets():
    pairs = RgbDepthPairBuffer(maximum_skew=0.02, capacity=4)
    assert pairs.add('color', object(), 5.0, 10.0) == ''
    assert pairs.add('color', object(), 5.0, 10.1) == 'DUPLICATE_FRAME'
    assert pairs.add('depth', object(), 5.2, 10.2) == ''
    assert pairs.pop_pair()[2] == 'RGB_FRAME_UNMATCHED'
    assert pairs.add('color', object(), 1.0, 11.0) == 'TIME_RESET'
    assert len(pairs.depth) == 0


def test_synthetic_synchronized_red_rgbd_produces_valid_observation():
    node = object.__new__(rgbd_target_localizer.RgbdTargetLocalizer)
    now = [100.12]
    node._ros_seconds = lambda: now[0]
    node.maximum_rgb_depth_skew = 0.02
    node.data_timeout = 0.5
    node.image_pair_buffer = RgbDepthPairBuffer(0.02, 4)
    node.pending_image_pairs = __import__('collections').deque(maxlen=4)
    node.image_clock_mapper = GazeboImageClockMapper()
    node.px4_clock_mapper = Px4RosClockMapper(0.05, calibration_samples=4)
    node.image_clock_mapper.add_anchor(10.0, 100.0, 100.01, 1.0)
    node.image_clock_mapper.add_anchor(10.2, 100.2, 100.21, 1.2)
    node.image_clock_wait_timeout = 0.15
    node.position_history = TimestampedVectorHistory(1.0)
    node.attitude_history = TimestampedVectorHistory(1.0)
    node.position_history.add(100.0, (0.0, 0.0, -5.0))
    node.position_history.add(100.2, (0.0, 0.0, -5.0))
    node.attitude_history.add(100.0, (1.0, 0.0, 0.0, 0.0))
    node.attitude_history.add(100.2, (1.0, 0.0, 0.0, 0.0))
    node.color_time = node.depth_time = -math.inf
    node.last_pair_rejection = ''
    node.last_image_timeout_report = -math.inf
    node.last_time_diagnostic = {}
    node.frame_id = 'local_ned'
    node.horizontal_fov = math.pi / 2.0
    node.minimum_red_pixels = 3
    node.minimum_depth = 0.2
    node.maximum_depth = 25.0
    node.minimum_depth_ratio = 0.5
    node.camera_translation_flu = (0.0, 0.0, 0.0)
    node.camera_pitch_down = 0.0
    node.target_radius = 0.0
    node.target_reference_z_offset = 0.0
    node.base_position_std = 0.08
    node.range_position_std_scale = 0.01
    observations = []
    positions = []
    node.observation_pub = SimpleNamespace(
        publish=lambda message: observations.append(message)
    )
    node.target_position_pub = SimpleNamespace(
        publish=lambda message: positions.append(message)
    )

    color_array = np.zeros((4, 4, 3), dtype=np.uint8)
    color_array[1:3, 1:3, 2] = 255
    depth_array = np.full((4, 4), 5.0, dtype='<f4')
    color = Image()
    color.header.stamp.sec = 10
    color.header.stamp.nanosec = 100_000_000
    color.width = color.height = 4
    color.step = 12
    color.encoding = 'bgr8'
    color.data = color_array.tobytes()
    depth = Image()
    depth.header.stamp.sec = 10
    depth.header.stamp.nanosec = 100_000_000
    depth.width = depth.height = 4
    depth.step = 16
    depth.encoding = '32FC1'
    depth.data = depth_array.tobytes()

    node.color_callback(color)
    now[0] = 100.13
    node.depth_callback(depth)
    node.localize()

    assert len(observations) == 1
    assert observations[0].valid
    assert observations[0].red_pixel_count == 4
    assert observations[0].rejection_reason == ''
    assert observations[0].rgb_depth_acquisition_skew == pytest.approx(0.0)
    assert len(positions) == 1


def _make_synthetic_localizer(now):
    node = object.__new__(rgbd_target_localizer.RgbdTargetLocalizer)
    node._ros_seconds = lambda: now[0]
    node.maximum_rgb_depth_skew = 0.02
    node.data_timeout = 0.5
    node.image_pair_buffer = RgbDepthPairBuffer(0.02, 8)
    node.pending_image_pairs = __import__('collections').deque(maxlen=8)
    node.waiting_image_pair = None
    node.pose_wait_timeout = 0.15
    node.maximum_pose_wait_gap = 0.15
    node.image_clock_wait_timeout = 0.15
    node.image_clock_mapper = GazeboImageClockMapper()
    node.image_clock_mapper.add_anchor(99.9, 99.9, 99.91, 1.0)
    node.image_clock_mapper.add_anchor(100.7, 100.7, 100.71, 1.8)
    node.px4_clock_mapper = Px4RosClockMapper(0.05, calibration_samples=4)
    node.position_history = TimestampedVectorHistory(1.0)
    node.attitude_history = TimestampedVectorHistory(
        1.0,
        interpolator=quaternion_slerp,
    )
    node.color_time = node.depth_time = -math.inf
    node.last_pair_rejection = ''
    node.last_image_timeout_report = -math.inf
    node.last_time_diagnostic = {}
    node.frame_id = 'local_ned'
    node.horizontal_fov = math.pi / 2.0
    node.minimum_red_pixels = 3
    node.minimum_depth = 0.2
    node.maximum_depth = 25.0
    node.minimum_depth_ratio = 0.5
    node.camera_translation_flu = (0.0, 0.0, 0.0)
    node.camera_pitch_down = 0.0
    node.target_radius = 0.0
    node.target_reference_z_offset = 0.0
    node.base_position_std = 0.08
    node.range_position_std_scale = 0.01
    node.position_source_stamp = math.nan
    node.position_mapped_stamp = math.nan
    node.attitude_source_stamp = math.nan
    node.attitude_mapped_stamp = math.nan
    observations = []
    node.observation_pub = SimpleNamespace(
        publish=lambda message: observations.append(message)
    )
    node.target_position_pub = SimpleNamespace(publish=lambda _message: None)
    return node, observations


def _rgbd_messages(stamp):
    color_array = np.zeros((4, 4, 3), dtype=np.uint8)
    color_array[1:3, 1:3, 2] = 255
    depth_array = np.full((4, 4), 5.0, dtype='<f4')
    seconds = int(stamp)
    nanoseconds = round((stamp - seconds) * 1e9)
    color = Image()
    color.header.stamp.sec = seconds
    color.header.stamp.nanosec = nanoseconds
    color.width = color.height = 4
    color.step = 12
    color.encoding = 'bgr8'
    color.data = color_array.tobytes()
    depth = Image()
    depth.header.stamp.sec = seconds
    depth.header.stamp.nanosec = nanoseconds
    depth.width = depth.height = 4
    depth.step = 16
    depth.encoding = '32FC1'
    depth.data = depth_array.tobytes()
    return color, depth


def _enqueue_rgbd(node, now, stamp, color_delay, depth_delay):
    color, depth = _rgbd_messages(stamp)
    now[0] = stamp + color_delay
    node.color_callback(color)
    now[0] = stamp + depth_delay
    node.depth_callback(depth)


def test_localizer_processes_latest_complete_pair_instead_of_fifo_backlog():
    now = [100.0]
    node, observations = _make_synthetic_localizer(now)
    node.position_history.add(100.0, (0.0, 0.0, -5.0))
    node.position_history.add(100.5, (0.0, 0.0, -5.0))
    node.attitude_history.add(100.0, (1.0, 0.0, 0.0, 0.0))
    node.attitude_history.add(100.5, (1.0, 0.0, 0.0, 0.0))
    for stamp in (100.10, 100.15, 100.20):
        _enqueue_rgbd(node, now, stamp, 0.02, 0.03)

    now[0] = 100.24
    node.localize()

    assert len(observations) == 1
    assert observations[0].valid
    assert observations[0].stamp.sec == 100
    assert observations[0].stamp.nanosec == 200_000_000
    assert not node.pending_image_pairs


def test_continuous_rgbd_batches_produce_monotonic_valid_observations():
    now = [100.0]
    node, observations = _make_synthetic_localizer(now)
    node.position_history.add(100.0, (0.0, 0.0, -5.0))
    node.position_history.add(100.6, (0.6, 0.0, -5.0))
    node.attitude_history.add(100.0, (1.0, 0.0, 0.0, 0.0))
    node.attitude_history.add(100.6, (1.0, 0.0, 0.0, 0.0))
    for first, second, delays in (
        (100.10, 100.15, (0.02, 0.04)),
        (100.20, 100.25, (0.05, 0.03)),
        (100.30, 100.35, (0.01, 0.06)),
    ):
        _enqueue_rgbd(node, now, first, *delays)
        _enqueue_rgbd(node, now, second, *delays)
        now[0] = second + max(delays) + 0.01
        node.localize()

    stamps = [
        item.stamp.sec + item.stamp.nanosec * 1e-9
        for item in observations
    ]
    assert all(item.valid for item in observations)
    assert stamps == pytest.approx((100.15, 100.25, 100.35))


@pytest.mark.parametrize(
    'position_range, attitude_range, expected',
    (
        ((100.0, 100.2), (100.2, 100.4),
         'ATTITUDE_TIMESTAMP_BEFORE_HISTORY'),
        ((100.2, 100.4), (100.0, 100.2),
         'POSITION_TIMESTAMP_BEFORE_HISTORY'),
    ),
)
def test_localizer_reports_which_pose_history_misses_image_time(
    position_range,
    attitude_range,
    expected,
):
    now = [100.13]
    node, observations = _make_synthetic_localizer(now)
    for stamp in position_range:
        node.position_history.add(stamp, (0.0, 0.0, -5.0))
    for stamp in attitude_range:
        node.attitude_history.add(stamp, (1.0, 0.0, 0.0, 0.0))
    _enqueue_rgbd(node, now, 100.10, 0.02, 0.03)

    node.localize()

    assert len(observations) == 1
    assert not observations[0].valid
    assert observations[0].rejection_reason == expected


def test_image_waits_for_slightly_late_pose_then_localizes():
    now = [100.0]
    node, observations = _make_synthetic_localizer(now)
    node.position_history.add(100.00, (0.0, 0.0, -5.0))
    node.position_history.add(100.08, (0.08, 0.0, -5.0))
    node.attitude_history.add(100.00, (1.0, 0.0, 0.0, 0.0))
    node.attitude_history.add(100.08, (1.0, 0.0, 0.0, 0.0))
    _enqueue_rgbd(node, now, 100.10, 0.02, 0.03)

    node.localize()

    assert observations == []
    assert node.waiting_image_pair is not None

    node.position_history.add(100.12, (0.12, 0.0, -5.0))
    node.attitude_history.add(100.12, (1.0, 0.0, 0.0, 0.0))
    now[0] = 100.16
    node.localize()

    assert len(observations) == 1
    assert observations[0].valid
    assert observations[0].stamp.sec == 100
    assert observations[0].stamp.nanosec == 100_000_000
    assert node.waiting_image_pair is None


def test_image_does_not_wait_for_multi_second_pose_mismatch():
    now = [100.0]
    node, observations = _make_synthetic_localizer(now)
    node.position_history.add(95.00, (0.0, 0.0, -5.0))
    node.position_history.add(95.10, (0.1, 0.0, -5.0))
    node.attitude_history.add(95.00, (1.0, 0.0, 0.0, 0.0))
    node.attitude_history.add(95.10, (1.0, 0.0, 0.0, 0.0))
    _enqueue_rgbd(node, now, 100.10, 0.02, 0.03)

    node.localize()

    assert len(observations) == 1
    assert not observations[0].valid
    assert observations[0].rejection_reason == (
        'POSITION_TIMESTAMP_AFTER_HISTORY'
    )
    assert node.waiting_image_pair is None


def test_image_wait_timeout_reports_original_pose_rejection():
    now = [100.0]
    node, observations = _make_synthetic_localizer(now)
    node.position_history.add(100.00, (0.0, 0.0, -5.0))
    node.position_history.add(100.08, (0.08, 0.0, -5.0))
    node.attitude_history.add(100.00, (1.0, 0.0, 0.0, 0.0))
    node.attitude_history.add(100.08, (1.0, 0.0, 0.0, 0.0))
    _enqueue_rgbd(node, now, 100.10, 0.02, 0.03)
    node.localize()
    assert observations == []

    now[0] = 100.29
    node.localize()

    assert len(observations) == 1
    assert not observations[0].valid
    assert observations[0].rejection_reason == (
        'POSITION_TIMESTAMP_AFTER_HISTORY'
    )
    assert node.waiting_image_pair is None


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


def test_px4_callbacks_use_timestamp_sample_in_ros_time_domain():
    node = object.__new__(rgbd_target_localizer.RgbdTargetLocalizer)
    node._ros_seconds = lambda: 100.30
    node.px4_clock_mapper = Px4RosClockMapper(
        source_is_ros_time=True,
    )
    node.position_history = TimestampedVectorHistory(1.0)
    node.attitude_history = TimestampedVectorHistory(1.0)
    node.uav_position = None
    node.uav_attitude = None
    node.uav_position_time = -math.inf
    node.uav_attitude_time = -math.inf
    position = VehicleLocalPosition()
    position.timestamp = 100_200_000
    position.timestamp_sample = 100_100_000
    position.x, position.y, position.z = 1.0, 2.0, -5.0
    attitude = VehicleAttitude()
    attitude.timestamp = 100_210_000
    attitude.timestamp_sample = 100_110_000
    attitude.q = [1.0, 0.0, 0.0, 0.0]

    node.position_callback(position)
    node.attitude_callback(attitude)

    assert node.position_history.time_range == pytest.approx(
        (100.1, 100.1)
    )
    assert node.attitude_history.time_range == pytest.approx(
        (100.11, 100.11)
    )
    assert node.position_mapped_stamp == pytest.approx(100.1)
    assert node.attitude_mapped_stamp == pytest.approx(100.11)


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
    for source_seconds in (2.05, 2.10):
        now[0] = source_seconds + 8.0
        intermediate = VehicleAttitude()
        intermediate.timestamp = int(source_seconds * 1_000_000)
        intermediate.q = [-1.0, 0.0, 0.0, 0.0]
        node.attitude_callback(intermediate)
    now[0] = 10.2
    second = VehicleAttitude()
    second.timestamp = 2_200_000
    second.q = [-1.0, 0.0, 0.0, 0.0]
    node.attitude_callback(second)

    assert node.attitude_history.value_at(10.1) == pytest.approx(
        (1.0, 0.0, 0.0, 0.0)
    )


def test_image_waits_for_next_clock_anchor_then_localizes():
    now = [100.0]
    node, observations = _make_synthetic_localizer(now)
    node.image_clock_mapper = GazeboImageClockMapper()
    node.image_clock_mapper.add_anchor(100.00, 100.00, 100.01, 1.0)
    node.image_clock_mapper.add_anchor(100.08, 100.08, 100.09, 1.08)
    node.position_history.add(100.0, (0.0, 0.0, -5.0))
    node.position_history.add(100.2, (0.0, 0.0, -5.0))
    node.attitude_history.add(100.0, (1.0, 0.0, 0.0, 0.0))
    node.attitude_history.add(100.2, (1.0, 0.0, 0.0, 0.0))
    _enqueue_rgbd(node, now, 100.10, 0.02, 0.03)

    node.localize()

    assert observations == []
    assert node.waiting_image_pair is not None

    node.image_clock_mapper.add_anchor(100.20, 100.20, 100.21, 1.2)
    now[0] = 100.14
    node.localize()

    assert len(observations) == 1
    assert observations[0].valid


def test_missing_clock_reference_times_out_without_fabricating_stamp():
    now = [100.0]
    node, observations = _make_synthetic_localizer(now)
    node.image_clock_mapper = GazeboImageClockMapper()
    _enqueue_rgbd(node, now, 10.10, 90.02, 90.03)

    node.localize()
    assert observations == []

    now[0] = 100.31
    node.localize()

    assert len(observations) == 1
    assert not observations[0].valid
    assert observations[0].rejection_reason == (
        'IMAGE_CLOCK_REFERENCE_UNAVAILABLE'
    )
    assert observations[0].stamp.sec == 0


def test_mapped_image_in_future_has_specific_rejection_reason():
    now = [100.0]
    node, observations = _make_synthetic_localizer(now)
    node.image_clock_mapper = GazeboImageClockMapper()
    node.image_clock_mapper.add_anchor(10.0, 100.5, 100.51, 1.0)
    node.image_clock_mapper.add_anchor(10.2, 100.7, 100.71, 1.2)
    color, depth = _rgbd_messages(10.1)
    now[0] = 100.10
    node.color_callback(color)
    now[0] = 100.11
    node.depth_callback(depth)
    now[0] = 100.12

    node.localize()

    assert observations[-1].rejection_reason == 'IMAGE_TIMESTAMP_IN_FUTURE'


def test_old_mapped_image_has_specific_stale_rejection_reason():
    now = [100.0]
    node, observations = _make_synthetic_localizer(now)
    node.image_clock_mapper = GazeboImageClockMapper()
    node.image_clock_mapper.add_anchor(10.0, 99.0, 99.01, 1.0)
    node.image_clock_mapper.add_anchor(11.0, 100.0, 100.01, 2.0)
    color, depth = _rgbd_messages(10.1)
    now[0] = 100.02
    node.color_callback(color)
    now[0] = 100.03
    node.depth_callback(depth)
    now[0] = 100.04

    node.localize()

    assert observations[-1].rejection_reason == 'IMAGE_TIMESTAMP_STALE'
