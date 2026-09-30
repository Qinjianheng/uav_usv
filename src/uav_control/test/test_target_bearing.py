"""RGB bearing must remain independent of depth/pose and acquisition timed."""

import math
from types import SimpleNamespace

import numpy as np
import pytest
from sensor_msgs.msg import Image

from uav_control.perception.rgbd_target_localizer import (
    camera_target_to_local_ned,
)
from uav_control.perception.target_bearing_node import (
    RgbBearingProcessor,
    image_target_bearing,
)


def image(stamp=10.1, column=6):
    msg = Image()
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = round((stamp - int(stamp)) * 1e9)
    msg.width, msg.height, msg.step = 8, 4, 24
    msg.encoding = 'rgb8'
    pixels = np.zeros((4, 8, 3), dtype=np.uint8)
    if column is not None:
        pixels[0:3, column, 0] = 255
    msg.data = pixels.tobytes()
    return msg


def processor():
    result = RgbBearingProcessor(horizontal_fov=math.pi / 2)
    result.add_clock(10.0, 100.0, 100.01, 1.0)
    result.add_clock(10.2, 100.2, 100.21, 1.2)
    return result


@pytest.mark.parametrize('column, expected', [(2, -0.463647609),
                                              (4, 0.0),
                                              (6, 0.463647609)])
def test_centroid_bearing_has_calibrated_image_left_right_sign(column, expected):
    detection = image_target_bearing(image(column=column), math.pi / 2, 3)
    assert detection.valid
    assert detection.bearing == pytest.approx(expected)
    assert 0 < detection.confidence <= 1


@pytest.mark.parametrize('column', [2, 6])
@pytest.mark.parametrize('pitch', [0.0, 0.35])
def test_bearing_sign_matches_camera_flu_to_px4_ned_clockwise_yaw(column, pitch):
    detection = image_target_bearing(image(column=column), math.pi / 2, 3)
    # Optical image-right is negative camera FLU left; pitch preserves lateral sign.
    target_ned = camera_target_to_local_ned(
        (1.0, -(column - 4) / 4, 0.0), (0, 0, 0),
        (1, 0, 0, 0), (0, 0, 0), pitch,
    )
    required_px4_yaw = math.atan2(target_ned[1], target_ned[0])
    assert detection.bearing * required_px4_yaw > 0


def test_valid_rgb_bearing_needs_no_depth_or_pose_input():
    detector = processor()
    detector.submit(image(), 100.11, 1.11)
    output = detector.poll(100.12, 1.12)
    assert output.valid
    assert output.stamp == pytest.approx(100.1)
    assert output.raw_stamp == pytest.approx(10.1)
    assert output.bearing == pytest.approx(math.atan(0.5))


@pytest.mark.parametrize('mutation', ['encoding', 'empty', 'missing', 'no_red'])
def test_invalid_rgb_preserves_mapped_acquisition_time(mutation):
    msg = image()
    if mutation == 'encoding':
        msg.encoding = 'mono8'
    elif mutation == 'empty':
        msg.width = 0
    elif mutation == 'missing':
        msg.data = b''
    else:
        msg = image(column=None)
    detector = processor()
    detector.submit(msg, 100.11, 1.11)
    output = detector.poll(100.12, 1.12)
    assert not output.valid
    assert output.stamp == pytest.approx(100.1)
    assert output.confidence == 0


def test_missing_clock_never_substitutes_rgb_receipt_timestamp():
    detector = RgbBearingProcessor()
    detector.submit(image(), 100.11, 1.11)
    assert detector.poll(100.12, 1.12) is None
    output = detector.poll(100.27, 1.27)
    assert not output.valid
    assert output.stamp == 0
    assert output.raw_stamp == pytest.approx(10.1)


def test_frame_waits_for_causal_clock_anchor_with_original_acquisition_stamp():
    detector = RgbBearingProcessor()
    detector.add_clock(10, 100, 100.01, 1)
    detector.submit(image(), 100.11, 1.11)
    assert detector.poll(100.12, 1.12) is None
    detector.add_clock(10.2, 100.2, 100.21, 1.2)
    output = detector.poll(100.22, 1.22)
    assert output.valid
    assert output.stamp == pytest.approx(100.1)


def test_old_frame_is_invalid_with_original_stamp_not_renewed():
    detector = processor()
    detector.submit(image(), 100.26, 1.26)
    output = detector.poll(100.27, 1.27)
    assert not output.valid
    assert output.stamp == pytest.approx(100.1)
    assert output.reason == 'IMAGE_TIMESTAMP_STALE'


def test_duplicate_and_out_of_order_frames_never_create_new_valid_samples():
    detector = processor()
    detector.submit(image(), 100.11, 1.11)
    assert detector.poll(100.12, 1.12).valid
    detector.submit(image(), 100.13, 1.13)
    assert detector.poll(100.14, 1.14) is None
    detector.submit(image(stamp=10.05), 100.14, 1.14)
    assert detector.poll(100.15, 1.15) is None
    detector.submit(image(stamp=10.15), 100.16, 1.16)
    output = detector.poll(100.17, 1.17)
    assert output.valid
    assert output.stamp == pytest.approx(100.15)


@pytest.mark.parametrize('stamp', [0, -1, math.nan])
def test_invalid_rgb_timestamp_is_invalid_without_receipt_substitution(stamp):
    msg = image()
    msg = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp, nanosec=0)),
        data=msg.data, width=msg.width, height=msg.height,
        step=msg.step, encoding=msg.encoding,
    )
    detector = processor()
    detector.submit(msg, 100.11, 1.11)
    output = detector.poll(100.12, 1.12)
    assert not output.valid
    assert output.stamp == 0


def test_clock_reset_discards_pending_rgb_and_previous_visibility():
    detector = processor()
    detector.submit(image(stamp=10.3), 100.31, 1.31)
    assert detector.poll(100.32, 1.32) is None
    detector.add_clock(0.1, 100.4, 100.41, 1.4)
    reset = detector.poll(100.42, 1.42)
    assert not reset.valid
    assert reset.stamp == 0
    detector.add_clock(0.3, 100.6, 100.61, 1.6)
    assert detector.poll(100.62, 1.62) is None
    detector.submit(image(stamp=0.2), 100.52, 1.52)
    output = detector.poll(100.63, 1.63)
    assert output.valid
    assert output.stamp == pytest.approx(100.5)


def test_system_clock_jump_clears_validity_and_pending_rgb():
    detector = processor()
    detector.submit(image(stamp=10.3), 100.31, 1.31)
    detector.add_clock(10.4, 101.4, 101.41, 1.4)
    output = detector.poll(101.42, 1.42)
    assert not output.valid
    assert output.stamp == 0
    assert detector.poll(101.43, 1.43) is None


def test_missing_rgb_timeout_invalidates_without_republishing_valid_frame():
    detector = processor()
    detector.submit(image(), 100.11, 1.11)
    assert detector.poll(100.12, 1.12).valid
    assert detector.poll(100.2, 1.2) is None
    output = detector.poll(100.62, 1.62)
    assert not output.valid
    assert output.stamp == pytest.approx(100.1)
    assert detector.poll(100.63, 1.63) is None
