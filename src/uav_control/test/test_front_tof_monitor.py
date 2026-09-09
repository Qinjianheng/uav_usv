import math

import numpy as np
import pytest

from uav_control.perception.front_tof_monitor import (
    camera_intrinsics,
    count_red_pixels,
    decode_float32_depth,
    red_pixel_mask,
    target_depth_statistics,
)


def test_camera_intrinsics_match_horizontal_field_of_view():
    fx, fy, cx, cy = camera_intrinsics(640, 480, math.pi / 2.0)

    assert fx == pytest.approx(320.0)
    assert fy == pytest.approx(320.0)
    assert cx == pytest.approx(320.0)
    assert cy == pytest.approx(240.0)


def test_count_red_pixels_supports_rgb_image_rows():
    image = np.zeros((3, 4, 3), dtype=np.uint8)
    image[0, 0] = (255, 0, 0)
    image[1, 2] = (220, 30, 20)
    image[2, 3] = (80, 0, 0)

    assert count_red_pixels(
        image.tobytes(),
        width=4,
        height=3,
        step=12,
        encoding='rgb8',
    ) == 2


def test_red_pixel_mask_supports_bgr_and_row_padding():
    rows = np.zeros((2, 16), dtype=np.uint8)
    pixels = rows[:, :12].reshape(2, 4, 3)
    pixels[0, 1] = (10, 20, 230)
    pixels[1, 2] = (255, 255, 255)

    mask = red_pixel_mask(
        rows.tobytes(),
        width=4,
        height=2,
        step=16,
        encoding='bgr8',
    )

    assert mask is not None
    assert int(np.count_nonzero(mask)) == 1
    assert mask[0, 1]


def test_decode_float32_depth_supports_row_padding():
    depth = np.array(
        [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]],
        dtype='<f4',
    )
    rows = np.zeros((2, 16), dtype=np.uint8)
    rows[:, :12] = depth.view(np.uint8).reshape(2, 12)

    decoded = decode_float32_depth(
        rows.tobytes(),
        width=3,
        height=2,
        step=16,
    )

    assert decoded is not None
    assert decoded == pytest.approx(depth)


def test_target_depth_statistics_reject_invalid_ranges():
    depth = np.array(
        [[2.0, np.inf], [30.0, 4.0]],
        dtype=np.float32,
    )
    target_mask = np.ones((2, 2), dtype=bool)

    target_range, valid_ratio = target_depth_statistics(
        depth,
        target_mask,
        minimum=0.2,
        maximum=25.0,
    )

    assert target_range == pytest.approx(3.0)
    assert valid_ratio == pytest.approx(0.5)
