import math

import numpy as np
import pytest

from uav_control.perception.camera_visibility_monitor import (
    camera_intrinsics,
    count_red_pixels,
    preferred_camera_for_geometry,
    quaternion_pitch,
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

    count = count_red_pixels(
        image.tobytes(),
        width=4,
        height=3,
        step=12,
        encoding='rgb8',
    )

    assert count == 2


def test_count_red_pixels_supports_bgr_and_row_padding():
    rows = np.zeros((2, 16), dtype=np.uint8)
    pixels = rows[:, :12].reshape(2, 4, 3)
    pixels[0, 1] = (10, 20, 230)
    pixels[1, 2] = (255, 255, 255)

    count = count_red_pixels(
        rows.tobytes(),
        width=4,
        height=2,
        step=16,
        encoding='bgr8',
    )

    assert count == 1


def test_quaternion_pitch_uses_hamilton_wxyz_order():
    half_angle = math.radians(15.0)
    quaternion = (
        math.cos(half_angle),
        0.0,
        math.sin(half_angle),
        0.0,
    )

    assert quaternion_pitch(quaternion) == pytest.approx(
        math.radians(30.0)
    )


@pytest.mark.parametrize(
    ('distance', 'pitch_degrees', 'expected'),
    (
        (8.0, 0.0, 'front'),
        (1.5, 10.0, 'down'),
        (1.5, 35.0, 'front'),
    ),
)
def test_preferred_camera_uses_distance_and_terminal_pitch(
    distance,
    pitch_degrees,
    expected,
):
    assert preferred_camera_for_geometry(
        distance,
        math.radians(pitch_degrees),
        close_observation_distance=2.0,
        camera_switch_pitch=math.radians(30.0),
    ) == expected
