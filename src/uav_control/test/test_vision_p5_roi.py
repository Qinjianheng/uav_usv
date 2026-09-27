"""Checks for evaluation-only ROI selection and offline sphere geometry."""

import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))
from vision_p5_roi_analysis import (  # noqa: E402
    fit_sphere, roi_points, sphere_surface_x,
)
from vision_p5_roi_capture import choose_trigger, roi_bounds  # noqa: E402


def test_roi_trigger_is_bounded_and_preserves_normal_controls():
    normal = SimpleNamespace(valid=True, rejection_reason='', depth_mad=0.02)
    rejected = SimpleNamespace(valid=False, rejection_reason='DEPTH_MAD_HIGH',
                               depth_mad=0.4)
    assert choose_trigger(rejected, math.nan, 0.25, {}, 20, 1.0) == 'mad_high'
    assert choose_trigger(normal, 0.35, 0.25, {}, 20, 1.0) == (
        'large_camera_error'
    )
    assert choose_trigger(normal, 0.02, 0.25, {}, 20, 0.01) == (
        'normal_control'
    )
    assert choose_trigger(normal, 0.02, 0.25,
                          {'normal_control': 15}, 20, 0.01) is None


def test_roi_bounds_clip_image_without_swapping_axes():
    message = SimpleNamespace(mask_bbox_left=1, mask_bbox_top=2,
                              mask_bbox_right=8, mask_bbox_bottom=9)
    assert roi_bounds(message, 10, 12) == (0, 0, 10, 12)


def test_known_radius_fit_uses_depth_points_and_intrinsics():
    radius = 0.25
    intrinsics = (200.0, 200.0, 20.0, 20.0)
    center = np.array((5.0, 0.1, -0.05))
    rows, cols = np.indices((41, 41))
    pixels = np.stack((cols.ravel(), rows.ravel()), axis=1)
    surface = sphere_surface_x(center, pixels, intrinsics, radius)
    mask = np.isfinite(surface).reshape(41, 41)
    depth = surface.reshape(41, 41).astype(np.float32)
    points, valid_pixels = roi_points(mask, depth, (0, 0, 41, 41),
                                      intrinsics)
    assert len(valid_pixels) >= 12
    for robust in (False, True):
        estimate = fit_sphere(points, (5.1, 0.1, -0.05), radius, robust)
        assert estimate == pytest.approx(center, abs=2e-3)
