"""Checks for evaluation-only ROI selection and offline sphere geometry."""

import json
import math
import threading
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))
from vision_p5_roi_analysis import (  # noqa: E402
    fit_sphere, roi_points, sphere_surface_x,
)
from vision_p5_roi_capture import (  # noqa: E402
    RoiWriter, choose_trigger, decode_float32_depth_roi,
    red_pixel_mask_roi, roi_bounds,
)
from uav_control.perception.front_tof_monitor import (  # noqa: E402
    decode_float32_depth, red_pixel_mask,
)


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


def test_roi_only_decode_equals_full_decode_with_row_padding():
    bounds = (1, 1, 4, 3)
    color_rows = np.zeros((4, 19), dtype=np.uint8)
    pixels = color_rows[:, :15].reshape(4, 5, 3)
    pixels[1, 1] = (250, 10, 0)
    pixels[2, 3] = (230, 30, 20)
    for encoding in ('rgb8', 'bgr8'):
        expected = red_pixel_mask(
            color_rows.tobytes(), 5, 4, 19, encoding)[1:3, 1:4]
        actual = red_pixel_mask_roi(
            color_rows.tobytes(), 5, 4, 19, encoding, bounds)
        np.testing.assert_array_equal(actual, expected)
    values = np.array([[1, 2, 3, 4, 5],
                       [6, np.nan, 8, np.inf, 10],
                       [11, 12, 13, 14, 15],
                       [16, 17, 18, 19, 20]], dtype='<f4')
    depth_rows = np.zeros((4, 24), dtype=np.uint8)
    depth_rows[:, :20] = values.view(np.uint8).reshape(4, 20)
    expected = decode_float32_depth(
        depth_rows.tobytes(), 5, 4, 24)[1:3, 1:4]
    actual = decode_float32_depth_roi(
        depth_rows.tobytes(), 5, 4, 24, bounds)
    np.testing.assert_array_equal(actual, expected)


def test_roi_writer_flushes_unique_npz_and_matching_metadata(tmp_path):
    writer = RoiWriter(tmp_path, maximum_pending=2)
    for index in range(2):
        mask = np.full((2, 3), index % 2, dtype=bool)
        depth = np.full((2, 3), index + 1, dtype=np.float32)
        assert writer.submit(index, mask, depth, {'trigger': 'normal_control'})
        mask[:] = False
        depth[:] = -1
    writer.close()
    records = [json.loads(line) for line in
               (tmp_path / 'roi.jsonl').read_text().splitlines()]
    assert [record['file'] for record in records] == [
        'roi_0000.npz', 'roi_0001.npz']
    assert writer.written == 2
    for index, record in enumerate(records):
        with np.load(tmp_path / record['file']) as archive:
            assert archive['depth'].tolist() == [[index + 1] * 3] * 2
            assert archive['mask'].tolist() == [[bool(index % 2)] * 3] * 2


def test_roi_writer_queue_full_never_blocks_callback(tmp_path, monkeypatch):
    import vision_p5_roi_capture as capture

    entered = threading.Event()
    release = threading.Event()
    original = capture.np.savez_compressed

    def blocked_save(*args, **kwargs):
        entered.set()
        assert release.wait(5)
        return original(*args, **kwargs)

    monkeypatch.setattr(capture.np, 'savez_compressed', blocked_save)
    writer = RoiWriter(tmp_path, maximum_pending=1)
    mask = np.ones((2, 2), dtype=bool)
    depth = np.ones((2, 2), dtype=np.float32)
    try:
        assert writer.submit(0, mask, depth, {})
        assert entered.wait(5)
        assert writer.submit(1, mask, depth, {})
        assert not writer.submit(2, mask, depth, {})
    finally:
        release.set()
        writer.close()
    assert writer.written == 2
    assert sorted(path.name for path in tmp_path.glob('*.npz')) == [
        'roi_0000.npz', 'roi_0001.npz']


def test_roi_writer_does_not_publish_metadata_for_failed_npz(tmp_path,
                                                             monkeypatch):
    import vision_p5_roi_capture as capture

    def fail_save(*args, **kwargs):
        raise OSError('simulated disk error')

    monkeypatch.setattr(capture.np, 'savez_compressed', fail_save)
    writer = RoiWriter(tmp_path, maximum_pending=1)
    mask = np.ones((2, 2), dtype=bool)
    depth = np.ones((2, 2), dtype=np.float32)
    assert writer.submit(0, mask, depth, {})
    with pytest.raises(RuntimeError, match='ROI writer failed'):
        writer.close()
    assert not list(tmp_path.glob('*.npz'))
    assert (tmp_path / 'roi.jsonl').read_text() == ''


def test_roi_writer_rejects_unbounded_queue(tmp_path):
    with pytest.raises(ValueError, match='positive'):
        RoiWriter(tmp_path, maximum_pending=0)
