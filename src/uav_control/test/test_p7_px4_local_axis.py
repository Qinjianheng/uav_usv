"""P7.2b independent world-axis analysis contracts."""

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest


SCRIPT = Path(__file__).resolve().parents[3] / 'scripts' / (
    'p7_px4_local_axis_analysis.py')


def load_script():
    spec = importlib.util.spec_from_file_location('p7_axis_analysis', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_enu_ned_and_displacement():
    module = load_script()
    assert module.enu_to_ned([2, 3, 4]) == [3, 2, -4]
    assert module.xy_displacement([1, 2], [7, 4]) == [6, 2]


def test_fixed_rotation_train_and_holdout_without_origin_fit():
    module = load_script()
    theta = 0.03
    rotation = np.array([[math.cos(theta), -math.sin(theta)],
                         [math.sin(theta), math.cos(theta)]])
    train_g = np.array([[4., 0.], [0., 5.]])
    train_p = train_g @ rotation.T
    hold_g = np.array([[-4., 0.], [0., -5.]])
    hold_p = hold_g @ rotation.T
    fitted = module.fit_fixed_rotation(train_g, train_p)
    assert fitted == pytest.approx(theta, abs=1e-8)
    assert module.displacement_rmse(
        hold_g, hold_p, fitted) == pytest.approx(0.0, abs=1e-8)
    assert module.displacement_rmse(hold_g, hold_p, 0.0) > 0.1


def test_collinear_training_cannot_certify_world_axes():
    module = load_script()
    with pytest.raises(ValueError, match='noncollinear'):
        module.require_noncollinear([[5., 0.], [-5., 0.]])


def test_bracket_rejects_extrapolation_and_wide_gap():
    module = load_script()
    assert module.bracket_gap(10.0, 9.98, 10.02, 0.05) == pytest.approx(
        0.02)
    with pytest.raises(ValueError, match='bracket'):
        module.bracket_gap(10.0, 10.01, 10.03, 0.05)
    with pytest.raises(ValueError, match='gap'):
        module.bracket_gap(10.0, 9.94, 10.02, 0.05)


def test_epoch_segmentation_and_yaw_only_no_world_displacement():
    module = load_script()
    rows = [
        {'ros_stamp': 1.0, 'reset_key': [1, 2]},
        {'ros_stamp': 1.1, 'reset_key': [1, 2]},
        {'ros_stamp': 1.2, 'reset_key': [2, 2]},
        {'ros_stamp': 1.15, 'reset_key': [2, 2]},
    ]
    assert module.segment_epochs(rows) == [0, 0, 1, 2]
    assert module.xy_displacement([2., 3.], [2., 3.]) == [0., 0.]


def test_ulog_clock_self_alignment_and_no_groundtruth_extrapolation():
    module = load_script()
    ulog_time = np.arange(0., 40., 0.02)
    trajectory = np.column_stack((
        0.2 * ulog_time + 0.03 * ulog_time ** 2,
        2.0 * np.sin(0.2 * ulog_time),
        np.zeros_like(ulog_time),
    ))
    selected = np.arange(8., 32., 0.5)
    captured = np.column_stack([
        np.interp(selected, ulog_time, trajectory[:, axis])
        for axis in range(3)
    ])
    ros_stamps = selected + 1000.0
    offset, rmse = module.align_ulog_ekf_to_capture(
        ros_stamps, captured, ulog_time, trajectory)
    assert offset == pytest.approx(1000.0, abs=0.01)
    assert rmse < 0.01
    with pytest.raises(ValueError, match='bracket'):
        module.interpolated_ulog_value(41.0, ulog_time, trajectory, 0.05)
