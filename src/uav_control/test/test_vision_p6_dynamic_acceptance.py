"""Focused checks for P6.3 evaluation-only dynamic analysis."""

import csv
import importlib.util
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[3]


def load_script(name):
    path = ROOT / 'scripts' / name
    spec = importlib.util.spec_from_file_location(
        name.removesuffix('.py'), path,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_dynamic_summary_keeps_rejections_and_skew_bins(tmp_path):
    csv_path = tmp_path / 'capture.csv'
    with csv_path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=(
            'measurement_stamp', 'valid', 'rejection_reason',
            'rgb_depth_acquisition_skew',
        ))
        writer.writeheader()
        writer.writerow({
            'measurement_stamp': '1.0', 'valid': 'True',
            'rejection_reason': '', 'rgb_depth_acquisition_skew': '0.05',
        })
        writer.writerow({
            'measurement_stamp': '2.0', 'valid': 'False',
            'rejection_reason': 'POSITION_TIMESTAMP_AFTER_HISTORY',
            'rgb_depth_acquisition_skew': '0.0',
        })
    frame_path = tmp_path / 'frame.json'
    frame_path.write_text(json.dumps({'scene': {
        'per_frame': [{'measurement_stamp': 1.0,
                       'camera_error': [0.1, 0.0, 0.0]}],
        'phases': {'all': {
            'camera_error': {'rmse_3d_m': 0.1},
            'same_frame_error': {'rmse_3d_m': 0.1},
            'cross_frame_error': {'rmse_3d_m': 1.0},
            'frame_yaw_rad': {}, 'frame_translation_m': {},
        }},
        'complete_rows': 1,
    }}))
    pose_path = tmp_path / 'pose.json'
    pose_path.write_text(json.dumps({
        'position_timestamp_after_history_count': 1,
        'position_timestamp_after_history_rate': 0.5,
        'valid': {'position_future_gap': {}, 'attitude_future_gap': {}},
    }))
    result = load_script('vision_p6_dynamic_acceptance.py').summarize(
        csv_path, frame_path, pose_path,
    )
    assert result['valid_rate'] == 0.5
    assert result['after_history_count'] == 1
    assert result['rejection_reasons'][
        'POSITION_TIMESTAMP_AFTER_HISTORY'] == 1
    assert result['skew_bin_e1_m'][
        'abs_skew_40_to_60_ms']['rmse'] == 0.1


def test_kf_interpolation_requires_two_close_samples():
    module = load_script('vision_p6_kf_paired_baseline.py')
    states = [
        {'stamp': 1.0, 'source_stamp': 0.9,
         'position_ned': [0, 0, 0], 'velocity_ned': [2, 0, 0]},
        {'stamp': 1.1, 'source_stamp': 1.0,
         'position_ned': [0.2, 0, 0], 'velocity_ned': [2, 0, 0]},
    ]
    result = module.interpolate_kf(states, [1.0, 1.1], 1.05)
    assert np.allclose(result[0], [0.1, 0, 0])
    assert np.allclose(result[1], [2, 0, 0])
    assert module.interpolate_kf(states, [1.0, 1.1], 0.99) is None
    assert module.interpolate_kf(states, [1.0, 1.1], 1.2) is None
