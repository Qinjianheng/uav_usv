"""Verify timestamp signs and rejection grouping in offline P6 analysis."""

import importlib.util
from pathlib import Path


def load_analysis():
    path = (Path(__file__).resolve().parents[3]
            / 'scripts/vision_p6_pose_history_analysis.py')
    spec = importlib.util.spec_from_file_location(
        'pose_history_analysis', path,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_pose_history_gap_and_rejection_rate():
    analyze = load_analysis().analyze
    base = {
        'measurement_stamp': '10.0',
        'position_history_end_stamp': '9.98',
        'attitude_history_end_stamp': '10.01',
        'position_mapped_stamp': '9.98',
        'attitude_mapped_stamp': '10.01',
        'position_source_stamp': '9.98',
        'attitude_source_stamp': '10.01',
        'px4_position_interval': '0.02',
        'px4_attitude_interval': '0.01',
    }
    valid = dict(base, valid='True', rejection_reason='')
    rejected = dict(base, valid='False',
                    rejection_reason='POSITION_TIMESTAMP_AFTER_HISTORY')
    result = analyze([valid, rejected])
    assert result['valid_rate'] == 0.5
    assert result['position_timestamp_after_history_rate'] == 0.5
    assert result['valid']['position_future_gap']['p50'] > 0
    assert result['valid']['attitude_future_gap']['p50'] < 0
    assert result['position_timestamp_after_history']['count'] == 1
