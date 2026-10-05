"""Check physical epochs and bounded truth interpolation in the offline audit."""

import importlib.util
from pathlib import Path

import numpy as np
import pandas as pd
import pytest


SCRIPT = Path(__file__).resolve().parents[3] / 'scripts/y_position_estimation_audit.py'
SPEC = importlib.util.spec_from_file_location('y_position_audit', SCRIPT)
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)


def test_reference_interpolation_rejects_extrapolation_and_missing_bracket():
    times = np.array([10., 10.1, 10.5])
    positions = np.array([[0., 0., 0.], [.4, 0., 0.], [2., 0., 0.]])
    assert AUDIT.bracketed_position(times, positions, 10.05)[0] == pytest.approx(.2)
    for stamp in (9.99, 10.3, 10.51):
        assert AUDIT.bracketed_position(times, positions, stamp) is None


def test_kf_comparison_uses_sample_epoch_despite_delivery_delay():
    times = np.arange(10., 10.51, .1)
    vision = pd.DataFrame({
        'measurement_stamp': times, 'gazebo_entity_available': True,
        'gazebo_entity_reference_x': 4 * (times - 10.),
        'gazebo_entity_reference_y': 0., 'gazebo_entity_reference_z': 0.,
        'valid': True, 'position_x': 4 * (times - 10.),
        'position_y': 0., 'position_z': 0.,
    })
    records = [
        {'topic': 'driver', 'receipt': 10.1, 'message': {'command': 'Y'}},
        {'topic': 'kf', 'receipt': 10.35, 'message': {
            'valid': True, 'stamp': {'sec': 10, 'nanosec': 200000000},
            'position': {'x': .8, 'y': 0., 'z': 0.},
        }},
    ]
    report, _ = AUDIT.audit(records, vision)
    stats = report['statistics']['y_early']['kf']
    assert stats['count'] == 1
    assert stats['horizontal_rmse_m'] == pytest.approx(0., abs=1e-12)
    assert report['role'] == 'OFFLINE_EVALUATION_ONLY'
