"""Distinguish clock-domain lag from an actual late pose callback."""
import importlib.util
from pathlib import Path

import pytest

_PATH = (Path(__file__).resolve().parents[3]
         / 'scripts' / 'p8_pose_time_audit.py')
_SPEC = importlib.util.spec_from_file_location('p8_pose_time_audit', _PATH)
_AUDIT = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_AUDIT)


def test_clock_budget_uses_offset_sign_and_no_future_sync_sample():
    """A 210 ms clock error must not be reported as transport latency."""
    row = {'rgb_raw_stamp': 10.5, 'image_clock_offset': 1000.21,
           'processed_stamp': 1010.74, 'position_history_end_stamp': 1010.51,
           'image_measurement_stamp': 1010.71, 'receipt_stamp': 1010.73}
    result = _AUDIT.clock_budget(row, [10.0, 11.0], [-1e9, -1000210000])
    assert result['clock_lag_s'] == pytest.approx(0.21)
    assert result['residual_age_s'] == pytest.approx(0.02)
    assert result['timesync_sample_age_s'] == pytest.approx(0.5)
    assert _AUDIT.clock_budget(row, [11.0], [-1e9]) is None


def test_real_delivery_delay_is_retained_when_clocks_agree():
    """Do not explain away genuine delay with an empirical clock offset."""
    row = {'rgb_raw_stamp': 10.5, 'image_clock_offset': 1000.0,
           'processed_stamp': 1010.74, 'position_history_end_stamp': 1010.51,
           'image_measurement_stamp': 1010.71, 'receipt_stamp': 1010.73}
    result = _AUDIT.clock_budget(row, [10.0], [-1e9])
    assert result['clock_lag_s'] == 0
    assert result['residual_age_s'] == pytest.approx(0.23)
