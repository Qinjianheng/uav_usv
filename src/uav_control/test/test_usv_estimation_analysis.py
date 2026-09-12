import math

import pytest

from uav_control.analysis.usv_estimation_analysis import analyze_rows


def test_analysis_reports_availability_and_error_statistics():
    rows = [
        {
            'camera_measurement_valid': '1',
            'kf_state_valid': '1',
            'camera_position_error': '1.0',
            'kf_position_error': '0.5',
            'guidance_prediction_0p5_error': '2.0',
        },
        {
            'camera_measurement_valid': '0',
            'kf_state_valid': '1',
            'camera_position_error': '',
            'kf_position_error': '1.5',
            'guidance_prediction_0p5_error': '4.0',
        },
    ]

    summary = analyze_rows(rows)

    assert summary['row_count'] == 2
    assert summary['camera_availability'] == pytest.approx(0.5)
    assert summary['kalman_availability'] == pytest.approx(1.0)
    assert summary['camera']['rmse'] == pytest.approx(1.0)
    assert summary['kalman']['mean'] == pytest.approx(1.0)
    assert summary['guidance_0.5s']['rmse'] == pytest.approx(
        math.sqrt(10.0),
    )
    assert summary['guidance_1.0s'] is None
