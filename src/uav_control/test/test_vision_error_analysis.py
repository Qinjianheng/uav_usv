import csv

import pytest

from uav_control.evaluation.vision_error_analysis import analyze_vision_csv


FIELDNAMES = (
    'measurement_stamp',
    'valid',
    'truth_available',
    'truth_x',
    'truth_y',
    'truth_z',
    'error_x',
    'error_y',
    'error_z',
    'position_3d_error',
    'observation_age',
    'rgb_depth_acquisition_skew',
    'image_clock_status',
    'rejection_reason',
)


def _write_rows(path, rows):
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


def test_analysis_reproduces_signed_and_motion_frame_errors(tmp_path):
    path = tmp_path / 'vision.csv'
    rows = []
    for index, skew in enumerate((0.0, 0.02, 0.0)):
        rows.append({
            'measurement_stamp': 10.0 + index,
            'valid': 'True',
            'truth_available': 'True',
            'truth_x': 4.0 * index,
            'truth_y': 0.0,
            'truth_z': 0.0,
            'error_x': 0.5,
            'error_y': -0.25,
            'error_z': 0.1,
            'position_3d_error': (0.5 ** 2 + 0.25 ** 2 + 0.1 ** 2) ** 0.5,
            'observation_age': 0.04 + 0.01 * index,
            'rgb_depth_acquisition_skew': skew,
            'image_clock_status': 'MAPPED_EXACT',
            'rejection_reason': '',
        })
    rows.append({
        'measurement_stamp': 13.0,
        'valid': 'False',
        'truth_available': 'False',
        'truth_x': '',
        'truth_y': '',
        'truth_z': '',
        'error_x': '',
        'error_y': '',
        'error_z': '',
        'position_3d_error': '',
        'observation_age': '',
        'rgb_depth_acquisition_skew': 0.0,
        'image_clock_status': 'MAPPED_EXACT',
        'rejection_reason': 'IMAGE_INVALID',
    })
    _write_rows(path, rows)

    result = analyze_vision_csv(path)

    assert result['observations'] == 4
    assert result['valid_truth_observations'] == 3
    assert result['signed_error']['x']['mean'] == pytest.approx(0.5)
    assert result['signed_error']['y']['mean'] == pytest.approx(-0.25)
    assert result['motion']['speed_median'] == pytest.approx(4.0)
    assert result['motion']['along']['mean'] == pytest.approx(0.5)
    assert result['motion']['cross']['mean'] == pytest.approx(-0.25)
    assert result['skew_groups']['zero']['count'] == 2
    assert result['skew_groups']['nonzero']['count'] == 1
    assert result['clock_status_histogram'] == {'MAPPED_EXACT': 4}
    assert result['rejection_histogram'] == {'IMAGE_INVALID': 1}


def test_analysis_does_not_invent_motion_direction_for_static_target(tmp_path):
    path = tmp_path / 'static.csv'
    _write_rows(path, [{
        'measurement_stamp': 10.0,
        'valid': 'True',
        'truth_available': 'True',
        'truth_x': 2.0,
        'truth_y': 3.0,
        'truth_z': -0.42,
        'error_x': 0.2,
        'error_y': 0.1,
        'error_z': -0.05,
        'position_3d_error': 0.2291287847,
        'observation_age': 0.03,
        'rgb_depth_acquisition_skew': 0.0,
        'image_clock_status': 'MAPPED_EXACT',
        'rejection_reason': '',
    }])

    result = analyze_vision_csv(path)

    assert result['motion']['count'] == 0
    assert result['motion']['speed_median'] is None
    assert result['signed_error']['x']['mean'] == pytest.approx(0.2)
