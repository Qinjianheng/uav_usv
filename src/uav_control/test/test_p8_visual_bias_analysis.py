"""Offline visual-bias diagnostics never fit or inject online corrections."""

import csv
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys

import pytest


_SCRIPT = (Path(__file__).resolve().parents[3]
           / 'scripts/p8_visual_bias_analysis.py')


@pytest.fixture
def analyzer():
    assert _SCRIPT.is_file(), 'offline analyzer implementation is missing'
    spec = importlib.util.spec_from_file_location(
        'p8_visual_bias_analysis', _SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def vision_row(stamp=10.0, **changes):
    row = {
        'measurement_stamp': stamp,
        'observation_valid': 'true', 'truth_available': 'true',
        'error_x': 0.22, 'error_y': -0.522, 'error_z': 0.03,
        'horizontal_error': 0.566431814, 'target_range': 10.0,
        'geometry_diagnostics_enabled': 'true',
        'mask_centroid_u': 110.0, 'camera_fx': 100.0, 'camera_cx': 100.0,
        'position_x': 11.0, 'position_y': 2.0,
        'interpolated_uav_x': 1.0, 'interpolated_uav_y': 2.0,
        'interpolated_attitude_w': 1.0, 'interpolated_attitude_x': 0.0,
        'interpolated_attitude_y': 0.0, 'interpolated_attitude_z': 0.0,
        'rejection_reason': '',
    }
    row.update(changes)
    return row


def test_latest_like_statistics_preserve_raw_bias_and_all_frame_denominator(
    analyzer,
):
    rows = [vision_row(), vision_row(10.1, error_x=-0.22),
            vision_row(10.2, truth_available='false'),
            vision_row(10.3, observation_valid='false',
                       rejection_reason='POSITION_TIMESTAMP_AFTER_HISTORY')]

    result = analyzer.analyze_rows(rows)

    assert result['total_rows'] == 4
    assert result['valid_observation_rows'] == 3
    assert result['valid_rate'] == pytest.approx(0.75)
    assert result['raw_error']['sample_count'] == 2
    assert result['raw_error']['horizontal_rmse_m'] == (
        pytest.approx(0.566466239)
    )
    assert result['raw_error']['y_mean_m'] == pytest.approx(-0.522)
    assert result['position_timestamp_after_history_rows'] == 1
    assert result['online_correction'] == 'none'
    assert result['root_cause'] == 'not_confirmed'


@pytest.mark.parametrize('change', (
    {'camera_fx': ''}, {'camera_fx': 0.0},
    {'geometry_diagnostics_enabled': 'false'},
))
def test_image_bearing_requires_actual_enabled_finite_camera_geometry(
    analyzer, change,
):
    result = analyzer.analyze_rows([vision_row(**change)])

    assert result['image_bearing']['status'] == 'not_evaluated'
    assert result['image_bearing']['sample_count'] == 0
    assert result['image_bearing']['mean_rad'] is None


def test_valid_image_bearing_is_atan_pixel_offset_over_fx(analyzer):
    result = analyzer.analyze_rows([vision_row()])
    assert result['image_bearing']['mean_rad'] == pytest.approx(0.09966865249)


def test_same_acquisition_heading_reference_can_diagnose_signed_ned_residual(
    analyzer,
):
    # Hand-computed errors for a ten-metre estimated NED relative vector;
    # the prescribed inverse yaw rotation removes this synthetic error.
    rows = [vision_row(10.0, error_x=0.049958347219742,
                       error_y=0.9983341664682815),
            vision_row(10.1, error_x=0.199334221587584,
                       error_y=1.9866933079506122),
            vision_row(10.2, error_x=0.446635108743940,
                       error_y=2.9552020666133956)]
    references = [
        {'measurement_stamp': 10.0, 'px4_heading': 0.1,
         'reference_heading': 0.0},
        {'measurement_stamp': 10.1, 'px4_heading': 0.2,
         'reference_heading': 0.0},
        {'measurement_stamp': 10.2, 'px4_heading': 0.3,
         'reference_heading': 0.0},
    ]

    diagnostic = analyzer.analyze_rows(rows, references)['heading_diagnostic']

    assert diagnostic['status'] == 'evaluated_diagnostic_only'
    assert diagnostic['sample_count'] == 3
    assert diagnostic['yaw_error_rad']['mean'] == pytest.approx(0.2)
    assert diagnostic['range_sin_yaw_error_m']['mean'] == (
        pytest.approx(1.980076514)
    )
    assert diagnostic['signed_ned_error_m']['y_error_correlation'] == (
        pytest.approx(1.0)
    )
    assert diagnostic['residual_horizontal_rmse_m'] < 1e-12
    assert diagnostic['root_cause'] == 'not_confirmed'


def test_misaligned_heading_reference_is_excluded_without_interpolation(
    analyzer,
):
    reference = [{'measurement_stamp': 10.001, 'px4_heading': 0.1,
                  'reference_heading': 0.0}]
    diagnostic = analyzer.analyze_rows([vision_row()], reference)[
        'heading_diagnostic']

    assert diagnostic['status'] == 'not_evaluated'
    assert diagnostic['reason'] == 'insufficient_same_time_reference'
    assert diagnostic['sample_count'] == 0
    assert diagnostic['unmatched_rows'] == 1
    assert diagnostic['residual_horizontal_rmse_m'] is None


def test_only_numeric_stamp_tolerance_up_to_one_microsecond_is_allowed(
    analyzer,
):
    reference = [{'measurement_stamp': 10.0000005, 'px4_heading': 0.1,
                  'reference_heading': 0.0}]
    assert analyzer.analyze_rows([vision_row()], reference)[
        'heading_diagnostic']['sample_count'] == 1
    with pytest.raises(ValueError, match='tolerance'):
        analyzer.analyze_rows([vision_row()], reference, match_tolerance=0.001)


def test_quaternion_and_truth_error_columns_cannot_supply_heading_reference(
    analyzer,
):
    diagnostic = analyzer.analyze_rows([vision_row()])['heading_diagnostic']

    assert diagnostic['reference_source'] == 'none'
    assert diagnostic['status'] == 'not_evaluated'
    assert diagnostic['reason'] == 'insufficient_same_time_reference'
    assert diagnostic['sample_count'] == 0


def test_missing_relative_geometry_does_not_claim_attribution(
    analyzer,
):
    row = vision_row(interpolated_uav_x='')
    references = [{'measurement_stamp': 10.0, 'px4_heading': 0.1,
                   'reference_heading': 0.0}]
    diagnostic = analyzer.analyze_rows([row], references)['heading_diagnostic']

    assert diagnostic['status'] == 'not_evaluated'
    assert diagnostic['reason'] == 'insufficient_same_time_reference'
    assert diagnostic['missing_geometry_rows'] == 1


def test_duplicate_same_time_heading_references_are_ambiguous(analyzer):
    references = [{'measurement_stamp': 10.0, 'px4_heading': 0.1,
                   'reference_heading': 0.0},
                  {'measurement_stamp': 10.0, 'px4_heading': 0.2,
                   'reference_heading': 0.0}]
    diagnostic = analyzer.analyze_rows([vision_row()], references)[
        'heading_diagnostic']
    assert diagnostic['sample_count'] == 0
    assert diagnostic['ambiguous_reference_rows'] == 1


def test_wrapped_yaw_delta_respects_angle_boundary(analyzer):
    references = [{'measurement_stamp': 10.0, 'px4_heading': math.pi - 0.05,
                   'reference_heading': -math.pi + 0.05}]
    diagnostic = analyzer.analyze_rows([vision_row()], references)[
        'heading_diagnostic']
    assert diagnostic['yaw_error_rad']['mean'] == pytest.approx(-0.1)
    assert diagnostic['range_sin_yaw_error_m']['mean'] == (
        pytest.approx(-0.9983341665)
    )


def test_cli_writes_strict_json_with_null_for_unavailable_statistics(
    analyzer, tmp_path,
):
    del analyzer
    source = tmp_path / 'vision.csv'
    with source.open('w', newline='') as stream:
        row = vision_row(error_x='nan', error_y='inf', camera_fx='nan')
        writer = csv.DictWriter(stream, fieldnames=list(row))
        writer.writeheader()
        writer.writerow(row)
    output = tmp_path / 'report.json'

    subprocess.run([sys.executable, str(_SCRIPT), '--vision', str(source),
                    '--output', str(output)], check=True, capture_output=True)
    result = json.loads(output.read_text(), parse_constant=lambda value: (
        pytest.fail(f'nonfinite JSON number {value}')))

    assert result['raw_error']['horizontal_rmse_m'] is None
    assert result['image_bearing']['mean_rad'] is None
    assert result['online_correction'] == 'none'


def test_large_finite_csv_errors_do_not_overflow_json_statistics(analyzer):
    result = analyzer.analyze_rows([vision_row(error_x=1e200, error_y=-1e200)])
    assert result['raw_error']['horizontal_rmse_m'] == (
        pytest.approx(1.414213562e200)
    )
    json.dumps(result, allow_nan=False)


def test_empty_csv_does_not_report_zero_error_or_zero_valid_rate(analyzer):
    result = analyzer.analyze_rows([])
    assert result['valid_rate'] is None
    assert result['raw_error']['horizontal_rmse_m'] is None
    assert result['heading_diagnostic']['sample_count'] == 0
