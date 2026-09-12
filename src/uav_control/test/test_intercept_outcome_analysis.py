import pytest

from uav_control.analysis.intercept_outcome_analysis import radial_components


def make_row(uav_vx, uav_vy, target_vx, target_vy):
    return {
        'uav_x': '0.0',
        'uav_y': '0.0',
        'target_x': '10.0',
        'target_y': '0.0',
        'uav_vx': str(uav_vx),
        'uav_vy': str(uav_vy),
        'target_vx': str(target_vx),
        'target_vy': str(target_vy),
    }


def test_radial_components_reports_positive_closing_rate():
    horizontal, closing, perpendicular = radial_components(
        make_row(3.0, 0.0, 1.0, 0.0)
    )

    assert horizontal == pytest.approx(10.0)
    assert closing == pytest.approx(2.0)
    assert perpendicular == pytest.approx(0.0)


def test_radial_components_reports_negative_opening_rate():
    _, closing, perpendicular = radial_components(
        make_row(1.0, 2.0, 3.0, 0.0)
    )

    assert closing == pytest.approx(-2.0)
    assert perpendicular == pytest.approx(2.0)
