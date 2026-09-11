import math

import pytest

from uav_control.figure_eight_trajectory import FigureEightTrajectory


def make_trajectory():
    return FigureEightTrajectory(
        initial_x=20.0,
        initial_y=0.0,
        x_amplitude=40.0,
        y_amplitude=20.0,
        speed=5.0,
    )


def test_starts_at_existing_position_moving_in_positive_y():
    trajectory = make_trajectory()

    x, y, vx, vy = trajectory.state()

    assert (x, y) == pytest.approx((20.0, 0.0))
    assert vx == pytest.approx(0.0, abs=1e-12)
    assert vy == pytest.approx(5.0)


def test_trajectory_stays_inside_configured_operating_area():
    trajectory = make_trajectory()
    positions = [trajectory.advance(0.05)[:2] for _ in range(1200)]
    x_values = [position[0] for position in positions]
    y_values = [position[1] for position in positions]

    assert min(x_values) >= 20.0
    assert max(x_values) <= 100.0
    assert min(y_values) >= -20.0
    assert max(y_values) <= 20.0
    assert min(x_values) == pytest.approx(20.0, abs=0.01)
    assert max(x_values) == pytest.approx(100.0, abs=0.01)
    assert min(y_values) == pytest.approx(-20.0, abs=0.01)
    assert max(y_values) == pytest.approx(20.0, abs=0.01)


def test_horizontal_speed_remains_five_metres_per_second():
    trajectory = make_trajectory()

    for _ in range(1200):
        _, _, vx, vy = trajectory.advance(0.05)
        assert math.hypot(vx, vy) == pytest.approx(5.0, abs=1e-12)


def test_speed_can_ramp_from_rest_without_position_jump():
    trajectory = make_trajectory()
    initial_state = trajectory.state()

    trajectory.set_speed(0.0)
    stopped_state = trajectory.advance(0.05)

    assert stopped_state[:2] == pytest.approx(initial_state[:2])
    assert stopped_state[2:] == pytest.approx((0.0, 0.0))


def test_configured_usv_kinematics_fit_representative_limits():
    trajectory = make_trajectory()

    turn_rate, lateral_acceleration = trajectory.kinematic_envelope(5.0)

    assert turn_rate == pytest.approx(0.599, abs=0.002)
    assert lateral_acceleration == pytest.approx(2.994, abs=0.01)
    assert turn_rate <= 0.65
    assert lateral_acceleration <= 3.2


@pytest.mark.parametrize(
    'keyword,value',
    [
        ('x_amplitude', 0.0),
        ('y_amplitude', -1.0),
        ('speed', float('nan')),
    ],
)
def test_invalid_trajectory_parameters_are_rejected(keyword, value):
    parameters = {
        'initial_x': 20.0,
        'initial_y': 0.0,
        'x_amplitude': 40.0,
        'y_amplitude': 20.0,
        'speed': 5.0,
    }
    parameters[keyword] = value

    with pytest.raises(ValueError):
        FigureEightTrajectory(**parameters)
