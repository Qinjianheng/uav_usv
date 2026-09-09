import math

import pytest

from uav_control.figure_eight_trajectory import FigureEightTrajectory
from uav_control.tracking.maneuvering_target_predictor import (
    ManeuveringTargetPredictor,
)


def test_falls_back_to_constant_velocity_until_turn_is_observed():
    predictor = ManeuveringTargetPredictor(minimum_updates=3)

    predicted = predictor.predict(
        1.0,
        2.0,
        -0.1,
        3.0,
        4.0,
        0.2,
        2.0,
    )

    assert predicted == pytest.approx((7.0, 10.0, 0.3, 3.0, 4.0, 0.2))


def test_constant_turn_prediction_matches_circular_motion():
    predictor = ManeuveringTargetPredictor(
        turn_rate_filter_alpha=1.0,
        max_turn_rate=2.0,
        maneuver_horizon=2.0,
        minimum_updates=2,
    )
    speed = 5.0
    turn_rate = 0.4
    sample_dt = 0.05

    for index in range(5):
        heading = turn_rate * index * sample_dt
        predictor.update_velocity(
            speed * math.cos(heading),
            speed * math.sin(heading),
            index * sample_dt,
        )

    current_time = 4 * sample_dt
    current_heading = turn_rate * current_time
    radius = speed / turn_rate
    current_x = radius * math.sin(current_heading)
    current_y = radius * (1.0 - math.cos(current_heading))
    horizon = 1.0

    predicted = predictor.predict(
        current_x,
        current_y,
        0.0,
        speed * math.cos(current_heading),
        speed * math.sin(current_heading),
        0.0,
        horizon,
    )
    final_heading = current_heading + turn_rate * horizon
    expected_x = radius * math.sin(final_heading)
    expected_y = radius * (1.0 - math.cos(final_heading))

    assert predicted[:2] == pytest.approx(
        (expected_x, expected_y),
        abs=1e-9,
    )
    assert predicted[3:5] == pytest.approx(
        (
            speed * math.cos(final_heading),
            speed * math.sin(final_heading),
        ),
        abs=1e-9,
    )


def test_turn_prediction_reduces_figure_eight_one_second_error():
    trajectory = FigureEightTrajectory(
        initial_x=20.0,
        initial_y=0.0,
        x_amplitude=40.0,
        y_amplitude=20.0,
        speed=5.0,
    )
    predictor = ManeuveringTargetPredictor(
        turn_rate_filter_alpha=0.25,
        max_turn_rate=1.2,
        maneuver_horizon=2.0,
    )
    sample_dt = 0.05
    horizon_steps = 20
    states = [trajectory.state()]
    states.extend(trajectory.advance(sample_dt) for _ in range(1600))

    maneuver_errors = []
    constant_velocity_errors = []
    for index, state in enumerate(states[:-horizon_steps]):
        x, y, vx, vy = state
        predictor.update_velocity(vx, vy, index * sample_dt)
        if index < 40:
            continue

        future_x, future_y = states[index + horizon_steps][:2]
        predicted = predictor.predict(
            x,
            y,
            0.0,
            vx,
            vy,
            0.0,
            horizon_steps * sample_dt,
        )
        maneuver_errors.append(
            math.hypot(predicted[0] - future_x, predicted[1] - future_y)
        )
        constant_velocity_errors.append(
            math.hypot(
                x + vx * horizon_steps * sample_dt - future_x,
                y + vy * horizon_steps * sample_dt - future_y,
            )
        )

    mean_maneuver_error = sum(maneuver_errors) / len(maneuver_errors)
    mean_cv_error = (
        sum(constant_velocity_errors) / len(constant_velocity_errors)
    )
    assert mean_maneuver_error < 0.8 * mean_cv_error


@pytest.mark.parametrize(
    'keyword,value',
    [
        ('turn_rate_filter_alpha', 1.1),
        ('max_turn_rate', 0.0),
        ('maneuver_horizon', float('nan')),
    ],
)
def test_invalid_configuration_is_rejected(keyword, value):
    parameters = {keyword: value}

    with pytest.raises(ValueError):
        ManeuveringTargetPredictor(**parameters)
