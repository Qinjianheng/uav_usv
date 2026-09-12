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


def test_turn_acceleration_reduces_figure_eight_two_second_error():
    trajectory = FigureEightTrajectory(
        initial_x=8.0,
        initial_y=0.0,
        x_amplitude=40.0,
        y_amplitude=20.0,
        speed=4.0,
    )
    predictor = ManeuveringTargetPredictor()
    sample_dt = 0.05
    horizon = 2.0
    horizon_steps = round(horizon / sample_dt)
    states = [trajectory.state()]
    states.extend(trajectory.advance(sample_dt) for _ in range(1800))
    adaptive_errors = []
    constant_turn_errors = []

    for index, state in enumerate(states[:-horizon_steps]):
        x, y, vx, vy = state
        predictor.update_velocity(vx, vy, index * sample_dt)
        if index < 40:
            continue
        future_x, future_y = states[index + horizon_steps][:2]
        adaptive = predictor.predict(x, y, 0.0, vx, vy, 0.0, horizon)
        heading = math.atan2(vy, vx)
        constant_turn = predictor._constant_turn_state(
            x,
            y,
            math.hypot(vx, vy),
            heading,
            predictor.turn_rate,
            horizon,
        )
        adaptive_errors.append(math.hypot(
            adaptive[0] - future_x,
            adaptive[1] - future_y,
        ))
        constant_turn_errors.append(math.hypot(
            constant_turn[0] - future_x,
            constant_turn[1] - future_y,
        ))

    assert sum(adaptive_errors) / len(adaptive_errors) < 0.7 * (
        sum(constant_turn_errors) / len(constant_turn_errors)
    )


def test_speed_acceleration_moves_prediction_beyond_constant_velocity():
    predictor = ManeuveringTargetPredictor(
        speed_acceleration_filter_alpha=0.5,
    )
    sample_dt = 0.05
    acceleration = 0.8
    x = 0.0
    for index in range(20):
        velocity = 2.0 + acceleration * index * sample_dt
        predictor.update_velocity(velocity, 0.0, index * sample_dt)
        x += velocity * sample_dt

    predicted = predictor.predict(x, 0.0, 0.0, velocity, 0.0, 0.0, 1.0)

    assert predicted[0] > x + velocity
    assert predictor.acceleration(predicted[3], predicted[4], 0.0)[0] > 0.0


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
