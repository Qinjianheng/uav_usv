import math

import numpy as np
import pytest

from uav_control.tracking.constant_velocity_kalman import (
    ConstantVelocityKalmanFilter,
)


def make_filter():
    return ConstantVelocityKalmanFilter(
        process_acceleration_std=0.5,
        measurement_position_std=0.02,
        initial_velocity_std=5.0,
    )


def test_filter_estimates_constant_velocity():
    target_filter = make_filter()
    dt = 0.05
    velocity = np.array([0.0, 5.0, 0.2])
    target_filter.initialize([20.0, 0.0, 0.0])

    for step in range(1, 201):
        target_filter.predict(dt)
        target_filter.update(
            np.array([20.0, 0.0, 0.0]) + velocity * step * dt
        )

    assert target_filter.state[:3] == pytest.approx(
        [20.0, 50.0, 2.0],
        abs=0.03,
    )
    assert target_filter.state[3:] == pytest.approx(
        velocity,
        abs=0.03,
    )


def test_filter_projects_position_without_mutating_state():
    target_filter = make_filter()
    target_filter.initialize([1.0, 2.0, 3.0])
    target_filter.state[3:] = [0.0, 5.0, -0.25]
    original_state = target_filter.state.copy()

    predicted = target_filter.predicted_position(2.0)

    assert predicted == pytest.approx([1.0, 12.0, 2.5])
    assert target_filter.state == pytest.approx(original_state)


def test_filter_covariance_remains_symmetric_positive_semidefinite():
    target_filter = make_filter()
    target_filter.initialize([0.0, 0.0, 0.0])

    for step in range(1, 50):
        target_filter.predict(0.05)
        target_filter.update([0.0, 0.25 * step, 0.0])

    assert target_filter.covariance == pytest.approx(
        target_filter.covariance.T,
        abs=1e-12,
    )
    assert np.linalg.eigvalsh(target_filter.covariance).min() >= -1e-12


def test_filter_rejects_invalid_time_values():
    target_filter = make_filter()
    target_filter.initialize([0.0, 0.0, 0.0])

    with pytest.raises(ValueError):
        target_filter.predict(-0.1)
    with pytest.raises(ValueError):
        target_filter.predicted_position(float('nan'))


def test_half_second_prediction_tracks_configured_heave():
    target_filter = make_filter()
    dt = 0.05
    horizon = 0.5
    vertical_errors = []

    for step in range(401):
        time = step * dt
        position = [
            20.0,
            5.0 * time,
            0.15 * math.sin(2.0 * math.pi * 0.25 * time),
        ]
        if not target_filter.initialized:
            target_filter.initialize(position)
        else:
            target_filter.predict(dt)
            target_filter.update(position)

        if time >= 5.0:
            future_time = time + horizon
            predicted_z = target_filter.predicted_position(horizon)[2]
            true_z = 0.15 * math.sin(
                2.0 * math.pi * 0.25 * future_time
            )
            vertical_errors.append(predicted_z - true_z)

    assert max(abs(error) for error in vertical_errors) < 0.11
