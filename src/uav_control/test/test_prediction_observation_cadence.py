"""BCTRA learns derivatives from image observations, not KF publication ticks."""

import math

import pytest

from uav_control.tracking.target_prediction import PredictionEngine
from uav_control.tracking.target_prediction import TargetKinematicState


def turning_state(index, delay=0.02):
    """Return a 4 m/s, 0.2 rad/s target observed every 50 ms."""
    observation = 10.0 + index * 0.05
    heading = index * 0.01
    return TargetKinematicState(
        stamp=observation + delay,
        observation_stamp=observation,
        position=(0.0, 0.0, 0.0),
        velocity=(4.0 * math.cos(heading), 4.0 * math.sin(heading), 0.0),
    )


def test_repeated_projected_kf_states_do_not_create_turn_observations():
    """Repeated image projections preserve motion derivatives and provenance."""
    engine = PredictionEngine()
    for index in range(10):
        original = turning_state(index)
        repeated = turning_state(index, delay=0.035)
        assert engine.update(original, now=original.stamp)
        assert engine.update(repeated, now=repeated.stamp)

    prediction = engine.generate(now=repeated.stamp, mission_id=1,
                                 sequence_id=1)
    assert prediction.turn_rate == pytest.approx(0.2)
    assert prediction.turn_acceleration == pytest.approx(0.0, abs=1e-10)
    assert prediction.longitudinal_acceleration == pytest.approx(
        0.0, abs=1e-10,
    )
    assert engine.predictor.valid_turn_updates == 9
    assert prediction.source_stamp == repeated.stamp
    assert prediction.observation_stamp == repeated.observation_stamp
    assert prediction.valid_until == repeated.observation_stamp + 0.125


def test_irregular_kf_delivery_does_not_change_acquisition_turn_rate():
    """Transport delay does not alter a known constant physical turn rate."""
    engine = PredictionEngine()
    for index, delay in enumerate([0.02, 0.04, 0.01, 0.035, 0.02, 0.03]):
        state = turning_state(index, delay=delay)
        assert engine.update(state, now=state.stamp)
    prediction = engine.generate(now=state.stamp, mission_id=1, sequence_id=1)
    assert prediction.turn_rate == pytest.approx(0.2)
    assert prediction.turn_acceleration == pytest.approx(0.0, abs=1e-10)


def test_new_projection_epoch_cannot_restore_an_older_image_observation():
    """An older image cannot replace a newer state via a fresh KF epoch."""
    engine = PredictionEngine()
    current = turning_state(2)
    assert engine.update(current, now=current.stamp)
    older = TargetKinematicState(
        stamp=current.stamp + 0.01,
        observation_stamp=10.05,
        position=(0.0, 0.0, 0.0),
        velocity=(4.0, 0.0, 0.0),
    )
    assert not engine.update(older, now=older.stamp)
    assert engine.latest_state == current
