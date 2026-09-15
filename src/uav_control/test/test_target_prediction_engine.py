"""Tests for timestamped target-prediction series generation."""

import pytest

from uav_control.tracking.target_prediction import PredictionEngine
from uav_control.tracking.target_prediction import TargetKinematicState


def make_state(stamp=10.0):
    """Return one finite target-state fixture."""
    return TargetKinematicState(
        stamp=stamp,
        position=(1.0, 2.0, 0.05),
        velocity=(4.0, 0.0, 0.4),
    )


def test_prediction_uses_original_state_stamp_and_fixed_sample_grid():
    engine = PredictionEngine(
        horizon=0.3,
        sample_period=0.1,
        input_timeout=0.125,
    )
    engine.update(make_state())

    result = engine.generate(now=10.05, mission_id=7, sequence_id=3)

    assert result.valid
    assert result.source_stamp == pytest.approx(10.0)
    assert result.generated_stamp == pytest.approx(10.05)
    assert result.valid_until == pytest.approx(10.125)
    assert result.mission_id == 7
    assert result.sequence_id == 3
    assert [sample.relative_time for sample in result.samples] == (
        pytest.approx([0.0, 0.1, 0.2, 0.3])
    )


def test_prediction_marks_stale_input_invalid_without_samples():
    engine = PredictionEngine(input_timeout=0.125)
    engine.update(make_state())

    result = engine.generate(now=10.126, mission_id=7, sequence_id=4)

    assert not result.valid
    assert result.invalid_reason == 'STATE_STALE'
    assert result.samples == ()


def test_prediction_rejects_non_monotonic_state_updates():
    engine = PredictionEngine()
    assert engine.update(make_state(stamp=10.0))

    assert not engine.update(make_state(stamp=9.9))
    assert engine.latest_state.stamp == pytest.approx(10.0)
