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


def test_projected_kf_epoch_cannot_freshen_old_image_measurement():
    engine = PredictionEngine(input_timeout=0.125, source='tracking')
    state = TargetKinematicState(
        stamp=10.5,
        observation_stamp=10.0,
        position=(3.0, 2.0, 0.05),
        velocity=(4.0, 0.0, 0.0),
    )
    engine.update(state)

    result = engine.generate(now=10.5, mission_id=7, sequence_id=5)

    assert not result.valid
    assert result.invalid_reason == 'STATE_STALE'
    assert result.observation_stamp == pytest.approx(10.0)
    assert result.source_stamp == pytest.approx(10.5)
    assert result.samples == ()


def test_prediction_geometry_keeps_evaluation_epoch_and_acquisition_deadline():
    engine = PredictionEngine(horizon=0.2, sample_period=0.1)
    engine.update(TargetKinematicState(
        stamp=10.1,
        observation_stamp=10.0,
        position=(3.0, 2.0, 0.05),
        velocity=(4.0, 0.0, 0.0),
    ))

    result = engine.generate(now=10.11, mission_id=7, sequence_id=5)

    assert result.valid
    assert result.source_stamp == pytest.approx(10.1)
    assert result.observation_stamp == pytest.approx(10.0)
    assert result.valid_until == pytest.approx(10.125)
    assert result.samples[0].position[0] == pytest.approx(3.0)
    assert result.samples[-1].position[0] == pytest.approx(3.8)


@pytest.mark.parametrize('observation_stamp', (0.0, -1.0, float('nan'), 10.2))
def test_invalid_new_measurement_clears_past_valid_predictor_cache(
    observation_stamp,
):
    engine = PredictionEngine()
    engine.update(make_state())
    bad = TargetKinematicState(
        stamp=10.1,
        observation_stamp=observation_stamp,
        position=(1.0, 2.0, 0.05),
        velocity=(4.0, 0.0, 0.0),
    )

    assert not engine.update(bad, now=10.1)
    assert engine.latest_state is None
    assert not engine.generate(10.11, 7, 5).valid


def test_stale_new_input_clears_cache_but_out_of_order_input_keeps_newer():
    engine = PredictionEngine()
    engine.update(make_state(stamp=10.1))
    assert not engine.update(make_state(stamp=9.0), now=10.11)
    assert engine.generate(10.11, 7, 5).valid
    assert not engine.update(TargetKinematicState(
        stamp=10.12,
        observation_stamp=9.5,
        position=(1.0, 2.0, 0.05),
        velocity=(4.0, 0.0, 0.0),
    ), now=10.12)
    assert engine.latest_state is None
    assert not engine.generate(10.12, 7, 6).valid
