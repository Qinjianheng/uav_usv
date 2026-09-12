import pytest

from uav_control.tracking.prediction_error_tracker import (
    PredictionErrorTracker,
)


def test_prediction_is_only_evaluated_after_horizon():
    tracker = PredictionErrorTracker((1.0,))
    tracker.add('guidance', 10.0, {1.0: (5.0, 2.0, 0.0)})

    assert tracker.evaluate(
        'guidance',
        1.0,
        10.9,
        (4.0, 2.0, 0.0),
    ) is None
    evaluation = tracker.evaluate(
        'guidance',
        1.0,
        11.0,
        (4.0, 2.0, 0.0),
    )

    assert evaluation.position == pytest.approx((5.0, 2.0, 0.0))
    assert evaluation.error == pytest.approx(1.0)
    assert evaluation.age == pytest.approx(1.0)


def test_newest_due_prediction_is_matched_to_current_truth():
    tracker = PredictionErrorTracker((0.5,))
    tracker.add('kf', 0.0, {0.5: (1.0, 0.0, 0.0)})
    tracker.add('kf', 0.1, {0.5: (1.2, 0.0, 0.0)})

    evaluation = tracker.evaluate(
        'kf',
        0.5,
        0.65,
        (1.3, 0.0, 0.0),
    )

    assert evaluation.position == pytest.approx((1.2, 0.0, 0.0))
    assert evaluation.error == pytest.approx(0.1)
    assert evaluation.age == pytest.approx(0.55)


def test_models_and_horizons_are_kept_independent():
    tracker = PredictionErrorTracker((0.5, 1.0))
    tracker.add('guidance', 0.0, {
        0.5: (1.0, 0.0, 0.0),
        1.0: (2.0, 0.0, 0.0),
    })
    tracker.add('kf', 0.0, {
        0.5: (0.8, 0.0, 0.0),
        1.0: (1.6, 0.0, 0.0),
    })

    guidance = tracker.evaluate(
        'guidance',
        0.5,
        0.5,
        (0.9, 0.0, 0.0),
    )
    filtered = tracker.evaluate(
        'kf',
        0.5,
        0.5,
        (0.9, 0.0, 0.0),
    )

    assert guidance.error == pytest.approx(0.1)
    assert filtered.error == pytest.approx(0.1)
    assert tracker.evaluate(
        'guidance',
        1.0,
        0.5,
        (0.9, 0.0, 0.0),
    ) is None
