"""Reject unsafe interpolation and separate clock error from attitude bias."""
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))
import p7_attitude_clock_analysis as analysis  # noqa: E402


def test_quaternion_bracket_rejects_extrapolation_gaps_and_resets():
    """Use the shortest arc and require a bounded, reset-free bracket."""
    q = Rotation.from_euler('z', [179, -179], degrees=True).as_quat()
    value = analysis.quaternion_at(1.01, [1, 1.02], q, [0, 0])
    assert abs(value.as_euler('xyz', degrees=True)[2]) == pytest.approx(180)
    for stamp, times, resets in [
        (0.99, [1, 1.02], [0, 0]),
        (1.01, [1, 1.2], [0, 0]),
        (1.01, [1, 1.02], [0, 1]),
    ]:
        with pytest.raises(ValueError):
            analysis.quaternion_at(stamp, times, q, resets)


def test_clock_and_real_bias_are_not_conflated_and_close_as_vectors():
    """Recover separately injected time and attitude errors."""
    # Three degrees real heading bias, one degree wrong-time rotation.
    rotations = [Rotation.from_euler('z', v, degrees=True)
                 for v in (4, 3, 0, 0)]
    terms = analysis.decompose([20, 0, 0], *rotations)
    assert terms['total'] == pytest.approx(
        terms['timing'] + terms['estimation'] + terms['reference'])
    assert np.linalg.norm(terms['estimation']) == pytest.approx(
        40 * np.sin(np.deg2rad(1.5)))
    assert np.linalg.norm(terms['timing']) == pytest.approx(
        40 * np.sin(np.deg2rad(0.5)))
    assert terms['reference'] == pytest.approx([0, 0, 0])


def test_static_bias_survives_perfect_timing_and_scales_with_range():
    """Retain static attitude bias when clock error is exactly zero."""
    biased = Rotation.from_euler('z', 2, degrees=True)
    truth = Rotation.identity()
    near = analysis.decompose([5, 0, 0], biased, biased, truth, truth)
    far = analysis.decompose([20, 0, 0], biased, biased, truth, truth)
    assert near['timing'] == pytest.approx([0, 0, 0])
    assert far['estimation'] == pytest.approx(4 * near['estimation'])
