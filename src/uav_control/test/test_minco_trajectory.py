import math

import pytest

from uav_control.guidance.minco_trajectory import MincoS3Trajectory


def test_single_piece_matches_complete_boundary_state():
    trajectory = MincoS3Trajectory(
        (0.0, 1.0, -2.0),
        (1.0, -0.5, 0.2),
        (0.1, 0.2, -0.1),
        (4.0, 3.0, -0.1),
        (2.0, 0.4, 0.0),
        (-0.2, 0.0, 0.1),
        durations=(1.7,),
    )

    start = trajectory.sample(0.0)
    end = trajectory.sample(trajectory.duration)
    assert start.position == pytest.approx((0.0, 1.0, -2.0))
    assert start.velocity == pytest.approx((1.0, -0.5, 0.2))
    assert start.acceleration == pytest.approx((0.1, 0.2, -0.1))
    assert end.position == pytest.approx((4.0, 3.0, -0.1))
    assert end.velocity == pytest.approx((2.0, 0.4, 0.0))
    assert end.acceleration == pytest.approx((-0.2, 0.0, 0.1))


def test_three_piece_trajectory_passes_waypoints_and_is_c4_continuous():
    trajectory = MincoS3Trajectory(
        (0.0, 0.0, -2.0),
        (1.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (6.0, 1.0, -0.1),
        (2.0, 0.2, 0.0),
        (0.0, 0.0, 0.0),
        ((2.0, 0.3, -1.5), (4.0, 0.8, -0.8)),
        (0.6, 0.7, 0.8),
    )

    assert trajectory.sample(0.6).position == pytest.approx(
        (2.0, 0.3, -1.5),
        abs=1e-8,
    )
    assert trajectory.sample(1.3).position == pytest.approx(
        (4.0, 0.8, -0.8),
        abs=1e-8,
    )
    for junction in (0.6, 1.3):
        left = trajectory.sample(junction - 1e-7)
        right = trajectory.sample(junction + 1e-7)
        assert left.velocity == pytest.approx(right.velocity, abs=2e-5)
        assert left.acceleration == pytest.approx(
            right.acceleration,
            abs=2e-5,
        )
        assert left.jerk == pytest.approx(right.jerk, abs=2e-4)


def test_control_effort_is_finite_and_positive():
    trajectory = MincoS3Trajectory(
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (3.0, 1.0, -1.0),
        (1.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        ((1.0, 0.4, -0.2),),
        (0.8, 0.9),
    )

    assert math.isfinite(trajectory.control_effort())
    assert trajectory.control_effort() > 0.0


@pytest.mark.parametrize(
    'waypoints,durations',
    [
        ((), ()),
        ((), (1.0, 1.0)),
        (((1.0, 0.0, 0.0),), (1.0, -1.0)),
    ],
)
def test_invalid_piece_definition_is_rejected(waypoints, durations):
    with pytest.raises(ValueError):
        MincoS3Trajectory(
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            waypoints,
            durations,
        )
