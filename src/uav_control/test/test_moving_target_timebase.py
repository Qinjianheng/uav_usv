import pytest

from uav_control.moving_target import measured_motion_step


def test_motion_step_uses_measured_ros_clock_interval():
    assert measured_motion_step(1_000_000_000, 1_087_000_000, 0.05) == (
        pytest.approx(0.087)
    )


@pytest.mark.parametrize(
    'previous,current',
    [
        (None, 1_000_000_000),
        (1_000_000_000, 1_000_000_000),
        (2_000_000_000, 1_000_000_000),
        (1_000_000_000, 2_500_000_000),
    ],
)
def test_motion_step_rejects_invalid_or_large_clock_jumps(previous, current):
    assert measured_motion_step(previous, current, 0.05) == pytest.approx(0.05)
