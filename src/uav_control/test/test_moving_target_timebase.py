import pytest

from uav_control.moving_target import measured_motion_step
from uav_control.moving_target import target_state_message


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


def test_target_state_message_is_timestamped_simulation_state():
    message = target_state_message(
        stamp_seconds=12.25,
        position=(1.0, 2.0, 0.1),
        velocity=(3.0, 4.0, 0.2),
    )

    assert message.stamp.sec == 12
    assert message.stamp.nanosec == 250_000_000
    assert message.frame_id == 'local_ned'
    assert (message.position.x, message.position.y, message.position.z) == (
        pytest.approx((1.0, 2.0, 0.1))
    )
    assert (message.velocity.x, message.velocity.y, message.velocity.z) == (
        pytest.approx((3.0, 4.0, 0.2))
    )
    assert message.valid
