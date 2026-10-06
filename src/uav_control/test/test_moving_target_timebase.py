import pytest
from types import SimpleNamespace

from uav_control.moving_target import MovingTarget, measured_motion_step
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


def paused_world_target():
    clock = [12_000_000_000]
    states, visual_updates = [], []
    target = SimpleNamespace(
        started=True, hit=False, gazebo_world_paused=False,
        gazebo_visualizer=SimpleNamespace(world_paused=True),
        x=8., y=0., z=.1, initial_z=.1, vx=4., vy=0., vz=0.,
        linear_vx=4., linear_vy=0., dt=.05,
        last_motion_update_time_ns=11_950_000_000,
        elapsed_time=0., figure_eight_trajectory=None,
        commanded_horizontal_speed=4., cruise_horizontal_speed=4.,
        horizontal_acceleration_limit=1., vertical_oscillation_amplitude=.15,
        vertical_angular_frequency=1.,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=clock[0])),
        position_pub=SimpleNamespace(publish=lambda _: None),
        velocity_pub=SimpleNamespace(publish=lambda _: None),
        state_pub=SimpleNamespace(publish=states.append),
        update_gazebo_visualization=lambda: visual_updates.append(True),
    )
    return target, clock, states, visual_updates


def test_native_world_pause_freezes_target_state_and_visual_pose_updates():
    target, clock, states, visual_updates = paused_world_target()
    for delta in (0, 400_000_000, 800_000_000):
        clock[0] = 12_000_000_000 + delta
        MovingTarget.timer_callback(target)
    assert (target.x, target.y, target.z) == pytest.approx((8., 0., .1))
    assert target.elapsed_time == 0.
    assert not visual_updates
    assert all((m.velocity.x, m.velocity.y, m.velocity.z) == (0., 0., 0.) for m in states)


def test_world_resume_does_not_integrate_the_paused_wall_time():
    target, clock, _, visual_updates = paused_world_target()
    MovingTarget.timer_callback(target)
    clock[0] += 800_000_000
    MovingTarget.timer_callback(target)
    target.gazebo_visualizer.world_paused = False
    clock[0] += 50_000_000
    MovingTarget.timer_callback(target)
    assert target.elapsed_time == pytest.approx(.05)
    assert target.x == pytest.approx(8.2)
    assert visual_updates == [True]
