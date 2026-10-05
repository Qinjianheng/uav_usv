"""Control acceptance uses physical UAV sample time, never delivery time."""

from types import SimpleNamespace

import pytest
from uav_control.control.trajectory_tracking import (
    PolynomialSegmentData, PolynomialTrajectory, TrajectoryRejectReason,
    TrajectoryTrackerCore,
)

from uav_control.control import trajectory_tracker_node as tracker
from uav_control.guidance import intercept_planner_node as planner


def navigation(stamp=10.02, valid=True):
    return SimpleNamespace(
        stamp=SimpleNamespace(sec=int(stamp), nanosec=round((stamp % 1) * 1e9)),
        valid=valid, frame_id='local_ned',
        position=SimpleNamespace(x=4., y=2., z=-5.),
        velocity=SimpleNamespace(x=4., y=0., z=0.),
        acceleration=SimpleNamespace(x=1., y=0., z=0.), heading=0.,
    )


def test_tracker_preserves_sample_time_when_delivery_is_late():
    state = tracker.tracker_state_from_navigation(navigation(), 10.1)
    assert state.stamp == pytest.approx(10.02)
    assert state.position == (4., 2., -5.)


def test_planner_preserves_sample_time_and_acceleration():
    state = planner.uav_state_from_navigation(navigation(), 10.1)
    assert state.stamp == pytest.approx(10.02)
    assert state.acceleration == (1., 0., 0.)


@pytest.mark.parametrize('converter', [
    lambda m: tracker.tracker_state_from_navigation(m, 10.1),
    lambda m: planner.uav_state_from_navigation(m, 10.1),
])
@pytest.mark.parametrize('mutation', ['invalid', 'future', 'zero', 'nonfinite'])
def test_navigation_time_and_geometry_fail_closed(converter, mutation):
    m = navigation()
    if mutation == 'invalid':
        m.valid = False
    elif mutation == 'future':
        m = navigation(10.11)
    elif mutation == 'zero':
        m = navigation(0.)
    else:
        m.position.x = float('nan')
    with pytest.raises(ValueError):
        converter(m)


def test_delayed_position_is_compared_to_reference_at_its_sample_time():
    trajectory = PolynomialTrajectory(
        mission_id=1, plan_id=1, prediction_sequence_id=1,
        source_stamp=10., generated_stamp=10., valid_until=12.,
        segments=(PolynomialSegmentData(2., (
            0., 4., 0., 0., 0., 0.,
            0., 0., 0., 0., 0., 0.,
            -5., 0., 0., 0., 0., 0.,
        )),), terminal_position=(8., 0., -5.),
        terminal_velocity=(4., 0., 0.), target_state_source='tracking',
    )
    m = navigation()
    m.position.x, m.position.y = .08, 0.
    state = tracker.tracker_state_from_navigation(m, 10.1)
    core = TrajectoryTrackerCore(maximum_position_error=.30)
    assert core.accept(trajectory, state, 1) == TrajectoryRejectReason.NONE
    # Relabelling the identical measurement with delivery time causes .32 m
    # false error, although the vehicle exactly follows the reference.
    receipt_state = type(state)(10.1, state.position, state.velocity)
    assert core.accept(trajectory, receipt_state, 1) == (
        TrajectoryRejectReason.STATE_POSITION_MISMATCH
    )
