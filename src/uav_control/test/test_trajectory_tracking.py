"""Tests for realtime trajectory acceptance, sampling, and safety."""

import pytest

from uav_control.control.trajectory_tracking import PolynomialSegmentData
from uav_control.control.trajectory_tracking import PolynomialTrajectory
from uav_control.control.trajectory_tracking import TrackerKinematicState
from uav_control.control.trajectory_tracking import TrajectoryRejectReason
from uav_control.control.trajectory_tracking import TrajectoryTrackerCore


def linear_trajectory(mission_id=4, source_stamp=10.0, generated_stamp=10.02):
    """Create x=t, y=0, z=-1+0.2t for two seconds."""
    coefficients = (
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        -1.0, 0.2, 0.0, 0.0, 0.0, 0.0,
    )
    return PolynomialTrajectory(
        mission_id=mission_id,
        plan_id=7,
        prediction_sequence_id=3,
        source_stamp=source_stamp,
        generated_stamp=generated_stamp,
        valid_until=source_stamp + 2.0,
        segments=(PolynomialSegmentData(2.0, coefficients),),
        terminal_position=(2.0, 0.0, -0.6),
        terminal_velocity=(1.0, 0.0, 0.2),
        target_state_source='simulation_truth',
    )


def state(
    stamp=10.05,
    position=(0.05, 0.0, -0.99),
    velocity=(1.0, 0.0, 0.2),
):
    """Return a UAV state close to the propagated trajectory start."""
    return TrackerKinematicState(
        stamp=stamp,
        position=position,
        velocity=velocity,
    )


def test_polynomial_is_sampled_from_original_source_time():
    trajectory = linear_trajectory()

    sample = trajectory.sample_at_ros_time(10.5)

    assert sample.position == pytest.approx((0.5, 0.0, -0.9))
    assert sample.velocity == pytest.approx((1.0, 0.0, 0.2))


def test_stale_or_wrong_mission_trajectory_cannot_replace_active_plan():
    tracker = TrajectoryTrackerCore(maximum_plan_age=0.125)
    accepted = tracker.accept(linear_trajectory(), state(), mission_id=4)
    wrong_mission = linear_trajectory(mission_id=5, generated_stamp=10.06)
    rejected = tracker.accept(wrong_mission, state(10.07), mission_id=4)

    assert accepted == TrajectoryRejectReason.NONE
    assert rejected == TrajectoryRejectReason.MISSION_MISMATCH
    assert tracker.active_trajectory.plan_id == 7


def test_old_trajectory_continues_when_new_plan_is_stale():
    tracker = TrajectoryTrackerCore(maximum_plan_age=0.125)
    active = linear_trajectory()
    tracker.accept(active, state(), mission_id=4)
    stale = linear_trajectory(source_stamp=9.0, generated_stamp=10.06)

    rejected = tracker.accept(stale, state(10.07), mission_id=4)

    assert rejected == TrajectoryRejectReason.SOURCE_STALE
    assert tracker.active_trajectory is active
    assert tracker.command(state(10.10), mission_id=4) is not None


def test_final_sea_guard_overrides_unsafe_descent_command():
    tracker = TrajectoryTrackerCore(
        maximum_plan_age=0.125,
        position_gain=1.0,
        sea_surface_z=0.0,
        reserve_clearance=0.07,
        safety_response_delay=0.15,
        vertical_braking_acceleration=2.5,
    )
    unsafe = PolynomialTrajectory(
        mission_id=4,
        plan_id=8,
        prediction_sequence_id=3,
        source_stamp=20.0,
        generated_stamp=20.02,
        valid_until=22.0,
        segments=(PolynomialSegmentData(2.0, (
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            -1.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        )),),
        terminal_position=(2.0, 0.0, 1.0),
        terminal_velocity=(1.0, 0.0, 1.0),
        target_state_source='simulation_truth',
    )
    safe_start = state(
        stamp=20.05,
        position=(0.05, 0.0, -0.95),
        velocity=(1.0, 0.0, 1.0),
    )
    near_sea = state(
        stamp=20.80,
        position=(0.80, 0.0, -0.20),
        velocity=(1.0, 0.0, 1.0),
    )
    assert tracker.accept(
        unsafe,
        safe_start,
        mission_id=4,
    ) == TrajectoryRejectReason.NONE

    command = tracker.command(near_sea, mission_id=4)

    assert command.safety_state in ('BRAKE', 'UNRECOVERABLE')
    assert command.velocity[2] < 1.0


def test_expired_active_trajectory_enters_recoverable_no_plan_state():
    tracker = TrajectoryTrackerCore()
    tracker.accept(linear_trajectory(), state(), mission_id=4)

    command = tracker.command(state(stamp=12.01), mission_id=4)

    assert command is None
    assert tracker.active_trajectory is None
    assert tracker.status == 'NO_VALID_PLAN'


def test_plan_with_too_little_remaining_time_is_not_accepted():
    tracker = TrajectoryTrackerCore(minimum_remaining_time=0.20)
    trajectory = linear_trajectory()
    object.__setattr__(trajectory, 'valid_until', 10.15)

    rejected = tracker.accept(
        trajectory,
        state(stamp=10.05),
        mission_id=4,
    )

    assert rejected == TrajectoryRejectReason.INSUFFICIENT_REMAINING_TIME


def test_plan_identity_frame_and_target_endpoint_are_rechecked():
    tracker = TrajectoryTrackerCore(
        target_endpoint_tolerance=0.50,
        expected_frame_id='local_ned',
    )
    trajectory = linear_trajectory()
    current = state()

    prediction_mismatch = tracker.accept(
        trajectory,
        current,
        mission_id=4,
        prediction_sequence_id=99,
        target_endpoint=(2.0, 0.0, -0.6),
    )
    frame_mismatch_trajectory = linear_trajectory()
    object.__setattr__(frame_mismatch_trajectory, 'frame_id', 'map')
    frame_mismatch = tracker.accept(
        frame_mismatch_trajectory,
        current,
        mission_id=4,
        prediction_sequence_id=3,
        target_endpoint=(2.0, 0.0, -0.6),
    )
    endpoint_mismatch = tracker.accept(
        trajectory,
        current,
        mission_id=4,
        prediction_sequence_id=3,
        target_endpoint=(3.0, 0.0, -0.6),
    )

    assert prediction_mismatch == TrajectoryRejectReason.NONE
    assert frame_mismatch == TrajectoryRejectReason.FRAME_MISMATCH
    assert endpoint_mismatch == TrajectoryRejectReason.TARGET_ENDPOINT_MISMATCH


def test_prediction_sequence_is_traceability_only_not_acceptance_gate():
    tracker = TrajectoryTrackerCore()

    rejected = tracker.accept(
        linear_trajectory(),
        state(),
        mission_id=4,
        prediction_sequence_id=99,
        target_endpoint=(2.0, 0.0, -0.6),
    )

    assert rejected == TrajectoryRejectReason.NONE


def test_unrecoverable_sea_margin_rejects_new_plan():
    tracker = TrajectoryTrackerCore(
        sea_surface_z=0.0,
        reserve_clearance=0.07,
        safety_response_delay=0.15,
        vertical_braking_acceleration=2.5,
    )
    unsafe = PolynomialTrajectory(
        mission_id=4,
        plan_id=10,
        prediction_sequence_id=3,
        source_stamp=30.0,
        generated_stamp=30.01,
        valid_until=32.0,
        segments=(PolynomialSegmentData(2.0, (
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            -0.25, 1.0, 0.0, 0.0, 0.0, 0.0,
        )),),
        terminal_position=(2.0, 0.0, 1.75),
        terminal_velocity=(1.0, 0.0, 1.0),
        target_state_source='simulation_truth',
    )

    rejected = tracker.accept(
        unsafe,
        state(
            stamp=30.05,
            position=(0.05, 0.0, -0.20),
            velocity=(1.0, 0.0, 1.0),
        ),
        mission_id=4,
    )

    assert rejected == TrajectoryRejectReason.SAFETY_REJECTED
