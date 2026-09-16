import pytest

from uav_control.control.flight_guidance import FlightGuidanceCore
from uav_control.control.flight_guidance import FlightKinematicState


def state(position=(0.0, 0.0, 0.0), velocity=(0.0, 0.0, 0.0)):
    return FlightKinematicState(tuple(position), tuple(velocity))


def test_ground_mode_holds_first_valid_position():
    guidance = FlightGuidanceCore(flight_altitude=-5.0)

    command = guidance.command('GROUND_HOLD', state((1.0, 2.0, -0.1)), None, 0.05)

    assert command.mode == 'POSITION'
    assert command.position == pytest.approx((1.0, 2.0, -0.1))


def test_takeoff_uses_bounded_vertical_velocity_and_settle_gate():
    guidance = FlightGuidanceCore(
        flight_altitude=-5.0,
        takeoff_maximum_vertical_speed=1.5,
        takeoff_settle_time=0.10,
        control_dt=0.05,
    )

    climbing = guidance.command('TAKEOFF', state(), None, 0.05)
    first_settled = guidance.command(
        'TAKEOFF',
        state((0.0, 0.0, -4.9), (0.0, 0.0, 0.0)),
        None,
        0.05,
    )
    settled = guidance.command(
        'TAKEOFF',
        state((0.0, 0.0, -4.9), (0.0, 0.0, 0.0)),
        None,
        0.05,
    )

    assert climbing.mode == 'VELOCITY'
    assert -1.5 <= climbing.velocity[2] < 0.0
    assert not first_settled.takeoff_complete
    assert settled.takeoff_complete


def test_takeoff_blends_horizontal_follow_by_height_without_changing_climb():
    """Catch modular TAKEOFF regressing to zero horizontal velocity."""
    guidance = FlightGuidanceCore(
        flight_altitude=-5.0,
        takeoff_horizontal_start_height=0.5,
        takeoff_horizontal_full_height=1.5,
        takeoff_maximum_horizontal_acceleration=10.0,
        control_dt=0.05,
    )
    target = state((0.0, 5.0, 0.0), (0.0, 4.0, 0.0))
    guidance.command('GROUND_HOLD', state(), target, 0.05)

    command = guidance.command(
        'TAKEOFF',
        state((0.0, 0.0, -1.0)),
        target,
        0.05,
    )

    assert command.velocity[0] == pytest.approx(0.0)
    assert command.velocity[1] == pytest.approx(2.0)
    assert command.velocity[2] < 0.0


def test_follow_tracks_behind_four_mps_target_within_limits():
    guidance = FlightGuidanceCore(
        flight_altitude=-5.0,
        follow_distance=5.0,
        maximum_horizontal_speed=6.5,
        maximum_horizontal_acceleration=3.0,
        control_dt=0.05,
    )
    target = state((8.0, 0.0, 0.0), (0.0, 4.0, 0.0))

    command = guidance.command(
        'FOLLOW',
        state((8.0, -5.0, -5.0)),
        target,
        0.05,
    )

    assert command.mode == 'VELOCITY'
    assert command.velocity == pytest.approx((0.0, 4.0, 0.0))
    assert command.far_guidance_available


def test_default_far_guidance_uses_six_point_two_mps_soft_limit():
    guidance = FlightGuidanceCore(
        flight_altitude=-5.0,
        maximum_horizontal_acceleration=100.0,
        control_dt=0.05,
    )
    target = state(
        (100.0, 0.0, -5.0),
        (0.0, 0.0, 0.0),
    )

    command = guidance.command(
        'FAR_GUIDANCE',
        state((0.0, 0.0, -5.0)),
        target,
        0.05,
    )

    assert command.velocity[0] == pytest.approx(6.2)
    assert command.velocity[1] == pytest.approx(0.0)


def test_missing_target_uses_position_hold_not_fast_pursuit():
    guidance = FlightGuidanceCore(flight_altitude=-5.0)

    command = guidance.command(
        'FAR_GUIDANCE',
        state((2.0, 3.0, -5.0), (1.0, 0.0, 0.0)),
        None,
        0.05,
    )

    assert command.mode == 'POSITION'
    assert command.position == pytest.approx((2.0, 3.0, -5.0))
    assert not command.far_guidance_available


def test_final_sea_guard_is_applied_after_follow_velocity_limit():
    guidance = FlightGuidanceCore(
        flight_altitude=-0.1,
        sea_surface_z=0.0,
        reserve_clearance=0.07,
        safety_response_delay=0.15,
        vertical_braking_acceleration=2.5,
    )
    target = state((1.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    command = guidance.command(
        'FOLLOW',
        state((0.0, 0.0, -0.1), (0.0, 0.0, 1.0)),
        target,
        0.05,
    )

    assert command.safety_state in ('BRAKE', 'UNRECOVERABLE')
    assert command.velocity[2] < 1.0
