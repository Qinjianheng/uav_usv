import math

import pytest
from px4_msgs.msg import VehicleLocalPosition
from uav_usv_interfaces.msg import (
    InterceptTrajectory,
    PolynomialSegment,
    PredictedTargetPoint,
    TargetPrediction,
)

from uav_control.control.trajectory_tracker_node import command_to_setpoint
from uav_control.control.trajectory_tracker_node import flight_command_to_setpoint
from uav_control.control.trajectory_tracker_node import trajectory_from_message
from uav_control.control.trajectory_tracker_node import tracker_state_from_message
from uav_control.control.trajectory_tracker_node import (
    prediction_endpoint_from_message,
)
from uav_control.control.flight_guidance import FlightGuidanceCommand
from uav_control.control.trajectory_tracking import TrackingCommand


def make_trajectory_message():
    message = InterceptTrajectory()
    message.mission_id = 3
    message.plan_id = 7
    message.prediction_sequence_id = 11
    message.source_stamp.sec = 100
    message.source_stamp.nanosec = 250_000_000
    message.generated_stamp.sec = 100
    message.generated_stamp.nanosec = 300_000_000
    message.valid_until.sec = 102
    message.valid_until.nanosec = 250_000_000
    message.target_state_source = 'simulation_truth'
    message.terminal_position.x = 4.0
    message.terminal_position.y = 5.0
    message.terminal_position.z = -0.1
    message.terminal_velocity.x = 1.0
    segment = PolynomialSegment()
    segment.duration.sec = 2
    segment.coefficients = [float(value) for value in range(18)]
    message.segments = [segment]
    message.piece_count = 1
    message.valid = True
    return message


def test_trajectory_conversion_preserves_source_time_and_coefficients():
    trajectory = trajectory_from_message(make_trajectory_message())

    assert trajectory.source_stamp == pytest.approx(100.25)
    assert trajectory.generated_stamp == pytest.approx(100.30)
    assert trajectory.valid_until == pytest.approx(102.25)
    assert trajectory.plan_id == 7
    assert trajectory.segments[0].duration == pytest.approx(2.0)
    assert trajectory.segments[0].coefficients == tuple(
        float(value) for value in range(18)
    )


def test_uav_conversion_uses_ros_receipt_time_not_px4_boot_time():
    message = VehicleLocalPosition()
    message.timestamp = 9_999_999
    message.x = 1.0
    message.y = 2.0
    message.z = -3.0
    message.vx = 4.0
    message.vy = 5.0
    message.vz = 6.0

    state = tracker_state_from_message(message, received_stamp=123.5)

    assert state.stamp == pytest.approx(123.5)
    assert state.position == pytest.approx((1.0, 2.0, -3.0))
    assert state.velocity == pytest.approx((4.0, 5.0, 6.0))


def test_tracking_command_maps_to_px4_feedforward_setpoint():
    command = TrackingCommand(
        position=(1.0, 2.0, -3.0),
        velocity=(4.0, 5.0, 1.5),
        acceleration=(0.1, 0.2, 0.3),
        plan_id=8,
        safety_state='SAFE',
        safety_margin=0.4,
    )

    message = command_to_setpoint(command, timestamp_us=55)

    assert message.timestamp == 55
    assert list(message.position) == pytest.approx([1.0, 2.0, -3.0])
    assert list(message.velocity) == pytest.approx([4.0, 5.0, 1.5])
    assert list(message.acceleration) == pytest.approx([0.1, 0.2, 0.3])
    assert math.isnan(message.yaw)


def test_invalid_message_is_rejected_before_reaching_tracker_core():
    message = make_trajectory_message()
    message.valid = False

    with pytest.raises(ValueError, match='invalid trajectory message'):
        trajectory_from_message(message)


def test_tracker_endpoint_query_uses_absolute_contact_stamp():
    message = make_prediction_message(
        source_stamp=10.2,
        samples=(
            (0.0, 40.8),
            (1.0, 44.8),
            (2.0, 48.8),
            (3.0, 52.8),
        ),
    )

    endpoint = prediction_endpoint_from_message(message, contact_stamp=13.0)

    assert endpoint[0] == pytest.approx(52.0)


def make_prediction_message(source_stamp, samples):
    """Create a complete prediction message for endpoint tests."""
    message = TargetPrediction()
    message.mission_id = 3
    message.sequence_id = 12
    message.source_stamp.sec = int(source_stamp)
    message.source_stamp.nanosec = int(
        round((source_stamp - int(source_stamp)) * 1e9)
    )
    message.valid = True
    message.prediction_horizon = samples[-1][0]
    for relative_time, x in samples:
        sample = PredictedTargetPoint()
        sample.relative_time.sec = int(relative_time)
        sample.relative_time.nanosec = int(
            round((relative_time - int(relative_time)) * 1e9)
        )
        sample.position.x = x
        message.samples.append(sample)
    return message


def test_flight_velocity_command_does_not_activate_position_control():
    command = FlightGuidanceCommand(
        mode='VELOCITY',
        position=(1.0, 2.0, -5.0),
        velocity=(3.0, 4.0, -1.0),
        acceleration=(0.0, 0.0, 0.0),
        takeoff_complete=False,
        far_guidance_available=True,
        safety_state='SAFE',
        safety_margin=1.0,
    )

    message = flight_command_to_setpoint(command, timestamp_us=99)

    assert all(math.isnan(value) for value in message.position)
    assert list(message.velocity) == pytest.approx([3.0, 4.0, -1.0])
