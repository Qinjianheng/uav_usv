import ast
import inspect
import math
import textwrap

import pytest
from px4_msgs.msg import VehicleLocalPosition
from uav_usv_interfaces.msg import (
    InterceptTrajectory,
    MissionState,
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
from uav_control.control import trajectory_tracker_node
from uav_control.control.flight_guidance import FlightGuidanceCommand
from uav_control.control.flight_guidance import FlightGuidanceCore
from uav_control.control.trajectory_tracking import TrackingCommand
from uav_control.control.trajectory_tracking import PolynomialTrajectory
from uav_control.control.trajectory_tracking import TrackerKinematicState
from uav_control.control.trajectory_tracking import TrajectoryTrackerCore


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


def test_terminal_mission_forces_trajectory_and_diagnostic_terminal_mode():
    """Catch TERMINAL_MINCO publishing terminal_mode=False downstream."""
    trajectory = trajectory_from_message(make_trajectory_message())
    terminal = trajectory_tracker_node.trajectory_for_mission(
        trajectory,
        MissionState.TERMINAL_MINCO,
    )
    node = object.__new__(trajectory_tracker_node.TrajectoryTrackerNode)
    node.mission_id = 3
    node.mission_state = MissionState.TERMINAL_MINCO
    node.tracker = TrajectoryTrackerCore()
    node.tracker.active_trajectory = terminal
    node.latest_prediction = None
    node.latest_target_state = None
    node.latest_state = None
    node.last_target_yaw = None
    node.last_rejection = trajectory_tracker_node.TrajectoryRejectReason.NONE
    node.terminal_mode_latched = True
    published = []
    node.diagnostic_pub = type(
        'Publisher',
        (),
        {'publish': lambda _self, message: published.append(message)},
    )()

    node._publish_diagnostic(100.5, None, 'TRACKING', 0.001)

    assert terminal.terminal_mode
    assert published[0].terminal_mode


def test_controller_diagnostic_separates_prediction_and_trajectory_age():
    trajectory = trajectory_from_message(make_trajectory_message())
    prediction = TargetPrediction()
    prediction.source_stamp.sec = 100
    prediction.source_stamp.nanosec = 400_000_000
    node = object.__new__(trajectory_tracker_node.TrajectoryTrackerNode)
    node.mission_id = 3
    node.tracker = TrajectoryTrackerCore()
    node.tracker.active_trajectory = trajectory
    node.latest_prediction = prediction
    node.latest_target_state = None
    node.latest_state = None
    node.last_target_yaw = None
    node.last_rejection = trajectory_tracker_node.TrajectoryRejectReason.NONE
    node.terminal_mode_latched = False
    published = []
    node.diagnostic_pub = type(
        'Publisher',
        (),
        {'publish': lambda _self, message: published.append(message)},
    )()

    node._publish_diagnostic(100.5, None, 'TRACKING', 0.001)

    assert published[0].prediction_age == pytest.approx(0.10)
    assert published[0].trajectory_age == pytest.approx(0.25)
    assert published[0].source_age == pytest.approx(0.25)


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


def test_target_yaw_wraps_short_way_and_respects_rate_limit():
    """Catch yaw jumps through 358 degrees at the pi boundary."""
    previous = math.radians(179.0)
    desired = math.radians(-179.0)

    command = trajectory_tracker_node.rate_limited_target_yaw(
        previous_yaw=previous,
        desired_yaw=desired,
        maximum_rate=1.0,
        dt=0.05,
    )
    limited = trajectory_tracker_node.rate_limited_target_yaw(
        previous_yaw=0.0,
        desired_yaw=math.pi / 2.0,
        maximum_rate=1.0,
        dt=0.05,
    )

    assert command == pytest.approx(desired)
    assert limited == pytest.approx(0.05)


def test_px4_setpoint_uses_explicit_target_facing_yaw():
    command = TrackingCommand(
        position=(1.0, 2.0, -3.0),
        velocity=(4.0, 5.0, 1.5),
        acceleration=(0.1, 0.2, 0.3),
        plan_id=8,
        safety_state='SAFE',
        safety_margin=0.4,
    )

    message = command_to_setpoint(command, timestamp_us=55, yaw=1.25)

    assert message.yaw == pytest.approx(1.25)


def test_px4_setpoint_tolerates_missing_target_yaw_during_ground_hold():
    """GROUND_HOLD starts before a target sample can provide a heading."""
    command = FlightGuidanceCommand(
        mode='POSITION',
        position=(0.0, 0.0, 0.0),
        velocity=(0.0, 0.0, 0.0),
        acceleration=(0.0, 0.0, 0.0),
        takeoff_complete=False,
        far_guidance_available=False,
        safety_state='SAFE',
        safety_margin=1.0,
    )

    message = flight_command_to_setpoint(command, 1, yaw=None)

    assert math.isnan(message.yaw)


def test_terminal_mission_clears_plan_and_raises_underwater_hold():
    """Catch terminal missions continuing an old MINCO or holding below sea."""
    node = object.__new__(trajectory_tracker_node.TrajectoryTrackerNode)
    node.mission_id = 3
    node.tracker = TrajectoryTrackerCore(reserve_clearance=0.07)
    node.tracker.active_trajectory = PolynomialTrajectory(
        mission_id=3,
        plan_id=8,
        prediction_sequence_id=1,
        source_stamp=10.0,
        generated_stamp=10.01,
        valid_until=12.0,
        segments=(),
        terminal_position=(0.0, 0.0, -0.1),
        terminal_velocity=(0.0, 0.0, 0.0),
        target_state_source='simulation_truth',
    )
    node.latest_state = TrackerKinematicState(
        stamp=10.5,
        position=(1.0, 2.0, 0.1),
        velocity=(0.0, 0.0, 0.0),
    )
    node.flight_guidance = FlightGuidanceCore()
    node.last_rejection = None
    node.takeoff_complete_sent = False
    node.terminal_hold_position = None
    node.last_target_yaw = None
    message = MissionState()
    message.mission_id = 3
    message.state = MissionState.FAILURE
    message.state_name = 'FAILURE'

    node.mission_callback(message)

    assert node.tracker.active_trajectory is None
    assert node.terminal_hold_position == pytest.approx((1.0, 2.0, -0.07))


def test_invalid_message_is_rejected_before_reaching_tracker_core():
    message = make_trajectory_message()
    message.valid = False

    with pytest.raises(ValueError, match='invalid trajectory message'):
        trajectory_from_message(message)


def test_rejected_candidate_reports_attempted_plan_without_replacing_active():
    node = object.__new__(trajectory_tracker_node.TrajectoryTrackerNode)
    node.mission_id = 3
    node.mission_state = MissionState.MINCO_TRACKING
    node.tracker = TrajectoryTrackerCore()
    active_message = make_trajectory_message()
    active_message.plan_id = 8
    node.tracker.active_trajectory = trajectory_from_message(active_message)
    node.latest_state = TrackerKinematicState(
        stamp=100.5,
        position=(0.0, 0.0, -1.0),
        velocity=(0.0, 0.0, 0.0),
    )
    node.latest_prediction = None
    node.latest_target_state = None
    node.last_target_yaw = None
    node.last_rejection = trajectory_tracker_node.TrajectoryRejectReason.NONE
    node.terminal_mode_latched = False
    node._ros_seconds = lambda: 100.5
    published = []
    node.diagnostic_pub = type(
        'Publisher',
        (),
        {'publish': lambda _self, message: published.append(message)},
    )()
    candidate = make_trajectory_message()
    candidate.plan_id = 9

    node.trajectory_callback(candidate)

    assert published[-1].status == 'PLAN_REJECTED'
    assert published[-1].plan_id == 8
    assert published[-1].attempted_plan_id == 9
    assert node.tracker.active_trajectory.plan_id == 8


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


def test_tracking_velocity_mode_drops_position_and_sends_the_command_velocity():
    """
    Guard velocity-driven MINCO tracking, which is what builds closing speed.

    In position mode the reference is rebuilt from the measured state on every
    replan, so the position error never grows past a few centimetres and the
    vehicle can only hold the speed it already has: it flew at the target's
    4 m/s and closed at under 0.7 m/s.  Velocity mode lets PX4 consume the
    tracker's acceleration-limited command instead.
    """
    command = TrackingCommand(
        position=(1.0, 2.0, -3.0),
        velocity=(6.2, 0.4, -0.3),
        acceleration=(0.1, 0.2, 0.3),
        plan_id=9,
        safety_state='SAFE',
        safety_margin=0.3,
    )

    message = command_to_setpoint(
        command,
        timestamp_us=77,
        velocity_mode=True,
    )

    assert message.timestamp == 77
    assert all(math.isnan(value) for value in message.position)
    assert list(message.velocity) == pytest.approx([6.2, 0.4, -0.3])
    assert all(math.isnan(value) for value in message.acceleration)
    assert math.isnan(message.yaw)


def test_approach_parameters_are_wired_to_flight_guidance_not_tracker():
    """Catch ROS-node startup failure from passing kwargs to the wrong core."""
    tree = ast.parse(textwrap.dedent(inspect.getsource(
        trajectory_tracker_node.TrajectoryTrackerNode.__init__
    )))
    calls = {
        call.func.id: {keyword.arg for keyword in call.keywords}
        for call in ast.walk(tree)
        if isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
        and call.func.id in ('TrajectoryTrackerCore', 'FlightGuidanceCore')
    }
    approach_parameters = {
        'approach_contact_clearance',
        'approach_closing_speed',
        'approach_horizon',
        'approach_response_delay',
    }

    assert approach_parameters.isdisjoint(calls['TrajectoryTrackerCore'])
    assert approach_parameters <= calls['FlightGuidanceCore']
