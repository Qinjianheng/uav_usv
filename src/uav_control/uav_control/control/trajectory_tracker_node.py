"""Independent 20 Hz polynomial tracker and final PX4 safety barrier."""

import math
import time
from dataclasses import replace

import rclpy
from builtin_interfaces.msg import Time
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from uav_usv_interfaces.msg import ControllerDiagnostic, InterceptTrajectory
from uav_usv_interfaces.msg import MissionState, PlannerDiagnostic
from uav_usv_interfaces.msg import TargetPrediction

from .trajectory_tracking import PolynomialSegmentData
from .trajectory_tracking import PolynomialTrajectory
from .trajectory_tracking import TrackerKinematicState
from .trajectory_tracking import TrajectoryRejectReason
from .trajectory_tracking import TrajectoryTrackerCore


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _duration_seconds(duration):
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def _seconds_to_time(value):
    value = max(float(value), 0.0)
    seconds = int(value)
    nanoseconds = int(round((value - seconds) * 1e9))
    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds -= 1_000_000_000
    return Time(sec=seconds, nanosec=nanoseconds)


def trajectory_from_message(message):
    """Reconstruct the complete polynomial without resetting source time."""
    if not message.valid:
        raise ValueError('invalid trajectory message')
    if message.piece_count != len(message.segments) or not message.segments:
        raise ValueError('invalid trajectory message piece count')
    segments = tuple(PolynomialSegmentData(
        duration=_duration_seconds(segment.duration),
        coefficients=tuple(float(value) for value in segment.coefficients),
    ) for segment in message.segments)
    return PolynomialTrajectory(
        mission_id=int(message.mission_id),
        plan_id=int(message.plan_id),
        prediction_sequence_id=int(message.prediction_sequence_id),
        source_stamp=_stamp_seconds(message.source_stamp),
        generated_stamp=_stamp_seconds(message.generated_stamp),
        valid_until=_stamp_seconds(message.valid_until),
        segments=segments,
        terminal_position=(
            float(message.terminal_position.x),
            float(message.terminal_position.y),
            float(message.terminal_position.z),
        ),
        terminal_velocity=(
            float(message.terminal_velocity.x),
            float(message.terminal_velocity.y),
            float(message.terminal_velocity.z),
        ),
        target_state_source=str(message.target_state_source),
        frame_id=str(message.frame_id),
    )


def tracker_state_from_message(message, received_stamp):
    """Use ROS receipt time, not the unrelated PX4 boot timestamp."""
    values = []
    for name in ('x', 'y', 'z', 'vx', 'vy', 'vz'):
        value = float(getattr(message, name, math.nan))
        values.append(value if math.isfinite(value) else 0.0)
    return TrackerKinematicState(
        stamp=float(received_stamp),
        position=tuple(values[:3]),
        velocity=tuple(values[3:]),
    )


def command_to_setpoint(command, timestamp_us):
    """Map the post-guard command to a PX4 position-feedforward setpoint."""
    message = TrajectorySetpoint()
    message.timestamp = int(timestamp_us)
    message.position = [float(value) for value in command.position]
    message.velocity = [float(value) for value in command.velocity]
    message.acceleration = [float(value) for value in command.acceleration]
    message.jerk = [math.nan, math.nan, math.nan]
    message.yaw = math.nan
    message.yawspeed = math.nan
    return message


def hold_setpoint(state, timestamp_us):
    """Hold the last valid position without a high-speed fallback."""
    message = TrajectorySetpoint()
    message.timestamp = int(timestamp_us)
    message.position = [float(value) for value in state.position]
    message.velocity = [0.0, 0.0, 0.0]
    message.acceleration = [math.nan, math.nan, math.nan]
    message.jerk = [math.nan, math.nan, math.nan]
    message.yaw = math.nan
    message.yawspeed = math.nan
    return message


def prediction_endpoint_from_message(message, relative_time):
    """Interpolate a target endpoint solely for trajectory acceptance."""
    if not message.valid or not message.samples:
        raise ValueError('target prediction is unavailable')
    desired_time = max(float(relative_time), 0.0)
    samples = list(message.samples)
    first_time = _duration_seconds(samples[0].relative_time)
    if desired_time <= first_time:
        point = samples[0].position
        return float(point.x), float(point.y), float(point.z)
    for left, right in zip(samples, samples[1:]):
        left_time = _duration_seconds(left.relative_time)
        right_time = _duration_seconds(right.relative_time)
        if desired_time <= right_time:
            span = max(right_time - left_time, 1e-9)
            ratio = min(max((desired_time - left_time) / span, 0.0), 1.0)
            return tuple(
                float(getattr(left.position, axis))
                + ratio * (
                    float(getattr(right.position, axis))
                    - float(getattr(left.position, axis))
                )
                for axis in ('x', 'y', 'z')
            )
    point = samples[-1].position
    return float(point.x), float(point.y), float(point.z)


class TrajectoryTrackerNode(Node):
    """Track accepted plans while keeping optimization out of control."""

    CONTROL_STATES = {
        MissionState.MINCO_READY,
        MissionState.MINCO_TRACKING,
        MissionState.TERMINAL_MINCO,
        MissionState.PLAN_RECOVERY,
        MissionState.SAFE_WAIT,
    }

    def __init__(self):
        super().__init__('trajectory_tracker_node')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('maximum_plan_age', 0.125)
        self.declare_parameter('minimum_remaining_time', 0.20)
        self.declare_parameter('maximum_position_error', 0.30)
        self.declare_parameter('maximum_velocity_error', 0.50)
        self.declare_parameter('target_endpoint_tolerance', 0.50)
        self.declare_parameter('position_gain', 1.2)
        self.declare_parameter('maximum_horizontal_speed', 6.5)
        self.declare_parameter('maximum_vertical_speed', 4.0)
        self.declare_parameter('maximum_horizontal_acceleration', 3.0)
        self.declare_parameter('maximum_vertical_acceleration', 3.0)
        self.declare_parameter('sea_surface_z', 0.0)
        self.declare_parameter('reserve_clearance', 0.07)
        self.declare_parameter('safety_response_delay', 0.15)
        self.declare_parameter('vertical_braking_acceleration', 2.5)
        self.declare_parameter('maximum_state_age', 0.125)
        self.declare_parameter('frame_id', 'local_ned')

        control_rate = float(self.get_parameter('control_rate_hz').value)
        if not math.isfinite(control_rate) or control_rate <= 0.0:
            raise ValueError('control_rate_hz must be finite and positive')
        self.maximum_state_age = float(
            self.get_parameter('maximum_state_age').value
        )
        self.expected_frame_id = str(self.get_parameter('frame_id').value)
        self.tracker = TrajectoryTrackerCore(
            maximum_plan_age=self.get_parameter('maximum_plan_age').value,
            minimum_remaining_time=self.get_parameter(
                'minimum_remaining_time'
            ).value,
            maximum_position_error=self.get_parameter(
                'maximum_position_error'
            ).value,
            maximum_velocity_error=self.get_parameter(
                'maximum_velocity_error'
            ).value,
            target_endpoint_tolerance=self.get_parameter(
                'target_endpoint_tolerance'
            ).value,
            expected_frame_id=self.expected_frame_id,
            position_gain=self.get_parameter('position_gain').value,
            maximum_horizontal_speed=self.get_parameter(
                'maximum_horizontal_speed'
            ).value,
            maximum_vertical_speed=self.get_parameter(
                'maximum_vertical_speed'
            ).value,
            maximum_horizontal_acceleration=self.get_parameter(
                'maximum_horizontal_acceleration'
            ).value,
            maximum_vertical_acceleration=self.get_parameter(
                'maximum_vertical_acceleration'
            ).value,
            sea_surface_z=self.get_parameter('sea_surface_z').value,
            reserve_clearance=self.get_parameter('reserve_clearance').value,
            safety_response_delay=self.get_parameter(
                'safety_response_delay'
            ).value,
            vertical_braking_acceleration=self.get_parameter(
                'vertical_braking_acceleration'
            ).value,
            control_dt=1.0 / control_rate,
        )

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.uav_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.uav_callback,
            state_qos,
        )
        self.trajectory_sub = self.create_subscription(
            InterceptTrajectory,
            '/planning/intercept_trajectory',
            self.trajectory_callback,
            reliable_qos,
        )
        self.prediction_sub = self.create_subscription(
            TargetPrediction,
            '/planning/target_prediction',
            self.prediction_callback,
            state_qos,
        )
        self.planner_diagnostic_sub = self.create_subscription(
            PlannerDiagnostic,
            '/planning/diagnostic',
            self.planner_diagnostic_callback,
            reliable_qos,
        )
        self.mission_sub = self.create_subscription(
            MissionState,
            '/mission/state',
            self.mission_callback,
            reliable_qos,
        )
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10,
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10,
        )
        self.reference_pub = self.create_publisher(
            TrajectorySetpoint,
            '/control/reference',
            reliable_qos,
        )
        self.diagnostic_pub = self.create_publisher(
            ControllerDiagnostic,
            '/control/diagnostic',
            reliable_qos,
        )
        self.timer = self.create_timer(1.0 / control_rate, self.timer_callback)

        self.latest_state = None
        self.latest_prediction = None
        self.latest_planner_diagnostic = None
        self.mission_id = 0
        self.mission_state = MissionState.INIT
        self.last_rejection = TrajectoryRejectReason.NONE
        self.last_callback_time = 0.0
        self.get_logger().info(
            'Trajectory tracker ready | rate='
            f'{control_rate:.1f} Hz | plan age<='
            f'{self.tracker.maximum_plan_age:.3f} s | vxy<='
            f'{self.tracker.maximum_horizontal_speed:.1f} m/s | vz<='
            f'{self.tracker.maximum_vertical_speed:.1f} m/s'
        )

    def _ros_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _timestamp_us(self):
        return self.get_clock().now().nanoseconds // 1000

    def uav_callback(self, message):
        self.latest_state = tracker_state_from_message(
            message,
            self._ros_seconds(),
        )

    def prediction_callback(self, message):
        if message.valid and message.frame_id == self.expected_frame_id:
            self.latest_prediction = message

    def planner_diagnostic_callback(self, message):
        self.latest_planner_diagnostic = message

    def mission_callback(self, message):
        new_mission_id = int(message.mission_id)
        if new_mission_id != self.mission_id:
            self.tracker.reset()
            self.last_rejection = TrajectoryRejectReason.NONE
        self.mission_id = new_mission_id
        self.mission_state = int(message.state)

    def trajectory_callback(self, message):
        started = time.perf_counter()
        rejection = TrajectoryRejectReason.INVALID_TRAJECTORY
        try:
            trajectory = trajectory_from_message(message)
            if self.latest_state is None or self.latest_prediction is None:
                rejection = TrajectoryRejectReason.PREDICTION_MISMATCH
            elif (
                int(self.latest_prediction.mission_id) != self.mission_id
                or int(self.latest_prediction.sequence_id)
                != trajectory.prediction_sequence_id
            ):
                rejection = TrajectoryRejectReason.PREDICTION_MISMATCH
            else:
                endpoint = prediction_endpoint_from_message(
                    self.latest_prediction,
                    trajectory.duration,
                )
                rejection = self.tracker.accept(
                    trajectory,
                    self.latest_state,
                    self.mission_id,
                    prediction_sequence_id=(
                        self.latest_prediction.sequence_id
                    ),
                    target_endpoint=endpoint,
                )
        except (TypeError, ValueError):
            rejection = TrajectoryRejectReason.INVALID_TRAJECTORY
        self.last_rejection = rejection
        self.last_callback_time = time.perf_counter() - started

    def _publish_offboard_mode(self, timestamp_us):
        message = OffboardControlMode()
        message.timestamp = int(timestamp_us)
        message.position = True
        message.velocity = False
        message.acceleration = False
        message.attitude = False
        message.body_rate = False
        message.thrust_and_torque = False
        message.direct_actuator = False
        self.offboard_pub.publish(message)

    def _publish_diagnostic(self, now, command, status, callback_time):
        message = ControllerDiagnostic()
        message.stamp = _seconds_to_time(now)
        message.mission_id = self.mission_id
        active = self.tracker.active_trajectory
        if active is not None:
            message.plan_id = active.plan_id
            message.prediction_sequence_id = active.prediction_sequence_id
            message.source_age = max(now - active.source_stamp, 0.0)
            message.remaining_time = max(active.valid_until - now, 0.0)
        message.status = str(status)
        message.rejection_reason = self.last_rejection.value
        message.callback_compute_time = float(callback_time)
        if command is not None:
            message.safety_state = command.safety_state
            message.safety_margin = command.safety_margin
            message.reference_position.x = command.position[0]
            message.reference_position.y = command.position[1]
            message.reference_position.z = command.position[2]
            message.command_velocity.x = command.velocity[0]
            message.command_velocity.y = command.velocity[1]
            message.command_velocity.z = command.velocity[2]
            message.command_acceleration.x = command.acceleration[0]
            message.command_acceleration.y = command.acceleration[1]
            message.command_acceleration.z = command.acceleration[2]
        else:
            message.safety_state = 'HOLD'
        self.diagnostic_pub.publish(message)

    def timer_callback(self):
        started = time.perf_counter()
        now = self._ros_seconds()
        if self.latest_state is None:
            self._publish_diagnostic(
                now,
                None,
                'WAITING_FOR_STATE',
                time.perf_counter() - started,
            )
            return
        if now - self.latest_state.stamp > self.maximum_state_age:
            self._publish_diagnostic(
                now,
                None,
                'STATE_STALE',
                time.perf_counter() - started,
            )
            return
        if self.mission_state not in self.CONTROL_STATES:
            self._publish_diagnostic(
                now,
                None,
                'INACTIVE',
                time.perf_counter() - started,
            )
            return

        current = replace(self.latest_state, stamp=now)
        command = self.tracker.command(current, self.mission_id)
        timestamp_us = self._timestamp_us()
        self._publish_offboard_mode(timestamp_us)
        if command is None:
            setpoint = hold_setpoint(current, timestamp_us)
            status = 'NO_VALID_PLAN'
        else:
            setpoint = command_to_setpoint(command, timestamp_us)
            status = 'TRACKING'
        self.setpoint_pub.publish(setpoint)
        self.reference_pub.publish(setpoint)
        self._publish_diagnostic(
            now,
            command,
            status,
            time.perf_counter() - started,
        )


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
