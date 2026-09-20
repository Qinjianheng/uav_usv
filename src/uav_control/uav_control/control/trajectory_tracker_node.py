"""Independent 20 Hz polynomial tracker and final PX4 safety barrier."""

import math
import time
from dataclasses import replace

import rclpy
from builtin_interfaces.msg import Time
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool
from uav_usv_interfaces.msg import ControllerDiagnostic, InterceptTrajectory
from uav_usv_interfaces.msg import MissionState, PlannerDiagnostic
from uav_usv_interfaces.msg import TargetPrediction, TargetState

from .flight_guidance import FlightGuidanceCore, FlightKinematicState
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
        contact_stamp=_stamp_seconds(message.contact_stamp),
        selected_t_go=float(message.selected_t_go),
        remaining_t_go=float(message.remaining_t_go),
        terminal_mode=bool(message.terminal_mode),
        planned_capture_margin=float(message.planned_capture_margin),
    )


def trajectory_for_mission(trajectory, mission_state):
    """Make the mission's committed terminal state authoritative."""
    if int(mission_state) == MissionState.TERMINAL_MINCO:
        return replace(trajectory, terminal_mode=True)
    return trajectory


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


def flight_target_from_message(message):
    """Convert the launch-selected current target source for flight guidance."""
    values = (
        float(message.position.x),
        float(message.position.y),
        float(message.position.z),
        float(message.velocity.x),
        float(message.velocity.y),
        float(message.velocity.z),
    )
    if not message.valid or not all(math.isfinite(value) for value in values):
        raise ValueError('target state is invalid')
    return FlightKinematicState(values[:3], values[3:])


def _wrap_angle(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def target_facing_yaw(uav_position, target_position):
    """Return the local-NED heading that points from UAV to target."""
    return math.atan2(
        float(target_position[1]) - float(uav_position[1]),
        float(target_position[0]) - float(uav_position[0]),
    )


def rate_limited_target_yaw(previous_yaw, desired_yaw, maximum_rate, dt):
    """Move toward target yaw through the shortest wrapped angular delta."""
    desired_yaw = _wrap_angle(desired_yaw)
    if previous_yaw is None or not math.isfinite(float(previous_yaw)):
        return desired_yaw
    maximum_step = max(float(maximum_rate), 0.0) * max(float(dt), 0.0)
    delta = _wrap_angle(desired_yaw - float(previous_yaw))
    delta = max(min(delta, maximum_step), -maximum_step)
    return _wrap_angle(float(previous_yaw) + delta)


def command_to_setpoint(command, timestamp_us, yaw=math.nan, velocity_mode=False):
    """
    Map the post-guard command to a PX4 setpoint.

    ``velocity_mode`` sends the tracker's acceleration-limited velocity as the
    commanded setpoint with NaN position, so PX4 consumes the closing speed
    the tracker builds.  Position mode keeps the previous behaviour of sending
    the MINCO reference and is retained for callers that still want it.
    """
    message = TrajectorySetpoint()
    message.timestamp = int(timestamp_us)
    if velocity_mode:
        message.position = [math.nan, math.nan, math.nan]
        message.velocity = [float(value) for value in command.velocity]
        message.acceleration = [math.nan, math.nan, math.nan]
    else:
        message.position = [float(value) for value in command.position]
        message.velocity = [float(value) for value in command.velocity]
        message.acceleration = [float(value) for value in command.acceleration]
    message.jerk = [math.nan, math.nan, math.nan]
    message.yaw = math.nan if yaw is None else float(yaw)
    message.yawspeed = math.nan
    return message


def hold_setpoint(state, timestamp_us, yaw=math.nan):
    """Hold the last valid position without a high-speed fallback."""
    message = TrajectorySetpoint()
    message.timestamp = int(timestamp_us)
    message.position = [float(value) for value in state.position]
    message.velocity = [0.0, 0.0, 0.0]
    message.acceleration = [math.nan, math.nan, math.nan]
    message.jerk = [math.nan, math.nan, math.nan]
    message.yaw = math.nan if yaw is None else float(yaw)
    message.yawspeed = math.nan
    return message


def flight_command_to_setpoint(command, timestamp_us, yaw=math.nan):
    """Map non-MINCO flight guidance without mixing PX4 control modes."""
    message = TrajectorySetpoint()
    message.timestamp = int(timestamp_us)
    if command.mode == 'VELOCITY':
        message.position = [math.nan, math.nan, math.nan]
        message.velocity = [float(value) for value in command.velocity]
    elif command.mode == 'POSITION':
        message.position = [float(value) for value in command.position]
        message.velocity = [math.nan, math.nan, math.nan]
    else:
        raise ValueError(f'unsupported flight command mode: {command.mode}')
    message.acceleration = [math.nan, math.nan, math.nan]
    message.jerk = [math.nan, math.nan, math.nan]
    message.yaw = math.nan if yaw is None else float(yaw)
    message.yawspeed = math.nan
    return message


def prediction_endpoint_from_message(message, contact_stamp):
    """Interpolate a target endpoint at one absolute ROS contact time."""
    if not message.valid or not message.samples:
        raise ValueError('target prediction is unavailable')
    desired_time = float(contact_stamp) - _stamp_seconds(message.source_stamp)
    if not math.isfinite(desired_time) or desired_time < 0.0:
        raise ValueError('contact time precedes prediction source')
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
    if desired_time > _duration_seconds(samples[-1].relative_time) + 1e-9:
        raise ValueError('contact time exceeds prediction horizon')
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
    TERMINAL_STATES = {
        MissionState.CAPTURE,
        MissionState.FAILURE,
        MissionState.ABORTED,
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
        self.declare_parameter(
            'guidance_maximum_horizontal_speed',
            6.2,
        )
        self.declare_parameter('maximum_vertical_speed', 4.0)
        self.declare_parameter('maximum_horizontal_acceleration', 3.0)
        self.declare_parameter('maximum_vertical_acceleration', 3.0)
        self.declare_parameter('sea_surface_z', 0.0)
        self.declare_parameter('reserve_clearance', 0.07)
        self.declare_parameter('safety_response_delay', 0.15)
        self.declare_parameter('vertical_braking_acceleration', 2.5)
        self.declare_parameter('recovery_clearance', 0.5)
        self.declare_parameter('recovery_climb_speed', 1.0)
        self.declare_parameter('maximum_command_dt', 0.1)
        self.declare_parameter('maximum_actual_vertical_acceleration', 4.0)
        self.declare_parameter('maximum_state_age', 0.125)
        self.declare_parameter('use_velocity_control', True)
        self.declare_parameter('frame_id', 'local_ned')
        self.declare_parameter('target_state_topic', '/target/state')
        self.declare_parameter('offboard_prestream_time', 2.0)
        self.declare_parameter('px4_command_retry_time', 1.0)
        self.declare_parameter('vehicle_status_timeout', 2.0)
        self.declare_parameter('flight_altitude', -5.0)
        self.declare_parameter('takeoff_tolerance', 0.5)
        self.declare_parameter('takeoff_settle_time', 1.0)
        self.declare_parameter('takeoff_maximum_vertical_speed', 1.5)
        self.declare_parameter('takeoff_maximum_vertical_acceleration', 1.0)
        self.declare_parameter('takeoff_maximum_horizontal_acceleration', 1.5)
        self.declare_parameter('takeoff_horizontal_start_height', 0.5)
        self.declare_parameter('takeoff_horizontal_full_height', 1.5)
        self.declare_parameter('follow_distance', 5.0)
        self.declare_parameter('follow_position_gain', 0.8)
        self.declare_parameter('altitude_velocity_gain', 1.0)
        self.declare_parameter('approach_contact_clearance', 0.33)
        self.declare_parameter('approach_closing_speed', 1.5)
        self.declare_parameter('approach_horizon', 4.0)
        self.declare_parameter('approach_response_delay', 0.15)
        self.declare_parameter('terminal_replacement_position_error', 0.15)
        self.declare_parameter('max_observation_yaw_rate', 1.0)

        control_rate = float(self.get_parameter('control_rate_hz').value)
        if not math.isfinite(control_rate) or control_rate <= 0.0:
            raise ValueError('control_rate_hz must be finite and positive')
        self.maximum_state_age = float(
            self.get_parameter('maximum_state_age').value
        )
        self.expected_frame_id = str(self.get_parameter('frame_id').value)
        # Drive MINCO tracking with the tracker's acceleration-limited velocity
        # command.  Position mode re-sends a reference that is rebuilt from the
        # measured state every replan, so the position error never grows past a
        # few centimetres and the vehicle can only hold the speed it already
        # has; velocity mode is what lets it build the closing speed.
        self.use_velocity_control = bool(
            self.get_parameter('use_velocity_control').value
        )
        self.target_state_topic = str(
            self.get_parameter('target_state_topic').value
        )
        self.offboard_prestream_cycles = max(
            int(round(
                self.get_parameter('offboard_prestream_time').value
                * control_rate
            )),
            1,
        )
        self.px4_command_retry_cycles = max(
            int(round(
                self.get_parameter('px4_command_retry_time').value
                * control_rate
            )),
            1,
        )
        self.vehicle_status_timeout = float(
            self.get_parameter('vehicle_status_timeout').value
        )
        self.terminal_replacement_position_error = float(
            self.get_parameter('terminal_replacement_position_error').value
        )
        self.max_observation_yaw_rate = float(
            self.get_parameter('max_observation_yaw_rate').value
        )
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
            approach_contact_clearance=self.get_parameter(
                'approach_contact_clearance'
            ).value,
            approach_closing_speed=self.get_parameter(
                'approach_closing_speed'
            ).value,
            approach_horizon=self.get_parameter('approach_horizon').value,
            approach_response_delay=self.get_parameter(
                'approach_response_delay'
            ).value,
            control_dt=1.0 / control_rate,
            recovery_clearance=self.get_parameter(
                'recovery_clearance'
            ).value,
            recovery_climb_speed=self.get_parameter(
                'recovery_climb_speed'
            ).value,
            maximum_command_dt=self.get_parameter(
                'maximum_command_dt'
            ).value,
            maximum_actual_vertical_acceleration=self.get_parameter(
                'maximum_actual_vertical_acceleration'
            ).value,
        )
        self.flight_guidance = FlightGuidanceCore(
            flight_altitude=self.get_parameter('flight_altitude').value,
            takeoff_tolerance=self.get_parameter('takeoff_tolerance').value,
            takeoff_settle_time=self.get_parameter(
                'takeoff_settle_time'
            ).value,
            takeoff_maximum_vertical_speed=self.get_parameter(
                'takeoff_maximum_vertical_speed'
            ).value,
            takeoff_maximum_vertical_acceleration=self.get_parameter(
                'takeoff_maximum_vertical_acceleration'
            ).value,
            takeoff_maximum_horizontal_acceleration=self.get_parameter(
                'takeoff_maximum_horizontal_acceleration'
            ).value,
            takeoff_horizontal_start_height=self.get_parameter(
                'takeoff_horizontal_start_height'
            ).value,
            takeoff_horizontal_full_height=self.get_parameter(
                'takeoff_horizontal_full_height'
            ).value,
            follow_distance=self.get_parameter('follow_distance').value,
            follow_position_gain=self.get_parameter(
                'follow_position_gain'
            ).value,
            altitude_velocity_gain=self.get_parameter(
                'altitude_velocity_gain'
            ).value,
            maximum_horizontal_speed=self.get_parameter(
                'guidance_maximum_horizontal_speed'
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
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.uav_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.uav_callback,
            state_qos,
        )
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v4',
            self.vehicle_status_callback,
            state_qos,
        )
        self.target_state_sub = self.create_subscription(
            TargetState,
            self.target_state_topic,
            self.target_state_callback,
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
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
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
        self.flight_ready_pub = self.create_publisher(
            Bool,
            '/simulation/impact/flight_ready',
            latched_qos,
        )
        self.takeoff_complete_pub = self.create_publisher(
            Bool,
            '/control/takeoff_complete',
            latched_qos,
        )
        self.far_guidance_pub = self.create_publisher(
            Bool,
            '/control/far_guidance_available',
            reliable_qos,
        )
        self.timer = self.create_timer(1.0 / control_rate, self.timer_callback)

        self.latest_state = None
        self.latest_prediction = None
        self.latest_target_state = None
        self.latest_planner_diagnostic = None
        self.vehicle_status_stamp = None
        self.offboard_active = False
        self.vehicle_armed = False
        self.mission_id = 0
        self.mission_state = MissionState.INIT
        self.mission_state_name = 'INIT'
        self.last_rejection = TrajectoryRejectReason.NONE
        self.last_callback_time = 0.0
        self.control_counter = 0
        self.preflight_counter = 0
        self.last_timer_stamp = None
        self.flight_ready = False
        self.takeoff_complete_sent = False
        self.last_target_yaw = None
        self.terminal_hold_position = None
        self.terminal_mode_latched = False
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

    def vehicle_status_callback(self, message):
        self.vehicle_status_stamp = self._ros_seconds()
        self.offboard_active = (
            message.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )
        self.vehicle_armed = (
            message.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )

    def target_state_callback(self, message):
        if message.frame_id != self.expected_frame_id:
            return
        try:
            self.latest_target_state = flight_target_from_message(message)
        except ValueError:
            self.latest_target_state = None

    def prediction_callback(self, message):
        if message.valid and message.frame_id == self.expected_frame_id:
            self.latest_prediction = message

    def planner_diagnostic_callback(self, message):
        self.latest_planner_diagnostic = message

    def mission_callback(self, message):
        new_mission_id = int(message.mission_id)
        if new_mission_id != self.mission_id:
            self.tracker.reset()
            self.flight_guidance.reset()
            self.last_rejection = TrajectoryRejectReason.NONE
            self.takeoff_complete_sent = False
            self.last_target_yaw = None
            self.terminal_hold_position = None
            self.terminal_mode_latched = False
        self.mission_id = new_mission_id
        self.mission_state = int(message.state)
        self.mission_state_name = str(message.state_name) or 'INIT'
        if self.mission_state == MissionState.TERMINAL_MINCO:
            self.terminal_mode_latched = True
        if self.mission_state in self.TERMINAL_STATES:
            self.tracker.reset()
            self.terminal_mode_latched = False
            if self.latest_state is not None:
                safe_z = min(
                    self.latest_state.position[2],
                    self.tracker.sea_surface_z - self.tracker.reserve_clearance,
                )
                self.terminal_hold_position = (
                    self.latest_state.position[0],
                    self.latest_state.position[1],
                    safe_z,
                )

    def trajectory_callback(self, message):
        started = time.perf_counter()
        attempted_plan_id = int(message.plan_id)
        rejection = TrajectoryRejectReason.INVALID_TRAJECTORY
        try:
            trajectory = trajectory_from_message(message)
            trajectory = trajectory_for_mission(
                trajectory,
                self.mission_state,
            )
        except (TypeError, ValueError):
            rejection = TrajectoryRejectReason.INVALID_TRAJECTORY
        else:
            if self.latest_state is None or self.latest_prediction is None:
                rejection = TrajectoryRejectReason.PREDICTION_MISMATCH
            elif int(self.latest_prediction.mission_id) != self.mission_id:
                rejection = TrajectoryRejectReason.PREDICTION_MISMATCH
            else:
                try:
                    endpoint = prediction_endpoint_from_message(
                        self.latest_prediction,
                        (
                            trajectory.contact_stamp
                            if trajectory.contact_stamp > 0.0
                            else trajectory.source_stamp + trajectory.duration
                        ),
                    )
                except (TypeError, ValueError):
                    rejection = TrajectoryRejectReason.TARGET_ENDPOINT_MISMATCH
                else:
                    rejection = self.tracker.accept(
                        trajectory,
                        self.latest_state,
                        self.mission_id,
                        prediction_sequence_id=None,
                        target_endpoint=endpoint,
                        maximum_position_error=(
                            self.terminal_replacement_position_error
                            if self.mission_state == MissionState.TERMINAL_MINCO
                            else None
                        ),
                    )
                    if (
                        rejection == TrajectoryRejectReason.NONE
                        and trajectory.terminal_mode
                    ):
                        self.terminal_mode_latched = True
        self.last_rejection = rejection
        self.last_callback_time = time.perf_counter() - started
        self._publish_diagnostic(
            self._ros_seconds(),
            None,
            (
                'PLAN_ACCEPTED'
                if rejection == TrajectoryRejectReason.NONE
                else 'PLAN_REJECTED'
            ),
            self.last_callback_time,
            attempted_plan_id=attempted_plan_id,
        )

    def _publish_offboard_mode(self, timestamp_us, velocity_control=False):
        message = OffboardControlMode()
        message.timestamp = int(timestamp_us)
        message.position = not bool(velocity_control)
        message.velocity = bool(velocity_control)
        message.acceleration = False
        message.attitude = False
        message.body_rate = False
        message.thrust_and_torque = False
        message.direct_actuator = False
        self.offboard_pub.publish(message)

    def _publish_vehicle_command(self, command, param1, param2=0.0):
        message = VehicleCommand()
        message.timestamp = self._timestamp_us()
        message.command = int(command)
        message.param1 = float(param1)
        message.param2 = float(param2)
        message.target_system = 1
        message.target_component = 1
        message.source_system = 1
        message.source_component = 1
        message.from_external = True
        self.vehicle_command_pub.publish(message)

    @staticmethod
    def _publish_bool(publisher, value):
        message = Bool()
        message.data = bool(value)
        publisher.publish(message)

    def _request_flight_mode(self):
        retry_due = self.control_counter % self.px4_command_retry_cycles == 0
        if not retry_due:
            return
        if not self.offboard_active:
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0,
            )
        if not self.vehicle_armed:
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0,
            )

    def _publish_diagnostic(
        self,
        now,
        command,
        status,
        callback_time,
        attempted_plan_id=0,
    ):
        message = ControllerDiagnostic()
        message.stamp = _seconds_to_time(now)
        message.mission_id = self.mission_id
        message.attempted_plan_id = int(attempted_plan_id)
        active = self.tracker.active_trajectory
        if active is not None:
            message.plan_id = active.plan_id
            message.prediction_sequence_id = active.prediction_sequence_id
            message.trajectory_age = max(now - active.source_stamp, 0.0)
            message.source_age = message.trajectory_age
            message.remaining_time = max(active.valid_until - now, 0.0)
            message.selected_t_go = active.selected_t_go
            message.contact_stamp = _seconds_to_time(active.contact_stamp)
            message.remaining_t_go = max(active.contact_stamp - now, 0.0)
            message.terminal_mode = active.terminal_mode
            message.planned_capture_margin = active.planned_capture_margin
        if self.latest_prediction is not None:
            message.prediction_age = max(
                now - _stamp_seconds(self.latest_prediction.source_stamp),
                0.0,
            )
        message.terminal_mode = bool(
            message.terminal_mode or self.terminal_mode_latched
        )
        if self.latest_target_state is not None and self.latest_state is not None:
            relative = tuple(
                target - current
                for target, current in zip(
                    self.latest_target_state.position,
                    self.latest_state.position,
                )
            )
            message.target_distance = math.sqrt(
                sum(value * value for value in relative)
            )
        message.target_yaw = (
            float(self.last_target_yaw)
            if self.last_target_yaw is not None
            else math.nan
        )
        message.status = str(status)
        message.rejection_reason = self.last_rejection.value
        message.trajectory_replaced = bool(
            self.tracker.last_replacement_performed
        )
        message.handover_position_error = float(
            self.tracker.last_handover_position_error
        )
        message.handover_velocity_error = float(
            self.tracker.last_handover_velocity_error
        )
        message.handover_acceleration_error = float(
            self.tracker.last_handover_acceleration_error
        )
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
            self._publish_bool(self.flight_ready_pub, False)
            self._publish_diagnostic(
                now,
                None,
                'WAITING_FOR_STATE',
                time.perf_counter() - started,
            )
            return
        if now - self.latest_state.stamp > self.maximum_state_age:
            self._publish_bool(self.flight_ready_pub, False)
            self._publish_diagnostic(
                now,
                None,
                'STATE_STALE',
                time.perf_counter() - started,
            )
            return
        current = replace(self.latest_state, stamp=now)
        flight_state = FlightKinematicState(
            current.position,
            current.velocity,
        )
        timestamp_us = self._timestamp_us()
        dt = self.tracker.control_dt
        if self.last_timer_stamp is not None:
            measured_dt = now - self.last_timer_stamp
            if 0.001 <= measured_dt <= 0.25:
                dt = measured_dt
        self.last_timer_stamp = now
        self.control_counter += 1

        target_yaw = self.last_target_yaw
        if self.latest_target_state is not None:
            desired_yaw = target_facing_yaw(
                current.position,
                self.latest_target_state.position,
            )
            target_yaw = rate_limited_target_yaw(
                self.last_target_yaw,
                desired_yaw,
                self.max_observation_yaw_rate,
                dt,
            )
            self.last_target_yaw = target_yaw

        if self.mission_state in (MissionState.INIT, MissionState.GROUND_HOLD):
            flight_command = self.flight_guidance.command(
                self.mission_state_name,
                flight_state,
                self.latest_target_state,
                dt,
            )
            self._publish_offboard_mode(timestamp_us, velocity_control=False)
            setpoint = flight_command_to_setpoint(
                flight_command,
                timestamp_us,
                yaw=target_yaw,
            )
            self.preflight_counter += 1
            mode_retry_due = (
                self.preflight_counter >= self.offboard_prestream_cycles
                and (
                    self.preflight_counter - self.offboard_prestream_cycles
                ) % self.px4_command_retry_cycles == 0
            )
            if mode_retry_due and not self.offboard_active:
                self._publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    1.0,
                    6.0,
                )
            if (
                self.vehicle_armed
                and self.preflight_counter % self.px4_command_retry_cycles == 0
            ):
                self._publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    0.0,
                )
            status_fresh = (
                self.vehicle_status_stamp is not None
                and now - self.vehicle_status_stamp
                <= self.vehicle_status_timeout
            )
            ready = self.offboard_active and not self.vehicle_armed and status_fresh
            self._publish_bool(self.flight_ready_pub, ready)
            if ready and not self.flight_ready:
                self.get_logger().info(
                    'FLIGHT READY | PX4 is disarmed in OFFBOARD ground hold'
                )
            self.flight_ready = ready
            status = self.mission_state_name
            command = flight_command
        elif self.mission_state in (
            MissionState.TAKEOFF,
            MissionState.FOLLOW,
            MissionState.FAR_GUIDANCE,
        ):
            self._publish_bool(self.flight_ready_pub, False)
            self._request_flight_mode()
            flight_command = self.flight_guidance.command(
                self.mission_state_name,
                flight_state,
                self.latest_target_state,
                dt,
            )
            velocity_control = flight_command.mode == 'VELOCITY'
            self._publish_offboard_mode(
                timestamp_us,
                velocity_control=velocity_control,
            )
            setpoint = flight_command_to_setpoint(
                flight_command,
                timestamp_us,
                yaw=target_yaw,
            )
            self._publish_bool(
                self.takeoff_complete_pub,
                flight_command.takeoff_complete,
            )
            self._publish_bool(
                self.far_guidance_pub,
                flight_command.far_guidance_available,
            )
            if (
                flight_command.takeoff_complete
                and not self.takeoff_complete_sent
            ):
                self.takeoff_complete_sent = True
                self.get_logger().info('TAKEOFF COMPLETE | entering FOLLOW')
            status = self.mission_state_name
            command = flight_command
        elif self.mission_state in self.CONTROL_STATES:
            self._publish_bool(self.flight_ready_pub, False)
            self._request_flight_mode()
            command = self.tracker.command(current, self.mission_id)
            velocity_mode = self.use_velocity_control
            if command is None:
                # Keep the recovery reference continuous instead of snapping
                # the position setpoint to the measured state, and climb out of
                # the sea margin while no plan is valid.
                command = self.tracker.recovery_command(current)
                setpoint = command_to_setpoint(
                    command,
                    timestamp_us,
                    yaw=target_yaw,
                    velocity_mode=velocity_mode,
                )
                self._publish_offboard_mode(
                    timestamp_us,
                    velocity_control=velocity_mode,
                )
                status = 'NO_VALID_PLAN'
            else:
                setpoint = command_to_setpoint(
                    command,
                    timestamp_us,
                    yaw=target_yaw,
                    velocity_mode=velocity_mode,
                )
                self._publish_offboard_mode(
                    timestamp_us,
                    velocity_control=velocity_mode,
                )
                status = 'TRACKING'
        else:
            self._publish_bool(self.flight_ready_pub, False)
            self._publish_offboard_mode(timestamp_us, velocity_control=False)
            hold_state = current
            if self.terminal_hold_position is not None:
                hold_state = replace(
                    current,
                    position=self.terminal_hold_position,
                    velocity=(0.0, 0.0, 0.0),
                )
            setpoint = hold_setpoint(
                hold_state,
                timestamp_us,
                yaw=target_yaw,
            )
            status = self.mission_state_name
            command = None
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
