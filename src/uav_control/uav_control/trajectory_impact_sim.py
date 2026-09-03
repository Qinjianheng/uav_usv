import csv
import math
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, Vector3
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String


class TrajectoryImpactSim(Node):
    """Simulation-only moving-target interception model.

    By default this node controls the PX4 Gazebo model. Pure numerical mode is
    available by setting enable_gazebo_control to false.
    """

    CSV_FIELDS = [
        'time',
        'uav_x',
        'uav_y',
        'uav_z',
        'uav_vx',
        'uav_vy',
        'target_x',
        'target_y',
        'target_vx',
        'target_vy',
        'distance',
        'intercept_x',
        'intercept_y',
        't_go',
        'uav_vz',
        'target_z',
        'target_vz',
        'horizontal_distance',
        'vertical_error',
        'intercept_z',
        'command_vx',
        'command_vy',
        'command_vz',
        'command_ax',
        'command_ay',
        'command_az',
        'uav_ax',
        'uav_ay',
        'uav_az',
        'terminal_mode',
        'phase',
    ]

    def __init__(self):
        super().__init__('trajectory_impact_sim')

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('offboard_prestream_time', 2.0)
        self.declare_parameter('px4_command_retry_time', 1.0)
        self.declare_parameter('max_speed', 8.0)
        self.declare_parameter('max_acceleration', 5.0)
        self.declare_parameter('max_prediction_time', 8.0)
        self.declare_parameter('terminal_radius', 3.0)
        self.declare_parameter('terminal_closing_speed', 3.0)
        self.declare_parameter('terminal_max_acceleration', 2.0)
        self.declare_parameter(
            'terminal_max_vertical_acceleration',
            1.0,
        )
        self.declare_parameter('impact_radius', 0.25)
        self.declare_parameter('max_sim_duration', 60.0)
        self.declare_parameter('log_directory', 'experiment_logs')
        self.declare_parameter('enable_gazebo_control', True)
        self.declare_parameter('flight_altitude', -5.0)
        self.declare_parameter('takeoff_tolerance', 0.5)
        self.declare_parameter('takeoff_settle_time', 1.0)
        self.declare_parameter('follow_distance', 20.0)
        self.declare_parameter('follow_position_gain', 0.8)
        self.declare_parameter('follow_max_closing_speed', 3.0)
        self.declare_parameter('follow_max_acceleration', 1.5)
        self.declare_parameter('altitude_velocity_gain', 1.0)
        self.declare_parameter('max_vertical_speed', 2.0)
        self.declare_parameter('max_vertical_acceleration', 2.0)
        self.declare_parameter('speed_guard_margin', 0.3)

        control_rate_hz = self.get_parameter(
            'control_rate_hz'
        ).value

        self.dt = 1.0 / max(float(control_rate_hz), 1.0)
        self.offboard_prestream_cycles = max(
            round(
                float(
                    self.get_parameter(
                        'offboard_prestream_time'
                    ).value
                ) / self.dt
            ),
            20,
        )
        self.px4_command_retry_cycles = max(
            round(
                float(
                    self.get_parameter(
                        'px4_command_retry_time'
                    ).value
                ) / self.dt
            ),
            1,
        )
        self.arm_request_start_cycle = (
            self.offboard_prestream_cycles
            + self.px4_command_retry_cycles
        )
        self.max_speed = max(
            float(self.get_parameter('max_speed').value),
            0.1
        )
        self.max_acceleration = max(
            float(self.get_parameter('max_acceleration').value),
            0.1
        )
        self.max_prediction_time = max(
            float(self.get_parameter('max_prediction_time').value),
            self.dt
        )
        self.terminal_radius = max(
            float(self.get_parameter('terminal_radius').value),
            0.1
        )
        self.terminal_closing_speed = max(
            float(self.get_parameter('terminal_closing_speed').value),
            0.1
        )
        self.terminal_max_acceleration = min(
            max(
                float(
                    self.get_parameter(
                        'terminal_max_acceleration'
                    ).value
                ),
                0.1,
            ),
            self.max_acceleration,
        )
        self.impact_radius = max(
            float(self.get_parameter('impact_radius').value),
            0.01
        )
        self.max_sim_duration = max(
            float(self.get_parameter('max_sim_duration').value),
            self.dt
        )
        self.enable_gazebo_control = bool(
            self.get_parameter('enable_gazebo_control').value
        )
        self.flight_altitude = float(
            self.get_parameter('flight_altitude').value
        )
        self.takeoff_tolerance = max(
            float(self.get_parameter('takeoff_tolerance').value),
            0.05
        )
        takeoff_settle_time = max(
            float(self.get_parameter('takeoff_settle_time').value),
            self.dt
        )
        self.takeoff_settle_cycles = max(
            round(takeoff_settle_time / self.dt),
            1
        )
        self.follow_distance = max(
            float(self.get_parameter('follow_distance').value),
            0.5
        )
        self.follow_position_gain = max(
            float(self.get_parameter('follow_position_gain').value),
            0.1
        )
        self.altitude_velocity_gain = max(
            float(self.get_parameter('altitude_velocity_gain').value),
            0.1
        )
        self.max_vertical_speed = max(
            float(self.get_parameter('max_vertical_speed').value),
            0.1
        )
        self.max_vertical_acceleration = max(
            float(
                self.get_parameter('max_vertical_acceleration').value
            ),
            0.1
        )
        self.terminal_max_vertical_acceleration = min(
            max(
                float(
                    self.get_parameter(
                        'terminal_max_vertical_acceleration'
                    ).value
                ),
                0.1,
            ),
            self.max_vertical_acceleration,
        )
        requested_guard_margin = max(
            float(self.get_parameter('speed_guard_margin').value),
            0.0
        )
        self.speed_guard_margin = min(
            requested_guard_margin,
            0.5 * self.max_speed
        )
        self.command_speed_limit = (
            self.max_speed - self.speed_guard_margin
        )
        self.follow_max_closing_speed = min(
            max(
                float(
                    self.get_parameter(
                        'follow_max_closing_speed'
                    ).value
                ),
                0.1,
            ),
            self.command_speed_limit,
        )
        self.follow_max_acceleration = min(
            max(
                float(
                    self.get_parameter(
                        'follow_max_acceleration'
                    ).value
                ),
                0.1,
            ),
            self.max_acceleration,
        )

        self.target_position_sub = self.create_subscription(
            Point,
            '/target/position',
            self.target_position_callback,
            10
        )
        self.target_velocity_sub = self.create_subscription(
            Vector3,
            '/target/velocity',
            self.target_velocity_callback,
            10
        )

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.vehicle_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.vehicle_position_callback,
            px4_qos
        )
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v4',
            self.vehicle_status_callback,
            px4_qos,
        )

        self.sim_position_pub = self.create_publisher(
            Point,
            '/simulation/impact/uav_position',
            10
        )
        self.sim_velocity_pub = self.create_publisher(
            Vector3,
            '/simulation/impact/uav_velocity',
            10
        )
        self.intercept_point_pub = self.create_publisher(
            Point,
            '/simulation/impact/intercept_point',
            10
        )
        self.hit_pub = self.create_publisher(
            Bool,
            '/simulation/impact/hit',
            10
        )
        self.command_sub = self.create_subscription(
            String,
            '/simulation/impact/command',
            self.command_callback,
            10
        )

        self.offboard_pub = None
        self.setpoint_pub = None
        self.command_pub = None

        if self.enable_gazebo_control:
            self.offboard_pub = self.create_publisher(
                OffboardControlMode,
                '/fmu/in/offboard_control_mode',
                10
            )
            self.setpoint_pub = self.create_publisher(
                TrajectorySetpoint,
                '/fmu/in/trajectory_setpoint',
                10
            )
            self.command_pub = self.create_publisher(
                VehicleCommand,
                '/fmu/in/vehicle_command',
                10
            )

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_vz = 0.0
        self.target_position_time_ns = None
        self.target_position_received = False
        self.target_velocity_received = False

        self.initial_uav_x = 0.0
        self.initial_uav_y = 0.0
        self.initial_uav_z = 0.0
        self.initial_uav_vx = 0.0
        self.initial_uav_vy = 0.0
        self.initial_uav_vz = 0.0
        self.uav_state_received = False
        self.vehicle_status_received = False
        self.offboard_active = False
        self.vehicle_armed = False

        self.sim_x = 0.0
        self.sim_y = 0.0
        self.sim_z = 0.0
        self.sim_vx = 0.0
        self.sim_vy = 0.0
        self.sim_vz = 0.0
        self.sim_time = 0.0
        self.started = False
        self.hit = False
        self.takeoff_requested = False
        self.takeoff_complete = False
        self.intercept_requested = False
        self.ready_for_takeoff_announced = False
        self.control_counter = 0
        self.takeoff_settle_counter = 0
        self.takeoff_x = None
        self.takeoff_y = None
        self.hold_x = None
        self.hold_y = None
        self.hold_z = None
        self.previous_relative_x = None
        self.previous_relative_y = None
        self.previous_relative_z = None
        self.previous_uav_x = None
        self.previous_uav_y = None
        self.previous_target_x = None
        self.previous_target_y = None
        self.previous_target_z = None
        self.command_vx = None
        self.command_vy = None
        self.command_vz = None
        self.command_ax = 0.0
        self.command_ay = 0.0
        self.command_az = 0.0
        self.measured_ax = 0.0
        self.measured_ay = 0.0
        self.measured_az = 0.0
        self.measured_acceleration = 0.0
        self.measured_vertical_acceleration = 0.0
        self.terminal_mode_active = False
        self.previous_measured_vx = None
        self.previous_measured_vy = None
        self.previous_measured_vz = None
        self.previous_velocity_timestamp_us = None
        self.last_constraint_warning_time = -math.inf

        self.csv_file = None
        self.csv_writer = None
        self.csv_path = None
        self.log_start_time_ns = None

        self.timer = self.create_timer(
            self.dt,
            self.timer_callback
        )

        if self.enable_gazebo_control:
            self.get_logger().warn(
                'GAZEBO CONTROL ENABLED: do not run another PX4 controller.'
            )
        else:
            self.get_logger().warn(
                'SIMULATION ONLY: this node does not publish PX4 commands.'
            )
        self.get_logger().info(
            'Waiting for target state and UAV initial state.'
        )

    def target_position_callback(self, msg):
        self.target_x = float(msg.x)
        self.target_y = float(msg.y)
        self.target_z = float(msg.z)
        self.target_position_time_ns = (
            self.get_clock().now().nanoseconds
        )
        self.target_position_received = True

    def target_velocity_callback(self, msg):
        self.target_vx = float(msg.x)
        self.target_vy = float(msg.y)
        self.target_vz = float(msg.z)
        self.target_velocity_received = True

    def command_callback(self, msg):
        command = msg.data.strip().upper()

        if command not in ('X', 'Y'):
            self.get_logger().warn(
                f'Ignoring unknown command: {msg.data!r}'
            )
            return

        if not self.enable_gazebo_control:
            self.get_logger().warn(
                f'{command} command is only used in Gazebo control mode.'
            )
            return

        if self.hit:
            self.get_logger().warn(
                f'{command} ignored: virtual impact already completed.'
            )
            return

        if command == 'X':
            if self.takeoff_requested:
                self.get_logger().info(
                    'X ignored: takeoff/follow sequence already requested.'
                )
                return

            state_ready = (
                self.target_position_received
                and self.target_velocity_received
                and self.uav_state_received
            )
            if not state_ready:
                self.get_logger().warn(
                    'X rejected: target/UAV state is not ready.'
                )
                return
            self.takeoff_requested = True
            self.log_start_time_ns = (
                self.get_clock().now().nanoseconds
            )
            self.begin_csv_logging()
            self.get_logger().info(
                'X accepted. Starting takeoff and follow sequence.'
            )
            return

        if not self.takeoff_requested:
            self.get_logger().warn(
                'Y rejected: send X first to start takeoff and follow.'
            )
            return

        if not self.takeoff_complete:
            self.get_logger().warn(
                'Y rejected: wait until FOLLOW MODE is active.'
            )
            return

        if self.started:
            self.get_logger().info(
                'Y ignored: interception is already active.'
            )
            return

        self.intercept_requested = True
        self.get_logger().info(
            'Y command received. Interception requested.'
        )

    def vehicle_position_callback(self, msg):
        current_vx = (
            float(msg.vx) if math.isfinite(msg.vx) else 0.0
        )
        current_vy = (
            float(msg.vy) if math.isfinite(msg.vy) else 0.0
        )
        current_vz = (
            float(msg.vz) if math.isfinite(msg.vz) else 0.0
        )
        timestamp_us = int(
            getattr(msg, 'timestamp_sample', 0)
            or getattr(msg, 'timestamp', 0)
        )

        if (
            self.previous_measured_vx is not None
            and timestamp_us > 0
            and self.previous_velocity_timestamp_us is not None
        ):
            sample_dt = (
                timestamp_us - self.previous_velocity_timestamp_us
            ) * 1e-6

            if 0.001 <= sample_dt <= 0.5:
                self.measured_ax = (
                    current_vx - self.previous_measured_vx
                ) / sample_dt
                self.measured_ay = (
                    current_vy - self.previous_measured_vy
                ) / sample_dt
                self.measured_acceleration = math.hypot(
                    self.measured_ax,
                    self.measured_ay,
                )
                if self.previous_measured_vz is not None:
                    self.measured_az = (
                        current_vz - self.previous_measured_vz
                    ) / sample_dt
                    self.measured_vertical_acceleration = abs(
                        self.measured_az
                    )

        self.previous_measured_vx = current_vx
        self.previous_measured_vy = current_vy
        self.previous_measured_vz = current_vz
        if timestamp_us > 0:
            self.previous_velocity_timestamp_us = timestamp_us

        self.initial_uav_x = float(msg.x)
        self.initial_uav_y = float(msg.y)
        self.initial_uav_z = float(msg.z)
        self.initial_uav_vx = current_vx
        self.initial_uav_vy = current_vy
        self.initial_uav_vz = current_vz
        self.uav_state_received = True

    def vehicle_status_callback(self, msg):
        previous_offboard = self.offboard_active
        previous_armed = self.vehicle_armed

        self.vehicle_status_received = True
        self.offboard_active = (
            msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )
        self.vehicle_armed = (
            msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )
        if self.offboard_active and not previous_offboard:
            self.get_logger().info('PX4 confirmed OFFBOARD mode.')
        if self.vehicle_armed and not previous_armed:
            self.get_logger().info('PX4 confirmed vehicle armed.')

    def timestamp(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def publish_offboard_mode(self):
        if self.offboard_pub is None:
            return

        msg = OffboardControlMode()
        msg.timestamp = self.timestamp()
        velocity_control = self.takeoff_complete and not self.hit
        msg.position = not velocity_control
        msg.velocity = velocity_control
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.offboard_pub.publish(msg)

    def publish_vehicle_command(self, command, param1, param2=0.0):
        if self.command_pub is None:
            return

        msg = VehicleCommand()
        msg.timestamp = self.timestamp()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def publish_gazebo_setpoint(
        self,
        position_x,
        position_y,
        position_z,
        velocity_x=None,
        velocity_y=None,
    ):
        if self.setpoint_pub is None:
            return

        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp()
        msg.position = [
            float(position_x),
            float(position_y),
            float(position_z),
        ]

        if velocity_x is None or velocity_y is None:
            msg.velocity = [math.nan, math.nan, math.nan]
        else:
            msg.velocity = [
                float(velocity_x),
                float(velocity_y),
                0.0,
            ]

        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = math.nan
        msg.yawspeed = math.nan
        self.setpoint_pub.publish(msg)

    def publish_gazebo_velocity_setpoint(
        self,
        velocity_x,
        velocity_y,
        velocity_z,
    ):
        if self.setpoint_pub is None:
            return

        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp()
        msg.position = [math.nan, math.nan, math.nan]
        msg.velocity = [
            float(velocity_x),
            float(velocity_y),
            float(velocity_z),
        ]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = math.nan
        msg.yawspeed = math.nan
        self.setpoint_pub.publish(msg)

    def monitor_actual_constraints(self):
        actual_speed = math.hypot(
            self.initial_uav_vx,
            self.initial_uav_vy,
        )
        actual_vertical_speed = abs(self.initial_uav_vz)
        speed_violation = actual_speed > self.max_speed + 0.1
        horizontal_acceleration_limit = (
            self.terminal_max_acceleration
            if self.terminal_mode_active
            else self.max_acceleration
        )
        vertical_acceleration_limit = (
            self.terminal_max_vertical_acceleration
            if self.terminal_mode_active
            else self.max_vertical_acceleration
        )
        acceleration_violation = (
            self.measured_acceleration
            > horizontal_acceleration_limit + 0.5
        )
        vertical_speed_violation = (
            actual_vertical_speed > self.max_vertical_speed + 0.1
        )
        vertical_acceleration_violation = (
            self.measured_vertical_acceleration
            > vertical_acceleration_limit + 0.5
        )

        if (
            (
                speed_violation
                or acceleration_violation
                or vertical_speed_violation
                or vertical_acceleration_violation
            )
            and self.sim_time - self.last_constraint_warning_time >= 1.0
        ):
            self.last_constraint_warning_time = self.sim_time
            self.get_logger().warn(
                f'ACTUAL CONSTRAINT WARNING | '
                f'speed={actual_speed:.2f}/{self.max_speed:.2f} m/s | '
                f'acceleration={self.measured_acceleration:.2f}/'
                f'{horizontal_acceleration_limit:.2f} m/s^2 | '
                f'vertical speed={actual_vertical_speed:.2f}/'
                f'{self.max_vertical_speed:.2f} m/s | '
                f'vertical acceleration='
                f'{self.measured_vertical_acceleration:.2f}/'
                f'{vertical_acceleration_limit:.2f} m/s^2'
            )

    def estimated_target_position(self):
        if self.target_position_time_ns is None:
            return self.target_x, self.target_y, self.target_z

        age = max(
            (
                self.get_clock().now().nanoseconds
                - self.target_position_time_ns
            ) * 1e-9,
            0.0
        )

        return (
            self.target_x + self.target_vx * age,
            self.target_y + self.target_vy * age,
            self.target_z + self.target_vz * age,
        )

    def calculate_follow_point(self, target_x, target_y):
        target_speed = math.hypot(
            self.target_vx,
            self.target_vy
        )

        if target_speed > 1e-6:
            direction_x = self.target_vx / target_speed
            direction_y = self.target_vy / target_speed
        else:
            direction_x = 1.0
            direction_y = 0.0

        return (
            target_x - self.follow_distance * direction_x,
            target_y - self.follow_distance * direction_y,
        )

    def plan_follow_velocity(self, follow_x, follow_y):
        """Approach the moving follow point without overshooting it."""
        error_x = follow_x - self.sim_x
        error_y = follow_y - self.sim_y
        distance = math.hypot(error_x, error_y)

        if distance <= 1e-6:
            return self.clamp_command_speed(
                self.target_vx,
                self.target_vy,
            )

        direction_x = error_x / distance
        direction_y = error_y / distance
        braking_speed = math.sqrt(
            2.0 * self.follow_max_acceleration * distance
        )
        closing_speed = min(
            self.follow_position_gain * distance,
            self.follow_max_closing_speed,
            braking_speed,
        )

        return self.clamp_command_speed(
            self.target_vx + closing_speed * direction_x,
            self.target_vy + closing_speed * direction_y,
        )

    def calculate_intercept_solution(
        self,
        uav_x,
        uav_y,
        uav_z,
        target_x,
        target_y,
        target_z,
    ):
        rx = target_x - uav_x
        ry = target_y - uav_y
        vx = self.target_vx
        vy = self.target_vy
        speed = (
            self.command_speed_limit
            if self.enable_gazebo_control
            else self.max_speed
        )

        a = vx * vx + vy * vy - speed * speed
        b = 2.0 * (rx * vx + ry * vy)
        c = rx * rx + ry * ry

        fallback = min(
            math.sqrt(c) / max(speed, 0.1),
            self.max_prediction_time
        )

        if c < 1e-12:
            t_go = 0.0
        elif abs(a) < 1e-9:
            t_go = -c / b if abs(b) >= 1e-9 else fallback
            if t_go <= 0.0:
                t_go = fallback
        else:
            discriminant = b * b - 4.0 * a * c

            if discriminant < 0.0:
                t_go = fallback
            else:
                sqrt_discriminant = math.sqrt(discriminant)
                roots = [
                    (-b + sqrt_discriminant) / (2.0 * a),
                    (-b - sqrt_discriminant) / (2.0 * a),
                ]
                positive_roots = [root for root in roots if root > 0.0]
                t_go = min(positive_roots) if positive_roots else fallback

        t_go = min(max(t_go, 0.0), self.max_prediction_time)
        relative_z = target_z - uav_z
        vertical_a = (
            self.target_vz * self.target_vz
            - self.max_vertical_speed * self.max_vertical_speed
        )
        vertical_b = 2.0 * relative_z * self.target_vz
        vertical_c = relative_z * relative_z

        if vertical_c < 1e-12:
            vertical_t_go = 0.0
        elif abs(vertical_a) < 1e-9:
            vertical_t_go = (
                -vertical_c / vertical_b
                if abs(vertical_b) >= 1e-9
                else self.max_prediction_time
            )
        else:
            vertical_discriminant = (
                vertical_b * vertical_b
                - 4.0 * vertical_a * vertical_c
            )
            if vertical_discriminant < 0.0:
                vertical_t_go = self.max_prediction_time
            else:
                vertical_root = math.sqrt(vertical_discriminant)
                vertical_roots = [
                    (-vertical_b + vertical_root) / (2.0 * vertical_a),
                    (-vertical_b - vertical_root) / (2.0 * vertical_a),
                ]
                positive_vertical_roots = [
                    root for root in vertical_roots if root > 0.0
                ]
                vertical_t_go = (
                    min(positive_vertical_roots)
                    if positive_vertical_roots
                    else self.max_prediction_time
                )

        t_go = min(
            max(t_go, vertical_t_go, 0.0),
            self.max_prediction_time,
        )
        intercept_x = target_x + self.target_vx * t_go
        intercept_y = target_y + self.target_vy * t_go
        intercept_z = target_z + self.target_vz * t_go

        return intercept_x, intercept_y, intercept_z, t_go

    def clamp_speed(self, vx, vy):
        speed = math.hypot(vx, vy)

        if speed <= self.max_speed:
            return vx, vy

        scale = self.max_speed / speed
        return vx * scale, vy * scale

    def clamp_command_speed(self, vx, vy):
        speed = math.hypot(vx, vy)

        if speed <= self.command_speed_limit:
            return vx, vy

        scale = self.command_speed_limit / speed
        return vx * scale, vy * scale

    def plan_velocity(self, target_x, target_y, target_z):
        dx = target_x - self.sim_x
        dy = target_y - self.sim_y
        dz = target_z - self.sim_z
        horizontal_distance = math.hypot(dx, dy)
        distance = math.sqrt(
            horizontal_distance * horizontal_distance + dz * dz
        )

        if horizontal_distance > 1e-9:
            horizontal_los_x = dx / horizontal_distance
            horizontal_los_y = dy / horizontal_distance
        else:
            horizontal_los_x = 0.0
            horizontal_los_y = 0.0

        if distance > 1e-9:
            los_x = dx / distance
            los_y = dy / distance
            los_z = dz / distance
        else:
            los_x = 0.0
            los_y = 0.0
            los_z = 0.0

        relative_closing_speed = (
            (self.sim_vx - self.target_vx) * los_x
            + (self.sim_vy - self.target_vy) * los_y
            + (self.sim_vz - self.target_vz) * los_z
        )
        braking_distance = max(
            (
                relative_closing_speed * relative_closing_speed
                - self.terminal_closing_speed
                * self.terminal_closing_speed
            ) / (2.0 * self.max_acceleration),
            0.0
        )
        terminal_entry_distance = max(
            self.terminal_radius,
            braking_distance + self.impact_radius
        )

        intercept_x, intercept_y, intercept_z, t_go = (
            self.calculate_intercept_solution(
                self.sim_x,
                self.sim_y,
                self.sim_z,
                target_x,
                target_y,
                target_z,
            )
        )

        terminal_mode = (
            distance <= terminal_entry_distance and distance > 1e-9
        )
        if terminal_mode:
            # Brake early enough to approach the requested non-zero impact
            # closing speed instead of crossing the virtual target at cruise.
            desired_vx = (
                self.target_vx
                + self.terminal_closing_speed * los_x
            )
            desired_vy = (
                self.target_vy
                + self.terminal_closing_speed * los_y
            )
        elif t_go > self.dt:
            desired_vx = (intercept_x - self.sim_x) / t_go
            desired_vy = (intercept_y - self.sim_y) / t_go
        elif horizontal_distance > 1e-9:
            desired_vx = self.max_speed * horizontal_los_x
            desired_vy = self.max_speed * horizontal_los_y
        else:
            desired_vx = self.target_vx
            desired_vy = self.target_vy

        desired_vx, desired_vy = self.clamp_speed(
            desired_vx,
            desired_vy
        )

        if terminal_mode:
            desired_vz = (
                self.target_vz
                + self.terminal_closing_speed * los_z
            )
        elif t_go > self.dt:
            desired_vz = (intercept_z - self.sim_z) / t_go
        elif abs(dz) > 1e-9:
            desired_vz = (
                self.max_vertical_speed
                if dz > 0.0
                else -self.max_vertical_speed
            )
        else:
            desired_vz = self.target_vz

        desired_vz = max(
            min(desired_vz, self.max_vertical_speed),
            -self.max_vertical_speed,
        )

        return (
            desired_vx,
            desired_vy,
            desired_vz,
            intercept_x,
            intercept_y,
            intercept_z,
            t_go,
            terminal_mode,
        )

    def acceleration_limited_velocity(
        self,
        desired_vx,
        desired_vy,
        acceleration_limit=None,
    ):
        if acceleration_limit is None:
            acceleration_limit = self.max_acceleration
        acceleration_limit = min(
            max(float(acceleration_limit), 0.1),
            self.max_acceleration,
        )

        if self.enable_gazebo_control:
            desired_vx, desired_vy = self.clamp_command_speed(
                desired_vx,
                desired_vy,
            )

            if self.command_vx is None or self.command_vy is None:
                base_vx, base_vy = self.clamp_command_speed(
                    self.sim_vx,
                    self.sim_vy,
                )
            else:
                base_vx = self.command_vx
                base_vy = self.command_vy
        else:
            base_vx = self.sim_vx
            base_vy = self.sim_vy

        delta_vx = desired_vx - base_vx
        delta_vy = desired_vy - base_vy
        delta_speed = math.hypot(delta_vx, delta_vy)
        max_delta_speed = acceleration_limit * self.dt

        if delta_speed > max_delta_speed:
            scale = max_delta_speed / delta_speed
            delta_vx *= scale
            delta_vy *= scale

        command_vx = base_vx + delta_vx
        command_vy = base_vy + delta_vy

        if self.enable_gazebo_control:
            command_vx, command_vy = self.clamp_command_speed(
                command_vx,
                command_vy,
            )
        else:
            command_vx, command_vy = self.clamp_speed(
                command_vx,
                command_vy,
            )

        self.command_vx = command_vx
        self.command_vy = command_vy
        self.command_ax = (command_vx - base_vx) / self.dt
        self.command_ay = (command_vy - base_vy) / self.dt

        return command_vx, command_vy

    def acceleration_limited_vertical_velocity(
        self,
        desired_vz,
        acceleration_limit=None,
    ):
        if acceleration_limit is None:
            acceleration_limit = self.max_vertical_acceleration
        acceleration_limit = min(
            max(float(acceleration_limit), 0.1),
            self.max_vertical_acceleration,
        )
        desired_vz = max(
            min(desired_vz, self.max_vertical_speed),
            -self.max_vertical_speed,
        )

        if self.enable_gazebo_control and self.command_vz is not None:
            base_vz = self.command_vz
        else:
            base_vz = self.sim_vz

        max_delta = acceleration_limit * self.dt
        delta_vz = max(
            min(desired_vz - base_vz, max_delta),
            -max_delta,
        )
        command_vz = max(
            min(base_vz + delta_vz, self.max_vertical_speed),
            -self.max_vertical_speed,
        )

        self.command_vz = command_vz
        self.command_az = (command_vz - base_vz) / self.dt

        return command_vz

    def impact_fraction(self, relative_start, relative_end):
        deltas = [
            end - start
            for start, end in zip(relative_start, relative_end)
        ]

        c = (
            sum(value * value for value in relative_start)
            - self.impact_radius * self.impact_radius
        )

        if c <= 0.0:
            return 0.0

        a = sum(delta * delta for delta in deltas)

        if a < 1e-12:
            return None

        b = 2.0 * sum(
            start * delta
            for start, delta in zip(relative_start, deltas)
        )
        discriminant = b * b - 4.0 * a * c

        if discriminant < 0.0:
            return None

        sqrt_discriminant = math.sqrt(discriminant)
        roots = [
            (-b - sqrt_discriminant) / (2.0 * a),
            (-b + sqrt_discriminant) / (2.0 * a),
        ]
        valid_roots = [root for root in roots if 0.0 <= root <= 1.0]

        return min(valid_roots) if valid_roots else None

    def open_csv_log(self):
        log_directory = Path(
            self.get_parameter(
                'log_directory'
            ).get_parameter_value().string_value
        ).expanduser()

        if not log_directory.is_absolute():
            log_directory = Path.cwd() / log_directory

        log_directory.mkdir(parents=True, exist_ok=True)
        filename = (
            'trajectory_impact_sim_'
            + datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            + '.csv'
        )
        self.csv_path = log_directory / filename
        self.csv_file = self.csv_path.open(
            mode='x',
            newline='',
            encoding='utf-8',
            buffering=1
        )
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(self.CSV_FIELDS)

    def begin_csv_logging(self):
        if self.csv_file is not None:
            return

        if self.log_start_time_ns is None:
            self.log_start_time_ns = (
                self.get_clock().now().nanoseconds
            )

        try:
            self.open_csv_log()
        except OSError as error:
            self.csv_file = None
            self.csv_writer = None
            self.csv_path = None
            self.get_logger().error(
                f'Failed to open CSV log: {error}'
            )
            return

        self.get_logger().info(
            f'CSV logging started at X command: {self.csv_path}'
        )

    def close_csv_log(self):
        if self.csv_file is None:
            return

        csv_path = self.csv_path

        try:
            self.csv_file.flush()
            self.csv_file.close()
        except OSError as error:
            message = f'Failed to close CSV log: {error}'
            if rclpy.ok():
                self.get_logger().error(message)
            else:
                print(message, flush=True)
        finally:
            self.csv_file = None
            self.csv_writer = None

        message = f'CSV log saved: {csv_path}'
        if rclpy.ok():
            self.get_logger().info(message)
        else:
            print(message, flush=True)

    def record_row(
        self,
        target_x,
        target_y,
        target_z,
        intercept_x,
        intercept_y,
        intercept_z,
        t_go,
        phase=None,
    ):
        if self.csv_writer is None:
            return

        if self.log_start_time_ns is None:
            elapsed_time = self.sim_time
        else:
            elapsed_time = max(
                (
                    self.get_clock().now().nanoseconds
                    - self.log_start_time_ns
                ) * 1e-9,
                0.0,
            )

        if phase is None:
            phase = (
                'intercept'
                if self.enable_gazebo_control
                else 'virtual_intercept'
            )

        horizontal_distance = math.hypot(
            target_x - self.sim_x,
            target_y - self.sim_y,
        )
        vertical_error = target_z - self.sim_z
        distance = math.sqrt(
            horizontal_distance * horizontal_distance
            + vertical_error * vertical_error
        )
        self.csv_writer.writerow([
            f'{elapsed_time:.6f}',
            f'{self.sim_x:.6f}',
            f'{self.sim_y:.6f}',
            f'{self.sim_z:.6f}',
            f'{self.sim_vx:.6f}',
            f'{self.sim_vy:.6f}',
            f'{target_x:.6f}',
            f'{target_y:.6f}',
            f'{self.target_vx:.6f}',
            f'{self.target_vy:.6f}',
            f'{distance:.6f}',
            f'{intercept_x:.6f}',
            f'{intercept_y:.6f}',
            f'{t_go:.6f}',
            f'{self.sim_vz:.6f}',
            f'{target_z:.6f}',
            f'{self.target_vz:.6f}',
            f'{horizontal_distance:.6f}',
            f'{vertical_error:.6f}',
            f'{intercept_z:.6f}',
            f'{self.command_vx if self.command_vx is not None else self.sim_vx:.6f}',
            f'{self.command_vy if self.command_vy is not None else self.sim_vy:.6f}',
            f'{self.command_vz if self.command_vz is not None else self.sim_vz:.6f}',
            f'{self.command_ax:.6f}',
            f'{self.command_ay:.6f}',
            f'{self.command_az:.6f}',
            f'{self.measured_ax:.6f}',
            f'{self.measured_ay:.6f}',
            f'{self.measured_az:.6f}',
            '1' if self.terminal_mode_active else '0',
            phase,
        ])

    def publish_simulation_state(
        self,
        intercept_x,
        intercept_y,
        intercept_z,
    ):
        position = Point()
        position.x = self.sim_x
        position.y = self.sim_y
        position.z = self.sim_z
        self.sim_position_pub.publish(position)

        velocity = Vector3()
        velocity.x = self.sim_vx
        velocity.y = self.sim_vy
        velocity.z = self.sim_vz
        self.sim_velocity_pub.publish(velocity)

        intercept = Point()
        intercept.x = intercept_x
        intercept.y = intercept_y
        intercept.z = intercept_z
        self.intercept_point_pub.publish(intercept)

    def start_simulation(self):
        self.sim_x = self.initial_uav_x
        self.sim_y = self.initial_uav_y
        self.sim_z = self.initial_uav_z
        self.sim_vx = self.initial_uav_vx
        self.sim_vy = self.initial_uav_vy
        self.sim_vz = self.initial_uav_vz
        self.sim_time = 0.0
        self.log_start_time_ns = self.get_clock().now().nanoseconds
        self.begin_csv_logging()

        self.started = True
        target_x, target_y, target_z = self.estimated_target_position()
        intercept_x, intercept_y, intercept_z, t_go = (
            self.calculate_intercept_solution(
                self.sim_x,
                self.sim_y,
                self.sim_z,
                target_x,
                target_y,
                target_z,
            )
        )
        self.record_row(
            target_x,
            target_y,
            target_z,
            intercept_x,
            intercept_y,
            intercept_z,
            t_go,
        )
        self.publish_simulation_state(
            intercept_x,
            intercept_y,
            intercept_z,
        )

        hit_message = Bool()
        hit_message.data = False
        self.hit_pub.publish(hit_message)

        self.get_logger().info(
            'Starting VIRTUAL TRAJECTORY INTERCEPTION'
        )
        if self.csv_path is not None:
            self.get_logger().info(
                f'CSV logging to: {self.csv_path}'
            )

    def start_gazebo_interception(self):
        self.sim_x = self.initial_uav_x
        self.sim_y = self.initial_uav_y
        self.sim_z = self.initial_uav_z
        self.sim_vx = self.initial_uav_vx
        self.sim_vy = self.initial_uav_vy
        self.sim_vz = self.initial_uav_vz
        self.sim_time = 0.0
        self.begin_csv_logging()

        self.started = True
        target_x, target_y, target_z = self.estimated_target_position()
        intercept_x, intercept_y, intercept_z, t_go = (
            self.calculate_intercept_solution(
                self.sim_x,
                self.sim_y,
                self.sim_z,
                target_x,
                target_y,
                target_z,
            )
        )

        self.previous_relative_x = target_x - self.sim_x
        self.previous_relative_y = target_y - self.sim_y
        self.previous_relative_z = target_z - self.sim_z
        self.previous_uav_x = self.sim_x
        self.previous_uav_y = self.sim_y
        self.previous_target_x = target_x
        self.previous_target_y = target_y
        self.previous_target_z = target_z

        self.record_row(
            target_x,
            target_y,
            target_z,
            intercept_x,
            intercept_y,
            intercept_z,
            t_go,
        )
        self.publish_simulation_state(
            intercept_x,
            intercept_y,
            intercept_z,
        )

        hit_message = Bool()
        hit_message.data = False
        self.hit_pub.publish(hit_message)

        self.get_logger().info(
            'Y accepted. Starting GAZEBO TRAJECTORY INTERCEPTION'
        )
        if self.csv_path is not None:
            self.get_logger().info(
                f'CSV logging to: {self.csv_path}'
            )

    def gazebo_timer_callback(self):
        state_ready = (
            self.target_position_received
            and self.target_velocity_received
            and self.uav_state_received
        )
        if not state_ready:
            return

        if self.takeoff_x is None:
            self.takeoff_x = self.initial_uav_x
            self.takeoff_y = self.initial_uav_y

        if not self.takeoff_requested:
            if not self.ready_for_takeoff_announced:
                self.ready_for_takeoff_announced = True
                self.get_logger().info(
                    'READY | waiting for X to take off and follow'
                )
            return

        self.publish_offboard_mode()
        self.control_counter += 1

        if self.hit:
            self.publish_gazebo_setpoint(
                self.hold_x,
                self.hold_y,
                self.hold_z,
            )
            return

        if not self.takeoff_complete:
            self.sim_x = self.initial_uav_x
            self.sim_y = self.initial_uav_y
            self.sim_z = self.initial_uav_z
            self.sim_vx = self.initial_uav_vx
            self.sim_vy = self.initial_uav_vy
            self.sim_vz = self.initial_uav_vz
            self.publish_gazebo_setpoint(
                self.takeoff_x,
                self.takeoff_y,
                self.flight_altitude,
            )

            mode_retry_due = (
                self.control_counter
                >= self.offboard_prestream_cycles
                and (
                    self.control_counter
                    - self.offboard_prestream_cycles
                ) % self.px4_command_retry_cycles == 0
            )
            if mode_retry_due and not self.offboard_active:
                self.get_logger().info('Requesting PX4 OFFBOARD mode')
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    1.0,
                    6.0,
                )

            arm_retry_due = (
                self.control_counter >= self.arm_request_start_cycle
                and (
                    self.control_counter
                    - self.arm_request_start_cycle
                ) % self.px4_command_retry_cycles == 0
            )
            if arm_retry_due and not self.vehicle_armed:
                self.get_logger().info('Requesting PX4 arming')
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    1.0,
                )
            target_x, target_y, target_z = (
                self.estimated_target_position()
            )
            intercept_x, intercept_y, intercept_z, t_go = (
                self.calculate_intercept_solution(
                    self.sim_x,
                    self.sim_y,
                    self.sim_z,
                    target_x,
                    target_y,
                    target_z,
                )
            )
            self.record_row(
                target_x,
                target_y,
                target_z,
                intercept_x,
                intercept_y,
                intercept_z,
                t_go,
                phase='takeoff',
            )

            altitude_ready = (
                abs(self.initial_uav_z - self.flight_altitude)
                <= self.takeoff_tolerance
                and abs(self.initial_uav_vz) <= 0.5
                and self.control_counter >= self.arm_request_start_cycle
            )

            if altitude_ready:
                self.takeoff_settle_counter += 1
            else:
                self.takeoff_settle_counter = 0

            if (
                self.takeoff_settle_counter
                >= self.takeoff_settle_cycles
            ):
                self.takeoff_complete = True
                self.command_vx, self.command_vy = (
                    self.clamp_command_speed(
                        self.initial_uav_vx,
                        self.initial_uav_vy,
                    )
                )
                self.command_vz = max(
                    min(
                        self.initial_uav_vz,
                        self.max_vertical_speed,
                    ),
                    -self.max_vertical_speed,
                )
                self.get_logger().info(
                    'Takeoff complete. Entering FOLLOW MODE.'
                )
                self.get_logger().info(
                    'Send Y to start interception.'
                )

            if self.control_counter % 40 == 0:
                self.get_logger().info(
                    f'TAKEOFF | z={self.initial_uav_z:.2f} m | '
                    f'target z={self.flight_altitude:.2f} m'
                )
            return

        if not self.started:
            self.terminal_mode_active = False
            target_x, target_y, target_z = (
                self.estimated_target_position()
            )
            follow_x, follow_y = self.calculate_follow_point(
                target_x,
                target_y,
            )
            self.sim_x = self.initial_uav_x
            self.sim_y = self.initial_uav_y
            self.sim_z = self.initial_uav_z
            self.sim_vx = self.initial_uav_vx
            self.sim_vy = self.initial_uav_vy
            self.sim_vz = self.initial_uav_vz

            desired_vx, desired_vy = self.plan_follow_velocity(
                follow_x,
                follow_y,
            )
            command_vx, command_vy = (
                self.acceleration_limited_velocity(
                    desired_vx,
                    desired_vy,
                    self.follow_max_acceleration,
                )
            )
            altitude_error = self.flight_altitude - self.sim_z
            desired_vz = max(
                min(
                    self.altitude_velocity_gain * altitude_error,
                    self.max_vertical_speed,
                ),
                -self.max_vertical_speed,
            )
            command_vz = self.acceleration_limited_vertical_velocity(
                desired_vz
            )
            self.publish_gazebo_velocity_setpoint(
                command_vx,
                command_vy,
                command_vz,
            )
            self.monitor_actual_constraints()
            self.publish_simulation_state(
                follow_x,
                follow_y,
                self.flight_altitude,
            )
            intercept_x, intercept_y, intercept_z, t_go = (
                self.calculate_intercept_solution(
                    self.sim_x,
                    self.sim_y,
                    self.sim_z,
                    target_x,
                    target_y,
                    target_z,
                )
            )
            self.record_row(
                target_x,
                target_y,
                target_z,
                intercept_x,
                intercept_y,
                intercept_z,
                t_go,
                phase='follow',
            )

            if self.intercept_requested:
                self.start_gazebo_interception()
                return

            if self.control_counter % 20 == 0:
                follow_error = math.hypot(
                    follow_x - self.sim_x,
                    follow_y - self.sim_y,
                )
                self.get_logger().info(
                    f'FOLLOW MODE | error={follow_error:.2f} m | '
                    f'speed={math.hypot(self.sim_vx, self.sim_vy):.2f} '
                    f'm/s | waiting for Y'
                )
            return

        self.sim_x = self.initial_uav_x
        self.sim_y = self.initial_uav_y
        self.sim_z = self.initial_uav_z
        self.sim_vx = self.initial_uav_vx
        self.sim_vy = self.initial_uav_vy
        self.sim_vz = self.initial_uav_vz
        target_x, target_y, target_z = self.estimated_target_position()
        current_relative = (
            target_x - self.sim_x,
            target_y - self.sim_y,
            target_z - self.sim_z,
        )
        previous_relative = (
            self.previous_relative_x,
            self.previous_relative_y,
            self.previous_relative_z,
        )
        hit_fraction = self.impact_fraction(
            previous_relative,
            current_relative,
        )

        if hit_fraction is not None:
            hit_target_x = (
                self.previous_target_x
                + hit_fraction
                * (target_x - self.previous_target_x)
            )
            hit_target_y = (
                self.previous_target_y
                + hit_fraction
                * (target_y - self.previous_target_y)
            )
            hit_target_z = (
                self.previous_target_z
                + hit_fraction
                * (target_z - self.previous_target_z)
            )
            self.hold_x = hit_target_x
            self.hold_y = hit_target_y
            self.hold_z = hit_target_z

            # The experiment ends at coordinate overlap. The measured
            # crossing is used for timing, then the final logged/held
            # 3-D position is aligned exactly with the target.
            self.sim_x = hit_target_x
            self.sim_y = hit_target_y
            self.sim_z = hit_target_z
            self.sim_time += self.dt * hit_fraction
            self.record_row(
                hit_target_x,
                hit_target_y,
                hit_target_z,
                hit_target_x,
                hit_target_y,
                hit_target_z,
                0.0,
            )
            self.publish_simulation_state(
                hit_target_x,
                hit_target_y,
                hit_target_z,
            )
            self.register_hit(
                hit_target_x,
                hit_target_y,
                hit_target_z,
            )
            self.publish_offboard_mode()
            self.publish_gazebo_setpoint(
                self.hold_x,
                self.hold_y,
                self.hold_z,
            )
            return

        (
            desired_vx,
            desired_vy,
            desired_vz,
            intercept_x,
            intercept_y,
            intercept_z,
            t_go,
            terminal_mode,
        ) = self.plan_velocity(target_x, target_y, target_z)
        self.terminal_mode_active = terminal_mode
        horizontal_acceleration_limit = (
            self.terminal_max_acceleration
            if terminal_mode
            else self.max_acceleration
        )
        vertical_acceleration_limit = (
            self.terminal_max_vertical_acceleration
            if terminal_mode
            else self.max_vertical_acceleration
        )
        command_vx, command_vy = self.acceleration_limited_velocity(
            desired_vx,
            desired_vy,
            horizontal_acceleration_limit,
        )
        command_vz = self.acceleration_limited_vertical_velocity(
            desired_vz,
            vertical_acceleration_limit,
        )

        self.publish_gazebo_velocity_setpoint(
            command_vx,
            command_vy,
            command_vz,
        )
        self.monitor_actual_constraints()

        self.sim_time += self.dt
        self.record_row(
            target_x,
            target_y,
            target_z,
            intercept_x,
            intercept_y,
            intercept_z,
            t_go,
        )
        self.publish_simulation_state(
            intercept_x,
            intercept_y,
            intercept_z,
        )

        self.previous_relative_x = current_relative[0]
        self.previous_relative_y = current_relative[1]
        self.previous_relative_z = current_relative[2]
        self.previous_uav_x = self.sim_x
        self.previous_uav_y = self.sim_y
        self.previous_target_x = target_x
        self.previous_target_y = target_y
        self.previous_target_z = target_z

        step_index = round(self.sim_time / self.dt)
        if step_index % round(1.0 / self.dt) == 0:
            horizontal_distance = math.hypot(
                current_relative[0],
                current_relative[1],
            )
            distance = math.sqrt(
                horizontal_distance * horizontal_distance
                + current_relative[2] * current_relative[2]
            )
            speed = math.hypot(self.sim_vx, self.sim_vy)
            command_speed = math.hypot(command_vx, command_vy)
            command_acceleration = math.hypot(
                self.command_ax,
                self.command_ay,
            )
            self.get_logger().info(
                f'Time={self.sim_time:.2f} s | '
                f'Distance={distance:.2f} m | '
                f'XY={horizontal_distance:.2f} m | '
                f'Z error={current_relative[2]:.2f} m | '
                f'T_go={t_go:.2f} s | '
                f'UAV speed={speed:.2f} m/s | '
                f'Command speed={command_speed:.2f} m/s | '
                f'Vz={self.sim_vz:.2f}/{command_vz:.2f} m/s | '
                f'Command acceleration={command_acceleration:.2f} m/s^2 | '
                f'Command az={self.command_az:.2f} m/s^2 | '
                f'UAV acceleration={self.measured_acceleration:.2f} m/s^2 | '
                f'Mode={"TERMINAL" if terminal_mode else "CRUISE"}'
            )

        if self.sim_time >= self.max_sim_duration:
            self.get_logger().warn(
                'Gazebo trajectory interception timed out.'
            )
            self.hold_x = self.sim_x
            self.hold_y = self.sim_y
            self.hold_z = self.sim_z
            self.close_csv_log()
            self.hit = True

    def register_hit(self, target_x, target_y, target_z):
        self.hit = True
        relative_speed = math.sqrt(
            (self.sim_vx - self.target_vx) ** 2
            + (self.sim_vy - self.target_vy) ** 2
            + (self.sim_vz - self.target_vz) ** 2
        )
        distance = math.sqrt(
            (target_x - self.sim_x) ** 2
            + (target_y - self.sim_y) ** 2
            + (target_z - self.sim_z) ** 2
        )

        hit_message = Bool()
        hit_message.data = True
        self.hit_pub.publish(hit_message)

        self.get_logger().info(
            '========================================'
        )
        self.get_logger().info('VIRTUAL IMPACT DETECTED')
        self.get_logger().info(
            f'Time = {self.sim_time:.3f} s | '
            f'Distance = {distance:.3f} m | '
            f'Relative speed = {relative_speed:.3f} m/s'
        )
        self.get_logger().info(
            '========================================'
        )
        self.close_csv_log()

    def timer_callback(self):
        if self.enable_gazebo_control:
            self.gazebo_timer_callback()
            return

        if self.hit:
            return

        ready = (
            self.target_position_received
            and self.target_velocity_received
            and self.uav_state_received
        )

        if not self.started:
            if ready:
                self.start_simulation()
            return

        target_x, target_y, target_z = self.estimated_target_position()
        relative_start = (
            target_x - self.sim_x,
            target_y - self.sim_y,
            target_z - self.sim_z,
        )

        (
            desired_vx,
            desired_vy,
            desired_vz,
            intercept_x,
            intercept_y,
            intercept_z,
            t_go,
            terminal_mode,
        ) = self.plan_velocity(target_x, target_y, target_z)
        self.terminal_mode_active = terminal_mode
        horizontal_acceleration_limit = (
            self.terminal_max_acceleration
            if terminal_mode
            else self.max_acceleration
        )
        vertical_acceleration_limit = (
            self.terminal_max_vertical_acceleration
            if terminal_mode
            else self.max_vertical_acceleration
        )

        old_x = self.sim_x
        old_y = self.sim_y
        old_vx = self.sim_vx
        old_vy = self.sim_vy
        old_z = self.sim_z
        old_vz = self.sim_vz
        new_vx, new_vy = self.acceleration_limited_velocity(
            desired_vx,
            desired_vy,
            horizontal_acceleration_limit,
        )
        new_vz = self.acceleration_limited_vertical_velocity(
            desired_vz,
            vertical_acceleration_limit,
        )

        acceleration_x = (new_vx - old_vx) / self.dt
        acceleration_y = (new_vy - old_vy) / self.dt
        acceleration_z = (new_vz - old_vz) / self.dt
        next_x = (
            old_x
            + old_vx * self.dt
            + 0.5 * acceleration_x * self.dt * self.dt
        )
        next_y = (
            old_y
            + old_vy * self.dt
            + 0.5 * acceleration_y * self.dt * self.dt
        )
        next_z = (
            old_z
            + old_vz * self.dt
            + 0.5 * acceleration_z * self.dt * self.dt
        )
        next_target_x = target_x + self.target_vx * self.dt
        next_target_y = target_y + self.target_vy * self.dt
        next_target_z = target_z + self.target_vz * self.dt
        relative_end = (
            next_target_x - next_x,
            next_target_y - next_y,
            next_target_z - next_z,
        )

        hit_fraction = self.impact_fraction(
            relative_start,
            relative_end
        )

        if hit_fraction is not None:
            partial_dt = self.dt * hit_fraction
            self.sim_x = (
                old_x
                + old_vx * partial_dt
                + 0.5 * acceleration_x * partial_dt * partial_dt
            )
            self.sim_y = (
                old_y
                + old_vy * partial_dt
                + 0.5 * acceleration_y * partial_dt * partial_dt
            )
            self.sim_z = (
                old_z
                + old_vz * partial_dt
                + 0.5 * acceleration_z * partial_dt * partial_dt
            )
            self.sim_vx = old_vx + acceleration_x * partial_dt
            self.sim_vy = old_vy + acceleration_y * partial_dt
            self.sim_vz = old_vz + acceleration_z * partial_dt
            self.sim_time += partial_dt
            hit_target_x = target_x + self.target_vx * partial_dt
            hit_target_y = target_y + self.target_vy * partial_dt
            hit_target_z = target_z + self.target_vz * partial_dt
            self.sim_x = hit_target_x
            self.sim_y = hit_target_y
            self.sim_z = hit_target_z
            self.record_row(
                hit_target_x,
                hit_target_y,
                hit_target_z,
                hit_target_x,
                hit_target_y,
                hit_target_z,
                0.0,
            )
            self.publish_simulation_state(
                hit_target_x,
                hit_target_y,
                hit_target_z,
            )
            self.register_hit(
                hit_target_x,
                hit_target_y,
                hit_target_z,
            )
            return

        self.sim_x = next_x
        self.sim_y = next_y
        self.sim_z = next_z
        self.sim_vx = new_vx
        self.sim_vy = new_vy
        self.sim_vz = new_vz
        self.sim_time += self.dt

        (
            next_intercept_x,
            next_intercept_y,
            next_intercept_z,
            next_t_go,
        ) = (
            self.calculate_intercept_solution(
                self.sim_x,
                self.sim_y,
                self.sim_z,
                next_target_x,
                next_target_y,
                next_target_z,
            )
        )
        self.record_row(
            next_target_x,
            next_target_y,
            next_target_z,
            next_intercept_x,
            next_intercept_y,
            next_intercept_z,
            next_t_go,
        )
        self.publish_simulation_state(
            next_intercept_x,
            next_intercept_y,
            next_intercept_z,
        )

        step_index = round(self.sim_time / self.dt)
        if step_index % round(1.0 / self.dt) == 0:
            distance = math.sqrt(
                (next_target_x - self.sim_x) ** 2
                + (next_target_y - self.sim_y) ** 2
                + (next_target_z - self.sim_z) ** 2
            )
            speed = math.sqrt(
                self.sim_vx * self.sim_vx
                + self.sim_vy * self.sim_vy
                + self.sim_vz * self.sim_vz
            )
            self.get_logger().info(
                f'Time={self.sim_time:.2f} s | '
                f'Distance={distance:.2f} m | '
                f'T_go={next_t_go:.2f} s | '
                f'Virtual UAV speed={speed:.2f} m/s'
            )

        if self.sim_time >= self.max_sim_duration:
            self.get_logger().warn(
                'Virtual interception timed out.'
            )
            self.close_csv_log()
            self.hit = True

    def destroy_node(self):
        self.close_csv_log()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = TrajectoryImpactSim()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
