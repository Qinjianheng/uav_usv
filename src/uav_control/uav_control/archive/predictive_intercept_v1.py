import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Vector3

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
)


class PredictiveIntercept(Node):

    def __init__(self):
        super().__init__('predictive_intercept')

        # ==============================
        # PX4 publishers
        # ==============================

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

        # ==============================
        # Target subscribers
        # ==============================

        self.target_pos_sub = self.create_subscription(
            Point,
            '/target/position',
            self.target_position_callback,
            10
        )

        self.target_vel_sub = self.create_subscription(
            Vector3,
            '/target/velocity',
            self.target_velocity_callback,
            10
        )

        # ==============================
        # PX4 QoS
        # ==============================

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.vehicle_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.vehicle_callback,
            px4_qos
        )

        # ==============================
        # Target state
        # ==============================

        self.target_x = 0.0
        self.target_y = 0.0

        self.target_vx = 0.0
        self.target_vy = 0.0

        self.target_received = False
        self.target_velocity_received = False

        # ==============================
        # UAV state
        # ==============================

        self.uav_x = 0.0
        self.uav_y = 0.0
        self.uav_z = 0.0

        self.uav_state_received = False

        # ==============================
        # Parameters
        # ==============================

        # 假定无人机水平截击速度
        self.interceptor_speed = 5.0

        # 避免预测太远
        self.max_prediction_time = 8.0

        # UAV飞行高度
        self.flight_altitude = -5.0

        # 截击成功距离
        self.intercept_distance = 0.5

        self.intercepted = False

        # ==============================
        # Timer
        # ==============================

        self.counter = 0

        # 4秒起飞阶段
        self.takeoff_counter = 80

        self.timer = self.create_timer(
            0.05,
            self.timer_callback
        )

        self.get_logger().info(
            'Predictive interception node started'
        )

    # ==================================
    # Timestamp
    # ==================================

    def timestamp(self):

        return int(
            self.get_clock().now().nanoseconds / 1000
        )

    # ==================================
    # Target callbacks
    # ==================================

    def target_position_callback(self, msg):

        self.target_x = msg.x
        self.target_y = msg.y

        self.target_received = True

    def target_velocity_callback(self, msg):

        self.target_vx = msg.x
        self.target_vy = msg.y

        self.target_velocity_received = True

    # ==================================
    # UAV state callback
    # ==================================

    def vehicle_callback(self, msg):

        self.uav_x = msg.x
        self.uav_y = msg.y
        self.uav_z = msg.z

        self.uav_state_received = True

    # ==================================
    # Solve interception time
    # ==================================

    def calculate_intercept_time(self):

        if (
            not self.target_received
            or not self.target_velocity_received
            or not self.uav_state_received
        ):
            return 0.0

        # 目标相对于无人机的位置
        rx = self.target_x - self.uav_x
        ry = self.target_y - self.uav_y

        vx = self.target_vx
        vy = self.target_vy

        vu = self.interceptor_speed

        # a*T^2 + b*T + c = 0
        a = vx * vx + vy * vy - vu * vu

        b = 2.0 * (
            rx * vx
            + ry * vy
        )

        c = (
            rx * rx
            + ry * ry
        )

        # 接近线性情况
        if abs(a) < 1e-6:

            if abs(b) < 1e-6:
                return 0.0

            t = -c / b

            if t > 0.0:
                return min(
                    t,
                    self.max_prediction_time
                )

            return 0.0

        discriminant = (
            b * b
            - 4.0 * a * c
        )

        # 没有实数截击解
        if discriminant < 0.0:

            distance = math.sqrt(c)

            return min(
                distance / vu,
                self.max_prediction_time
            )

        sqrt_d = math.sqrt(discriminant)

        t1 = (
            -b + sqrt_d
        ) / (2.0 * a)

        t2 = (
            -b - sqrt_d
        ) / (2.0 * a)

        positive_times = [
            t for t in [t1, t2]
            if t > 0.0
        ]

        if not positive_times:

            distance = math.sqrt(c)

            return min(
                distance / vu,
                self.max_prediction_time
            )

        return min(
            min(positive_times),
            self.max_prediction_time
        )

    # ==================================
    # Calculate predicted point
    # ==================================

    def calculate_intercept_point(self):

        t = self.calculate_intercept_time()

        predicted_x = (
            self.target_x
            + self.target_vx * t
        )

        predicted_y = (
            self.target_y
            + self.target_vy * t
        )

        return predicted_x, predicted_y, t

    # ==================================
    # PX4 heartbeat
    # ==================================

    def publish_offboard_mode(self):

        msg = OffboardControlMode()

        msg.timestamp = self.timestamp()

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False

        self.offboard_pub.publish(msg)

    # ==================================
    # Trajectory setpoint
    # ==================================

    def publish_setpoint(self):

        msg = TrajectorySetpoint()

        msg.timestamp = self.timestamp()

        # ------------------------------
        # 起飞阶段
        # ------------------------------

        if self.counter < self.takeoff_counter:

            msg.position = [
                0.0,
                0.0,
                self.flight_altitude
            ]

        # ------------------------------
        # Predictive Interception
        # ------------------------------

        else:

            if (
                self.target_received
                and self.target_velocity_received
                and self.uav_state_received
            ):

                px, py, _ = (
                    self.calculate_intercept_point()
                )

                msg.position = [
                    px,
                    py,
                    self.flight_altitude
                ]

            else:

                # 没收到目标信息时保持当前位置附近
                msg.position = [
                    0.0,
                    0.0,
                    self.flight_altitude
                ]

        msg.velocity = [
            math.nan,
            math.nan,
            math.nan
        ]

        msg.acceleration = [
            math.nan,
            math.nan,
            math.nan
        ]

        msg.yaw = math.nan

        self.setpoint_pub.publish(msg)

    # ==================================
    # OFFBOARD
    # ==================================

    def offboard_mode(self):

        msg = VehicleCommand()

        msg.timestamp = self.timestamp()

        msg.command = (
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        )

        msg.param1 = 1.0
        msg.param2 = 6.0

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        self.command_pub.publish(msg)

    # ==================================
    # ARM
    # ==================================

    def arm(self):

        msg = VehicleCommand()

        msg.timestamp = self.timestamp()

        msg.command = (
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        )

        msg.param1 = 1.0

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        self.command_pub.publish(msg)

    # ==================================
    # Interception check
    # ==================================

    def check_interception(self):

        if (
            not self.target_received
            or not self.uav_state_received
        ):
            return

        dx = self.target_x - self.uav_x
        dy = self.target_y - self.uav_y

        distance = math.sqrt(
            dx * dx
            + dy * dy
        )

        if self.counter % 20 == 0:

            px, py, t = (
                self.calculate_intercept_point()
            )

            self.get_logger().info(
                f'Distance={distance:.2f} m | '
                f'T_go={t:.2f} s | '
                f'Target=({self.target_x:.2f}, '
                f'{self.target_y:.2f}) | '
                f'Intercept=({px:.2f}, {py:.2f})'
            )

        if (
            distance < self.intercept_distance
            and not self.intercepted
        ):

            self.intercepted = True

            self.get_logger().info(
                '================================'
            )

            self.get_logger().info(
                f'INTERCEPTION SUCCESS! '
                f'Distance={distance:.2f} m'
            )

            self.get_logger().info(
                '================================'
            )

    # ==================================
    # Main timer
    # ==================================

    def timer_callback(self):

        self.publish_offboard_mode()

        self.publish_setpoint()

        self.counter += 1

        if self.counter == 20:

            self.get_logger().info(
                'Switching to OFFBOARD'
            )

            self.offboard_mode()

        if self.counter == 40:

            self.get_logger().info(
                'Arming'
            )

            self.arm()

        if self.counter == self.takeoff_counter:

            self.get_logger().info(
                'Starting PREDICTIVE INTERCEPTION'
            )

        self.check_interception()


def main(args=None):

    rclpy.init(args=args)

    node = PredictiveIntercept()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
