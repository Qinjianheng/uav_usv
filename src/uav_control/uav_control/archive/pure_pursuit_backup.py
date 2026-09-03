import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
)


class PurePursuit(Node):

    def __init__(self):
        super().__init__('pure_pursuit')

        # =========================
        # PX4 publishers
        # =========================

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

        # =========================
        # Target subscriber
        # =========================

        self.target_sub = self.create_subscription(
            Point,
            '/target/position',
            self.target_callback,
            10
        )

        # 目标初始位置
        self.target_x = 20.0
        self.target_y = 0.0

        self.target_received = False

        # UAV飞行高度
        # PX4 NED：负数表示向上
        self.flight_altitude = -5.0

        # 20 Hz
        self.timer = self.create_timer(
            0.05,
            self.timer_callback
        )

        self.counter = 0

        # 前4秒先稳定在起飞点
        self.takeoff_counter = 80

        self.get_logger().info(
            'Pure pursuit node started'
        )

    def timestamp(self):

        return int(
            self.get_clock().now().nanoseconds / 1000
        )

    def target_callback(self, msg):

        self.target_x = msg.x
        self.target_y = msg.y

        if not self.target_received:

            self.get_logger().info(
                f'Target received: '
                f'x={msg.x:.2f}, y={msg.y:.2f}'
            )

            self.target_received = True

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

    def publish_setpoint(self):

        msg = TrajectorySetpoint()

        msg.timestamp = self.timestamp()

        # ==================================
        # 起飞阶段
        # ==================================

        if self.counter < self.takeoff_counter:

            msg.position = [
                0.0,
                0.0,
                self.flight_altitude
            ]

        # ==================================
        # 追击阶段
        # Pure Pursuit:
        # 直接追目标当前位置
        # ==================================

        else:

            msg.position = [
                self.target_x,
                self.target_y,
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

    def timer_callback(self):

        # Offboard heartbeat
        self.publish_offboard_mode()

        # Setpoint持续发送
        self.publish_setpoint()

        self.counter += 1

        # 1秒后进入OFFBOARD
        if self.counter == 20:

            self.get_logger().info(
                'Switching to OFFBOARD'
            )

            self.offboard_mode()

        # 2秒后解锁
        if self.counter == 40:

            self.get_logger().info(
                'Arming'
            )

            self.arm()

        # 4秒后开始追击
        if self.counter == self.takeoff_counter:

            if self.target_received:

                self.get_logger().info(
                    'Takeoff complete. '
                    'Starting PURE PURSUIT.'
                )

            else:

                self.get_logger().warn(
                    'No target received yet.'
                )


def main(args=None):

    rclpy.init(args=args)

    node = PurePursuit()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
