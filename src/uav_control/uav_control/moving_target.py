import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Bool, String
from uav_control.figure_eight_trajectory import FigureEightTrajectory
from uav_usv_interfaces.msg import InterceptResult


class MovingTarget(Node):

    def __init__(self):
        super().__init__('moving_target')

        self.declare_parameter('initial_x', 20.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_z', 0.0)
        self.declare_parameter('velocity_x', 0.0)
        self.declare_parameter('velocity_y', 5.0)
        self.declare_parameter('velocity_z', 0.0)
        self.declare_parameter('trajectory_type', 'figure_eight')
        self.declare_parameter('horizontal_speed', 5.0)
        self.declare_parameter('figure_eight_x_amplitude', 40.0)
        self.declare_parameter('figure_eight_y_amplitude', 20.0)
        self.declare_parameter('vertical_oscillation_amplitude', 0.15)
        self.declare_parameter('vertical_oscillation_frequency', 0.25)
        self.declare_parameter('update_rate_hz', 20.0)
        self.declare_parameter('start_on_command', True)
        self.declare_parameter('enable_gazebo_visualization', True)
        self.declare_parameter('gazebo_world_name', 'default')
        self.declare_parameter('gazebo_entity_name', 'usv_target')
        self.declare_parameter('gazebo_sphere_diameter', 0.5)
        self.declare_parameter('gazebo_visual_height_offset', 0.42)
        self.declare_parameter('pause_gazebo_on_hit', True)

        self.position_pub = self.create_publisher(
            Point,
            '/target/position',
            10
        )

        self.velocity_pub = self.create_publisher(
            Vector3,
            '/target/velocity',
            10
        )

        self.hit_sub = self.create_subscription(
            Bool,
            '/simulation/impact/hit',
            self.hit_callback,
            10
        )
        self.result_sub = self.create_subscription(
            InterceptResult,
            '/simulation/impact/result',
            self.result_callback,
            10,
        )
        self.command_sub = self.create_subscription(
            String,
            '/simulation/impact/command',
            self.command_callback,
            10,
        )
        self.flight_ready_sub = self.create_subscription(
            Bool,
            '/simulation/impact/flight_ready',
            self.flight_ready_callback,
            10,
        )

        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.initial_z = float(self.get_parameter('initial_z').value)
        self.z = self.initial_z

        self.linear_vx = float(self.get_parameter('velocity_x').value)
        self.linear_vy = float(self.get_parameter('velocity_y').value)
        self.vx = self.linear_vx
        self.vy = self.linear_vy
        self.vz = float(self.get_parameter('velocity_z').value)
        self.trajectory_type = str(
            self.get_parameter('trajectory_type').value
        ).strip().lower()
        self.figure_eight_trajectory = None
        if self.trajectory_type == 'figure_eight':
            self.figure_eight_trajectory = FigureEightTrajectory(
                initial_x=self.x,
                initial_y=self.y,
                x_amplitude=self.get_parameter(
                    'figure_eight_x_amplitude'
                ).value,
                y_amplitude=self.get_parameter(
                    'figure_eight_y_amplitude'
                ).value,
                speed=self.get_parameter('horizontal_speed').value,
            )
            self.x, self.y, self.vx, self.vy = (
                self.figure_eight_trajectory.state()
            )
        elif self.trajectory_type != 'linear':
            raise ValueError(
                'trajectory_type must be "linear" or "figure_eight".'
            )
        self.vertical_oscillation_amplitude = float(
            self.get_parameter('vertical_oscillation_amplitude').value
        )
        self.vertical_oscillation_frequency = float(
            self.get_parameter('vertical_oscillation_frequency').value
        )
        self.vertical_angular_frequency = (
            2.0 * math.pi * self.vertical_oscillation_frequency
        )
        self.elapsed_time = 0.0

        self.hit = False
        self.flight_ready = False
        self.started = not bool(
            self.get_parameter('start_on_command').value
        )

        update_rate_hz = max(
            float(self.get_parameter('update_rate_hz').value),
            1.0,
        )
        self.dt = 1.0 / update_rate_hz

        self.timer = self.create_timer(
            self.dt,
            self.timer_callback
        )

        self.gazebo_visualizer = None
        self.gazebo_visualization_ready = False
        self.gazebo_world_paused = False
        self.last_gazebo_warning_time = -math.inf
        self.pause_gazebo_on_hit = bool(
            self.get_parameter('pause_gazebo_on_hit').value
        )
        self.gazebo_visual_height_offset = max(
            float(
                self.get_parameter(
                    'gazebo_visual_height_offset'
                ).value
            ),
            0.0,
        )
        if bool(
            self.get_parameter('enable_gazebo_visualization').value
        ):
            try:
                from uav_control.gazebo_target_visualizer import (
                    GazeboTargetVisualizer,
                )

                self.gazebo_visualizer = GazeboTargetVisualizer(
                    world_name=self.get_parameter(
                        'gazebo_world_name'
                    ).value,
                    entity_name=self.get_parameter(
                        'gazebo_entity_name'
                    ).value,
                    diameter=self.get_parameter(
                        'gazebo_sphere_diameter'
                    ).value,
                )
            except (ImportError, RuntimeError, ValueError) as exc:
                self.get_logger().warn(
                    'Gazebo target visualization disabled: '
                    f'{exc}'
                )

        self.get_logger().info(
            'Moving target node ready; waiting for X to start motion.'
            if not self.started
            else 'Moving target node started in immediate-motion mode.'
        )
        if self.figure_eight_trajectory is not None:
            x_min, x_max = self.figure_eight_trajectory.x_limits
            y_min, y_max = self.figure_eight_trajectory.y_limits
            self.get_logger().info(
                'USV figure-eight trajectory enabled | '
                f'speed={self.figure_eight_trajectory.speed:.2f} m/s | '
                f'X=[{x_min:.1f}, {x_max:.1f}] m | '
                f'Y=[{y_min:.1f}, {y_max:.1f}] m'
            )

    def command_callback(self, msg):
        if msg.data.strip().upper() == 'X' and not self.started:
            if not self.flight_ready:
                self.get_logger().warn(
                    'X ignored: UAV flight preparation is not ready.'
                )
                return
            self.started = True
            self.get_logger().info(
                'X received. Moving target motion started.'
            )

    def flight_ready_callback(self, msg):
        self.flight_ready = bool(msg.data)

    def hit_callback(self, msg):

        if msg.data and not self.hit:
            self.hit = True
            self.get_logger().info(
                'Virtual impact received. Target motion frozen.'
            )
            if self.pause_gazebo_on_hit:
                self.pause_gazebo_world()

    def result_callback(self, msg):
        if self.gazebo_world_paused:
            return

        self.hit = True
        self.get_logger().info(
            f'Interception finished with {msg.outcome}; '
            'target motion frozen.'
        )
        if self.pause_gazebo_on_hit:
            self.pause_gazebo_world()

    def timer_callback(self):

        # 更新目标位置
        if self.started and not self.hit:
            self.elapsed_time += self.dt
            if self.figure_eight_trajectory is None:
                self.x += self.linear_vx * self.dt
                self.y += self.linear_vy * self.dt
                self.vx = self.linear_vx
                self.vy = self.linear_vy
            else:
                self.x, self.y, self.vx, self.vy = (
                    self.figure_eight_trajectory.advance(self.dt)
                )
            self.z = (
                self.initial_z
                + self.vz * self.elapsed_time
                + self.vertical_oscillation_amplitude * math.sin(
                    self.vertical_angular_frequency * self.elapsed_time
                )
            )

        # 发布位置
        position_msg = Point()

        position_msg.x = self.x
        position_msg.y = self.y
        position_msg.z = self.z

        self.position_pub.publish(position_msg)

        # 发布速度
        velocity_msg = Vector3()

        if self.hit or not self.started:
            velocity_msg.x = 0.0
            velocity_msg.y = 0.0
            velocity_msg.z = 0.0
        else:
            velocity_msg.x = self.vx
            velocity_msg.y = self.vy
            velocity_msg.z = (
                self.vz
                + self.vertical_oscillation_amplitude
                * self.vertical_angular_frequency
                * math.cos(
                    self.vertical_angular_frequency * self.elapsed_time
                )
            )

        self.velocity_pub.publish(velocity_msg)

        if not self.gazebo_world_paused:
            self.update_gazebo_visualization()

    def pause_gazebo_world(self):
        if self.gazebo_visualizer is None:
            self.get_logger().warn(
                'Cannot pause Gazebo: target visualizer is unavailable.'
            )
            return

        try:
            paused = self.gazebo_visualizer.pause_world()
        except RuntimeError as exc:
            paused = False
            self.gazebo_visualizer.last_error = str(exc)

        if paused:
            self.gazebo_world_paused = True
            self.get_logger().info(
                'Gazebo world paused at the experiment terminal event.'
            )
        else:
            detail = (
                self.gazebo_visualizer.last_error
                or 'unknown Gazebo Transport error'
            )
            self.get_logger().warn(
                f'Unable to pause Gazebo after terminal event: {detail}'
            )

    def update_gazebo_visualization(self):
        if self.gazebo_visualizer is None:
            return

        try:
            updated = self.gazebo_visualizer.update(
                self.x,
                self.y,
                self.z - self.gazebo_visual_height_offset,
            )
        except RuntimeError as exc:
            updated = False
            self.gazebo_visualizer.last_error = str(exc)

        if updated:
            if not self.gazebo_visualization_ready:
                self.gazebo_visualization_ready = True
                self.get_logger().info(
                    'Gazebo red USV target sphere is visible.'
                )
            return

        warning_time = self.get_clock().now().nanoseconds * 1e-9
        if warning_time - self.last_gazebo_warning_time >= 5.0:
            self.last_gazebo_warning_time = warning_time
            detail = (
                self.gazebo_visualizer.last_error
                or 'waiting for Gazebo entity creation'
            )
            self.get_logger().warn(
                f'Gazebo red target sphere not ready: {detail}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MovingTarget()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
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
