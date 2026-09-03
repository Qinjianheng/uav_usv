import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Bool


class MovingTarget(Node):

    def __init__(self):
        super().__init__('moving_target')

        self.declare_parameter('initial_x', 20.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_z', 0.0)
        self.declare_parameter('velocity_x', 0.0)
        self.declare_parameter('velocity_y', 1.0)
        self.declare_parameter('velocity_z', 0.0)
        self.declare_parameter('update_rate_hz', 20.0)

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

        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.z = float(self.get_parameter('initial_z').value)

        self.vx = float(self.get_parameter('velocity_x').value)
        self.vy = float(self.get_parameter('velocity_y').value)
        self.vz = float(self.get_parameter('velocity_z').value)

        self.hit = False

        update_rate_hz = max(
            float(self.get_parameter('update_rate_hz').value),
            1.0,
        )
        self.dt = 1.0 / update_rate_hz

        self.timer = self.create_timer(
            self.dt,
            self.timer_callback
        )

        self.get_logger().info(
            'Moving target node started'
        )

    def hit_callback(self, msg):

        if msg.data and not self.hit:
            self.hit = True
            self.get_logger().info(
                'Virtual impact received. Target motion frozen.'
            )

    def timer_callback(self):

        # 更新目标位置
        if not self.hit:
            self.x += self.vx * self.dt
            self.y += self.vy * self.dt
            self.z += self.vz * self.dt

        # 发布位置
        position_msg = Point()

        position_msg.x = self.x
        position_msg.y = self.y
        position_msg.z = self.z

        self.position_pub.publish(position_msg)

        # 发布速度
        velocity_msg = Vector3()

        if self.hit:
            velocity_msg.x = 0.0
            velocity_msg.y = 0.0
            velocity_msg.z = 0.0
        else:
            velocity_msg.x = self.vx
            velocity_msg.y = self.vy
            velocity_msg.z = self.vz

        self.velocity_pub.publish(velocity_msg)


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
