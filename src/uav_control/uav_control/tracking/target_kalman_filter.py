"""ROS 2 node that filters and predicts the moving target state."""

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.node import Node
from rclpy.time import Time
from uav_usv_interfaces.msg import TargetState

from .constant_velocity_kalman import ConstantVelocityKalmanFilter


class TargetKalmanFilterNode(Node):
    """Publish filtered target state for guidance and path planning."""

    def __init__(self):
        super().__init__('target_kalman_filter')

        self.declare_parameter('frame_id', 'local_ned')
        self.declare_parameter('input_topic', '/target/position')
        self.declare_parameter('state_topic', '/tracking/target_state')
        self.declare_parameter(
            'prediction_topic',
            '/tracking/predicted_position',
        )
        self.declare_parameter('update_rate_hz', 20.0)
        self.declare_parameter('process_acceleration_std', 0.5)
        self.declare_parameter('measurement_position_std', 0.02)
        self.declare_parameter('initial_velocity_std', 5.0)
        self.declare_parameter('prediction_horizon', 0.5)
        self.declare_parameter('measurement_timeout', 0.5)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.prediction_horizon = max(
            float(self.get_parameter('prediction_horizon').value),
            0.0,
        )
        self.measurement_timeout = max(
            float(self.get_parameter('measurement_timeout').value),
            0.01,
        )
        update_rate_hz = max(
            float(self.get_parameter('update_rate_hz').value),
            1.0,
        )

        self.filter = ConstantVelocityKalmanFilter(
            process_acceleration_std=self.get_parameter(
                'process_acceleration_std'
            ).value,
            measurement_position_std=self.get_parameter(
                'measurement_position_std'
            ).value,
            initial_velocity_std=self.get_parameter(
                'initial_velocity_std'
            ).value,
        )
        self.filter_time_ns = None
        self.last_measurement_time_ns = None

        input_topic = str(self.get_parameter('input_topic').value)
        state_topic = str(self.get_parameter('state_topic').value)
        prediction_topic = str(
            self.get_parameter('prediction_topic').value
        )
        self.measurement_sub = self.create_subscription(
            Point,
            input_topic,
            self.measurement_callback,
            10,
        )
        self.state_pub = self.create_publisher(
            TargetState,
            state_topic,
            10,
        )
        self.prediction_pub = self.create_publisher(
            PointStamped,
            prediction_topic,
            10,
        )
        self.timer = self.create_timer(
            1.0 / update_rate_hz,
            self.timer_callback,
        )

        self.get_logger().info(
            f'Target Kalman filter ready | input={input_topic} | '
            f'state={state_topic} | horizon={self.prediction_horizon:.2f} s'
        )

    def measurement_callback(self, msg):
        measurement = np.array([msg.x, msg.y, msg.z], dtype=float)
        if not np.all(np.isfinite(measurement)):
            self.get_logger().warn('Ignoring non-finite target position.')
            return

        now_ns = self.get_clock().now().nanoseconds
        if not self.filter.initialized:
            self.filter.initialize(measurement)
            self.filter_time_ns = now_ns
        else:
            self.predict_to(now_ns)
            self.filter.update(measurement)
        self.last_measurement_time_ns = now_ns

    def predict_to(self, timestamp_ns):
        if not self.filter.initialized or self.filter_time_ns is None:
            return
        dt = max((timestamp_ns - self.filter_time_ns) * 1e-9, 0.0)
        if dt > 0.0:
            self.filter.predict(dt)
            self.filter_time_ns = timestamp_ns

    def timer_callback(self):
        if not self.filter.initialized:
            return

        now = self.get_clock().now()
        self.predict_to(now.nanoseconds)
        measurement_age = (
            now.nanoseconds - self.last_measurement_time_ns
        ) * 1e-9
        valid = measurement_age <= self.measurement_timeout

        state_message = TargetState()
        state_message.stamp = now.to_msg()
        state_message.frame_id = self.frame_id
        state_message.position.x = float(self.filter.state[0])
        state_message.position.y = float(self.filter.state[1])
        state_message.position.z = float(self.filter.state[2])
        state_message.velocity.x = float(self.filter.state[3])
        state_message.velocity.y = float(self.filter.state[4])
        state_message.velocity.z = float(self.filter.state[5])
        state_message.acceleration.x = 0.0
        state_message.acceleration.y = 0.0
        state_message.acceleration.z = 0.0
        state_message.covariance = [
            float(value)
            for value in self.filter.covariance.reshape(-1)
        ]
        state_message.valid = valid
        self.state_pub.publish(state_message)

        predicted_position = self.filter.predicted_position(
            self.prediction_horizon
        )
        prediction_message = PointStamped()
        prediction_time_ns = (
            now.nanoseconds
            + round(self.prediction_horizon * 1e9)
        )
        prediction_message.header.stamp = Time(
            nanoseconds=prediction_time_ns
        ).to_msg()
        prediction_message.header.frame_id = self.frame_id
        prediction_message.point.x = float(predicted_position[0])
        prediction_message.point.y = float(predicted_position[1])
        prediction_message.point.z = float(predicted_position[2])
        self.prediction_pub.publish(prediction_message)


def main(args=None):
    rclpy.init(args=args)
    node = TargetKalmanFilterNode()

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
