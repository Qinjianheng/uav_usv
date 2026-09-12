"""
Localize the simulated red USV target from aligned RGB and depth images.

The detector is intentionally limited to the red validation sphere.  The
camera geometry and filtering interface remain reusable when a non-cooperative
USV detector replaces the color mask.
"""

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from uav_usv_interfaces.msg import TargetObservation

from .front_tof_monitor import (
    camera_intrinsics,
    decode_float32_depth,
    red_pixel_mask,
)


def body_frd_to_ned_rotation(quaternion):
    """Return the body-FRD to local-NED rotation matrix used by PX4."""
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,):
        raise ValueError('attitude quaternion must contain four values')
    norm = float(np.linalg.norm(q))
    if not math.isfinite(norm) or norm < 1.0e-9:
        raise ValueError('attitude quaternion must be finite and nonzero')
    w, x, y, z = q / norm
    return np.array([
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ])


def target_vector_from_rgbd(
    target_mask,
    depth,
    horizontal_fov,
    minimum_depth,
    maximum_depth,
    target_radius=0.0,
):
    """Recover the target-center vector in camera FLU coordinates."""
    if target_mask is None or depth is None:
        return None
    if target_mask.shape != depth.shape:
        return None
    valid = (
        target_mask
        & np.isfinite(depth)
        & (depth >= float(minimum_depth))
        & (depth <= float(maximum_depth))
    )
    rows, columns = np.nonzero(valid)
    if columns.size == 0:
        return None
    height, width = depth.shape
    fx, fy, cx, cy = camera_intrinsics(width, height, horizontal_fov)
    forward = float(np.median(depth[valid])) + max(
        float(target_radius),
        0.0,
    )
    image_x = float(np.median(columns))
    image_y = float(np.median(rows))

    # Gazebo's camera optical axis is +X. Image right is camera -Y and
    # image down is camera -Z for the FLU camera-link convention.
    left = -(image_x - cx) * forward / fx
    up = -(image_y - cy) * forward / fy
    return np.array((forward, left, up), dtype=float)


def camera_target_to_local_ned(
    camera_vector_flu,
    uav_position_ned,
    attitude_quaternion,
    camera_translation_flu,
    camera_pitch_down,
    target_reference_z_offset=0.0,
):
    """Transform a camera-FLU target vector to PX4 local NED position."""
    camera_vector = np.asarray(camera_vector_flu, dtype=float)
    uav_position = np.asarray(uav_position_ned, dtype=float)
    translation = np.asarray(camera_translation_flu, dtype=float)
    if (
        camera_vector.shape != (3,)
        or uav_position.shape != (3,)
        or translation.shape != (3,)
        or not np.all(np.isfinite(camera_vector))
        or not np.all(np.isfinite(uav_position))
        or not np.all(np.isfinite(translation))
    ):
        raise ValueError('camera, vehicle, and translation must be 3-vectors')

    pitch = float(camera_pitch_down)
    cosine = math.cos(pitch)
    sine = math.sin(pitch)
    camera_x, camera_y, camera_z = camera_vector
    body_flu = translation + np.array((
        cosine * camera_x + sine * camera_z,
        camera_y,
        -sine * camera_x + cosine * camera_z,
    ))
    body_frd = np.array((body_flu[0], -body_flu[1], -body_flu[2]))
    position_ned = (
        uav_position
        + body_frd_to_ned_rotation(attitude_quaternion) @ body_frd
    )
    position_ned[2] += float(target_reference_z_offset)
    return position_ned


class RgbdTargetLocalizer(Node):
    """Publish camera-derived USV position in the PX4 local NED frame."""

    def __init__(self):
        """Configure image, vehicle-state, and observation interfaces."""
        super().__init__('rgbd_target_localizer')
        self.declare_parameter('color_topic', '/camera/front/image_raw')
        self.declare_parameter(
            'depth_topic',
            '/camera/front/depth/image_raw',
        )
        self.declare_parameter(
            'observation_topic',
            '/perception/front/target_observation',
        )
        self.declare_parameter(
            'position_topic',
            '/perception/front/target_position',
        )
        self.declare_parameter('frame_id', 'local_ned')
        self.declare_parameter('horizontal_fov', 1.74)
        self.declare_parameter('minimum_red_pixels', 3)
        self.declare_parameter('minimum_depth', 0.2)
        self.declare_parameter('maximum_depth', 25.0)
        self.declare_parameter('minimum_depth_ratio', 0.5)
        self.declare_parameter('maximum_rgb_depth_skew', 0.1)
        self.declare_parameter('data_timeout', 0.5)
        self.declare_parameter('localization_rate_hz', 20.0)
        self.declare_parameter('camera_pitch_down', 0.20944)
        self.declare_parameter('camera_translation_x', 0.35)
        self.declare_parameter('camera_translation_y', 0.0)
        self.declare_parameter('camera_translation_z', -0.05)
        self.declare_parameter('target_radius', 0.25)
        self.declare_parameter('target_reference_z_offset', 0.42)
        self.declare_parameter('base_position_std', 0.08)
        self.declare_parameter('range_position_std_scale', 0.01)

        self.horizontal_fov = float(
            self.get_parameter('horizontal_fov').value
        )
        self.minimum_red_pixels = max(
            int(self.get_parameter('minimum_red_pixels').value),
            1,
        )
        self.minimum_depth = max(
            float(self.get_parameter('minimum_depth').value),
            0.0,
        )
        self.maximum_depth = max(
            float(self.get_parameter('maximum_depth').value),
            self.minimum_depth,
        )
        self.minimum_depth_ratio = min(max(
            float(self.get_parameter('minimum_depth_ratio').value),
            0.0,
        ), 1.0)
        self.maximum_rgb_depth_skew = max(
            float(self.get_parameter('maximum_rgb_depth_skew').value),
            0.0,
        )
        self.data_timeout = max(
            float(self.get_parameter('data_timeout').value),
            0.05,
        )
        self.camera_pitch_down = float(
            self.get_parameter('camera_pitch_down').value
        )
        self.camera_translation_flu = tuple(
            float(self.get_parameter(name).value)
            for name in (
                'camera_translation_x',
                'camera_translation_y',
                'camera_translation_z',
            )
        )
        self.target_radius = max(
            float(self.get_parameter('target_radius').value),
            0.0,
        )
        self.target_reference_z_offset = float(
            self.get_parameter('target_reference_z_offset').value
        )
        self.base_position_std = max(
            float(self.get_parameter('base_position_std').value),
            1.0e-3,
        )
        self.range_position_std_scale = max(
            float(
                self.get_parameter('range_position_std_scale').value
            ),
            0.0,
        )
        self.frame_id = str(self.get_parameter('frame_id').value)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        color_topic = str(self.get_parameter('color_topic').value)
        depth_topic = str(self.get_parameter('depth_topic').value)
        self.color_sub = self.create_subscription(
            Image,
            color_topic,
            self.color_callback,
            sensor_qos,
        )
        self.depth_sub = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            sensor_qos,
        )
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            sensor_qos,
        )
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            sensor_qos,
        )
        self.observation_pub = self.create_publisher(
            TargetObservation,
            str(self.get_parameter('observation_topic').value),
            10,
        )
        self.target_position_pub = self.create_publisher(
            Point,
            str(self.get_parameter('position_topic').value),
            10,
        )

        self.color_mask = None
        self.color_time = -math.inf
        self.depth = None
        self.depth_time = -math.inf
        self.uav_position = None
        self.uav_position_time = -math.inf
        self.uav_attitude = None
        self.uav_attitude_time = -math.inf
        self.processed_color_time = -math.inf
        localization_rate_hz = max(
            float(self.get_parameter('localization_rate_hz').value),
            1.0,
        )
        self.timer = self.create_timer(
            1.0 / localization_rate_hz,
            self.localize,
        )
        self.get_logger().info(
            'RGB-D TARGET LOCALIZER READY | '
            f'RGB={color_topic} | depth={depth_topic} | '
            'output frame=local_ned | detector=red validation sphere'
        )

    def color_callback(self, message):
        """Extract the red-target mask from the newest RGB frame."""
        self.color_mask = red_pixel_mask(
            message.data,
            message.width,
            message.height,
            message.step,
            message.encoding.lower(),
        )
        self.color_time = time.monotonic()

    def depth_callback(self, message):
        """Decode the aligned float depth image."""
        if message.encoding.lower() not in ('32fc1', '32fc'):
            return
        self.depth = decode_float32_depth(
            message.data,
            message.width,
            message.height,
            message.step,
        )
        self.depth_time = time.monotonic()

    def position_callback(self, message):
        """Store the latest finite UAV local-NED position."""
        values = (message.x, message.y, message.z)
        if all(math.isfinite(value) for value in values):
            self.uav_position = tuple(float(value) for value in values)
            self.uav_position_time = time.monotonic()

    def attitude_callback(self, message):
        """Store the latest finite PX4 body-to-NED quaternion."""
        quaternion = tuple(float(value) for value in message.q)
        if all(math.isfinite(value) for value in quaternion):
            self.uav_attitude = quaternion
            self.uav_attitude_time = time.monotonic()

    def publish_invalid_observation(self):
        """Publish an explicit invalid observation without updating the KF."""
        message = TargetObservation()
        message.stamp = self.get_clock().now().to_msg()
        message.frame_id = self.frame_id
        message.position.x = math.nan
        message.position.y = math.nan
        message.position.z = math.nan
        message.covariance = [math.nan] * 9
        message.confidence = 0.0
        message.source = 'front_rgbd_red_sphere'
        message.valid = False
        self.observation_pub.publish(message)

    def localize(self):
        """Publish one synchronized RGB-D target observation when valid."""
        now = time.monotonic()
        if self.color_time <= self.processed_color_time:
            return
        self.processed_color_time = self.color_time
        state_fresh = (
            self.uav_position is not None
            and self.uav_attitude is not None
            and now - self.uav_position_time <= self.data_timeout
            and now - self.uav_attitude_time <= self.data_timeout
        )
        images_fresh = (
            self.color_mask is not None
            and self.depth is not None
            and now - self.color_time <= self.data_timeout
            and now - self.depth_time <= self.data_timeout
            and abs(self.color_time - self.depth_time)
            <= self.maximum_rgb_depth_skew
        )
        red_pixels = (
            0
            if self.color_mask is None
            else int(np.count_nonzero(self.color_mask))
        )
        if not state_fresh or not images_fresh or (
            red_pixels < self.minimum_red_pixels
        ):
            self.publish_invalid_observation()
            return

        valid_depth = (
            self.color_mask
            & np.isfinite(self.depth)
            & (self.depth >= self.minimum_depth)
            & (self.depth <= self.maximum_depth)
        )
        depth_ratio = float(np.count_nonzero(valid_depth)) / red_pixels
        if depth_ratio < self.minimum_depth_ratio:
            self.publish_invalid_observation()
            return
        camera_vector = target_vector_from_rgbd(
            self.color_mask,
            self.depth,
            self.horizontal_fov,
            self.minimum_depth,
            self.maximum_depth,
            self.target_radius,
        )
        if camera_vector is None:
            self.publish_invalid_observation()
            return
        try:
            target_position = camera_target_to_local_ned(
                camera_vector,
                self.uav_position,
                self.uav_attitude,
                self.camera_translation_flu,
                self.camera_pitch_down,
                self.target_reference_z_offset,
            )
        except ValueError:
            self.publish_invalid_observation()
            return

        target_range = float(np.linalg.norm(camera_vector))
        position_std = (
            self.base_position_std
            + self.range_position_std_scale * target_range
        )
        variance = position_std * position_std
        covariance = [
            variance, 0.0, 0.0,
            0.0, variance, 0.0,
            0.0, 0.0, variance,
        ]
        confidence = min(
            depth_ratio * red_pixels / max(self.minimum_red_pixels, 20),
            1.0,
        )
        observation = TargetObservation()
        observation.stamp = self.get_clock().now().to_msg()
        observation.frame_id = self.frame_id
        observation.position.x = float(target_position[0])
        observation.position.y = float(target_position[1])
        observation.position.z = float(target_position[2])
        observation.covariance = covariance
        observation.confidence = float(confidence)
        observation.source = 'front_rgbd_red_sphere'
        observation.valid = True
        self.observation_pub.publish(observation)

        point = Point()
        point.x = observation.position.x
        point.y = observation.position.y
        point.z = observation.position.z
        self.target_position_pub.publish(point)


def main(args=None):
    """Run the RGB-D target localizer node."""
    rclpy.init(args=args)
    node = RgbdTargetLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
