"""Bridge Gazebo cameras to ROS and validate simulated USV visibility."""

import math
import threading
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
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String


PIXEL_FORMATS = {
    1: ('mono8', 1),
    3: ('rgb8', 3),
    4: ('rgba8', 4),
    5: ('bgra8', 4),
    8: ('bgr8', 3),
}


def camera_intrinsics(width, height, horizontal_fov):
    """Calculate pinhole intrinsics with square pixels."""
    width = int(width)
    height = int(height)
    horizontal_fov = float(horizontal_fov)
    if width <= 0 or height <= 0:
        raise ValueError('Camera image dimensions must be positive.')
    if not 0.0 < horizontal_fov < math.pi:
        raise ValueError('Horizontal field of view must be in (0, pi).')
    focal_length = width / (2.0 * math.tan(horizontal_fov / 2.0))
    return focal_length, focal_length, width / 2.0, height / 2.0


def count_red_pixels(data, width, height, step, encoding):
    """Count strongly red pixels in a supported packed 8-bit image."""
    channels = {
        'rgb8': (3, 0, 1, 2),
        'rgba8': (4, 0, 1, 2),
        'bgr8': (3, 2, 1, 0),
        'bgra8': (4, 2, 1, 0),
    }
    if encoding not in channels:
        return 0
    channel_count, red_index, green_index, blue_index = channels[encoding]
    minimum_step = int(width) * channel_count
    if int(step) < minimum_step:
        return 0
    raw = np.frombuffer(data, dtype=np.uint8)
    required_size = int(height) * int(step)
    if raw.size < required_size:
        return 0
    rows = raw[:required_size].reshape(int(height), int(step))
    pixels = rows[:, :minimum_step].reshape(
        int(height),
        int(width),
        channel_count,
    )
    red = pixels[:, :, red_index].astype(np.uint16)
    green = pixels[:, :, green_index].astype(np.uint16)
    blue = pixels[:, :, blue_index].astype(np.uint16)
    mask = (
        (red >= 160)
        & (red >= 3 * green // 2)
        & (red >= 3 * blue // 2)
        & (red - np.minimum(green, blue) >= 60)
    )
    return int(np.count_nonzero(mask))


def quaternion_pitch(quaternion):
    """Return body pitch from a Hamilton quaternion [w, x, y, z]."""
    w, x, y, z = (float(value) for value in quaternion)
    sine_pitch = 2.0 * (w * y - z * x)
    return math.asin(max(min(sine_pitch, 1.0), -1.0))


def preferred_camera_for_geometry(
    target_distance,
    uav_pitch,
    close_observation_distance,
    camera_switch_pitch,
):
    """Select the paper-inspired camera for the current geometry."""
    close = float(target_distance) <= float(close_observation_distance)
    low_pitch = abs(float(uav_pitch)) < float(camera_switch_pitch)
    return 'down' if close and low_pitch else 'front'


class CameraVisibilityMonitor(Node):
    """Publish dual-camera images and truth-assisted visibility diagnostics."""

    def __init__(self):
        super().__init__('camera_visibility_monitor')
        self.declare_parameter(
            'front_gazebo_topic',
            '/uav/camera/front/image',
        )
        self.declare_parameter(
            'down_gazebo_topic',
            '/uav/camera/down/image',
        )
        self.declare_parameter(
            'front_ros_topic',
            '/camera/front/image_raw',
        )
        self.declare_parameter(
            'down_ros_topic',
            '/camera/down/image_raw',
        )
        self.declare_parameter('front_horizontal_fov', 1.74)
        self.declare_parameter('down_horizontal_fov', 2.0)
        self.declare_parameter('minimum_red_pixels', 20)
        self.declare_parameter('image_timeout', 0.5)
        self.declare_parameter('close_observation_distance', 2.0)
        self.declare_parameter('camera_switch_pitch_deg', 30.0)

        self.minimum_red_pixels = max(
            int(self.get_parameter('minimum_red_pixels').value),
            1,
        )
        self.image_timeout = max(
            float(self.get_parameter('image_timeout').value),
            0.05,
        )
        self.close_observation_distance = max(
            float(
                self.get_parameter('close_observation_distance').value
            ),
            0.1,
        )
        self.camera_switch_pitch = math.radians(
            max(
                float(
                    self.get_parameter(
                        'camera_switch_pitch_deg'
                    ).value
                ),
                0.0,
            )
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.cameras = {
            'front': self._camera_state(
                'front',
                self.get_parameter('front_ros_topic').value,
                'front_camera_optical_frame',
                float(
                    self.get_parameter('front_horizontal_fov').value
                ),
                sensor_qos,
            ),
            'down': self._camera_state(
                'down',
                self.get_parameter('down_ros_topic').value,
                'down_camera_optical_frame',
                float(
                    self.get_parameter('down_horizontal_fov').value
                ),
                sensor_qos,
            ),
        }
        self.visible_pub = self.create_publisher(
            Bool,
            '/perception/usv_visible',
            status_qos,
        )
        self.active_camera_pub = self.create_publisher(
            String,
            '/perception/active_camera',
            status_qos,
        )
        self.target_sub = self.create_subscription(
            Point,
            '/target/position',
            self.target_callback,
            10,
        )
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            sensor_qos,
        )
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude_v1',
            self.attitude_callback,
            sensor_qos,
        )

        self.target_position = None
        self.uav_position = None
        self.uav_pitch = 0.0
        self.last_visible = None
        self.last_active_camera = ''
        self.lock = threading.Lock()

        from gz.msgs10.image_pb2 import Image as GazeboImage
        from gz.transport13 import Node as GazeboTransportNode

        self.gazebo_image_type = GazeboImage
        self.gazebo_node = GazeboTransportNode()
        for camera_name, parameter_name in (
            ('front', 'front_gazebo_topic'),
            ('down', 'down_gazebo_topic'),
        ):
            gazebo_topic = str(
                self.get_parameter(parameter_name).value
            )
            self.gazebo_node.subscribe(
                GazeboImage,
                gazebo_topic,
                lambda message, name=camera_name: (
                    self.gazebo_image_callback(name, message)
                ),
            )
            self.get_logger().info(
                f'{camera_name.upper()} CAMERA BRIDGE | '
                f'Gazebo={gazebo_topic} | '
                f'ROS={self.cameras[camera_name]["image_topic"]}'
            )

        self.status_timer = self.create_timer(0.1, self.publish_status)
        self.get_logger().info(
            'Camera visibility validation ready; target truth remains '
            'enabled for control and evaluation.'
        )

    def _camera_state(
        self,
        name,
        image_topic,
        frame_id,
        horizontal_fov,
        qos,
    ):
        image_topic = str(image_topic)
        return {
            'name': name,
            'image_topic': image_topic,
            'frame_id': frame_id,
            'horizontal_fov': horizontal_fov,
            'image_pub': self.create_publisher(Image, image_topic, qos),
            'info_pub': self.create_publisher(
                CameraInfo,
                image_topic.rsplit('/', 1)[0] + '/camera_info',
                qos,
            ),
            'last_image_time': -math.inf,
            'red_pixels': 0,
        }

    def target_callback(self, message):
        self.target_position = (
            float(message.x),
            float(message.y),
            float(message.z),
        )

    def position_callback(self, message):
        if all(
            math.isfinite(value)
            for value in (message.x, message.y, message.z)
        ):
            self.uav_position = (
                float(message.x),
                float(message.y),
                float(message.z),
            )

    def attitude_callback(self, message):
        if all(math.isfinite(value) for value in message.q):
            self.uav_pitch = quaternion_pitch(message.q)

    def gazebo_image_callback(self, camera_name, message):
        camera = self.cameras[camera_name]
        format_info = PIXEL_FORMATS.get(int(message.pixel_format_type))
        if format_info is None:
            return
        encoding, _ = format_info
        stamp = self.get_clock().now().to_msg()
        ros_image = Image()
        ros_image.header.stamp = stamp
        ros_image.header.frame_id = camera['frame_id']
        ros_image.height = int(message.height)
        ros_image.width = int(message.width)
        ros_image.encoding = encoding
        ros_image.is_bigendian = 0
        ros_image.step = int(message.step)
        ros_image.data = bytes(message.data)

        red_pixels = count_red_pixels(
            ros_image.data,
            ros_image.width,
            ros_image.height,
            ros_image.step,
            ros_image.encoding,
        )
        camera_info = self.make_camera_info(
            camera,
            ros_image.width,
            ros_image.height,
            stamp,
        )
        with self.lock:
            camera['last_image_time'] = time.monotonic()
            camera['red_pixels'] = red_pixels
        camera['image_pub'].publish(ros_image)
        camera['info_pub'].publish(camera_info)

    @staticmethod
    def make_camera_info(camera, width, height, stamp):
        fx, fy, cx, cy = camera_intrinsics(
            width,
            height,
            camera['horizontal_fov'],
        )
        message = CameraInfo()
        message.header.stamp = stamp
        message.header.frame_id = camera['frame_id']
        message.height = height
        message.width = width
        message.distortion_model = 'plumb_bob'
        message.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        message.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        message.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return message

    def target_distance(self):
        if self.target_position is None or self.uav_position is None:
            return math.inf
        return math.sqrt(sum(
            (target - uav) ** 2
            for target, uav in zip(
                self.target_position,
                self.uav_position,
            )
        ))

    def preferred_camera(self):
        return preferred_camera_for_geometry(
            self.target_distance(),
            self.uav_pitch,
            self.close_observation_distance,
            self.camera_switch_pitch,
        )

    def publish_status(self):
        now = time.monotonic()
        with self.lock:
            visible = {
                name: (
                    now - camera['last_image_time'] <= self.image_timeout
                    and camera['red_pixels'] >= self.minimum_red_pixels
                )
                for name, camera in self.cameras.items()
            }
            red_pixels = {
                name: camera['red_pixels']
                for name, camera in self.cameras.items()
            }
        preferred = self.preferred_camera()
        if visible[preferred]:
            active_camera = preferred
        elif visible['front']:
            active_camera = 'front'
        elif visible['down']:
            active_camera = 'down'
        else:
            active_camera = 'none'
        target_visible = active_camera != 'none'

        visible_message = Bool()
        visible_message.data = target_visible
        self.visible_pub.publish(visible_message)
        camera_message = String()
        camera_message.data = active_camera
        self.active_camera_pub.publish(camera_message)

        state_changed = (
            target_visible != self.last_visible
            or active_camera != self.last_active_camera
        )
        if state_changed:
            log = (
                self.get_logger().info
                if target_visible
                else self.get_logger().warn
            )
            log(
                f'USV {"VISIBLE" if target_visible else "NOT VISIBLE"} | '
                f'active={active_camera} | preferred={preferred} | '
                f'distance={self.target_distance():.2f} m | '
                f'pitch={math.degrees(self.uav_pitch):.1f} deg | '
                f'red pixels front/down='
                f'{red_pixels["front"]}/{red_pixels["down"]}'
            )
        self.last_visible = target_visible
        self.last_active_camera = active_camera


def main(args=None):
    rclpy.init(args=args)
    node = CameraVisibilityMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
