"""
Localize the simulated red USV target from aligned RGB and depth images.

The detector is intentionally limited to the red validation sphere.  The
camera geometry and filtering interface remain reusable when a non-cooperative
USV detector replaces the color mask.
"""

import math
import time
from dataclasses import dataclass
from collections import deque

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
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
from std_msgs.msg import Float32
from uav_usv_interfaces.msg import TargetObservation

from .front_tof_monitor import (
    camera_intrinsics,
    decode_float32_depth,
    red_pixel_mask,
)


def validated_sensor_stamp(measurement_stamp, receipt_stamp, maximum_age):
    """Accept only acquisition stamps demonstrably in the ROS clock domain."""
    measurement_stamp = float(measurement_stamp)
    receipt_stamp = float(receipt_stamp)
    maximum_age = max(float(maximum_age), 0.0)
    if (
        not math.isfinite(measurement_stamp)
        or not math.isfinite(receipt_stamp)
        or measurement_stamp <= 0.0
    ):
        return None
    age = receipt_stamp - measurement_stamp
    if age < -1e-9 or age > maximum_age + 1e-9:
        return None
    return measurement_stamp


def synchronized_measurement_time(color_stamp, depth_stamp, maximum_skew):
    """Return the midpoint acquisition time for a valid RGB/depth pair."""
    color_stamp = float(color_stamp)
    depth_stamp = float(depth_stamp)
    if (
        not math.isfinite(color_stamp)
        or not math.isfinite(depth_stamp)
        or abs(color_stamp - depth_stamp) > float(maximum_skew) + 1e-9
    ):
        return None
    return 0.5 * (color_stamp + depth_stamp)


def due_data_timeout(now, latest_receipt, last_report, timeout):
    """Report a missing image only after timeout and at a bounded rate."""
    timeout = max(float(timeout), 1e-3)
    return (
        float(now) - float(latest_receipt) > timeout
        and float(now) - float(last_report) >= timeout
    )


def quaternion_slerp(left, right, fraction):
    """Interpolate equivalent unit quaternions along the shortest arc."""
    left = np.asarray(left, dtype=float)
    right = np.asarray(right, dtype=float)
    if left.shape != (4,) or right.shape != (4,):
        raise ValueError('quaternions must contain four values')
    left_norm = float(np.linalg.norm(left))
    right_norm = float(np.linalg.norm(right))
    if left_norm < 1e-9 or right_norm < 1e-9:
        raise ValueError('quaternions must be nonzero')
    left /= left_norm
    right /= right_norm
    dot = float(np.dot(left, right))
    if dot < 0.0:
        right = -right
        dot = -dot
    dot = min(max(dot, -1.0), 1.0)
    fraction = min(max(float(fraction), 0.0), 1.0)
    if dot > 0.9995:
        result = left + fraction * (right - left)
    else:
        angle = math.acos(dot)
        sine = math.sin(angle)
        result = (
            math.sin((1.0 - fraction) * angle) / sine * left
            + math.sin(fraction * angle) / sine * right
        )
    result /= np.linalg.norm(result)
    return tuple(float(value) for value in result)


def pose_history_rejection_reason(
    measurement_stamp,
    position_range,
    attitude_range,
):
    """Explain why the acquisition time lacks a complete UAV pose."""
    if position_range is None:
        return 'POSITION_HISTORY_EMPTY'
    if attitude_range is None:
        return 'ATTITUDE_HISTORY_EMPTY'
    stamp = float(measurement_stamp)
    if stamp < position_range[0] - 1e-9:
        return 'POSITION_TIMESTAMP_BEFORE_HISTORY'
    if stamp < attitude_range[0] - 1e-9:
        return 'ATTITUDE_TIMESTAMP_BEFORE_HISTORY'
    if stamp > position_range[1] + 1e-9:
        return 'POSITION_TIMESTAMP_AFTER_HISTORY'
    if stamp > attitude_range[1] + 1e-9:
        return 'ATTITUDE_TIMESTAMP_AFTER_HISTORY'
    return ''


class TimestampedVectorHistory:
    """Interpolate a bounded monotonic history without extrapolation."""

    def __init__(self, maximum_age=1.0, interpolator=None):
        self.maximum_age = max(float(maximum_age), 1e-3)
        self.interpolator = interpolator
        self._samples = deque()

    @property
    def latest_value(self):
        return self._samples[-1][1] if self._samples else None

    @property
    def time_range(self):
        if not self._samples:
            return None
        return self._samples[0][0], self._samples[-1][0]

    def add(self, stamp, value):
        stamp = float(stamp)
        value = tuple(float(component) for component in value)
        if not math.isfinite(stamp) or not all(map(math.isfinite, value)):
            return False
        if self._samples and stamp < self._samples[-1][0] - 1e-9:
            return False
        if self._samples and abs(stamp - self._samples[-1][0]) <= 1e-9:
            self._samples[-1] = stamp, value
        else:
            self._samples.append((stamp, value))
        cutoff = stamp - self.maximum_age
        while len(self._samples) > 2 and self._samples[1][0] < cutoff:
            self._samples.popleft()
        return True

    def clear(self):
        self._samples.clear()

    def value_at(self, stamp):
        stamp = float(stamp)
        if not self._samples:
            return None
        if (
            stamp < self._samples[0][0] - 1e-9
            or stamp > self._samples[-1][0] + 1e-9
        ):
            return None
        for index, (sample_stamp, value) in enumerate(self._samples):
            if abs(stamp - sample_stamp) <= 1e-9:
                return value
            if sample_stamp > stamp and index > 0:
                previous_stamp, previous = self._samples[index - 1]
                fraction = (
                    (stamp - previous_stamp)
                    / max(sample_stamp - previous_stamp, 1e-9)
                )
                if self.interpolator is not None:
                    return self.interpolator(previous, value, fraction)
                return tuple(
                    left + fraction * (right - left)
                    for left, right in zip(previous, value)
                )
        return self._samples[-1][1]


class Px4RosClockMapper:
    """Map one clock shared by independently ordered source streams."""

    def __init__(
        self,
        maximum_offset_jump=0.05,
        calibration_samples=4,
        reset_regression_threshold=0.5,
        reset_confirmation_window=0.5,
        offset_recovery_samples=3,
        source_is_ros_time=False,
    ):
        self.maximum_offset_jump = max(float(maximum_offset_jump), 1e-6)
        self.calibration_samples = max(int(calibration_samples), 1)
        self.reset_regression_threshold = max(
            float(reset_regression_threshold), 1e-3
        )
        self.reset_confirmation_window = max(
            float(reset_confirmation_window), 1e-3
        )
        self.offset_recovery_samples = max(
            int(offset_recovery_samples), 2
        )
        self.source_is_ros_time = bool(source_is_ros_time)
        self.offset = 0.0 if self.source_is_ros_time else None
        self._offset_samples = deque(maxlen=32)
        self._offset_outlier_samples = {}
        self._last_source_times = {}
        self._last_receipt_time = None
        self._reset_candidates = {}
        self.reset_count = 0
        self.calibration_count = 1 if self.source_is_ros_time else 0
        self.offset_recalibration_count = 0
        self.last_status = 'UNCALIBRATED'

    def reset(self):
        self.offset = 0.0 if self.source_is_ros_time else None
        self._offset_samples.clear()
        self._offset_outlier_samples.clear()
        self._last_source_times.clear()
        self._last_receipt_time = None
        self._reset_candidates.clear()
        self.reset_count += 1
        self.last_status = 'CLOCK_RESET'

    def _seed_after_reset(
        self,
        stream_name,
        source_time,
        receipt_ros_time,
        observed_offset,
    ):
        self._last_source_times[stream_name] = source_time
        self._last_receipt_time = receipt_ros_time
        if not self.source_is_ros_time:
            self._offset_samples.append(observed_offset)

    def _confirmed_source_reset(
        self,
        stream_name,
        source_time,
        receipt_ros_time,
    ):
        cutoff = receipt_ros_time - self.reset_confirmation_window
        self._reset_candidates = {
            name: candidate
            for name, candidate in self._reset_candidates.items()
            if candidate[1] >= cutoff
        }
        self._reset_candidates[stream_name] = (
            source_time,
            receipt_ros_time,
        )
        return any(
            name != stream_name
            and abs(candidate[0] - source_time)
            <= self.reset_regression_threshold
            for name, candidate in self._reset_candidates.items()
        )

    def _confirmed_offset_change(
        self,
        stream_name,
        observed_offset,
        receipt_ros_time,
    ):
        samples = self._offset_outlier_samples.setdefault(
            stream_name,
            deque(maxlen=self.offset_recovery_samples),
        )
        if (
            samples
            and (
                receipt_ros_time - samples[-1][1]
                > self.reset_confirmation_window
                or abs(observed_offset - samples[-1][0])
                > self.maximum_offset_jump
            )
        ):
            samples.clear()
        samples.append((observed_offset, receipt_ros_time))
        if len(samples) < self.offset_recovery_samples:
            return False
        candidate = sum(value for value, _stamp in samples) / len(samples)
        for other_stream, other_samples in self._offset_outlier_samples.items():
            if (
                other_stream == stream_name
                or len(other_samples) < self.offset_recovery_samples
            ):
                continue
            other_candidate = sum(
                value for value, _stamp in other_samples
            ) / len(other_samples)
            if abs(candidate - other_candidate) <= self.maximum_offset_jump:
                return True
        return False

    def to_ros_time(
        self,
        source_time,
        receipt_ros_time,
        stream_name=None,
    ):
        source_time = float(source_time)
        receipt_ros_time = float(receipt_ros_time)
        if (
            not math.isfinite(source_time)
            or source_time <= 0.0
            or not math.isfinite(receipt_ros_time)
        ):
            self.last_status = 'INVALID_TIMESTAMP'
            return None
        stream = (
            '__single_stream__'
            if stream_name is None else str(stream_name)
        )
        observed_offset = receipt_ros_time - source_time
        if (
            self._last_receipt_time is not None
            and receipt_ros_time < self._last_receipt_time - 1e-6
        ):
            self.reset()
            self._seed_after_reset(
                stream, source_time, receipt_ros_time, observed_offset
            )
            self.last_status = 'ROS_CLOCK_RESET'
            return None
        self._last_receipt_time = receipt_ros_time

        previous_source = self._last_source_times.get(stream)
        if previous_source is not None:
            regression = previous_source - source_time
            if abs(regression) <= 1e-9:
                self.last_status = 'DUPLICATE_SOURCE_TIMESTAMP'
                return None
            if regression > 0.0:
                if (
                    stream_name is not None
                    and regression <= self.reset_regression_threshold
                ):
                    self.last_status = 'OUT_OF_ORDER_SOURCE_TIMESTAMP'
                    return None
                confirmed = (
                    stream_name is None
                    or self._confirmed_source_reset(
                        stream, source_time, receipt_ros_time
                    )
                )
                if not confirmed:
                    self.last_status = 'SOURCE_RESET_CANDIDATE'
                    return None
                self.reset()
                self._seed_after_reset(
                    stream, source_time, receipt_ros_time, observed_offset
                )
                self.last_status = 'PX4_CLOCK_RESET'
                return None
        if self.source_is_ros_time:
            self._last_source_times[stream] = source_time
            self._reset_candidates.pop(stream, None)
            self.last_status = 'DIRECT_TIMESTAMP'
            return source_time
        if (
            stream_name is not None
            and self.offset is not None
            and observed_offset
            < self.offset - self.maximum_offset_jump
        ):
            if self._confirmed_offset_change(
                stream,
                observed_offset,
                receipt_ros_time,
            ):
                self.reset()
                self.offset_recalibration_count += 1
                self._seed_after_reset(
                    stream,
                    source_time,
                    receipt_ros_time,
                    observed_offset,
                )
                self.last_status = 'OFFSET_RECALIBRATING'
                return None
            self.last_status = 'OFFSET_OUTLIER'
            return None
        self._last_source_times[stream] = source_time
        self._reset_candidates.pop(stream, None)
        self._offset_outlier_samples.pop(stream, None)
        self._offset_samples.append(observed_offset)
        if len(self._offset_samples) < self.calibration_samples:
            self.last_status = 'UNCALIBRATED'
            return None
        if (
            max(self._offset_samples) - min(self._offset_samples)
            > self.maximum_offset_jump
            and self.offset is None
        ):
            self.last_status = 'OFFSET_UNSTABLE'
            return None
        candidate = min(self._offset_samples)
        if self.offset is not None and abs(
            candidate - self.offset
        ) > self.maximum_offset_jump:
            self.reset()
            self._seed_after_reset(
                stream, source_time, receipt_ros_time, observed_offset
            )
            self.last_status = 'OFFSET_JUMP_RESET'
            return None
        if self.offset is None:
            self.offset = candidate
            self.calibration_count += 1
        else:
            self.offset = min(self.offset, candidate)
        self.last_status = 'MAPPED'
        return source_time + self.offset


@dataclass(frozen=True)
class TimestampedImageFrame:
    message: object
    raw_stamp: float
    receipt_stamp: float


class RgbDepthPairBuffer:
    """Pair each frame at most once using bounded acquisition-time queues."""

    def __init__(self, maximum_skew, capacity=8):
        self.maximum_skew = max(float(maximum_skew), 0.0)
        self.capacity = max(int(capacity), 2)
        self.color = deque(maxlen=self.capacity)
        self.depth = deque(maxlen=self.capacity)
        self.last_stamp = {'color': None, 'depth': None}

    def reset(self):
        self.color.clear()
        self.depth.clear()
        self.last_stamp = {'color': None, 'depth': None}

    def add(self, stream, message, raw_stamp, receipt_stamp):
        raw_stamp = float(raw_stamp)
        receipt_stamp = float(receipt_stamp)
        if stream not in self.last_stamp:
            raise ValueError('stream must be color or depth')
        if not math.isfinite(raw_stamp) or raw_stamp <= 0.0:
            return 'RAW_TIMESTAMP_INVALID'
        previous = self.last_stamp[stream]
        if previous is not None and raw_stamp <= previous + 1e-9:
            if raw_stamp < previous - 1.0:
                self.reset()
                self.last_stamp[stream] = raw_stamp
                getattr(self, stream).append(TimestampedImageFrame(
                    message, raw_stamp, receipt_stamp
                ))
                return 'TIME_RESET'
            return (
                'DUPLICATE_FRAME' if abs(raw_stamp - previous) <= 1e-9
                else 'OUT_OF_ORDER_FRAME'
            )
        self.last_stamp[stream] = raw_stamp
        getattr(self, stream).append(TimestampedImageFrame(
            message, raw_stamp, receipt_stamp
        ))
        return ''

    def pop_pair(self):
        while self.color and self.depth:
            color = self.color[0]
            depth = self.depth[0]
            skew = color.raw_stamp - depth.raw_stamp
            if abs(skew) <= self.maximum_skew + 1e-9:
                return self.color.popleft(), self.depth.popleft(), ''
            if skew < 0.0:
                self.color.popleft()
                return None, None, 'RGB_FRAME_UNMATCHED'
            self.depth.popleft()
            return None, None, 'DEPTH_FRAME_UNMATCHED'
        return None, None, ''


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _seconds_to_time(value):
    if value is None or not math.isfinite(float(value)) or value <= 0.0:
        return Time()
    seconds = int(value)
    nanoseconds = int(round((float(value) - seconds) * 1e9))
    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds -= 1_000_000_000
    return Time(sec=seconds, nanosec=nanoseconds)


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
        self.declare_parameter('state_history_duration', 1.0)
        self.declare_parameter('maximum_clock_offset_jump', 0.05)
        self.declare_parameter('px4_timestamp_is_ros_time', True)
        self.declare_parameter('pose_wait_timeout', 0.15)
        self.declare_parameter('maximum_pose_wait_gap', 0.15)
        self.declare_parameter('image_pair_buffer_size', 8)
        self.declare_parameter('time_pair_diagnostics_enabled', False)
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
        self.state_history_duration = max(
            float(self.get_parameter('state_history_duration').value),
            self.data_timeout,
        )
        self.maximum_clock_offset_jump = max(
            float(self.get_parameter('maximum_clock_offset_jump').value),
            1e-3,
        )
        self.px4_timestamp_is_ros_time = bool(
            self.get_parameter('px4_timestamp_is_ros_time').value
        )
        self.pose_wait_timeout = max(
            float(self.get_parameter('pose_wait_timeout').value),
            0.0,
        )
        self.maximum_pose_wait_gap = max(
            float(self.get_parameter('maximum_pose_wait_gap').value),
            0.0,
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
        self.time_pair_diagnostics_enabled = bool(
            self.get_parameter('time_pair_diagnostics_enabled').value
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
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
        self.compute_time_pub = self.create_publisher(
            Float32,
            '/diagnostics/rgbd_localizer/compute_time',
            10,
        )

        self.latest_color_message = None
        self.latest_depth_message = None
        self.color_mask = None
        self.color_time = -math.inf
        self.color_measurement_time = None
        self.depth = None
        self.depth_time = -math.inf
        self.depth_measurement_time = None
        self.uav_position = None
        self.uav_position_time = -math.inf
        self.uav_attitude = None
        self.uav_attitude_time = -math.inf
        self.position_source_stamp = math.nan
        self.position_mapped_stamp = math.nan
        self.attitude_source_stamp = math.nan
        self.attitude_mapped_stamp = math.nan
        self.processed_color_time = -math.inf
        self.last_image_timeout_report = -math.inf
        self.last_pair_rejection = ''
        self.pending_image_pairs = deque(maxlen=max(
            int(self.get_parameter('image_pair_buffer_size').value), 2
        ))
        self.waiting_image_pair = None
        self.image_pair_buffer = RgbDepthPairBuffer(
            self.maximum_rgb_depth_skew,
            self.pending_image_pairs.maxlen,
        )
        self.last_time_diagnostic = {}
        self.last_time_diagnostic_log = -math.inf
        self._pending_pose_samples = deque(maxlen=8)
        self.position_history = TimestampedVectorHistory(
            self.state_history_duration
        )
        self.attitude_history = TimestampedVectorHistory(
            self.state_history_duration,
            interpolator=quaternion_slerp,
        )
        self.image_clock_mapper = Px4RosClockMapper(
            self.maximum_clock_offset_jump
        )
        self.px4_clock_mapper = Px4RosClockMapper(
            self.maximum_clock_offset_jump,
            source_is_ros_time=self.px4_timestamp_is_ros_time,
        )
        localization_rate_hz = max(
            float(self.get_parameter('localization_rate_hz').value),
            1.0,
        )
        self.timer = self.create_timer(
            1.0 / localization_rate_hz,
            self.timed_localize,
        )
        self.get_logger().info(
            'RGB-D TARGET LOCALIZER READY | '
            f'RGB={color_topic} | depth={depth_topic} | '
            'output frame=local_ned | detector=red validation sphere'
        )

    def color_callback(self, message):
        """Cache RGB by acquisition stamp without image work in DDS."""
        receipt = self._ros_seconds()
        source = _stamp_seconds(message.header.stamp)
        self.latest_color_message = message
        self.color_time = receipt
        self.last_pair_rejection = self.image_pair_buffer.add(
            'color', message, source, receipt
        )
        if self.last_pair_rejection == 'TIME_RESET':
            self.pending_image_pairs.clear()
            self.waiting_image_pair = None
            self.image_clock_mapper.reset()
        self._collect_image_pair()

    def depth_callback(self, message):
        """Cache depth by acquisition stamp without decoding it in DDS."""
        receipt = self._ros_seconds()
        source = _stamp_seconds(message.header.stamp)
        self.latest_depth_message = message
        self.depth_time = receipt
        self.last_pair_rejection = self.image_pair_buffer.add(
            'depth', message, source, receipt
        )
        if self.last_pair_rejection == 'TIME_RESET':
            self.pending_image_pairs.clear()
            self.waiting_image_pair = None
            self.image_clock_mapper.reset()
        self._collect_image_pair()

    def _collect_image_pair(self):
        while True:
            color, depth, rejection = self.image_pair_buffer.pop_pair()
            if rejection:
                self.last_pair_rejection = rejection
                continue
            if color is None:
                return
            self.pending_image_pairs.append((color, depth))
            self.last_pair_rejection = ''

    def position_callback(self, message):
        """Store the latest finite UAV local-NED position."""
        values = (message.x, message.y, message.z)
        if all(math.isfinite(value) for value in values):
            receipt = self._ros_seconds()
            source_timestamp = (
                getattr(message, 'timestamp_sample', 0)
                or getattr(message, 'timestamp', 0)
            )
            source_stamp = float(source_timestamp) * 1e-6
            self.position_source_stamp = source_stamp
            reset_count = self.px4_clock_mapper.reset_count
            stamp = self.px4_clock_mapper.to_ros_time(
                source_stamp,
                receipt,
                stream_name='position',
            )
            clock_reset = (
                self.px4_clock_mapper.reset_count != reset_count
            )
            if clock_reset:
                self._reset_pose_time_state()
            if stamp is None:
                if (
                    clock_reset
                    or self.px4_clock_mapper.last_status == 'UNCALIBRATED'
                ):
                    self._pending_pose_samples_for_node().append((
                        'position',
                        source_stamp,
                        tuple(float(v) for v in values),
                    ))
                return
            self._flush_pending_pose_samples()
            self.position_mapped_stamp = stamp
            position = tuple(float(value) for value in values)
            if self.position_history.add(stamp, position):
                self.uav_position = position
                self.uav_position_time = stamp

    def attitude_callback(self, message):
        """Store the latest finite PX4 body-to-NED quaternion."""
        quaternion = tuple(float(value) for value in message.q)
        if all(math.isfinite(value) for value in quaternion):
            receipt = self._ros_seconds()
            source_timestamp = (
                getattr(message, 'timestamp_sample', 0)
                or getattr(message, 'timestamp', 0)
            )
            source_stamp = float(source_timestamp) * 1e-6
            self.attitude_source_stamp = source_stamp
            reset_count = self.px4_clock_mapper.reset_count
            stamp = self.px4_clock_mapper.to_ros_time(
                source_stamp,
                receipt,
                stream_name='attitude',
            )
            clock_reset = (
                self.px4_clock_mapper.reset_count != reset_count
            )
            if clock_reset:
                self._reset_pose_time_state()
            if stamp is None:
                if (
                    clock_reset
                    or self.px4_clock_mapper.last_status == 'UNCALIBRATED'
                ):
                    self._pending_pose_samples_for_node().append(
                        ('attitude', source_stamp, quaternion)
                    )
                return
            self._flush_pending_pose_samples()
            self.attitude_mapped_stamp = stamp
            self._add_attitude_sample(stamp, quaternion)

    def _pending_pose_samples_for_node(self):
        if not hasattr(self, '_pending_pose_samples'):
            self._pending_pose_samples = deque(maxlen=8)
        return self._pending_pose_samples

    def _add_attitude_sample(self, stamp, quaternion):
        previous = self.attitude_history.latest_value
        if (
            previous is not None
            and sum(left * right for left, right in zip(
                previous,
                quaternion,
            )) < 0.0
        ):
            quaternion = tuple(-value for value in quaternion)
        if self.attitude_history.add(stamp, quaternion):
            self.uav_attitude = quaternion
            self.uav_attitude_time = stamp

    def _reset_pose_time_state(self):
        self._pending_pose_samples_for_node().clear()
        self.waiting_image_pair = None
        self.position_history.clear()
        self.attitude_history.clear()
        self.uav_position = None
        self.uav_attitude = None
        self.uav_position_time = -math.inf
        self.uav_attitude_time = -math.inf
        self.position_mapped_stamp = math.nan
        self.attitude_mapped_stamp = math.nan

    def _flush_pending_pose_samples(self):
        if self.px4_clock_mapper.offset is None:
            return
        pending = self._pending_pose_samples_for_node()
        while pending:
            kind, source_stamp, values = pending.popleft()
            stamp = source_stamp + self.px4_clock_mapper.offset
            if kind == 'position':
                if self.position_history.add(stamp, values):
                    self.uav_position = values
                    self.uav_position_time = stamp
                    self.position_mapped_stamp = stamp
            else:
                self._add_attitude_sample(stamp, values)
                self.attitude_mapped_stamp = stamp

    def _ros_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_invalid_observation(
        self,
        reason='INVALID_OBSERVATION',
        measurement_stamp=None,
        received_stamp=None,
    ):
        """Publish an explicit invalid observation without updating the KF."""
        message = TargetObservation()
        published = self._ros_seconds()
        message.stamp = _seconds_to_time(measurement_stamp)
        message.received_stamp = _seconds_to_time(received_stamp)
        message.processed_stamp = _seconds_to_time(published)
        message.published_stamp = _seconds_to_time(published)
        message.frame_id = self.frame_id
        message.position.x = math.nan
        message.position.y = math.nan
        message.position.z = math.nan
        message.covariance = [math.nan] * 9
        message.confidence = 0.0
        message.source = 'front_rgbd_red_sphere'
        message.rejection_reason = str(reason)
        self._fill_time_diagnostic(message)
        message.red_pixel_count = 0
        message.valid_depth_ratio = 0.0
        message.target_range = math.nan
        message.view_angle = math.nan
        message.valid = False
        self.observation_pub.publish(message)
        if (
            getattr(self, 'time_pair_diagnostics_enabled', False)
            and published - getattr(
                self, 'last_time_diagnostic_log', -math.inf
            )
            >= 1.0
        ):
            values = getattr(self, 'last_time_diagnostic', {})
            self.get_logger().warning(
                'RGB-D time reject | reason=%s | rgb_raw=%.6f | '
                'depth_raw=%.6f | rgb_receipt=%.6f | depth_receipt=%.6f'
                % (
                    reason,
                    values.get('rgb_raw_stamp', math.nan),
                    values.get('depth_raw_stamp', math.nan),
                    values.get('rgb_receipt_stamp', math.nan),
                    values.get('depth_receipt_stamp', math.nan),
                )
            )
            self.last_time_diagnostic_log = published

    def _fill_time_diagnostic(self, message):
        values = getattr(self, 'last_time_diagnostic', {})
        for name in (
            'rgb_raw_stamp', 'depth_raw_stamp',
            'rgb_receipt_stamp', 'depth_receipt_stamp',
            'rgb_mapped_stamp', 'depth_mapped_stamp',
            'rgb_depth_acquisition_skew',
            'pose_history_start_stamp', 'pose_history_end_stamp',
            'position_source_stamp', 'position_mapped_stamp',
            'attitude_source_stamp', 'attitude_mapped_stamp',
            'position_history_start_stamp',
            'position_history_end_stamp',
            'attitude_history_start_stamp',
            'attitude_history_end_stamp',
            'image_clock_offset', 'px4_clock_offset',
        ):
            setattr(message, name, float(values.get(name, math.nan)))
        message.px4_clock_reset_count = int(values.get(
            'px4_clock_reset_count', 0
        ))
        message.px4_clock_calibration_count = int(values.get(
            'px4_clock_calibration_count', 0
        ))
        message.px4_clock_recalibration_count = int(values.get(
            'px4_clock_recalibration_count', 0
        ))
        message.px4_clock_status = str(values.get(
            'px4_clock_status', 'UNKNOWN'
        ))

    def timed_localize(self):
        """Measure one rate-limited localization pass, including failures."""
        started = time.perf_counter()
        try:
            self.localize()
        finally:
            message = Float32()
            message.data = float(time.perf_counter() - started)
            self.compute_time_pub.publish(message)

    def localize(self):
        """Publish one synchronized RGB-D target observation when valid."""
        now = self._ros_seconds()
        waiting_pair = getattr(self, 'waiting_image_pair', None)
        if waiting_pair is None and not self.pending_image_pairs:
            if self.last_pair_rejection:
                self.publish_invalid_observation(
                    self.last_pair_rejection,
                    received_stamp=max(self.color_time, self.depth_time),
                )
                self.last_pair_rejection = ''
                return
            if due_data_timeout(
                now,
                max(self.color_time, self.depth_time),
                self.last_image_timeout_report,
                self.data_timeout,
            ):
                self.publish_invalid_observation(
                    'IMAGE_TIMEOUT',
                    received_stamp=now,
                )
                self.last_image_timeout_report = now
            return
        if waiting_pair is not None:
            color_frame, depth_frame = waiting_pair
        else:
            color_frame, depth_frame = self.pending_image_pairs.pop()
            self.pending_image_pairs.clear()
        color_message = color_frame.message
        depth_message = depth_frame.message
        raw_measurement_stamp = synchronized_measurement_time(
            color_frame.raw_stamp,
            depth_frame.raw_stamp,
            self.maximum_rgb_depth_skew,
        )
        received_stamp = max(
            color_frame.receipt_stamp, depth_frame.receipt_stamp
        )
        direct_measurement_stamp = (
            validated_sensor_stamp(
                raw_measurement_stamp, received_stamp, self.data_timeout
            ) if raw_measurement_stamp is not None else None
        )
        mapped_measurement_stamp = (
            direct_measurement_stamp
            if direct_measurement_stamp is not None else
            self.image_clock_mapper.to_ros_time(
                raw_measurement_stamp, received_stamp
            ) if raw_measurement_stamp is not None else None
        )
        measurement_stamp = (
            validated_sensor_stamp(
                mapped_measurement_stamp, received_stamp, self.data_timeout
            ) if mapped_measurement_stamp is not None else None
        )
        image_offset = (
            0.0 if direct_measurement_stamp is not None else
            self.image_clock_mapper.offset
            if self.image_clock_mapper.offset is not None else math.nan
        )
        position_range = self.position_history.time_range
        attitude_range = self.attitude_history.time_range
        pose_start = max(
            position_range[0] if position_range else -math.inf,
            attitude_range[0] if attitude_range else -math.inf,
        )
        pose_end = min(
            position_range[1] if position_range else math.inf,
            attitude_range[1] if attitude_range else math.inf,
        )
        self.last_time_diagnostic = {
            'rgb_raw_stamp': color_frame.raw_stamp,
            'depth_raw_stamp': depth_frame.raw_stamp,
            'rgb_receipt_stamp': color_frame.receipt_stamp,
            'depth_receipt_stamp': depth_frame.receipt_stamp,
            'rgb_mapped_stamp': color_frame.raw_stamp + image_offset,
            'depth_mapped_stamp': depth_frame.raw_stamp + image_offset,
            'rgb_depth_acquisition_skew': abs(
                color_frame.raw_stamp - depth_frame.raw_stamp
            ),
            'pose_history_start_stamp': pose_start,
            'pose_history_end_stamp': pose_end,
            'position_source_stamp': getattr(
                self, 'position_source_stamp', math.nan
            ),
            'position_mapped_stamp': getattr(
                self, 'position_mapped_stamp', math.nan
            ),
            'attitude_source_stamp': getattr(
                self, 'attitude_source_stamp', math.nan
            ),
            'attitude_mapped_stamp': getattr(
                self, 'attitude_mapped_stamp', math.nan
            ),
            'position_history_start_stamp': (
                position_range[0] if position_range else math.nan
            ),
            'position_history_end_stamp': (
                position_range[1] if position_range else math.nan
            ),
            'attitude_history_start_stamp': (
                attitude_range[0] if attitude_range else math.nan
            ),
            'attitude_history_end_stamp': (
                attitude_range[1] if attitude_range else math.nan
            ),
            'image_clock_offset': image_offset,
            'px4_clock_offset': (
                self.px4_clock_mapper.offset
                if self.px4_clock_mapper.offset is not None else math.nan
            ),
            'px4_clock_reset_count': self.px4_clock_mapper.reset_count,
            'px4_clock_calibration_count': (
                self.px4_clock_mapper.calibration_count
            ),
            'px4_clock_recalibration_count': (
                self.px4_clock_mapper.offset_recalibration_count
            ),
            'px4_clock_status': self.px4_clock_mapper.last_status,
        }
        if mapped_measurement_stamp is None:
            self.waiting_image_pair = None
            self.publish_invalid_observation(
                'IMAGE_CLOCK_UNCALIBRATED',
                received_stamp=received_stamp,
            )
            return
        if measurement_stamp is None:
            self.waiting_image_pair = None
            self.publish_invalid_observation(
                'IMAGE_CLOCK_DOMAIN_MISMATCH', received_stamp=received_stamp
            )
            return
        pose_rejection = pose_history_rejection_reason(
            measurement_stamp,
            position_range,
            attitude_range,
        )
        if pose_rejection:
            missing_future_gap = max(
                measurement_stamp - position_range[1]
                if position_range else math.inf,
                measurement_stamp - attitude_range[1]
                if attitude_range else math.inf,
            )
            if (
                pose_rejection in (
                    'POSITION_TIMESTAMP_AFTER_HISTORY',
                    'ATTITUDE_TIMESTAMP_AFTER_HISTORY',
                )
                and now - received_stamp
                <= getattr(self, 'pose_wait_timeout', 0.0) + 1e-9
                and missing_future_gap
                <= getattr(self, 'maximum_pose_wait_gap', 0.0) + 1e-9
            ):
                self.waiting_image_pair = (color_frame, depth_frame)
                return
            self.waiting_image_pair = None
            self.publish_invalid_observation(
                pose_rejection,
                measurement_stamp,
                received_stamp,
            )
            return
        self.waiting_image_pair = None
        self.color_mask = red_pixel_mask(
            color_message.data,
            color_message.width,
            color_message.height,
            color_message.step,
            color_message.encoding.lower(),
        )
        if depth_message.encoding.lower() not in ('32fc1', '32fc'):
            self.depth = None
        else:
            self.depth = decode_float32_depth(
                depth_message.data,
                depth_message.width,
                depth_message.height,
                depth_message.step,
            )
        uav_position = self.position_history.value_at(measurement_stamp)
        uav_attitude = self.attitude_history.value_at(measurement_stamp)
        if uav_attitude is not None:
            norm = math.sqrt(sum(value * value for value in uav_attitude))
            uav_attitude = (
                tuple(value / norm for value in uav_attitude)
                if norm > 1e-9 else None
            )
        state_fresh = uav_position is not None and uav_attitude is not None
        images_fresh = (
            self.color_mask is not None
            and self.depth is not None
            and now - color_frame.receipt_stamp <= self.data_timeout
            and now - depth_frame.receipt_stamp <= self.data_timeout
            and now - measurement_stamp <= self.data_timeout
        )
        red_pixels = (
            0
            if self.color_mask is None
            else int(np.count_nonzero(self.color_mask))
        )
        if not state_fresh or not images_fresh or (
            red_pixels < self.minimum_red_pixels
        ):
            self.publish_invalid_observation(
                'POSE_TIME_GAP_TOO_LARGE'
                if not state_fresh else 'IMAGE_INVALID',
                measurement_stamp,
                received_stamp,
            )
            return

        valid_depth = (
            self.color_mask
            & np.isfinite(self.depth)
            & (self.depth >= self.minimum_depth)
            & (self.depth <= self.maximum_depth)
        )
        depth_ratio = float(np.count_nonzero(valid_depth)) / red_pixels
        if depth_ratio < self.minimum_depth_ratio:
            self.publish_invalid_observation(
                'DEPTH_RATIO_LOW', measurement_stamp, received_stamp
            )
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
            self.publish_invalid_observation(
                'TARGET_VECTOR_INVALID', measurement_stamp, received_stamp
            )
            return
        try:
            target_position = camera_target_to_local_ned(
                camera_vector,
                uav_position,
                uav_attitude,
                self.camera_translation_flu,
                self.camera_pitch_down,
                self.target_reference_z_offset,
            )
        except ValueError:
            self.publish_invalid_observation(
                'TRANSFORM_INVALID', measurement_stamp, received_stamp
            )
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
        processed_stamp = self._ros_seconds()
        observation.stamp = _seconds_to_time(measurement_stamp)
        observation.received_stamp = _seconds_to_time(received_stamp)
        observation.processed_stamp = _seconds_to_time(processed_stamp)
        observation.published_stamp = _seconds_to_time(
            self._ros_seconds()
        )
        observation.frame_id = self.frame_id
        observation.position.x = float(target_position[0])
        observation.position.y = float(target_position[1])
        observation.position.z = float(target_position[2])
        observation.covariance = covariance
        observation.confidence = float(confidence)
        observation.source = 'front_rgbd_red_sphere'
        observation.rejection_reason = ''
        self._fill_time_diagnostic(observation)
        observation.red_pixel_count = red_pixels
        observation.valid_depth_ratio = float(depth_ratio)
        observation.target_range = float(target_range)
        observation.view_angle = float(math.atan2(
            math.hypot(camera_vector[1], camera_vector[2]),
            max(camera_vector[0], 1e-9),
        ))
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
