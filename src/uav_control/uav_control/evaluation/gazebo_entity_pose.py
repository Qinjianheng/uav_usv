"""Timestamp-preserving Gazebo entity pose diagnostics for evaluation only."""

import math
import threading
from dataclasses import dataclass

import numpy as np

from uav_control.perception.rgbd_target_localizer import (
    camera_target_to_body_flu,
    GazeboImageClockMapper,
    local_ned_target_to_camera_flu,
    TimestampedVectorHistory,
)


def gazebo_enu_to_ned(position):
    """Convert a Gazebo world ENU position to PX4 local NED."""
    x, y, z = (float(value) for value in position)
    if not all(math.isfinite(value) for value in (x, y, z)):
        raise ValueError('Gazebo entity position must be finite')
    return y, x, -z


@dataclass(frozen=True)
class EntityGeometryResiduals:
    """Stage-level residuals against the sampled rendered sphere center."""

    expected_camera: tuple
    expected_body_flu: tuple
    expected_projection: tuple
    pixel_error: tuple
    camera_error: tuple
    body_error: tuple


def entity_geometry_residuals(
    entity_center_ned,
    uav_position_ned,
    attitude_quaternion,
    camera_translation_flu,
    camera_pitch_down,
    intrinsics,
    observed_projection_center,
    observed_center_camera,
    observed_body_flu,
):
    """Compare each observed geometry stage with one timed entity pose."""
    expected_camera = local_ned_target_to_camera_flu(
        entity_center_ned,
        uav_position_ned,
        attitude_quaternion,
        camera_translation_flu,
        camera_pitch_down,
    )
    if expected_camera[0] <= 1e-9:
        raise ValueError('Gazebo entity is not in front of the camera')
    expected_body = camera_target_to_body_flu(
        expected_camera,
        camera_translation_flu,
        camera_pitch_down,
    )
    fx, fy, cx, cy = (float(value) for value in intrinsics)
    if not all(math.isfinite(value) for value in (fx, fy, cx, cy)):
        raise ValueError('camera intrinsics must be finite')
    expected_projection = np.array((
        cx - expected_camera[1] * fx / expected_camera[0],
        cy - expected_camera[2] * fy / expected_camera[0],
    ))
    observed_projection = np.asarray(observed_projection_center, dtype=float)
    observed_camera = np.asarray(observed_center_camera, dtype=float)
    observed_body = np.asarray(observed_body_flu, dtype=float)
    if (
        observed_projection.shape != (2,)
        or observed_camera.shape != (3,)
        or observed_body.shape != (3,)
        or not np.all(np.isfinite(observed_projection))
        or not np.all(np.isfinite(observed_camera))
        or not np.all(np.isfinite(observed_body))
    ):
        raise ValueError('observed geometry diagnostics must be finite')
    return EntityGeometryResiduals(
        expected_camera=tuple(expected_camera),
        expected_body_flu=tuple(expected_body),
        expected_projection=tuple(expected_projection),
        pixel_error=tuple(observed_projection - expected_projection),
        camera_error=tuple(observed_camera - expected_camera),
        body_error=tuple(observed_body - expected_body),
    )


class GazeboEntityPoseTracker:
    """Align sampled Gazebo entity poses with ROS time without receipt time."""

    def __init__(
        self,
        history_duration=1.0,
        maximum_reference_age=0.5,
        clock_history_duration=2.0,
        maximum_system_clock_step=0.25,
    ):
        self.clock_mapper = GazeboImageClockMapper(
            maximum_reference_age=maximum_reference_age,
            history_duration=clock_history_duration,
            maximum_system_clock_step=maximum_system_clock_step,
        )
        self.history = TimestampedVectorHistory(history_duration)
        self.lock = threading.Lock()
        self.last_raw_stamp = math.nan
        self.last_mapped_stamp = math.nan
        self.last_status = 'CLOCK_REFERENCE_UNAVAILABLE'

    @property
    def reset_count(self):
        return self.clock_mapper.reset_count

    def add_clock_anchor(
        self,
        sim_time,
        system_time,
        receipt_ros_time,
        monotonic_time,
    ):
        with self.lock:
            before = self.clock_mapper.reset_count
            accepted = self.clock_mapper.add_anchor(
                sim_time,
                system_time,
                receipt_ros_time,
                monotonic_time,
            )
            if self.clock_mapper.reset_count != before:
                self.history.clear()
            self.last_status = self.clock_mapper.last_status
            return accepted

    def add_pose(self, sim_stamp, gazebo_enu_position, now_ros_time):
        with self.lock:
            self.last_raw_stamp = float(sim_stamp)
            mapped, status = self.clock_mapper.map_time(
                sim_stamp,
                now_ros_time,
            )
            self.last_status = status
            if mapped is None:
                return False
            try:
                position = gazebo_enu_to_ned(gazebo_enu_position)
            except ValueError:
                self.last_status = 'ENTITY_POSITION_INVALID'
                return False
            if not self.history.add(mapped, position):
                self.last_status = 'ENTITY_POSE_OUT_OF_ORDER'
                return False
            self.last_mapped_stamp = mapped
            return True

    def position_at(self, ros_stamp):
        with self.lock:
            return self.history.value_at(ros_stamp)
