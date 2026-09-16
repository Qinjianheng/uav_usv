"""Realtime polynomial tracking independent of planning and ROS transport."""

import math
from dataclasses import dataclass
from enum import Enum

from uav_control.common.sea_safety import apply_sea_safety_guard


class TrajectoryRejectReason(str, Enum):
    """Reasons a new plan cannot atomically replace the active trajectory."""

    NONE = 'NONE'
    MISSION_MISMATCH = 'MISSION_MISMATCH'
    PREDICTION_MISMATCH = 'PREDICTION_MISMATCH'
    FRAME_MISMATCH = 'FRAME_MISMATCH'
    SOURCE_STALE = 'SOURCE_STALE'
    EXPIRED = 'EXPIRED'
    INSUFFICIENT_REMAINING_TIME = 'INSUFFICIENT_REMAINING_TIME'
    STATE_POSITION_MISMATCH = 'STATE_POSITION_MISMATCH'
    STATE_VELOCITY_MISMATCH = 'STATE_VELOCITY_MISMATCH'
    TARGET_ENDPOINT_MISMATCH = 'TARGET_ENDPOINT_MISMATCH'
    SAFETY_REJECTED = 'SAFETY_REJECTED'
    INVALID_TRAJECTORY = 'INVALID_TRAJECTORY'


@dataclass(frozen=True)
class PolynomialSegmentData:
    """One local-time three-axis quintic segment."""

    duration: float
    coefficients: tuple

    def __post_init__(self):
        if (
            not math.isfinite(float(self.duration))
            or self.duration <= 0.0
            or len(self.coefficients) != 18
            or not all(math.isfinite(value) for value in self.coefficients)
        ):
            raise ValueError('polynomial segment must be finite and complete')

    def sample(self, local_time):
        """Return position, velocity, and acceleration at local time."""
        local_time = min(max(float(local_time), 0.0), self.duration)
        position = []
        velocity = []
        acceleration = []
        for axis in range(3):
            offset = 6 * axis
            c0, c1, c2, c3, c4, c5 = self.coefficients[
                offset:offset + 6
            ]
            t = local_time
            position.append(
                c0 + c1 * t + c2 * t**2 + c3 * t**3
                + c4 * t**4 + c5 * t**5
            )
            velocity.append(
                c1 + 2.0 * c2 * t + 3.0 * c3 * t**2
                + 4.0 * c4 * t**3 + 5.0 * c5 * t**4
            )
            acceleration.append(
                2.0 * c2 + 6.0 * c3 * t + 12.0 * c4 * t**2
                + 20.0 * c5 * t**3
            )
        return TrajectorySample(
            tuple(position),
            tuple(velocity),
            tuple(acceleration),
        )


@dataclass(frozen=True)
class TrajectorySample:
    """One desired state sampled from a polynomial trajectory."""

    position: tuple
    velocity: tuple
    acceleration: tuple


@dataclass(frozen=True)
class PolynomialTrajectory:
    """Complete cross-process MINCO trajectory with original timing."""

    mission_id: int
    plan_id: int
    prediction_sequence_id: int
    source_stamp: float
    generated_stamp: float
    valid_until: float
    segments: tuple
    terminal_position: tuple
    terminal_velocity: tuple
    target_state_source: str
    frame_id: str = 'local_ned'
    contact_stamp: float = 0.0
    selected_t_go: float = 0.0
    remaining_t_go: float = 0.0
    terminal_mode: bool = False
    planned_capture_margin: float = 0.0

    @property
    def duration(self):
        """Return total piece duration."""
        return sum(segment.duration for segment in self.segments)

    def sample(self, elapsed):
        """Sample piecewise coefficients using each segment's local time."""
        if not self.segments:
            raise ValueError('trajectory must contain at least one segment')
        remaining = min(max(float(elapsed), 0.0), self.duration)
        for index, segment in enumerate(self.segments):
            if remaining <= segment.duration or index == len(self.segments) - 1:
                return segment.sample(remaining)
            remaining -= segment.duration
        raise RuntimeError('trajectory sampling failed')

    def sample_at_ros_time(self, now):
        """Sample against source time, never against generation time."""
        return self.sample(float(now) - self.source_stamp)


@dataclass(frozen=True)
class TrackerKinematicState:
    """Current UAV state in the ROS clock domain."""

    stamp: float
    position: tuple
    velocity: tuple


@dataclass(frozen=True)
class TrackingCommand:
    """Post-limit command ready for PX4 publication."""

    position: tuple
    velocity: tuple
    acceleration: tuple
    plan_id: int
    safety_state: str
    safety_margin: float


class TrajectoryTrackerCore:
    """Atomically accept and track valid plans with final sea protection."""

    def __init__(
        self,
        maximum_plan_age=0.125,
        minimum_remaining_time=0.20,
        maximum_position_error=0.30,
        maximum_velocity_error=0.50,
        target_endpoint_tolerance=0.50,
        expected_frame_id='local_ned',
        position_gain=1.2,
        maximum_horizontal_speed=6.5,
        maximum_vertical_speed=4.0,
        maximum_horizontal_acceleration=3.0,
        maximum_vertical_acceleration=3.0,
        sea_surface_z=0.0,
        reserve_clearance=0.07,
        safety_response_delay=0.15,
        vertical_braking_acceleration=2.5,
        control_dt=0.05,
    ):
        self.maximum_plan_age = float(maximum_plan_age)
        self.minimum_remaining_time = float(minimum_remaining_time)
        self.maximum_position_error = float(maximum_position_error)
        self.maximum_velocity_error = float(maximum_velocity_error)
        self.target_endpoint_tolerance = float(target_endpoint_tolerance)
        self.expected_frame_id = str(expected_frame_id)
        self.position_gain = float(position_gain)
        self.maximum_horizontal_speed = float(maximum_horizontal_speed)
        self.maximum_vertical_speed = float(maximum_vertical_speed)
        self.maximum_horizontal_acceleration = float(
            maximum_horizontal_acceleration
        )
        self.maximum_vertical_acceleration = float(
            maximum_vertical_acceleration
        )
        self.sea_surface_z = float(sea_surface_z)
        self.reserve_clearance = float(reserve_clearance)
        self.safety_response_delay = float(safety_response_delay)
        self.vertical_braking_acceleration = float(
            vertical_braking_acceleration
        )
        self.control_dt = float(control_dt)
        self.active_trajectory = None
        self.status = 'NO_VALID_PLAN'
        self.previous_command_velocity = None
        self.previous_command_stamp = None

    @staticmethod
    def _norm(values):
        return math.sqrt(sum(value * value for value in values))

    def reset(self):
        """Discard mission-scoped state without retaining prior commands."""
        self.active_trajectory = None
        self.status = 'NO_VALID_PLAN'
        self.previous_command_velocity = None
        self.previous_command_stamp = None

    def accept(
        self,
        trajectory,
        state,
        mission_id,
        prediction_sequence_id=None,
        target_endpoint=None,
        maximum_position_error=None,
    ):
        """Validate then atomically replace the active trajectory."""
        if trajectory.mission_id != int(mission_id):
            return TrajectoryRejectReason.MISSION_MISMATCH
        if trajectory.frame_id != self.expected_frame_id:
            return TrajectoryRejectReason.FRAME_MISMATCH
        source_age = state.stamp - trajectory.source_stamp
        if source_age < 0.0 or source_age > self.maximum_plan_age:
            return TrajectoryRejectReason.SOURCE_STALE
        if state.stamp >= trajectory.valid_until or not trajectory.segments:
            return TrajectoryRejectReason.EXPIRED
        if (
            trajectory.valid_until - state.stamp
            < self.minimum_remaining_time
        ):
            return TrajectoryRejectReason.INSUFFICIENT_REMAINING_TIME
        try:
            expected = trajectory.sample_at_ros_time(state.stamp)
        except ValueError:
            return TrajectoryRejectReason.INVALID_TRAJECTORY
        position_error = self._norm(tuple(
            current - desired
            for current, desired in zip(state.position, expected.position)
        ))
        position_limit = (
            self.maximum_position_error
            if maximum_position_error is None
            else float(maximum_position_error)
        )
        if position_error > position_limit:
            return TrajectoryRejectReason.STATE_POSITION_MISMATCH
        velocity_error = self._norm(tuple(
            current - desired
            for current, desired in zip(state.velocity, expected.velocity)
        ))
        if velocity_error > self.maximum_velocity_error:
            return TrajectoryRejectReason.STATE_VELOCITY_MISMATCH
        if target_endpoint is not None:
            endpoint_error = self._norm(tuple(
                predicted - planned
                for predicted, planned in zip(
                    target_endpoint,
                    trajectory.terminal_position,
                )
            ))
            if endpoint_error > self.target_endpoint_tolerance:
                return TrajectoryRejectReason.TARGET_ENDPOINT_MISMATCH
        safety = apply_sea_safety_guard(
            current_z=state.position[2],
            current_vz=state.velocity[2],
            proposed_vz=expected.velocity[2],
            sea_surface_z=self.sea_surface_z,
            reserve_clearance=self.reserve_clearance,
            response_delay=self.safety_response_delay,
            effective_braking_acceleration=(
                self.vertical_braking_acceleration
            ),
            control_dt=self.control_dt,
            maximum_vertical_speed=self.maximum_vertical_speed,
        )
        if safety.unrecoverable or safety.response_margin <= 0.0:
            return TrajectoryRejectReason.SAFETY_REJECTED
        self.active_trajectory = trajectory
        self.status = 'TRACKING'
        return TrajectoryRejectReason.NONE

    @staticmethod
    def _limit_horizontal(values, maximum):
        magnitude = math.hypot(values[0], values[1])
        if magnitude <= maximum or magnitude <= 1e-12:
            return values[0], values[1]
        scale = maximum / magnitude
        return values[0] * scale, values[1] * scale

    def _shape_velocity(self, desired, stamp):
        horizontal = self._limit_horizontal(
            desired[:2],
            self.maximum_horizontal_speed,
        )
        limited = (
            horizontal[0],
            horizontal[1],
            max(min(desired[2], self.maximum_vertical_speed),
                -self.maximum_vertical_speed),
        )
        if self.previous_command_velocity is None:
            return limited
        dt = max(float(stamp) - self.previous_command_stamp, self.control_dt)
        delta_xy = (
            limited[0] - self.previous_command_velocity[0],
            limited[1] - self.previous_command_velocity[1],
        )
        limited_delta_xy = self._limit_horizontal(
            delta_xy,
            self.maximum_horizontal_acceleration * dt,
        )
        delta_z = max(
            min(
                limited[2] - self.previous_command_velocity[2],
                self.maximum_vertical_acceleration * dt,
            ),
            -self.maximum_vertical_acceleration * dt,
        )
        return (
            self.previous_command_velocity[0] + limited_delta_xy[0],
            self.previous_command_velocity[1] + limited_delta_xy[1],
            self.previous_command_velocity[2] + delta_z,
        )

    def command(self, state, mission_id):
        """Return one post-Safety-Guard command or no valid-plan status."""
        trajectory = self.active_trajectory
        if (
            trajectory is None
            or trajectory.mission_id != int(mission_id)
            or state.stamp >= trajectory.valid_until
        ):
            self.active_trajectory = None
            self.status = 'NO_VALID_PLAN'
            return None
        desired = trajectory.sample_at_ros_time(state.stamp)
        feedback_velocity = tuple(
            velocity + self.position_gain * (reference - current)
            for velocity, reference, current in zip(
                desired.velocity,
                desired.position,
                state.position,
            )
        )
        previous_velocity = self.previous_command_velocity
        previous_stamp = self.previous_command_stamp
        velocity = self._shape_velocity(feedback_velocity, state.stamp)
        safety = apply_sea_safety_guard(
            current_z=state.position[2],
            current_vz=state.velocity[2],
            proposed_vz=velocity[2],
            sea_surface_z=self.sea_surface_z,
            reserve_clearance=self.reserve_clearance,
            response_delay=self.safety_response_delay,
            effective_braking_acceleration=(
                self.vertical_braking_acceleration
            ),
            control_dt=self.control_dt,
            maximum_vertical_speed=self.maximum_vertical_speed,
        )
        velocity = (velocity[0], velocity[1], safety.command_vz)
        velocity_changed = self._norm(tuple(
            commanded - planned
            for commanded, planned in zip(velocity, desired.velocity)
        )) > 1e-6
        if velocity_changed and previous_velocity is not None:
            dt = max(
                float(state.stamp) - float(previous_stamp),
                self.control_dt,
            )
            raw_acceleration = tuple(
                (current - previous) / dt
                for current, previous in zip(velocity, previous_velocity)
            )
            horizontal_acceleration = self._limit_horizontal(
                raw_acceleration[:2],
                self.maximum_horizontal_acceleration,
            )
            acceleration = (
                horizontal_acceleration[0],
                horizontal_acceleration[1],
                max(
                    min(
                        raw_acceleration[2],
                        self.maximum_vertical_acceleration,
                    ),
                    -self.maximum_vertical_acceleration,
                ),
            )
        elif velocity_changed:
            acceleration = (math.nan, math.nan, math.nan)
        else:
            acceleration = desired.acceleration
        self.previous_command_velocity = velocity
        self.previous_command_stamp = state.stamp
        self.status = 'TRACKING'
        return TrackingCommand(
            position=desired.position,
            velocity=velocity,
            acceleration=acceleration,
            plan_id=trajectory.plan_id,
            safety_state=safety.state.value,
            safety_margin=safety.response_margin,
        )
