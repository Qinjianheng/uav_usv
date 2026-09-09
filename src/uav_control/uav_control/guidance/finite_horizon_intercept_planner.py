"""Finite-horizon polynomial trajectory planning for moving interception."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class TrajectorySample:
    """Position, velocity, acceleration, and jerk at one trajectory time."""

    position: tuple
    velocity: tuple
    acceleration: tuple
    jerk: tuple


@dataclass(frozen=True)
class QuinticAxis:
    """Quintic polynomial satisfying position, velocity, and acceleration."""

    coefficients: tuple

    @classmethod
    def from_boundary(cls, p0, v0, a0, pf, vf, af, duration):
        """Construct an axis polynomial from complete endpoint states."""
        duration = float(duration)
        if not math.isfinite(duration) or duration <= 0.0:
            raise ValueError('duration must be finite and positive')
        values = tuple(float(value) for value in (p0, v0, a0, pf, vf, af))
        if not all(math.isfinite(value) for value in values):
            raise ValueError('boundary states must be finite')

        p0, v0, a0, pf, vf, af = values
        t2 = duration * duration
        t3 = t2 * duration
        t4 = t3 * duration
        t5 = t4 * duration
        c0 = p0
        c1 = v0
        c2 = 0.5 * a0
        c3 = (
            20.0 * (pf - p0)
            - (8.0 * vf + 12.0 * v0) * duration
            - (3.0 * a0 - af) * t2
        ) / (2.0 * t3)
        c4 = (
            30.0 * (p0 - pf)
            + (14.0 * vf + 16.0 * v0) * duration
            + (3.0 * a0 - 2.0 * af) * t2
        ) / (2.0 * t4)
        c5 = (
            12.0 * (pf - p0)
            - (6.0 * vf + 6.0 * v0) * duration
            - (a0 - af) * t2
        ) / (2.0 * t5)
        return cls((c0, c1, c2, c3, c4, c5))

    def sample(self, time):
        """Return position, velocity, acceleration, and jerk."""
        t = float(time)
        if not math.isfinite(t) or t < 0.0:
            raise ValueError('sample time must be finite and non-negative')
        c0, c1, c2, c3, c4, c5 = self.coefficients
        position = (
            c0 + c1 * t + c2 * t**2 + c3 * t**3
            + c4 * t**4 + c5 * t**5
        )
        velocity = (
            c1 + 2.0 * c2 * t + 3.0 * c3 * t**2
            + 4.0 * c4 * t**3 + 5.0 * c5 * t**4
        )
        acceleration = (
            2.0 * c2 + 6.0 * c3 * t + 12.0 * c4 * t**2
            + 20.0 * c5 * t**3
        )
        jerk = 6.0 * c3 + 24.0 * c4 * t + 60.0 * c5 * t**2
        return position, velocity, acceleration, jerk


@dataclass(frozen=True)
class InterceptPlan:
    """One dynamically checked terminal interception trajectory."""

    axes: tuple
    duration: float
    closing_speed: float
    target_position: tuple
    target_velocity: tuple
    target_acceleration: tuple
    maximum_horizontal_speed: float
    maximum_vertical_speed: float
    maximum_horizontal_acceleration: float
    maximum_vertical_acceleration: float
    cost: float

    def sample(self, time):
        """Sample all three axes of the planned trajectory."""
        values = [
            axis.sample(min(max(float(time), 0.0), self.duration))
            for axis in self.axes
        ]
        return TrajectorySample(
            tuple(value[0] for value in values),
            tuple(value[1] for value in values),
            tuple(value[2] for value in values),
            tuple(value[3] for value in values),
        )


class FiniteHorizonInterceptPlanner:
    """Search terminal time and closing speed for a feasible trajectory."""

    def __init__(
        self,
        minimum_duration,
        maximum_duration,
        duration_step,
        sample_step,
        maximum_horizontal_speed,
        maximum_vertical_speed,
        maximum_horizontal_acceleration,
        maximum_vertical_acceleration,
        desired_closing_speed,
        minimum_closing_speed,
        closing_speed_step,
        capture_radius,
        sea_surface_z,
        contact_clearance,
    ):
        """Configure the finite-horizon feasibility search."""
        self.minimum_duration = self._positive(
            minimum_duration,
            'minimum duration',
        )
        self.maximum_duration = self._positive(
            maximum_duration,
            'maximum duration',
        )
        if self.maximum_duration < self.minimum_duration:
            raise ValueError('maximum duration must not be less than minimum')
        self.duration_step = self._positive(duration_step, 'duration step')
        self.sample_step = self._positive(sample_step, 'sample step')
        self.maximum_horizontal_speed = self._positive(
            maximum_horizontal_speed,
            'maximum horizontal speed',
        )
        self.maximum_vertical_speed = self._positive(
            maximum_vertical_speed,
            'maximum vertical speed',
        )
        self.maximum_horizontal_acceleration = self._positive(
            maximum_horizontal_acceleration,
            'maximum horizontal acceleration',
        )
        self.maximum_vertical_acceleration = self._positive(
            maximum_vertical_acceleration,
            'maximum vertical acceleration',
        )
        self.desired_closing_speed = self._positive(
            desired_closing_speed,
            'desired closing speed',
        )
        self.minimum_closing_speed = self._positive(
            minimum_closing_speed,
            'minimum closing speed',
        )
        self.minimum_closing_speed = min(
            self.minimum_closing_speed,
            self.desired_closing_speed,
        )
        self.closing_speed_step = self._positive(
            closing_speed_step,
            'closing speed step',
        )
        self.capture_radius = self._positive(capture_radius, 'capture radius')
        self.sea_surface_z = float(sea_surface_z)
        self.contact_clearance = self._positive(
            contact_clearance,
            'contact clearance',
        )
        if not math.isfinite(self.sea_surface_z):
            raise ValueError('sea surface z must be finite')
        if self.contact_clearance >= self.capture_radius:
            raise ValueError('contact clearance must be below capture radius')

    @staticmethod
    def _positive(value, name):
        value = float(value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    @staticmethod
    def _vector(values, name):
        vector = tuple(float(value) for value in values)
        if (
            len(vector) != 3
            or not all(math.isfinite(value) for value in vector)
        ):
            raise ValueError(f'{name} must contain three finite values')
        return vector

    @staticmethod
    def _direction(start, end, fallback_velocity):
        delta = tuple(final - initial for initial, final in zip(start, end))
        distance = math.sqrt(sum(value * value for value in delta))
        if distance > 1e-9:
            return tuple(value / distance for value in delta)
        speed = math.sqrt(sum(value * value for value in fallback_velocity))
        if speed > 1e-9:
            return tuple(value / speed for value in fallback_velocity)
        return 1.0, 0.0, 0.0

    def _duration_candidates(self):
        count = int(math.floor(
            (self.maximum_duration - self.minimum_duration)
            / self.duration_step
            + 1e-9
        ))
        durations = [
            self.minimum_duration + index * self.duration_step
            for index in range(count + 1)
        ]
        if durations[-1] < self.maximum_duration - 1e-9:
            durations.append(self.maximum_duration)
        return durations

    def _closing_speed_candidates(self):
        speeds = []
        speed = self.desired_closing_speed
        while speed > self.minimum_closing_speed + 1e-9:
            speeds.append(speed)
            speed -= self.closing_speed_step
        speeds.append(self.minimum_closing_speed)
        return speeds

    def _candidate(
        self,
        initial_position,
        initial_velocity,
        initial_acceleration,
        target_position,
        target_velocity,
        target_acceleration,
        duration,
        closing_speed,
        previous_acceleration,
    ):
        contact_position = list(target_position)
        contact_position[2] = min(
            contact_position[2],
            self.sea_surface_z - self.contact_clearance,
        )
        if abs(contact_position[2] - target_position[2]) > self.capture_radius:
            return None
        contact_position = tuple(contact_position)
        direction = self._direction(
            initial_position,
            contact_position,
            target_velocity,
        )
        terminal_velocity = tuple(
            target_component + closing_speed * direction_component
            for target_component, direction_component in zip(
                target_velocity,
                direction,
            )
        )
        axes = tuple(
            QuinticAxis.from_boundary(
                initial_position[index],
                initial_velocity[index],
                initial_acceleration[index],
                contact_position[index],
                terminal_velocity[index],
                target_acceleration[index],
                duration,
            )
            for index in range(3)
        )

        maximum_horizontal_speed = 0.0
        maximum_vertical_speed = 0.0
        maximum_horizontal_acceleration = 0.0
        maximum_vertical_acceleration = 0.0
        effort_cost = 0.0
        sample_count = max(int(math.ceil(duration / self.sample_step)), 1)
        for index in range(sample_count + 1):
            time = min(index * duration / sample_count, duration)
            values = [axis.sample(time) for axis in axes]
            velocity = tuple(value[1] for value in values)
            acceleration = tuple(value[2] for value in values)
            jerk = tuple(value[3] for value in values)
            horizontal_speed = math.hypot(velocity[0], velocity[1])
            vertical_speed = abs(velocity[2])
            horizontal_acceleration = math.hypot(
                acceleration[0],
                acceleration[1],
            )
            vertical_acceleration = abs(acceleration[2])
            maximum_horizontal_speed = max(
                maximum_horizontal_speed,
                horizontal_speed,
            )
            maximum_vertical_speed = max(maximum_vertical_speed, vertical_speed)
            maximum_horizontal_acceleration = max(
                maximum_horizontal_acceleration,
                horizontal_acceleration,
            )
            maximum_vertical_acceleration = max(
                maximum_vertical_acceleration,
                vertical_acceleration,
            )
            if (
                horizontal_speed > self.maximum_horizontal_speed + 1e-6
                or vertical_speed > self.maximum_vertical_speed + 1e-6
                or horizontal_acceleration
                > self.maximum_horizontal_acceleration + 1e-6
                or vertical_acceleration
                > self.maximum_vertical_acceleration + 1e-6
                or values[2][0] >= self.sea_surface_z
            ):
                return None
            effort_cost += (
                horizontal_acceleration**2
                + vertical_acceleration**2
                + 0.01 * sum(component**2 for component in jerk)
            ) * duration / sample_count

        first_values = [
            axis.sample(min(self.sample_step, duration))
            for axis in axes
        ]
        first_acceleration = tuple(value[2] for value in first_values)
        continuity_cost = sum(
            (current - previous) ** 2
            for current, previous in zip(
                first_acceleration,
                previous_acceleration,
            )
        )
        closing_relaxation = (
            self.desired_closing_speed - closing_speed
        ) ** 2
        cost = (
            duration
            + 0.02 * effort_cost
            + 0.1 * continuity_cost
            + 0.2 * closing_relaxation
        )
        return InterceptPlan(
            axes,
            duration,
            closing_speed,
            contact_position,
            target_velocity,
            target_acceleration,
            maximum_horizontal_speed,
            maximum_vertical_speed,
            maximum_horizontal_acceleration,
            maximum_vertical_acceleration,
            cost,
        )

    def plan(
        self,
        initial_position,
        initial_velocity,
        initial_acceleration,
        target_state_at_time,
        previous_acceleration=(0.0, 0.0, 0.0),
    ):
        """Return the lowest-cost feasible trajectory or ``None``."""
        initial_position = self._vector(initial_position, 'initial position')
        initial_velocity = self._vector(initial_velocity, 'initial velocity')
        initial_acceleration = self._vector(
            initial_acceleration,
            'initial acceleration',
        )
        previous_acceleration = self._vector(
            previous_acceleration,
            'previous acceleration',
        )
        best_plan = None
        for duration in self._duration_candidates():
            target_state = target_state_at_time(duration)
            if len(target_state) != 3:
                raise ValueError(
                    'target state must contain position, velocity, '
                    'acceleration'
                )
            target_position = self._vector(target_state[0], 'target position')
            target_velocity = self._vector(target_state[1], 'target velocity')
            target_acceleration = self._vector(
                target_state[2],
                'target acceleration',
            )
            for closing_speed in self._closing_speed_candidates():
                candidate = self._candidate(
                    initial_position,
                    initial_velocity,
                    initial_acceleration,
                    target_position,
                    target_velocity,
                    target_acceleration,
                    duration,
                    closing_speed,
                    previous_acceleration,
                )
                if candidate is not None and (
                    best_plan is None or candidate.cost < best_plan.cost
                ):
                    best_plan = candidate
        return best_plan
