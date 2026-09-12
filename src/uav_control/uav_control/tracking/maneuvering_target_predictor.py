"""Bounded short-horizon prediction for an independently manoeuvring target."""

import math


class ManeuveringTargetPredictor:
    """
    Predict target motion with bounded turn and speed acceleration.

    A constant-turn-rate model is systematically late when an evasive target
    changes curvature.  This model estimates turn acceleration and
    longitudinal acceleration online, integrates them with exponential decay,
    and clamps every derivative to a physical envelope.  It requires no known
    route or target cooperation.  Until enough velocity changes have been
    observed it falls back to constant-velocity prediction.
    """

    def __init__(
        self,
        turn_rate_filter_alpha=0.25,
        max_turn_rate=1.2,
        maneuver_horizon=2.0,
        minimum_speed=0.2,
        minimum_updates=3,
        turn_acceleration_filter_alpha=0.2,
        max_turn_acceleration=1.5,
        speed_acceleration_filter_alpha=0.2,
        max_longitudinal_acceleration=2.0,
        acceleration_decay_time=1.0,
        integration_step=0.02,
    ):
        """Configure turn-rate filtering and the curved forecast horizon."""
        self.turn_rate_filter_alpha = self._bounded_value(
            turn_rate_filter_alpha,
            'turn-rate filter alpha',
            0.0,
            1.0,
        )
        self.max_turn_rate = self._positive_value(
            max_turn_rate,
            'maximum turn rate',
        )
        self.maneuver_horizon = self._positive_value(
            maneuver_horizon,
            'maneuver horizon',
        )
        self.minimum_speed = self._positive_value(
            minimum_speed,
            'minimum speed',
        )
        self.minimum_updates = max(int(minimum_updates), 1)
        self.turn_acceleration_filter_alpha = self._bounded_value(
            turn_acceleration_filter_alpha,
            'turn-acceleration filter alpha',
            0.0,
            1.0,
        )
        self.max_turn_acceleration = self._positive_value(
            max_turn_acceleration,
            'maximum turn acceleration',
        )
        self.speed_acceleration_filter_alpha = self._bounded_value(
            speed_acceleration_filter_alpha,
            'speed-acceleration filter alpha',
            0.0,
            1.0,
        )
        self.max_longitudinal_acceleration = self._positive_value(
            max_longitudinal_acceleration,
            'maximum longitudinal acceleration',
        )
        self.acceleration_decay_time = self._positive_value(
            acceleration_decay_time,
            'acceleration decay time',
        )
        self.integration_step = self._positive_value(
            integration_step,
            'integration step',
        )

        self.turn_rate = 0.0
        self.turn_acceleration = 0.0
        self.speed_acceleration = 0.0
        self.valid_turn_updates = 0
        self.previous_vx = None
        self.previous_vy = None
        self.previous_timestamp = None
        self.previous_raw_turn_rate = None

    @staticmethod
    def _positive_value(value, name):
        value = float(value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    @staticmethod
    def _bounded_value(value, name, lower, upper):
        value = float(value)
        if not math.isfinite(value) or not lower <= value <= upper:
            raise ValueError(
                f'{name} must be finite and between {lower} and {upper}'
            )
        return value

    @staticmethod
    def _wrapped_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @property
    def maneuver_model_active(self):
        """Report whether enough velocity changes have been observed."""
        return self.valid_turn_updates >= self.minimum_updates

    def update_velocity(self, vx, vy, timestamp):
        """Update the turn-rate estimate from one velocity observation."""
        vx = float(vx)
        vy = float(vy)
        timestamp = float(timestamp)
        if not all(math.isfinite(value) for value in (vx, vy, timestamp)):
            return False

        updated = False
        if (
            self.previous_timestamp is not None
            and timestamp > self.previous_timestamp
            and self.previous_vx is not None
            and self.previous_vy is not None
        ):
            sample_dt = timestamp - self.previous_timestamp
            previous_speed = math.hypot(
                self.previous_vx,
                self.previous_vy,
            )
            current_speed = math.hypot(vx, vy)
            if (
                1e-3 <= sample_dt <= 0.5
                and previous_speed >= self.minimum_speed
                and current_speed >= self.minimum_speed
            ):
                previous_heading = math.atan2(
                    self.previous_vy,
                    self.previous_vx,
                )
                current_heading = math.atan2(vy, vx)
                raw_turn_rate = self._wrapped_angle(
                    current_heading - previous_heading
                ) / sample_dt
                raw_turn_rate = max(
                    min(raw_turn_rate, self.max_turn_rate),
                    -self.max_turn_rate,
                )
                if self.previous_raw_turn_rate is not None:
                    raw_turn_acceleration = (
                        raw_turn_rate - self.previous_raw_turn_rate
                    ) / sample_dt
                    raw_turn_acceleration = max(
                        min(
                            raw_turn_acceleration,
                            self.max_turn_acceleration,
                        ),
                        -self.max_turn_acceleration,
                    )
                    alpha = self.turn_acceleration_filter_alpha
                    self.turn_acceleration = (
                        alpha * raw_turn_acceleration
                        + (1.0 - alpha) * self.turn_acceleration
                    )
                raw_speed_acceleration = (
                    current_speed - previous_speed
                ) / sample_dt
                raw_speed_acceleration = max(
                    min(
                        raw_speed_acceleration,
                        self.max_longitudinal_acceleration,
                    ),
                    -self.max_longitudinal_acceleration,
                )
                speed_alpha = self.speed_acceleration_filter_alpha
                self.speed_acceleration = (
                    speed_alpha * raw_speed_acceleration
                    + (1.0 - speed_alpha) * self.speed_acceleration
                )
                if self.valid_turn_updates == 0:
                    self.turn_rate = raw_turn_rate
                else:
                    alpha = self.turn_rate_filter_alpha
                    self.turn_rate = (
                        alpha * raw_turn_rate
                        + (1.0 - alpha) * self.turn_rate
                    )
                self.valid_turn_updates += 1
                self.previous_raw_turn_rate = raw_turn_rate
                updated = True

        self.previous_vx = vx
        self.previous_vy = vy
        self.previous_timestamp = timestamp
        return updated

    @staticmethod
    def _constant_turn_state(x, y, speed, heading, turn_rate, dt):
        if abs(turn_rate) <= 1e-6:
            vx = speed * math.cos(heading)
            vy = speed * math.sin(heading)
            return x + vx * dt, y + vy * dt, vx, vy

        final_heading = heading + turn_rate * dt
        radius = speed / turn_rate
        predicted_x = x + radius * (
            math.sin(final_heading) - math.sin(heading)
        )
        predicted_y = y - radius * (
            math.cos(final_heading) - math.cos(heading)
        )
        predicted_vx = speed * math.cos(final_heading)
        predicted_vy = speed * math.sin(final_heading)
        return predicted_x, predicted_y, predicted_vx, predicted_vy

    def predict(self, x, y, z, vx, vy, vz, horizon):
        """Return predicted ``(x, y, z, vx, vy, vz)`` at ``horizon``."""
        values = tuple(
            float(value)
            for value in (x, y, z, vx, vy, vz, horizon)
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError('prediction inputs must be finite')

        x, y, z, vx, vy, vz, horizon = values
        if horizon < 0.0:
            raise ValueError('prediction horizon must be non-negative')
        if horizon == 0.0:
            return x, y, z, vx, vy, vz

        speed = math.hypot(vx, vy)
        if not self.maneuver_model_active or speed < self.minimum_speed:
            return (
                x + vx * horizon,
                y + vy * horizon,
                z + vz * horizon,
                vx,
                vy,
                vz,
            )

        heading = math.atan2(vy, vx)
        curved_time = min(horizon, self.maneuver_horizon)
        if (
            abs(self.turn_acceleration) <= 1e-12
            and abs(self.speed_acceleration) <= 1e-12
        ):
            (
                predicted_x,
                predicted_y,
                predicted_vx,
                predicted_vy,
            ) = self._constant_turn_state(
                x,
                y,
                speed,
                heading,
                self.turn_rate,
                curved_time,
            )
            straight_time = horizon - curved_time
            return (
                predicted_x + predicted_vx * straight_time,
                predicted_y + predicted_vy * straight_time,
                z + vz * horizon,
                predicted_vx,
                predicted_vy,
                vz,
            )
        predicted_x = x
        predicted_y = y
        predicted_speed = speed
        predicted_turn_rate = self.turn_rate
        elapsed = 0.0
        while elapsed < curved_time - 1e-12:
            step = min(self.integration_step, curved_time - elapsed)
            decay = math.exp(
                -(elapsed + 0.5 * step) / self.acceleration_decay_time
            )
            turn_acceleration = self.turn_acceleration * decay
            longitudinal_acceleration = self.speed_acceleration * decay
            next_turn_rate = max(
                min(
                    predicted_turn_rate + turn_acceleration * step,
                    self.max_turn_rate,
                ),
                -self.max_turn_rate,
            )
            next_speed = max(
                predicted_speed + longitudinal_acceleration * step,
                0.0,
            )
            midpoint_turn_rate = 0.5 * (
                predicted_turn_rate + next_turn_rate
            )
            midpoint_speed = 0.5 * (predicted_speed + next_speed)
            midpoint_heading = heading + 0.5 * midpoint_turn_rate * step
            predicted_x += midpoint_speed * math.cos(
                midpoint_heading
            ) * step
            predicted_y += midpoint_speed * math.sin(
                midpoint_heading
            ) * step
            heading += midpoint_turn_rate * step
            predicted_turn_rate = next_turn_rate
            predicted_speed = next_speed
            elapsed += step

        predicted_vx = predicted_speed * math.cos(heading)
        predicted_vy = predicted_speed * math.sin(heading)

        straight_time = horizon - curved_time
        predicted_x += predicted_vx * straight_time
        predicted_y += predicted_vy * straight_time
        predicted_z = z + vz * horizon
        return (
            predicted_x,
            predicted_y,
            predicted_z,
            predicted_vx,
            predicted_vy,
            vz,
        )

    def acceleration(self, predicted_vx, predicted_vy, horizon):
        """Return bounded horizontal acceleration at a prediction horizon."""
        horizon = max(min(float(horizon), self.maneuver_horizon), 0.0)
        decay = math.exp(-horizon / self.acceleration_decay_time)
        integrated_turn_change = (
            self.turn_acceleration
            * self.acceleration_decay_time
            * (1.0 - decay)
        )
        turn_rate = max(
            min(
                self.turn_rate + integrated_turn_change,
                self.max_turn_rate,
            ),
            -self.max_turn_rate,
        )
        speed = math.hypot(predicted_vx, predicted_vy)
        if speed < self.minimum_speed:
            return 0.0, 0.0, 0.0
        direction_x = predicted_vx / speed
        direction_y = predicted_vy / speed
        longitudinal = self.speed_acceleration * decay
        return (
            longitudinal * direction_x - turn_rate * predicted_vy,
            longitudinal * direction_y + turn_rate * predicted_vx,
            0.0,
        )
