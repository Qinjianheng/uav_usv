"""Short-horizon target prediction for curved surface-vessel motion."""

import math


class ManeuveringTargetPredictor:
    """
    Predict target motion with an adaptively estimated coordinated turn.

    The predictor estimates horizontal turn rate from consecutive velocity
    observations.  It uses a constant-turn-rate-and-speed (CTRV) model for a
    configurable short horizon, then continues along the last predicted
    tangent.  Until enough valid observations are available, it falls back to
    constant-velocity prediction.
    """

    def __init__(
        self,
        turn_rate_filter_alpha=0.25,
        max_turn_rate=1.2,
        maneuver_horizon=2.0,
        minimum_speed=0.2,
        minimum_updates=3,
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

        self.turn_rate = 0.0
        self.valid_turn_updates = 0
        self.previous_vx = None
        self.previous_vy = None
        self.previous_timestamp = None

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
                if self.valid_turn_updates == 0:
                    self.turn_rate = raw_turn_rate
                else:
                    alpha = self.turn_rate_filter_alpha
                    self.turn_rate = (
                        alpha * raw_turn_rate
                        + (1.0 - alpha) * self.turn_rate
                    )
                self.valid_turn_updates += 1
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
