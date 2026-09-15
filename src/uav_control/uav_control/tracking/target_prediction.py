"""Timestamp-preserving target prediction independent of ROS transport."""

import math
from dataclasses import dataclass

from .maneuvering_target_predictor import ManeuveringTargetPredictor


@dataclass(frozen=True)
class TargetKinematicState:
    """One timestamped target state in the local planning frame."""

    stamp: float
    position: tuple
    velocity: tuple


@dataclass(frozen=True)
class PredictedSample:
    """One point in a target prediction series."""

    relative_time: float
    position: tuple
    velocity: tuple
    acceleration: tuple


@dataclass(frozen=True)
class PredictionResult:
    """Transport-neutral target prediction result."""

    mission_id: int
    sequence_id: int
    source_stamp: float
    generated_stamp: float
    valid_until: float
    source: str
    model: str
    horizon: float
    samples: tuple
    turn_rate: float
    turn_acceleration: float
    longitudinal_acceleration: float
    valid: bool
    invalid_reason: str


class PredictionEngine:
    """Generate bounded BCTRA prediction series from the latest input only."""

    def __init__(
        self,
        predictor=None,
        horizon=3.0,
        sample_period=0.1,
        input_timeout=0.125,
        source='simulation_truth',
    ):
        self.predictor = predictor or ManeuveringTargetPredictor()
        self.horizon = self._positive(horizon, 'prediction horizon')
        self.sample_period = self._positive(
            sample_period,
            'prediction sample period',
        )
        self.input_timeout = self._positive(
            input_timeout,
            'prediction input timeout',
        )
        self.source = str(source)
        self.latest_state = None

    @staticmethod
    def _positive(value, name):
        value = float(value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    @staticmethod
    def _finite_state(state):
        values = (
            state.stamp,
            *state.position,
            *state.velocity,
        )
        return len(state.position) == 3 and len(state.velocity) == 3 and all(
            math.isfinite(float(value)) for value in values
        )

    def update(self, state):
        """Accept a finite, newer state and update motion history."""
        if not self._finite_state(state):
            return False
        if (
            self.latest_state is not None
            and state.stamp <= self.latest_state.stamp
        ):
            return False
        self.predictor.update_velocity(
            state.velocity[0],
            state.velocity[1],
            state.stamp,
        )
        self.predictor.update_vertical(
            state.position[2],
            state.velocity[2],
            state.stamp,
        )
        self.latest_state = state
        return True

    def _sample_times(self):
        count = int(math.floor(self.horizon / self.sample_period))
        sample_times = [
            index * self.sample_period for index in range(count + 1)
        ]
        if self.horizon - sample_times[-1] > 1e-9:
            sample_times.append(self.horizon)
        else:
            sample_times[-1] = self.horizon
        return sample_times

    def _invalid(self, now, mission_id, sequence_id, reason):
        source_stamp = (
            self.latest_state.stamp if self.latest_state is not None else now
        )
        return PredictionResult(
            mission_id=int(mission_id),
            sequence_id=int(sequence_id),
            source_stamp=source_stamp,
            generated_stamp=float(now),
            valid_until=source_stamp + self.input_timeout,
            source=self.source,
            model='BCTRA_BOUNDED_Z',
            horizon=self.horizon,
            samples=(),
            turn_rate=self.predictor.turn_rate,
            turn_acceleration=self.predictor.turn_acceleration,
            longitudinal_acceleration=self.predictor.speed_acceleration,
            valid=False,
            invalid_reason=reason,
        )

    def generate(self, now, mission_id, sequence_id):
        """Generate one prediction while retaining the input source stamp."""
        now = float(now)
        if not math.isfinite(now):
            raise ValueError('generation time must be finite')
        if self.latest_state is None:
            return self._invalid(
                now,
                mission_id,
                sequence_id,
                'NO_TARGET_STATE',
            )
        age = now - self.latest_state.stamp
        if age < 0.0:
            return self._invalid(
                now,
                mission_id,
                sequence_id,
                'STATE_TIME_IN_FUTURE',
            )
        if age > self.input_timeout:
            return self._invalid(
                now,
                mission_id,
                sequence_id,
                'STATE_STALE',
            )

        state = self.latest_state
        samples = []
        for relative_time in self._sample_times():
            predicted = self.predictor.predict(
                *state.position,
                *state.velocity,
                relative_time,
            )
            acceleration = self.predictor.acceleration(
                predicted[3],
                predicted[4],
                relative_time,
            )
            acceleration = (
                acceleration[0],
                acceleration[1],
                self.predictor.vertical_acceleration(
                    state.velocity[2],
                    relative_time,
                ),
            )
            samples.append(PredictedSample(
                relative_time=relative_time,
                position=predicted[:3],
                velocity=predicted[3:],
                acceleration=acceleration,
            ))

        return PredictionResult(
            mission_id=int(mission_id),
            sequence_id=int(sequence_id),
            source_stamp=state.stamp,
            generated_stamp=now,
            valid_until=state.stamp + self.input_timeout,
            source=self.source,
            model='BCTRA_BOUNDED_Z',
            horizon=self.horizon,
            samples=tuple(samples),
            turn_rate=self.predictor.turn_rate,
            turn_acceleration=self.predictor.turn_acceleration,
            longitudinal_acceleration=self.predictor.speed_acceleration,
            valid=True,
            invalid_reason='',
        )
