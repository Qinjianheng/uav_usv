"""Transport-neutral input handling for the independent MINCO planner."""

import math
import threading
from dataclasses import dataclass

from .fast_minco_planner import FastPlanningFailure


@dataclass(frozen=True)
class PredictionSample:
    """One timestamp-relative state in a target prediction."""

    relative_time: float
    position: tuple
    velocity: tuple
    acceleration: tuple


@dataclass(frozen=True)
class PredictionSeries:
    """One immutable prediction snapshot."""

    mission_id: int
    sequence_id: int
    source_stamp: float
    valid_until: float
    samples: tuple
    source: str

    @staticmethod
    def _blend(first, second, fraction):
        return tuple(
            start + fraction * (end - start)
            for start, end in zip(first, second)
        )

    def state_at(self, relative_time):
        """Linearly interpolate one state without extrapolating the horizon."""
        if not self.samples:
            raise ValueError('prediction series must contain samples')
        relative_time = float(relative_time)
        if not math.isfinite(relative_time) or relative_time < 0.0:
            raise ValueError('prediction time must be finite and non-negative')
        if relative_time <= self.samples[0].relative_time:
            sample = self.samples[0]
            return sample.position, sample.velocity, sample.acceleration
        for previous, current in zip(self.samples, self.samples[1:]):
            if relative_time <= current.relative_time:
                interval = current.relative_time - previous.relative_time
                fraction = (
                    (relative_time - previous.relative_time) / interval
                    if interval > 1e-12 else 0.0
                )
                return (
                    self._blend(
                        previous.position,
                        current.position,
                        fraction,
                    ),
                    self._blend(
                        previous.velocity,
                        current.velocity,
                        fraction,
                    ),
                    self._blend(
                        previous.acceleration,
                        current.acceleration,
                        fraction,
                    ),
                )
        raise ValueError('prediction time exceeds available horizon')

    def state_at_absolute_time(self, absolute_time):
        """Return the state at an absolute ROS time in this snapshot."""
        relative_time = float(absolute_time) - self.source_stamp
        return self.state_at(relative_time)


@dataclass(frozen=True)
class UavKinematicState:
    """One UAV state in the ROS clock domain."""

    stamp: float
    position: tuple
    velocity: tuple
    acceleration: tuple


@dataclass(frozen=True)
class PlannerRequest:
    """Synchronized latest snapshots for one planning event."""

    mission_id: int
    prediction: PredictionSeries
    uav: UavKinematicState

    @property
    def source_stamp(self):
        """Return the oldest input stamp so newer data cannot hide staleness."""
        return min(self.prediction.source_stamp, self.uav.stamp)


class LatestRequestSlot:
    """A thread-safe single pending request with replacement accounting."""

    def __init__(self):
        self._lock = threading.Lock()
        self._pending = None
        self.replaced_request_count = 0

    def submit(self, request):
        """Atomically replace any pending request."""
        with self._lock:
            if self._pending is not None:
                self.replaced_request_count += 1
            self._pending = request

    def take(self):
        """Atomically take the latest request, leaving the slot empty."""
        with self._lock:
            request = self._pending
            self._pending = None
            return request


def validate_input(request, now, maximum_age):
    """Classify stale planner inputs before any MINCO computation."""
    now = float(now)
    maximum_age = float(maximum_age)
    prediction_age = now - request.prediction.source_stamp
    state_age = now - request.uav.stamp
    if prediction_age < 0.0 or prediction_age > maximum_age:
        return FastPlanningFailure.PREDICTION_STALE
    if state_age < 0.0 or state_age > maximum_age:
        return FastPlanningFailure.STATE_STALE
    if request.prediction.mission_id != request.mission_id:
        return FastPlanningFailure.PREDICTION_STALE
    return FastPlanningFailure.NONE


def validate_plan_arrival(request, generated_stamp, maximum_age):
    """Reject a result whose oldest source was already stale on arrival."""
    oldest_age = float(generated_stamp) - request.source_stamp
    if oldest_age > float(maximum_age) or oldest_age < 0.0:
        return FastPlanningFailure.PLAN_STALE_ON_ARRIVAL
    return FastPlanningFailure.NONE


def validate_target_shift(
    request,
    latest_prediction,
    intercept_time,
    tolerance,
):
    """Reject a plan when a newer prediction moved its contact endpoint."""
    if latest_prediction is None:
        return FastPlanningFailure.PREDICTION_STALE
    if latest_prediction.mission_id != request.mission_id:
        return FastPlanningFailure.PLAN_STALE_ON_ARRIVAL
    contact_stamp = request.prediction.source_stamp + float(intercept_time)
    try:
        planned_position = request.prediction.state_at_absolute_time(
            contact_stamp
        )[0]
        latest_position = latest_prediction.state_at_absolute_time(
            contact_stamp
        )[0]
    except (TypeError, ValueError):
        return FastPlanningFailure.PLAN_STALE_ON_ARRIVAL
    shift = math.sqrt(sum(
        (latest - planned) ** 2
        for latest, planned in zip(latest_position, planned_position)
    ))
    if shift > float(tolerance):
        return FastPlanningFailure.PLAN_STALE_ON_ARRIVAL
    return FastPlanningFailure.NONE


def validate_total_deadline(elapsed, hard_deadline):
    """Apply the process-level budget including non-solver overhead."""
    if float(elapsed) >= float(hard_deadline):
        return FastPlanningFailure.DEADLINE_EXCEEDED
    return FastPlanningFailure.NONE
