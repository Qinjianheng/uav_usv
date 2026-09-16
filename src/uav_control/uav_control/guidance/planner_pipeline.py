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

    @property
    def end_stamp(self):
        """Return the absolute end time covered by prediction samples."""
        if not self.samples:
            return float(self.source_stamp)
        return float(self.source_stamp) + max(
            float(sample.relative_time)
            for sample in self.samples
        )

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
    trajectory_start_stamp: float
    contact_stamp: float = None
    terminal_mode: bool = False
    minimum_duration: float = None

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


@dataclass(frozen=True)
class PlanningDecision:
    """One cadence decision for normal or committed terminal planning."""

    submit: bool
    terminal_mode: bool
    minimum_duration: float


def terminal_mode_for_plan(
    mission_state,
    request_terminal_mode=False,
    remaining_t_go=None,
    terminal_time_threshold=1.0,
    terminal_state=7,
):
    """Return the shared terminal-state decision used by planner outputs."""
    return bool(
        request_terminal_mode
        or int(mission_state) == int(terminal_state)
        or (
            remaining_t_go is not None
            and float(remaining_t_go)
            <= float(terminal_time_threshold) + 1e-9
        )
    )


class PlanningRequestPolicy:
    """Select normal/terminal planning without changing motion limits."""

    def __init__(
        self,
        normal_minimum_duration=1.0,
        terminal_minimum_duration=0.30,
        terminal_freeze_time=0.30,
        terminal_time_threshold=1.0,
        terminal_state=7,
    ):
        self.normal_minimum_duration = float(normal_minimum_duration)
        self.terminal_minimum_duration = float(terminal_minimum_duration)
        self.terminal_freeze_time = float(terminal_freeze_time)
        self.terminal_time_threshold = float(terminal_time_threshold)
        self.terminal_state = int(terminal_state)

    def decide(self, mission_state, remaining_t_go=None):
        terminal = terminal_mode_for_plan(
            mission_state=mission_state,
            remaining_t_go=remaining_t_go,
            terminal_time_threshold=self.terminal_time_threshold,
            terminal_state=self.terminal_state,
        )
        if terminal:
            submit = (
                remaining_t_go is not None
                and float(remaining_t_go)
                > self.terminal_freeze_time + 1e-9
            )
            return PlanningDecision(
                submit=submit,
                terminal_mode=True,
                minimum_duration=self.terminal_minimum_duration,
            )
        return PlanningDecision(
            submit=True,
            terminal_mode=False,
            minimum_duration=self.normal_minimum_duration,
        )


class ContactTimeSchedule:
    """Keep one absolute contact time across rolling prediction snapshots."""

    def __init__(
        self,
        terminal_threshold=1.0,
        freeze_time=0.30,
        terminal_max_reschedule_delay=0.30,
    ):
        self.terminal_threshold = max(float(terminal_threshold), 0.0)
        self.freeze_time = max(float(freeze_time), 0.0)
        self.terminal_max_reschedule_delay = float(
            terminal_max_reschedule_delay
        )
        self.contact_stamp = None

    def reset(self):
        self.contact_stamp = None

    def accept_plan(self, source_stamp, selected_t_go, rescheduled=False):
        candidate = float(source_stamp) + float(selected_t_go)
        if self.contact_stamp is None:
            self.contact_stamp = candidate
        elif bool(rescheduled) and (
            0.0 < candidate - self.contact_stamp
            <= self.terminal_max_reschedule_delay
        ):
            self.contact_stamp = candidate
        return self.contact_stamp

    def remaining_t_go(self, source_stamp):
        if self.contact_stamp is None:
            return None
        return self.contact_stamp - float(source_stamp)

    def preferred_t_go(self, source_stamp):
        remaining = self.remaining_t_go(source_stamp)
        if remaining is None or remaining <= 0.0:
            return None
        return remaining

    def is_terminal(self, stamp):
        remaining = self.remaining_t_go(stamp)
        return (
            remaining is not None
            and remaining <= self.terminal_threshold + 1e-9
        )

    def should_replan(self, source_stamp):
        """Allow terminal refreshes only before the final freeze window."""
        remaining = self.remaining_t_go(source_stamp)
        return remaining is None or remaining > self.freeze_time + 1e-9


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
    contact_stamp=None,
):
    """Reject a plan when a newer prediction moved its contact endpoint."""
    if latest_prediction is None:
        return FastPlanningFailure.PREDICTION_STALE
    if latest_prediction.mission_id != request.mission_id:
        return FastPlanningFailure.PLAN_STALE_ON_ARRIVAL
    if contact_stamp is None:
        contact_stamp = (
            request.trajectory_start_stamp + float(intercept_time)
        )
    else:
        contact_stamp = float(contact_stamp)
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
