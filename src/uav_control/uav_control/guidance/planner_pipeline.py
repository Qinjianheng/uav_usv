"""Transport-neutral input handling for the independent MINCO planner."""

import math
import threading
from dataclasses import dataclass

from uav_control.common.sea_safety import apply_sea_safety_guard

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
    planning_cycle_id: int = 0

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


@dataclass(frozen=True)
class TerminalAdmissionDecision:
    """Explain whether one fully validated plan may start final approach."""

    admitted: bool
    reason: str


def evaluate_terminal_admission(
    horizontal_min_time,
    vertical_min_time,
    selected_t_go,
    planned_capture_margin,
    current_z,
    current_vz,
    planned_initial_vz,
    sea_surface_z,
    reserve_clearance,
    response_delay,
    braking_acceleration,
    maximum_vertical_speed,
    time_sync_tolerance,
):
    """Gate the first descent on synchronized reachability and sea margin."""
    values = (
        horizontal_min_time,
        vertical_min_time,
        selected_t_go,
        planned_capture_margin,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return TerminalAdmissionDecision(False, 'REACHABILITY_UNKNOWN')
    if planned_capture_margin < 0.0:
        return TerminalAdmissionDecision(False, 'CAPTURE_MARGIN_INSUFFICIENT')
    if (
        horizontal_min_time > selected_t_go + 1e-9
        or vertical_min_time > selected_t_go + 1e-9
    ):
        return TerminalAdmissionDecision(False, 'CONTACT_TIME_UNREACHABLE')
    if horizontal_min_time > vertical_min_time + time_sync_tolerance:
        return TerminalAdmissionDecision(False, 'HORIZONTAL_NOT_READY')
    safety = apply_sea_safety_guard(
        current_z=current_z,
        current_vz=current_vz,
        proposed_vz=planned_initial_vz,
        sea_surface_z=sea_surface_z,
        reserve_clearance=reserve_clearance,
        response_delay=response_delay,
        effective_braking_acceleration=braking_acceleration,
        control_dt=0.05,
        maximum_vertical_speed=maximum_vertical_speed,
    )
    if safety.unrecoverable or safety.response_margin <= 0.0:
        return TerminalAdmissionDecision(False, 'SEA_MARGIN_INSUFFICIENT')
    return TerminalAdmissionDecision(True, 'ADMITTED')


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
    """Keep committed and tracker-pending absolute contact times separate."""

    def __init__(
        self,
        terminal_threshold=1.0,
        freeze_time=0.30,
    ):
        self.terminal_threshold = max(float(terminal_threshold), 0.0)
        self.freeze_time = max(float(freeze_time), 0.0)
        self.contact_stamp = None
        self.committed_plan_id = 0
        self.pending_plan_id = 0
        self.pending_contact_stamp = None
        self.pending_planning_cycle_id = 0
        self.pending_since = None

    def reset(self):
        self.contact_stamp = None
        self.committed_plan_id = 0
        self._clear_pending()

    def _clear_pending(self):
        self.pending_plan_id = 0
        self.pending_contact_stamp = None
        self.pending_planning_cycle_id = 0
        self.pending_since = None

    def cancel_pending(self):
        """Discard the pending proposal while preserving committed contact."""
        had_pending = self.has_pending
        self._clear_pending()
        return had_pending

    @property
    def has_pending(self):
        return self.pending_plan_id > 0

    def propose(
        self,
        plan_id,
        contact_stamp,
        planning_cycle_id,
        proposed_at,
    ):
        """Replace the pending proposal without committing its contact time."""
        plan_id = int(plan_id)
        contact_stamp = float(contact_stamp)
        proposed_at = float(proposed_at)
        if (
            plan_id <= 0
            or not math.isfinite(contact_stamp)
            or not math.isfinite(proposed_at)
        ):
            return False
        self.pending_plan_id = plan_id
        self.pending_contact_stamp = contact_stamp
        self.pending_planning_cycle_id = int(planning_cycle_id)
        self.pending_since = proposed_at
        return True

    def confirm(self, plan_id, planning_cycle_id):
        """Commit only the exact proposal accepted by the tracker."""
        if (
            int(plan_id) != self.pending_plan_id
            or int(planning_cycle_id) != self.pending_planning_cycle_id
        ):
            return False
        self.contact_stamp = self.pending_contact_stamp
        self.committed_plan_id = self.pending_plan_id
        self._clear_pending()
        return True

    def reject(self, plan_id, planning_cycle_id):
        """Drop only the exact rejected proposal and keep the old contact."""
        if (
            int(plan_id) != self.pending_plan_id
            or int(planning_cycle_id) != self.pending_planning_cycle_id
        ):
            return False
        self._clear_pending()
        return True

    def expire(self, now, timeout):
        """Expire an unacknowledged proposal without changing committed state."""
        if not self.has_pending or self.pending_since is None:
            return False
        if float(now) - self.pending_since <= float(timeout):
            return False
        self._clear_pending()
        return True

    def accept_plan(self, source_stamp, selected_t_go, rescheduled=False):
        """Directly seed a committed contact for compatibility and startup."""
        candidate = float(source_stamp) + float(selected_t_go)
        if self.contact_stamp is None:
            self.contact_stamp = candidate
        elif (
            bool(rescheduled)
            and candidate > self.contact_stamp + 1e-9
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


def target_shift_distance(
    request,
    latest_prediction,
    intercept_time,
    contact_stamp=None,
):
    """Measure endpoint movement at one shared absolute contact time."""
    if latest_prediction is None:
        raise ValueError('latest prediction is unavailable')
    if latest_prediction.mission_id != request.mission_id:
        raise ValueError('latest prediction mission does not match request')
    if contact_stamp is None:
        contact_stamp = (
            request.trajectory_start_stamp + float(intercept_time)
        )
    planned_position = request.prediction.state_at_absolute_time(
        float(contact_stamp)
    )[0]
    latest_position = latest_prediction.state_at_absolute_time(
        float(contact_stamp)
    )[0]
    return math.sqrt(sum(
        (latest - planned) ** 2
        for latest, planned in zip(latest_position, planned_position)
    ))


def validate_total_deadline(elapsed, hard_deadline):
    """Apply the process-level budget including non-solver overhead."""
    if float(elapsed) >= float(hard_deadline):
        return FastPlanningFailure.DEADLINE_EXCEEDED
    return FastPlanningFailure.NONE
