"""Deadline-aware MINCO planning with at most six realtime candidates."""

import math
import time
from dataclasses import dataclass, replace
from enum import Enum

from .finite_horizon_intercept_planner import (
    FiniteHorizonInterceptPlanner,
    PlanningFailureReason,
)


class FastPlanningFailure(str, Enum):
    """Auditable failure classes for the realtime planner."""

    NONE = 'NONE'
    STATE_STALE = 'STATE_STALE'
    PREDICTION_STALE = 'PREDICTION_STALE'
    HORIZON_INSUFFICIENT = 'HORIZON_INSUFFICIENT'
    CAPTURE_GEOMETRY = 'CAPTURE_GEOMETRY'
    DYNAMIC_LIMIT_HORIZONTAL = 'DYNAMIC_LIMIT_HORIZONTAL'
    DYNAMIC_LIMIT_VERTICAL = 'DYNAMIC_LIMIT_VERTICAL'
    SEA_CLEARANCE = 'SEA_CLEARANCE'
    MINCO_CONSTRUCTION_FAIL = 'MINCO_CONSTRUCTION_FAIL'
    OPTIMIZATION_FAIL = 'OPTIMIZATION_FAIL'
    DEADLINE_EXCEEDED = 'DEADLINE_EXCEEDED'
    PLAN_STALE_ON_ARRIVAL = 'PLAN_STALE_ON_ARRIVAL'


@dataclass(frozen=True)
class FastPlanningOutcome:
    """One fast planning result and its event-level diagnostics."""

    plan: object
    failure: FastPlanningFailure
    diagnostics: object


class FastMincoPlanner(FiniteHorizonInterceptPlanner):
    """Use reachability-driven horizons and no realtime L-BFGS search."""

    def __init__(
        self,
        minimum_duration,
        maximum_duration,
        duration_margin,
        sample_step,
        maximum_horizontal_speed,
        maximum_vertical_speed,
        maximum_horizontal_acceleration,
        maximum_vertical_acceleration,
        preferred_closing_speed,
        conservative_closing_speed,
        capture_radius,
        sea_surface_z,
        contact_clearance,
        preferred_clearance,
        piece_count=3,
        target_curve_weight=0.7,
        deadline_seconds=0.08,
        response_delay=0.15,
        effective_vertical_braking_acceleration=2.5,
        quadrature_intervals_per_piece=6,
        clock=time.perf_counter,
    ):
        self.duration_margin = self._finite_positive(
            duration_margin,
            'duration margin',
        )
        self.deadline_seconds = self._finite_positive(
            deadline_seconds,
            'planning deadline',
        )
        self.preferred_realtime_closing_speed = self._finite_positive(
            preferred_closing_speed,
            'preferred closing speed',
        )
        self.conservative_realtime_closing_speed = self._finite_positive(
            conservative_closing_speed,
            'conservative closing speed',
        )
        self.realtime_curve_weight = float(target_curve_weight)
        self.clock = clock
        self.last_success_duration = None
        self._preferred_duration = None
        self._active_deadline = None
        self._deadline_exceeded = False
        self._candidate_failures = []
        super().__init__(
            minimum_duration=minimum_duration,
            maximum_duration=maximum_duration,
            absolute_maximum_duration=maximum_duration,
            horizon_extra_margin=2.0 * self.duration_margin,
            duration_step=self.duration_margin,
            sample_step=sample_step,
            maximum_horizontal_speed=maximum_horizontal_speed,
            maximum_vertical_speed=maximum_vertical_speed,
            maximum_horizontal_acceleration=maximum_horizontal_acceleration,
            maximum_vertical_acceleration=maximum_vertical_acceleration,
            desired_closing_speed=preferred_closing_speed,
            minimum_closing_speed=conservative_closing_speed,
            closing_speed_step=max(
                preferred_closing_speed - conservative_closing_speed,
                1e-3,
            ),
            capture_radius=capture_radius,
            sea_surface_z=sea_surface_z,
            contact_clearance=contact_clearance,
            preferred_clearance=preferred_clearance,
            minco_piece_count=piece_count,
            minco_target_curve_weight=target_curve_weight,
            enable_minco_geometric_optimization=False,
            minco_optimization_max_iterations=0,
            minco_quadrature_intervals_per_piece=(
                quadrature_intervals_per_piece
            ),
            response_delay=response_delay,
            effective_vertical_braking_acceleration=(
                effective_vertical_braking_acceleration
            ),
        )

    @staticmethod
    def _finite_positive(value, name):
        value = float(value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    @staticmethod
    def _unique(values):
        unique = []
        for value in values:
            if not any(abs(value - current) <= 1e-9 for current in unique):
                unique.append(value)
        return unique

    def _duration_candidates(self, minimum=None, maximum=None):
        minimum = self.minimum_duration if minimum is None else float(minimum)
        maximum = self.maximum_duration if maximum is None else float(maximum)
        middle = min(minimum + self.duration_margin, maximum)
        candidates = [minimum, middle, maximum]
        preferred = self._preferred_duration
        if preferred is not None and minimum <= preferred <= maximum:
            candidates = [preferred, minimum, maximum]
        elif (
            self.last_success_duration is not None
            and minimum <= self.last_success_duration <= maximum
        ):
            candidates = [
                self.last_success_duration,
                minimum,
                maximum,
            ]
        return self._unique(candidates)[:3]

    def _closing_speed_candidates(self):
        return tuple(self._unique((
            self.preferred_realtime_closing_speed,
            self.conservative_realtime_closing_speed,
        )))

    def _curve_weight_candidates(self):
        return (self.realtime_curve_weight,)

    def _deadline_reached(self):
        return (
            self._active_deadline is not None
            and self.clock() >= self._active_deadline
        )

    def _classify_infeasible(self, candidate):
        sample_count = max(
            int(math.ceil(candidate.duration / self.sample_step)),
            1,
        )
        sea_violation = any(
            candidate.sample(
                candidate.duration * index / sample_count
            ).position[2] > candidate.sea_clearance_ceiling_z + 1e-6
            for index in range(sample_count + 1)
        )
        if sea_violation:
            return FastPlanningFailure.SEA_CLEARANCE
        horizontal_ratio = max(
            candidate.maximum_horizontal_speed
            / self.maximum_horizontal_speed,
            candidate.maximum_horizontal_acceleration
            / self.maximum_horizontal_acceleration,
        )
        vertical_ratio = max(
            candidate.maximum_vertical_speed / self.maximum_vertical_speed,
            candidate.maximum_vertical_acceleration
            / self.maximum_vertical_acceleration,
        )
        if vertical_ratio > horizontal_ratio:
            return FastPlanningFailure.DYNAMIC_LIMIT_VERTICAL
        return FastPlanningFailure.DYNAMIC_LIMIT_HORIZONTAL

    def _candidate(self, *args, **kwargs):
        if self._deadline_reached():
            self._deadline_exceeded = True
            return None
        target_position = kwargs.get('target_position')
        if target_position is None and len(args) >= 4:
            target_position = args[3]
        if self._capture_contact_position(target_position) is None:
            self._candidate_failures.append(
                FastPlanningFailure.CAPTURE_GEOMETRY
            )
            return None
        candidate = super()._candidate(*args, **kwargs)
        if self._deadline_reached():
            self._deadline_exceeded = True
            return None
        if candidate is None:
            self._candidate_failures.append(
                FastPlanningFailure.MINCO_CONSTRUCTION_FAIL
            )
        elif not candidate.dynamically_feasible:
            self._candidate_failures.append(
                self._classify_infeasible(candidate)
            )
        return candidate

    def _mapped_failure(self):
        if self._deadline_exceeded:
            return FastPlanningFailure.DEADLINE_EXCEEDED
        if self.last_diagnostics.failure_reason == (
            PlanningFailureReason.HORIZON_INSUFFICIENT
        ):
            return FastPlanningFailure.HORIZON_INSUFFICIENT
        if (
            self.last_diagnostics.failure_reason
            == PlanningFailureReason.SEA_CLEARANCE
            and self.last_diagnostics.candidates_checked == 0
        ):
            return FastPlanningFailure.CAPTURE_GEOMETRY
        priority = (
            FastPlanningFailure.CAPTURE_GEOMETRY,
            FastPlanningFailure.SEA_CLEARANCE,
            FastPlanningFailure.DYNAMIC_LIMIT_VERTICAL,
            FastPlanningFailure.DYNAMIC_LIMIT_HORIZONTAL,
            FastPlanningFailure.MINCO_CONSTRUCTION_FAIL,
        )
        for failure in priority:
            if failure in self._candidate_failures:
                return failure
        return FastPlanningFailure.MINCO_CONSTRUCTION_FAIL

    def plan(self, *args, **kwargs):
        """Return the first feasible candidate within the hard deadline."""
        preferred_duration = kwargs.pop('preferred_duration', None)
        minimum_duration_override = kwargs.pop(
            'minimum_duration_override',
            None,
        )
        normal_minimum_duration = self.minimum_duration
        if minimum_duration_override is not None:
            minimum_duration_override = self._finite_positive(
                minimum_duration_override,
                'minimum duration override',
            )
            if minimum_duration_override > self.maximum_duration:
                raise ValueError(
                    'minimum duration override exceeds maximum duration'
                )
            self.minimum_duration = minimum_duration_override
        self._preferred_duration = (
            None if preferred_duration is None else float(preferred_duration)
        )
        start = self.clock()
        self._active_deadline = start + self.deadline_seconds
        self._deadline_exceeded = False
        self._candidate_failures = []
        try:
            plan = super().plan(*args, **kwargs)
            if self.clock() >= self._active_deadline:
                self._deadline_exceeded = True
                plan = None
        finally:
            self._active_deadline = None
            self._preferred_duration = None
            self.minimum_duration = normal_minimum_duration

        if plan is None:
            return FastPlanningOutcome(
                plan=None,
                failure=self._mapped_failure(),
                diagnostics=self.last_diagnostics,
            )
        self.last_success_duration = plan.duration
        plan = replace(plan, planner_type='MINCO_T3_FAST')
        return FastPlanningOutcome(
            plan=plan,
            failure=FastPlanningFailure.NONE,
            diagnostics=self.last_diagnostics,
        )
