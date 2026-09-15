"""
Three-dimensional reachability and final sea-safety primitives.

NED coordinates are used throughout: positive vertical velocity points down
towards the sea and a smaller ``z`` value is higher.
"""

import math
from dataclasses import dataclass
from enum import Enum


class SeaSafetyState(str, Enum):
    """Runtime state of the final sea-surface control barrier."""

    SAFE = 'SAFE'
    WARNING = 'WARNING'
    BRAKE = 'BRAKE'
    UNRECOVERABLE = 'UNRECOVERABLE'


@dataclass(frozen=True)
class SeaSafetyResult:
    """One evaluation of the final vertical-command safety barrier."""

    state: SeaSafetyState
    command_vz: float
    clearance: float
    response_margin: float
    immediate_margin: float
    stopping_distance: float
    unrecoverable: bool


@dataclass(frozen=True)
class ReachabilityEstimate:
    """Lower bounds used to choose a finite-horizon search interval."""

    horizontal_min_time: float
    vertical_min_time: float
    sea_safe_min_time: float

    @property
    def required_time(self):
        return max(
            self.horizontal_min_time,
            self.vertical_min_time,
            self.sea_safe_min_time,
        )


def _finite(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f'{name} must be finite')
    return value


def _velocity_envelope_integral(
    duration,
    initial_velocity,
    final_velocity,
    maximum_speed,
    positive_acceleration,
    negative_acceleration,
    maximize,
):
    """
    Integrate the extremal bounded-velocity envelope.

    The upper envelope is ``min(v0+a*t, vf+b*(T-t), vmax)``.  The
    corresponding lower envelope is its sign-reversed counterpart.  A fixed
    96-interval trapezoid is deterministic and sufficiently conservative for
    a horizon gate; every accepted polynomial is still densely checked later.
    """
    duration = max(float(duration), 0.0)
    if duration <= 0.0:
        return 0.0
    intervals = 96
    step = duration / intervals

    def envelope(time):
        if maximize:
            return min(
                maximum_speed,
                initial_velocity + positive_acceleration * time,
                final_velocity
                + negative_acceleration * (duration - time),
            )
        return max(
            -maximum_speed,
            initial_velocity - negative_acceleration * time,
            final_velocity
            - positive_acceleration * (duration - time),
        )

    total = 0.5 * (envelope(0.0) + envelope(duration))
    total += sum(envelope(index * step) for index in range(1, intervals))
    return total * step


def minimum_time_1d(
    displacement,
    initial_velocity,
    final_velocity,
    maximum_speed,
    maximum_acceleration,
    maximum_braking_acceleration=None,
    response_delay=0.0,
    search_limit=30.0,
):
    """
    Return a bounded-velocity 1-D minimum time or ``math.inf``.

    A response delay is represented as unavoidable constant-velocity motion.
    The subsequent feasibility test uses both the maximum and minimum
    displacement envelopes, so an endpoint that is too close while moving
    quickly is not incorrectly declared reachable.
    """
    displacement = _finite(displacement, 'displacement')
    initial_velocity = _finite(initial_velocity, 'initial velocity')
    final_velocity = _finite(final_velocity, 'final velocity')
    maximum_speed = _finite(maximum_speed, 'maximum speed')
    maximum_acceleration = _finite(
        maximum_acceleration,
        'maximum acceleration',
    )
    if maximum_braking_acceleration is None:
        maximum_braking_acceleration = maximum_acceleration
    maximum_braking_acceleration = _finite(
        maximum_braking_acceleration,
        'maximum braking acceleration',
    )
    response_delay = max(_finite(response_delay, 'response delay'), 0.0)
    search_limit = max(_finite(search_limit, 'search limit'), 0.0)
    if (
        maximum_speed <= 0.0
        or maximum_acceleration <= 0.0
        or maximum_braking_acceleration <= 0.0
    ):
        raise ValueError('motion limits must be positive')
    if abs(initial_velocity) > maximum_speed + 1e-6:
        return math.inf
    if abs(final_velocity) > maximum_speed + 1e-6:
        return math.inf

    remaining = displacement - initial_velocity * response_delay

    def feasible(control_time):
        velocity_change = final_velocity - initial_velocity
        if velocity_change >= 0.0:
            if velocity_change > maximum_acceleration * control_time + 1e-9:
                return False
        elif -velocity_change > (
            maximum_braking_acceleration * control_time + 1e-9
        ):
            return False
        lower = _velocity_envelope_integral(
            control_time,
            initial_velocity,
            final_velocity,
            maximum_speed,
            maximum_acceleration,
            maximum_braking_acceleration,
            False,
        )
        upper = _velocity_envelope_integral(
            control_time,
            initial_velocity,
            final_velocity,
            maximum_speed,
            maximum_acceleration,
            maximum_braking_acceleration,
            True,
        )
        return lower - 2e-3 <= remaining <= upper + 2e-3

    if feasible(0.0):
        return response_delay
    if search_limit <= response_delay:
        return math.inf
    control_limit = search_limit - response_delay
    scan_step = min(0.02, max(control_limit / 200.0, 1e-3))
    lower = 0.0
    upper = min(scan_step, control_limit)
    while upper <= control_limit + 1e-12 and not feasible(upper):
        lower = upper
        upper = min(upper + scan_step, control_limit)
        if upper <= lower + 1e-12:
            break
    if not feasible(upper):
        return math.inf
    for _ in range(45):
        midpoint = 0.5 * (lower + upper)
        if feasible(midpoint):
            upper = midpoint
        else:
            lower = midpoint
    return response_delay + upper


def estimate_reachability(
    initial_position,
    initial_velocity,
    target_position,
    target_velocity,
    maximum_horizontal_speed,
    maximum_vertical_speed,
    maximum_horizontal_acceleration,
    maximum_vertical_acceleration,
    vertical_braking_acceleration,
    response_delay,
    search_limit,
):
    """Estimate separate horizontal, vertical and sea-safe time bounds."""
    dx = float(target_position[0]) - float(initial_position[0])
    dy = float(target_position[1]) - float(initial_position[1])
    horizontal_distance = math.hypot(dx, dy)
    if horizontal_distance > 1e-9:
        line_x = dx / horizontal_distance
        line_y = dy / horizontal_distance
    else:
        line_x, line_y = 1.0, 0.0
    initial_along = (
        float(initial_velocity[0]) * line_x
        + float(initial_velocity[1]) * line_y
    )
    target_along = (
        float(target_velocity[0]) * line_x
        + float(target_velocity[1]) * line_y
    )
    horizontal_time = minimum_time_1d(
        horizontal_distance,
        initial_along,
        target_along,
        maximum_horizontal_speed,
        maximum_horizontal_acceleration,
        maximum_horizontal_acceleration,
        response_delay,
        search_limit,
    )
    vertical_time = minimum_time_1d(
        float(target_position[2]) - float(initial_position[2]),
        float(initial_velocity[2]),
        float(target_velocity[2]),
        maximum_vertical_speed,
        maximum_vertical_acceleration,
        vertical_braking_acceleration,
        response_delay,
        search_limit,
    )
    vertical_descent_speed = max(float(initial_velocity[2]), 0.0)
    sea_safe_time = (
        response_delay
        + vertical_descent_speed / max(vertical_braking_acceleration, 1e-6)
    )
    return ReachabilityEstimate(
        horizontal_time,
        vertical_time,
        sea_safe_time,
    )


def apply_sea_safety_guard(
    current_z,
    current_vz,
    proposed_vz,
    sea_surface_z,
    reserve_clearance,
    response_delay,
    effective_braking_acceleration,
    control_dt,
    warning_margin=0.15,
    maximum_vertical_speed=math.inf,
):
    """Apply the last vertical-command guard after all other limiters."""
    current_z = _finite(current_z, 'current z')
    current_vz = _finite(current_vz, 'current vz')
    proposed_vz = _finite(proposed_vz, 'proposed vz')
    sea_surface_z = _finite(sea_surface_z, 'sea surface z')
    reserve_clearance = max(
        _finite(reserve_clearance, 'reserve clearance'),
        0.0,
    )
    response_delay = max(_finite(response_delay, 'response delay'), 0.0)
    braking = _finite(
        effective_braking_acceleration,
        'effective braking acceleration',
    )
    control_dt = max(_finite(control_dt, 'control dt'), 1e-3)
    warning_margin = max(_finite(warning_margin, 'warning margin'), 0.0)
    if braking <= 0.0:
        raise ValueError('effective braking acceleration must be positive')

    clearance = sea_surface_z - current_z
    descent_speed = max(current_vz, 0.0)
    braking_distance = descent_speed * descent_speed / (2.0 * braking)
    immediate_margin = clearance - braking_distance - reserve_clearance
    response_stopping_distance = (
        descent_speed * response_delay + braking_distance
    )
    response_margin = (
        clearance - response_stopping_distance - reserve_clearance
    )
    unrecoverable = immediate_margin < 0.0

    if unrecoverable:
        state = SeaSafetyState.UNRECOVERABLE
    elif response_margin <= 0.0:
        state = SeaSafetyState.BRAKE
    elif response_margin <= warning_margin:
        state = SeaSafetyState.WARNING
    else:
        state = SeaSafetyState.SAFE

    command_vz = proposed_vz
    if state == SeaSafetyState.WARNING:
        command_vz = min(command_vz, current_vz)
    elif state in (SeaSafetyState.BRAKE, SeaSafetyState.UNRECOVERABLE):
        command_vz = min(command_vz, current_vz - braking * control_dt)
    command_vz = max(
        min(command_vz, maximum_vertical_speed),
        -maximum_vertical_speed,
    )
    return SeaSafetyResult(
        state,
        command_vz,
        clearance,
        response_margin,
        immediate_margin,
        response_stopping_distance + reserve_clearance,
        unrecoverable,
    )
