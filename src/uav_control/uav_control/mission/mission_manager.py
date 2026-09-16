"""Pure recoverable mission state machine for modular interception."""

from enum import IntEnum
import math


class MissionPhase(IntEnum):
    """Stable numeric phases shared with MissionState.msg."""

    INIT = 0
    GROUND_HOLD = 1
    TAKEOFF = 2
    FOLLOW = 3
    FAR_GUIDANCE = 4
    MINCO_READY = 5
    MINCO_TRACKING = 6
    TERMINAL_MINCO = 7
    PLAN_RECOVERY = 8
    SAFE_WAIT = 9
    CAPTURE = 10
    FAILURE = 11
    ABORTED = 12


class MissionManagerCore:
    """Own commands and state transitions, but no control algorithms."""

    def __init__(
        self,
        maximum_tracker_age=0.125,
        minimum_plan_remaining_time=0.20,
        plan_recovery_timeout=0.50,
        terminal_time_threshold=1.0,
        terminal_distance_threshold=2.0,
    ):
        self.maximum_tracker_age = float(maximum_tracker_age)
        self.minimum_plan_remaining_time = float(
            minimum_plan_remaining_time
        )
        self.plan_recovery_timeout = float(plan_recovery_timeout)
        self.terminal_time_threshold = float(terminal_time_threshold)
        self.terminal_distance_threshold = float(terminal_distance_threshold)
        self.mission_id = 0
        self.phase = MissionPhase.INIT
        self.flight_ready = False
        self.intercept_requested = False
        self.completed = False
        self.active_plan_id = 0
        self.last_tracker_accept_time = None
        self.recovery_started_at = None
        self.last_transition_time = 0.0

    def _transition(self, phase, now):
        changed = self.phase != MissionPhase(phase)
        self.phase = MissionPhase(phase)
        if changed:
            self.last_transition_time = float(now)
        return changed

    def set_flight_ready(self, ready, now=0.0):
        """Reflect external PX4 preparation without owning PX4 control."""
        self.flight_ready = bool(ready)
        if self.flight_ready and self.phase == MissionPhase.INIT:
            self._transition(MissionPhase.GROUND_HOLD, now)
            return True
        if not self.flight_ready and self.phase == MissionPhase.GROUND_HOLD:
            self._transition(MissionPhase.INIT, now)
            return True
        return False

    def handle_command(self, command, now):
        """Apply one X/Y/R/Q command and report whether it was accepted."""
        command = str(command).strip().upper()
        now = float(now)
        if command == 'R':
            self.mission_id += 1
            self.intercept_requested = False
            self.completed = False
            self.active_plan_id = 0
            self.last_tracker_accept_time = None
            self.recovery_started_at = None
            destination = (
                MissionPhase.GROUND_HOLD
                if self.flight_ready
                else MissionPhase.INIT
            )
            self._transition(destination, now)
            return True
        if command == 'Q':
            self.completed = True
            self._transition(MissionPhase.ABORTED, now)
            return True
        if command == 'X':
            if not self.flight_ready or self.phase != MissionPhase.GROUND_HOLD:
                return False
            if self.mission_id == 0:
                self.mission_id = 1
            self._transition(MissionPhase.TAKEOFF, now)
            return True
        if command == 'Y':
            if self.phase != MissionPhase.FOLLOW:
                return False
            self.intercept_requested = True
            self._transition(MissionPhase.FAR_GUIDANCE, now)
            return True
        return False

    def mark_takeoff_complete(self, now):
        """Enter follow only from the explicit takeoff phase."""
        if self.phase != MissionPhase.TAKEOFF:
            return False
        self._transition(MissionPhase.FOLLOW, now)
        return True

    def observe_planner(self, success, plan_id, now):
        """Record no state: solver success is not tracker acceptance."""
        del success, plan_id, now
        return False

    def observe_tracker(
        self,
        mission_id,
        plan_id,
        status,
        prediction_age,
        remaining_time,
        now,
        target_distance=math.inf,
    ):
        """Accept only a fresh, currently executable tracker decision."""
        now = float(now)
        current_mission = int(mission_id) == self.mission_id
        fresh = (
            0.0 <= float(prediction_age) <= self.maximum_tracker_age
            and float(remaining_time) >= self.minimum_plan_remaining_time
        )
        tracking = (
            str(status) in ('PLAN_ACCEPTED', 'TRACKING')
            and int(plan_id) > 0
        )
        if (
            current_mission
            and fresh
            and tracking
            and self.intercept_requested
        ):
            new_plan = int(plan_id) != self.active_plan_id
            self.active_plan_id = int(plan_id)
            self.last_tracker_accept_time = now
            self.recovery_started_at = None
            terminal = (
                float(remaining_time) <= self.terminal_time_threshold
                or float(target_distance) <= self.terminal_distance_threshold
            )
            if terminal or self.phase == MissionPhase.TERMINAL_MINCO:
                self._transition(MissionPhase.TERMINAL_MINCO, now)
            elif new_plan or self.phase in (
                MissionPhase.FAR_GUIDANCE,
                MissionPhase.PLAN_RECOVERY,
                MissionPhase.SAFE_WAIT,
            ):
                self._transition(MissionPhase.MINCO_READY, now)
            return True

        if (
            current_mission
            and str(status) == 'NO_VALID_PLAN'
            and self.phase in (
                MissionPhase.MINCO_READY,
                MissionPhase.MINCO_TRACKING,
                MissionPhase.TERMINAL_MINCO,
            )
        ):
            self.recovery_started_at = now
            self._transition(MissionPhase.PLAN_RECOVERY, now)
        return False

    def tick(self, now, far_guidance_available=False):
        """Advance transient and timeout-driven recoverable phases."""
        now = float(now)
        if self.phase == MissionPhase.MINCO_READY:
            self._transition(MissionPhase.MINCO_TRACKING, now)
        elif (
            self.phase == MissionPhase.PLAN_RECOVERY
            and self.recovery_started_at is not None
            and now - self.recovery_started_at >= self.plan_recovery_timeout
        ):
            self._transition(MissionPhase.SAFE_WAIT, now)
        elif (
            self.phase == MissionPhase.SAFE_WAIT
            and bool(far_guidance_available)
        ):
            self._transition(MissionPhase.FAR_GUIDANCE, now)
        return self.phase

    def mark_capture(self, now):
        """Finish the mission with a truth-evaluated capture event."""
        self.completed = True
        self._transition(MissionPhase.CAPTURE, now)

    def mark_failure(self, now):
        """Finish the mission with an evaluator or PX4 failure event."""
        self.completed = True
        self._transition(MissionPhase.FAILURE, now)
