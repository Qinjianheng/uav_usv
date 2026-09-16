from uav_control.mission.mission_manager import MissionManagerCore
from uav_control.mission.mission_manager import MissionPhase


def start_intercept(core):
    assert core.set_flight_ready(True)
    assert core.phase == MissionPhase.GROUND_HOLD
    assert core.handle_command('X', now=1.0)
    assert core.phase == MissionPhase.TAKEOFF
    assert core.mark_takeoff_complete(now=3.0)
    assert core.phase == MissionPhase.FOLLOW
    assert core.handle_command('Y', now=4.0)
    assert core.phase == MissionPhase.FAR_GUIDANCE


def test_x_then_y_produces_explicit_takeoff_follow_intercept_sequence():
    core = MissionManagerCore()

    start_intercept(core)

    assert core.mission_id == 1
    assert core.intercept_requested


def test_planner_success_alone_never_latches_minco_ready():
    core = MissionManagerCore()
    start_intercept(core)

    core.observe_planner(success=True, plan_id=12, now=4.1)

    assert core.phase == MissionPhase.FAR_GUIDANCE


def test_only_fresh_current_tracker_acceptance_enters_minco_ready():
    core = MissionManagerCore(maximum_tracker_age=0.125)
    start_intercept(core)

    wrong_mission = core.observe_tracker(
        mission_id=99,
        plan_id=12,
        status='TRACKING',
        prediction_age=0.02,
        remaining_time=1.0,
        now=4.1,
    )
    stale = core.observe_tracker(
        mission_id=core.mission_id,
        plan_id=12,
        status='TRACKING',
        prediction_age=0.20,
        remaining_time=1.0,
        now=4.2,
    )
    accepted = core.observe_tracker(
        mission_id=core.mission_id,
        plan_id=13,
        status='PLAN_ACCEPTED',
        prediction_age=0.02,
        remaining_time=1.5,
        now=4.3,
    )

    assert not wrong_mission
    assert not stale
    assert accepted
    assert core.phase == MissionPhase.MINCO_READY
    core.tick(now=4.35)
    assert core.phase == MissionPhase.MINCO_TRACKING


def test_terminal_minco_latches_when_contact_is_near():
    """Catch new 5 Hz plans pushing a terminal mission back to MINCO_READY."""
    core = MissionManagerCore(
        terminal_time_threshold=1.0,
        terminal_distance_threshold=2.0,
    )
    start_intercept(core)
    core.observe_tracker(
        mission_id=core.mission_id,
        plan_id=1,
        status='TRACKING',
        prediction_age=0.02,
        remaining_time=2.0,
        target_distance=3.0,
        now=4.1,
    )
    core.tick(4.15)
    assert core.phase == MissionPhase.MINCO_TRACKING

    accepted = core.observe_tracker(
        mission_id=core.mission_id,
        plan_id=2,
        status='PLAN_ACCEPTED',
        prediction_age=0.02,
        remaining_time=0.8,
        target_distance=1.8,
        now=5.0,
    )
    core.tick(5.05)

    assert accepted
    assert core.phase == MissionPhase.TERMINAL_MINCO

    core.observe_tracker(
        mission_id=core.mission_id,
        plan_id=3,
        status='PLAN_ACCEPTED',
        prediction_age=0.02,
        remaining_time=1.4,
        target_distance=2.5,
        now=5.1,
    )

    assert core.phase == MissionPhase.TERMINAL_MINCO


def test_safe_wait_recovers_when_a_new_plan_is_tracker_accepted():
    core = MissionManagerCore(
        plan_recovery_timeout=0.25,
        maximum_tracker_age=0.125,
    )
    start_intercept(core)
    core.observe_tracker(
        core.mission_id,
        1,
        'TRACKING',
        prediction_age=0.01,
        remaining_time=1.0,
        now=4.1,
    )
    core.tick(now=4.15)

    core.observe_tracker(
        core.mission_id,
        1,
        'NO_VALID_PLAN',
        prediction_age=0.3,
        remaining_time=0.0,
        now=4.4,
    )
    assert core.phase == MissionPhase.PLAN_RECOVERY
    core.tick(now=4.7)
    assert core.phase == MissionPhase.SAFE_WAIT

    recovered = core.observe_tracker(
        core.mission_id,
        2,
        'TRACKING',
        prediction_age=0.01,
        remaining_time=0.8,
        now=4.8,
    )
    assert recovered
    assert core.phase == MissionPhase.TERMINAL_MINCO


def test_safe_wait_can_reestablish_far_guidance_when_geometry_allows():
    core = MissionManagerCore(plan_recovery_timeout=0.1)
    start_intercept(core)
    core.observe_tracker(
        core.mission_id,
        1,
        'TRACKING',
        prediction_age=0.01,
        remaining_time=1.0,
        now=4.1,
    )
    core.tick(4.11)
    core.observe_tracker(
        core.mission_id,
        1,
        'NO_VALID_PLAN',
        prediction_age=0.3,
        remaining_time=0.0,
        now=4.2,
    )
    core.tick(4.31)
    assert core.phase == MissionPhase.SAFE_WAIT

    core.tick(now=4.4, far_guidance_available=True)

    assert core.phase == MissionPhase.FAR_GUIDANCE


def test_r_starts_a_clean_mission_and_q_aborts():
    core = MissionManagerCore()
    start_intercept(core)
    original_mission_id = core.mission_id

    assert core.handle_command('R', now=5.0)
    assert core.mission_id == original_mission_id + 1
    assert core.phase == MissionPhase.GROUND_HOLD
    assert not core.intercept_requested
    assert core.active_plan_id == 0

    assert core.handle_command('Q', now=5.1)
    assert core.phase == MissionPhase.ABORTED
    assert core.completed
