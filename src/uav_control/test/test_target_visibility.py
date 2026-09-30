"""Acquisition-time visual lock, bounded search and terminal safety contracts."""

import math

import pytest

from uav_control.mission.target_visibility import TargetVisibilityState
from uav_control.mission.target_visibility import VisibilityConfig


def tick(core, now, yaw=0.0, height=2.0, kf_fresh=True, terminal=False, dt=0.02):
    return core.update(now, yaw, height, dt, kf_fresh, terminal=terminal)


def lock(core, start=1.0):
    for stamp in (start, start + 0.02, start + 0.04):
        core.observe(stamp, 0.05, True, stamp, position_valid=True)
    decision = tick(core, start + 0.04)
    assert decision.state == 'TARGET_LOCK'
    assert decision.locked
    return start + 0.04


def lose(core, start=1.06):
    for stamp in (start, start + 0.02, start + 0.04):
        core.observe(stamp, 0.0, False, stamp)
    return start + 0.04


def test_lock_requires_three_unique_current_centered_frames():
    core = TargetVisibilityState()
    for _ in range(10):
        core.observe(1.0, 0.05, True, 1.0, position_valid=True)
    assert core.consecutive_valid_frames == 1
    assert not tick(core, 1.0).locked
    core.observe(1.02, 0.05, True, 1.02, position_valid=True)
    assert not tick(core, 1.02).locked
    core.observe(1.04, 0.05, True, 1.04, position_valid=True)
    assert tick(core, 1.04).state == 'TARGET_LOCK'
    assert tick(core, 1.05).state == 'TRACKING'


def test_out_of_order_bearing_callbacks_do_not_add_or_remove_frames():
    core = TargetVisibilityState()
    lock(core)
    assert not core.observe(1.01, -0.8, False, 1.05)
    assert core.consecutive_valid_frames == 3
    assert core.consecutive_lost_frames == 0
    assert tick(core, 1.05).locked


@pytest.mark.parametrize('bearing,expected', [(0.4, 0.4), (-0.4, -0.4)])
def test_rgb_only_bearing_centers_in_correct_ned_yaw_direction(bearing, expected):
    core = TargetVisibilityState()
    for stamp in (1.0, 1.02, 1.04):
        core.observe(stamp, bearing, True, stamp)
    decision = tick(core, 1.04)
    assert decision.visible
    assert not decision.locked
    assert decision.yaw_rate == pytest.approx(expected)
    assert core.last_valid_observation_stamp is None


def test_separate_position_callback_can_complete_lock_gate():
    core = TargetVisibilityState()
    for stamp in (1.0, 1.02, 1.04):
        core.observe(stamp, 0.05, True, stamp)
    assert not tick(core, 1.04).locked
    core.mark_observation(1.04, True, now=1.04)
    assert tick(core, 1.04).locked


@pytest.mark.parametrize('missing', ['kf', 'position', 'centered', 'current_frames'])
def test_lock_requires_all_independent_gates(missing):
    core = TargetVisibilityState()
    stamps = (1.0, 1.2, 1.22) if missing == 'current_frames' else (1.0, 1.02, 1.04)
    for stamp in stamps:
        core.observe(stamp, 0.3 if missing == 'centered' else 0.05,
                     True, stamp, position_valid=missing != 'position')
    assert not tick(core, stamps[-1], kf_fresh=missing != 'kf').locked


def test_one_or_two_missing_frames_preserve_lock_then_three_reacquire():
    core = TargetVisibilityState()
    lock(core)
    for stamp in (1.06, 1.08):
        core.observe(stamp, 0.0, False, stamp)
        assert tick(core, stamp).locked
    core.observe(1.10, 0.0, False, 1.10)
    decision = tick(core, 1.10)
    assert decision.state == 'REACQUIRE'
    assert not decision.locked


def test_repeated_invalid_callback_never_manufactures_frame_loss():
    core = TargetVisibilityState()
    lock(core)
    for _ in range(10):
        core.observe(1.06, 0.0, False, 1.06)
    assert core.consecutive_lost_frames == 1
    assert tick(core, 1.06).locked


def test_silence_has_bounded_debounce_without_callbacks():
    core = TargetVisibilityState()
    lock(core)
    assert tick(core, 1.20).locked
    decision = tick(core, 1.50)
    assert not decision.locked
    assert not decision.visible
    assert decision.state == 'REACQUIRE'


def test_reacquisition_one_frame_never_relocks():
    core = TargetVisibilityState()
    lock(core)
    tick(core, lose(core))
    core.observe(1.12, 0.05, True, 1.12, position_valid=True)
    assert not tick(core, 1.12).locked
    for stamp in (1.14, 1.16):
        core.observe(stamp, 0.05, True, stamp, position_valid=True)
    assert tick(core, 1.16).state == 'TARGET_LOCK'


@pytest.mark.parametrize('bearing,sign', [(0.4, 1), (-0.4, -1)])
def test_last_visible_image_side_selects_first_search_direction(bearing, sign):
    core = TargetVisibilityState()
    core.observe(1.0, bearing, True, 1.0)
    core.observe(1.02, 0.0, True, 1.02)
    lose(core, 1.04)
    decision = tick(core, 1.08)
    assert decision.search_direction == sign
    assert decision.yaw_rate * sign > 0.0


def test_no_target_initial_sweep_is_height_gated():
    core = TargetVisibilityState()
    assert tick(core, 1.0, height=0.5).yaw_rate == 0.0
    decision = tick(core, 1.02)
    assert decision.state == 'TARGET_ACQUIRE'
    assert decision.yaw_rate == pytest.approx(0.25)
    assert not core.has_ever_seen_target


def test_initial_sweep_stops_after_one_full_turn_through_yaw_wrap():
    core = TargetVisibilityState()
    yaw = 2.9
    for index in range(1400):
        decision = tick(core, index * 0.02, yaw=yaw)
        yaw = (yaw + decision.yaw_rate * 0.02 + math.pi) % (2 * math.pi) - math.pi
    assert decision.state == 'SAFE_WAIT'
    assert decision.yaw_rate == 0.0
    assert tick(core, 40.0, yaw=yaw).yaw_rate == 0.0


def test_reacquire_alternates_expanding_arcs_and_finishes():
    core = TargetVisibilityState()
    lock(core)
    now = lose(core)
    yaw = 0.0
    extrema = []
    previous_sign = 1
    for index in range(12000):
        decision = tick(core, now + index * 0.02, yaw=yaw)
        direction = 1 if decision.yaw_rate > 0 else -1 if decision.yaw_rate < 0 else 0
        if direction and direction != previous_sign:
            extrema.append(yaw)
            previous_sign = direction
        yaw += decision.yaw_rate * 0.02
    assert extrema[:3] == pytest.approx([0.52, -1.04, 1.56], abs=0.015)
    assert min(extrema) >= -math.pi - 0.015
    assert max(extrema) <= math.pi + 0.015
    assert decision.state == 'SAFE_WAIT'
    assert decision.yaw_rate == 0.0


def test_reacquire_timeout_is_safe_wait_while_finite_yaw_scan_continues():
    core = TargetVisibilityState()
    lock(core)
    now = lose(core)
    tick(core, now)
    decision = tick(core, now + 2.01, yaw=0.4)
    assert decision.state == 'SAFE_WAIT'
    assert decision.yaw_rate != 0.0


@pytest.mark.parametrize('sign', [-1, 1])
def test_scan_advances_after_actual_heading_crosses_arc_endpoint(sign):
    core = TargetVisibilityState()
    core.observe(1.0, sign * .4, True, 1.0)
    now = lose(core, 1.02)
    tick(core, now, yaw=0.0)
    tick(core, now + .02, yaw=sign * .55)
    # Moving back through the first endpoint must keep moving toward the
    # second arc, rather than oscillate about the already completed arc.
    decision = tick(core, now + .04, yaw=sign * .51)
    assert decision.yaw_rate * sign < 0.0


def test_stalled_aircraft_cannot_command_an_unbounded_turn():
    core = TargetVisibilityState()
    for index in range(3000):
        decision = tick(core, index * 0.02, yaw=0.0)
    assert decision.state == 'SAFE_WAIT'
    assert decision.yaw_rate == 0.0


def test_target_reappearing_midscan_restarts_next_loss_toward_new_side():
    core = TargetVisibilityState()
    core.observe(1.0, 0.4, True, 1.0)
    tick(core, lose(core, 1.02))
    tick(core, 1.5, yaw=0.52)
    core.observe(1.52, 0.4, True, 1.52)
    tick(core, 1.52, yaw=0.52)
    now = lose(core, 1.54)
    decision = tick(core, now, yaw=0.52)
    assert decision.search_direction == 1
    assert decision.yaw_rate > 0.0


def test_fresh_rgb_cannot_reuse_stale_3d_or_relock_when_kf_expires():
    core = TargetVisibilityState()
    lock(core)
    assert not tick(core, 1.05, kf_fresh=False).locked
    for stamp in (1.30, 1.32, 1.34):
        core.observe(stamp, 0.05, True, stamp)
    decision = tick(core, 1.34)
    assert not decision.locked
    assert decision.visible


def test_already_counted_bearing_can_receive_its_position_without_an_extra_frame():
    core = TargetVisibilityState()
    core.observe(1.0, 0.05, True, 1.0)
    assert not core.observe(1.0, 0.05, True, 1.0, position_valid=True)
    assert core.consecutive_valid_frames == 1
    assert core.last_valid_observation_stamp == 1.0


def test_small_arc_increment_remains_a_finite_scan():
    core = TargetVisibilityState(VisibilityConfig(search_arc_increment=1e-300))
    core.observe(1.0, 0.4, True, 1.0)
    now = lose(core, 1.02)
    for index in range(30000):
        decision = tick(core, now + index * 0.02)
    assert decision.state == 'SAFE_WAIT'
    assert decision.yaw_rate == 0.0


def test_three_consecutive_ten_hertz_frames_lock_with_current_latest_stamp():
    core = TargetVisibilityState()
    for stamp in (10.0, 10.1, 10.2):
        core.observe(stamp, 0.05, True, stamp, position_valid=True)
    assert tick(core, 10.2).locked


@pytest.mark.parametrize('stamp', [0.0, -0.1])
def test_nonpositive_acquisition_stamp_cannot_count_as_measurement(stamp):
    core = TargetVisibilityState()
    assert not core.observe(stamp, 0.05, True, 1.0, position_valid=True)
    assert not core.mark_observation(stamp, True, now=1.0)
    assert core.consecutive_valid_frames == 0


def test_visual_servo_deadband_and_rate_limit():
    core = TargetVisibilityState()
    core.observe(1.0, 0.02, True, 1.0)
    assert tick(core, 1.0).yaw_rate == 0.0
    core.observe(1.02, 1.0, True, 1.02)
    assert tick(core, 1.02).yaw_rate == pytest.approx(0.6)


def test_terminal_loss_recovers_before_search_and_latches_until_clearance():
    core = TargetVisibilityState()
    lock(core)
    core.observe(1.06, 0.0, False, 1.06)
    assert tick(core, 1.06, terminal=True).locked
    decision = tick(core, 1.37, terminal=True)
    assert decision.state == 'SAFE_RECOVERY'
    assert decision.yaw_rate == 0.0
    for stamp in (1.38, 1.40, 1.42):
        core.observe(stamp, 0.05, True, stamp, position_valid=True)
    assert tick(core, 1.42, terminal=True).state == 'SAFE_RECOVERY'
    assert tick(core, 1.44, terminal=False).state == 'TARGET_LOCK'


def test_terminal_confirmed_loss_does_not_yaw_during_recovery_grace():
    core = TargetVisibilityState()
    lock(core)
    decision = tick(core, lose(core), terminal=True)
    assert not decision.locked
    assert decision.yaw_rate == 0.0


def test_terminal_silent_stream_recovers_after_freshness_plus_timeout():
    core = TargetVisibilityState()
    lock(core)
    decision = tick(core, 1.50, terminal=True)
    assert decision.state == 'SAFE_RECOVERY'
    assert decision.yaw_rate == 0.0


@pytest.mark.parametrize('reset_kind', ['explicit', 'time_rewind'])
def test_new_mission_or_clock_rewind_drops_lock_and_target_history(reset_kind):
    core = TargetVisibilityState()
    lock(core)
    if reset_kind == 'explicit':
        core.reset()
    decision = tick(core, 0.1)
    assert not decision.locked
    assert not core.has_ever_seen_target
    assert not core.has_ever_locked_target
    assert core.last_valid_observation_stamp is None
    assert core.consecutive_valid_frames == 0


@pytest.mark.parametrize('stamp,bearing', [(2.0, 0.1), (1.0, math.nan)])
def test_future_or_nonfinite_measurement_never_locks(stamp, bearing):
    core = TargetVisibilityState()
    core.observe(stamp, bearing, True, 1.0, position_valid=True)
    assert not tick(core, 1.0).locked
    assert not core.has_ever_seen_target


@pytest.mark.parametrize('changes', [
    {'target_lock_min_frames': 0}, {'target_loss_frames': 1.5},
    {'target_lock_max_age': math.nan}, {'vision_yaw_gain': -1},
    {'search_initial_arc': math.inf}, {'initial_search_yaw_rate': 0},
    {'target_center_deadband_rad': 0.2},
])
def test_invalid_configuration_fails_closed(changes):
    with pytest.raises(ValueError):
        TargetVisibilityState(VisibilityConfig(**changes))
