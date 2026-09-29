"""Safety and reference boundaries for the isolated P7 motion driver."""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
from px4_msgs.msg import VehicleStatus


SCRIPT = Path(__file__).resolve().parents[3] / 'scripts' / (
    'p7_px4_local_axis_motion.py')


def load_script():
    spec = importlib.util.spec_from_file_location('p7_axis_motion', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def status(preflight=True, failsafe=False, armed=False):
    return SimpleNamespace(
        pre_flight_checks_pass=preflight, failsafe=failsafe,
        nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD,
        arming_state=(VehicleStatus.ARMING_STATE_ARMED if armed else
                      VehicleStatus.ARMING_STATE_DISARMED),
    )


def test_x_gate_requires_preflight_offboard_and_disarmed():
    module = load_script()
    assert module.ready_for_takeoff(status(), True, 2.0)
    assert not module.ready_for_takeoff(status(preflight=False), True, 2.0)
    assert not module.ready_for_takeoff(status(failsafe=True), True, 2.0)
    assert not module.ready_for_takeoff(status(armed=True), True, 2.0)
    assert not module.ready_for_takeoff(status(), False, 2.0)
    assert not module.ready_for_takeoff(status(), True, 1.9)


def test_axis_moves_and_yaw_do_not_mix_world_displacement():
    module = load_script()
    origin = (3.0, 5.0)
    initial_yaw = 0.2
    north, yaw = module.phase_target('NORTH_OUT', 10.0,
                                     origin, initial_yaw)
    assert north == pytest.approx((11.0, 5.0, -5.0))
    assert yaw == pytest.approx(initial_yaw)
    east, _ = module.phase_target('EAST_OUT', 10.0,
                                  origin, initial_yaw)
    assert east == pytest.approx((3.0, 13.0, -5.0))
    fixed, yaw = module.phase_target('YAW_P90', 5.0,
                                     origin, initial_yaw)
    assert fixed == pytest.approx((3.0, 5.0, -5.0))
    assert yaw == pytest.approx(initial_yaw + math.pi / 2)
    assert module.clamp_rate(0.0, 8.0, 1.0, 0.1) == pytest.approx(0.1)


def test_landing_does_not_publish_hover_setpoint():
    module = load_script()
    assert module.should_publish_reference('HOVER')
    assert module.should_publish_reference('TAKEOFF')
    assert not module.should_publish_reference('LANDING')
    assert not module.should_publish_reference('DONE')


def test_yaw_offset_profile_keeps_nonzero_world_position():
    module = load_script()
    phases = module.phase_schedule('yaw_offset')
    names = [name for name, _ in phases]
    assert names.index('NORTH_OUT') < names.index('YAW_P90')
    assert names.index('YAW_P90') < names.index('NORTH_RETURN')
    position, _ = module.phase_target(
        'YAW_P90', 5.0, (3.0, 5.0), 0.2,
        yaw_north_offset_m=8.0)
    assert position == pytest.approx((11.0, 5.0, -5.0))
