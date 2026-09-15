from uav_control.mission.mission_manager import MissionManagerCore
from uav_control.mission.mission_manager import MissionPhase
from uav_control.mission.mission_manager_node import mission_state_message


def test_mission_state_message_uses_numeric_phase_and_current_identity():
    core = MissionManagerCore()
    core.set_flight_ready(True)
    core.handle_command('X', now=1.0)

    message = mission_state_message(core, stamp_seconds=12.25)

    assert message.stamp.sec == 12
    assert message.stamp.nanosec == 250_000_000
    assert message.mission_id == 1
    assert message.state == MissionPhase.TAKEOFF
    assert message.state_name == 'TAKEOFF'
    assert not message.intercept_requested
    assert not message.completed
