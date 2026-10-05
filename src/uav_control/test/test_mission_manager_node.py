from uav_control.mission.mission_manager import MissionManagerCore
from uav_control.mission.mission_manager import MissionPhase
from uav_control.mission.mission_manager_node import mission_state_message
from uav_control.mission.mission_manager_node import MissionManagerNode


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


def test_pending_y_callback_publishes_intent_and_explains_waiting():
    from types import SimpleNamespace
    core = MissionManagerCore()
    core.phase = MissionPhase.FOLLOW
    node = object.__new__(MissionManagerNode)
    node.core = core
    node._now = lambda: 12.
    published, logs = [], []
    node._publish = lambda: published.append(core.intercept_requested)
    node.get_logger = lambda: SimpleNamespace(info=logs.append)
    node.command_callback(SimpleNamespace(data='Y'))
    assert published == [True]
    assert core.phase == MissionPhase.FOLLOW
    assert 'pending fresh visual/KF lock' in logs[0]
