import pytest
from px4_msgs.msg import VehicleLocalPosition
from uav_usv_interfaces.msg import TargetState

from uav_control.evaluation.intercept_evaluator import EvaluationResult
from uav_control.evaluation.intercept_evaluator_node import result_to_message
from uav_control.evaluation.intercept_evaluator_node import truth_from_message
from uav_control.evaluation.intercept_evaluator_node import uav_from_message


def test_truth_and_uav_messages_convert_without_changing_coordinates():
    target = TargetState()
    target.position.x = 1.0
    target.position.y = 2.0
    target.position.z = 0.1
    target.velocity.x = 3.0
    target.velocity.y = 4.0
    target.velocity.z = 0.2
    target.valid = True
    vehicle = VehicleLocalPosition()
    vehicle.x = -1.0
    vehicle.y = -2.0
    vehicle.z = -3.0
    vehicle.vx = 0.5
    vehicle.vy = 0.6
    vehicle.vz = 0.7

    truth_state = truth_from_message(target)
    uav_state = uav_from_message(vehicle)

    assert truth_state.position == pytest.approx((1.0, 2.0, 0.1))
    assert truth_state.velocity == pytest.approx((3.0, 4.0, 0.2))
    assert uav_state.position == pytest.approx((-1.0, -2.0, -3.0))
    assert uav_state.velocity == pytest.approx((0.5, 0.6, 0.7))


def test_result_message_preserves_truth_metrics_and_mission_identity():
    result = EvaluationResult(
        mission_id=9,
        success=False,
        outcome='FAILURE',
        reason='SEA_CONTACT',
        elapsed_time=4.0,
        minimum_distance=0.4,
        horizontal_distance=0.3,
        vertical_error=0.2,
        relative_speed=1.5,
        closing_speed=1.0,
        maximum_horizontal_speed=6.0,
        maximum_vertical_speed=3.0,
        maximum_horizontal_acceleration=2.5,
        maximum_vertical_acceleration=2.0,
    )

    message = result_to_message(result, stamp_seconds=50.25, radius=0.25)

    assert message.stamp.sec == 50
    assert message.stamp.nanosec == 250_000_000
    assert message.mission_id == 9
    assert message.reason == 'SEA_CONTACT'
    assert message.minimum_distance == pytest.approx(0.4)
    assert message.max_horizontal_speed == pytest.approx(6.0)


def test_result_message_carries_the_body_contact_detail():
    result = EvaluationResult(
        mission_id=9,
        success=False,
        outcome='FAILURE',
        reason='SEA_CONTACT',
        elapsed_time=4.0,
        minimum_distance=0.4,
        horizontal_distance=0.3,
        vertical_error=0.2,
        relative_speed=1.5,
        closing_speed=1.0,
        maximum_horizontal_speed=6.0,
        maximum_vertical_speed=3.0,
        maximum_horizontal_acceleration=2.5,
        maximum_vertical_acceleration=2.0,
        detail='BODY_LOWEST_POINT_AT_SEA_SURFACE',
    )

    message = result_to_message(result, stamp_seconds=1.0, radius=0.25)

    assert message.detail == 'BODY_LOWEST_POINT_AT_SEA_SURFACE'
