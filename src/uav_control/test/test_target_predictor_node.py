"""ROS message boundary tests for the target predictor node."""

import pytest
from uav_usv_interfaces.msg import TargetState

from uav_control.tracking.target_prediction import PredictionEngine
from uav_control.tracking.target_predictor_node import prediction_to_message
from uav_control.tracking.target_predictor_node import state_from_message


def test_target_state_conversion_preserves_measurement_time():
    message = TargetState()
    message.stamp.sec = 12
    message.stamp.nanosec = 250_000_000
    message.position.x = 1.0
    message.position.y = 2.0
    message.position.z = 0.1
    message.velocity.x = 3.0
    message.velocity.y = 4.0
    message.velocity.z = 0.2
    message.valid = True

    state = state_from_message(message)

    assert state.stamp == pytest.approx(12.25)
    assert state.position == pytest.approx((1.0, 2.0, 0.1))
    assert state.velocity == pytest.approx((3.0, 4.0, 0.2))


def test_prediction_message_keeps_source_age_and_samples():
    state_message = TargetState()
    state_message.stamp.sec = 20
    state_message.position.x = 1.0
    state_message.position.y = 2.0
    state_message.position.z = 0.0
    state_message.velocity.x = 4.0
    state_message.velocity.z = 0.2
    state_message.valid = True
    engine = PredictionEngine(horizon=0.2, sample_period=0.1)
    engine.update(state_from_message(state_message))
    result = engine.generate(now=20.05, mission_id=9, sequence_id=4)

    message = prediction_to_message(result, compute_time=0.001)

    assert message.mission_id == 9
    assert message.sequence_id == 4
    assert message.source_stamp.sec == 20
    assert message.generated_stamp.sec == 20
    assert message.generated_stamp.nanosec == 50_000_000
    assert message.compute_time == pytest.approx(0.001)
    assert len(message.samples) == 3
    assert message.samples[-1].relative_time.nanosec == 200_000_000
    assert message.samples[-1].position.x == pytest.approx(1.8)
    assert len(message.samples[-1].position_covariance) == 9
