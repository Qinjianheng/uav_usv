import numpy as np
import pytest
from types import SimpleNamespace

from uav_control.tracking import target_kalman_filter
from uav_usv_interfaces.msg import TargetObservation


def test_measurement_from_observation_preserves_stamp_and_covariance():
    message = TargetObservation()
    message.stamp.sec = 12
    message.stamp.nanosec = 345000000
    message.frame_id = 'local_ned'
    message.position.x = 1.0
    message.position.y = 2.0
    message.position.z = -0.1
    message.covariance = [
        0.04, 0.0, 0.0,
        0.0, 0.09, 0.0,
        0.0, 0.0, 0.16,
    ]
    message.confidence = 0.8
    message.source = 'front_rgbd_red_sphere'
    message.valid = True

    helper = getattr(
        target_kalman_filter,
        'measurement_from_observation',
        None,
    )

    assert helper is not None

    measurement, covariance, timestamp_ns = helper(
        message,
        'local_ned',
    )

    assert measurement == pytest.approx([1.0, 2.0, -0.1])
    assert covariance == pytest.approx(np.diag([0.04, 0.09, 0.16]))
    assert timestamp_ns == 12_345_000_000


def test_invalid_time_rejected_observation_never_enters_kf():
    message = TargetObservation()
    message.valid = False
    message.rejection_reason = 'POSITION_TIMESTAMP_BEFORE_HISTORY'
    message.frame_id = 'local_ned'

    with pytest.raises(ValueError, match='invalid'):
        target_kalman_filter.measurement_from_observation(
            message,
            'local_ned',
        )


def test_kf_projection_keeps_last_real_observation_as_source_stamp():
    node = object.__new__(target_kalman_filter.TargetKalmanFilterNode)
    node.filter = SimpleNamespace(
        initialized=True,
        project=lambda _dt: (np.zeros(6), np.eye(6)),
    )
    node.filter_time_ns = 10_000_000_000
    node.last_measurement_time_ns = 10_000_000_000
    node.measurement_timeout = 0.5
    node.prediction_horizon = 0.5
    node.frame_id = 'local_ned'
    now = [10_200_000_000]
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: target_kalman_filter.Time(nanoseconds=now[0])
    )
    states = []
    node.state_pub = SimpleNamespace(
        publish=lambda message: states.append(message)
    )
    node.prediction_pub = SimpleNamespace(publish=lambda _message: None)

    node.timer_callback()
    now[0] = 10_400_000_000
    node.timer_callback()

    source_stamps = [
        message.source_stamp.sec + message.source_stamp.nanosec * 1e-9
        for message in states
    ]
    assert source_stamps == pytest.approx([10.0, 10.0])
