import numpy as np
import pytest

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
