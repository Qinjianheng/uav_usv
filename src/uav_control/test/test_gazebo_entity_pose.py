import pytest
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.pose_v_pb2 import Pose_V

from uav_control.evaluation.gazebo_entity_pose import (
    entity_geometry_residuals,
    GazeboEntityPoseTracker,
)
from uav_control.evaluation.intercept_evaluator_node import (
    InterceptEvaluatorNode,
)


def _gazebo_time(message, value):
    message.sec = int(value)
    message.nsec = round((value - int(value)) * 1e9)


def test_entity_pose_uses_gazebo_sample_stamp_and_enu_to_ned_mapping():
    tracker = GazeboEntityPoseTracker(history_duration=1.0)
    assert tracker.add_clock_anchor(10.0, 1000.0, 1000.01, 50.0)
    assert tracker.add_clock_anchor(10.1, 1000.1, 1000.11, 50.1)

    assert tracker.add_pose(
        sim_stamp=10.05,
        gazebo_enu_position=(2.0, 1.0, 0.42),
        now_ros_time=1000.11,
    )

    sample = tracker.position_at(1000.05)
    assert sample == pytest.approx((1.0, 2.0, -0.42))
    assert tracker.last_raw_stamp == pytest.approx(10.05)
    assert tracker.last_mapped_stamp == pytest.approx(1000.05)
    assert tracker.last_status == 'MAPPED_INTERPOLATED'


def test_entity_pose_without_trusted_clock_mapping_is_not_recorded():
    tracker = GazeboEntityPoseTracker(history_duration=1.0)

    assert not tracker.add_pose(
        sim_stamp=10.0,
        gazebo_enu_position=(2.0, 1.0, 0.42),
        now_ros_time=1000.0,
    )
    assert tracker.position_at(1000.0) is None
    assert tracker.last_status == 'CLOCK_REFERENCE_UNAVAILABLE'


def test_entity_pose_clock_reset_clears_old_history():
    tracker = GazeboEntityPoseTracker(history_duration=1.0)
    tracker.add_clock_anchor(10.0, 1000.0, 1000.01, 50.0)
    tracker.add_clock_anchor(10.1, 1000.1, 1000.11, 50.1)
    assert tracker.add_pose(10.05, (2.0, 1.0, 0.42), 1000.11)

    assert not tracker.add_clock_anchor(1.0, 1000.2, 1000.21, 50.2)

    assert tracker.position_at(1000.05) is None
    assert tracker.reset_count == 1


def test_evaluator_callbacks_select_target_entity_and_keep_sample_time():
    node = object.__new__(InterceptEvaluatorNode)
    node.gazebo_entity_tracker = GazeboEntityPoseTracker(
        history_duration=1.0
    )
    node.gazebo_target_entity_name = 'usv_target'
    now = [1000.01]
    node._now = lambda: now[0]

    for sim_time, system_time in ((10.0, 1000.0), (10.1, 1000.1)):
        clock = Clock()
        _gazebo_time(clock.sim, sim_time)
        _gazebo_time(clock.system, system_time)
        now[0] = system_time + 0.01
        node.gazebo_entity_clock_callback(clock)

    poses = Pose_V()
    _gazebo_time(poses.header.stamp, 10.05)
    distractor = poses.pose.add()
    distractor.name = 'other_model'
    target = poses.pose.add()
    target.name = 'default::usv_target'
    target.position.x = 2.0
    target.position.y = 1.0
    target.position.z = 0.42
    node.gazebo_entity_pose_callback(poses)

    assert node.gazebo_entity_tracker.position_at(1000.05) == (
        pytest.approx((1.0, 2.0, -0.42))
    )


def test_entity_geometry_residuals_isolate_projection_and_camera_stages():
    result = entity_geometry_residuals(
        entity_center_ned=(8.0, 0.0, -0.42),
        uav_position_ned=(0.0, 0.0, -5.0),
        attitude_quaternion=(1.0, 0.0, 0.0, 0.0),
        camera_translation_flu=(0.35, 0.0, -0.05),
        camera_pitch_down=0.0,
        intrinsics=(100.0, 100.0, 50.0, 40.0),
        observed_projection_center=(48.0, 43.0),
        observed_center_camera=(7.65, 0.2, -4.38),
        observed_body_flu=(8.0, 0.2, -4.43),
    )

    assert result.expected_camera == pytest.approx((7.65, 0.0, -4.53))
    assert result.expected_projection == pytest.approx((50.0, 99.21568627))
    assert result.pixel_error == pytest.approx((-2.0, -56.21568627))
    assert result.camera_error == pytest.approx((0.0, 0.2, 0.15))
    assert result.body_error == pytest.approx((0.0, 0.2, 0.15))
