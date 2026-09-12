import math
from collections import deque
from types import SimpleNamespace

import pytest
from px4_msgs.msg import VehicleCommand

from uav_control.moving_target import MovingTarget
from uav_control.trajectory_impact_sim import TrajectoryImpactSim


def test_csv_core_state_columns_are_grouped():
    assert TrajectoryImpactSim.CSV_FIELDS[:13] == [
        'time',
        'uav_x',
        'uav_y',
        'uav_z',
        'uav_vx',
        'uav_vy',
        'uav_vz',
        'target_x',
        'target_y',
        'target_z',
        'target_vx',
        'target_vy',
        'target_vz',
    ]


def test_csv_records_intercept_reference_kinematics():
    intercept_index = TrajectoryImpactSim.CSV_FIELDS.index('intercept_x')

    assert TrajectoryImpactSim.CSV_FIELDS[
        intercept_index:intercept_index + 9
    ] == [
        'intercept_x',
        'intercept_y',
        'intercept_z',
        'intercept_vx',
        'intercept_vy',
        'intercept_vz',
        'intercept_ax',
        'intercept_ay',
        'intercept_az',
    ]


def test_csv_records_camera_kalman_and_delayed_prediction_errors():
    target_velocity_index = TrajectoryImpactSim.CSV_FIELDS.index('target_vz')
    assert TrajectoryImpactSim.CSV_FIELDS[
        target_velocity_index + 1:target_velocity_index + 16
    ] == [
        'camera_measurement_valid',
        'camera_x',
        'camera_y',
        'camera_z',
        'camera_position_error',
        'camera_confidence',
        'kf_state_valid',
        'kf_x',
        'kf_y',
        'kf_z',
        'kf_vx',
        'kf_vy',
        'kf_vz',
        'kf_position_error',
        'kf_state_age',
    ]
    for model in ('guidance', 'kf'):
        for horizon in ('0p5', '1p0', '2p0'):
            prefix = f'{model}_prediction_{horizon}'
            for suffix in ('x', 'y', 'z', 'error', 'age'):
                assert f'{prefix}_{suffix}' in TrajectoryImpactSim.CSV_FIELDS


def test_csv_records_front_view_guidance_diagnostics():
    t_go_index = TrajectoryImpactSim.CSV_FIELDS.index('t_go')

    assert TrajectoryImpactSim.CSV_FIELDS[
        t_go_index + 1:t_go_index + 3
    ] == [
        'guidance_altitude_reference',
        'guidance_closing_speed',
    ]


def test_takeoff_follow_setpoint_uses_smooth_three_axis_velocity():
    published = []
    controller = SimpleNamespace(
        setpoint_pub=SimpleNamespace(publish=published.append),
        timestamp=lambda: 123,
        update_observation_yaw=lambda: 0.75,
    )

    TrajectoryImpactSim.publish_takeoff_follow_setpoint(
        controller,
        3.0,
        4.0,
        -1.0,
    )

    assert len(published) == 1
    message = published[0]
    assert math.isnan(message.position[0])
    assert math.isnan(message.position[1])
    assert math.isnan(message.position[2])
    assert message.velocity == pytest.approx([3.0, 4.0, -1.0])
    assert message.yaw == pytest.approx(0.75)


@pytest.mark.parametrize(
    ('clearance', 'expected'),
    [
        (0.0, 0.0),
        (0.5, 0.0),
        (1.0, 0.5),
        (1.5, 1.0),
        (2.0, 1.0),
    ],
)
def test_takeoff_horizontal_follow_blends_after_liftoff(
    clearance,
    expected,
):
    scale = TrajectoryImpactSim.takeoff_horizontal_scale(
        clearance,
        0.5,
        1.5,
    )

    assert scale == pytest.approx(expected)


@pytest.mark.parametrize(
    ('takeoff_requested', 'completed', 'position', 'velocity'),
    [
        (False, False, True, False),
        (True, False, False, True),
        (True, True, True, False),
    ],
)
def test_offboard_mode_has_no_takeoff_control_mode_switch(
    takeoff_requested,
    completed,
    position,
    velocity,
):
    published = []
    controller = SimpleNamespace(
        offboard_pub=SimpleNamespace(publish=published.append),
        timestamp=lambda: 123,
        takeoff_requested=takeoff_requested,
        completed=completed,
    )

    TrajectoryImpactSim.publish_offboard_mode(controller)

    assert len(published) == 1
    assert published[0].position is position
    assert published[0].velocity is velocity


def test_observation_yaw_points_from_uav_to_target():
    yaw = TrajectoryImpactSim.target_observation_yaw(
        2.0,
        3.0,
        2.0,
        8.0,
    )

    assert yaw == pytest.approx(math.pi / 2.0)


def test_observation_yaw_rate_limit_crosses_pi_continuously():
    command = TrajectoryImpactSim.rate_limited_yaw(
        math.radians(179.0),
        math.radians(-179.0),
        math.radians(1.0),
    )

    assert abs(
        TrajectoryImpactSim.wrap_angle(
            command - math.radians(180.0)
        )
    ) < 1e-9


def test_intercept_prediction_horizon_stays_between_one_and_two_seconds():
    horizons = []

    def predict_state(x, y, z, horizon):
        horizons.append(horizon)
        return x, y, z, 5.0, 0.0, 0.0

    controller = SimpleNamespace(
        intercept_guidance_horizon_min=1.0,
        intercept_guidance_horizon_max=2.0,
        intercept_guidance_horizon_distance=20.0,
        predict_target_state=predict_state,
        target_predictor=SimpleNamespace(
            turn_rate=0.0,
            maneuver_model_active=False,
        ),
    )

    near_solution = TrajectoryImpactSim.calculate_intercept_solution(
        controller,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    far_solution = TrajectoryImpactSim.calculate_intercept_solution(
        controller,
        0.0,
        0.0,
        0.0,
        40.0,
        0.0,
        0.0,
    )

    assert horizons == pytest.approx([1.0, 2.0])
    assert near_solution[3] == pytest.approx(1.0)
    assert far_solution[3] == pytest.approx(2.0)


def make_front_view_altitude_controller(terminal_minimum_clearance=0.0):
    controller = SimpleNamespace(
        sea_surface_z=0.0,
        flight_altitude=-5.0,
        terminal_dive_angle=math.radians(45.0),
        front_camera_max_depression_angle=0.85,
        terminal_contact_clearance=0.05,
        impact_radius=0.25,
        terminal_minimum_clearance=terminal_minimum_clearance,
        dt=0.05,
    )
    controller.pursuit_altitude_reference = lambda distance, target_z: (
        TrajectoryImpactSim.pursuit_altitude_reference(
            controller,
            distance,
            target_z,
        )
    )
    return controller


def test_pursuit_holds_cruise_altitude_before_descent_window():
    controller = make_front_view_altitude_controller()

    reference_z = TrajectoryImpactSim.pursuit_altitude_reference(
        controller,
        15.0,
        0.15,
    )

    assert reference_z == pytest.approx(-5.0)


def test_pursuit_holds_cruise_height_before_45_degree_dive_line():
    controller = make_front_view_altitude_controller()

    reference_z = TrajectoryImpactSim.pursuit_altitude_reference(
        controller,
        7.5,
        0.0,
    )

    assert reference_z == pytest.approx(-5.0)


def test_pursuit_follows_45_degree_dive_line_to_capture_point():
    controller = make_front_view_altitude_controller()

    far_reference_z = TrajectoryImpactSim.pursuit_altitude_reference(
        controller,
        4.0,
        0.0,
    )
    near_reference_z = TrajectoryImpactSim.pursuit_altitude_reference(
        controller,
        2.0,
        0.0,
    )

    assert far_reference_z == pytest.approx(-4.05)
    assert near_reference_z == pytest.approx(-2.05)
    assert far_reference_z - near_reference_z == pytest.approx(-2.0)


def test_pursuit_vertical_velocity_tracks_45_degree_dive_slope():
    controller = make_front_view_altitude_controller()
    controller.sim_z = -4.05
    controller.altitude_velocity_gain = 1.0
    controller.max_vertical_speed = 4.0

    desired_vz = TrajectoryImpactSim.pursuit_vertical_velocity(
        controller,
        horizontal_distance=4.0,
        target_z=0.0,
        target_vz=0.0,
        altitude_reference=-4.05,
        horizontal_closing_speed=1.5,
    )

    assert desired_vz == pytest.approx(1.5)


def test_pursuit_near_target_respects_front_camera_depression_limit():
    controller = make_front_view_altitude_controller()
    horizontal_distance = 0.2

    reference_z = TrajectoryImpactSim.pursuit_altitude_reference(
        controller,
        horizontal_distance,
        0.0,
    )

    height = -reference_z
    depression = math.atan2(height, horizontal_distance)
    assert depression == pytest.approx(0.85)
    assert reference_z < controller.sea_surface_z


def test_terminal_closing_speed_ceiling_is_independent_of_follow():
    controller = SimpleNamespace(
        max_acceleration=4.8,
        terminal_max_acceleration=3.0,
        max_actual_horizontal_acceleration=5.0,
        horizontal_acceleration_guard_margin=0.5,
        enable_gazebo_control=True,
        follow_max_closing_speed=1.5,
        impact_radius=0.25,
    )
    controller.limit_horizontal_acceleration = lambda requested: (
        TrajectoryImpactSim.limit_horizontal_acceleration(
            controller,
            requested,
        )
    )

    follow_ceiling = TrajectoryImpactSim.pursuit_closing_speed(
        controller,
        3.0,
        0.3,
    )
    terminal_ceiling = TrajectoryImpactSim.pursuit_closing_speed(
        controller,
        3.0,
        0.3,
        3.0,
    )

    # The braking law is what stops the vehicle at the capture
    # neighbourhood; only the ceiling differs between the two phases.
    assert follow_ceiling == pytest.approx(1.5)
    assert terminal_ceiling > follow_ceiling


def test_terminal_closing_speed_still_brakes_at_capture_neighbourhood():
    controller = SimpleNamespace(
        max_acceleration=4.8,
        terminal_max_acceleration=3.0,
        max_actual_horizontal_acceleration=5.0,
        horizontal_acceleration_guard_margin=0.5,
        enable_gazebo_control=True,
        follow_max_closing_speed=1.5,
        impact_radius=0.25,
    )
    controller.limit_horizontal_acceleration = lambda requested: (
        TrajectoryImpactSim.limit_horizontal_acceleration(
            controller,
            requested,
        )
    )

    inside_capture = TrajectoryImpactSim.pursuit_closing_speed(
        controller,
        0.25,
        0.0,
        3.0,
    )

    assert inside_capture == pytest.approx(0.0)


def test_terminal_closing_speed_uses_executable_acceleration_limit():
    controller = SimpleNamespace(
        max_acceleration=4.8,
        terminal_max_acceleration=2.0,
        max_actual_horizontal_acceleration=5.0,
        horizontal_acceleration_guard_margin=0.5,
        enable_gazebo_control=True,
        follow_max_closing_speed=3.0,
        impact_radius=0.25,
    )
    controller.limit_horizontal_acceleration = lambda requested: (
        TrajectoryImpactSim.limit_horizontal_acceleration(
            controller,
            requested,
        )
    )

    closing_speed = TrajectoryImpactSim.pursuit_closing_speed(
        controller,
        horizontal_distance=0.5,
        vertical_distance=0.0,
    )

    assert closing_speed == pytest.approx(1.0)


def test_optional_clearance_preserves_disabled_zero():
    assert TrajectoryImpactSim.normalize_optional_clearance(
        0.0,
        0.05,
        3.0,
    ) == pytest.approx(0.0)
    assert TrajectoryImpactSim.normalize_optional_clearance(
        0.01,
        0.05,
        3.0,
    ) == pytest.approx(0.05)


def test_terminal_altitude_ceiling_is_disabled_by_default():
    controller = make_front_view_altitude_controller()

    assert TrajectoryImpactSim.terminal_altitude_ceiling(
        controller,
        0.15,
    ) == math.inf


def test_terminal_altitude_ceiling_keeps_capture_compatible_clearance():
    controller = make_front_view_altitude_controller(
        terminal_minimum_clearance=0.5,
    )
    controller.terminal_descent_committed = True
    controller.terminal_descent_release_distance = 4.0

    reference_z, terminal_pursuit = (
        TrajectoryImpactSim.terminal_fallback_altitude_reference(
            controller,
            1.41,
            0.15,
        )
    )

    # Committed fallback would otherwise jump directly to capture_z=-0.05.
    # A strict 0.5 m clearance would prevent entry into the 0.25 m capture
    # sphere, so the ceiling relaxes to the sphere top at -0.1.
    assert terminal_pursuit is True
    assert reference_z == pytest.approx(-0.1)


def test_altitude_ceiling_does_not_lower_safe_dive_reference():
    controller = make_front_view_altitude_controller(
        terminal_minimum_clearance=0.5,
    )

    reference_z = TrajectoryImpactSim.pursuit_altitude_reference(
        controller,
        1.41,
        0.15,
    )

    assert reference_z == pytest.approx(-1.4550491256520994)


def test_terminal_altitude_ceiling_never_blocks_a_capture():
    controller = make_front_view_altitude_controller(
        terminal_minimum_clearance=0.5,
    )

    # NED grows downward.  The maximum allowed z must be no smaller than the
    # top of the capture sphere, leaving at least one reachable intersection.
    for target_z in (0.5, 0.15, -0.15):
        ceiling_z = TrajectoryImpactSim.terminal_altitude_ceiling(
            controller,
            target_z,
        )
        assert ceiling_z >= target_z - controller.impact_radius


def test_terminal_altitude_ceiling_brakes_descent_but_allows_climb():
    controller = make_front_view_altitude_controller(
        terminal_minimum_clearance=0.5,
    )
    controller.altitude_velocity_gain = 1.0
    controller.max_vertical_speed = 4.0

    # The preferred -0.5 m limit relaxes to -0.1 m to preserve capture.  At
    # that NED z ceiling the dive feed-forward must not descend farther.
    controller.sim_z = -0.1
    braking_vz = TrajectoryImpactSim.pursuit_vertical_velocity(
        controller,
        horizontal_distance=1.41,
        target_z=0.15,
        target_vz=0.0,
        altitude_reference=-0.1,
        horizontal_closing_speed=0.5,
    )
    assert braking_vz == pytest.approx(0.0)

    # Past the ceiling (larger NED z, hence lower altitude), the one-step
    # guard requests a climb back to the limit.
    controller.sim_z = -0.05
    overrun_vz = TrajectoryImpactSim.pursuit_vertical_velocity(
        controller,
        horizontal_distance=1.41,
        target_z=0.15,
        target_vz=0.0,
        altitude_reference=-0.1,
        horizontal_closing_speed=0.5,
    )
    assert overrun_vz == pytest.approx(-1.0)

    # Above the ceiling (smaller NED z, hence higher altitude), descent is
    # still allowed up to the remaining one-step distance.
    controller.sim_z = -0.25
    tracking_vz = TrajectoryImpactSim.pursuit_vertical_velocity(
        controller,
        horizontal_distance=1.41,
        target_z=0.15,
        target_vz=0.0,
        altitude_reference=-0.1,
        horizontal_closing_speed=0.5,
    )
    assert tracking_vz > 0.0


def test_measured_horizontal_closing_speed_uses_relative_velocity():
    controller = SimpleNamespace(
        sim_vx=3.0,
        sim_vy=1.0,
        target_vx=1.0,
        target_vy=2.0,
    )

    closing = TrajectoryImpactSim.measured_horizontal_closing_speed(
        controller,
        1.0,
        0.0,
    )
    opening = TrajectoryImpactSim.measured_horizontal_closing_speed(
        controller,
        0.0,
        1.0,
    )

    assert closing == pytest.approx(2.0)
    assert opening == pytest.approx(-1.0)


def test_retained_terminal_plan_advances_and_expires_with_clock():
    plan = SimpleNamespace(duration=1.0)
    controller = SimpleNamespace(
        terminal_control_lookahead=0.15,
        retained_terminal_plan=None,
        retained_terminal_plan_time_ns=None,
    )

    TrajectoryImpactSim.retain_terminal_trajectory_plan(
        controller,
        plan,
        timestamp_ns=1_000_000_000,
    )
    retained = TrajectoryImpactSim.retained_terminal_trajectory_sample(
        controller,
        timestamp_ns=1_100_000_000,
    )

    assert retained[0] is plan
    assert retained[1] == pytest.approx(0.25)
    assert retained[2] == pytest.approx(0.9)

    expired = TrajectoryImpactSim.retained_terminal_trajectory_sample(
        controller,
        timestamp_ns=2_000_000_000,
    )

    assert expired is None
    assert controller.retained_terminal_plan is None
    assert controller.retained_terminal_plan_time_ns is None


def test_retained_terminal_plan_obeys_short_hold_limit():
    controller = SimpleNamespace(
        terminal_control_lookahead=0.15,
        terminal_plan_max_hold_time=0.2,
        retained_terminal_plan=SimpleNamespace(duration=2.0),
        retained_terminal_plan_time_ns=1_000_000_000,
    )

    retained = TrajectoryImpactSim.retained_terminal_trajectory_sample(
        controller,
        timestamp_ns=1_200_000_000,
    )
    expired = TrajectoryImpactSim.retained_terminal_trajectory_sample(
        controller,
        timestamp_ns=1_201_000_000,
    )

    assert retained is not None
    assert expired is None


def test_retained_plan_is_rejected_after_target_endpoint_moves():
    controller = SimpleNamespace(
        terminal_plan_target_error_limit=0.5,
        target_planning_state=lambda x, y, z, horizon: (
            ((x + horizon, y + 1.0, z), (1.0, 0.0, 0.0), (0.0,) * 3)
        ),
    )
    plan = SimpleNamespace(target_position=(2.0, 0.0, -0.1))

    matches = TrajectoryImpactSim.retained_plan_matches_target(
        controller,
        plan,
        1.0,
        1.0,
        0.0,
        0.0,
    )

    assert matches is False


def test_terminal_trajectory_velocity_closes_position_tracking_error():
    controller = SimpleNamespace(
        sim_x=0.0,
        sim_y=0.0,
        sim_z=-1.0,
        sim_vz=0.0,
        dt=0.05,
        terminal_trajectory_position_gain=1.0,
        terminal_trajectory_vertical_gain=1.0,
        terminal_contact_clearance=0.05,
        terminal_minimum_clearance=0.1,
        terminal_safety_margin=0.02,
        terminal_safety_response_time=0.15,
        terminal_max_vertical_acceleration=3.0,
        impact_radius=0.25,
        sea_surface_z=0.0,
        max_vertical_speed=4.0,
        clamp_command_speed=lambda vx, vy: (vx, vy),
    )
    sample = SimpleNamespace(
        position=(1.0, -1.0, -0.8),
        velocity=(2.0, 1.0, 0.5),
    )

    desired = TrajectoryImpactSim.terminal_trajectory_velocity(
        controller,
        sample,
        target_z=0.0,
    )

    assert desired == pytest.approx((3.0, 0.0, 0.7))


def test_terminal_safety_barrier_accounts_for_descent_response_distance():
    controller = SimpleNamespace(
        sim_z=-0.3,
        sim_vz=2.0,
        dt=0.05,
        terminal_contact_clearance=0.05,
        terminal_minimum_clearance=0.1,
        terminal_safety_margin=0.02,
        terminal_safety_response_time=0.15,
        terminal_max_vertical_acceleration=3.0,
        impact_radius=0.25,
        sea_surface_z=0.0,
    )

    desired_vz = TrajectoryImpactSim.terminal_safe_vertical_velocity(
        controller,
        3.0,
        target_z=0.15,
    )

    assert desired_vz <= 0.0


def test_control_timing_uses_measured_callback_interval():
    controller = SimpleNamespace(
        dt=0.05,
        control_dt=0.05,
        last_control_callback_time_ns=None,
    )

    first = TrajectoryImpactSim.update_control_timing(
        controller,
        1_000_000_000,
    )
    delayed = TrajectoryImpactSim.update_control_timing(
        controller,
        1_120_000_000,
    )

    assert first == pytest.approx(0.05)
    assert delayed == pytest.approx(0.12)


def test_terminal_pursuit_keeps_committed_capture_descent():
    controller = make_front_view_altitude_controller()
    controller.terminal_descent_committed = True
    controller.terminal_descent_release_distance = 4.0

    reference_z, committed = (
        TrajectoryImpactSim.terminal_fallback_altitude_reference(
            controller,
            1.0,
            -0.13,
        )
    )

    assert committed is True
    assert reference_z == pytest.approx(-0.18)
    assert controller.terminal_descent_committed is True


def test_terminal_pursuit_caps_descent_above_sea_surface():
    controller = make_front_view_altitude_controller()
    controller.terminal_descent_committed = True
    controller.terminal_descent_release_distance = 4.0

    reference_z, committed = (
        TrajectoryImpactSim.terminal_fallback_altitude_reference(
            controller,
            0.1,
            0.15,
        )
    )

    assert committed is True
    assert reference_z == pytest.approx(-0.05)


def test_terminal_pursuit_releases_after_distance_hysteresis():
    controller = make_front_view_altitude_controller()
    controller.terminal_descent_committed = True
    controller.terminal_descent_release_distance = 4.0

    reference_z, committed = (
        TrajectoryImpactSim.terminal_fallback_altitude_reference(
            controller,
            4.1,
            0.0,
        )
    )
    pursuit_z = TrajectoryImpactSim.pursuit_altitude_reference(
        controller,
        4.1,
        0.0,
    )

    assert committed is False
    assert reference_z == pytest.approx(pursuit_z)
    assert controller.terminal_descent_committed is False


def test_plan_velocity_uses_retained_plan_during_replan_dropout():
    sample_times = []
    sample = SimpleNamespace(
        position=(1.0, 2.0, -0.2),
        velocity=(3.0, 4.0, 0.5),
        acceleration=(0.1, 0.2, 0.3),
    )
    plan = SimpleNamespace(
        duration=1.0,
        planner_type='MINCO_T3',
        target_curve_weight=0.7,
        closing_speed=0.6,
        maximum_horizontal_speed=6.0,
        maximum_vertical_speed=1.0,
        maximum_horizontal_acceleration=3.0,
        maximum_vertical_acceleration=1.5,
        target_position=(5.0, 6.0, -0.1),
        sample=lambda time: sample_times.append(time) or sample,
    )
    controller = SimpleNamespace(
        sim_x=0.0,
        sim_y=0.0,
        sim_z=-0.7,
        terminal_radius=3.0,
        terminal_control_lookahead=0.15,
        retained_terminal_plan=plan,
        retained_terminal_plan_time_ns=1_000_000_000,
        terminal_descent_committed=False,
        terminal_trajectory_plan=lambda x, y, z: None,
        continuous_intercept_solution=lambda x, y, z: (
            x,
            y,
            z,
            1.0,
        ),
    )
    controller.retained_terminal_trajectory_sample = lambda: (
        TrajectoryImpactSim.retained_terminal_trajectory_sample(
            controller,
            timestamp_ns=1_100_000_000,
        )
    )

    command = TrajectoryImpactSim.plan_velocity(
        controller,
        1.0,
        0.0,
        0.0,
    )

    assert sample_times == pytest.approx([0.25])
    assert command[:7] == pytest.approx((
        3.0,
        4.0,
        0.5,
        5.0,
        6.0,
        -0.1,
        0.9,
    ))
    assert command[7] is True
    assert controller.trajectory_planner_type == 'MINCO_T3_HOLD'
    assert controller.terminal_descent_committed is True


def test_pursuit_keeps_forward_closure_inside_terminal_planning_radius():
    controller = SimpleNamespace(
        max_acceleration=4.8,
        terminal_max_acceleration=3.0,
        max_actual_horizontal_acceleration=5.0,
        horizontal_acceleration_guard_margin=0.5,
        enable_gazebo_control=True,
        follow_max_closing_speed=3.0,
        impact_radius=0.25,
    )
    controller.limit_horizontal_acceleration = lambda requested: (
        TrajectoryImpactSim.limit_horizontal_acceleration(
            controller,
            requested,
        )
    )

    closing_speed = TrajectoryImpactSim.pursuit_closing_speed(
        controller,
        3.0,
        1.0,
    )

    assert closing_speed == pytest.approx(3.0)


def test_pursuit_does_not_stop_horizontally_before_vertical_capture():
    controller = SimpleNamespace(
        max_acceleration=4.8,
        terminal_max_acceleration=3.0,
        max_actual_horizontal_acceleration=5.0,
        horizontal_acceleration_guard_margin=0.5,
        enable_gazebo_control=True,
        follow_max_closing_speed=3.0,
        impact_radius=0.25,
    )
    controller.limit_horizontal_acceleration = lambda requested: (
        TrajectoryImpactSim.limit_horizontal_acceleration(
            controller,
            requested,
        )
    )

    closing_speed = TrajectoryImpactSim.pursuit_closing_speed(
        controller,
        0.25,
        1.0,
    )

    assert closing_speed > 0.0


def test_target_visual_height_offset_raises_sphere_without_changing_truth():
    update_calls = []
    target = SimpleNamespace(
        gazebo_visualizer=SimpleNamespace(
            update=lambda x, y, z: update_calls.append((x, y, z)) or True,
        ),
        gazebo_visual_height_offset=0.42,
        x=20.0,
        y=3.0,
        z=0.15,
        gazebo_visualization_ready=True,
        last_gazebo_warning_time=-math.inf,
    )

    MovingTarget.update_gazebo_visualization(target)

    assert update_calls == [(20.0, 3.0, -0.27)]
    assert target.z == pytest.approx(0.15)


def make_speed_governor(actual_speed):
    controller = SimpleNamespace(
        command_speed_limit=7.4,
        speed_governor_gain=1.0,
        sim_vx=actual_speed,
        sim_vy=0.0,
    )
    controller.clamp_command_speed = lambda vx, vy: (
        TrajectoryImpactSim.clamp_command_speed(controller, vx, vy)
    )
    return controller


def test_speed_governor_reduces_forward_demand_above_soft_cap():
    controller = make_speed_governor(7.8)

    vx, vy = TrajectoryImpactSim.speed_governed_velocity(
        controller,
        8.0,
        0.0,
    )

    assert vx == pytest.approx(7.0)
    assert vy == pytest.approx(0.0)


def test_speed_governor_keeps_command_when_already_decelerating():
    controller = make_speed_governor(7.8)

    vx, vy = TrajectoryImpactSim.speed_governed_velocity(
        controller,
        6.8,
        0.0,
    )

    assert vx == pytest.approx(6.8)
    assert vy == pytest.approx(0.0)


def test_speed_governor_uses_static_cap_below_activation_speed():
    controller = make_speed_governor(7.2)

    vx, vy = TrajectoryImpactSim.speed_governed_velocity(
        controller,
        8.0,
        0.0,
    )

    assert vx == pytest.approx(7.4)
    assert vy == pytest.approx(0.0)


def test_gazebo_acceleration_limit_reserves_hard_limit_margin():
    controller = SimpleNamespace(
        max_acceleration=4.8,
        max_actual_horizontal_acceleration=5.0,
        horizontal_acceleration_guard_margin=0.5,
        enable_gazebo_control=True,
    )

    limit = TrajectoryImpactSim.limit_horizontal_acceleration(
        controller,
        4.8,
    )

    assert limit == pytest.approx(4.5)


def test_virtual_acceleration_limit_does_not_apply_px4_margin():
    controller = SimpleNamespace(
        max_acceleration=4.8,
        max_actual_horizontal_acceleration=5.0,
        horizontal_acceleration_guard_margin=0.5,
        enable_gazebo_control=False,
    )

    limit = TrajectoryImpactSim.limit_horizontal_acceleration(
        controller,
        4.8,
    )

    assert limit == pytest.approx(4.8)


def test_closest_relative_vector_finds_point_between_samples():
    closest = TrajectoryImpactSim.closest_relative_vector(
        (1.0, 0.2, 0.0),
        (-1.0, 0.2, 0.0),
    )

    assert closest == pytest.approx((0.0, 0.2, 0.0))


def test_impact_fraction_detects_capture_between_samples():
    evaluator = SimpleNamespace(impact_radius=0.25)

    fraction = TrajectoryImpactSim.impact_fraction(
        evaluator,
        (1.0, 0.0, 0.0),
        (-1.0, 0.0, 0.0),
    )

    assert fraction == pytest.approx(0.375)


def test_impact_fraction_rejects_near_miss():
    evaluator = SimpleNamespace(impact_radius=0.25)

    fraction = TrajectoryImpactSim.impact_fraction(
        evaluator,
        (1.0, 0.3, 0.0),
        (-1.0, 0.3, 0.0),
    )

    assert fraction is None


def test_sea_contact_fraction_uses_ned_down_positive_axis():
    evaluator = SimpleNamespace(
        enable_sea_contact_failure=True,
        sea_surface_z=0.0,
    )

    fraction = TrajectoryImpactSim.sea_contact_fraction(
        evaluator,
        -0.2,
        0.2,
    )

    assert fraction == pytest.approx(0.5)


def test_terminal_event_prefers_capture_before_sea_contact():
    evaluator = SimpleNamespace(
        impact_radius=0.25,
        enable_sea_contact_failure=True,
        sea_surface_z=0.0,
    )

    reason, fraction = TrajectoryImpactSim.terminal_event(
        evaluator,
        (1.0, 0.0, 0.0),
        (-1.0, 0.0, 0.0),
        -1.0,
        1.0,
    )

    assert reason == 'CAPTURE_RADIUS_REACHED'
    assert fraction == pytest.approx(0.375)


def test_terminal_event_returns_sea_contact_for_missed_target():
    evaluator = SimpleNamespace(
        impact_radius=0.25,
        enable_sea_contact_failure=True,
        sea_surface_z=0.0,
    )

    reason, fraction = TrajectoryImpactSim.terminal_event(
        evaluator,
        (1.0, 0.3, 0.0),
        (-1.0, 0.3, 0.0),
        -1.0,
        1.0,
    )

    assert reason == 'SEA_CONTACT'
    assert fraction == pytest.approx(0.5)


def test_failure_result_freezes_target_and_requests_gazebo_pause():
    pause_requests = []
    evaluator = SimpleNamespace(
        gazebo_world_paused=False,
        hit=False,
        pause_gazebo_on_hit=True,
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
        pause_gazebo_world=lambda: pause_requests.append(True),
    )

    MovingTarget.result_callback(
        evaluator,
        SimpleNamespace(outcome='FAILURE'),
    )

    assert evaluator.hit is True
    assert pause_requests == [True]


def test_moving_target_rejects_x_until_flight_is_ready():
    messages = []
    target = SimpleNamespace(
        started=False,
        flight_ready=False,
        get_logger=lambda: SimpleNamespace(
            warn=messages.append,
            info=messages.append,
        ),
    )

    MovingTarget.command_callback(target, SimpleNamespace(data='X'))
    assert target.started is False

    target.flight_ready = True
    MovingTarget.command_callback(target, SimpleNamespace(data='X'))
    assert target.started is True


def test_x_requests_arming_only_after_operator_command():
    commands = []
    controller = SimpleNamespace(
        enable_gazebo_control=True,
        completed=False,
        takeoff_requested=False,
        flight_ready=True,
        log_start_time_ns=None,
        begin_csv_logging=lambda: None,
        publish_vehicle_command=lambda command, param1, param2=0.0: (
            commands.append((command, param1, param2))
        ),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)
        ),
        get_logger=lambda: SimpleNamespace(
            info=lambda message: None,
            warn=lambda message: None,
        ),
    )

    TrajectoryImpactSim.command_callback(
        controller,
        SimpleNamespace(data='X'),
    )

    assert controller.takeoff_requested is True
    assert commands == [(
        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
        1.0,
        0.0,
    )]


def test_ground_preparation_holds_position_and_requests_offboard():
    actions = []
    controller = SimpleNamespace(
        takeoff_x=1.0,
        takeoff_y=2.0,
        takeoff_z=-0.1,
        preflight_counter=39,
        offboard_prestream_cycles=40,
        px4_command_retry_cycles=20,
        offboard_active=False,
        vehicle_armed=False,
        vehicle_status_time_ns=None,
        vehicle_status_timeout=2.0,
        publish_offboard_mode=lambda: actions.append('stream'),
        publish_gazebo_setpoint=lambda x, y, z, **kwargs: actions.append(
            ('hold', x, y, z, kwargs)
        ),
        publish_vehicle_command=lambda command, param1, param2=0.0: (
            actions.append(('command', command, param1, param2))
        ),
        publish_flight_ready=lambda ready: actions.append(
            ('ready', ready)
        ),
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)
        ),
    )

    TrajectoryImpactSim.prepare_flight_on_ground(controller)

    assert actions[0] == 'stream'
    assert actions[1] == (
        'hold',
        1.0,
        2.0,
        -0.1,
        {'track_target_yaw': False},
    )
    assert actions[2][0] == 'command'
    assert actions[-1] == ('ready', False)


def test_ground_preparation_reports_ready_only_after_px4_confirmation():
    readiness = []
    controller = SimpleNamespace(
        takeoff_x=0.0,
        takeoff_y=0.0,
        takeoff_z=0.0,
        preflight_counter=100,
        offboard_prestream_cycles=40,
        px4_command_retry_cycles=20,
        offboard_active=True,
        vehicle_armed=False,
        vehicle_status_time_ns=900_000_000,
        vehicle_status_timeout=2.0,
        publish_offboard_mode=lambda: None,
        publish_gazebo_setpoint=lambda x, y, z, **kwargs: None,
        publish_vehicle_command=lambda command, param1, param2=0.0: None,
        publish_flight_ready=readiness.append,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)
        ),
    )

    TrajectoryImpactSim.prepare_flight_on_ground(controller)

    assert readiness == [True]


def test_ground_preparation_disarms_unexpected_pre_x_arming():
    commands = []
    controller = SimpleNamespace(
        takeoff_x=0.0,
        takeoff_y=0.0,
        takeoff_z=0.0,
        preflight_counter=59,
        offboard_prestream_cycles=40,
        px4_command_retry_cycles=20,
        offboard_active=True,
        vehicle_armed=True,
        vehicle_status_time_ns=900_000_000,
        vehicle_status_timeout=2.0,
        publish_offboard_mode=lambda: None,
        publish_gazebo_setpoint=lambda x, y, z, **kwargs: None,
        publish_vehicle_command=lambda command, param1, param2=0.0: (
            commands.append((command, param1, param2))
        ),
        publish_flight_ready=lambda ready: None,
        get_logger=lambda: SimpleNamespace(
            info=lambda message: None,
            warn=lambda message: None,
        ),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)
        ),
    )

    TrajectoryImpactSim.prepare_flight_on_ground(controller)

    assert commands == [(
        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
        0.0,
        0.0,
    )]


def make_constraint_monitor(horizontal_acceleration):
    evaluator = SimpleNamespace(
        initial_uav_vx=0.0,
        initial_uav_vy=0.0,
        initial_uav_vz=0.0,
        measured_acceleration=horizontal_acceleration,
        measured_vertical_acceleration=0.0,
        max_speed=8.0,
        max_vertical_speed=2.0,
        max_acceleration=5.0,
        max_vertical_acceleration=2.0,
        terminal_max_acceleration=2.0,
        terminal_max_vertical_acceleration=1.0,
        max_actual_horizontal_acceleration=5.0,
        max_actual_vertical_acceleration=3.0,
        terminal_mode_active=True,
        started=True,
        max_observed_horizontal_speed=0.0,
        max_observed_vertical_speed=0.0,
        max_observed_horizontal_acceleration=0.0,
        max_observed_vertical_acceleration=0.0,
        constraint_violation_cycles=0,
        constraint_violation_cycles_limit=5,
        active_constraint_violations=[],
        sim_time=3.0,
        last_constraint_warning_time=-float('inf'),
        dt=0.05,
        failure_detail='',
        get_logger=lambda: SimpleNamespace(warn=lambda message: None),
    )
    evaluator.horizontal_acceleration_guard_margin = 0.5
    evaluator.enable_gazebo_control = True
    evaluator.limit_horizontal_acceleration = lambda requested: (
        TrajectoryImpactSim.limit_horizontal_acceleration(
            evaluator,
            requested,
        )
    )
    evaluator.command_acceleration_limits = lambda terminal_mode: (
        TrajectoryImpactSim.command_acceleration_limits(
            evaluator,
            terminal_mode,
        )
    )
    return evaluator


def test_terminal_command_uses_its_configured_acceleration_limits():
    evaluator = make_constraint_monitor(0.0)

    terminal_limits = TrajectoryImpactSim.command_acceleration_limits(
        evaluator,
        True,
    )
    pursuit_limits = TrajectoryImpactSim.command_acceleration_limits(
        evaluator,
        False,
    )

    assert terminal_limits == pytest.approx((2.0, 1.0))
    assert pursuit_limits == pytest.approx((4.5, 2.0))


def test_terminal_tracking_response_is_not_a_hard_violation():
    evaluator = make_constraint_monitor(2.84)

    failed = TrajectoryImpactSim.monitor_actual_constraints(evaluator)

    assert failed is False
    assert evaluator.constraint_violation_cycles == 0
    assert evaluator.active_constraint_violations == []


def test_persistent_actual_acceleration_hard_violation_fails():
    evaluator = make_constraint_monitor(5.6)

    results = [
        TrajectoryImpactSim.monitor_actual_constraints(evaluator)
        for _ in range(5)
    ]

    assert results == [False, False, False, False, True]
    assert evaluator.active_constraint_violations == [
        'horizontal_acceleration'
    ]


def test_path_reference_uses_arc_length_instead_of_tangent_offset():
    path = deque([
        (0.0, 0.0, 0.0),
        (10.0, 10.0, 0.0),
        (20.0, 10.0, 10.0),
        (30.0, 10.0, 20.0),
    ])

    reference = TrajectoryImpactSim.sample_path_reference(
        path,
        15.0,
        5.0,
    )

    assert reference == pytest.approx((10.0, 5.0, 0.0, 5.0))


def test_incomplete_path_history_holds_oldest_reference():
    evaluator = SimpleNamespace(
        target_vx=5.0,
        target_vy=0.0,
        target_path_history=deque([
            (0.0, 0.0, 0.0),
            (10.0, 10.0, 0.0),
        ]),
        target_path_distance=10.0,
        follow_distance=20.0,
    )

    reference = TrajectoryImpactSim.calculate_follow_reference(
        evaluator,
        10.0,
        0.0,
    )

    assert reference == pytest.approx((0.0, 0.0, 0.0, 0.0))
