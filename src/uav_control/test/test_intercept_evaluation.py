import math
from collections import deque
from types import SimpleNamespace

import pytest

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


def test_takeoff_follow_setpoint_combines_xy_velocity_and_z_position():
    published = []
    controller = SimpleNamespace(
        setpoint_pub=SimpleNamespace(publish=published.append),
        timestamp=lambda: 123,
        flight_altitude=-5.0,
        update_observation_yaw=lambda: 0.75,
    )

    TrajectoryImpactSim.publish_takeoff_follow_setpoint(
        controller,
        3.0,
        4.0,
    )

    assert len(published) == 1
    message = published[0]
    assert math.isnan(message.position[0])
    assert math.isnan(message.position[1])
    assert message.position[2] == pytest.approx(-5.0)
    assert message.velocity[:2] == pytest.approx([3.0, 4.0])
    assert math.isnan(message.velocity[2])
    assert message.yaw == pytest.approx(0.75)


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


def test_ground_preparation_holds_position_and_requests_offboard():
    actions = []
    controller = SimpleNamespace(
        takeoff_x=1.0,
        takeoff_y=2.0,
        takeoff_z=-0.1,
        preflight_counter=39,
        offboard_prestream_cycles=40,
        px4_command_retry_cycles=20,
        arm_request_start_cycle=60,
        offboard_active=False,
        vehicle_armed=False,
        vehicle_status_time_ns=None,
        vehicle_status_timeout=2.0,
        publish_offboard_mode=lambda: actions.append('stream'),
        publish_gazebo_setpoint=lambda x, y, z: actions.append(
            ('hold', x, y, z)
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
    assert actions[1] == ('hold', 1.0, 2.0, -0.1)
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
        arm_request_start_cycle=60,
        offboard_active=True,
        vehicle_armed=True,
        vehicle_status_time_ns=900_000_000,
        vehicle_status_timeout=2.0,
        publish_offboard_mode=lambda: None,
        publish_gazebo_setpoint=lambda x, y, z: None,
        publish_vehicle_command=lambda command, param1, param2=0.0: None,
        publish_flight_ready=readiness.append,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)
        ),
    )

    TrajectoryImpactSim.prepare_flight_on_ground(controller)

    assert readiness == [True]


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
    return evaluator


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
