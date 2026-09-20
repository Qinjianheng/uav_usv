from pathlib import Path

import pytest
import yaml


CONFIG_FILE = (
    Path(__file__).parents[2]
    / 'uav_usv_bringup'
    / 'config'
    / 'baseline.yaml'
)


def parameters(config, node):
    return config[node]['ros__parameters']


def test_shared_dynamic_and_safety_limits_are_synchronized():
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding='utf-8'))
    planner = parameters(config, 'intercept_planner_node')
    tracker = parameters(config, 'trajectory_tracker_node')
    evaluator = parameters(config, 'intercept_evaluator_node')

    assert planner['maximum_horizontal_speed'] == pytest.approx(7.0)
    assert tracker['maximum_horizontal_speed'] == pytest.approx(6.5)
    assert tracker['guidance_maximum_horizontal_speed'] == pytest.approx(6.2)
    assert (
        tracker['maximum_horizontal_speed']
        < planner['maximum_horizontal_speed']
    )

    for name in (
        'maximum_vertical_speed',
        'maximum_horizontal_acceleration',
        'maximum_vertical_acceleration',
    ):
        assert tracker[name] == pytest.approx(planner[name])
    assert tracker['sea_surface_z'] == pytest.approx(planner['sea_surface_z'])
    assert evaluator['sea_surface_z'] == pytest.approx(planner['sea_surface_z'])
    assert evaluator['evaluation_capture_radius'] == pytest.approx(0.50)
    assert planner['planned_capture_radius'] == pytest.approx(0.50)
    assert evaluator['planned_capture_radius'] == pytest.approx(
        planner['planned_capture_radius']
    )


def test_planned_contact_point_stays_above_the_body_contact_threshold():
    """
    Guard the invariant that produced the 2026-09-20 sea-contact failures.

    The planner aims at ``sea_surface_z - preferred_clearance`` while the
    evaluator fails the run as soon as the airframe reaches the water, which
    happens ``body_lower_extent`` below the PX4 reference.  If the planned
    endpoint sits at or below that threshold, flying the plan to completion
    always ends in SEA_CONTACT before capture.
    """
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding='utf-8'))
    planner = parameters(config, 'intercept_planner_node')
    evaluator = parameters(config, 'intercept_evaluator_node')

    planned_contact_z = planner['sea_surface_z'] - planner['preferred_clearance']
    body_contact_z = evaluator['sea_surface_z'] - evaluator['body_lower_extent']

    assert planned_contact_z < body_contact_z
    # The validated ceiling must sit on the hard floor, strictly below the
    # contact altitude, or the planned endpoint rests exactly on the validated
    # boundary and millimetre sag is reported as a clearance violation.
    assert planner['contact_clearance'] < planner['preferred_clearance']
    # The aim sphere must still contain a watered-off contact at the largest
    # downward target heave.
    heave = parameters(config, 'moving_target')[
        'vertical_oscillation_amplitude'
    ]
    assert (
        planner['preferred_clearance'] + heave
        <= planner['planned_capture_radius'] + 1e-9
    )


def test_tracking_velocity_mode_stays_off_until_the_reference_leads():
    """
    Guard the mode that made the 2026-09-20 intercept worse.

    Velocity-driven tracking is only viable once the plan stops re-anchoring
    its initial state to the measurement on every replan; with the current
    planner it executes a ~4 m/s reference and the vehicle falls behind.
    """
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding='utf-8'))
    tracker = parameters(config, 'trajectory_tracker_node')

    assert tracker['use_velocity_control'] is False


def test_sea_barrier_reserve_covers_the_airframe():
    """Guard the barrier that reported SAFE while the gear was at the water."""
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding='utf-8'))
    tracker = parameters(config, 'trajectory_tracker_node')
    planner = parameters(config, 'intercept_planner_node')
    evaluator = parameters(config, 'intercept_evaluator_node')

    reserve = tracker['reserve_clearance']
    body = evaluator['body_lower_extent']

    # The planned contact must stay reachable, so the barrier sits below it.
    assert reserve < planner['preferred_clearance']
    # A token reserve far below the airframe extent lets the vehicle fly with
    # its gear level with the water while the guard still reports SAFE.
    assert reserve >= 0.6 * body


def test_curve_weight_stays_inside_the_lateral_acceleration_budget():
    """Guard the 2026-09-20 jittering first-descent regression."""
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding='utf-8'))
    planner = parameters(config, 'intercept_planner_node')

    # The figure-eight target turns hard enough that blending the MINCO
    # waypoints toward its curved path breaks the 3.0 m/s^2 planning limit
    # from about 0.2 upward, which rejected every terminal plan while the
    # vehicle hovered near the sea.
    assert planner['target_curve_weight'] <= 0.1


def test_shadow_perception_rates_are_below_control_rate():
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding='utf-8'))
    tracker = parameters(config, 'trajectory_tracker_node')
    localizer = parameters(config, 'rgbd_target_localizer')
    front = parameters(config, 'front_tof_monitor')

    assert tracker['control_rate_hz'] == pytest.approx(20.0)
    assert localizer['localization_rate_hz'] == pytest.approx(10.0)
    assert front['analysis_rate_hz'] == pytest.approx(5.0)


def test_terminal_planning_uses_fast_rate_and_short_freeze_window():
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding='utf-8'))
    planner = parameters(config, 'intercept_planner_node')

    assert planner['planning_rate_hz'] == pytest.approx(5.0)
    assert planner['terminal_planning_rate_hz'] == pytest.approx(10.0)
    assert planner['minimum_duration'] == pytest.approx(1.0)
    assert planner['terminal_minimum_duration'] == pytest.approx(0.30)
    assert planner['terminal_freeze_time'] == pytest.approx(0.30)


def test_approach_preparation_uses_existing_horizon_and_safety_limits():
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding='utf-8'))
    planner = parameters(config, 'intercept_planner_node')
    tracker = parameters(config, 'trajectory_tracker_node')
    evaluator = parameters(config, 'intercept_evaluator_node')

    assert tracker['approach_horizon'] == pytest.approx(
        planner['maximum_duration']
    )
    assert tracker['approach_contact_clearance'] == pytest.approx(
        planner['preferred_clearance']
    )
    assert tracker['approach_closing_speed'] == pytest.approx(
        planner['preferred_closing_speed']
    )
    assert planner['approach_reserve_clearance'] == pytest.approx(
        tracker['reserve_clearance']
    )
    assert evaluator['detailed_diagnostics_enabled'] is False


def test_plan_age_bound_matches_four_mps_endpoint_tolerance():
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding='utf-8'))
    target = parameters(config, 'moving_target')
    planner = parameters(config, 'intercept_planner_node')
    tracker = parameters(config, 'trajectory_tracker_node')

    assert target['horizontal_speed'] == pytest.approx(4.0)
    derived_maximum_age = (
        planner['endpoint_tolerance'] / target['horizontal_speed']
    )
    assert planner['maximum_input_age'] <= derived_maximum_age
    assert tracker['maximum_plan_age'] <= derived_maximum_age


def test_truth_control_source_is_explicit_and_shared_by_launch_parameters():
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding='utf-8'))
    predictor = parameters(config, 'target_predictor_node')
    tracker = parameters(config, 'trajectory_tracker_node')
    evaluator = parameters(config, 'intercept_evaluator_node')

    assert predictor['target_state_source'] == 'simulation_truth'
    assert predictor['simulation_truth_topic'] == tracker['target_state_topic']
    assert evaluator['truth_topic'] == tracker['target_state_topic']


def test_prediction_horizon_covers_vertical_intercept_duration():
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding='utf-8'))
    legacy = parameters(config, 'trajectory_impact_sim')
    predictor = parameters(config, 'target_predictor_node')
    planner = parameters(config, 'intercept_planner_node')

    # A five-metre, acceleration-limited descent with a near-zero terminal
    # vertical speed needs more than the old three-second hard cap.
    assert planner['maximum_duration'] >= 3.5
    assert predictor['prediction_horizon'] >= planner['maximum_duration']
    assert legacy['terminal_plan_absolute_max_duration'] == pytest.approx(
        planner['maximum_duration']
    )


def test_shadow_camera_prediction_chain_is_isolated_from_truth_control():
    config = yaml.safe_load(CONFIG_FILE.read_text(encoding='utf-8'))

    control = parameters(config, 'target_predictor_node')
    shadow = parameters(config, 'shadow_target_predictor_node')
    kalman = parameters(config, 'target_kalman_filter')
    localizer = parameters(config, 'rgbd_target_localizer')
    evaluator = parameters(config, 'intercept_evaluator_node')

    assert control['target_state_source'] == 'simulation_truth'
    assert control['prediction_topic'] == '/planning/target_prediction'

    assert shadow['target_state_source'] == 'tracking'
    assert shadow['tracking_topic'] == kalman['state_topic']
    assert shadow['prediction_topic'] != control['prediction_topic']

    assert kalman['input_topic'] == localizer['observation_topic']

    assert (
        evaluator['shadow_prediction_topic']
        == shadow['prediction_topic']
    )
