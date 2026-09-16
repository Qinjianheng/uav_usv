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

    for name in (
        'maximum_horizontal_speed',
        'maximum_vertical_speed',
        'maximum_horizontal_acceleration',
        'maximum_vertical_acceleration',
    ):
        assert tracker[name] == pytest.approx(planner[name])
    assert tracker['sea_surface_z'] == pytest.approx(planner['sea_surface_z'])
    assert evaluator['sea_surface_z'] == pytest.approx(planner['sea_surface_z'])
    assert evaluator['evaluation_capture_radius'] == pytest.approx(0.50)
    assert planner['planned_capture_radius'] == pytest.approx(0.35)
    assert evaluator['planned_capture_radius'] == pytest.approx(
        planner['planned_capture_radius']
    )


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
