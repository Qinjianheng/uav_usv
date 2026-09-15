from pathlib import Path


LAUNCH_FILE = (
    Path(__file__).parents[2]
    / 'uav_usv_bringup'
    / 'launch'
    / 'modular_intercept.launch.py'
)


def test_modular_launch_contains_five_pipeline_processes():
    source = LAUNCH_FILE.read_text(encoding='utf-8')

    for executable in (
        'target_predictor_node',
        'intercept_planner_node',
        'trajectory_tracker_node',
        'mission_manager_node',
        'intercept_evaluator_node',
    ):
        assert source.count(f"executable='{executable}'") == 1


def test_modular_launch_has_only_new_tracker_as_px4_control_owner():
    source = LAUNCH_FILE.read_text(encoding='utf-8')

    assert "executable='trajectory_impact_sim'" not in source
    assert "executable='predictive_intercept'" not in source
    assert "executable='pure_pursuit'" not in source
