"""P7.3 X-only FOLLOW capture: current RGB-D and KF in one run."""

from datetime import datetime
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
import yaml


SCENARIO_SPEED = {'static': 0.0, 'plus_y': 4.0, 'minus_y': -4.0}


def generate_launch_description():
    workspace = Path(os.environ.get('UAV_USV_WS', '/home/qin/data/uav_usv'))
    scenario = os.environ.get('P7_KF_SCENARIO', 'static')
    if scenario not in SCENARIO_SPEED:
        raise ValueError(
            f'P7_KF_SCENARIO must be one of {tuple(SCENARIO_SPEED)}')
    session = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output = (workspace / 'data/experiments/current/p7_kf_20260929'
              / scenario / session)
    output.mkdir(parents=True, exist_ok=False)
    baseline = (workspace / 'src/uav_usv_bringup/config/baseline.yaml')
    config = yaml.safe_load(baseline.read_text(encoding='utf-8'))
    target = config['moving_target']['ros__parameters']
    target.update(velocity_y=SCENARIO_SPEED[scenario],
                  trajectory_type='linear', horizontal_speed=4.0,
                  horizontal_acceleration_limit=4.0,
                  vertical_oscillation_amplitude=0.0)
    config['rgbd_target_localizer']['ros__parameters'][
        'geometry_diagnostics_enabled'] = True
    config['intercept_evaluator_node']['ros__parameters'][
        'gazebo_entity_diagnostics_enabled'] = True
    config_path = output / 'config.yaml'
    with config_path.open('x', encoding='utf-8') as stream:
        yaml.safe_dump(config, stream, sort_keys=False)
    print(f'P7.3 scenario={scenario} output={output}')
    modular = (workspace / 'src/uav_usv_bringup/launch'
               / 'modular_intercept.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('enable_shadow_perception',
                              default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(modular)),
            launch_arguments={
                'config_file': str(config_path),
                'log_directory': str(output),
                'enable_shadow_perception': 'true',
            }.items(),
        ),
        ExecuteProcess(
            cmd=['python3', str(workspace / 'scripts'
                                / 'vision_p6_dynamic_capture.py'),
                 '--seconds', '100', '--output-directory', str(output)],
            output='screen',
        ),
    ])
