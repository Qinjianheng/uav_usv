"""Isolated P7.2b axis validation, with one PX4 command owner."""

from datetime import datetime
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess


def generate_launch_description():
    workspace = Path(os.environ.get('UAV_USV_WS', '/home/qin/data/uav_usv'))
    session = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output = (workspace / 'data/experiments/current/p7_axis_20260929'
              / session / 'axis_capture.jsonl')
    profile = os.environ.get('P7_AXIS_PROFILE', 'full')
    if profile not in ('full', 'yaw_offset'):
        raise ValueError('invalid P7_AXIS_PROFILE')
    print(f'P7 AXIS capture: {output}; profile={profile}')
    return LaunchDescription([
        # The standard launcher always forwards this argument.
        DeclareLaunchArgument('enable_shadow_perception',
                              default_value='false'),
        ExecuteProcess(
            cmd=['python3', str(
                workspace / 'scripts' / 'p7_px4_local_axis_capture.py'),
                 '--output', str(output)],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['python3', str(
                workspace / 'scripts' / 'p7_px4_local_axis_motion.py'),
                 '--profile', profile],
            output='screen',
        ),
    ])
