import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('uav_usv_bringup')
    default_config = os.path.join(
        package_share,
        'config',
        'baseline.yaml',
    )
    default_log_directory = os.path.join(
        os.environ.get('UAV_USV_WS', '/home/qin/data/uav_usv'),
        'data',
        'experiments',
        'current',
    )

    config_file = LaunchConfiguration('config_file')
    log_directory = LaunchConfiguration('log_directory')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Experiment parameter YAML file.',
        ),
        DeclareLaunchArgument(
            'log_directory',
            default_value=default_log_directory,
            description='Directory for CSV experiment logs.',
        ),
        Node(
            package='uav_control',
            executable='moving_target',
            name='moving_target',
            output='screen',
            parameters=[config_file],
        ),
        Node(
            package='uav_control',
            executable='target_kalman_filter',
            name='target_kalman_filter',
            output='screen',
            parameters=[config_file],
        ),
        Node(
            package='uav_control',
            executable='trajectory_impact_sim',
            name='trajectory_impact_sim',
            output='screen',
            parameters=[
                config_file,
                {'log_directory': log_directory},
            ],
        ),
    ])
