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
            package='ros_gz_image',
            executable='image_bridge',
            name='dual_tof_image_bridge',
            output='screen',
            arguments=[
                '/uav/camera/front/image',
                '/uav/camera/front/depth_image',
                '/uav/camera/down/image',
                '/uav/camera/down/depth_image',
            ],
            remappings=[
                (
                    '/uav/camera/front/image',
                    '/camera/front/image_raw',
                ),
                (
                    '/uav/camera/front/depth_image',
                    '/camera/front/depth/image_raw',
                ),
                (
                    '/uav/camera/down/image',
                    '/camera/down/image_raw',
                ),
                (
                    '/uav/camera/down/depth_image',
                    '/camera/down/depth/image_raw',
                ),
            ],
        ),
        Node(
            package='uav_control',
            executable='front_tof_monitor',
            name='front_tof_monitor',
            output='screen',
            parameters=[config_file],
        ),
        Node(
            package='uav_control',
            executable='rgbd_target_localizer',
            name='rgbd_target_localizer',
            output='screen',
            parameters=[config_file],
        ),
        Node(
            package='uav_control',
            executable='front_tof_monitor',
            name='down_tof_monitor',
            output='screen',
            parameters=[
                config_file,
                {
                    'camera_name': 'down',
                    'diagnostic_prefix': '/perception/down',
                    'camera_frame_id': 'down_camera_optical_frame',
                    'color_gazebo_topic': '/uav/camera/down/image',
                    'depth_gazebo_topic': (
                        '/uav/camera/down/depth_image'
                    ),
                    'color_ros_topic': '/camera/down/image_raw',
                    'depth_ros_topic': '/camera/down/depth/image_raw',
                    'camera_pitch_down': 1.57079632679,
                    'target_visual_height_offset': 0.42,
                },
            ],
        ),
        Node(
            package='uav_control',
            executable='dual_tof_selector',
            name='dual_tof_selector',
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
