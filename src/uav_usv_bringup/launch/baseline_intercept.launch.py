"""Compatibility launch: all flight control uses the strict visual pipeline."""

from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Forward launch arguments to the strict process-isolated pipeline."""
    path = Path(__file__).with_name('modular_intercept.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path))),
    ])
