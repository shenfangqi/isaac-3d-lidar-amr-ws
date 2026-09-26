"""Compatibility wrapper for the Carbot Isaac Sim navigation launch."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    target = Path(__file__).with_name("carbot_sim.launch.py")
    return LaunchDescription(
        [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(target)))]
    )
