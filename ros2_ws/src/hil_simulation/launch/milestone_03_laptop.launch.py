#!/usr/bin/env python3
"""Launch the laptop side of the Milestone 3 distributed simulation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    simulation_share = get_package_share_directory("hil_simulation")
    milestone_01 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_share, "launch", "milestone_01.launch.py")
        ),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "motion_autostart": "false",
            "run_motion_source": "false",
            "controller_diagnostics": LaunchConfiguration("controller_diagnostics"),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="true",
                description="Run Gazebo server without the GUI.",
            ),
            DeclareLaunchArgument(
                "controller_diagnostics",
                default_value="false",
                description="Enable opt-in controller timing diagnostics.",
            ),
            milestone_01,
        ]
    )
