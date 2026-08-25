#!/usr/bin/env python3
"""Laptop Gazebo plus software-MCU Milestone 5A autonomy stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("hil_autonomy")
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "milestone_05a_stack.launch.py")),
        launch_arguments={
            "backend": "software",
            "headless": LaunchConfiguration("headless"),
            "scenario": LaunchConfiguration("scenario"),
            "scan_topic": LaunchConfiguration("scan_topic"),
            "run_evaluator": LaunchConfiguration("run_evaluator"),
            "evidence_output": LaunchConfiguration("evidence_output"),
            "scenario_timeout_sec": LaunchConfiguration("scenario_timeout_sec"),
            "start_delay_sec": LaunchConfiguration("start_delay_sec"),
            "arm_on_start": "false",
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("scenario", default_value="open_square"),
        DeclareLaunchArgument("scan_topic", default_value="/hil/sensors/scan"),
        DeclareLaunchArgument("run_evaluator", default_value="false"),
        DeclareLaunchArgument("evidence_output", default_value="/tmp/milestone_05a_result.json"),
        DeclareLaunchArgument("scenario_timeout_sec", default_value="80.0"),
        DeclareLaunchArgument("start_delay_sec", default_value="2.0"),
        stack,
    ])
