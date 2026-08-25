#!/usr/bin/env python3
"""Laptop Gazebo plus the real STM32 serial backend for Milestone 5A."""

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
            "backend": "stm32",
            "headless": LaunchConfiguration("headless"),
            "scenario": LaunchConfiguration("scenario"),
            "scan_topic": LaunchConfiguration("scan_topic"),
            "run_evaluator": LaunchConfiguration("run_evaluator"),
            "evidence_output": LaunchConfiguration("evidence_output"),
            "scenario_timeout_sec": LaunchConfiguration("scenario_timeout_sec"),
            "start_delay_sec": LaunchConfiguration("start_delay_sec"),
            "arm_on_start": LaunchConfiguration("arm_on_start"),
            "serial_device": LaunchConfiguration("serial_device"),
            "baud_rate": LaunchConfiguration("baud_rate"),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("scenario", default_value="open_square"),
        DeclareLaunchArgument("scan_topic", default_value="/hil/sensors/scan"),
        DeclareLaunchArgument("run_evaluator", default_value="false"),
        DeclareLaunchArgument("evidence_output", default_value="/tmp/milestone_05a_stm32_result.json"),
        DeclareLaunchArgument("scenario_timeout_sec", default_value="80.0"),
        DeclareLaunchArgument("start_delay_sec", default_value="2.0"),
        DeclareLaunchArgument("arm_on_start", default_value="false"),
        DeclareLaunchArgument("serial_device", default_value="/dev/serial/by-id"),
        DeclareLaunchArgument("baud_rate", default_value="115200"),
        stack,
    ])
