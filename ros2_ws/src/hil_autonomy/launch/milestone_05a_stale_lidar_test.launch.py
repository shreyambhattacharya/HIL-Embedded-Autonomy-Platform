#!/usr/bin/env python3
"""Software stale-LiDAR test using the explicit test-only scan gate."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("hil_autonomy")
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "milestone_05a_software.launch.py")),
        launch_arguments={
            "headless": "true",
            "scenario": "stale_lidar",
            "scan_topic": "/hil/autonomy/test_scan",
            "run_evaluator": "false",
        }.items(),
    )
    gate = Node(
        package="hil_autonomy",
        executable="scan_gate",
        name="milestone_05a_scan_gate",
        parameters=[{"use_sim_time": True, "drop_after_sec": 3.0,
                     "output_topic": "/hil/autonomy/test_scan"}],
        output="screen",
    )
    evaluator = Node(
        package="hil_autonomy",
        executable="milestone_05a_evaluator",
        name="milestone_05a_evaluator",
        parameters=[{
            "use_sim_time": True,
            "scenario": "stale_lidar",
            "backend": "software",
            "output_path": LaunchConfiguration("evidence_output"),
            "timeout_sec": 15.0,
            "start_delay_sec": 1.0,
            "arm_on_start": False,
        }],
        output="screen",
    )
    stop_gazebo = ExecuteProcess(
        cmd=["gz", "service", "-s", "/server_control", "--reqtype", "gz.msgs.ServerControl",
             "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", "stop: true"],
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("evidence_output", default_value="/tmp/milestone_05a_stale_lidar.json"),
        stack,
        gate,
        evaluator,
        RegisterEventHandler(OnProcessExit(target_action=evaluator, on_exit=[stop_gazebo])),
    ])
