#!/usr/bin/env python3
"""One deterministic software-backend Milestone 5A scenario run."""

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
            "scenario": LaunchConfiguration("scenario"),
            "run_evaluator": "false",
            "evidence_output": LaunchConfiguration("evidence_output"),
            "scenario_timeout_sec": LaunchConfiguration("scenario_timeout_sec"),
            "start_delay_sec": LaunchConfiguration("start_delay_sec"),
        }.items(),
    )
    evaluator = Node(
        package="hil_autonomy",
        executable="milestone_05a_evaluator",
        name="milestone_05a_evaluator",
        parameters=[{
            "use_sim_time": True,
            "scenario": LaunchConfiguration("scenario"),
            "backend": "software",
            "output_path": LaunchConfiguration("evidence_output"),
            "timeout_sec": LaunchConfiguration("scenario_timeout_sec"),
            "start_delay_sec": LaunchConfiguration("start_delay_sec"),
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
        DeclareLaunchArgument("scenario", default_value="open_square"),
        DeclareLaunchArgument("evidence_output", default_value="/tmp/milestone_05a_result.json"),
        DeclareLaunchArgument("scenario_timeout_sec", default_value="80.0"),
        DeclareLaunchArgument("start_delay_sec", default_value="2.0"),
        stack,
        evaluator,
        RegisterEventHandler(OnProcessExit(
            target_action=evaluator,
            on_exit=[stop_gazebo],
        )),
    ])
