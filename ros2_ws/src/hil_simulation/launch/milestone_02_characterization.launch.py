#!/usr/bin/env python3
"""Run the self-contained Milestone 2 timing characterization."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    simulation_share = get_package_share_directory("hil_simulation")
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_share, "launch", "milestone_01.launch.py")
        ),
        launch_arguments={
            "headless": "true",
            "motion_autostart": "false",
            "controller_diagnostics": "true",
        }.items(),
    )
    characterization = Node(
        package="hil_simulation",
        executable="milestone_02_characterization",
        name="milestone_02_characterization",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    stop_gazebo = ExecuteProcess(
        cmd=[
            "gz",
            "service",
            "-s",
            "/server_control",
            "--reqtype",
            "gz.msgs.ServerControl",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "3000",
            "--req",
            "stop: true",
        ],
        output="screen",
    )
    stop_after_characterization = RegisterEventHandler(
        OnProcessExit(
            target_action=characterization,
            on_exit=[stop_gazebo],
        )
    )
    return LaunchDescription(
        [stack, characterization, stop_after_characterization]
    )
