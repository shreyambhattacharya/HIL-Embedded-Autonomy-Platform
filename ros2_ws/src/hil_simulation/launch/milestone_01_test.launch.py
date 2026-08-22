#!/usr/bin/env python3
"""Run the complete headless Milestone 1 integration test."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    simulation_share = get_package_share_directory("hil_simulation")
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                simulation_share,
                "launch",
                "milestone_01.launch.py",
            )
        ),
        launch_arguments={
            "headless": "true",
            "motion_autostart": "false",
        }.items(),
    )
    discovery_warmup = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(cmd=["ros2", "node", "list"], output="log")
        ],
    )

    smoke_test = Node(
        package="hil_simulation",
        executable="milestone_01_smoke_test",
        name="milestone_01_smoke_test",
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
    stop_after_test = RegisterEventHandler(
        OnProcessExit(
            target_action=smoke_test,
            on_exit=[stop_gazebo],
        )
    )
    return LaunchDescription([stack, discovery_warmup, smoke_test, stop_after_test])
