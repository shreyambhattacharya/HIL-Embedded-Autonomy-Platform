#!/usr/bin/env python3
"""Launch the complete Milestone 1 software-only stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    del args, kwargs

    simulation_share = get_package_share_directory("hil_simulation")
    control_share = get_package_share_directory("hil_control_stub")
    description_share = get_package_share_directory("hil_description")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    world_path = os.path.join(
        simulation_share,
        "worlds",
        "baseline.sdf",
    )
    bridge_path = os.path.join(
        simulation_share,
        "config",
        "bridge.yaml",
    )
    controller_path = os.path.join(
        control_share,
        "config",
        "controller.yaml",
    )
    motion_path = os.path.join(
        control_share,
        "config",
        "motion_test.yaml",
    )

    # Make the installed rover model discoverable by Gazebo while preserving
    # any resource paths already configured by ROS / ros_gz.
    model_resource_path = os.path.join(description_share, "models")
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")

    if existing_resource_path:
        gazebo_resource_path_value = (
            model_resource_path
            + os.pathsep
            + existing_resource_path
        )
    else:
        gazebo_resource_path_value = model_resource_path

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=gazebo_resource_path_value,
    )

    headless = (
        LaunchConfiguration("headless")
        .perform(context)
        .lower()
        == "true"
    )

    gz_flags = "-r -v3 "
    if headless:
        gz_flags += "-s "
    gz_flags += world_path

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_share,
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": gz_flags,
            "on_exit_shutdown": "true",
        }.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="milestone_01_bridge",
        parameters=[{"config_file": bridge_path}],
        output="screen",
    )

    controller = Node(
        package="hil_control_stub",
        executable="software_mcu_stub",
        name="software_mcu_stub",
        parameters=[
            controller_path,
            {"use_sim_time": True},
            {
                "publish_timing_diagnostics": LaunchConfiguration("controller_diagnostics")
            },
        ],
        output="screen",
    )

    motion_test = Node(
        package="hil_control_stub",
        executable="motion_test_node",
        name="motion_test_node",
        parameters=[
            motion_path,
            {"use_sim_time": True},
            {"autostart": LaunchConfiguration("motion_autostart")},
        ],
        condition=IfCondition(LaunchConfiguration("run_motion_source")),
        output="screen",
    )

    return [
        gazebo_resource_path,
        gazebo,
        bridge,
        controller,
        motion_test,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo server without the GUI when true.",
            ),
            DeclareLaunchArgument(
                "motion_autostart",
                default_value="true",
                description="Start the deterministic motion profile automatically.",
            ),
            DeclareLaunchArgument(
                "run_motion_source",
                default_value="true",
                description="Launch the deterministic target-twist source locally.",
            ),
            DeclareLaunchArgument(
                "controller_diagnostics",
                default_value="false",
                description="Publish opt-in Milestone 2 controller timing diagnostics.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
