#!/usr/bin/env python3
"""Launch the complete Milestone 1 software-only stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    simulation_share = get_package_share_directory("hil_simulation")
    control_share = get_package_share_directory("hil_control_stub")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    world_path = os.path.join(simulation_share, "worlds", "baseline.sdf")
    bridge_path = os.path.join(simulation_share, "config", "bridge.yaml")
    controller_path = os.path.join(control_share, "config", "controller.yaml")
    motion_path = os.path.join(control_share, "config", "motion_test.yaml")

    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    gz_flags = "-r -v3 " + ("-s " if headless else "") + world_path

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
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
        parameters=[controller_path, {"use_sim_time": True}],
        output="screen",
    )

    motion_test = Node(
        package="hil_control_stub",
        executable="motion_test_node",
        name="motion_test_node",
        parameters=[motion_path, {"use_sim_time": True}],
        output="screen",
    )

    return [gazebo, bridge, controller, motion_test]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo server without the GUI when true.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
