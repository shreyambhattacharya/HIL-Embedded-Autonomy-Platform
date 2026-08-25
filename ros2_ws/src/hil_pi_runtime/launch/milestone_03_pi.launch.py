#!/usr/bin/env python3
"""Launch the Raspberry Pi-side Milestone 3 processes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_share = get_package_share_directory("hil_control_stub")
    motion_path = os.path.join(control_share, "config", "motion_test.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "autostart",
                default_value="false",
                description="Start the deterministic profile without a Trigger call.",
            ),
            DeclareLaunchArgument(
                "status_rate_hz",
                default_value="1.0",
                description="Pi status publication rate.",
            ),
            Node(
                package="hil_control_stub",
                executable="motion_test_node",
                name="motion_test_node",
                parameters=[
                    motion_path,
                    {"use_sim_time": False},
                    {"autostart": LaunchConfiguration("autostart")},
                ],
                output="screen",
            ),
            Node(
                package="hil_pi_runtime",
                executable="pi_status_node",
                name="pi_status_node",
                parameters=[{"status_rate_hz": LaunchConfiguration("status_rate_hz")}],
                output="screen",
            ),
        ]
    )
