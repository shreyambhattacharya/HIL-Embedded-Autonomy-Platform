#!/usr/bin/env python3
"""Common Milestone 5A stack for software and physical STM32 backends."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    autonomy_share = get_package_share_directory("hil_autonomy")
    simulation_share = get_package_share_directory("hil_simulation")
    control_share = get_package_share_directory("hil_control_stub")
    description_share = get_package_share_directory("hil_description")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    config_path = os.path.join(autonomy_share, "config", "autonomy.yaml")
    bridge_path = os.path.join(simulation_share, "config", "bridge.yaml")
    baseline_path = os.path.join(simulation_share, "worlds", "baseline.sdf")
    obstacle_path = os.path.join(autonomy_share, "worlds", "autonomy_obstacle.sdf")
    scenario = LaunchConfiguration("scenario").perform(context)
    world_path = obstacle_path if scenario == "obstacle_stop" else baseline_path
    backend = LaunchConfiguration("backend").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    model_resource_path = os.path.join(description_share, "models")
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = model_resource_path + (os.pathsep + existing_resource_path if existing_resource_path else "")
    gz_flags = "-r -v3 " + ("-s " if headless else "") + world_path

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_flags, "on_exit_shutdown": "true"}.items(),
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="milestone_05a_gz_bridge",
        parameters=[{"config_file": bridge_path}],
        output="screen",
    )
    estimator = Node(
        package="hil_autonomy",
        executable="state_estimator_node",
        name="state_estimator_node",
        parameters=[config_path, {"use_sim_time": True}],
        output="screen",
    )
    follower = Node(
        package="hil_autonomy",
        executable="waypoint_follower_node",
        name="waypoint_follower_node",
        parameters=[config_path, {"use_sim_time": True}],
        output="screen",
    )
    collision_filter = Node(
        package="hil_autonomy",
        executable="collision_filter_node",
        name="collision_filter_node",
        parameters=[
            config_path,
            {"use_sim_time": True, "scan_topic": LaunchConfiguration("scan_topic")},
        ],
        output="screen",
    )
    software = Node(
        package="hil_control_stub",
        executable="software_mcu_stub",
        name="software_mcu_stub",
        parameters=[
            os.path.join(control_share, "config", "controller.yaml"),
            {"use_sim_time": True},
        ],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("backend"), "' == 'software'"])),
        output="screen",
    )
    serial = Node(
        package="hil_serial_bridge",
        executable="hil_serial_bridge",
        name="hil_serial_bridge",
        parameters=[{
            "serial_device": LaunchConfiguration("serial_device"),
            "baud_rate": LaunchConfiguration("baud_rate"),
        }],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("backend"), "' == 'stm32'"])),
        output="screen",
    )
    evaluator = Node(
        package="hil_autonomy",
        executable="milestone_05a_evaluator",
        name="milestone_05a_evaluator",
        parameters=[{
            "use_sim_time": True,
            "scenario": LaunchConfiguration("scenario"),
            "backend": LaunchConfiguration("backend"),
            "output_path": LaunchConfiguration("evidence_output"),
            "timeout_sec": LaunchConfiguration("scenario_timeout_sec"),
            "start_delay_sec": LaunchConfiguration("start_delay_sec"),
            "arm_on_start": LaunchConfiguration("arm_on_start"),
        }],
        condition=IfCondition(LaunchConfiguration("run_evaluator")),
        output="screen",
    )
    return [SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=resource_path),
            gazebo, bridge, software, serial, estimator, follower, collision_filter, evaluator]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="software"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("scenario", default_value="open_square"),
        DeclareLaunchArgument("scan_topic", default_value="/hil/sensors/scan"),
        DeclareLaunchArgument("run_evaluator", default_value="false"),
        DeclareLaunchArgument("evidence_output", default_value="/tmp/milestone_05a_result.json"),
        DeclareLaunchArgument("scenario_timeout_sec", default_value="80.0"),
        DeclareLaunchArgument("start_delay_sec", default_value="2.0"),
        DeclareLaunchArgument("arm_on_start", default_value="false"),
        DeclareLaunchArgument("serial_device", default_value="/dev/serial/by-id"),
        DeclareLaunchArgument("baud_rate", default_value="115200"),
        OpaqueFunction(function=_launch_setup),
    ])
