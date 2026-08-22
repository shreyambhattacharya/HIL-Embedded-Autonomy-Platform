# HIL Embedded Autonomy Platform

This repository is the starting point for a Hardware-in-the-Loop (HIL) embedded autonomy platform. The long-term system will partition high-level autonomy on a Raspberry Pi 5, real-time wheel control and safety on an STM32 with FreeRTOS, and the simulated physical plant on a laptop running Gazebo.

The project is intentionally being built in milestones. The current tree implements **Milestone 1 — Deterministic Simulation Foundation** only. It establishes a differential-drive rover, a ROS 2/Gazebo boundary, a temporary software low-level controller, and a repeatable motion profile. It does not contain hardware or claim hardware validation.

## Current milestone

Implemented in this milestone:

- simple primitive-geometry rover model in Gazebo Harmonic SDF;
- baseline world with fixed initial conditions and static reference objects;
- direct wheel-joint force actuation through Gazebo's `ApplyJointForce` system;
- ROS 2/Gazebo communication through `ros_gz_bridge` YAML configuration;
- wheel joint feedback, IMU, 2D GPU LiDAR, and ground-truth odometry;
- `software_mcu_stub`, a temporary ROS 2 C++ wheel-speed controller;
- `motion_test_node`, a deterministic command profile;
- pure C++ unit tests for kinematics and effort limiting;
- an optional ROS 2 smoke-test executable for a running stack.

Not implemented yet: Raspberry Pi software, UART, STM32 firmware, FreeRTOS, binary protocols, watchdogs, fault injection, cameras, ML, localization, planning, Nav2, and HIL hardware testing.

## Repository layout

```text
docs/
  architecture.md
  milestone_01.md
ros2_ws/src/
  hil_description/     Rover SDF model and model assets
  hil_simulation/      Baseline world, bridge, launch, smoke test
  hil_control_stub/    Temporary controller, motion test, parameters, unit tests
```

## Prerequisites

The target environment is Ubuntu 24.04 LTS with ROS 2 Jazzy and Gazebo Harmonic. Install ROS 2 Jazzy using the [official ROS 2 Ubuntu instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), then install the project dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-sim \
  python3-colcon-common-extensions \
  python3-rosdep \
  build-essential \
  cmake \
  git
```

The source tree does not silently install packages or modify the host. If `rosdep` has not been initialized on the machine yet, initialize it according to the ROS 2 installation instructions.

## Build

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

## Launch Milestone 1

The single launch command starts Gazebo, the bridge, the temporary controller, and the deterministic command source:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch hil_simulation milestone_01.launch.py
```

To run the same stack without the Gazebo GUI:

```bash
ros2 launch hil_simulation milestone_01.launch.py headless:=true
```

The launch starts the simulation running. Gazebo should show the rover driving forward, turning in place, driving forward again, and then stopping.

## Tests

Run the pure software tests:

```bash
cd ros2_ws
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

For the running-stack smoke test, start the launch command in one terminal and quickly run this in a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 run hil_simulation milestone_01_smoke_test
```

The smoke test checks for wheel feedback, IMU, LiDAR (including valid +Inf no-return ranges), ground-truth odometry, actuator effort messages, valid numeric values, observable motion, and a return to zero actuator effort after the motion profile. It is an integration check against a running local simulation; it is not an HIL test.

Useful inspection commands while the stack is running:

```bash
ros2 topic list | grep '^/hil/'
ros2 topic echo /hil/sensors/wheel_states
ros2 topic echo /hil/sensors/imu
ros2 topic echo /hil/sensors/scan
ros2 topic echo /hil/ground_truth/odom
ros2 topic echo /hil/actuator/left_effort
```

## Engineering status

The Milestone 1 build, unit tests, Gazebo GUI/headless launch, ROS/Gazebo bridges, deterministic motion profile, stopped state, and running-stack smoke test have been verified in WSL Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic. This is software-only validation; no hardware or benchmark result is implied.

See [docs/architecture.md](docs/architecture.md) and [docs/milestone_01.md](docs/milestone_01.md) for the runtime boundary, parameters, interfaces, acceptance criteria, and limitations.
