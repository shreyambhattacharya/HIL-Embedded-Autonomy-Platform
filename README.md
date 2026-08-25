# HIL Embedded Autonomy Platform

This repository is a validated Hardware-in-the-Loop (HIL) embedded autonomy platform. It combines a deterministic Gazebo plant, ROS 2 Jazzy, a real STM32F446RE FreeRTOS controller, a portable binary UART transport, physical STM32 HIL, and a high-level wheel/IMU/LiDAR autonomy path.

Current status: Milestones 1, 2B, 3A software preparation, 4A, 4B, and 5A are complete. The software and physical-STM32 autonomy paths have both been validated, including waypoint tracking, LiDAR slowdown/stop behavior, watchdog/fault handling, and quantitative evidence.

Physical Raspberry Pi validation and full distributed Pi + STM32 deployment remain deferred. This project does not claim SLAM, Nav2, global obstacle planning, or production real-time guarantees.

## Milestone 5A — autonomy foundation

Milestone 5A is complete. The `hil_autonomy` package contains a wheel/IMU-only estimator, configurable waypoint follower, LiDAR slowdown/stop/stale filter, software and physical-STM32 launch paths, deterministic scenarios, and machine-readable evidence. Software open-square repeatability is 5/5 PASS; physical STM32 open-square repeatability is 3/3 PASS; software and STM32 obstacle-stop evidence is PASS; stale-LiDAR zero-command evidence is PASS.

Run the software path:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch hil_autonomy milestone_05a_software_test.launch.py
```

Run the real NUCLEO-F446RE path:

```bash
ros2 launch hil_autonomy milestone_05a_stm32_test.launch.py \
  serial_device:=/dev/serial/by-id baud_rate:=115200
```

Ground truth is consumed only by the evaluator. The autonomy workload was validated on the laptop and through the physical STM32; Raspberry Pi validation remains deferred. See [docs/milestone_05a.md](docs/milestone_05a.md) for architecture, metrics, evidence, limitations, and the exact milestone decision.

## Current system boundary

```text
Laptop:          Gazebo + ROS 2 autonomy + hil_serial_bridge
                 (or the software controller for simulation-only runs)
Physical STM32:  FreeRTOS, 100 Hz wheel control, safety, watchdog, UART
Future Pi:       same autonomy + bridge workload on Raspberry Pi 5
```

The current physical HIL boundary is laptop/Gazebo/autonomy through the real NUCLEO-F446RE. Milestone 3A prepared the Pi software boundary, but physical Pi execution and distributed deployment are future work.

## Implemented foundation

Milestone 1 provides:

- simple primitive-geometry rover model in Gazebo Harmonic SDF;
- baseline world with fixed initial conditions and static reference objects;
- direct wheel-joint force actuation through Gazebo's `ApplyJointForce` system;
- ROS 2/Gazebo communication through `ros_gz_bridge` YAML configuration;
- wheel joint feedback, IMU, 2D GPU LiDAR, and ground-truth odometry;
- `software_mcu_stub`, a temporary ROS 2 C++ wheel-speed controller;
- `motion_test_node`, a deterministic command profile with interactive autostart and a test Trigger service;
- pure C++ unit tests for kinematics and effort limiting;
- an automated ROS 2 smoke test that owns readiness, stationary settling, profile triggering, phase checks, and final stop verification.

The first Milestone 2 slice adds:

- opt-in steady-clock controller period, command-age, and feedback-age diagnostics;
- a self-contained characterization launch using the same deterministic command profile;
- measured ROS topic rates and inter-arrival jitter;
- observed target-to-effort and target-to-wheel-motion latency;
- controller execution-time and execution-budget diagnostics;
- isolated multi-run suites with retained failures and machine-readable aggregates;
- 10-run normal and controlled-load baselines;
- provisional Pi/STM32 transport requirements and analytical UART budgets.

The portable Version 1 protocol, COBS framing, CRC-16, POSIX ROS 2 serial bridge, target-specific FreeRTOS firmware, physical UART, sustained-link, safety-fault, timing, and disconnect/reconnect evidence are documented in the Milestone 4A/4B records. Earlier milestone documents retain historical scope and limitations.

## Repository layout

```text
common/                 Portable protocol, control, and safety libraries
firmware/stm32/         STM32F446RE FreeRTOS controller and safety firmware
ros2_ws/src/
  hil_description/      Rover SDF model and assets
  hil_simulation/       Gazebo worlds, bridges, launches, and regressions
  hil_control_stub/     Temporary software controller and tests
  hil_serial_bridge/    POSIX ROS 2 <-> STM32 UART bridge
  hil_pi_runtime/       Raspberry Pi software-preparation boundary
  hil_autonomy/         Estimator, waypoint follower, LiDAR safety filter
tools/                  Characterization, STM32, and Milestone 5A runners
results/                Compact milestone evidence; raw local data ignored
docs/                   Architecture, protocol, setup, and milestone records
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

To hold the rover at zero target velocity for stationary inspection:

```bash
ros2 launch hil_simulation milestone_01.launch.py motion_autostart:=false
```

The normal launch autostarts the profile. With autostart disabled, the node continuously publishes zero until the `/hil/test/start_motion` Trigger service is called.

## Tests

Run the pure software tests:

```bash
cd ros2_ws
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Run the self-contained headless integration test:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch hil_simulation milestone_01_test.launch.py
```

The test launch disables motion autostart. The smoke node waits for every required interface, observes stationary stability, triggers the profile, checks both forward phases and positive yaw, enforces the effort limit, validates IMU/LiDAR/odometry values, requires a final stop, and then stops Gazebo cleanly. It is a simulation integration test, not an HIL test.

Run the initial Milestone 2 characterization:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch hil_simulation milestone_02_characterization.launch.py
```

This launch enables controller timing diagnostics only for the characterization run. It reports JSON metrics, applies broad regression thresholds, and shuts Gazebo down automatically. Topic intervals and response latency use simulation time; controller period and input-age measurements use a steady wall clock inside the controller.

Run fresh-process repeatability suites from the repository root:

```bash
python3 tools/run_characterization_suite.py \
  --runs 10 \
  --mode normal \
  --evidence-output results/milestone_02/normal_summary.json

python3 tools/run_characterization_suite.py \
  --runs 10 \
  --max-attempts 12 \
  --mode cpu-loaded \
  --stress-workers 4 \
  --evidence-output results/milestone_02/cpu_loaded_summary.json
```

The loaded runner uses `stress-ng` when installed and otherwise falls back to pinned standard `taskset` + `yes` workers. Its JSON records which tool and configuration were actually used; configured capacity is not reported as measured CPU utilization. Raw run JSON and logs are kept under `results/milestone_02/local/` and are ignored by Git. Run the runner's pure Python tests with:

```bash
python3 -m unittest discover -s tools/tests -v
```

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

The Milestone 1 build, control-math suite, SDF validation, Gazebo GUI/headless launch, ROS/Gazebo bridges, triggered deterministic profile, stationary state, and automated smoke test have been verified in WSL Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic. Milestone 2B completed 10 normal and 10 moderate controlled-load fresh-Gazebo runs with all selected trials passing. These are software-only, host-specific observations; no hardware, real-time guarantee, or portable benchmark result is implied.

See [docs/architecture.md](docs/architecture.md), [docs/milestone_01.md](docs/milestone_01.md), [docs/milestone_02.md](docs/milestone_02.md), [docs/milestone_02_repeatability.md](docs/milestone_02_repeatability.md), and [docs/transport_requirements.md](docs/transport_requirements.md) for the runtime boundary, parameters, interfaces, measurements, derived requirements, and limitations.
