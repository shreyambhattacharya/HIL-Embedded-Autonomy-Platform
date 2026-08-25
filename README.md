# HIL Embedded Autonomy Platform

This repository is the starting point for a Hardware-in-the-Loop (HIL) embedded autonomy platform. The long-term system will partition high-level autonomy on a Raspberry Pi 5, real-time wheel control and safety on an STM32 with FreeRTOS, and the simulated physical plant on a laptop running Gazebo.

The project is intentionally being built in milestones. **Milestone 1 — Deterministic Simulation Foundation** is validated, and **Milestone 2B — Repeatability Characterization and Transport Requirements** is complete on the local software-only host. It measures the existing boundary before introducing transport or fault infrastructure. It does not contain hardware or claim hardware validation.

Milestone 4B is complete: the portable Version 1 protocol, POSIX ROS 2 serial bridge, and target-specific FreeRTOS firmware for the user-confirmed NUCLEO-F446RE have passed host/ARM builds, verified flashing, physical UART, sustained-link, safety-fault, timing, and disconnect/reconnect HIL gates.

## Milestone 5A — autonomy foundation

Milestone 5A is complete on branch `milestone-05a-autonomy-foundation`. The new `hil_autonomy` package contains a wheel/IMU-only estimator, configurable waypoint follower, LiDAR slowdown/stop/stale filter, software and physical-STM32 launch paths, deterministic scenarios, and machine-readable evidence. Software open-square repeatability is 5/5 PASS; physical STM32 open-square repeatability is 3/3 PASS; software and STM32 obstacle-stop evidence is PASS; stale-LiDAR zero-command evidence is PASS.

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

Milestone 4A currently provides:

- `common/`: shared C protocol and controller-core libraries with host tests;
- `ros2_ws/src/hil_serial_bridge/`: POSIX serial bridge and ROS ARM/DISARM/PING services;
- `firmware/stm32/`: STM32F446RE FreeRTOS application, startup, linker script, and Make build;
- `docs/protocol.md`, `docs/stm32_setup.md`, and `docs/milestone_04a.md`: wire contract, board setup, and evidence status.

Optional target timing, physical disconnect, and paired A/B evidence remain open. Cameras, ML, localization, planning, and Nav2 are also outside the current scope.

## Repository layout

```text
docs/
  milestone_02_repeatability.md
  architecture.md
  milestone_01.md
  milestone_02.md
  transport_requirements.md
results/milestone_02/   Checked-in aggregate evidence; ignored local raw logs
tools/                  Repeatability runner and pure Python unit tests
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
