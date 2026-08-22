# Milestone 1 — Deterministic Simulation Foundation

## Objective

Create a repeatable software-only plant and control boundary that can later be connected to a real Raspberry Pi and real STM32 without changing the simulated rover interface.

## Design decisions

1. The rover is a small differential-drive model made only from SDF primitive boxes, cylinders, and low-friction spherical supports. A front support at x = +0.30 m, rear caster at x = -0.30 m, and the drive wheels at x = 0 surround the center-of-mass projection and prevent longitudinal pitch instability while preserving differential steering.
2. Each drive wheel is a revolute joint with a Gazebo Harmonic `ApplyJointForce` system. The controller publishes independent `std_msgs/msg/Float64` effort values that `ros_gz_bridge` maps to Gazebo `gz.msgs.Double` joint-force topics.
3. The rover does not load Gazebo's `DiffDrive` system. Differential-drive kinematics and wheel-speed feedback control remain visible in `software_mcu_stub`.
4. `software_mcu_stub` is temporary ROS 2 infrastructure. It has no UART, FreeRTOS, watchdog, protocol, or simulated STM32 implementation.
5. The bridge uses a YAML file so the message boundary is reviewable and not hidden in a long launch command.
6. Ground-truth odometry is a separate topic from IMU, LiDAR, and wheel feedback. It is for evaluation and smoke tests only.
7. The `0.001 s` physics step and the stated update rates are provisional starting points. They are not measured performance claims.

## Components

- `hil_description/models/hil_rover/model.sdf`: model, joints, sensors, and Gazebo systems.
- `hil_simulation/worlds/baseline.sdf`: flat ground, rover include, lights, and static reference objects.
- `hil_simulation/config/bridge.yaml`: only the required ROS/Gazebo topic mappings.
- `hil_simulation/launch/milestone_01.launch.py`: reproducible stack launch.
- `hil_control_stub/software_mcu_stub`: wheel kinematics, P control, limits, and temporary stale-data stop behavior.
- `hil_control_stub/motion_test_node`: fixed time-based command profile with autostart and one-shot Trigger control.
- `hil_control_stub/test/test_control_math.cpp`: pure unit tests.
- `hil_simulation/scripts/milestone_01_smoke_test.py`: active readiness, stability, motion-phase, sensor, effort, and stop validator.
- `hil_simulation/launch/milestone_01_test.launch.py`: self-contained headless integration-test entry point.

## Motion profile

| Simulation time | Command |
| --- | --- |
| 0–2 s | zero linear velocity and yaw rate |
| 2–7 s | `linear.x = 0.25 m/s` |
| 7–10 s | `angular.z = 0.5 rad/s` in place |
| 10–15 s | `linear.x = 0.25 m/s` |
| 15 s onward | zero command |

The profile is deliberately simple. Interactive launch uses `autostart=true`. Automated testing uses `autostart=false` and calls `/hil/test/start_motion` only after subscriptions and stationary checks are ready. It is not a planner, waypoint follower, obstacle avoidance system, or autonomy stack.

## Launch procedure

```bash
source /opt/ros/jazzy/setup.bash
cd <repository>/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ros2 launch hil_simulation milestone_01.launch.py
```

The optional `headless:=true` launch argument passes `-s` to Gazebo. Set `motion_autostart:=false` to keep publishing a zero command for stationary inspection; call `/hil/test/start_motion` to begin the one-shot profile.

## Interface checks

With the stack running:

```bash
ros2 topic list | grep '^/hil/'
ros2 topic echo /hil/sensors/wheel_states
ros2 topic echo /hil/sensors/imu
ros2 topic echo /hil/sensors/scan
ros2 topic echo /hil/ground_truth/odom
ros2 topic echo /hil/actuator/left_effort
ros2 topic echo /hil/actuator/right_effort
```

The software controller should show non-zero effort messages during the forward and turn phases. The efforts should return to zero after the command profile completes.

## Test procedure

Pure tests:

```bash
cd <repository>/ros2_ws
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Self-contained integration test:

```bash
source /opt/ros/jazzy/setup.bash
source <repository>/ros2_ws/install/setup.bash
ros2 launch hil_simulation milestone_01_test.launch.py
```

The smoke test waits for all required topics and the Trigger service, verifies a stationary zero-effort window, starts the profile, validates phase order, wheel signs, forward displacement, positive yaw, effort bounds, IMU orientation, LiDAR metadata/ranges, odometry, and the final stopped state. Positive infinity remains valid for LiDAR no-return rays; NaN, negative infinity, and out-of-range finite samples fail.

## Acceptance criteria

- [x] `colcon build` completes on Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic.
- [x] Gazebo loads `baseline` and the `hil_rover` model.
- [x] The bridge creates all configured mappings without warnings about unsupported types.
- [x] Wheel feedback, IMU, LiDAR, and ground-truth odometry are visible on their ROS topics.
- [x] `software_mcu_stub` publishes independent left/right effort values.
- [x] The rover moves during the commanded phases and stops after the profile.
- [x] Unit tests pass.
- [x] Both rover and world SDF files pass `gz sdf -k` with the model search path configured.
- [x] The rover remains stationary and level before the profile is triggered.
- [x] The triggered smoke test passes against the self-contained local stack.

The Milestone 1 runtime criteria were verified in WSL Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic; this remains software-only validation.

## Verified observations

The final automated run observed 0.530 m in the first forward phase, 0.196 rad positive yaw during the turn, and 0.423 m in the second forward phase. Peak absolute wheel effort was 1.357 N m under the 1.5 N m limit. The two-second pre-trigger window had 0.0 m measured drift and zero body speed; maximum absolute roll and pitch over the run were approximately 1.1e-7 rad and 5.6e-7 rad. Final body velocity was zero and wheel effort/speed returned to numerical zero.

## Known limitations

- The selected gains, effort limit, friction, and timing rates are initial engineering parameters, not calibrated values.
- The smoke test is deterministic but remains an integration check of a software-only simulation rather than hardware-in-the-loop validation.
- The Gazebo GUI and GPU LiDAR require a functioning Linux graphics/rendering setup; use the documented headless mode for server-only checks.
- There is no TF broadcaster in this milestone. Frame IDs are documented and attached to the sensor/odometry messages where the bridge supports them.
- The ground-truth odometry topic is intentionally available for evaluation and must not be used as a future localization input by accident.

## Milestone 2 direction

Milestone 2 should first characterize the software-only boundary: measured sensor/control rates, command-to-wheel latency, scheduling jitter, and controller behavior under repeatable disturbances. It can then introduce a clearly versioned transport interface for replacing the stub, but should not add hardware or fault infrastructure until the Milestone 1 interfaces are measured and stable.
