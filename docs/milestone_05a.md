# Milestone 5A — High-Level Autonomy Foundation

## Initial repository state

- Authoritative checkout: `/home/shrey/projects/HIL-Embedded-Autonomy-Platform` in WSL Ubuntu 24.04.
- Starting branch: `milestone-04b-stm32-hardening`.
- Starting commit: `5b7995ff51ee427b7f89dae8e21a43c9955496db` (`Complete Milestone 4B transport hardening`).
- Working tree: clean before creating `milestone-05a-autonomy-foundation`; Milestone 5A changes remain uncommitted and unpushed.
- STM32: physical NUCLEO-F446RE `NUF446RE$KU1` visible through `/dev/serial/by-id`, using the validated 115200 baud path.
- Raspberry Pi: physically unavailable; no Pi execution is claimed.

## Scope and exclusions

This milestone adds wheel/IMU state estimation, a configurable waypoint follower, and a reactive LiDAR collision filter. It deliberately does not add SLAM, Nav2, AMCL, EKF packages, cameras, vision, ML, GPS, maps, global planning, or obstacle circumnavigation.

The autonomy nodes do not subscribe to `/hil/ground_truth/odom`. The evaluator alone consumes ground truth for aligned error and safety metrics, and publishes no control.

## Architecture

The staged control path is:

```text
/hil/sensors/wheel_states + /hil/sensors/imu
                    |
                    v
          state_estimator_node
                    |
                    v
          /hil/estimate/odom
                    |
                    v
          waypoint_follower_node
                    |
                    v
          /hil/autonomy/raw_twist
                    ^
                    |
             LiDAR collision_filter
                    |
                    v
          /hil/control/target_twist
                    |
          software_mcu_stub OR hil_serial_bridge
                    |
                 STM32
```

The software launch uses Gazebo, `ros_gz_bridge`, `software_mcu_stub`, and the three autonomy nodes. The STM32 launch replaces only the software controller with `hil_serial_bridge`; it does not run `software_mcu_stub` or `motion_test_node`. STM32 ARM is explicit in the hardware test launch. The existing STM32 remains responsible for command/feedback freshness, ARM/DISARM, watchdog behavior, wheel control, and zero-effort safety.

## State estimator

`state_estimator_node` uses only `/hil/sensors/wheel_states` and `/hil/sensors/imu`. It uses the configured wheel radius (0.18 m) and separation (0.86 m), derives differential-drive linear and wheel yaw rates, blends wheel yaw rate with IMU z rate using the configured complementary weight (0.25), and integrates a local odometric frame initialized at startup. Wheel/IMU samples with invalid timestamps, nonfinite values, negative/zero time, or excessive time steps are rejected. A sensor freshness timeout marks the estimate invalid and the node publishes a conservative zero-velocity state.

The evaluator aligns estimator and truth displacements at autonomy start. One expanded fresh run per backend produced:

| Metric | Software controller | Physical STM32 |
| --- | ---: | ---: |
| Open-square completion time | 47.6 s | 47.8 s |
| RMS position error | 0.038830 m | 0.039020 m |
| Maximum position error | 0.063602 m | 0.062654 m |
| Final position error | 0.062484 m | 0.061529 m |
| RMS yaw error | 0.023244 rad | 0.023030 rad |
| Maximum yaw error | 0.048757 rad | 0.049496 rad |
| Final yaw error | 0.014231 rad | 0.011917 rad |

These values are evaluation measurements, not controller inputs or production accuracy guarantees.

## Waypoint follower

The follower tracks the YAML open square `(1,0) -> (1,1) -> (0,1) -> (0,0)). It computes distance and wrapped heading error from estimated odometry, commands proportional heading correction, scales forward speed by heading alignment, and clamps to 0.25 m/s and 0.8 rad/s. It has explicit `/hil/autonomy/start` and `/hil/autonomy/stop` Trigger services and states `IDLE`, `TRACKING`, `COMPLETE`, and `ABORTED`. It publishes zero after stop and after completion.

## Collision filter

The filter examines a configurable +/-0.70 rad frontal sector. It treats positive infinity as a valid clear-space LiDAR return, ignores NaN, negative infinity, nonpositive, and out-of-sector samples, linearly slows between 1.20 m and 0.75 m, and commands zero linear velocity at or inside 0.75 m. A scan older than 0.50 s produces `STALE` and a zero target twist. This is a high-level availability filter; it does not replace STM32 safety authority.

## Scenario evidence

- Software open square: 5/5 fresh runs PASS. Completion times measured from retained logs were 48.171–48.338 s; final return error was 0.113077–0.114229 m; all runs completed with a zero target twist.
- Physical STM32 open square: 3/3 fresh runs PASS. Completion times were 48.485–48.710 s; final return error was 0.111870–0.113525 m; all launches returned cleanly, with no observed transport failure or SAFE transition.
- Software obstacle stop: PASS. The filter reached `STOP` at 0.650040 m, target linear speed was zero, and truth progress after the stop was 0.0 m.
- Physical STM32 obstacle stop: PASS. The same 0.650040 m stop distance produced zero target linear speed and 0.0 m post-stop truth progress.
- Software stale LiDAR: PASS. The test-only relay stopped forwarding valid scans, the filter reached `STALE`, and the target twist became zero.
- Software/STM32 command-source exclusivity: autonomy launches omit `motion_test_node`; the target-twist path is the collision filter into either the software controller or the serial bridge.

The machine-readable records are:

- [software_open_square_summary.json](../results/milestone_05a/software_open_square_summary.json)
- [stm32_open_square_summary.json](../results/milestone_05a/stm32_open_square_summary.json)
- [software_open_square_metrics.json](../results/milestone_05a/software_open_square_metrics.json)
- [stm32_open_square_metrics.json](../results/milestone_05a/stm32_open_square_metrics.json)
- [software_obstacle_stop.json](../results/milestone_05a/software_obstacle_stop.json)
- [stm32_obstacle_stop.json](../results/milestone_05a/stm32_obstacle_stop.json)
- [software_stale_lidar.json](../results/milestone_05a/software_stale_lidar.json)

## Regression evidence

- Full ROS workspace build: PASS, 6 packages.
- ROS tests: PASS, 18 tests, 0 errors, 0 failures, 0 skipped.
- `hil_autonomy` pure tests: PASS, estimator/waypoint/collision core tests.
- Portable common-library CMake/CTest: PASS, protocol, control, and safety tests (3/3).
- Milestone 1 Gazebo smoke test: PASS, including forward motion, turn, final zero command, and phase checks.
- Physical STM32 nominal path: PASS through `hil_serial_bridge` at 115200 baud with 100 Hz command/feedback configuration; the three repeatability runs opened the real ST-Link serial endpoint and completed without observed transport failures.

## How to reproduce

```bash
source /opt/ros/jazzy/setup.bash
cd ~/projects/HIL-Embedded-Autonomy-Platform/ros2_ws
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose

ros2 launch hil_autonomy milestone_05a_software_test.launch.py
ros2 launch hil_autonomy milestone_05a_stm32_test.launch.py \
  serial_device:=/dev/serial/by-id baud_rate:=115200
```

The repeatability runner uses five software runs and three STM32 runs by default:

```bash
cd ~/projects/HIL-Embedded-Autonomy-Platform
python3 tools/run_milestone_05a.py --backend software --scenario open_square --runs 5
python3 tools/run_milestone_05a.py --backend stm32 --scenario open_square --runs 3 \
  --serial-device /dev/serial/by-id --baud-rate 115200
```

## Known limitations

The estimator is a deliberately small wheel/IMU complementary odometer, not a production EKF. The waypoint follower performs local reactive tracking only; it does not plan around obstacles. The obstacle scenario measures commanded stop and post-stop progress but does not claim a separate contact sensor or full collision-free proof. The current evidence is laptop Gazebo plus either the software controller or physical STM32; it is not a Raspberry Pi execution.

Physical Raspberry Pi validation remains deferred.
The Milestone 5A autonomy workload is designed to migrate to Pi later.

MILESTONE 5A COMPLETE
