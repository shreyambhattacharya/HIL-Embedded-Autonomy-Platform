# Milestone 3 — Raspberry Pi 5 Distributed Linux Integration

## Status

The software preparation slice is implemented on branch `milestone-03-pi-integration`, based on Milestone 2B commit `4070349fd92165ad5483def14960159b7bd2d8a6`. No Raspberry Pi host, user, SSH alias, or `HIL_PI_HOST`/`HIL_PI_USER` configuration was present in the development environment, so all Pi-dependent build and runtime evidence are `NOT RUN`. No hardware claim is made.

## Objective

Introduce the first physical embedded-Linux boundary without adding the STM32. The laptop remains responsible for Gazebo, `ros_gz_bridge`, and the temporary `software_mcu_stub`; a future Raspberry Pi 5 runs the deterministic command source and a small integration-status node over ROS 2/DDS Ethernet.

## Partition

```text
Laptop / WSL Ubuntu 24.04
  Gazebo Harmonic
  ros_gz_bridge
  software_mcu_stub
       ^  /hil/sensors/* and /hil/ground_truth/odom
       |
       | ROS 2 / DDS over wired Ethernet
       |
       v  /hil/control/target_twist and /hil/pi/status
Raspberry Pi 5 / Ubuntu Server 24.04 ARM64
  existing motion_test_node, launched with wall time
  pi_status_node
```

The STM32, UART, custom framing, watchdog authority, FreeRTOS, and hardware safety remain outside this milestone.

## Architecture changes

- `milestone_01.launch.py` now has `run_motion_source`, defaulting to `true`; the existing Milestone 1 behavior is unchanged.
- `milestone_03_laptop.launch.py` includes the existing laptop stack with `run_motion_source:=false`, so no local target command publisher is launched.
- `hil_pi_runtime` is a dependency-light Pi-side package. Its launch includes the already validated `motion_test_node` from `hil_control_stub` and starts `pi_status_node`.
- The Pi launch explicitly sets `use_sim_time:=false`; there is no Gazebo `/clock` on the Pi.
- `pi_status_node` publishes `/hil/pi/status` as `std_msgs/msg/String` and exposes `/hil/pi/ping` as `std_srvs/srv/Trigger` for caller-local RTT probing.

The deterministic source was not duplicated or moved. Keeping it in `hil_control_stub` is justified because that package has no Gazebo dependency and is already the validated owner of the source and controller boundary. The targeted Pi build stops at `hil_pi_runtime`, so the simulator package is not required on the Pi.

## Pi environment

| Item | Result |
| --- | --- |
| Physical Pi model | NOT RUN — no Pi endpoint configured |
| OS / architecture | NOT RUN — no Pi endpoint configured |
| ROS 2 Jazzy ARM64 build | NOT RUN — no Pi endpoint configured |
| Host/user/SSH configuration | NOT RUN — no configuration present |
| Wired Ethernet discovery | NOT RUN |
| Pi middleware selection | NOT RUN; repository default remains preferred |

The procedure for actual setup is in `docs/raspberry_pi_setup.md`. The repository does not contain credentials, private IP addresses, or a hidden host assumption.

## Launch procedure

On the laptop:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch hil_simulation milestone_03_laptop.launch.py
```

On the Pi, after the targeted ARM64 build:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch hil_pi_runtime milestone_03_pi.launch.py autostart:=false
```

Then verify `/hil/pi/status`, `/hil/pi/ping`, `/hil/control/target_twist`, the simulator sensor topics, and `/hil/test/start_motion` as described in the setup guide. The profile must be triggered only after both hosts are visible in the ROS graph.

## Distributed test procedure

1. Confirm both hosts are on the same wired network and use the same `ROS_DOMAIN_ID`.
2. Start the laptop launch and verify Gazebo, bridge, and `software_mcu_stub`.
3. Start the Pi launch and verify `/hil/pi/status` contains the actual Pi hostname.
4. From the Pi, observe wheel states, IMU, LiDAR, and ground-truth odometry.
5. From the laptop, use verbose topic info to confirm the target publisher is the Pi-hosted source.
6. Trigger `/hil/test/start_motion`.
7. Observe forward motion, yaw, second forward phase, and final stop.
8. Stop only the Pi command-source process and measure laptop-local time from the last valid target to zero left/right effort.
9. Run the caller-local ICMP and ROS service RTT measurements; never report one-way latency from unsynchronized clocks.

## Actual results

The following Pi-dependent checks are intentionally not claimed:

```text
PI ARM64 BUILD                          NOT RUN
PI ROS 2 STARTUP                        NOT RUN
LAPTOP <-> PI DDS DISCOVERY             NOT RUN
PI RECEIVES SIMULATED SENSOR TOPICS     NOT RUN
LAPTOP RECEIVES PI COMMAND TOPIC        NOT RUN
COMMAND SOURCE CONFIRMED ON PI          NOT RUN
ROVER FORWARD / TURN / FINAL STOP       NOT RUN
PI COMMAND PROCESS LOSS TEST             NOT RUN
STALE-COMMAND-TO-ZERO-EFFORT LATENCY    NOT RUN
ICMP RTT                                NOT RUN
ROS RTT                                 NOT RUN
```

The ROS RTT tool is ready at `tools/pi/ros_rtt_probe.py`; it reports `NOT_RUN` when `/hil/pi/ping` is unavailable. The existing controller timeout is `command_timeout_sec=0.5` seconds. That value is a temporary software behavior, not an STM32 watchdog requirement.

## Local validation

Local-only validation completed in WSL; these checks do not substitute for ARM64 or cross-host evidence:

```text
ROS dependency check                     PASS / local only
Laptop workspace build                   PASS / local only
ROS package tests                        PASS / local only
Milestone 1 integration                  PASS / local only
Milestone 2 characterization tooling     PASS / local only
Milestone 3 laptop wrapper startup       PASS / local only; bounded startup smoke
Milestone 3 Pi runtime smoke             PASS / local only; emulated Pi launch
RTT probe without Pi service             NOT_RUN / expected structured result
```

The local Pi-runtime smoke started `motion_test_node` and `pi_status_node`, observed `/hil/pi/status`, received `pong` from `/hil/pi/ping`, and shut down cleanly. The RTT probe returned a structured `NOT_RUN` result when the service was absent. These local checks cannot substitute for ARM64 or cross-host evidence.

## Limitations

- No physical Pi was available to this task environment.
- No network RTT, ROS RTT, sensor visibility, Pi-origin proof, or command-loss timing was measured.
- WSL cross-host DDS behavior remains to be tested on the actual wired topology.
- `/hil/pi/status` identifies a participating process but is not an authenticated device identity or health protocol.
- ROS 2/DDS is appropriate for the laptop/Pi simulation boundary; the future Pi/STM32 link still requires the separately documented UART protocol milestone.

## Next milestone

After a real Pi passes the distributed acceptance criteria, the next milestone is **Milestone 4 — STM32 FreeRTOS Foundation and Physical Pi↔STM32 UART Bring-Up**. Do not implement that milestone here.
