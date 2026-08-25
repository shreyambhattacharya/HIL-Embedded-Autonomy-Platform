# Architecture

## Long-term system

The platform is a distributed embedded computing system. The laptop owns the simulated plant and environment. The Raspberry Pi 5 will eventually run embedded Linux, autonomy, supervision, and a UART bridge. The STM32 will eventually run the low-level real-time controller and safety functions under FreeRTOS.

```text
FUTURE

Gazebo physical plant and sensors
             |
             v
       ROS 2 / Ethernet
             |
             v
      Raspberry Pi 5
      embedded Linux
      autonomy + bridge
             |
          real UART
             |
             v
      STM32 + FreeRTOS
      wheel control + safety
             |
          real UART
             |
             v
      Raspberry Pi bridge
             |
             v
       ROS 2 / Ethernet
             |
             v
      Gazebo physical plant
```

## Current validated deployment - Milestone 5A

The validated current boundary keeps high-level autonomy and the simulated plant on the laptop while using the real NUCLEO-F446RE as the low-level controller:

```text
CURRENT

Laptop:                 Gazebo + ROS 2 autonomy + hil_serial_bridge
                         (or software_mcu_stub for software-only runs)
                                  |
                             real UART / ST-Link VCP
                                  v
Physical NUCLEO-F446RE:  STM32F446RE FreeRTOS
                         100 Hz wheel controller + safety + watchdog

FUTURE

Laptop:                 Gazebo
                                  |
                             ROS 2 / Ethernet
                                  v
Raspberry Pi 5:         autonomy + hil_serial_bridge
                                  |
                             real UART
                                  v
STM32:                  same real-time controller and safety boundary
```

Physical Raspberry Pi validation and distributed Pi + STM32 execution remain deferred. The current autonomy evidence is laptop Gazebo plus either the software controller or the physical STM32.

## Today — Milestone 1

The temporary software node occupies the future low-level-controller boundary. It receives a body twist command and wheel feedback through ROS 2, computes wheel-speed targets, applies a P controller, clamps the resulting efforts, and publishes two independent effort commands. Gazebo consumes those effort commands at its wheel joints; it does not run a differential-drive controller for this rover.

```text
motion_test_node
        ^ /hil/test/start_motion
        |
        | /hil/control/target_twist
        v
software_mcu_stub
        ^
        | /hil/sensors/wheel_states
        |
        | /hil/actuator/left_effort
        | /hil/actuator/right_effort
        v
Gazebo ApplyJointForce systems
        |
        v
left/right wheel joints -> rover dynamics
        |
        +--> wheel state publisher
        +--> IMU sensor
        +--> GPU LiDAR sensor
        +--> ground-truth odometry publisher
                 |
                 v
             ros_gz_bridge
```

The command source supports normal interactive autostart and test-controlled start through one Trigger service. The software controller is deliberately isolated from Gazebo topic names. Its ROS inputs and outputs are the replacement seam for a future Pi-to-STM32 transport adapter. No UART, protocol, FreeRTOS, or fake hardware abstraction is present in this milestone.

## Milestone 2 characterization overlay

Milestone 2 adds observability without changing the control path. When `publish_timing_diagnostics=true`, `software_mcu_stub` publishes steady-clock measurements of its control period, controller execution duration, and the ages of its latest command and wheel feedback. The default is false, so Milestone 1 interactive and smoke-test behavior is unchanged.

```text
deterministic command profile -> software_mcu_stub -> wheel effort -> Gazebo
            |                         |                   |          |
            +-------------------------+-------------------+----------+
                                      |
                                      v
                         milestone_02_characterization
                         rates + jitter + response latency
```

The characterization node observes the boundary; it does not sit in the command or actuator path. It uses the repeatable command steps as excitation. The repository-level suite runner creates a fresh launch per attempt, reads a result file rather than scraping console text, retains PASS/FAIL/INVALID records, and aggregates only valid passing run values while preserving the other attempts in the full local summary.

A pre-measurement ROS graph query warms Fast DDS endpoint discovery in this WSL environment before readiness is evaluated. This is discovery stabilization, not part of the measured control path. External-force disturbance injection, communication faults, and a transport adapter remain future work.

## Milestone 3 distributed Linux integration (software preparation)

Milestone 3 introduces the first intended physical embedded-Linux participant while keeping the plant and temporary low-level controller on the laptop. The laptop launch deliberately omits the local deterministic command source; the Pi launch runs that existing source with wall time and adds a small integration witness.

```text
Laptop / WSL Ubuntu 24.04
  Gazebo + ros_gz_bridge + software_mcu_stub
       ^ /hil/sensors/* and /hil/ground_truth/odom
       |
       | ROS 2 / DDS over Ethernet
       |
       v /hil/control/target_twist and /hil/pi/status
Raspberry Pi 5 / Ubuntu Server 24.04 ARM64
  motion_test_node, use_sim_time=false
  pi_status_node + /hil/pi/ping
```

`milestone_03_laptop.launch.py` includes the existing Milestone 1 stack with `run_motion_source=false`. The default `milestone_01.launch.py` behavior remains unchanged because `run_motion_source` defaults to `true`. `hil_pi_runtime` depends on the already validated `hil_control_stub` source but does not depend on the simulator package, so a Pi build can stop at `hil_pi_runtime` without installing Gazebo.

No Pi was reachable or configured during the software-preparation run. This section describes the intended runtime and its evidence procedure; it does not claim cross-host discovery or hardware execution.

## Milestone 4A STM32 UART integration (historical transport base)

Milestone 4A adds the first concrete Linux-to-controller transport and an STM32F446RE FreeRTOS application while preserving the Milestone 1 software controller as a separate simulation path.

```text
ROS 2/Gazebo -> hil_serial_bridge -> USB serial/ST-LINK VCP
                                      -> NUCLEO-F446RE / STM32F446RE
                                         FreeRTOS control + safety
```

The exact target is the user-confirmed NUCLEO-F446RE `NUF446RE$KU1`; firmware uses USART2 on PA2/PA3 through the board's ST-LINK VCP. `hil_serial_bridge` owns the POSIX serial device, COBS framing, CRC checking, sequence accounting, ROS-to-wire conversion, and zero-output simulation mirror when the device is absent or stale.

The portable C protocol and control libraries are shared by the Linux bridge and STM32 firmware. The STM32 owns the real-time state machine, freshness watchdogs, wheel-effort computation, ACK handling, and safe zero outputs. The physical NUCLEO-F446RE target has been flashed, verified, and exercised through the bridge at the validated 115200 baud configuration.

## Current — Milestone 5A autonomy foundation

Milestone 5A adds a Pi-targeted high-level workload while preserving the validated STM32 control boundary. The laptop currently hosts Gazebo, ROS 2 autonomy, and either the software controller or the serial bridge. The physical path is:

```text
wheel states + IMU -> state_estimator -> /hil/estimate/odom
                                  -> waypoint_follower
                                  -> /hil/autonomy/raw_twist
LiDAR ---------------------------> collision_filter
                                  -> /hil/control/target_twist
                                  -> software_mcu_stub OR hil_serial_bridge
                                  -> NUCLEO-F446RE STM32 -> Gazebo wheel effort
```

The estimator subscribes only to wheel states and IMU. The waypoint follower subscribes only to estimated odometry. The collision filter subscribes to raw twist and LiDAR. `/hil/ground_truth/odom` is subscribed to by the evaluator only for aligned metrics; it is never in the autonomy control graph. The autonomy launches omit `motion_test_node`, and the STM32 launch omits `software_mcu_stub`.

The deployment seam is explicit:

```text
NOW:      Laptop = Gazebo + autonomy + bridge; STM32 = low-level control
FUTURE:   Laptop = Gazebo; Pi = same autonomy + bridge; STM32 = same control
```

Milestone 5A is wheel/IMU state estimation, waypoint following, and reactive collision stopping. It is not SLAM, Nav2, global planning, camera perception, or Raspberry Pi validation.

## Package ownership

| Package | Responsibility |
| --- | --- |
| `hil_description` | Only the primitive-geometry rover model and its SDF assets. |
| `hil_simulation` | Baseline world, bridge map, interactive/test launch files, smoke test, and timing characterization. |
| `hil_control_stub` | Temporary software MCU controller, opt-in timing diagnostics, deterministic command source, parameters, and mathematical unit tests. |
| `hil_pi_runtime` | Pi-side launch, deterministic source composition, status publisher, and diagnostic ping service; no Gazebo ownership. |
| `hil_serial_bridge` | Linux POSIX serial endpoint, shared protocol, ROS control/sensor conversion, status, and ARM/DISARM/PING services. |
| `hil_autonomy` | Wheel/IMU estimator, waypoint follower, LiDAR collision filter, autonomy launch paths, pure tests, scenario evaluator, and evidence runner. |

## Coordinate frames

The SDF uses the conventional robotics convention: `x` forward, `y` left, `z` up, in meters, seconds, radians, newtons, and newton-meters.

```text
world
  |
  +-- base_link
        +-- left_wheel
        +-- right_wheel
        +-- caster_link
        +-- imu_link
        +-- lidar_link
```

`/hil/ground_truth/odom` is a ground-truth evaluation input. It is intentionally separate from the sensor topics that a future autonomy stack may consume. This milestone does not publish a full ROS TF tree.

## Runtime interfaces

| ROS topic | ROS type | Direction | Meaning |
| --- | --- | --- | --- |
| `/hil/test/start_motion` | `std_srvs/srv/Trigger` | smoke test -> command source | Starts the one-shot deterministic profile when autostart is disabled. |
| `/hil/control/target_twist` | `geometry_msgs/msg/Twist` | test source -> stub | Desired body linear velocity and yaw rate. |
| `/hil/sensors/wheel_states` | `sensor_msgs/msg/JointState` | Gazebo -> stub/test | Left and right wheel position and angular velocity. |
| `/hil/actuator/left_effort` | `std_msgs/msg/Float64` | stub -> Gazebo | Left wheel joint force in N·m. |
| `/hil/actuator/right_effort` | `std_msgs/msg/Float64` | stub -> Gazebo | Right wheel joint force in N·m. |
| `/hil/sensors/imu` | `sensor_msgs/msg/Imu` | Gazebo -> ROS | Simulated IMU at `imu_link`. |
| `/hil/sensors/scan` | `sensor_msgs/msg/LaserScan` | Gazebo -> ROS | Simulated planar GPU LiDAR at `lidar_link`. |
| `/hil/ground_truth/odom` | `nav_msgs/msg/Odometry` | Gazebo -> test | Ground-truth 2D pose and twist in `world` / `base_link`. |

Milestone 5A autonomy interfaces:

| ROS interface | ROS type | Direction | Meaning |
| --- | --- | --- | --- |
| `/hil/estimate/odom` | `nav_msgs/msg/Odometry` | estimator -> follower/evaluator | Local wheel/IMU odometry; no ground-truth input. |
| `/hil/estimate/valid` | `std_msgs/msg/Bool` | estimator -> follower | Freshness/validity gate for estimated state. |
| `/hil/autonomy/start` | `std_srvs/srv/Trigger` | operator/evaluator -> follower | Explicitly starts waypoint tracking. |
| `/hil/autonomy/stop` | `std_srvs/srv/Trigger` | operator/evaluator -> follower | Stops tracking and publishes zero raw command. |
| `/hil/autonomy/raw_twist` | `geometry_msgs/msg/Twist` | follower -> collision filter | Unfiltered high-level body command. |
| `/hil/control/target_twist` | `geometry_msgs/msg/Twist` | collision filter -> controller backend | Exactly one intended autonomy command output. |
| `/hil/autonomy/collision_state` | `std_msgs/msg/String` | collision filter -> evaluator | `CLEAR`, `SLOWDOWN`, `STOP`, `STALE`, or `INVALID`. |

Opt-in Milestone 2 diagnostics use `std_msgs/msg/Float64`:

| ROS topic | Meaning |
| --- | --- |
| `/hil/diagnostics/control_period_ms` | Steady-clock interval between controller timer callbacks. |
| `/hil/diagnostics/control_execution_ms` | Steady-clock duration of decision, kinematics, limiting, and effort publication within one control step. |
| `/hil/diagnostics/command_age_ms` | Steady-clock age of the latest valid body command at each control step. |
| `/hil/diagnostics/feedback_age_ms` | Steady-clock age of the latest valid wheel feedback at each control step. |

The Gazebo-side command topics are `/model/hil_rover/joint/left_wheel_joint/cmd_force` and the corresponding right-wheel topic. Those are implementation details of the bridge, not the controller API.

Milestone 4A serial bridge interfaces:

| ROS interface | ROS type | Direction | Meaning |
| --- | --- | --- | --- |
| `/hil/control/target_twist` | `geometry_msgs/msg/Twist` | ROS -> bridge | Body command converted to a `CONTROL_COMMAND` frame. |
| `/hil/sensors/wheel_states` | `sensor_msgs/msg/JointState` | ROS -> bridge | Latest wheel feedback converted to a `WHEEL_FEEDBACK` frame. |
| `/hil/actuator/left_effort` | `std_msgs/msg/Float64` | bridge -> ROS | Left effort received from STM32, or zero while the serial link is not valid. |
| `/hil/actuator/right_effort` | `std_msgs/msg/Float64` | bridge -> ROS | Right effort received from STM32, or zero while the serial link is not valid. |
| `/hil/stm32/status` | `std_msgs/msg/String` | bridge -> ROS | Link, state, boot identity, and protocol counter summary. |
| `/hil/stm32/arm` | `std_srvs/srv/Trigger` | ROS -> bridge | Requests ARM after recent wheel feedback; sends a mode transaction. |
| `/hil/stm32/disarm` | `std_srvs/srv/Trigger` | ROS -> bridge | Sends DISARM and causes zero output. |
| `/hil/stm32/ping` | `std_srvs/srv/Trigger` | ROS -> bridge | Sends a tokenized PING and reports the matching PONG. |

The concrete wire contract is documented in [`docs/protocol.md`](protocol.md). It uses COBS, CRC-16/CCITT-FALSE, explicit little-endian scalar encoding, a 64-byte payload limit, and a 96-byte encoded-frame limit. The earlier transport document retains historical planning assumptions rather than hardware evidence.

## Timing and physics

The baseline world uses a provisional fixed `0.001 s` physics step and a target real-time factor of `1.0`. The controller target update is `100 Hz`; IMU is `100 Hz`; LiDAR is `10 Hz`; ground-truth odometry is `50 Hz`. These are configuration targets, not measured timing results. They must be characterized later on the target machine and under HIL conditions.

The Milestone 2 observer computes topic rates, topic inter-arrival jitter, and target-response latency in simulation time. The controller computes its own period, execution duration, and input ages with `std::chrono::steady_clock`, making those values independent of `/clock`. Repeatability distributions and their host/load metadata are documented in `milestone_02.md`; they are not portable timing guarantees. Future transport safety timers must use endpoint-local monotonic time, as defined in `transport_requirements.md`.

The physics engine is selected by Gazebo's default Harmonic physics configuration (`type="ignored"` in the SDF) while the fixed step, gravity, contact stiffness/damping, wheel radius, wheel separation, mass, and friction are explicit. The project does not claim bit-for-bit determinism.
The chassis is supported longitudinally by low-friction spherical contacts at x = +/-0.30 m, with the driven wheel contacts at x = 0. All contacts touch the z = 0 plane in the nominal pose. This puts the center-of-mass projection inside the support polygon without adding another driven or controlled joint.

## Official integration references

- [Gazebo Harmonic ROS 2 integration](https://gazebosim.org/docs/harmonic/ros2_integration/)
- [Gazebo Harmonic sensor configuration](https://gazebosim.org/docs/harmonic/sensors/)
- [Gazebo ApplyJointForce API](https://gazebosim.org/api/sim/8/jointforcecmdcomponent.html)
- [Gazebo OdometryPublisher API](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1OdometryPublisher.html)
- [ros_gz_bridge Jazzy documentation](https://docs.ros.org/en/ros2_packages/jazzy/api/ros_gz_bridge/index.html)

Milestone 3 integration interfaces:

| Interface | ROS type | Meaning |
| --- | --- | --- |
| `/hil/pi/status` | `std_msgs/msg/String` | Pi hostname, local monotonic uptime, and process startup identity for integration evidence. |
| `/hil/pi/ping` | `std_srvs/srv/Trigger` | Diagnostic service used for caller-local ROS RTT measurement. |


The Pi status and RTT interfaces are diagnostics, not authenticated device identity, health authority, or control transport. Endpoint-local monotonic clocks remain the only valid basis for future safety freshness decisions.
