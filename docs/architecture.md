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

Milestone 2 currently adds observability without changing the control path. When `publish_timing_diagnostics=true`, `software_mcu_stub` publishes steady-clock measurements of its control period and the ages of its latest command and wheel feedback. The default is false, so Milestone 1 interactive and smoke-test behavior is unchanged.

```text
deterministic command profile -> software_mcu_stub -> wheel effort -> Gazebo
            |                         |                   |          |
            +-------------------------+-------------------+----------+
                                      |
                                      v
                         milestone_02_characterization
                         rates + jitter + response latency
```

The characterization node observes the boundary; it does not sit in the command or actuator path. It uses the repeatable command steps as excitation. External-force disturbance injection, communication faults, and a transport adapter remain future work.

## Package ownership

| Package | Responsibility |
| --- | --- |
| `hil_description` | Only the primitive-geometry rover model and its SDF assets. |
| `hil_simulation` | Baseline world, bridge map, interactive/test launch files, smoke test, and timing characterization. |
| `hil_control_stub` | Temporary software MCU controller, opt-in timing diagnostics, deterministic command source, parameters, and mathematical unit tests. |

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

Opt-in Milestone 2 diagnostics use `std_msgs/msg/Float64`:

| ROS topic | Meaning |
| --- | --- |
| `/hil/diagnostics/control_period_ms` | Steady-clock interval between controller timer callbacks. |
| `/hil/diagnostics/command_age_ms` | Steady-clock age of the latest valid body command at each control step. |
| `/hil/diagnostics/feedback_age_ms` | Steady-clock age of the latest valid wheel feedback at each control step. |

The Gazebo-side command topics are `/model/hil_rover/joint/left_wheel_joint/cmd_force` and the corresponding right-wheel topic. Those are implementation details of the bridge, not the controller API.

## Timing and physics

The baseline world uses a provisional fixed `0.001 s` physics step and a target real-time factor of `1.0`. The controller target update is `100 Hz`; IMU is `100 Hz`; LiDAR is `10 Hz`; ground-truth odometry is `50 Hz`. These are configuration targets, not measured timing results. They must be characterized later on the target machine and under HIL conditions.

The Milestone 2 observer computes topic rates, topic inter-arrival jitter, and target-response latency in simulation time. The controller computes its own period and input ages with `std::chrono::steady_clock`, making those values independent of `/clock`. The first local run is documented in `milestone_02.md`; it is not a portable timing guarantee.

The physics engine is selected by Gazebo's default Harmonic physics configuration (`type="ignored"` in the SDF) while the fixed step, gravity, contact stiffness/damping, wheel radius, wheel separation, mass, and friction are explicit. The project does not claim bit-for-bit determinism.
The chassis is supported longitudinally by low-friction spherical contacts at x = +/-0.30 m, with the driven wheel contacts at x = 0. All contacts touch the z = 0 plane in the nominal pose. This puts the center-of-mass projection inside the support polygon without adding another driven or controlled joint.

## Official integration references

- [Gazebo Harmonic ROS 2 integration](https://gazebosim.org/docs/harmonic/ros2_integration/)
- [Gazebo Harmonic sensor configuration](https://gazebosim.org/docs/harmonic/sensors/)
- [Gazebo ApplyJointForce API](https://gazebosim.org/api/sim/8/jointforcecmdcomponent.html)
- [Gazebo OdometryPublisher API](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1OdometryPublisher.html)
- [ros_gz_bridge Jazzy documentation](https://docs.ros.org/en/ros2_packages/jazzy/api/ros_gz_bridge/index.html)
