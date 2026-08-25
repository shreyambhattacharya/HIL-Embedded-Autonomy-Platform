# Raspberry Pi 5 Setup for Milestone 3

## Scope

This procedure prepares a headless Raspberry Pi 5 to participate in the ROS 2/DDS simulation link. It does not install or run Gazebo, RViz, a desktop environment, GPU simulation, UART, STM32 firmware, or a custom transport. The Pi runs the deterministic target-twist source and the small `/hil/pi/status` identity node; the laptop remains the simulator and low-level software-controller host.

No Pi was configured or reached from the development environment used for Milestone 3 preparation. Host-specific commands below are therefore a reproducible procedure, not a hardware validation result.

## Supported target

- Hardware: Raspberry Pi 5.
- OS: Ubuntu Server 24.04 LTS ARM64 (`aarch64`).
- ROS: ROS 2 Jazzy.
- Network: wired Ethernet for the baseline; Wi-Fi is outside this milestone.
- Suggested hostname: `hil-pi5`, if that name is available on the local network. Do not assume it resolves; use the actual configured hostname or a user-provided address.

The Pi package boundary is `hil_pi_runtime` plus its `hil_control_stub` dependency. The targeted build does not require `hil_simulation`, Gazebo, `ros_gz_bridge`, or the rover model.

## Flashing and first boot assumptions

Flash Ubuntu Server 24.04 LTS ARM64 using the user's approved Raspberry Pi imaging workflow. During first boot:

1. Set a unique hostname, for example `hil-pi5`.
2. Create a normal non-root user with `sudo` access.
3. Connect the Pi to the same wired Ethernet network as the laptop.
4. Apply Ubuntu updates and reboot if the kernel or firmware was updated.
5. Record the actual hostname with `hostnamectl`; do not put a personal IP address in the repository.

Do not put passwords, SSH private keys, access tokens, or personal addresses in this repository.

## SSH setup

From the laptop, create an SSH key if one is not already available, copy the public key to the Pi using the user's normal secure process, and verify:

```bash
ssh <pi-user>@<pi-host>
uname -m
hostnamectl
```

The expected architecture output is `aarch64`. The repository's helper scripts do not assume SSH credentials or automatically start remote processes.

For optional deployment helpers, configure the values only in the shell environment on the laptop:

```bash
export HIL_PI_HOST=<pi-host-or-address>
export HIL_PI_USER=<pi-user>
```

If either value is absent, Pi-dependent helpers must be treated as `NOT RUN`; there is no repository default.

## ROS installation

Configure the official ROS 2 Jazzy Ubuntu 24.04 ARM64 apt repository according to the ROS documentation, then install ROS base and the project prerequisites. The repository includes a deliberately narrow helper:

```bash
cd <repo>/tools/pi
./bootstrap_pi.sh
```

The helper checks for `/opt/ros/jazzy/setup.bash`, installs `ros-jazzy-ros-base`, `build-essential`, Git, `rosdep`, and colcon extensions, and does not alter networking. It does not silently install Gazebo or a desktop stack.

If rosdep has never been initialized on the machine, initialize it using the standard ROS installation procedure. Then update the user's rosdep cache:

```bash
rosdep update
```

## Clone and build

Clone the validated branch, not `main`:

```bash
git clone --branch milestone-03-pi-integration \
  https://github.com/shreyambhattacharya/HIL-Embedded-Autonomy-Platform.git
cd HIL-Embedded-Autonomy-Platform
```

Build only the Pi-targeted dependency closure:

```bash
./tools/pi/build_pi.sh
source ros2_ws/install/setup.bash
```

The script runs rosdep only over `hil_control_stub` and `hil_pi_runtime`, then uses `colcon build --packages-up-to hil_pi_runtime`. It is idempotent at the workspace level and does not copy files one by one. The build is the point at which ARM64 compatibility is established; a laptop build is not a Pi build.

## ROS domain and middleware

Use the same domain on both hosts. The project setup default is the documented value `42`; it is environment configuration, not a C++ constant:

```bash
export ROS_DOMAIN_ID=42
```

Set it in the shell that starts both the laptop and Pi processes. Keep `ROS_LOCALHOST_ONLY` unset or `0` for cross-host operation.

Use the repository's current/default ROS 2 middleware first. Do not switch RMW implementations merely for variety. If discovery fails, diagnose network and firewall behavior before considering another supported RMW such as Cyclone DDS. Any future RMW selection must be recorded with its reason and configuration.

## Wired Ethernet and firewall checks

Prefer:

```text
Pi <---- wired Ethernet ----> laptop or local switch
```

Confirm both hosts have addresses on the intended interface and can reach each other. Confirm the host firewall permits the ROS 2 DDS UDP traffic and multicast on that interface. If `ufw` is enabled, inspect its status and add a narrowly scoped rule for the trusted lab network according to local policy; do not disable the firewall blindly. WSL networking mode, Windows firewall behavior, and multicast support must be verified on the actual laptop.

WSL has shown discovery variability in earlier local experiments. If WSL cannot provide a reliable cross-host path, document the observed limitation and test native Ubuntu or an explicitly supported WSL networking mode rather than adding unbounded retry hacks.

## Time synchronization

Enable normal NTP or chrony synchronization on both hosts for log correlation:

```bash
timedatectl status
```

Clock synchronization is not a control-safety mechanism. The future Pi-to-STM32 protocol must continue to use endpoint-local monotonic clocks for freshness and watchdog behavior. Do not infer one-way network latency from synchronized wall clocks.

## Pi launch

After sourcing ROS and the workspace:

```bash
export ROS_DOMAIN_ID=42
ros2 launch hil_pi_runtime milestone_03_pi.launch.py autostart:=false
```

This starts the existing deterministic `motion_test_node` with wall time (`use_sim_time=false`) and `pi_status_node`. It does not start Gazebo or the software controller. Trigger the profile from the laptop or Pi only after verifying the graph:

```bash
ros2 node list
ros2 topic echo /hil/pi/status
ros2 service list | grep '/hil/pi/ping\|/hil/test/start_motion'
ros2 service call /hil/test/start_motion std_srvs/srv/Trigger '{}'
```

The Pi source publishes `/hil/control/target_twist` and preserves the existing `/hil/test/start_motion` Trigger behavior.

## Laptop launch

On the laptop, source ROS and the workspace, set the same domain, and run:

```bash
export ROS_DOMAIN_ID=42
ros2 launch hil_simulation milestone_03_laptop.launch.py
```

This starts Gazebo, `ros_gz_bridge`, and `software_mcu_stub` with no local `motion_test_node`. The default Milestone 1 launch remains unchanged and still starts its local source because `run_motion_source` defaults to `true`.

## Sensor and command checks

On the Pi, verify laptop-to-Pi visibility:

```bash
ros2 topic echo /hil/sensors/wheel_states
ros2 topic echo /hil/sensors/imu
ros2 topic echo /hil/sensors/scan
ros2 topic echo /hil/ground_truth/odom
```

Ground-truth odometry is test-only and must not become an autonomy input.

On the laptop, verify the Pi publisher:

```bash
ros2 topic info /hil/control/target_twist --verbose
ros2 topic echo /hil/control/target_twist
ros2 topic echo /hil/pi/status
```

The `/hil/pi/status` message includes only the Pi hostname, local monotonic uptime, and a process startup identity. It is an integration witness, not a security or hardware-health protocol.

## RTT probe

From the laptop, after `/hil/pi/ping` is visible:

```bash
python3 tools/pi/ros_rtt_probe.py --samples 50
```

The probe measures caller-local steady-clock service RTT and reports mean, median, P95, minimum, and maximum. It never reports one-way latency. If the service is unavailable or a response times out, it emits `NOT_RUN` and exits nonzero.

ICMP measurements are also caller-local:

```bash
ping -c 50 <pi-host>
```

Record the actual host, interface, packet loss, median, P95, and maximum in the Milestone 3 report; do not copy values from another network.

## Failure test

With the laptop stack active and the Pi source publishing, stop only the Pi `motion_test_node` process. The laptop controller should continue to apply its existing temporary command-staleness behavior and publish zero left/right effort after the configured `command_timeout_sec` (currently 0.5 s). Measure this on the laptop using one local steady clock if the test is executed.

This is temporary software command-staleness behavior. It is not the independent STM32 watchdog or a hardware-safety claim.
