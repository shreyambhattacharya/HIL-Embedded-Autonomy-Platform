# Milestone 2 — Software Boundary Characterization

## Status

The Milestone 2B repeatability slice is complete on the local software-only WSL host: 10 normal and 10 moderate CPU-loaded fresh-Gazebo runs passed, machine-readable aggregate evidence is checked in, and provisional Pi/STM32 transport requirements have been derived. Transport implementation, disturbance testing, and hardware validation remain future work.

## Objective

Measure the timing and response characteristics of the validated Milestone 1 software boundary before replacing the temporary controller or adding communication and fault infrastructure.

## Current scope

1. Preserve the Milestone 1 command, controller, actuator, and sensor interfaces.
2. Add controller instrumentation that is disabled by default.
3. Measure configured ROS topic rates and inter-arrival jitter.
4. Measure controller scheduling period, command age, and feedback age.
5. Measure observed target-to-effort and target-to-wheel-motion latency.
6. Use the existing deterministic profile as repeatable command excitation.
7. Emit structured JSON and fail broad regression limits.
8. Measure controller execution time separately from callback period and input age.
9. Preserve failed and infrastructure-invalid attempts rather than filtering them from evidence.

No characterization component modifies or relays commands. There is no UART, binary protocol, fake STM32, physical disturbance, communication fault injection, or HIL claim in this slice.

## Measurement definitions

| Measurement | Clock domain | Definition |
| --- | --- | --- |
| Topic rate | Gazebo simulation time | Callback count divided by elapsed observed simulation time. |
| Topic jitter | Gazebo simulation time | 95th percentile absolute deviation from configured period, or from the measured median when no configured period exists. |
| Controller period | Steady wall clock | Interval between consecutive controller timer callbacks. |
| Command age | Steady wall clock | Time from the latest valid command callback to a controller step. |
| Controller execution | Steady wall clock | Time spent in one controller step's decision, kinematics, limiting, and effort publication path; diagnostic publication is excluded. |
| Feedback age | Steady wall clock | Time from the latest valid wheel callback to a controller step. |
| Target-to-effort | Gazebo simulation time | First active target observed to first effort above `0.05 N m`. |
| Target-to-wheel motion | Gazebo simulation time | First active target observed to first wheel speed above `0.05 rad/s`. |

The observer-based response measurements include ROS delivery and callback ordering. Their simulation-time resolution is bounded by the `0.001 s` physics step. They are not wire-level or hardware interrupt latency measurements.

## Run procedure

```bash
source /opt/ros/jazzy/setup.bash
cd <repository>/ros2_ws
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ros2 launch hil_simulation milestone_02_characterization.launch.py
```

The launch starts the existing stack headlessly with motion autostart disabled and diagnostics enabled. The observer waits for all interfaces, records a stationary window, triggers the profile, measures through the final stop, prints JSON, and requests clean Gazebo shutdown.

For repeatable fresh-process suites, use `tools/run_characterization_suite.py`. `--runs` is the required passing sample count; optional `--max-attempts` permits bounded replacements while retaining every failed or invalid attempt. Raw outputs default to the ignored `results/milestone_02/local/` directory, and `--evidence-output` writes a compact checked-in aggregate. Exact commands and results are in `docs/milestone_02_repeatability.md`.

The loaded mode prefers `stress-ng` when installed. This host used a recorded `taskset` + `yes` fallback without installing software or claiming measured CPU utilization.

## Historical single-run baseline

Environment: WSL Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic. One clean run produced:

| Metric | Observation |
| --- | ---: |
| Controller period median | `10.000 ms` |
| Controller period p95 | `10.086 ms` |
| Controller period p95 absolute jitter from 10 ms | `0.108 ms` |
| Command age median / p95 | `28.621 / 48.672 ms` |
| Feedback age median / p95 | `0.401 / 1.208 ms` |
| Target-to-effort response | `9 ms` |
| Target-to-wheel-motion response | `27 ms` |
| Target command rate | `20.039 Hz` |
| Left/right effort rate | `100.194 Hz` |
| Wheel-state rate | `999.941 Hz` |
| IMU rate | `100.000 Hz` |
| LiDAR rate | `10.002 Hz` |
| Ground-truth odometry rate | `49.997 Hz` |

These observations characterize one run on one host. They are not guarantees, target-hardware benchmarks, or evidence of real-time scheduling.

## Initial regression limits

The executable currently requires minimum rates of 15 Hz for target commands, 50 Hz for wheel states, 80 Hz for IMU and each effort channel, 8 Hz for LiDAR, and 40 Hz for odometry. It also requires controller period p95 at or below 20 ms, command and feedback age p95 at or below 100 ms, target-to-effort response at or below 100 ms, target-to-wheel response at or below 500 ms, and a stationary wheel speed and effort at or below `0.02` in their respective units.

These are regression tripwires, not final real-time requirements. Final limits must be derived from repeated host runs and the eventual control and safety analysis.

## Repeatability result

The selected evidence contains 10/10 normal passes and 10/10 moderate controlled-load passes, each with a fresh Gazebo launch. Across the loaded runs, the across-run P95 of each run's execution-time P95 was 0.544 ms, the largest observed execution sample was 4.097 ms, and the worst observed execution margin within the nominal 10 ms budget was 5.903 ms. The largest target-to-effort and target-to-wheel-motion observations were 11 ms and 30 ms in the loaded set.

Earlier exploratory attempts that exposed DDS discovery and timeout-cleanup problems remain preserved locally and are described rather than hidden. Full mean, median, sample stdev, min, max, nearest-rank P95, topic-rate, response, environment, and load details are in `docs/milestone_02_repeatability.md` and the two JSON summaries under `results/milestone_02/`.

The derived future transport envelope, freshness rules, sequence/timestamp/acknowledgement requirements, and analytical 115200/460800/921600 UART budgets are in `docs/transport_requirements.md`. They are provisional and do not claim transport implementation.

## Acceptance status

- [x] Instrumentation is opt-in and does not alter the normal Milestone 1 path.
- [x] Existing build and 12-test aggregate pass.
- [x] Self-contained characterization launch exits cleanly.
- [x] Initial topic-rate, timing, latency, and stationary checks pass.
- [x] Repeat characterization across 10 normal and 10 moderate loaded fresh-process runs.
- [x] Record machine-readable mean, median, sample stdev, min, max, and across-run P95 aggregates.
- [x] Derive a clearly labeled provisional transport envelope and UART bandwidth budget.
- [x] Add build/unit CI while keeping Gazebo integration local.
- [ ] Add repeatable external disturbances only after baseline repeatability is established.
- [ ] Validate the transport and controller on actual Pi/STM32 hardware.
