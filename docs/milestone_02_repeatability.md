# Milestone 2B Repeatability Evidence

## Environment and method

The selected evidence was collected on Ubuntu 24.04.4 LTS in WSL2, ROS 2 Jazzy, Gazebo Sim 8.11.0, Linux `6.18.33.2-microsoft-standard-WSL2`, Intel Core Ultra 7 155H, and 22 logical CPUs. The source revision was `2f2c483618bafd5f2f046dc6bdd24077a239545c` with the Milestone 2B working tree intentionally dirty. Every run launched a fresh Gazebo process.

Normal condition: 10 attempted, 10 passed, 0 failed, 0 invalid. Loaded condition: 10 attempted, 10 passed, 0 failed, 0 invalid. The selected loaded set used four pinned 100%-duty workers, analytically 18.18% of host logical CPU capacity. Actual CPU utilization was not sampled and is not claimed.

Each row below aggregates one named scalar from each of 10 runs. P95 is deterministic nearest-rank across those ten run-level values; it is distinct from a within-run P95. The complete machine-readable aggregates are `results/milestone_02/normal_summary.json` and `results/milestone_02/cpu_loaded_summary.json`.

## Timing and execution budget

| Condition / run-level metric (ms) | Mean | Median | Stdev | Min | Max | P95 across runs |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Normal: control-period median | 9.9998 | 9.9999 | 0.0004 | 9.9991 | 10.0005 | 10.0005 |
| Normal: control-period p95 | 10.0754 | 10.0753 | 0.0090 | 10.0594 | 10.0880 | 10.0880 |
| Normal: execution median | 0.1290 | 0.1258 | 0.0122 | 0.1069 | 0.1490 | 0.1490 |
| Normal: execution p95 | 0.2870 | 0.2822 | 0.0128 | 0.2708 | 0.3102 | 0.3102 |
| Normal: execution max | 0.7805 | 0.7974 | 0.1702 | 0.5515 | 1.0816 | 1.0816 |
| Loaded: control-period median | 9.9999 | 9.9998 | 0.0008 | 9.9988 | 10.0011 | 10.0011 |
| Loaded: control-period p95 | 10.1039 | 10.0988 | 0.0169 | 10.0862 | 10.1396 | 10.1396 |
| Loaded: execution median | 0.1617 | 0.1630 | 0.0109 | 0.1444 | 0.1782 | 0.1782 |
| Loaded: execution p95 | 0.3351 | 0.3191 | 0.0758 | 0.2753 | 0.5438 | 0.5438 |
| Loaded: execution max | 1.9095 | 1.9034 | 0.9506 | 0.8427 | 4.0969 | 4.0969 |

The worst observed execution sample left 8.918 ms normal and 5.903 ms loaded inside the nominal 10 ms execution budget. This is host margin, not a real-time guarantee: Linux/WSL scheduling produced isolated callback periods as large as 19.160 ms normal and 23.636 ms loaded even though measured controller work stayed below budget.

## Response and freshness

| Condition / run-level metric (ms) | Mean | Median | Stdev | Min | Max | P95 across runs |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Normal: target to effort | 5.9 | 5.0 | 2.77 | 3.0 | 12.0 | 12.0 |
| Normal: target to wheel motion | 23.2 | 23.0 | 1.55 | 21.0 | 26.0 | 26.0 |
| Normal: command-age p95 | 44.874 | 44.780 | 1.464 | 42.438 | 47.705 | 47.705 |
| Normal: feedback-age p95 | 1.045 | 1.038 | 0.051 | 0.950 | 1.141 | 1.141 |
| Loaded: target to effort | 5.6 | 4.5 | 3.60 | 1.0 | 11.0 | 11.0 |
| Loaded: target to wheel motion | 23.5 | 22.0 | 3.92 | 18.0 | 30.0 | 30.0 |
| Loaded: command-age p95 | 45.305 | 44.090 | 3.530 | 40.494 | 49.672 | 49.672 |
| Loaded: feedback-age p95 | 1.418 | 1.195 | 0.392 | 1.077 | 2.182 | 2.182 |

## Topic-rate repeatability

| Topic | Normal mean Hz (run min–max) | Loaded mean Hz (run min–max) |
| --- | ---: | ---: |
| Target twist | 20.085 (20.026–20.197) | 20.826 (20.051–22.335) |
| Wheel states | 997.439 (994.301–1000.059) | 983.584 (957.313–999.295) |
| IMU | 100.001 (99.994–100.006) | 100.003 (100.000–100.006) |
| LiDAR | 10.000 (9.995–10.005) | 9.999 (9.995–10.004) |
| Odometry | 50.001 (49.997–50.009) | 50.000 (49.994–50.003) |
| Left/right effort | 100.424 / 100.425 | 104.127 / 104.128 |

## Invalid-attempt history

Exploratory fresh-process runs exposed intermittent Fast DDS endpoint discovery in WSL. Retained invalid records identify missing target discovery, Trigger timeouts, and three 120-second launch timeouts under an aggressive eight-worker load. An explicit pre-measurement graph-discovery warm-up made the normal set repeatable.

The aggressive eight-worker experiment retained 5 passes and 5 infrastructure-invalid attempts; there were no threshold failures. A leaked Gazebo child from the first timeout-cleanup implementation was identified, stopped, and the process-group cleanup was hardened. A moderate four-worker preliminary set retained 9 passes and 1 Trigger-timeout invalid. The final selected normal and four-worker suites each completed 10 passes with no invalid attempts. Exploratory raw JSON and logs remain under the ignored `results/milestone_02/local/` directory.

## Interpretation

The control computation has substantial observed margin on this host, including under moderate pinned CPU contention. Callback-period maxima still demonstrate that this general-purpose WSL/Linux environment is not a hard-real-time platform. Topic and response values are observer measurements that include ROS delivery and callback ordering; they are not UART or interrupt latency.

The results support a future 100 Hz command/feedback transport envelope and bounded freshness requirements, but do not select a packet format or baud rate. See `docs/transport_requirements.md` for measured-versus-derived labels, clock-domain rules, acknowledgement classes, and analytical UART budgets.

## Reproduction

After building and sourcing ROS 2 plus `ros2_ws/install/setup.bash`:

```bash
python3 tools/run_characterization_suite.py \
  --runs 10 --mode normal \
  --evidence-output results/milestone_02/normal_summary.json

python3 tools/run_characterization_suite.py \
  --runs 10 --max-attempts 12 \
  --mode cpu-loaded --stress-workers 4 \
  --evidence-output results/milestone_02/cpu_loaded_summary.json
```

`--runs` is the required passing count. `--max-attempts` permits bounded replacement attempts while retaining every failed or invalid attempt in the full local summary. `stress-ng` is preferred when installed; this host used the recorded `taskset` + `yes` fallback because installing packages required unavailable interactive sudo authentication.
