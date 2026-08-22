# Milestone 2 results

`local/` contains raw per-run JSON, launch logs, stress-tool output, and full suite summaries. It is ignored because those artifacts are host-specific and can grow across repeated experiments.

Compact aggregate evidence files may be kept beside this README when they are intentionally generated with `--evidence-output`. They contain environment metadata, experiment configuration, pass/fail counts, and across-run statistics but omit raw run records and ROS/Gazebo logs.

Run a suite after building and sourcing the workspace:

```bash
python3 tools/run_characterization_suite.py --runs 10 --mode normal
```

`--runs` specifies the required passing count. Use a bounded number of replacement attempts without hiding invalid records:

```bash
python3 tools/run_characterization_suite.py \
  --runs 10 --max-attempts 12 \
  --mode cpu-loaded --stress-workers 4
```

Loaded mode uses `stress-ng` when it is installed. Otherwise `--load-tool auto` uses pinned `taskset` + `yes` workers and records the analytical logical-capacity fraction; it does not claim measured utilization or silently install packages. Use `--load-tool stress-ng` to require that specific tool and obtain `NOT_RUN` when it is absent.
