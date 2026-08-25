#!/usr/bin/env python3
"""Run isolated Milestone 2 characterizations and aggregate across runs."""

import argparse
from datetime import UTC, datetime
import json
import math
import os
from pathlib import Path
import platform
import shlex
import shutil
import signal
import socket
import statistics
import subprocess
import sys
import time


SCHEMA_VERSION = 1
RUN_TIMEOUT_SEC = 120
METRIC_PATHS = {
    "controller_period_median_ms": (
        ("controller_timing", "control_period", "median_ms"), "ms"
    ),
    "controller_period_p95_ms": (
        ("controller_timing", "control_period", "p95_ms"), "ms"
    ),
    "controller_period_max_ms": (
        ("controller_timing", "control_period", "max_ms"), "ms"
    ),
    "control_execution_median_ms": (
        ("controller_timing", "control_execution", "median_ms"), "ms"
    ),
    "control_execution_p95_ms": (
        ("controller_timing", "control_execution", "p95_ms"), "ms"
    ),
    "control_execution_max_ms": (
        ("controller_timing", "control_execution", "max_ms"), "ms"
    ),
    "command_age_median_ms": (
        ("controller_timing", "command_age", "median_ms"), "ms"
    ),
    "command_age_p95_ms": (
        ("controller_timing", "command_age", "p95_ms"), "ms"
    ),
    "feedback_age_median_ms": (
        ("controller_timing", "feedback_age", "median_ms"), "ms"
    ),
    "feedback_age_p95_ms": (
        ("controller_timing", "feedback_age", "p95_ms"), "ms"
    ),
    "target_to_effort_ms": (
        ("response_latency_ms", "target_to_effort"), "ms"
    ),
    "target_to_wheel_motion_ms": (
        ("response_latency_ms", "target_to_wheel_motion"), "ms"
    ),
    "minimum_observed_margin_ms": (
        ("controller_budget", "minimum_observed_margin_ms"), "ms"
    ),
    "target_twist_rate_hz": (("topics", "target_twist", "rate_hz"), "Hz"),
    "wheel_states_rate_hz": (("topics", "wheel_states", "rate_hz"), "Hz"),
    "imu_rate_hz": (("topics", "imu", "rate_hz"), "Hz"),
    "scan_rate_hz": (("topics", "scan", "rate_hz"), "Hz"),
    "odometry_rate_hz": (("topics", "odometry", "rate_hz"), "Hz"),
    "left_effort_rate_hz": (("topics", "left_effort", "rate_hz"), "Hz"),
    "right_effort_rate_hz": (("topics", "right_effort", "rate_hz"), "Hz"),
}


def percentile(values, fraction):
    """Return the deterministic nearest-rank percentile."""
    ordered = sorted(values)
    if not ordered:
        return None
    index = max(0, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


def aggregate_values(values):
    """Summarize one scalar per successful run; P95 is across runs."""
    finite = [float(value) for value in values if value is not None and math.isfinite(value)]
    if not finite:
        return {
            "samples": 0,
            "mean": None,
            "median": None,
            "standard_deviation": None,
            "minimum": None,
            "maximum": None,
            "p95_across_runs": None,
        }
    return {
        "samples": len(finite),
        "mean": statistics.fmean(finite),
        "median": statistics.median(finite),
        "standard_deviation": statistics.stdev(finite) if len(finite) > 1 else 0.0,
        "minimum": min(finite),
        "maximum": max(finite),
        "p95_across_runs": percentile(finite, 0.95),
    }


def nested_value(document, path):
    value = document
    for key in path:
        if not isinstance(value, dict) or key not in value:
            return None
        value = value[key]
    return value


def validate_run_document(document):
    if not isinstance(document, dict):
        raise ValueError("run output is not a JSON object")
    if document.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("unsupported or missing run schema_version")
    if document.get("result") not in {"PASS", "FAIL", "INVALID"}:
        raise ValueError("run result is missing or invalid")
    for key in (
        "run_metadata",
        "topics",
        "controller_timing",
        "controller_budget",
        "response_latency_ms",
        "stationary",
        "failures",
    ):
        if key not in document:
            raise ValueError(f"run output is missing {key}")
    return document


def classify_run(document, return_code):
    result = document.get("result")
    if result == "PASS" and return_code == 0:
        return "passed"
    if result == "FAIL":
        return "failed"
    return "invalid"


def command_output(command):
    try:
        completed = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    output = (completed.stdout or completed.stderr).strip()
    return output or None


def read_os_release():
    values = {}
    try:
        for line in Path("/etc/os-release").read_text(encoding="utf-8").splitlines():
            if "=" in line:
                key, value = line.split("=", 1)
                values[key] = value.strip().strip('"')
    except OSError:
        pass
    return values


def cpu_model():
    try:
        for line in Path("/proc/cpuinfo").read_text(encoding="utf-8").splitlines():
            if line.lower().startswith("model name"):
                return line.split(":", 1)[1].strip()
    except (OSError, IndexError):
        pass
    return None


def available_memory_kib():
    try:
        for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
            if line.startswith("MemAvailable:"):
                return int(line.split()[1])
    except (OSError, ValueError, IndexError):
        pass
    return None


def git_metadata(repository_root):
    commit = command_output(["git", "-C", str(repository_root), "rev-parse", "HEAD"])
    status = command_output(
        ["git", "-C", str(repository_root), "status", "--porcelain"]
    )
    return {
        "commit_sha": commit,
        "working_tree": "dirty" if status else "clean",
    }


def environment_metadata(repository_root):
    release = platform.release()
    os_release = read_os_release()
    version_text = ""
    try:
        version_text = Path("/proc/version").read_text(encoding="utf-8").lower()
    except OSError:
        pass
    return {
        "timestamp_utc": datetime.now(UTC).isoformat(),
        "hostname": socket.gethostname(),
        "kernel_release": release,
        "os_pretty_name": os_release.get("PRETTY_NAME"),
        "execution_environment": (
            "WSL" if "microsoft" in release.lower() or "microsoft" in version_text
            else "native_or_other"
        ),
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "gazebo_version": command_output(["gz", "sim", "--version"]),
        "cpu_model": cpu_model(),
        "logical_cpu_count": os.cpu_count(),
        "available_memory_kib_at_start": available_memory_kib(),
        "git": git_metadata(repository_root),
    }


def write_json(path, document):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    temporary.replace(path)


def stop_process_group(process, initial_signal=signal.SIGINT):
    try:
        os.killpg(process.pid, initial_signal)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    if process.poll() is not None:
        time.sleep(1.0)
    else:
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    if process.poll() is None:
        process.wait(timeout=5)


def run_one(index, mode, run_directory):
    run_id = f"{mode}-{index:02d}"
    result_path = (run_directory / f"run_{index:02d}.json").resolve()
    log_path = run_directory / f"run_{index:02d}.log"
    command = [
        "ros2",
        "launch",
        "hil_simulation",
        "milestone_02_characterization.launch.py",
        f"result_path:={result_path}",
        f"run_id:={run_id}",
        f"test_mode:={mode}",
    ]
    started = time.monotonic()
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    timed_out = False
    try:
        output, _ = process.communicate(timeout=RUN_TIMEOUT_SEC)
    except subprocess.TimeoutExpired:
        timed_out = True
        stop_process_group(process)
        output = "characterization launch exceeded timeout"
    duration = time.monotonic() - started
    log_path.write_text(output or "", encoding="utf-8")

    if timed_out:
        return {
            "run_index": index,
            "run_id": run_id,
            "classification": "invalid",
            "duration_wall_sec": duration,
            "return_code": process.returncode,
            "result_file": str(result_path),
            "reason": "launch timeout",
            "document": None,
        }
    if not result_path.exists():
        return {
            "run_index": index,
            "run_id": run_id,
            "classification": "invalid",
            "duration_wall_sec": duration,
            "return_code": process.returncode,
            "result_file": str(result_path),
            "reason": "machine-readable result file was not produced",
            "document": None,
        }
    try:
        document = validate_run_document(
            json.loads(result_path.read_text(encoding="utf-8"))
        )
    except (OSError, json.JSONDecodeError, ValueError) as error:
        return {
            "run_index": index,
            "run_id": run_id,
            "classification": "invalid",
            "duration_wall_sec": duration,
            "return_code": process.returncode,
            "result_file": str(result_path),
            "reason": str(error),
            "document": None,
        }
    return {
        "run_index": index,
        "run_id": run_id,
        "classification": classify_run(document, process.returncode),
        "duration_wall_sec": duration,
        "return_code": process.returncode,
        "result_file": str(result_path),
        "reason": None,
        "document": document,
    }


def aggregate_runs(run_records):
    passed_documents = [
        record["document"]
        for record in run_records
        if record["classification"] == "passed" and record["document"] is not None
    ]
    aggregates = {}
    for name, (path, unit) in METRIC_PATHS.items():
        summary = aggregate_values(
            [nested_value(document, path) for document in passed_documents]
        )
        summary["unit"] = unit
        aggregates[name] = summary
    return aggregates


def start_load(args, expected_duration_sec):
    if args.mode != "cpu-loaded":
        return None, {"tool": None, "configuration": None}
    executable = shutil.which("stress-ng")
    if args.load_tool == "stress-ng" and executable is None:
        raise RuntimeError("stress-ng was explicitly requested but is not installed")

    if executable is None and args.load_tool == "auto":
        taskset = shutil.which("taskset")
        yes = shutil.which("yes")
        if taskset is None or yes is None:
            raise RuntimeError(
                "cpu-loaded mode requires stress-ng, or taskset and yes for the fallback"
            )
        logical_cpus = max(1, os.cpu_count() or 1)
        workers = min(args.stress_workers, logical_cpus)
        worker_commands = [
            f"{shlex.quote(taskset)} -c {cpu} {shlex.quote(yes)} >/dev/null &"
            for cpu in range(workers)
        ]
        command = [
            "bash",
            "-c",
            "trap 'exit 0' INT TERM; " + " ".join(worker_commands) + " wait",
        ]
        configuration = {
            "tool": "taskset+yes",
            "cpu_workers": workers,
            "configured_cpu_load_percent_per_worker": 100,
            "expected_host_logical_capacity_percent": 100.0 * workers / logical_cpus,
            "measured_cpu_utilization_percent": None,
            "measurement_note": "capacity fraction is analytical; utilization was not sampled",
            "command": command,
        }
    else:
        command = [
            executable,
            "--cpu",
            str(args.stress_workers),
            "--cpu-load",
            str(args.stress_load_percent),
            "--timeout",
            f"{expected_duration_sec}s",
            "--metrics-brief",
        ]
        configuration = {
            "tool": "stress-ng",
            "cpu_workers": args.stress_workers,
            "configured_cpu_load_percent_per_worker": args.stress_load_percent,
            "measured_cpu_utilization_percent": None,
            "command": command,
        }
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    time.sleep(2.0)
    if process.poll() is not None:
        output, _ = process.communicate()
        raise RuntimeError(f"controlled-load process exited before the suite: {output.strip()}")
    return process, configuration


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", type=int, default=10)
    parser.add_argument("--max-attempts", type=int)
    parser.add_argument("--mode", choices=("normal", "cpu-loaded"), default="normal")
    parser.add_argument("--load-tool", choices=("auto", "stress-ng"), default="auto")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--evidence-output", type=Path)
    parser.add_argument(
        "--stress-workers", type=int, default=max(1, (os.cpu_count() or 2) // 2)
    )
    parser.add_argument("--stress-load-percent", type=int, default=70)
    return parser.parse_args()


def main():
    args = parse_arguments()
    if args.runs <= 0:
        raise SystemExit("--runs must be positive")
    max_attempts = args.max_attempts if args.max_attempts is not None else args.runs
    if max_attempts < args.runs:
        raise SystemExit("--max-attempts must be at least --runs")
    if args.stress_workers <= 0:
        raise SystemExit("--stress-workers must be positive")
    if not 1 <= args.stress_load_percent <= 100:
        raise SystemExit("--stress-load-percent must be between 1 and 100")

    repository_root = Path(__file__).resolve().parents[1]
    timestamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
    output_directory = (
        args.output_dir
        if args.output_dir is not None
        else repository_root / "results" / "milestone_02" / "local" / f"{timestamp}-{args.mode}"
    ).resolve()
    output_directory.mkdir(parents=True, exist_ok=True)
    metadata = environment_metadata(repository_root)
    load_process = None
    load_output = None
    run_records = []
    configuration = {
        "mode": args.mode,
        "passing_runs_required": args.runs,
        "maximum_attempts": max_attempts,
        "fresh_gazebo_per_run": True,
        "run_timeout_sec": RUN_TIMEOUT_SEC,
    }

    try:
        load_process, load_configuration = start_load(
            args, max_attempts * RUN_TIMEOUT_SEC + 60
        )
        configuration["controlled_load"] = load_configuration
        for index in range(1, max_attempts + 1):
            print(f"[{index}/{max_attempts}] starting {args.mode} characterization", flush=True)
            record = run_one(index, args.mode, output_directory)
            run_records.append(record)
            print(
                f"[{index}/{max_attempts}] {record['classification'].upper()} "
                f"({record['duration_wall_sec']:.1f}s)",
                flush=True,
            )
            if sum(item["classification"] == "passed" for item in run_records) >= args.runs:
                break
    except RuntimeError as error:
        configuration["controlled_load"] = {
            "tool": args.load_tool if args.mode == "cpu-loaded" else None,
            "status": "NOT_RUN",
            "reason": str(error),
        }
    finally:
        if load_process is not None:
            stop_process_group(load_process, signal.SIGTERM)
            if load_process.stdout is not None:
                load_output = load_process.stdout.read()
                (output_directory / "controlled_load.log").write_text(
                    load_output, encoding="utf-8"
                )

    counts = {
        "attempted": len(run_records),
        "passed": sum(record["classification"] == "passed" for record in run_records),
        "failed": sum(record["classification"] == "failed" for record in run_records),
        "invalid": sum(record["classification"] == "invalid" for record in run_records),
    }
    suite_result = (
        "PASS" if counts["passed"] >= args.runs
        else "NOT_RUN" if counts["attempted"] == 0
        else "FAIL"
    )
    suite_document = {
        "schema_version": SCHEMA_VERSION,
        "suite_metadata": metadata,
        "configuration": configuration,
        "counts": counts,
        "aggregates": aggregate_runs(run_records),
        "runs": run_records,
        "result": suite_result,
    }
    summary_path = output_directory / "suite_summary.json"
    write_json(summary_path, suite_document)

    if args.evidence_output is not None:
        evidence = {key: value for key, value in suite_document.items() if key != "runs"}
        write_json(args.evidence_output.resolve(), evidence)

    print(f"suite result: {suite_result}")
    print(f"summary: {summary_path}")
    return 0 if suite_result == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
