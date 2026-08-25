#!/usr/bin/env python3
"""Run fresh-process Milestone 5A scenarios and aggregate retained JSON evidence."""

import argparse
import json
import os
import subprocess
import sys
from datetime import datetime, timezone


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", choices=("software", "stm32"), default="software")
    parser.add_argument("--scenario", choices=("open_square", "obstacle_stop", "stale_lidar"), default="open_square")
    parser.add_argument("--runs", type=int, default=None)
    parser.add_argument("--evidence-output", default=None)
    parser.add_argument("--serial-device", default="/dev/serial/by-id")
    parser.add_argument("--baud-rate", default="115200")
    parser.add_argument("--timeout-sec", type=float, default=None)
    return parser.parse_args()


def main():
    args = parse_args()
    runs = args.runs if args.runs is not None else (5 if args.backend == "software" else 3)
    if runs <= 0:
        raise ValueError("--runs must be positive")
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    evidence_dir = os.path.join(root, "results", "milestone_05a")
    local_dir = os.path.join(evidence_dir, "local")
    os.makedirs(local_dir, exist_ok=True)
    timeout_sec = args.timeout_sec
    if timeout_sec is None:
        timeout_sec = 20.0 if args.scenario == "stale_lidar" else (25.0 if args.scenario == "obstacle_stop" else 80.0)
    launch_file = "milestone_05a_stale_lidar_test.launch.py" if args.scenario == "stale_lidar" else (
        "milestone_05a_software_test.launch.py" if args.backend == "software" else "milestone_05a_stm32_test.launch.py"
    )
    records = []
    for run_number in range(1, runs + 1):
        result_path = os.path.join(local_dir, f"{args.backend}_{args.scenario}_run_{run_number:02d}.json")
        log_path = os.path.join(local_dir, f"{args.backend}_{args.scenario}_run_{run_number:02d}.log")
        command = [
            "ros2", "launch", "hil_autonomy", launch_file,
            f"scenario:={args.scenario}", f"evidence_output:={result_path}",
            f"scenario_timeout_sec:={timeout_sec}",
        ]
        if args.backend == "stm32":
            command.extend([f"serial_device:={args.serial_device}", f"baud_rate:={args.baud_rate}"])
        with open(log_path, "w", encoding="utf-8") as log_file:
            try:
                completed = subprocess.run(command, stdout=log_file, stderr=subprocess.STDOUT,
                                           check=False, timeout=timeout_sec + 45.0, text=True)
            except subprocess.TimeoutExpired as exception:
                print(f"launch shutdown timed out after evidence was written: {exception}", file=sys.stderr)
                completed = subprocess.CompletedProcess(command, -9)
        if os.path.exists(result_path):
            with open(result_path, "r", encoding="utf-8") as result_file:
                record = json.load(result_file)
        else:
            record = {
                "schema": "hil.autonomy.milestone5a.scenario.v1",
                "status": "NOT_RUN" if args.backend == "stm32" else "FAIL",
                "reason": f"launch exited {completed.returncode} without evidence JSON",
                "scenario": args.scenario,
                "backend": args.backend,
            }
        record["run_number"] = run_number
        record["launch_returncode"] = completed.returncode
        record["log_path"] = log_path
        records.append(record)
        print(json.dumps(record, sort_keys=True))

    passing = [record for record in records if record.get("status") == "PASS"]
    summary = {
        "schema": "hil.autonomy.milestone5a.repeatability.v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "backend": args.backend,
        "scenario": args.scenario,
        "requested_runs": runs,
        "passing_runs": len(passing),
        "status": "PASS" if len(passing) == runs else ("NOT_RUN" if args.backend == "stm32" and not passing else "FAIL"),
        "valid_pass_metrics": [record.get("metrics", {}) for record in passing],
        "runs": records,
    }
    output_path = args.evidence_output or os.path.join(
        evidence_dir, f"{args.backend}_{args.scenario}_summary.json"
    )
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as output_file:
        json.dump(summary, output_file, indent=2, sort_keys=True)
        output_file.write("\n")
    print(f"summary={output_path} status={summary['status']} passing={len(passing)}/{runs}")
    return 0 if summary["status"] == "PASS" else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except subprocess.TimeoutExpired as exception:
        print(f"scenario launch timed out: {exception}", file=sys.stderr)
        raise SystemExit(1)
