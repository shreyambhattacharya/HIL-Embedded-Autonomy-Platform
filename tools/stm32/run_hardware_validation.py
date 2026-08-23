#!/usr/bin/env python3
"""Run bounded Milestone 4B hardware checks and emit machine-readable evidence."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from run_timing_characterization import characterize


def parse_bauds(value: str) -> list[int]:
    bauds = [int(part.strip()) for part in value.split(",") if part.strip()]
    allowed = {115200, 230400, 460800, 921600}
    if not bauds or any(baud not in allowed for baud in bauds):
        raise argparse.ArgumentTypeError("bauds must be a comma-separated subset of 115200,230400,460800,921600")
    return bauds


def run_validation(device: str, bauds: list[int], duration_s: float) -> dict[str, Any]:
    cases = []
    for baud in bauds:
        cases.append(characterize(device, baud, duration_s))
    complete = [case for case in cases if case["status"] == "complete"]
    not_run = [case for case in cases if case["status"] == "not_run"]
    failed = [case for case in cases if case["status"] == "fail"]
    comparison: dict[str, Any] = {"status": "not_run", "reason": "fewer than two completed baud cases"}
    if len(complete) >= 2:
        rows = []
        for case in complete:
            summary = case.get("summary", {})
            rows.append({
                "baud": case["baud"],
                "observed_exec_max_us": summary.get("observed_exec_max_us"),
                "observed_period_mean_min_us": summary.get("observed_period_mean_min_us"),
                "observed_period_mean_max_us": summary.get("observed_period_mean_max_us"),
                "latest_deadline_misses": summary.get("latest_deadline_misses"),
            })
        comparison = {"status": "complete", "cases": rows}
    if failed:
        decision = "fail"
    elif complete and not_run:
        decision = "partial"
    elif complete:
        decision = "complete"
    else:
        decision = "not_run"
    return {
        "schema": "hil.stm32.milestone4b.validation.v1",
        "decision": decision,
        "requested_device": device,
        "bauds": bauds,
        "duration_s": duration_s,
        "cases": cases,
        "comparison": comparison,
        "watchdog_test": {
            "status": "not_run",
            "reason": "requires a separately built HIL_TEST_WATCHDOG=1 image and an intentional reset observation",
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/serial/by-id")
    parser.add_argument("--bauds", type=parse_bauds, default=[115200, 460800])
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--allow-not-run", action="store_true")
    args = parser.parse_args()
    result = run_validation(args.device, args.bauds, args.duration)
    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")
    if result["decision"] == "not_run" and args.allow_not_run:
        return 0
    return 0 if result["decision"] in ("complete", "partial") else 2


if __name__ == "__main__":
    raise SystemExit(main())
