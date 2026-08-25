#!/usr/bin/env python3
"""Run bounded Milestone 4B hardware checks with honest gate semantics."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from run_timing_characterization import characterize


ALLOWED_BAUDS = {115200, 230400, 460800, 921600}


def parse_bauds(value: str) -> list[int]:
    bauds = [int(part.strip()) for part in value.split(",") if part.strip()]
    if not bauds or any(baud not in ALLOWED_BAUDS for baud in bauds):
        raise argparse.ArgumentTypeError(
            "bauds must be a comma-separated subset of 115200,230400,460800,921600")
    return bauds


def validation_decision(cases: list[dict[str, Any]]) -> str:
    """Return PASS, FAIL, or NOT_RUN based only on required cases."""
    required = [case for case in cases if case.get("required", False)]
    if any(case.get("status") == "fail" for case in required):
        return "FAIL"
    if not required or any(case.get("status") != "complete" for case in required):
        return "NOT_RUN"
    return "PASS"


def run_validation(
    device: str,
    required_baud: int = 115200,
    experimental_bauds: list[int] | None = None,
    duration_s: float = 60.0,
    command_rate_hz: float = 100.0,
    feedback_rate_hz: float = 100.0,
) -> dict[str, Any]:
    experimental = [] if experimental_bauds is None else experimental_bauds
    bauds = [required_baud] + [baud for baud in experimental if baud != required_baud]
    cases = []
    for baud in bauds:
        case = characterize(
            device, baud, duration_s, command_rate_hz, feedback_rate_hz)
        case["required"] = baud == required_baud
        case["classification"] = "required_operational" if case["required"] else "experimental"
        cases.append(case)
    decision = validation_decision(cases)
    experimental_failures = [
        case["baud"] for case in cases
        if not case["required"] and case["status"] in ("fail", "not_run")
    ]
    comparison_rows = []
    for case in cases:
        summary = case.get("summary", {})
        comparison_rows.append({
            "baud": case["baud"],
            "required": case["required"],
            "status": case["status"],
            "execution_max_us": summary.get("execution_max_us"),
            "period_mean_us": summary.get("period_mean_us"),
            "deadline_misses_delta": summary.get("deadline_misses_delta"),
        })
    return {
        "schema": "hil.stm32.milestone4b.validation.v2",
        "decision": decision,
        "requested_device": device,
        "required_baud": required_baud,
        "experimental_bauds": experimental,
        "duration_s": duration_s,
        "command_rate_hz": command_rate_hz,
        "feedback_rate_hz": feedback_rate_hz,
        "cases": cases,
        "comparison": {"cases": comparison_rows},
        "experimental_failures_are_informational": experimental_failures,
        "watchdog_test": {
            "status": "not_run",
            "reason": "requires a separately built HIL_TEST_WATCHDOG=1 image and an intentional reset observation",
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/serial/by-id")
    parser.add_argument("--required-baud", type=int, default=115200,
                        choices=tuple(sorted(ALLOWED_BAUDS)))
    parser.add_argument("--experimental-bauds", type=parse_bauds, default=[460800])
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--command-rate-hz", type=float, default=100.0)
    parser.add_argument("--feedback-rate-hz", type=float, default=100.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--allow-not-run", action="store_true",
                        help="explicitly allow a required case that could not run")
    parser.add_argument("--allow-partial", action="store_true",
                        help="explicitly allow a non-PASS gate for exploratory automation")
    args = parser.parse_args()
    result = run_validation(
        args.device, args.required_baud, args.experimental_bauds, args.duration,
        args.command_rate_hz, args.feedback_rate_hz)
    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")
    if result["decision"] == "PASS":
        return 0
    if result["decision"] == "NOT_RUN" and (args.allow_not_run or args.allow_partial):
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
