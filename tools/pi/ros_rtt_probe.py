#!/usr/bin/env python3
"""Measure caller-local ROS service round-trip time to the Pi status node."""

import argparse
from datetime import UTC, datetime
import json
import math
import statistics
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return None
    index = max(0, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--service", default="/hil/pi/ping")
    parser.add_argument("--samples", type=int, default=50)
    parser.add_argument("--timeout-sec", type=float, default=2.0)
    args = parser.parse_args()
    if args.samples <= 0 or args.timeout_sec <= 0.0:
        raise SystemExit("--samples and --timeout-sec must be positive")

    rclpy.init()
    node = Node("hil_pi_ros_rtt_probe")
    client = node.create_client(Trigger, args.service)
    result = {
        "schema_version": 1,
        "timestamp_utc": datetime.now(UTC).isoformat(),
        "service": args.service,
        "requested_samples": args.samples,
        "clock_domain": "caller_steady_monotonic",
    }
    try:
        if not client.wait_for_service(timeout_sec=args.timeout_sec):
            result.update({"result": "NOT_RUN", "reason": "service unavailable"})
            print(json.dumps(result, indent=2, sort_keys=True))
            return 2

        samples_ms = []
        for _ in range(args.samples):
            started_ns = time.monotonic_ns()
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(
                node, future, timeout_sec=args.timeout_sec
            )
            if not future.done():
                result.update({
                    "result": "NOT_RUN",
                    "reason": "service response timeout",
                    "successful_samples": len(samples_ms),
                })
                print(json.dumps(result, indent=2, sort_keys=True))
                return 2
            samples_ms.append((time.monotonic_ns() - started_ns) / 1e6)

        result.update({
            "result": "PASS",
            "successful_samples": len(samples_ms),
            "mean_ms": statistics.fmean(samples_ms),
            "median_ms": statistics.median(samples_ms),
            "p95_ms": percentile(samples_ms, 0.95),
            "minimum_ms": min(samples_ms),
            "maximum_ms": max(samples_ms),
        })
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
