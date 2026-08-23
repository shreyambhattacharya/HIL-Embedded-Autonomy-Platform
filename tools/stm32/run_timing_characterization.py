#!/usr/bin/env python3
"""Collect STM32 Milestone 4B timing-status evidence at one UART baud."""

from __future__ import annotations

import argparse
import json
import select
import time
from pathlib import Path
from typing import Any

from protocol import (
    ACK_OK,
    MODE_ARM,
    MODE_DISARM,
    MSG_ACK,
    MSG_CONTROL_COMMAND,
    MSG_HELLO,
    MSG_MODE_COMMAND,
    MSG_STATUS,
    MSG_TIMING_STATUS,
    MSG_WHEEL_FEEDBACK,
    FrameReader,
    SerialTransport,
    encode_frame,
    pack_control,
    pack_hello,
    pack_mode,
    pack_wheel_feedback,
    parse_ack,
    parse_status,
    parse_timing_status,
)


def send_frame(transport: SerialTransport, message_type: int, payload: bytes, sequence: int) -> int:
    transport.write(encode_frame(message_type, payload, sequence=sequence))
    return (sequence + 1) & 0xFFFFFFFF


def characterize(device: str, baud: int, duration_s: float, stream_period_s: float = 0.02) -> dict[str, Any]:
    result: dict[str, Any] = {
        "schema": "hil.stm32.milestone4b.timing.v1",
        "status": "not_run",
        "requested_device": device,
        "baud": baud,
        "duration_s": duration_s,
        "started_unix_s": time.time(),
        "timing_status": [],
        "acks": [],
        "latest_status": None,
        "checks": {},
    }
    try:
        transport = SerialTransport(device, baud)
    except (OSError, ValueError, EOFError) as error:
        result["reason"] = f"{type(error).__name__}: {error}"
        result["finished_unix_s"] = time.time()
        return result

    reader = FrameReader()
    sequence = 0
    transaction = 0
    hello_seen = False
    arm_sent = False
    arm_ack_ok = False
    disarm_ack_ok = False
    next_stream = time.monotonic()
    end_time = next_stream + duration_s
    arm_at = next_stream + min(0.5, max(0.1, duration_s / 5.0))
    timing_reports: list[dict[str, Any]] = []
    try:
        sequence = send_frame(transport, MSG_HELLO, pack_hello(), sequence)
        while time.monotonic() < end_time:
            now = time.monotonic()
            if now >= next_stream:
                sequence = send_frame(transport, MSG_CONTROL_COMMAND, pack_control(0.0, 0.0), sequence)
                sequence = send_frame(transport, MSG_WHEEL_FEEDBACK, pack_wheel_feedback(0.0, 0.0), sequence)
                next_stream = now + stream_period_s
            if not arm_sent and now >= arm_at:
                transaction += 1
                sequence = send_frame(transport, MSG_MODE_COMMAND, pack_mode(MODE_ARM, transaction), sequence)
                arm_sent = True
            ready, _, _ = select.select([transport.fd], [], [], 0.002)
            if ready:
                for frame in reader.feed(transport.read_available()):
                    if frame.message_type == MSG_HELLO:
                        hello_seen = True
                    elif frame.message_type == MSG_ACK:
                        command, ack_transaction, ack_result = parse_ack(frame.payload)
                        result["acks"].append({
                            "command": command,
                            "transaction": ack_transaction,
                            "result": ack_result,
                        })
                        if command == 7 and ack_transaction == transaction and ack_result == ACK_OK:
                            arm_ack_ok = True
                    elif frame.message_type == MSG_STATUS and len(frame.payload) >= 56:
                        result["latest_status"] = parse_status(frame.payload)
                    elif frame.message_type == MSG_TIMING_STATUS:
                        timing = parse_timing_status(frame.payload)
                        timing_reports.append(timing)
                        if len(timing_reports) > 60:
                            timing_reports.pop(0)
        if transport.fd >= 0:
            transaction += 1
            sequence = send_frame(transport, MSG_MODE_COMMAND, pack_mode(MODE_DISARM, transaction), sequence)
            disarm_deadline = time.monotonic() + 0.3
            while time.monotonic() < disarm_deadline:
                ready, _, _ = select.select([transport.fd], [], [], 0.02)
                if not ready:
                    continue
                for frame in reader.feed(transport.read_available()):
                    if frame.message_type == MSG_ACK:
                        command, ack_transaction, ack_result = parse_ack(frame.payload)
                        if command == 7 and ack_transaction == transaction and ack_result == ACK_OK:
                            disarm_ack_ok = True
    except (OSError, EOFError, TimeoutError, ValueError) as error:
        result["reason"] = f"{type(error).__name__}: {error}"
    finally:
        transport.close()

    result["timing_status"] = timing_reports
    result["checks"] = {
        "hello_seen": hello_seen,
        "timing_status_seen": bool(timing_reports),
        "arm_ack_ok": arm_ack_ok,
        "disarm_ack_ok": disarm_ack_ok,
        "decoder_errors": reader.decode_errors,
    }
    if result.get("reason"):
        result["status"] = "fail"
    elif not timing_reports:
        result["status"] = "fail"
        result["reason"] = "no TIMING_STATUS received"
    elif not arm_ack_ok or not disarm_ack_ok:
        result["status"] = "fail"
        result["reason"] = "ARM/DISARM handshake incomplete"
    else:
        result["status"] = "complete"
        execution = [int(item["exec_max_us"]) for item in timing_reports]
        periods = [int(item["period_mean_us"]) for item in timing_reports if int(item["period_mean_us"]) > 0]
        result["summary"] = {
            "reports": len(timing_reports),
            "latest_sample_count": int(timing_reports[-1]["sample_count"]),
            "observed_exec_max_us": max(execution),
            "observed_period_mean_min_us": min(periods) if periods else None,
            "observed_period_mean_max_us": max(periods) if periods else None,
            "latest_deadline_misses": int(timing_reports[-1]["deadline_misses"]),
            "latest_safety_reason": timing_reports[-1]["safety_reason_name"],
        }
    result["finished_unix_s"] = time.time()
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/serial/by-id")
    parser.add_argument("--baud", type=int, default=115200, choices=(115200, 230400, 460800, 921600))
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--stream-period", type=float, default=0.02)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--allow-not-run", action="store_true")
    args = parser.parse_args()
    result = characterize(args.device, args.baud, args.duration, args.stream_period)
    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")
    if result["status"] == "not_run" and args.allow_not_run:
        return 0
    return 0 if result["status"] == "complete" else 2


if __name__ == "__main__":
    raise SystemExit(main())
