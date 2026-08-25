#!/usr/bin/env python3
"""Collect STM32 Milestone 4B timing and per-run transport evidence."""

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
    ROLE_STM32,
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

COUNTER_FIELDS = (
    "rx_valid",
    "rx_crc",
    "rx_decode",
    "rx_length",
    "rx_version",
    "rx_duplicate",
    "rx_stale",
    "rx_gaps",
    "rx_stream_drops",
    "tx_queue_drops",
    "uart_overruns",
)
TIMING_COUNTER_FIELDS = (
    "deadline_misses",
    "rx_stream_drops",
    "tx_queue_drops",
    "uart_overruns",
)
TRANSPORT_ERROR_FIELDS = (
    "rx_crc",
    "rx_decode",
    "rx_length",
    "rx_version",
    "rx_duplicate",
    "rx_stale",
    "rx_gaps",
    "rx_stream_drops",
    "tx_queue_drops",
    "uart_overruns",
)


def uint32_delta(final: int, baseline: int) -> int:
    """Return unsigned 32-bit counter subtraction, including wraparound."""
    return (int(final) - int(baseline)) & 0xFFFFFFFF


def counter_delta(final: dict[str, int], baseline: dict[str, int]) -> dict[str, int]:
    return {
        field: uint32_delta(final.get(field, 0), baseline.get(field, 0))
        for field in COUNTER_FIELDS
    }


def timing_counter_delta(final: dict[str, Any], baseline: dict[str, Any]) -> dict[str, int]:
    return {
        field: uint32_delta(int(final.get(field, 0)), int(baseline.get(field, 0)))
        for field in TIMING_COUNTER_FIELDS
    }


def send_frame(transport: SerialTransport, message_type: int, payload: bytes, sequence: int) -> int:
    transport.write(encode_frame(message_type, payload, sequence=sequence))
    return (sequence + 1) & 0xFFFFFFFF


def characterize(
    device: str,
    baud: int,
    duration_s: float,
    command_rate_hz: float = 100.0,
    feedback_rate_hz: float = 100.0,
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "schema": "hil.stm32.milestone4b.timing.v2",
        "status": "not_run",
        "requested_device": device,
        "baud": baud,
        "duration_s": duration_s,
        "command_rate_hz": command_rate_hz,
        "feedback_rate_hz": feedback_rate_hz,
        "started_unix_s": time.time(),
        "timing_status": [],
        "acks": [],
        "baseline_status": None,
        "final_status": None,
        "latest_status": None,
        "transport_counters": {"baseline": None, "final": None, "delta": None},
        "timing_counters": {"baseline": None, "final": None, "delta": None},
        "host_decoder_errors": {"baseline": None, "final": None, "delta": None},
        "checks": {},
    }
    if duration_s <= 0.0 or command_rate_hz <= 0.0 or feedback_rate_hz <= 0.0:
        result["reason"] = "duration and stream rates must be positive"
        result["finished_unix_s"] = time.time()
        return result
    try:
        transport = SerialTransport(device, baud)
    except (OSError, ValueError, EOFError) as error:
        result["reason"] = f"{type(error).__name__}: {error}"
        result["finished_unix_s"] = time.time()
        return result

    reader = FrameReader()
    sequence = 0
    transaction = 0
    stm32_hello_seen = False
    arm_sent = False
    arm_ack_ok = False
    disarm_ack_ok = False
    baseline_status: dict[str, int] | None = None
    baseline_timing: dict[str, Any] | None = None
    status_samples = 0
    timing_reports: list[dict[str, Any]] = []
    sync_deadline = time.monotonic() + max(5.0, min(15.0, duration_s / 2.0))

    def consume_frames(data: bytes, measured: bool = False) -> None:
        nonlocal stm32_hello_seen, baseline_status, baseline_timing, status_samples
        nonlocal arm_ack_ok, disarm_ack_ok
        for frame in reader.feed(data):
            if frame.message_type == MSG_HELLO:
                if len(frame.payload) >= 2 and frame.payload[0] == ROLE_STM32:
                    stm32_hello_seen = True
            elif frame.message_type == MSG_ACK:
                command, ack_transaction, ack_result = parse_ack(frame.payload)
                result["acks"].append({
                    "command": command,
                    "transaction": ack_transaction,
                    "result": ack_result,
                })
                if command == MSG_MODE_COMMAND and ack_transaction == transaction:
                    if ack_result == ACK_OK and arm_sent:
                        arm_ack_ok = True
                    if ack_result == ACK_OK and not arm_sent:
                        disarm_ack_ok = True
            elif frame.message_type == MSG_STATUS and len(frame.payload) >= 56:
                status = parse_status(frame.payload)
                result["latest_status"] = status
                if not measured and stm32_hello_seen:
                    if baseline_status is None or status["boot_id"] == baseline_status["boot_id"]:
                        baseline_status = status
                        status_samples += 1
            elif frame.message_type == MSG_TIMING_STATUS:
                timing = parse_timing_status(frame.payload)
                if measured:
                    timing_reports.append(timing)
                    if len(timing_reports) > 120:
                        timing_reports.pop(0)
                elif stm32_hello_seen and baseline_timing is None:
                    baseline_timing = timing

    measured_start: float | None = None
    try:
        # The STM32 responds to this HELLO. Waiting for that response makes the
        # status baseline session-bound instead of dependent on open timing.
        sequence = send_frame(transport, MSG_HELLO, pack_hello(), sequence)
        next_sync_hello = time.monotonic() + 0.25
        while time.monotonic() < sync_deadline and (
            not stm32_hello_seen or status_samples < 2 or baseline_timing is None
        ):
            now = time.monotonic()
            if now >= next_sync_hello:
                sequence = send_frame(transport, MSG_HELLO, pack_hello(), sequence)
                next_sync_hello = now + 0.25
            ready, _, _ = select.select([transport.fd], [], [], 0.02)
            if ready:
                consume_frames(transport.read_available())
        if not stm32_hello_seen:
            raise TimeoutError("STM32 HELLO was not observed after host HELLO")
        if baseline_status is None or baseline_timing is None:
            raise TimeoutError("stable STATUS and TIMING_STATUS baseline was not observed")

        # Let final synchronization bytes drain, then use a STATUS emitted
        # after the quiet window as the session-bound measurement baseline.
        quiesce_deadline = time.monotonic() + 0.3
        while time.monotonic() < quiesce_deadline:
            ready, _, _ = select.select([transport.fd], [], [], 0.02)
            if ready:
                consume_frames(transport.read_available())
        status_target = status_samples + 1
        status_deadline = time.monotonic() + 0.3
        while time.monotonic() < status_deadline and status_samples < status_target:
            ready, _, _ = select.select([transport.fd], [], [], 0.02)
            if ready:
                consume_frames(transport.read_available())
        if status_samples < status_target:
            raise TimeoutError("post-quiescence STATUS baseline was not observed")

        result["baseline_status"] = baseline_status
        result["transport_counters"]["baseline"] = {
            field: int(baseline_status.get(field, 0)) for field in COUNTER_FIELDS
        }
        result["timing_counters"]["baseline"] = {
            field: int(baseline_timing.get(field, 0)) for field in TIMING_COUNTER_FIELDS
        }
        result["host_decoder_errors"]["baseline"] = reader.decode_errors
        measured_start = time.monotonic()
        end_time = measured_start + duration_s
        command_period = 1.0 / command_rate_hz
        feedback_period = 1.0 / feedback_rate_hz
        next_command = measured_start
        next_feedback = measured_start
        arm_at = measured_start + min(0.5, max(0.1, duration_s / 5.0))

        while time.monotonic() < end_time:
            now = time.monotonic()
            if now >= next_command:
                sequence = send_frame(
                    transport, MSG_CONTROL_COMMAND, pack_control(0.0, 0.0), sequence)
                next_command += command_period
            if now >= next_feedback:
                sequence = send_frame(
                    transport, MSG_WHEEL_FEEDBACK, pack_wheel_feedback(0.0, 0.0), sequence)
                next_feedback += feedback_period
            if not arm_sent and now >= arm_at:
                transaction += 1
                sequence = send_frame(
                    transport, MSG_MODE_COMMAND, pack_mode(MODE_ARM, transaction), sequence)
                arm_sent = True
            ready, _, _ = select.select([transport.fd], [], [], 0.002)
            if ready:
                consume_frames(transport.read_available(), measured=True)

        # Drain the last measured pair before the explicit stop command.
        settle_deadline = time.monotonic() + 0.15
        while time.monotonic() < settle_deadline:
            ready, _, _ = select.select([transport.fd], [], [], 0.02)
            if ready:
                consume_frames(transport.read_available(), measured=True)
        result["pre_disarm_status"] = result.get("latest_status")
        if result["pre_disarm_status"] is not None:
            result["pre_disarm_counters"] = {
                field: int(result["pre_disarm_status"].get(field, 0))
                for field in COUNTER_FIELDS
            }
            result["pre_disarm_delta"] = counter_delta(
                result["pre_disarm_counters"],
                result["transport_counters"]["baseline"],
            )

        transaction += 1
        disarm_transaction = transaction
        sequence = send_frame(
            transport, MSG_MODE_COMMAND, pack_mode(MODE_DISARM, disarm_transaction), sequence)
        disarm_deadline = time.monotonic() + 0.8
        while time.monotonic() < disarm_deadline:
            ready, _, _ = select.select([transport.fd], [], [], 0.02)
            if ready:
                consume_frames(transport.read_available(), measured=True)
            if result["acks"]:
                for ack in result["acks"]:
                    if (ack["command"] == MSG_MODE_COMMAND and
                        ack["transaction"] == disarm_transaction and
                        ack["result"] == ACK_OK):
                        disarm_ack_ok = True

        final_status = result.get("latest_status")
        if final_status is None:
            raise TimeoutError("final STATUS was not observed")
        result["final_status"] = final_status
        result["transport_counters"]["final"] = {
            field: int(final_status.get(field, 0)) for field in COUNTER_FIELDS
        }
        result["transport_counters"]["delta"] = counter_delta(
            result["transport_counters"]["final"],
            result["transport_counters"]["baseline"],
        )
        final_timing = timing_reports[-1] if timing_reports else None
        if final_timing is not None:
            result["timing_counters"]["final"] = {
                field: int(final_timing.get(field, 0)) for field in TIMING_COUNTER_FIELDS
            }
            result["timing_counters"]["delta"] = timing_counter_delta(
                final_timing, baseline_timing)
            result["final_timing_status"] = final_timing
        result["measured_duration_s"] = time.monotonic() - measured_start
    except (OSError, EOFError, TimeoutError, ValueError) as error:
        result["reason"] = f"{type(error).__name__}: {error}"
    finally:
        result["host_decoder_errors"]["final"] = reader.decode_errors
        baseline_host_errors = result["host_decoder_errors"].get("baseline")
        if baseline_host_errors is not None:
            result["host_decoder_errors"]["delta"] = reader.decode_errors - int(baseline_host_errors)
        transport.close()

    result["timing_status"] = timing_reports
    counter_deltas = result["transport_counters"].get("delta") or {}
    timing_deltas = result["timing_counters"].get("delta") or {}
    clean_transport = bool(counter_deltas) and all(
        counter_deltas.get(field, 0) == 0 for field in TRANSPORT_ERROR_FIELDS
    )
    deadline_misses_delta = int(timing_deltas.get("deadline_misses", 0))
    host_decoder_errors_delta = result["host_decoder_errors"].get("delta")
    host_decoder_clean = host_decoder_errors_delta == 0
    result["checks"] = {
        "protocol_synchronized": stm32_hello_seen and baseline_status is not None,
        "hello_seen": stm32_hello_seen,
        "stable_status_baseline": status_samples >= 2,
        "timing_status_seen": bool(timing_reports),
        "arm_ack_ok": arm_ack_ok,
        "disarm_ack_ok": disarm_ack_ok,
        "decoder_errors": reader.decode_errors,
        "host_decoder_errors_delta": host_decoder_errors_delta,
        "host_decoder_clean": host_decoder_clean,
        "transport_clean": clean_transport,
        "deadline_misses_delta": deadline_misses_delta,
        "timing_clean": bool(timing_deltas) and deadline_misses_delta == 0,
    }
    if result.get("reason"):
        result["status"] = "fail"
    elif not result["checks"]["protocol_synchronized"]:
        result["status"] = "fail"
        result["reason"] = "protocol synchronization incomplete"
    elif not timing_reports:
        result["status"] = "fail"
        result["reason"] = "no post-baseline TIMING_STATUS received"
    elif not arm_ack_ok or not disarm_ack_ok:
        result["status"] = "fail"
        result["reason"] = "ARM/DISARM handshake incomplete"
    elif not clean_transport:
        result["status"] = "fail"
        result["reason"] = "nonzero per-run transport counter delta"
    elif not host_decoder_clean:
        result["status"] = "fail"
        result["reason"] = "nonzero per-run host decoder error delta"
    elif deadline_misses_delta != 0:
        result["status"] = "fail"
        result["reason"] = "nonzero per-run control deadline-miss delta"
    else:
        result["status"] = "complete"

    if timing_reports:
        result["summary"] = {
            "reports": len(timing_reports),
            "latest_sample_count": int(timing_reports[-1]["sample_count"]),
            "execution_min_us": min(int(item["exec_min_us"]) for item in timing_reports),
            "execution_mean_us": int(timing_reports[-1]["exec_mean_us"]),
            "execution_max_us": max(int(item["exec_max_us"]) for item in timing_reports),
            "period_min_us": min(int(item["period_min_us"]) for item in timing_reports),
            "period_mean_us": int(timing_reports[-1]["period_mean_us"]),
            "period_max_us": max(int(item["period_max_us"]) for item in timing_reports),
            "deadline_misses_delta": deadline_misses_delta,
            "latest_safety_reason": timing_reports[-1]["safety_reason_name"],
            "rx_stack_hwm": int(timing_reports[-1]["rx_stack_hwm"]),
            "control_stack_hwm": int(timing_reports[-1]["control_stack_hwm"]),
            "tx_stack_hwm": int(timing_reports[-1]["tx_stack_hwm"]),
        }
    result["finished_unix_s"] = time.time()
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/serial/by-id")
    parser.add_argument("--baud", type=int, default=115200,
                        choices=(115200, 230400, 460800, 921600))
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--command-rate-hz", type=float, default=100.0)
    parser.add_argument("--feedback-rate-hz", type=float, default=100.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--allow-not-run", action="store_true")
    args = parser.parse_args()
    result = characterize(
        args.device, args.baud, args.duration,
        args.command_rate_hz, args.feedback_rate_hz)
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
