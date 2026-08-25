#!/usr/bin/env python3
"""Minimal dependency-free HIL protocol and POSIX serial helpers for STM32 validation."""

from __future__ import annotations

import errno
import os
import select
import struct
import termios
import time
import tty
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

PROTOCOL_VERSION = 1
MAX_PAYLOAD = 64
MSG_HELLO = 1
MSG_HEARTBEAT = 2
MSG_CONTROL_COMMAND = 3
MSG_WHEEL_FEEDBACK = 4
MSG_WHEEL_EFFORT = 5
MSG_STATUS = 6
MSG_MODE_COMMAND = 7
MSG_ACK = 8
MSG_PING = 9
MSG_PONG = 10
MSG_TIMING_STATUS = 11
ROLE_LINUX_BRIDGE = 1
ROLE_STM32 = 2
MODE_ARM = 1
MODE_DISARM = 2
ACK_OK = 0
ACK_REJECTED = 1
ACK_INVALID = 2
STATE_WAIT_LINK = 0
STATE_DISARMED = 1
STATE_ACTIVE = 2
STATE_SAFE = 3
STATE_FAULT = 4
TIMING_STATUS_SIZE = 62
STATUS_SIZE = 56
SAFETY_NAMES = {
    0: "none",
    1: "command_stale",
    2: "feedback_stale",
    3: "manual_disarm",
    4: "protocol_incompatible",
    5: "internal_error",
    6: "watchdog_reset",
}


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    output = bytearray(b"\x00")
    code_index = 0
    code = 1
    for byte in data:
        if byte == 0:
            output[code_index] = code
            code_index = len(output)
            output.append(0)
            code = 1
        else:
            output.append(byte)
            code += 1
            if code == 0xFF:
                output[code_index] = code
                code_index = len(output)
                output.append(0)
                code = 1
    output[code_index] = code
    return bytes(output)


def cobs_decode(data: bytes) -> bytes:
    if not data:
        raise ValueError("empty COBS frame")
    output = bytearray()
    index = 0
    while index < len(data):
        code = data[index]
        if code == 0:
            raise ValueError("zero code in COBS frame")
        index += 1
        copy_length = code - 1
        if index + copy_length > len(data):
            raise ValueError("COBS code exceeds frame")
        output.extend(data[index:index + copy_length])
        index += copy_length
        if code != 0xFF and index < len(data):
            output.append(0)
    return bytes(output)


@dataclass(frozen=True)
class Frame:
    message_type: int
    payload: bytes
    sequence: int
    sender_tick_ms: int
    flags: int = 0
    protocol_version: int = PROTOCOL_VERSION


def encode_frame(message_type: int, payload: bytes = b"", sequence: int = 0, sender_tick_ms: int | None = None) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload exceeds protocol maximum")
    tick = int(time.monotonic() * 1000) & 0xFFFFFFFF if sender_tick_ms is None else sender_tick_ms & 0xFFFFFFFF
    header = struct.pack("<BBBBII", PROTOCOL_VERSION, message_type, 0, len(payload), sequence & 0xFFFFFFFF, tick)
    body = header + payload
    return cobs_encode(body + struct.pack("<H", crc16(body))) + b"\x00"


def decode_frame(encoded_without_delimiter: bytes) -> Frame:
    decoded = cobs_decode(encoded_without_delimiter)
    if len(decoded) < 14:
        raise ValueError("decoded frame is too short")
    version, message_type, flags, payload_length, sequence, sender_tick_ms = struct.unpack_from("<BBBBII", decoded, 0)
    expected_length = 12 + payload_length + 2
    if version != PROTOCOL_VERSION:
        raise ValueError(f"unsupported protocol version {version}")
    if payload_length > MAX_PAYLOAD or len(decoded) != expected_length:
        raise ValueError("payload length does not match decoded frame")
    expected_crc = struct.unpack_from("<H", decoded, expected_length - 2)[0]
    if crc16(decoded[:expected_length - 2]) != expected_crc:
        raise ValueError("CRC mismatch")
    return Frame(message_type, decoded[12:12 + payload_length], sequence, sender_tick_ms, flags, version)


class FrameReader:
    def __init__(self) -> None:
        self._encoded = bytearray()
        self.decode_errors = 0

    def feed(self, data: bytes) -> Iterable[Frame]:
        frames: list[Frame] = []
        for byte in data:
            if byte == 0:
                if self._encoded:
                    try:
                        frames.append(decode_frame(bytes(self._encoded)))
                    except ValueError:
                        self.decode_errors += 1
                    self._encoded.clear()
            elif len(self._encoded) < 96:
                self._encoded.append(byte)
            else:
                self.decode_errors += 1
                self._encoded.clear()
        return frames


def resolve_device(device: str) -> str:
    path = Path(device)
    if not path.is_dir():
        return device
    entries = sorted(path.iterdir())
    stable = [entry for entry in entries if "STMicroelectronics_STM32_STLink" in entry.name]
    if stable:
        return str(stable[0])
    acm = [entry for entry in entries if "ttyACM" in entry.name]
    return str(acm[0]) if acm else ""


class SerialTransport:
    def __init__(self, device: str, baud: int) -> None:
        self.requested_device = device
        self.device = resolve_device(device)
        if not self.device:
            raise FileNotFoundError(f"no serial device found under {device}")
        speed_name = f"B{baud}"
        speed = getattr(termios, speed_name, None)
        if speed is None:
            raise ValueError(f"this Linux termios does not expose {speed_name}")
        self.fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            tty.setraw(self.fd)
            attrs = termios.tcgetattr(self.fd)
            attrs[4] = speed
            attrs[5] = speed
            attrs[2] |= termios.CLOCAL | termios.CREAD
            attrs[2] &= ~getattr(termios, "CRTSCTS", 0)
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            termios.tcflush(self.fd, termios.TCIOFLUSH)
        except Exception:
            os.close(self.fd)
            raise

    def close(self) -> None:
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1

    def __enter__(self) -> "SerialTransport":
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()

    def write(self, data: bytes, timeout: float = 0.2) -> None:
        end = time.monotonic() + timeout
        offset = 0
        while offset < len(data):
            remaining = max(0.0, end - time.monotonic())
            if remaining == 0.0:
                raise TimeoutError("serial write timed out")
            _, writable, _ = select.select([], [self.fd], [], remaining)
            if not writable:
                raise TimeoutError("serial write timed out")
            try:
                count = os.write(self.fd, data[offset:])
            except BlockingIOError:
                continue
            if count <= 0:
                raise EOFError("serial write returned zero")
            offset += count

    def read_available(self) -> bytes:
        try:
            return os.read(self.fd, 4096)
        except BlockingIOError:
            return b""
        except OSError as error:
            if error.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                return b""
            raise


def pack_hello(capabilities: int = 0) -> bytes:
    return bytes((ROLE_LINUX_BRIDGE, PROTOCOL_VERSION)) + struct.pack("<II", 0, capabilities)


def pack_control(linear_m_s: float, yaw_rad_s: float) -> bytes:
    return struct.pack("<ff", linear_m_s, yaw_rad_s)


def pack_wheel_feedback(left_rad_s: float, right_rad_s: float) -> bytes:
    return struct.pack("<ff", left_rad_s, right_rad_s)


def pack_mode(mode: int, transaction: int) -> bytes:
    return struct.pack("<BI", mode, transaction & 0xFFFFFFFF)


def pack_ping(token: int) -> bytes:
    return struct.pack("<I", token & 0xFFFFFFFF)


def parse_ack(payload: bytes) -> tuple[int, int, int]:
    if len(payload) != 6:
        raise ValueError("invalid ACK payload")
    command, transaction, result = struct.unpack("<BIB", payload)
    return command, transaction, result


def parse_status(payload: bytes) -> dict[str, int]:
    if len(payload) < STATUS_SIZE:
        raise ValueError("not a Milestone 4B status payload")
    return {
        "state": payload[0],
        "safety_reason": payload[1],
        "reset_cause": payload[2],
        "boot_id": struct.unpack_from("<I", payload, 4)[0],
        "uptime_ms": struct.unpack_from("<I", payload, 8)[0],
        "rx_valid": struct.unpack_from("<I", payload, 12)[0],
        "rx_crc": struct.unpack_from("<I", payload, 16)[0],
        "rx_decode": struct.unpack_from("<I", payload, 20)[0],
        "rx_length": struct.unpack_from("<I", payload, 24)[0],
        "rx_version": struct.unpack_from("<I", payload, 28)[0],
        "rx_duplicate": struct.unpack_from("<I", payload, 32)[0],
        "rx_stale": struct.unpack_from("<I", payload, 36)[0],
        "rx_gaps": struct.unpack_from("<I", payload, 40)[0],
        "rx_stream_drops": struct.unpack_from("<I", payload, 44)[0],
        "tx_queue_drops": struct.unpack_from("<I", payload, 48)[0],
        "uart_overruns": struct.unpack_from("<I", payload, 52)[0],
    }


def parse_timing_status(payload: bytes) -> dict[str, int | str]:
    if len(payload) != TIMING_STATUS_SIZE:
        raise ValueError("invalid timing-status payload")
    values = struct.unpack_from("<BBBB" + "I" * 13 + "HHH", payload)
    names = (
        "state", "safety_reason", "reset_cause", "reserved", "boot_id", "uptime_ms",
        "sample_count", "exec_min_us", "exec_mean_us", "exec_max_us",
        "period_min_us", "period_mean_us", "period_max_us", "deadline_misses",
        "rx_stream_drops", "tx_queue_drops", "uart_overruns",
        "rx_stack_hwm", "control_stack_hwm", "tx_stack_hwm",
    )
    result = dict(zip(names, values))
    result["safety_reason_name"] = SAFETY_NAMES.get(int(result["safety_reason"]), "unknown")
    return result
