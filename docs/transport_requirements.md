# Pi/STM32 Transport Requirements

## Scope and status

This document is the concrete Version 1 UART transport envelope used by Milestone 4B. The link uses COBS framing with a zero delimiter, CRC-16/CCITT-FALSE, explicit little-endian integer fields, IEEE-754 binary32 values, and a 64-byte application payload limit. The required operational baud is 115200; higher baud cases are experimental until separately characterized.

The bridge and STM32 enforce local monotonic freshness. Commands older than 100 ms or wheel feedback older than 50 ms force a safe output. ARM is explicit and requires a compatible HELLO session, fresh command and feedback, and no blocking fault.

## Runtime service rates

| Traffic | Direction | Rate | Payload |
| --- | --- | ---: | ---: |
| Control command | Linux → STM32 | 100 Hz | 8 B |
| Wheel feedback | Linux → STM32 | 100 Hz | 8 B |
| Heartbeat | Both directions | 10 Hz each | 5 B |
| Wheel effort | STM32 → Linux | 100 Hz | 8 B |
| STATUS | STM32 → Linux | 10 Hz | 56 B |
| TIMING_STATUS | STM32 → Linux | 1 Hz | 62 B |

The host and STM32 use absolute-period schedules for the two 100 Hz streams. Streaming frames are not individually acknowledged. HELLO starts a session, and ARM/DISARM mode transactions receive ACKs.

## Exact wire-byte calculation

The encoded frame contains a 12-byte header, the application payload, a 2-byte CRC, one COBS overhead byte, and one zero delimiter. Therefore:

```text
encoded_wire_bytes = payload_bytes + 16
8N1_serial_bits = encoded_wire_bytes * 10
```

| Frame | Payload | Encoded bytes | Rate | Bytes/s | 8N1 bit/s |
| --- | ---: | ---: | ---: | ---: | ---: |
| Control command, Linux → STM32 | 8 | 24 | 100 Hz | 2,400 | 24,000 |
| Wheel feedback, Linux → STM32 | 8 | 24 | 100 Hz | 2,400 | 24,000 |
| Heartbeat, Linux → STM32 | 5 | 21 | 10 Hz | 210 | 2,100 |
| **Linux → STM32 steady state** |  |  |  | **5,010** | **50,100** |
| Wheel effort, STM32 → Linux | 8 | 24 | 100 Hz | 2,400 | 24,000 |
| Heartbeat, STM32 → Linux | 5 | 21 | 10 Hz | 210 | 2,100 |
| STATUS, STM32 → Linux | 56 | 72 | 10 Hz | 720 | 7,200 |
| TIMING_STATUS, STM32 → Linux | 62 | 78 | 1 Hz | 78 | 780 |
| **STM32 → Linux steady state** |  |  |  | **3,408** | **34,080** |

At 115200 baud, host-direction utilization is 43.5% and STM32-direction utilization is 29.6%. Since the UART is full duplex, combined steady-state traffic is 8,418 B/s, or 36.5% of the aggregate two-direction capacity. A 10% planning reserve gives 47.8% and 32.5% per direction. HELLO and two mode ACKs are bounded session overhead, not continuous traffic.

The three required 60-second runs delivered 12,002 MCU-valid frames each and recorded zero CRC, decode, length, version, duplicate, stale, gap, queue, UART, DMA/stream, and deadline deltas. See [`results/milestone_04b/operational_115200_summary.json`](../results/milestone_04b/operational_115200_summary.json).

## Baud policy

115200 is the required pass/fail case for Milestone 4B. 230400, 460800, and 921600 are optional experimental configurations. The validation runner reports experimental outcomes separately and never allows an experimental failure to hide a required failure or convert a partial required set into PASS.

## Safety and session rules

- Fresh HELLO resets command/feedback freshness and starts a new sequence-number session.
- The bridge resets its transmit sequence on every serial open and retries HELLO until a valid STM32 HELLO is observed.
- Status and timing counters are compared as per-run uint32 deltas, not as absolute lifetime totals.
- Receive-side DMA and UART overrun/drop telemetry remains visible to the operator.
- A physical serial failure clears ARM state, publishes zero effort, and reconnects through `/dev/serial/by-id`.
- ROS/Gazebo time is not used as the transport safety clock.
