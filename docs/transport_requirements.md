# Pi/STM32 Transport Requirements

## Status and scope

This document derives a provisional transport envelope from the Milestone 2 software-boundary measurements. It specifies requirements for a Raspberry Pi 5/Linux bridge to STM32 link. The concrete Version 1 packet format, Linux serial bridge, and STM32 implementation are staged under Milestone 4A; see [`docs/protocol.md`](protocol.md) for the normative wire contract.

The tables and analytical budget below are retained as historical measured, derived, provisional, or TBD planning evidence. They were written before a concrete encoding was selected and must not be read as a claim that the physical UART has passed validation. Actual encoded sizes, latency, loss, watchdog, reset, and fault-injection measurements remain required on the NUCLEO-F446RE.

## Implemented Version 1 boundary

The current implementation uses COBS framing with a zero delimiter, CRC-16/CCITT-FALSE, explicit little-endian integer fields, IEEE-754 binary32 float fields, a 64-byte application payload limit, and a 96-byte encoded-frame limit. The portable C library is compiled into both `hil_serial_bridge` and the STM32 firmware. The default bring-up baud is 115200, selectable up to 460800 in the bridge; this is an initial configuration rather than a final transport requirement.

The STM32 enforces local monotonic freshness: command age is limited to 100 ms and wheel-feedback age to 50 ms while ACTIVE. A stale stream forces zero effort and SAFE. ARM is an explicit mode transaction and requires a compatible link, fresh command and feedback, and no blocking fault. These are software behaviors awaiting target and fault-injection evidence.

## Measurement basis

The local WSL host completed 10 clean normal runs and 10 clean runs with four pinned CPU workers. The controller runs at a 10 ms nominal period. Across the loaded runs, the across-run P95 of each run's execution-time P95 was 0.544 ms, the largest observed execution sample was 4.097 ms, and the worst observed execution margin was 5.903 ms. Target-to-effort response was at most 12 ms in the normal set and 11 ms in the selected loaded set. Target-to-wheel-motion response was at most 26 ms normal and 30 ms loaded.

These are measured host observations, not wire latency or target-hardware timing. The source evidence is in `results/milestone_02/normal_summary.json` and `results/milestone_02/cpu_loaded_summary.json`.

## Provisional service envelope

| Requirement | Value | Classification | Rationale |
| --- | ---: | --- | --- |
| Wheel setpoint publication | 100 Hz | Provisional | One update per nominal 10 ms controller period; avoids the current 20 Hz profile's command-age pattern. |
| Wheel feedback publication | 100 Hz | Provisional | One fresh feedback sample per controller period. |
| Heartbeat, each direction | 10 Hz | Provisional | Supervisory liveness independent of command traffic. |
| Status publication | 10 Hz | Provisional | State, mode, fault flags, and counters. |
| General telemetry | 20 Hz nominal | Provisional | Non-control observability without competing with 100 Hz control traffic. |
| Fault/event burst | Up to 10 frames/s | Provisional | Bounded short burst; exact queue policy is TBD. |
| One-way command delivery | p95 <= 5 ms, max <= 10 ms | Derived target | Leaves most of a 10 ms controller period available after serialization and scheduling. Must be validated on hardware. |
| One-way feedback delivery | p95 <= 5 ms, max <= 10 ms | Derived target | Supports one feedback sample per control period. Must be validated on hardware. |
| Valid command age at controller | <= 30 ms | Provisional | Allows limited scheduling variation while preventing old setpoints from appearing current. |
| Command-loss safe transition | <= 100 ms | Provisional safety limit | Tighter than the current 500 ms simulation timeout; final value needs hazard analysis. |
| Valid feedback age at controller | <= 20 ms | Provisional | Two nominal periods. |
| Feedback-loss safe transition | <= 50 ms | Provisional safety limit | Tighter than the current 200 ms simulation timeout; final value needs plant testing. |
| Maximum application payload | 64 bytes | Provisional | Covers the fields below with extension room. |
| Maximum encoded frame | 96 bytes | Provisional | Includes framing, integrity, and encoding expansion; exact encoding is TBD. |

The command and feedback safety limits dominate the slower 10 Hz heartbeat. A heartbeat receiver may declare the supervisory link degraded after three missed heartbeats (300 ms), but it must not delay a command- or feedback-loss safe transition.

## Historical provisional frame semantics

The following requirement set predates the concrete Version 1 format. The selected fields and encoding are now specified in `docs/protocol.md`.

Every control-relevant frame should carry:

- protocol version and message type;
- payload length;
- independent per-stream sequence number;
- sender monotonic timestamp or tick count;
- payload;
- integrity check strong enough to reject corruption (CRC choice is TBD).

Sequence counters detect loss, duplication, and reordering. The receiver must reject stale or duplicate control setpoints. Status must expose at least receive sequence, drop/error counters, controller state, safe-state reason, and reset/boot identity.

Acknowledgements are required for configuration, mode changes, fault clearing, and other state-changing transactions. Streaming setpoint, feedback, heartbeat, and telemetry frames should not require an acknowledgement per frame; that would couple control progress to reverse-path traffic. Their health is inferred from sequence progress, freshness, and periodic status. Retry counts and idempotency rules remain TBD.

## Clock-domain rules

ROS/Gazebo simulation time must not be used as a transport safety clock. Pi and STM32 monotonic clocks are initially independent.

- Each endpoint uses its local monotonic receive time for watchdogs.
- A sender timestamp is directly converted to one-way age only after a clock-offset/error bound has been established.
- Without synchronization, freshness is enforced from receive time and sequence progress; round-trip request/ack timing may characterize the link.
- Clock synchronization method, drift limit, timestamp width, tick resolution, and wrap handling are TBD and must be versioned with the protocol.

## Analytical UART bandwidth budget

The planning calculation assumes an expected 12-byte encoded overhead per frame (header, sequence, timestamp, length, CRC, and delimiter combined) and UART 8N1, which consumes 10 serial bits per encoded byte. The overhead is an assumption, not a selected wire format.

| Traffic class | Aggregate direction | Rate | Payload | Expected frame | Encoded bytes/s |
| --- | --- | ---: | ---: | ---: | ---: |
| Wheel setpoint | Pi -> STM32 | 100 Hz | 12 B | 24 B | 2,400 |
| Wheel feedback | STM32 -> Pi | 100 Hz | 20 B | 32 B | 3,200 |
| Heartbeat | Both directions | 10 Hz each | 4 B | 16 B | 320 |
| Status | STM32 -> Pi | 10 Hz | 16 B | 28 B | 280 |
| Telemetry | STM32 -> Pi | 20 Hz | 24 B | 36 B | 720 |
| Fault/event burst | STM32 -> Pi | 10 Hz burst | 24 B | 36 B | 360 |
| Transaction ack/response | Both directions | 10 Hz aggregate | 8 B | 20 B | 200 |
| **Total planning traffic** |  |  |  |  | **7,480 B/s** |

The expected traffic requires 74,800 bit/s under 8N1. A 10% scheduling/encoding reserve raises the planning value to 82,280 bit/s. This reserve is not a substitute for measuring actual framing expansion and burst behavior.

| Baud rate | Utilization at 74,800 bit/s | Raw headroom | Utilization with 10% reserve | Interpretation |
| ---: | ---: | ---: | ---: | --- |
| 115,200 | 64.93% | 40,400 bit/s (35.07%) | 71.42% | Analytically possible for expected frames, but limited burst and encoding margin. |
| 460,800 | 16.23% | 386,000 bit/s (83.77%) | 17.86% | Strong candidate for initial hardware tests. |
| 921,600 | 8.12% | 846,800 bit/s (91.88%) | 8.93% | Ample analytical bandwidth; signal integrity and endpoint support remain TBD. |

A deliberately conservative case with both 100 Hz streams using the provisional 96-byte maximum would consume 192,000 bit/s before heartbeat or telemetry. Therefore 115,200 baud cannot support unconstrained maximum-size control frames. The implementation must either enforce compact control frames or select a higher baud rate. No final baud is selected by this document.

## Required hardware validation

Before freezing the protocol or claiming transport compliance:

1. Measure encoded frame sizes and escaping expansion from the actual format.
2. Measure one-way latency only with a bounded clock relationship; otherwise measure RTT and receiver-local freshness.
3. Repeat latency, loss, CRC-error, queue-depth, and watchdog tests under Pi and STM32 CPU load.
4. Inject disconnects, truncation, corruption, duplication, reordering, stale sequences, endpoint reset, and clock wrap.
5. Verify safe outputs independently on the STM32 when commands or feedback become stale.
6. Select baud rate from measured error margin, cable/interface characteristics, and worst-case burst traffic.
