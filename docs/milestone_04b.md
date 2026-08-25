# Milestone 4B — STM32 timing, transport, watchdog, and bridge hardening

## Decision

Milestone 4B is complete on branch `milestone-04b-stm32-hardening`.

The required operational baud is 115200. Three valid 60-second runs at the required 100 Hz command and 100 Hz feedback rates passed with zero MCU transport-error deltas, zero queue/UART/DMA-drop deltas, zero host decoder-error deltas, and zero actual control deadline misses.

## Implementation completed

- Per-run counter accounting now records synchronized baseline, final, and uint32 wrap-safe delta values. Historical counters are not mistaken for errors in the current run.
- A fresh host HELLO resets the STM32 session sequence state. The bridge resets its transmit sequence on every serial open, tracks HELLO readiness separately from timing/status reception, and retries HELLO until the STM32 responds.
- The STM32 responds to a valid host HELLO and clears stale command/feedback freshness for the new session.
- STM32 RX uses a 256-byte circular DMA ring polled every 1 ms. Hardware UART overrun and queue/drop telemetry remain visible.
- Control timing uses FreeRTOS release/deadline behavior for the deadline metric. Raw period statistics remain available; the observed 13 ms maximum period is not counted as a miss when the task release and execution deadline were met.
- The ROS bridge tolerates the Linux USB CDC-ACM `POLLHUP` behavior seen while the ST-Link VCP remains present, uses status/write failures for link loss, zeroes effort on link failure, and reconnects through the stable by-id path.
- Hardware-validation tooling treats 115200 as required and higher bauds as experimental/informational. Experimental failures or not-run cases cannot turn a required pass into a pass or fail; the decision is explicit.

## Required 115200 characterization

Command used for each valid run:

```bash
python3 tools/stm32/run_timing_characterization.py \
  --device /dev/serial/by-id \
  --baud 115200 \
  --duration 60 \
  --command-rate 100 \
  --feedback-rate 100
```

| Run | Measured duration | MCU valid-frame delta | MCU transport errors | Timing drops/overruns | Deadline delta | Host decoder delta | ARM/DISARM |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | 60.956 s | 12,002 | 0 | 0 | 0 | 0 | pass/pass |
| 2 | 60.956 s | 12,002 | 0 | 0 | 0 | 0 | pass/pass |
| 3 | 60.952 s | 12,002 | 0 | 0 | 0 | 0 | pass/pass |

The detailed machine-readable record is [`results/milestone_04b/operational_115200_summary.json`](../results/milestone_04b/operational_115200_summary.json). The final DMA firmware was flashed and verified by OpenOCD. Image hashes are recorded in that file.

Observed timing across the runs: execution 26–82 us; raw activation period 9,973–13,000 us; mean period 10,020 us; actual release/deadline miss delta 0 in every run.

## Physical disconnect/reconnect

The connected NUCLEO-F446RE was operated through the ROS 2 bridge at 115200 baud with the deterministic motion publisher, explicit ARM, and nonzero effort. The user unplugged and reconnected the USB cable. On disconnect, the bridge reported a write failure, removed the serial link, entered device-unavailable reconnect polling, and executed its zero-effort fail-safe. After the user reconnected the cable and usbipd bus `2-8` was reattached to WSL, `/dev/ttyACM0` returned under the stable by-id path. The bridge reopened, retried HELLO until synchronization, accepted ARM, returned to ACTIVE, and produced nonzero effort.

The physical evidence is [`results/milestone_04b/physical_disconnect_reconnect.json`](../results/milestone_04b/physical_disconnect_reconnect.json). A power-cycle on USB also advanced the boot ID and reported the expected power-on reset cause.

## Bandwidth basis

For this protocol, wire bytes are `payload + 16`: 12-byte header, payload, 2-byte CRC, one COBS overhead byte, and the zero delimiter. At the required steady-state rates:

- Linux → STM32: control 100 Hz × 24 B, wheel feedback 100 Hz × 24 B, and heartbeat 10 Hz × 21 B = 5,010 B/s = 50,100 8N1 bit/s, or 43.5% of 115200.
- STM32 → Linux: wheel effort 100 Hz × 24 B, heartbeat 10 Hz × 21 B, status 10 Hz × 72 B, and timing 1 Hz × 78 B = 3,408 B/s = 34,080 8N1 bit/s, or 29.6% of 115200.
- The UART is full duplex; combined traffic is 8,418 B/s, or 36.5% of the two-direction 230400 bit/s aggregate capacity. Transaction ACKs and HELLO frames are bounded setup/session overhead.

The exact frame-size calculation and assumptions are documented in [`docs/transport_requirements.md`](transport_requirements.md).

## Regression evidence

- Python tool tests: 7/7 passed.
- Portable CMake suite: protocol, control, and safety tests: 3/3 passed.
- ROS 2 workspace rebuild: 5 packages completed.
- Firmware build: `text=17,860`, `data=4`, `bss=8,508` bytes.
- OpenOCD flash and verify: passed on STLINK V2J46M33 / STM32F446RE.
- Watchdog behavior from the prior 4B evidence remains preserved; the DMA, session, and bridge changes do not alter watchdog supervision.

Raspberry Pi deployment remains outside this milestone.

MILESTONE 4B COMPLETE
