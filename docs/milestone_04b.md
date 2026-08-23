# Milestone 4B — STM32 timing, watchdog, and bridge hardening

## Scope

This milestone hardens the real NUCLEO-F446RE (`NUF446RE$KU1`) controller without advancing to a later milestone. The STM32 remains the authority for command/feedback freshness, explicit ARM/DISARM, zero-effort behavior, and watchdog recovery. The Linux bridge mirrors link-down behavior and reconnects through the stable `/dev/serial/by-id` directory.

The implementation is on branch `milestone-04b-stm32-hardening`. It is intentionally left uncommitted and unpushed for review.

## Implemented behavior

- DWT cycle-counter instrumentation measures control execution time and activation period with wrap-safe arithmetic.
- `TIMING_STATUS` (message 11, 62-byte payload) reports safety state/reason, reset cause, boot ID, uptime, sample count, execution and period min/mean/max, deadline misses, transport drops/overruns, and task stack high-water marks.
- `STATUS` is now a fixed 56-byte payload with explicit decoder and transport counters.
- The safety module enters SAFE on stale command or feedback, manual disarm, protocol incompatibility, or internal error. A new HELLO clears prior streaming freshness, so a reboot cannot reuse stale inputs.
- IWDG supervision refreshes only when the control task is making progress. The normal image uses a nominal approximately 500 ms timeout derived from the uncalibrated 32 kHz LSI.
- `TEST_WATCHDOG=1` creates an intentional test image that withholds refresh after the hold interval.
- The bridge accepts 115200/230400/460800/921600, resolves the ST-LINK VCP by-id directory, emits link-down zero effort on open/configure/read/timeout failure, and retries opening on a timer.
- The FreeRTOS vendor tree retains only the V11.3.1 core, required headers, the GCC ARM_CM4F port, and license/version provenance.

## Validation commands

```bash
cmake -S common -B /tmp/hil-common-4b -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/hil-common-4b
ctest --test-dir /tmp/hil-common-4b --output-on-failure

make -C firmware/stm32 clean all BAUD=115200 TEST_WATCHDOG=0
python3 tools/stm32/run_timing_characterization.py \
  --device /dev/serial/by-id --baud 115200 --duration 10
python3 tools/stm32/run_hardware_validation.py \
  --device /dev/serial/by-id --bauds 115200,460800 --duration 10

make -C firmware/stm32 clean all BAUD=115200 TEST_WATCHDOG=1
# Flash the test image, observe reset_cause=3 and boot_id advancement, then
# rebuild/flash the normal image before operating the board.
```

The Python tools emit schemas `hil.stm32.milestone4b.timing.v1` and `hil.stm32.milestone4b.validation.v1`. A baud case is not considered complete unless timing telemetry and both ARM and DISARM ACKs are observed.

## Physical evidence on the connected NUCLEO

OpenOCD 0.12.0 identified STLINK V2J46M33, STM32F446RE, approximately 3.25 V target power, 512 KiB flash, and verified the programmed image. The normal 115200 image was rebuilt and restored after the watchdog test.

| Case | Result |
| --- | --- |
| 115200, 10 s, 20 ms command/feedback stream period | ARM/DISARM ACKs and timing telemetry passed; `rx_crc=0`, `uart_overruns=0`, `rx_stream_drops=0`; latest timing observed execution max 186 us and period mean 10020–10021 us. The MCU counters recorded `rx_decode=2`, `rx_length=1`, `rx_gaps=3`, so this is functional evidence with a small framing margin, not a zero-error transport claim. |
| 115200, 10 s, 30 ms stream period | ARM/DISARM ACKs and timing telemetry passed; no CRC failures or UART overruns; observed execution max 161 us and period mean 10024–10027 us. This lower-load run still recorded one decode/gap event at the MCU boundary. |
| 460800, 10 s | Timing telemetry arrived, but ARM/DISARM handshakes did not complete. The MCU reported `rx_crc=1`, `rx_decode=804`, `rx_length=3`, and `uart_overruns=127`; this baud is rejected for the current ISR/transport implementation. |
| Intentional watchdog image | Passed. At 115200, boot IDs advanced 17 → 18 → 19; successive reports carried `reset_cause=3` (IWDG) after approximately 3.4 s of the intentional holdoff. The normal image was restored afterward. |

The selected operational baud is 115200. The nominal transport characterization is not a clean zero-error pass, so any requirement for zero framing/gap counters at the full bridge stream rate remains open for a later tuning pass.

## Remaining gate

The bridge reconnect code is implemented and the ROS bridge compiles, but a physical unplug/replug disconnect/reconnect run was not performed in this session. Raspberry Pi deployment is outside this STM32 hardening milestone. These omissions are reflected in the final decision below.

MILESTONE 4B INCOMPLETE
