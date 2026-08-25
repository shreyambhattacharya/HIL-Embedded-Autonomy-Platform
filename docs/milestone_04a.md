# Milestone 4A — STM32 FreeRTOS controller and physical UART

## Status

This milestone is in progress on branch `milestone-04a-stm32-uart`, based on Milestone 3 commit `3a08e0bf9284e0ff23b062b4d9ff9ab9d205d6f5`. The target is the user-confirmed NUCLEO-F446RE (`NUF446RE$KU1`), STM32F446RETx, Cortex-M4F.

The portable protocol, shared control core, Linux POSIX bridge, STM32F446RE firmware, and host tests are implemented. The exact target has been built, flashed, verified, and exercised through the real ST-LINK VCP. The mandatory 100 Hz link, RTT, HIL motion, safety-fault, and MCU-reset gates now have physical evidence.

The Raspberry Pi remains unavailable. This milestone intentionally uses the laptop as the temporary Linux-side bridge and does not change Milestone 3 Pi status.

## Repository state

```text
branch: milestone-04a-stm32-uart
starting commit: 3a08e0b Implement Milestone 3 Raspberry Pi integration
working tree: implementation in progress; not committed or pushed
```

## Runtime partition

```text
Laptop / WSL
  motion_test_node
  Gazebo + ros_gz_bridge
  hil_serial_bridge (/dev/ttyACM0, configurable baud)
       | COBS + CRC-16/CCITT-FALSE over ST-LINK VCP
       v
NUCLEO-F446RE / STM32F446RE
  USART2 PA2/PA3
  uart_rx_task -> control_task (100 Hz) -> tx_task
  local freshness safety and wheel-speed P control
       |
       +--> WHEEL_EFFORT -> laptop bridge -> Gazebo joint force
```

The new launch file `milestone_04a_stm32.launch.py` starts Gazebo, the Gazebo bridge, `motion_test_node`, and `hil_serial_bridge`. It deliberately does not start `software_mcu_stub`. The existing Milestone 1 launch and stub remain unchanged for reference and regression use.

## Firmware architecture

The target uses the 16 MHz HSI clock directly for first bring-up, avoiding an unvalidated PLL configuration. USART2 is configured for the NUCLEO-F446RE ST-LINK VCP at 115200 baud, 8-N-1. PA2 and PA3 use alternate function 7. The PA5 user LED is a state indicator only.

The firmware uses static task stacks and a static RX stream buffer/TX queue. Normal operation has no `malloc` or `free` calls. The RX interrupt only pushes bytes into the bounded stream buffer; it never runs the parser or controller. The control task uses `vTaskDelayUntil` at 100 Hz and never blocks on UART. The TX task owns serialized UART output.

The local state machine is:

```text
WAIT_LINK -> DISARMED -> ACTIVE
                         |
                         +-- stale command/feedback -> SAFE
```

ARM is explicit and ACKed. It requires a compatible HELLO plus fresh valid command and wheel feedback. DISARM immediately zeros efforts. Reset creates a boot identity and requires a new ARM. Command freshness is 100 ms and feedback freshness is 50 ms, both measured with the STM32 FreeRTOS tick.

## Protocol and bridge

The wire contract is documented in `docs/protocol.md` and implemented once in `common/hil_protocol`. It uses version 1, explicit little-endian fields, IEEE-754 binary32 helpers, COBS with `0x00` delimiter, CRC-16/CCITT-FALSE, 64-byte maximum application payload, and 96-byte maximum encoded frame.

The bridge uses POSIX `termios`, a nonblocking descriptor, `poll`, fixed protocol buffers, monotonic local timestamps, and reconnect handling. It transmits commands and latest wheel feedback at 100 Hz, heartbeat at 10 Hz, and publishes zero Gazebo effort on missing/disconnected/silent serial. That zero is a simulation enforcement mirror, not the STM32 safety mechanism.

Bridge services:

```text
/hil/stm32/arm
/hil/stm32/disarm
/hil/stm32/ping
```

The bridge publishes `/hil/stm32/status` and the existing actuator effort topics. It refuses to request ARM until wheel feedback has arrived.

## Validation record

| Check | Result | Evidence classification |
| --- | --- | --- |
| Portable protocol/control CMake build | PASS | measured locally; 2 CTest tests |
| Protocol malformed/random-input tests | PASS | measured locally in host test |
| ROS serial bridge build | PASS | measured locally in ROS 2 Jazzy workspace |
| Full ROS 2 workspace build/tests | PASS | 5 packages built; 12 tests passed |
| Host sanitizer build/tests | PASS | AddressSanitizer/UndefinedBehaviorSanitizer; 2 CTest tests |
| Python launch syntax | PASS | milestone_04a_stm32.launch.py parsed successfully |
| ARM GCC firmware build | PASS | GCC 13.2.1; 24,952 bytes; ELF SHA-256 c1dc73f6057e55c3bb95e4941aae6cc44453c86462fbdedd7e4e0e992e097938 |
| Firmware artifacts | RECORDED | BIN SHA-256 655971d50f867748b2bdf4e0308b3ac4a9daa2e352b6e9f742a7c1297c1db7a9; HEX SHA-256 21474cb4754ddab239ad706c9e919fd92a45ffb3ae5a12bb407c0a1b5a79e255 |
| Exact target probe | PASS | OpenOCD 0.12.0; STLINK V2J46M33; STM32F446RE Cortex-M4; target voltage approximately 3.25 V |
| Firmware flash/verify/reset | PASS | device ID `0x10006421`; detected flash size 512 KiB; OpenOCD reported `Verified OK` and reset the target |
| FreeRTOS scheduler/tasks | PASS | live physical HELLO, HEARTBEAT, STATUS, and WHEEL_EFFORT frames observed after reset |
| HELLO/PING/PONG/STATUS | PASS | bridge opened the ST-LINK VCP; `pong=received`; STATUS reported `link=UP` and `rx_crc=0` |
| ARM precondition rejection | PASS | bridge refused ARM before wheel feedback arrived |
| ARM/DISARM ACK and state transition | PASS | physical bridge capture showed ACK result 0, `state=2`, then ACK result 0 and `state=1` |
| 100 Hz sustained link | PASS | 60 s at 115200 baud; 6,000 CONTROL_COMMAND and 6,000 WHEEL_FEEDBACK frames sent; MCU accepted 12,001; CRC 0; decode 0; sequence gaps 0 |
| 100 Hz control task | PASS | corrected SysTick; 60 s physical run reported 62,204 ms MCU uptime and approximately 100 Hz WHEEL_EFFORT output |
| UART RTT | MEASURED | 1,000 PING/PONG samples; 0 timeouts; median 6.765 ms; P95 12.720 ms; maximum 14.764 ms |
| Gazebo STM32 HIL motion | PASS | first forward 0.530 m; turn 0.195 rad; second forward 0.424 m; final body speed and effort returned to zero |
| command/feedback/CRC/sequence fault tests | PASS | command and feedback loss entered SAFE with zero effort; bad CRC incremented CRC counter while ACTIVE remained; duplicate/stale frames were ignored |
| MCU reset and bridge re-sync | PASS | connected capture changed boot ID 3→4, emitted HELLO/WAIT_LINK, and returned to non-active state; subsequent ARM transmission required |
| Physical disconnect | NOT RUN | no physical cable-disconnect test claimed |
| STM32 control timing | NOT RUN | requires target-local instrumentation |
| Software controller regression | PASS | existing software stub remains intact; common control tests and ROS regression remain passing |
| Milestone 1 regression | PASS | existing software-only smoke path previously passed; physical path also passed the same profile assertions |
| Milestone 2 regression | PASS | existing characterization suite evidence retained; no Milestone 2 measurement code was changed |
| Software vs STM32 quantitative A/B comparison | NOT RUN | physical STM32 profile passed independently; no paired fresh-process trajectory run retained |
| Raspberry Pi physical validation | DEFERRED | unchanged Milestone 3B hardware blocker |

The ARM toolchain and OpenOCD are installed. The final NUCLEO-F446RE image was attached to WSL, probed, flashed, verified, and exercised successfully. Re-run `make -C firmware/stm32 clean all` after source changes and repeat the explicit target/verify step before relying on a new image.

## Remaining non-gating measurements

- STM32-local control period/execution measurements.
- physical disconnect behavior.
- paired fresh-process software-stub versus STM32 trajectory comparison.

The mandatory physical evidence is retained in the validation record above. The remaining items are useful follow-up measurements, not unobserved PASS claims.

## Decision

MILESTONE 4A COMPLETE

All mandatory acceptance evidence is now recorded for the exact NUCLEO-F446RE target. Physical Raspberry Pi validation remains deferred; the optional timing, disconnect, and paired A/B measurements are documented as not run.
