# STM32F446RE FreeRTOS controller

This target is for the ST NUCLEO-F446RE (`NUF446RE$KU1`), using the STM32F446RE MCU and its ST-LINK/V2-1 virtual COM port.

## Hardware routing

The NUCLEO-F446RE ST-LINK VCP is wired to the MCU's USART2:

- PA2: USART2_TX
- PA3: USART2_RX
- AF7
- 8-N-1
- default bring-up baud: 115200

The firmware uses the 16 MHz HSI clock directly. This avoids an unvalidated PLL/clock-tree dependency during first bring-up; the F446RE can be moved to a higher clock after the serial path is proven.

The user LED on PA5 is off while disarmed, on while active, and blinks while safe.

## Build

Install the host prerequisites in WSL:

```bash
sudo apt-get update
sudo apt-get install gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi openocd
```

Then build from this directory:

```bash
make -C firmware/stm32
```

The build is command-line reproducible and uses the pinned FreeRTOS-Kernel V11.3.1 source in `third_party/FreeRTOS-Kernel`. No dynamic allocation is used by the application; all application tasks, queues/buffers, and protocol storage are static.

Milestone 4B build variants:

```bash
make -C firmware/stm32 clean all BAUD=115200
make -C firmware/stm32 clean all BAUD=460800
make -C firmware/stm32 clean all BAUD=115200 TEST_WATCHDOG=1
```

The normal image enables the IWDG and refreshes it only while the supervisor observes control-task progress. `TEST_WATCHDOG=1` intentionally withholds refresh after the test hold interval so the next STATUS/TIMING_STATUS reports an IWDG reset cause. Use the host tools in `tools/stm32` to collect fixed-schema timing and health evidence.

## Flashing

Do not flash until the exact target has been checked. With the board attached to WSL and OpenOCD available:

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program build/hil_stm32_f446re.elf verify reset exit"
```

The expected target is STM32F446RETx. A successful build is not evidence that flashing succeeded; record probe and flash output separately.

## Runtime architecture

- `uart_rx_task`: receives bytes from the USART2 RX interrupt through a bounded FreeRTOS stream buffer and feeds the shared parser.
- `control_task`: runs at 100 Hz with `vTaskDelayUntil`, applies command/feedback freshness supervision, differential-drive kinematics, and limited P control.
- `tx_task`: emits HELLO, heartbeat/status, ACK/PONG responses, and the latest wheel effort over the UART.

The safety state machine is `WAIT_LINK -> DISARMED -> ACTIVE`, with `SAFE` entered on command freshness >100 ms, feedback freshness >50 ms, or transport/parser loss. ARM is explicit and requires fresh valid command and feedback. DISARM immediately zeros both efforts.

The protocol implementation is shared with Linux under `common/hil_protocol`; the control math is shared under `common/hil_control`.

## Current limitation

Physical firmware build, flash, scheduler, UART, and closed-loop HIL evidence remain local hardware gates until the ARM toolchain is installed and the board is probed. The laptop bridge's zero-effort behavior is a simulation enforcement mirror and is not an STM32 hardware safety mechanism.
