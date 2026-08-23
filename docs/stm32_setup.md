# NUCLEO-F446RE STM32 setup

## Target identity

This Milestone 4A target is the user-confirmed `NUCLEO-F446RE NUF446RE$KU1`, with STM32F446RETx, Cortex-M4F, and ST-LINK/V2-1. Do not reuse this firmware Makefile for another Nucleo without changing and re-verifying the target, memory map, clock, UART pins, and flashing configuration.

The ST-LINK VCP was observed by Windows as COM6 and by WSL as `/dev/ttyACM0` after USB passthrough. The stable Linux path is:

```bash
/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066FFF575171525067054337-if02
```

Use the by-id path in a launch override when multiple serial devices may exist.

## USB and permissions

If the board is visible in Windows but not WSL, install `usbipd-win` and run PowerShell as Administrator:

```powershell
usbipd list
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```

Then in WSL:

```bash
sudo usermod -aG dialout "$USER"
ls -l /dev/ttyACM0
udevadm info --query=all --name=/dev/ttyACM0
```

Restart the WSL distribution after changing group membership. Do not claim an ST-LINK debug probe is available merely because its VCP is visible; flashing requires an accessible debug interface and a working probe tool.

## Verified physical bring-up

On 2026-08-22 the confirmed NUCLEO-F446RE was attached to WSL through usbipd-win and was visible through the stable by-id path above. OpenOCD 0.12.0 identified STLINK V2J46M33, detected the STM32F446RE Cortex-M4 and approximately 3.25 V target power, and reported a verified 512 KiB flash image after programming. The bridge then observed live STATUS traffic with `rx_crc=0`, received PONG, and completed ARM (`state=2`) and DISARM (`state=1`) transitions with successful ACK results.

## Host prerequisites

```bash
sudo apt-get update
sudo apt-get install -y \
  gcc-arm-none-eabi \
  binutils-arm-none-eabi \
  libnewlib-arm-none-eabi \
  openocd
```

The repository vendors only the pinned FreeRTOS-Kernel V11.3.1 source needed by this target. The application has static task, queue, stream-buffer, and protocol storage and does not call `malloc` or `free` during normal operation.

## Build and inspect

From the repository root:

```bash
make -C firmware/stm32 clean all
arm-none-eabi-size firmware/stm32/build/hil_stm32_f446re.elf
```

Expected outputs are ELF, BIN, HEX, and map files. Record the compiler version, commit SHA, target, and size output with physical evidence.

## Flash

Only after confirming the target and build:

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program firmware/stm32/build/hil_stm32_f446re.elf verify reset exit"
```

If OpenOCD cannot access the ST-LINK debug interface, stop and report the exact error. Do not switch to a guessed target or claim flashing success. A Windows STM32CubeProgrammer GUI or administrator-only operation is a manual gate; report the artifact path and exact target so the user can flash it.

## UART routing

The NUCLEO-F446RE ST-LINK VCP is connected to USART2 on the MCU:

```text
PA2 / USART2_TX / AF7
PA3 / USART2_RX / AF7
8 data bits, no parity, 1 stop bit
default bring-up baud: 115200
```

The Linux bridge parameters are configurable:

```text
serial_device
baud_rate: 115200, 230400, or 460800
command_tx_rate_hz: 100
feedback_tx_rate_hz: 100
heartbeat_rate_hz: 10
status_timeout_ms: 250
```

115200 is the initial bring-up setting, not the final selected baud. The earlier 460800 value is an analytical candidate from Milestone 2; select it only after physical error/RTT evidence.

## Incremental bring-up

Use these stages and retain actual output:

1. Confirm `/dev/ttyACM0` or the by-id path.
2. Build and flash the verified F446RE image.
3. Confirm repeated HELLO frames on the Linux bridge.
4. Send PING and observe PONG/RTT.
5. Observe heartbeat and STATUS counters.
6. Test ARM/DISARM; ARM must reject stale or absent inputs.
7. Send synthetic command/feedback and verify numerical efforts.
8. Start Gazebo with `milestone_04a_stm32.launch.py` only after the serial stages pass.

Do not run the deterministic motion profile until the graph, serial link, status, and explicit ARM state are visible.
