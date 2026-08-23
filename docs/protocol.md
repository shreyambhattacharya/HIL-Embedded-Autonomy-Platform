# Laptop/Pi ↔ STM32 serial protocol

## Status

Version 1 is implemented in the portable C library at `common/hil_protocol`. The same serializer, COBS decoder, CRC, field helpers, and sequence policy are compiled into the Linux serial bridge and the STM32 firmware. This is the first concrete protocol choice; the earlier transport document remains the historical source of provisional bandwidth assumptions.

## Framing

Each frame is:

```text
COBS( header || payload || crc16 ) || 0x00
```

The delimiter is a single zero byte. COBS removes zero bytes from the encoded body, so a receiver can resynchronize at the next delimiter after truncation or corruption. The decoder has fixed storage and rejects malformed or oversized input without heap allocation.

The decoded header is exactly 12 bytes, followed by 0–64 payload bytes and a 2-byte CRC:

| Offset | Field | Width | Encoding |
| ---: | --- | ---: | --- |
| 0 | protocol version | 1 | unsigned integer, value `1` |
| 1 | message type | 1 | enum |
| 2 | flags | 1 | reserved, currently zero |
| 3 | payload length | 1 | unsigned bytes, maximum 64 |
| 4 | sequence | 4 | little-endian `uint32` |
| 8 | sender tick | 4 | little-endian `uint32`, endpoint-local monotonic milliseconds |
| 12 | payload | 0–64 | message-specific |
| 12 + payload length | CRC-16 | 2 | little-endian CRC value |

The maximum decoded body is 78 bytes. The maximum encoded frame, including the delimiter, is 96 bytes. The library checks both bounds.

## Integrity and scalar encoding

CRC is CRC-16/CCITT-FALSE: initial value `0xffff`, polynomial `0x1021`, no reflection, no final XOR. It covers the decoded header and payload, excluding the CRC field. The known ASCII vector `123456789` produces `0x29b1`.

Integers are written explicitly in little-endian order. Float fields are explicitly copied as IEEE-754 binary32 bit patterns and then written little-endian. Both host tests and the target header enforce a 32-bit binary float assumption. No received byte buffer is cast to a C struct.

## Message types

| Value | Name | Direction | Payload |
| ---: | --- | --- | --- |
| 1 | HELLO | both | role, protocol version, boot identity, capabilities |
| 2 | HEARTBEAT | both | uptime tick and state |
| 3 | CONTROL_COMMAND | Linux → STM32 | `float32 linear_m_s`, `float32 yaw_rad_s` |
| 4 | WHEEL_FEEDBACK | Linux → STM32 | `float32 left_rad_s`, `float32 right_rad_s` |
| 5 | WHEEL_EFFORT | STM32 → Linux | `float32 left_Nm`, `float32 right_Nm` |
| 6 | STATUS | STM32 → Linux | state, fault, boot identity, uptime, error counters |
| 7 | MODE_COMMAND | Linux → STM32 | mode byte and `uint32` transaction ID |
| 8 | ACK | STM32 → Linux | command type, transaction ID, result byte |
| 9 | PING | Linux → STM32 | `uint32` caller token |
| 10 | PONG | STM32 → Linux | same token |

Roles are `1 = Linux bridge` and `2 = STM32`. Modes are `1 = ARM` and `2 = DISARM`. ACK results are `0 = OK`, `1 = REJECTED`, and `2 = INVALID`. Controller states are `0 = WAIT_LINK`, `1 = DISARMED`, `2 = ACTIVE`, `3 = SAFE`, and `4 = FAULT`.

The current STATUS payload is 26 bytes:

```text
u8 state
u8 fault_reason
u32 boot_id
u32 uptime_ms
u32 accepted_frames
u32 crc_failures
u32 decode_failures
u32 sequence_gaps
```

## Sequence semantics

Each endpoint has one independent transmit sequence counter. A receiver accepts the first sequence, accepts a forward modulo-`uint32` sequence, counts a forward delta greater than one as a gap, rejects equal sequence as duplicate, and rejects a modulo delta at or above `0x80000000` as stale. Streaming loss is counted but is not itself a latched fault; freshness supervision determines safe output.

`HELLO` is the session boundary. A valid HELLO is accepted even when its sequence restarts or is behind the previous session, and it resets the receiver's sequence baseline. This allows MCU reset/reboot to be detected and recovered without accepting stale streaming control data.

The decoder separately counts accepted frames, CRC failures, decode failures, length failures, version failures, duplicates, stale frames, and sequence gaps. A rejected frame never updates command or feedback state.

## State-changing transactions and safety

ARM and DISARM use MODE_COMMAND plus ACK. The STM32 never arms from an arbitrary CONTROL_COMMAND. ARM is accepted only after a compatible HELLO, valid fresh command and wheel feedback, and no blocking fault. DISARM immediately sets both efforts to zero.

The STM32 uses its local FreeRTOS tick for freshness:

- command freshness limit: 100 ms;
- wheel feedback freshness limit: 50 ms;
- stale command or feedback while ACTIVE: enter SAFE and output zero effort;
- reset: new boot identity and non-active state; explicit ARM is required again.

The Linux bridge also publishes zero simulated effort when the serial device is missing, disconnected, or silent beyond its configured status timeout. That is a simulation enforcement mirror, not independent STM32 safety.

## Test coverage

The standalone host suite covers CRC and COBS known vectors, empty and maximum payloads, round trips, message helpers, malformed frames, CRC rejection, sequence duplicate/stale/gap behavior, embedded zero bytes, concatenated frames, and deterministic random byte input. It does not replace physical UART corruption, timing, or reset tests.
