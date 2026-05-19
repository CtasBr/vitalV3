# UART binary protocol (MCU ↔ Mac)

Transport: **COBS** framing + **CRC16-CCITT** (poly 0x1021, init 0xFFFF).

## Raw packet (60 bytes, before COBS)

| Offset | Size | Field |
|--------|------|-------|
| 0 | 1 | `magic` = `0x56` ('V') |
| 1 | 1 | `version` = `1` |
| 2 | 1 | `type` — see below |
| 3 | 1 | `flags` |
| 4 | 2 | `seq` uint16 LE |
| 6 | 2 | `payload_len` uint16 LE (0..44) |
| 8 | 44 | `payload` |
| 52 | 2 | `crc16` LE over bytes [0..51] |

## Packet types (M4+)

| Value | Name | Direction |
|-------|------|-----------|
| `0x01` | `PKT_PING` | host→mcu |
| `0x02` | `PKT_PONG` | mcu→host |
| `0x10` | `PKT_MOVE_SEGMENT` | host→mcu |
| `0x20` | `PKT_TELEMETRY` | mcu→host @ 100 Hz |
| `0x21` | `PKT_SEGMENT_DONE` | mcu→host |
| `0x30` | `PKT_ESTOP` | host→mcu |
| `0x31` | `PKT_FAULT` | mcu→host |
| `0x3F` | `PKT_HEARTBEAT` | both @ 10 Hz |

## COBS

- Encoder replaces `0x00` in `[raw 60 bytes]` → COBS codewords, terminates with `0x00`.
- Max encoded length: 64 bytes + 1 delimiter.

## M4 test

Host sends COBS(`PKT_PING`, seq=N). MCU replies COBS(`PKT_PONG`, same seq).

Text commands (`PING`, `STEP`) remain for bring-up until M5.
