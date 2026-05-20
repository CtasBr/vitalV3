# UART binary protocol (MCU ↔ Mac)

Transport: **COBS** framing + **CRC16-CCITT** (poly 0x1021, init 0xFFFF).

## Raw packet (54 bytes, before COBS)

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
| `0x32` | `PKT_RESET_FAULT` | host→mcu |
| `0x3F` | `PKT_HEARTBEAT` | both |

## COBS

- Encoder replaces `0x00` in `[raw 54 bytes]` → COBS codewords, terminates with `0x00`.
- Max encoded length: 60 bytes + 1 delimiter.

## M4 test

Host sends COBS(`PKT_PING`, seq=N). MCU replies COBS(`PKT_PONG`, same seq).

Text commands (`PING`, `STEP`) remain for bring-up until M5.

## M5 payload (implemented)

`PKT_MOVE_SEGMENT` payload (8 bytes, LE):

| Offset | Type | Meaning |
|--------|------|---------|
| 0 | int32 | `steps_a` |
| 4 | uint32 | `arr_a` (TIM1 ARR, speed) |

MCU executes `motor_axis_a_move(steps_a, arr_a)` and replies:

- `PKT_SEGMENT_DONE` with same `seq`
- payload 16 bytes: `done_steps_a/b/c/d` (int32 ×4)

Current firmware stage:

- executes all 4 axes (TIM1..TIM4) in parallel
- supports `PKT_TELEMETRY` every ~100 ms, payload:
  - `[0..15] int32 pos_steps_a/b/c/d`
  - `[16] uint8 in_motion_mask` (bit0..bit3 => A..D)
  - `[17] uint8 fault`
  - `[18..19] reserved`
  - `[20..23] int32 done_reserved`
- supports `PKT_HEARTBEAT` echo (host→mcu, mcu→host)
- supports `PKT_ESTOP` → immediate `motor_estop_all()` and `PKT_FAULT(-2001)`
- supports `PKT_RESET_FAULT` → clear latched fault and ack with `PKT_FAULT(0)`
- enforces soft-limits in MCU step-space before move:
  - per axis target must stay in [-4800, +4800] steps
  - on violation MCU returns `PKT_FAULT(-1004)` and latches `fault_code=4`

## Telemetry `fault` byte (latched)

| Code | Meaning |
|------|---------|
| `0` | OK |
| `1` | ESTOP |
| `2` | move timeout / motor busy |
| `3` | **heartbeat watchdog** — no `PKT_HEARTBEAT` from host for >1 s after first HB |
| `4` | soft limit violation |

Host `motion_daemon` must send heartbeat at `motion.heartbeat_hz` (default 10 Hz). Clear with `PKT_RESET_FAULT` / `motion_cli reset-fault`.
