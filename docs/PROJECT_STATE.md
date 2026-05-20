# vitalV3 — состояние проекта (контекст для разработки)

> **Обновлять этот файл** при завершении каждого milestone.  
> Репозиторий: https://github.com/CtasBr/vitalV3

---

## Что строим

**Робот-манипулятор 4 оси** (шаговые A–D + кинематика плеча 250 мм).

| Слой | Где | Технологии |
|------|-----|------------|
| **Motion MCU** | STM32F429 NUCLEO / кастомная плата | FreeRTOS, TIM1–4 STEP, бинарный UART |
| **Всё остальное** | Mac (iMac M4) | Python `pyrobot/`, ZMQ, msgpack, Pydantic `proto/` |

**Принцип:** STM32 = только моторы + safety. Кинематика, vision, AI, UI — на Mac.

**Старый код** (`vitalSoft`, `vitalDriver` текстовый UART) **удалён**. Смысл сохранён в корневом `README.md`.

---

## Структура репозитория

```
vitalV3/
├── docs/PROJECT_STATE.md      ← этот файл
├── config/robot.yaml          ← единый конфиг (порты, ZMQ, кинематика)
├── proto/                     ← Pydantic: MotionCommand, MotionState, …
├── pyrobot/                   ← Python: hal/, kinematics/, … (этап 0 сделан)
├── tools/                     ← uart_ping.py, uart_step.py, uart_pkt_ping.py
├── firmware/robot.ioc           ← эталон пинов CubeMX
├── vital_motion/              ← CubeIDE проект прошивки (активная разработка)
└── README.md                  ← legacy-функции, кинематика, пины
```

---

## Пины STM32 (не менять без перепайки)

| Ось | STEP | DIR | TIM |
|-----|------|-----|-----|
| A | PE9 | PF0 | TIM1 |
| B | PA0 | PF1 | TIM2 |
| C | PA6 | PF2 | TIM3 |
| D | PD12 | PF3 | TIM4 |

| UART | Пины | Использование |
|------|------|----------------|
| **USART3** | PD8/PD9 | **NUCLEO: ST-Link USB** — сейчас motion + лог (один кабель) |
| **USART2** | PD5/PA3 | Кастомная плата (переключатель `board_config.h`) |

`vital_motion/Core/Inc/board_config.h`: `MOTION_LINK_USE_USART3 1`

---

## Этапы и статус

| Этап | Описание | Статус |
|------|----------|--------|
| **0** | Монорепо, proto, yaml, FakeMotionBus, ZMQ | ✅ |
| **1** | Прошивка motion-core | 🔄 в `vital_motion/` |
| **1-M0** | LED LD1 (PB0) | ✅ |
| **1-M1** | printf / USART3 | ✅ |
| **1-M2** | UART echo, `PING`→`PONG` | ✅ |
| **1-M3** | Ось A: `STEP n [arr]`, TIM1 | ✅ **проверено пользователем** |
| **1-M4** | Бинарный протокол COBS+CRC, PKT PING/PONG | ✅ **проверено пользователем** |
| **1-M5** | PKT_MOVE_SEGMENT (4-axis payload), 4-axis exec, SEGMENT_DONE/FAULT | ✅ |
| **1-M6** | Heartbeat + ESTOP + fault bit in telemetry (soft-limits позже) | 🔄 код; нужна проверка |
| **2** | Python HAL daemons (motion + encoders ZMQ) | 🔄 (`robot-encoder-daemon`, `robot-motion-daemon`, unified A/B) |
| **3–7** | Vision, kinematics, skills, AI | ⏳ |

---

## Прошивка `vital_motion/` — файлы

| Файл | Роль |
|------|------|
| `main.c` | init, FreeRTOS tasks, `HAL_TIM_PeriodElapsedCallback` → TIM7 tick + TIM1 motor |
| `comm_uart.c` | Текст + PKT (`PING/PONG`, `MOVE_SEGMENT`, `SEGMENT_DONE`, `FAULT`, `TELEMETRY`, `HEARTBEAT`, `ESTOP`) |
| `motor.c` | `motor_move_4axes(steps[4], arr[4])` — TIM1..TIM4 PWM |
| `board_config.h` | USART3 vs USART2 |
| `protocol.c` | COBS/CRC, PKT dispatch (`PING/MOVE/TELEMETRY/HEARTBEAT/ESTOP`) |
| `BRINGUP.md` | Пошаговая инструкция |

**CubeIDE:** новые `.c` в `Core/Src` нужно **перетащить в Project Explorer** (не появляются автоматически). `Debug/` в `.gitignore`.

**Задачи FreeRTOS:**

- `defaultTask` — LED 500 ms
- `move` — `comm_uart_poll_loop()` (UART RX/TX)

---

## Текстовый протокол (M2–M3, до полного M4)

Через USART3 @ 115200, строки `\n`:

| Команда | Ответ |
|---------|--------|
| `PING` | `PONG` |
| `STEP 50` | `RUN STEP...` → `OK STEP` |
| `STEP -100 4000` | шаги + TIM1 ARR (скорость) |

Mac: `tools/uart_ping.py`, `tools/uart_step.py`

---

## Бинарный протокол (M4+, целевой)

См. `vital_motion/docs/protocol.md`.

- Raw packet **54 B** + COBS frame
- CRC16-CCITT, `seq`, типы пакетов
- Смысл полей согласован с `proto/motion.py` (Mac — msgpack, MCU — packed struct)

**Не смешивать** на одном UART долгий printf и бинарный поток 100 Hz.

---

## Python (Mac) — этап 0

- `config/robot.yaml`: `motion.backend: fake`, порт `usbmodem` для NUCLEO
- `pyrobot/hal/fake_motion.py` + `robot-fake-motion` daemon
- `pytest` — 8 тестов

**Следующий Python-шаг:** `stm32_motion_bridge` после M4–M5 на MCU.
`pyrobot/hal/stm32_motion.py` добавлен: serial MotionBus к реальной плате.
Safety note: current `move_joints()` in `Stm32MotionBus` treats input as **step deltas**
(temporary transition), with hard clamp `abs(step)<=500` per axis.
Added `create_motion_bus()` and CLI entrypoint `robot-motion-cli`.

---

## Кинематика (из legacy, для `pyrobot/kinematics/`)

- `la = lb = 250` mm
- Cartesian → углы: `l`, `b = acos(1-l²/(2la·lb))`, `a`, `c = atan(y/x)`
- `deg_per_step = 360/(200·16·6)`
- Home: (250, 0, 250), углы 90/90/0

---

## Решения и антипаттерны

| Делаем | Не делаем |
|--------|-----------|
| ZMQ + Pydantic на Mac | ROS2 |
| SHM для камер (позже) | pickle + JPEG по socket |
| Один `robot.yaml` | хардкод `/dev/cu.*` |
| Skills вместо G-code API | G-code как язык поведения |
| USART3 на NUCLEO (1 USB) | обязательно 2 USB на столе |

---

## Команды для проверки

```bash
# Python
cd vitalV3 && source .venv/bin/activate
pytest -q
uv run robot-fake-motion

# MCU (после прошивки)
screen /dev/cu.usbmodem* 115200
python3 tools/uart_ping.py
python3 tools/uart_step.py 20
python3 tools/uart_pkt_ping.py   # M4
python3 tools/uart_pkt_step.py 20 5000   # M5 payload (B/C/D=0)
python3 tools/uart_pkt_heartbeat.py   # M6 heartbeat echo
python3 tools/uart_pkt_estop.py       # M6 estop -> fault
```

Примечание: после включения `PKT_TELEMETRY` скрипт `uart_pkt_ping.py` должен игнорировать тип `0x20` и ждать `PKT_PONG`.

---

## Git

- Ветка: `main`
- Коммиты: этап 0 (`e32b896`), vital_motion M2–M3 (`bafa0e3`), M4 — следующий
- **Пушить** после каждого завершённого milestone

---

## Следующая работа (M5 → M6)

1. Host-side telemetry viewer / logger
2. Привязать soft-limits к `config/robot.yaml` (сейчас в firmware константы +/-4800 шагов)
3. После M6: `pyrobot/hal/stm32_bridge.py` + `motion.backend: stm32`

---

## M6 safety status (current)

- Heartbeat watchdog активируется только после первого heartbeat (`g_hb_seen`) и не даёт ложный fault на старте.
- Добавлен `PKT_RESET_FAULT` (`0x32`) host→mcu; MCU отвечает `PKT_FAULT(0)` при успешном сбросе.
- Добавлены firmware soft-limits по каждой оси: значения берутся из `config/robot.yaml` (`motion.soft_limits_steps`) через `tools/generate_motion_limits_header.py` -> `vital_motion/Core/Inc/motion_limits.h`.
- `robot-motion-cli` поддерживает `reset-fault`.

---

## Encoders A/B (UART, legacy AT protocol)

- `pyrobot/hal/encoder_bus.py`: чтение `port_a` / `port_b`, `AT+PRATE=0`, парсинг `Angle:...`, legacy transform (как vitalSoft).
- `Stm32MotionBus.q_enc_deg`:
  - **A/B** — реальные энкодеры (если порты доступны),
  - **C/D** — пока step counters STM32.
- Offsets: `config/encoders_offsets.json`, home `[90, 90, 0, 0]`.
- CLI:
  - `python -m pyrobot.hal.motion_cli enc-state` — только энкодеры A/B
  - `python -m pyrobot.hal.motion_cli zero-encoders` — калибровка нуля (AT+ZERO + offset, home 90/90)
- ZMQ:
  - `robot-encoder-daemon` — единственный владелец UART энкодеров → `encoders.state`
  - `robot-motion-daemon` — STM32 motion + подписка на `encoders.state` для A/B в `motion.state`
  - `python tools/encoder_sub.py`, `python tools/motion_sub.py` — отладка
- Offsets: `config/encoders_offsets.json` (в .gitignore), шаблон `config/encoders_offsets.json.example`

### Запуск HAL (2 терминала + STM32 прошит)

```bash
# T1 — энкодеры (обязательно для motion_daemon A/B)
python -m pyrobot.hal.encoder_daemon

# T2 — motion (STM32 USB + ZMQ encoders)
python -m pyrobot.hal.motion_daemon

# T3 — смотреть состояние
python tools/motion_sub.py --count 5
```

`motion_cli` по-прежнему работает напрямую (без ZMQ), открывая UART энкодеров сам.
