# vital_motion — что делать в CubeIDE сейчас

Проект сгенерирован из `vital_motion.ioc`. Пины **совпадают** с legacy `vitalDriver` / `firmware/robot.ioc`.

## Таблица пинов (не менять)

| Сигнал | Pin | Периферия |
|--------|-----|-----------|
| STEP A | PE9 | TIM1 CH1 |
| STEP B | PA0 | TIM2 CH1 |
| STEP C | PA6 | TIM3 CH1 |
| STEP D | PD12 | TIM4 CH1 |
| DIR A | PF0 | GPIO (aDir) |
| DIR B | PF1 | bDir |
| DIR C | PF2 | cDir |
| DIR D | PF3 | dDir |
| Motion RX | PA3 | USART2 |
| Motion TX | PD5 | USART2 |
| Log RX | PD9 | USART3 (ST-Link) |
| Log TX | PD8 | USART3 |
| LED | PB0 | LD1 |

TIM prescaler/period: TIM1 167/10000, TIM2–4 83/10000 — как было.

---

## Ты уже открыл .ioc — что нажать

### Сейчас (M0+M1) — **можно не менять CubeMX**

Код уже в `Core/Src/main.c` (USER CODE):

- мигает **LD1**
- `printf` на **USART3**

1. **Project → Build All**
2. **Run → Debug (F11)** → **Resume (F8)**
3. Терминал Mac:
   ```bash
   ls /dev/cu.usbmodem*
   screen /dev/cu.usbmodemXXXX 115200
   ```
   Должно быть: `=== vital_motion M1 ===` и `alive tick=...` каждые 2 с.

### Одна правка в CubeMX (рекомендуется)

**Middleware → FREERTOS → Tasks and Queues**

| Task | Stack [words] | Priority |
|------|---------------|----------|
| defaultTask | **256** (было 128) | Normal |
| move | 256 | Below Normal (пока пустой) |

`printf` ест больше стека. **Save → Generate Code** — USER CODE в `main.c` сохранится.

### Позже (M2+, не сейчас)

- **Connectivity → USART2**: DMA RX circular, NVIC USART2, baud 921600
- Новые задачи: `safety_task`, `comm_rx`, `comm_tx` (переименовать/заменить `move`)
- **System Core → IWDG** — на этапе safety

**Не трогай Pinout** — только параметры периферии.

---

## Где писать код дальше

| Milestone | Файл | Что |
|-----------|------|-----|
| **M0–M1** (сейчас) | `main.c` USER CODE | LED + printf ✅ |
| **M2** | `Core/Src/comm_uart.c` (создать) | echo на USART2 |
| **M3** | `Core/Src/motor.c` | один мотор, N шагов |
| **M4+** | `Core/Src/protocol.c` | COBS + CRC |

После **Generate Code** правки только в:

- `/* USER CODE BEGIN … */` блоках
- своих `.c/.h` (добавить в проект: ПКМ → Add Existing Files)

---

## Что **не** писать в прошивке

- G-code, `sscanf`, кинематика
- логика YOLO / камеры

Это остаётся на Mac (vitalV3 / pyrobot).

---

## Если Generate Code затёр код

Восстанови блоки USER CODE из git или из коммита; `__io_putchar` и цикл в `StartDefaultTask` — в `main.c`.

---

## Один USB (ST-Link) для всего — NUCLEO

На Nucleo **motion = USART3** = тот же виртуальный COM, что и программатор.

В `Core/Inc/board_config.h`:

```c
#define MOTION_LINK_USE_USART3 1   /* NUCLEO, один кабель */
// #define MOTION_LINK_USE_USART3 0  /* кастомная плата: USART2 PD5/PA3 */
```

**Важно:** на одном UART не мешай долгий `printf` и бинарный протокол (M4+). Сейчас M2 — только текст echo/PING.

---

## M2 — прошить и проверить

1. **Добавить `comm_uart.c` в проект CubeIDE** (файл в Finder есть, в дереве IDE — нет):

   **Способ A (проще):** открой **Finder** → `vital_motion/Core/Src/` → перетащи `comm_uart.c` на папку **Core/Src** в **Project Explorer** (слева в CubeIDE). Подтверди *Copy* или *Link*.

   **Способ B:** меню **File → Import…** → **General → File System** → Next →  
   *From directory:* выбери `…/vital_motion/Core/Src` → отметь `comm_uart.c` →  
   *Into folder:* выбери проект `vital_motion/Core/Src` → Finish.

   **Способ C:** ПКМ по имени проекта **vital_motion** (корень) → **Refresh** (F5). Если не появился — A или B.

   В дереве должно быть: `vital_motion → Core → Src → comm_uart.c` (рядом с `main.c`).

2. **Project → Build All** — в Console не должно быть `undefined reference to comm_uart_init`.
2. **Build → Flash**
3. Закрой `screen` (два клиента на один порт не работают)
4. Mac:
   ```bash
   cd /path/to/vitalV3
   .venv/bin/python tools/uart_ping.py
   ```
   Ожидаешь: `PING -> 'PONG\r\n'`, `echo ABC -> b'ABC'`, `OK`

Ручной тест: `screen /dev/cu.usbmodemXXXX 115200`, набери `PING` + Enter → `PONG`.

---

## M3 — одна ось A

**Проводка:** STEP = PE9 (TIM1), DIR = PF0 (aDir). Питание драйвера отдельно, сначала мало шагов.

1. Добавь в проект CubeIDE: `motor.c` (как `comm_uart.c` — drag & drop в `Core/Src`)
2. Build → Flash
3. `screen` 115200 → команда:
   ```text
   STEP 50
   ```
   Медленно (~5000 ARR по умолчанию). Ответ: `RUN STEP...` → `OK STEP`.

   Быстрее: `STEP 100 3000` — второе число = TIM1 ARR (меньше = быстрее).

   Назад: `STEP -50`

4. Или с Mac:
   ```bash
   python3 tools/uart_step.py 50
   python3 tools/uart_step.py -30 6000
   ```

Если крутится не туда — поменяй знак или инверсию DIR в `motor.c`.

## M4 — бинарный PKT_PING / PKT_PONG

1. Добавь `protocol.c` в проект (drag → `Core/Src`)
2. Build → Flash
3. **Закрой screen** — бинарный тест не совместим с echo в терминале
4. ```bash
   python3 tools/uart_pkt_ping.py
   ```
   Ожидаешь: `OK: PKT_PONG seq=42`

Текст `PING` / `STEP` по-прежнему работают. Текст `BPING` — отправить бинарный PONG вручную.

## После M4

1. M5 payload (в коде) — `PKT_MOVE_SEGMENT` с 4 осями:  
   ```bash
   python3 tools/uart_pkt_step.py 20 5000
   ```
   Ожидаешь: `OK: PKT_SEGMENT_DONE ...`
   Пример всех осей:
   ```bash
   python3 tools/uart_pkt_step.py 50 5000 /dev/cu.usbmodemXXXX --steps-b 30 --steps-c -20 --steps-d 10
   ```
2. M5 full — реализовать физическое движение B/C/D  
2. M6 — heartbeat / ESTOP  
   ```bash
   python3 tools/uart_pkt_heartbeat.py /dev/cu.usbmodemXXXX
   python3 tools/uart_pkt_estop.py /dev/cu.usbmodemXXXX
   ```
   heartbeat: `OK: heartbeat echo ...`
   estop: `OK: fault ... code=-2001`
3. Кастомная плата: `MOTION_LINK_USE_USART3 0` → USART2
