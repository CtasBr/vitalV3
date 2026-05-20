# Robot Manipulator — новая кодовая база

Проект переписывается **с нуля**. Папки `vitalSoft/` и `vitalDriver/` удалены; этот README — единственный «мост» к старой системе: функции, кинематика, пины CubeMX, протоколы.

---

## Разделение ответственности

| Где | Что |
|-----|-----|
| **STM32** | Только шаговые моторы (4 оси), PWM/step-gen, safety, бинарный UART/Ethernet |
| **Mac (iMac M4)** | Кинематика, планирование траекторий, vision, behaviour, AI, UI, энкодеры (опрос по USB) |

Между STM32 и Mac — **одна бинарная шина** (COBS + CRC, сегментный буфер).  
Между процессами на Mac — **ZeroMQ + msgpack + Pydantic** (без ROS 2).

---

## Подходы: делаем / не делаем

### Делаем

- **Монорепо**: `firmware/`, `proto/`, `pyrobot/`, `config/`, `tools/`, `tests/`
- **Один конфиг** `config/robot.yaml` — порты, кинематика, лимиты; `load_config()` везде
- **Контракты сообщений** в `proto/` с `schema_version`; на Mac — Pydantic + msgpack
- **Процессы-демоны** (motion_bridge, encoder_bridge, camera_node, tof_node, yolo_node…) + IPC ZMQ
- **Shared memory** для кадров RGB/depth (метаданные в топиках, не пиксели)
- **Recorder (mcap)** с этапа HAL — данные дороже кода
- **Skills** вместо G-code как языка поведения; G-code только тонкий адаптер для слайсера (DIW)
- **FakeMotionBus** — симулятор до появления железа
- **Юнит-тесты** кинематики и host-side тесты протокола STM32

### Не делаем (переделываем)

| Было | Станет |
|------|--------|
| Текстовый UART + `sscanf` на MCU | Packed binary + COBS + CRC16 + state machine |
| `aData/bData/cData/dData`, копипаста по осям | `motor_t motors[NUM_AXES]` |
| Блокирующий `ready` → 60 байт → ждать конца | Сегментный ring buffer, look-ahead, 100 Hz телеметрия |
| Один процесс `main_oop.py` на всё | Узлы-процессы + ZMQ |
| Pickle + JPEG по Unix-socket | SHM + msgpack metadata |
| Глобальные `ser`, `ser1` | Класс/процесс-владелец порта |
| `get_controller()` в web — второй экземпляр робота | Web → ZMQ → skills/motion |
| G-code + M-коды как API навыков | `Skills.home()`, `find_object()`, … |
| Кинематика внутри G-code processor | Чистый `kinematics/` + `motion_planner` |
| Хардкод `/dev/cu.usbserial-*` | `robot.yaml` |

---

## Структура репозитория (целевая)

```
robot/   (корень newVit)
├── firmware/                 # STM32 CubeIDE / CMake
├── proto/                    # Pydantic-модели (единый контракт)
├── pyrobot/
│   ├── hal/                  # motion_bus, encoder_bus, vision_bus
│   ├── kinematics/
│   ├── motion/
│   ├── perception/
│   ├── behaviour/            # skills, state machines
│   ├── ai/
│   ├── ui/
│   ├── recorder/
│   └── config/               # load_config()
├── config/
│   └── robot.yaml
├── tools/
├── tests/
└── README.md
```

Python: **uv**, **ruff**, **mypy strict**, **pre-commit**.

**Состояние разработки и контекст для агента:** [`docs/PROJECT_STATE.md`](docs/PROJECT_STATE.md)

### Быстрый старт (этап 0)

```bash
uv sync --extra dev
uv run pytest
uv run python -m pyrobot.hal.fake_motion          # локальный симулятор
uv run robot-fake-motion                          # ZMQ-демон motion
uv run robot-motion-cli state                     # CLI к motion backend из robot.yaml
```

---


## CubeMX / STM32F429ZI (NUCLEO-F429ZI) — не менять проводку

Восстановить проект в CubeIDE с **теми же пинами**. Файл-источник сохранён: [`docs/hardware-reference/vitalDriver.ioc`](docs/hardware-reference/vitalDriver.ioc) (открыть в CubeMX → Generate Code).

### МК и тактирование

- **MCU**: STM32F429ZIT6, LQFP144  
- **Board**: NUCLEO-F429ZI  
- **SYSCLK**: 168 MHz (HSE 8 MHz, PLL)  
- **HAL timebase**: TIM7 (не SysTick для HAL tick)

### Моторы (STEP/PWM + DIR)

| Ось | Таймер | Канал | Pin STEP | Pin DIR | Label в .ioc |
|-----|--------|-------|----------|---------|--------------|
| **A** | TIM1 | CH1 | **PE9** | **PF0** | aDir |
| **B** | TIM2 | CH1 | **PA0** | **PF1** | bDir |
| **C** | TIM3 | CH1 | **PA6** | **PF2** | cDir |
| **D** | TIM4 | CH1 | **PD12** | **PF3** | dDir |

**Параметры таймеров (как было):**

| TIM | Prescaler | Period | Примечание |
|-----|-----------|--------|------------|
| TIM1 | 167 | 10000 | ось A |
| TIM2 | 83 | 10000 | ось B |
| TIM3 | 83 | 10000 | ось C |
| TIM4 | 83 | 10000 | ось D |

Прерывания: TIM1 (BRK, UP, TRG/COM, CC), TIM2, TIM3, TIM4 — priority **5**.

### UART (связь с ПК)

| UART | Назначение | TX | RX | Baud (было) |
|------|------------|----|----|-------------|
| **USART2** | Протокол движения (в прошивке `huart2`) | **PD5** | **PA3** | **115200** |
| **USART3** | ST-Link VCP (отладочный) | **PD8** | **PD9** | **115200** |

> На Mac драйвер мотора подключался как `/dev/cu.usbserial-2120` (часто это **VCP ST-Link = USART3**). В старой прошивке обмен шёл через **`huart2` (PD5/PA3)** — при сборке новой платы убедиться, какой физический USB-UART идёт на USART2, и не путать с VCP. В `robot.yaml` зафиксировать фактический порт.

### Ethernet (фаза 2, пины уже разведены)

RMII + LAN8742A: PA1 REF_CLK, PA2 MDIO, PA7 CRS_DV, PC1 MDC, PC4 RXD0, PC5 RXD1, PB13 TXD1, PG11 TX_EN, PG13 TXD0, PHY address 0.

### USB

- OTG FS: PA11 DM, PA12 DP, PA9 VBUS, PA10 ID, PA8 SOF  
- PG6 USB_PowerSwitchOn, PG7 USB_OverCurrent  

### Прочее

- **PC13**: USER button (EXTI)  
- **PB0** LD1 green, **PB7** LD2 blue, **PB14** LD3 red — GPIO output  
- **FreeRTOS**: CMSIS-OS v2; задача `move` (priority 8, stack 256) — в новой архитектуре заменить на `safety_task`, `traj_task`, `comm_rx/tx`

### Сохранить при регенерации CubeMX

`ProjectManager.KeepUserCode=true` — пользовательский код в `/* USER CODE */` не затирать без необходимости.

---

## Старый протокол STM32 ↔ PC (только для миграции / LegacyMotionBus)

**Не воспроизводить в новой прошивке.** Описано, чтобы понимать legacy.

1. MCU шлёт `ready\n`
2. Host шлёт **ровно 60 байт** ASCII (дополнение пробелами):  
   `command a_steps b_steps c_steps d_steps periodA periodB periodC periodD [доп. поля для энкодеров]`
3. Во время движения MCU шлёт `+1000\n` каждые 1000 шагов
4. Парсинг на MCU: `sscanf(receiveBuffer, "%d %d ...")`

**command:**

| command | Смысл |
|---------|--------|
| 0 | G0 — быстрый move |
| 1 | G1 — move (экструдер / toolhead) |
| 3 | G28 / homing |

**period** в пакете — период PWM в мкс (минимальный = быстрее). На MCU: `aData[3]=period`, разгон за `steps_to_speed` (5) шагов, `initial_period=2000`.

**Оси в пакете:** `a_steps`, `b_steps`, `c_steps`, `d_steps` — шаги; знак задаёт направление (на MCU модуль берётся, DIR инвертируется для оси A).

---

## Кинематика (из `RobotArmCalculator`) — сохранить математику

### Геометрия манипулятора

- Два плеча одинаковой длины: **`la = lb = 250` мм**
- Состояние в декартовых координатах: **`x, y, z`** (мм), база по умолчанию после G28: **(250, 0, 250)**
- Углы суставов в градусах: **`a`, `b`** — плечи; **`c`** — поворот основания (азимут); **`c_added`** — накопленный offset команды `C`
- Шаговый привод: **1.8°**, микрошаг **16**, редуктор **6:1**  
  → `deg_per_step = 360 / (200 * 16 * 6)` ≈ **0.01875°/шаг** (для A, B, C, D)

### Прямая задача (Cartesian → углы)

Для целевой точки `(x, y, z)` и добавки `c_add`:

```
l = sqrt(x² + y² + z²)

b = acos(1 - l² / (2·la·lb)) · 180/π          # угол «локтя»

a = (180 - b)/2 + asin(z/l)·180/π   (если l≠0, иначе a_prev)

c = atan(y/x)·180/π + c_added       (если x≠0, иначе c_prev)
```

### «Обратная» в том виде, как было в коде

Отдельной функции IK «pose → несколько решений» **не было**. Использовалась **дельта по шагам** от предыдущих углов:

```
da = a - a_prev
db = b - b_prev + da          # связка B с A
dc = c - c_prev

a_steps = int(da / deg_per_step_a)
b_steps = int(db / deg_per_step_b)
c_steps = int(dc / deg_per_step_c)
```

После расчёта обновлять `a_prev`, `b_prev`, `c_prev`.

### Скорость (синхронизация осей)

Для перемещения `(dx, dy, dz)` и подачи `F` (мм/мин):

```
t = sqrt(dx² + dy² + dz²) / (F/60)   # время сегмента, с

a_period_us = t / |a_steps| · 1e6   # период шага оси A, мкс
# аналогично B, C; если шагов 0 → 1000 мкс
```

Ось **D**: при экструдере/слайсере `d_steps` связан с `c_steps` через отношение `deg_per_step_d / deg_per_step_a`.

### Давление DIW (слайсер)

```
V = π · e
t = sqrt(dx² + dy²) / f
Q = V / t
pressure = coef · Q / 1000
```

Коэффициенты в state: `coef_0 = 7.1e5`, `coef_1 = 2.6e7`.

### Что заложить в новый `kinematics/`

- [ ] `forward(q) -> pose` — восстановить из геометрии выше  
- [ ] `inverse(pose, q_current) -> list[Solution]` — **несколько** решений, выбор ближайшего к `q_current`  
- [ ] `is_reachable(pose) -> bool`  
- [ ] Юнит-тесты на G28, диагональ, границы workspace  
- [ ] Jacobian — для IBVS (этап 7)

### Коррекция по энкодерам (было на host)

Перед отправкой на MCU измерялись `angle_a`, `angle_b` с энкодеров и корректировались `a_steps`, `b_steps`:

```
da_enc = round((angle_a_enc - angle_a_cmd + I_err_a) / (deg_per_step_a · 4))
db_enc = round((angle_b_enc - angle_b_cmd + I_err_b) / (deg_per_step_b · 4))
# для G28 делитель без ·4
```

В новой системе: либо closed-loop на PC в planner, либо отдельный `encoder fusion` в `robot.state`.

---

## Карта функций legacy → новые модули

### `main_oop.py`

| Класс / функция | Назначение | Новый модуль |
|-----------------|------------|--------------|
| `VisionDataReceiver` | Unix-socket + pickle, JPEG decode, YOLO list | `camera_node`, `tof_node`, `yolo_node` |
| `EncoderDataReceiver` | `/tmp/encoder_socket` | `encoder_bridge` |
| `RobotArmState` | x,y,z,e, углы, F, coef, toolhead | `robot.state` + yaml |
| `RobotArmCalculator.calculate_pressure` | DIW давление | `motion/extruder` или slicer adapter |
| `RobotArmCalculator.calculate_command` | toolhead 0/1 | config toolheads |
| `RobotArmCalculator.calculate_steps` | Cartesian→шаги | `kinematics/inverse_delta` или planner |
| `RobotArmCalculator.speed_params` | периоды PWM | `motion_planner` |
| `GcodeProcessor.process_gcode_string` | G/M/T parser | `behaviour/gcode_adapter` (тонкий) |
| `GcodeProcessor._process_movement_command` | G0/G1 | `Skills.move_to` / planner |
| `GcodeProcessor._process_homing_command` | G28 → (250,0,250) | `Skills.home` |
| `GcodeProcessor._process_vision_command` | M111 ToF centering | `Skills.center_over_tof` |
| `GcodeProcessor.search_object_by_name` | M112 grid + YOLO | `Skills.find_object` |
| `GcodeProcessor.center_over_object_visual` | RGB/YOLO steps 10 mm | `Skills.center_over` → later IBVS |
| `GcodeProcessor.nod_over_object` | жест-кивок | `Skills.nod` |
| `GcodeProcessor._process_capture_command` | M200 save stereo | `recorder` / `Skills.capture` |
| `GcodeProcessor._move_and_wait` | блокирующая очередь | async segment completion |
| `SerialCommunicator.communicate` | UART handshake | `motion_bridge` |
| `SerialCommunicator.send_zero_command` | AT+ZERO encoders | `encoder_bridge` |
| `SerialCommunicator.check_102_send_103` | таймер M103 | extruder/temp policy |
| `RobotArmController` | сборка всего | orchestration / CLI only |

### G/M/T коды

| Код | Действие |
|-----|----------|
| **G0/G1** X Y Z C E F | движение + экструдер |
| **G28** | home → (250, 0, 250), углы 90/90/0 |
| **T0/T1/…** | смена toolhead, X mid + поворот C на 45°·(n+1) |
| **M100** | G0 X250 Z-100 |
| **M101** | G0 X250 Z200 |
| **M102** | в очередь `"102"` (ожидание) |
| **M103** | отправка `"103"` на MCU |
| **M111** | ToF visual servo (contour offsets) |
| **M112 S"name"** | поиск объекта YOLO, змейка X200–400, Y±100 |
| **M200 S"name"** | сохранить left/right JPEG в `imgs/` |

**Параметры поиска M112:** step_x=40 mm, step_y=50 mm, check_interval=0.5 s, `mid_x_toolhead=[354,340,320]`, `angle_between_toolheads=45°`, `table_len=200`.

**ToF centering (M111):** tolerance 5 mm XY, 10 mm Z, target distance ~5 mm; mapping: `x += offset_y`, `y -= offset_x`, `z -= dist_mm`.

### `services/vision_server.py`

| Функция | Назначение |
|---------|------------|
| `denoise_depth_image` | bilateral filter depth |
| `temporal_filter` | EMA кадров depth |
| `sobel_edge_detection` | маска контуров по depth |
| `contour_similarity` | стабильность контура |
| `compute_offsets_mm` | пиксель→мм (FOV 60°×49.5°) |
| `find_best_stable_contour` | выбор контура по score |
| `draw_best_contour` | визуализация + median depth |
| `VisionServer` | MetaSense + YOLOv8s + USB cam |
| `run_vision_server` | Unix socket `/tmp/vision_socket` |
| `USBCameraCapture` | 1920×1080, index 0 |

**MetaSense (serial):** порт был `COM_PORT=/dev/cu.usbserial-202206_DD98130`, 115200; команды `AT+DISP=3`, `AT+UNIT=2`, `AT+FPS=10`.

**Константы vision:** `QUANTIZE=2`, `DELTA_MM=38`, `MIN/MAX_CONTOUR_AREA=60/8000`, `DISPLAY_SIZE=(640,480)`.

### `services/read_send_data_from_encoders.py`

| Функция | Назначение |
|---------|------------|
| `send_at_command` | AT на оба порта |
| `parse_angle` | regex `Angle:([0-9.\-]+)` |
| `run_socket_server` | `/tmp/encoder_socket`, pickle `{angle_a, angle_b}` |

**Калибровка углов на host:**

```
if 180 < angle <= 360: angle -= 360
angles[1] = 360 - angles[1]
angles[1] = 90 - (angles[1] - angles[0])
angles[0] = 90 - angles[0]
```

**Порты (примеры, в yaml):** `3140`, `3130` @ 9600; AT+PRATE=0.

### `services/metasense.py`

- Поток чтения serial, протокол кадров `0xCC 0xA0`, depth frame в queue  
- `sendCmd`, `decodeData`, `start`/`terminate`

### `modules/web_interface.py`

- FastAPI + WebSocket `/ws` @ port **8000**
- Команды WS: `gcode`, `home`, `find_object` (M111), `m102`, `m103`, `jog`, `jog_multi`, `searchobject` (M112), `check_yolo`, `goodbye`
- **Антипаттерн:** `get_controller()` — не повторять
- `build_state_snapshot`, `encode_image_to_base64`, YOLO translation dict

### `modules/gui.py`

- PyQt5: 2D схема руки, вкладки камер, G-code файл, клавиатурный jog (step 10 mm), M102/M103, потоки serial

### Вспомогательные скрипты (не переносить как есть)

- `test_tof_camera/`, `test_drgb/`, `debug_files/`, `hapt_data/` — эксперименты; полезные идеи → tools/

---

## Порты и сокеты (шаблон для `config/robot.yaml`)

```yaml
schema_version: 1

motion:
  port: "/dev/cu.usbserial-2120"   # уточнить: USART2 vs ST-Link VCP
  baudrate: 115200

encoders:
  port_a: "/dev/cu.usbserial-3140"
  port_b: "/dev/cu.usbserial-3130"
  baudrate: 9600

tof:
  port: "/dev/cu.usbserial-202206_DD98130"
  baudrate: 115200

camera:
  index: 0
  width: 1920
  height: 1080

kinematics:
  link_length_mm: 250
  home: { x: 250, y: 0, z: 250 }
  deg_per_step: 0.01875   # 360/(200*16*6)

toolheads:
  angle_between_deg: 45
  mid_x: [354, 340, 320]
  table_len_mm: 200
```

---

## Дорожная карта (каждый этап — рабочая система)

| Этап | Содержание | Срок (вечера+выходные) |
|------|------------|------------------------|
| **0** | Структура, yaml, proto, ZMQ, FakeMotionBus, pre-commit | ~1 неделя |
| **1** | Новая прошивка: binary protocol, segments, safety, IWDG | 1.5–2 недели |
| **2** | HAL-демоны, SHM камера, recorder mcap | ~1 неделя |
| **3** | yolo_node (CoreML), калибровка cam, hand-eye, world.objects | ~1 неделя |
| **4** | kinematics + planner + robot.state | ~0.5 недели |
| **5** | Skills + state machine | ~0.5 недели |
| **6** | Web тонкий клиент, Foxglove, structlog | параллельно |
| **7** | LLM tools, grasp, IBVS, IL, MuJoCo опционально | итеративно |

### Этап 0 — ближайшие шаги

1. `uv init`, `pyproject.toml`, ruff, mypy, pre-commit  
2. `proto/motion.py`, `proto/vision.py` — минимальные модели  
3. `pyrobot/config/load_config.py` + `config/robot.yaml`  
4. `pyrobot/hal/zmq_bus.py` — PUB/SUB/REQ ipc:///tmp/robot/…  
5. `pyrobot/hal/fake_motion.py` — RK4 / lerp → MotionState 100 Hz  
6. `tests/test_fake_motion.py`  
7. Скопировать `docs/hardware-reference/vitalDriver.ioc` → `firmware/robot.ioc` при создании прошивки

### Этап 1 — прошивка

- Задачи FreeRTOS: `safety_task` 1 kHz, `traj_task` 1 kHz, `comm_rx/tx`  
- `motor_t`, ring buffer 32 сегмента, S-curve / trapezoid  
- UART2 921600 (цель), DMA + IDLE + COBS  
- Heartbeat 10 Hz, ESTOP если нет heartbeat 200 ms  

### Этап 2 — первый запуск на железе

```bash
# пример (будущий)
uv run python -m pyrobot.hal.motion_bridge
uv run python -m pyrobot.hal.encoder_bridge
uv run python -m pyrobot.perception.camera_node
uv run python -m pyrobot.recorder
```

---

## ZMQ-топики (черновик)

```
motion.cmd          REQ/REP
motion.state        PUB 100 Hz
encoders.state      PUB
camera.rgb          PUB (meta + shm)
tof.depth           PUB
vision.detections   PUB
world.objects       PUB
skills.cmd          REP
heartbeat           PUB 10 Hz
```

---

## Зависимости legacy (для справки)

- Python 3.9+, OpenCV, NumPy, PySerial, Ultralytics YOLOv8, FastAPI, PyQt5, gcodeparser  
- STM32: FreeRTOS, HAL, Cube FW_F4 1.28.2  

В новом проекте: зафиксировать версии в `pyproject.toml`; ONNX Runtime + CoreML для YOLO на M4.

---

## История

- **2025-05**: удалены `vitalSoft/`, `vitalDriver/`; старт чистой разработки по этому README.
