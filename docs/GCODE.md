# G-code — как отправлять команды

Тонкий адаптер (не язык поведения). Поддерживаются **G28**, **G0**, **G1** (одна строка за раз).

## Предусловие

Запущен стек (одной командой):

```bash
python -m pyrobot.launcher
# или после установки:
robot
```

Откройте веб: http://127.0.0.1:8080 (на RPi: http://\<ip-rpi\>:8080)

---

## 1. Веб-интерфейс

1. Поле **G-code** → введите строку → **Отправить**
2. Примеры:
   - `G28` — домой (углы 90/90/0/0)
   - `G1 X260 Y0 Z240 F300` — линейный ход, подача F мм/мин
   - `G0 X250 Y0 Z250` — быстрый ход

---

## 2. CLI (через motion_daemon + ZMQ)

Демоны должны быть запущены (`robot` или вручную `encoder_daemon` + `motion_daemon`).

```bash
python -m pyrobot.hal.motion_cli g28
python -m pyrobot.hal.motion_cli g1 255 0 245 --f 300
python -m pyrobot.hal.motion_cli g0 250 0 250
python -m pyrobot.hal.motion_cli gcode "G1 X250 Y0 Z250 F600"
python -m pyrobot.hal.motion_cli state
python -m pyrobot.hal.motion_cli reset-fault
python -m pyrobot.hal.motion_cli estop
```

Прямой UART (без daemon): добавьте `--direct` (остановите `motion_daemon`).

---

## 3. HTTP API (для скриптов / UI)

| Метод | URL | Тело |
|--------|-----|------|
| GET | `/api/state` | — |
| POST | `/api/gcode` | `{"line": "G1 X250 Y0 Z250 F300"}` |
| POST | `/api/home` | — (G28) |
| POST | `/api/reset-fault` | — |
| POST | `/api/estop` | — |

```bash
curl -s http://127.0.0.1:8080/api/state | jq
curl -s -X POST http://127.0.0.1:8080/api/gcode \
  -H 'Content-Type: application/json' \
  -d '{"line":"G28"}'
```

---

## Синтаксис строки

| Код | Смысл |
|-----|--------|
| `G28` | Home |
| `G0 X.. Y.. Z..` | Rapid linear (пропущенные оси = текущая поза) |
| `G1 X.. Y.. Z.. F..` | Linear, F = мм/мин |
| `; комментарий` | Игнорируется после `;` |

Ось **E** (если есть) задаёт целевой угол **D** в градусах.
