from __future__ import annotations

MCU_FAULT_LABELS: dict[int, str] = {
    0: "",
    1: "estop",
    2: "move_timeout",
    3: "heartbeat_watchdog",
    4: "soft_limit",
}


def mcu_fault_message(code: int) -> str:
    if code == 0:
        return ""
    label = MCU_FAULT_LABELS.get(code, f"mcu_fault_{code}")
    return label
