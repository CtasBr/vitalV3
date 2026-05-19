# Firmware (STM32F429)

Stage 1: motion core with binary protocol (COBS + CRC), segment buffer, safety.

## CubeMX

Open [`docs/hardware-reference/vitalDriver.ioc`](../docs/hardware-reference/vitalDriver.ioc) or copy to `robot.ioc` here. **Do not change pin assignment** — see root [README.md](../README.md#cubemx--stm32f429zi-nucleo-f429zi--не-менять-проводку).

Generate code with:

- MCU: STM32F429ZIT6, NUCLEO-F429ZI
- FreeRTOS CMSIS-OS v2
- TIM1–4 PWM (step), PF0–PF3 DIR
- USART2 @ PD5/PA3 (motion link to host)
- ETH RMII (phase 2)
