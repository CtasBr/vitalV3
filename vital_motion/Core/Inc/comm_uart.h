/**
 * @file comm_uart.h
 * @brief M2: приём/ответ по motion-UART (echo, PING).
 */
#ifndef COMM_UART_H
#define COMM_UART_H

#ifdef __cplusplus
extern "C" {
#endif

void comm_uart_init(void);
/** Блокирующий цикл: опрос RX, ответы. Вызывать из задачи FreeRTOS. */
void comm_uart_poll_loop(void);

#ifdef __cplusplus
}
#endif

#endif /* COMM_UART_H */
