/**
 * @file comm_uart.c
 * @brief M2/M3: UART команды (PING, STEP).
 */
#include "comm_uart.h"
#include "board_config.h"
#include "main.h"
#include "motor.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if MOTION_LINK_USE_USART3
extern UART_HandleTypeDef huart3;
#define MOTION_UART (&huart3)
#else
extern UART_HandleTypeDef huart2;
#define MOTION_UART (&huart2)
#endif

#define RX_LINE_MAX 48

static void comm_uart_tx(const uint8_t *data, uint16_t len)
{
  (void)HAL_UART_Transmit(MOTION_UART, (uint8_t *)data, len, 200);
}

static void comm_uart_tx_str(const char *s)
{
  comm_uart_tx((const uint8_t *)s, (uint16_t)strlen(s));
}

void comm_uart_init(void)
{
#if MOTION_LINK_USE_USART3
  comm_uart_tx_str("\r\n=== vital_motion M3 (USART3 / ST-Link) ===\r\n");
#else
  comm_uart_tx_str("\r\n=== vital_motion M3 (USART2) ===\r\n");
#endif
  comm_uart_tx_str("PING -> PONG | STEP <n> [arr] -> move axis A\r\n");
  comm_uart_tx_str("Example: STEP 200  (slow)  STEP -100 4000\r\n");
}

static void comm_uart_handle_line(const char *line)
{
  if (strcmp(line, "PING") == 0)
  {
    comm_uart_tx_str("PONG\r\n");
    return;
  }

  if (strncmp(line, "STEP ", 5) == 0)
  {
    int steps = 0;
    unsigned long arr = 5000UL;
    const int n = sscanf(line + 5, "%d %lu", &steps, &arr);
    if (n < 1)
    {
      comm_uart_tx_str("ERR STEP parse\r\n");
      return;
    }
    if (n < 2)
    {
      arr = 5000UL;
    }

    comm_uart_tx_str("RUN STEP...\r\n");
    const int rc = motor_axis_a_move((int32_t)steps, (uint32_t)arr);
    if (rc == 0)
    {
      comm_uart_tx_str("OK STEP\r\n");
    }
    else
    {
      comm_uart_tx_str("ERR STEP timeout\r\n");
    }
    return;
  }

  comm_uart_tx_str("ERR unknown cmd\r\n");
}

void comm_uart_poll_loop(void)
{
  uint8_t byte;
  char line[RX_LINE_MAX];
  unsigned line_len = 0;

  for (;;)
  {
    if (HAL_UART_Receive(MOTION_UART, &byte, 1, 20) != HAL_OK)
    {
      osDelay(1);
      continue;
    }

    comm_uart_tx(&byte, 1);

    if (byte == '\r')
    {
      continue;
    }

    if (byte == '\n')
    {
      line[line_len] = '\0';
      if (line_len > 0U)
      {
        comm_uart_handle_line(line);
      }
      line_len = 0;
      continue;
    }

    if (line_len < RX_LINE_MAX - 1U)
    {
      line[line_len++] = (char)byte;
    }
    else
    {
      line_len = 0;
    }
  }
}
