/**
 * @file comm_uart.c
 * @brief M2–M4: текст (PING, STEP) + бинарные пакеты COBS/CRC.
 */
#include "comm_uart.h"
#include "board_config.h"
#include "main.h"
#include "motor.h"
#include "protocol.h"
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

static void comm_uart_send_raw_cobs(const pkt_raw_t *pkt)
{
  uint8_t wire[PKT_COBS_MAX + 1];
  const uint32_t enc_len = protocol_cobs_encode((const uint8_t *)pkt, PKT_RAW_SIZE, wire, PKT_COBS_MAX);
  if (enc_len == 0)
  {
    return;
  }
  comm_uart_tx(wire, (uint16_t)enc_len);
  const uint8_t zero = 0;
  comm_uart_tx(&zero, 1);
}

static void comm_uart_send_pong(uint16_t seq)
{
  pkt_raw_t pkt;
  if (protocol_build(&pkt, PKT_PONG, seq, NULL, 0) != 0)
  {
    return;
  }
  comm_uart_send_raw_cobs(&pkt);
}

static void comm_uart_handle_binary(const pkt_raw_t *pkt)
{
  if (pkt->type == (uint8_t)PKT_PING)
  {
    comm_uart_send_pong(pkt->seq);
  }
}

void comm_uart_init(void)
{
#if MOTION_LINK_USE_USART3
  comm_uart_tx_str("\r\n=== vital_motion M4 (USART3 / ST-Link) ===\r\n");
#else
  comm_uart_tx_str("\r\n=== vital_motion M4 (USART2) ===\r\n");
#endif
  comm_uart_tx_str("Text: PING | STEP <n> [arr]\r\n");
  comm_uart_tx_str("Binary: COBS frame PKT_PING (see tools/uart_pkt_ping.py)\r\n");
  protocol_rx_reset();
}

static void comm_uart_handle_line(const char *line)
{
  if (strcmp(line, "PING") == 0)
  {
    comm_uart_tx_str("PONG\r\n");
    return;
  }

  if (strcmp(line, "BPING") == 0)
  {
    comm_uart_send_pong(0);
    comm_uart_tx_str("BIN PONG sent\r\n");
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
  pkt_raw_t pkt;

  for (;;)
  {
    if (HAL_UART_Receive(MOTION_UART, &byte, 1, 20) != HAL_OK)
    {
      osDelay(1);
      continue;
    }

    const int br = protocol_rx_feed(byte, &pkt);
    if (br == 1)
    {
      comm_uart_handle_binary(&pkt);
      line_len = 0;
      continue;
    }
    if (br < 0)
    {
      protocol_rx_reset();
    }

    /* Текстовый режим (с echo) */
    comm_uart_tx(&byte, 1);

    if (byte == '\r')
    {
      continue;
    }

    if (byte == '\n')
    {
      protocol_rx_reset();
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
