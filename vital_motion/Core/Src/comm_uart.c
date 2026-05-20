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
static uint16_t g_tx_seq;

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

static void comm_uart_send_segment_done(uint16_t seq, int32_t done_steps)
{
  uint8_t payload[4];
  pkt_raw_t pkt;
  memcpy(payload, &done_steps, sizeof(done_steps));
  if (protocol_build(&pkt, PKT_SEGMENT_DONE, seq, payload, sizeof(payload)) != 0)
  {
    return;
  }
  /* USB-CDC/ST-Link can occasionally drop a byte; duplicate ACK improves robustness. */
  comm_uart_send_raw_cobs(&pkt);
  osDelay(2);
  comm_uart_send_raw_cobs(&pkt);
}

static void comm_uart_send_fault(uint16_t seq, int32_t code)
{
  uint8_t payload[4];
  pkt_raw_t pkt;
  memcpy(payload, &code, sizeof(code));
  if (protocol_build(&pkt, PKT_FAULT, seq, payload, sizeof(payload)) != 0)
  {
    return;
  }
  comm_uart_send_raw_cobs(&pkt);
}

static void comm_uart_send_telemetry(void)
{
  uint8_t payload[12];
  pkt_raw_t pkt;
  int32_t pos = motor_axis_a_pos_steps();
  uint8_t in_motion = motor_axis_a_in_motion();
  uint8_t fault = 0;
  uint16_t reserved = 0;
  int32_t done = 0;

  memcpy(&payload[0], &pos, sizeof(pos));
  payload[4] = in_motion;
  payload[5] = fault;
  memcpy(&payload[6], &reserved, sizeof(reserved));
  memcpy(&payload[8], &done, sizeof(done));

  g_tx_seq++;
  if (protocol_build(&pkt, PKT_TELEMETRY, g_tx_seq, payload, sizeof(payload)) != 0)
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
    return;
  }

  if (pkt->type == (uint8_t)PKT_MOVE_SEGMENT)
  {
    /* M5 payload layout (LE):
     * [0..3]   int32 steps_a
     * [4..7]   int32 steps_b
     * [8..11]  int32 steps_c
     * [12..15] int32 steps_d
     * [16..19] uint32 arr_a
     * [20..23] uint32 arr_b
     * [24..27] uint32 arr_c
     * [28..31] uint32 arr_d
     */
    if (pkt->payload_len < 32U)
    {
      comm_uart_send_fault(pkt->seq, -1001);
      return;
    }

    int32_t steps_a = 0, steps_b = 0, steps_c = 0, steps_d = 0;
    uint32_t arr = 5000U;
    memcpy(&steps_a, &pkt->payload[0], sizeof(steps_a));
    memcpy(&steps_b, &pkt->payload[4], sizeof(steps_b));
    memcpy(&steps_c, &pkt->payload[8], sizeof(steps_c));
    memcpy(&steps_d, &pkt->payload[12], sizeof(steps_d));
    memcpy(&arr, &pkt->payload[16], sizeof(arr));

    /* M5-stage: implemented physically only for axis A; require B/C/D to be 0. */
    if ((steps_b != 0) || (steps_c != 0) || (steps_d != 0))
    {
      comm_uart_send_fault(pkt->seq, -1002);
      return;
    }

    if (motor_axis_a_move(steps_a, arr) == 0)
    {
      comm_uart_send_segment_done(pkt->seq, steps_a);
    }
    else
    {
      comm_uart_send_fault(pkt->seq, -1003);
    }
  }
}

void comm_uart_init(void)
{
#if MOTION_LINK_USE_USART3
  comm_uart_tx_str("\r\n=== vital_motion M5 (USART3 / ST-Link) ===\r\n");
#else
  comm_uart_tx_str("\r\n=== vital_motion M5 (USART2) ===\r\n");
#endif
  comm_uart_tx_str("Text: PING | STEP <n> [arr]\r\n");
  comm_uart_tx_str("Binary: PKT_PING, PKT_MOVE_SEGMENT(4axis), PKT_TELEMETRY\r\n");
  protocol_rx_reset();
  g_tx_seq = 0;
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
  uint32_t last_telemetry_ms = 0;

  for (;;)
  {
    uint32_t now = HAL_GetTick();
    if ((now - last_telemetry_ms) >= 100U)
    {
      comm_uart_send_telemetry();
      last_telemetry_ms = now;
    }

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

    /* In binary COBS mode do not echo bytes back to host. */
    if (protocol_rx_busy())
    {
      continue;
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
