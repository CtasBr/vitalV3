/**
 * @file comm_uart.c
 * @brief M2–M5: текст (PING, STEP) + бинарные пакеты COBS/CRC.
 */
#include "comm_uart.h"
#include "board_config.h"
#include "main.h"
#include "motor.h"
#include "motion_limits.h"
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
#define AXIS_COUNT MOTION_LIMIT_AXES
/* Host HB while idle; must exceed motion_daemon period (e.g. 100 ms @ 10 Hz). */
#define COMM_UART_WATCHDOG_IDLE_MS 2500U
static uint16_t g_tx_seq;
static uint8_t g_fault_code;
static uint32_t g_last_hb_ms;
static uint8_t g_hb_seen;

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

static void comm_uart_send_segment_done(uint16_t seq, const int32_t done_steps[4])
{
  uint8_t payload[16];
  pkt_raw_t pkt;
  memcpy(&payload[0], &done_steps[0], sizeof(done_steps[0]));
  memcpy(&payload[4], &done_steps[1], sizeof(done_steps[1]));
  memcpy(&payload[8], &done_steps[2], sizeof(done_steps[2]));
  memcpy(&payload[12], &done_steps[3], sizeof(done_steps[3]));
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

static void comm_uart_send_heartbeat(uint16_t seq)
{
  uint8_t payload[4];
  pkt_raw_t pkt;
  uint32_t uptime = HAL_GetTick();
  memcpy(payload, &uptime, sizeof(uptime));
  if (protocol_build(&pkt, PKT_HEARTBEAT, seq, payload, sizeof(payload)) != 0)
  {
    return;
  }
  comm_uart_send_raw_cobs(&pkt);
}

static void comm_uart_send_telemetry(void)
{
  uint8_t payload[24];
  pkt_raw_t pkt;
  int32_t pos[4] = {
      motor_axis_pos_steps(0),
      motor_axis_pos_steps(1),
      motor_axis_pos_steps(2),
      motor_axis_pos_steps(3),
  };
  uint8_t in_motion_mask = (motor_axis_in_motion(0) ? 0x1U : 0U) |
                           (motor_axis_in_motion(1) ? 0x2U : 0U) |
                           (motor_axis_in_motion(2) ? 0x4U : 0U) |
                           (motor_axis_in_motion(3) ? 0x8U : 0U);
  uint8_t fault = g_fault_code;
  uint16_t reserved = 0;
  int32_t done = 0; /* reserved */

  memcpy(&payload[0], &pos[0], sizeof(pos[0]));
  memcpy(&payload[4], &pos[1], sizeof(pos[1]));
  memcpy(&payload[8], &pos[2], sizeof(pos[2]));
  memcpy(&payload[12], &pos[3], sizeof(pos[3]));
  payload[16] = in_motion_mask;
  payload[17] = fault;
  memcpy(&payload[18], &reserved, sizeof(reserved));
  memcpy(&payload[20], &done, sizeof(done));

  g_tx_seq++;
  if (protocol_build(&pkt, PKT_TELEMETRY, g_tx_seq, payload, sizeof(payload)) != 0)
  {
    return;
  }
  comm_uart_send_raw_cobs(&pkt);
}

static int comm_uart_steps_within_soft_limits(const int32_t steps[4])
{
  for (uint8_t i = 0; i < AXIS_COUNT; i++)
  {
    const int32_t pos = motor_axis_pos_steps(i);
    const int32_t target = pos + steps[i];
    if (target < MOTION_SOFT_LIMIT_MIN[i] || target > MOTION_SOFT_LIMIT_MAX[i])
    {
      return 0;
    }
  }
  return 1;
}

static void comm_uart_handle_binary(const pkt_raw_t *pkt)
{
  if (pkt->type == (uint8_t)PKT_HEARTBEAT)
  {
    g_hb_seen = 1U;
    g_last_hb_ms = HAL_GetTick();
    comm_uart_send_heartbeat(pkt->seq);
    return;
  }

  if (pkt->type == (uint8_t)PKT_ESTOP)
  {
    motor_estop_all();
    g_fault_code = 1;
    comm_uart_send_fault(pkt->seq, -2001);
    return;
  }

  if (pkt->type == (uint8_t)PKT_RESET_FAULT)
  {
    g_fault_code = 0;
    g_last_hb_ms = HAL_GetTick();
    comm_uart_send_fault(pkt->seq, 0);
    return;
  }

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

    int32_t steps[4] = {0, 0, 0, 0};
    uint32_t arr[4] = {5000U, 5000U, 5000U, 5000U};
    memcpy(&steps[0], &pkt->payload[0], sizeof(steps[0]));
    memcpy(&steps[1], &pkt->payload[4], sizeof(steps[1]));
    memcpy(&steps[2], &pkt->payload[8], sizeof(steps[2]));
    memcpy(&steps[3], &pkt->payload[12], sizeof(steps[3]));
    memcpy(&arr[0], &pkt->payload[16], sizeof(arr[0]));
    memcpy(&arr[1], &pkt->payload[20], sizeof(arr[1]));
    memcpy(&arr[2], &pkt->payload[24], sizeof(arr[2]));
    memcpy(&arr[3], &pkt->payload[28], sizeof(arr[3]));

    if (!comm_uart_steps_within_soft_limits(steps))
    {
      g_fault_code = 4;
      comm_uart_send_fault(pkt->seq, -1004);
      return;
    }

    if (motor_move_4axes(steps, arr) == 0)
    {
      g_fault_code = 0;
      comm_uart_send_segment_done(pkt->seq, steps);
    }
    else
    {
      g_fault_code = 2;
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
  comm_uart_tx_str("Binary: PING/MOVE/TELEMETRY/HEARTBEAT/ESTOP/RESET_FAULT\r\n");
  protocol_rx_reset();
  g_tx_seq = 0;
  g_fault_code = 0;
  g_last_hb_ms = HAL_GetTick();
  g_hb_seen = 0U;
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
    const int32_t steps4[4] = {(int32_t)steps, 0, 0, 0};
    const uint32_t arr4[4] = {(uint32_t)arr, 5000U, 5000U, 5000U};
    const int rc = motor_move_4axes(steps4, arr4);
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

void comm_uart_service_rx(void)
{
  uint8_t byte;
  pkt_raw_t pkt;

  while (HAL_UART_Receive(MOTION_UART, &byte, 1, 0) == HAL_OK)
  {
    const int br = protocol_rx_feed(byte, &pkt);
    if (br == 1)
    {
      comm_uart_handle_binary(&pkt);
      continue;
    }
    if (br < 0)
    {
      protocol_rx_reset();
    }
  }
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

    if (g_hb_seen && !motor_any_in_motion() && (now - g_last_hb_ms) > COMM_UART_WATCHDOG_IDLE_MS)
    {
      g_fault_code = 3;
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
