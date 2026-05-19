/**
 * @file protocol.c
 */
#include "protocol.h"
#include <string.h>

static uint8_t rx_cobs[PKT_COBS_MAX];
static uint32_t rx_len;

uint16_t protocol_crc16(const uint8_t *data, uint32_t len)
{
  uint16_t crc = 0xFFFFU;
  for (uint32_t i = 0; i < len; i++)
  {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++)
    {
      if (crc & 0x8000U)
      {
        crc = (uint16_t)((crc << 1) ^ 0x1021U);
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}

int protocol_build(pkt_raw_t *out, pkt_type_t type, uint16_t seq, const uint8_t *payload,
                   uint16_t payload_len)
{
  if (out == NULL || payload_len > PKT_PAYLOAD_MAX)
  {
    return -1;
  }
  memset(out, 0, sizeof(*out));
  out->magic = PKT_MAGIC;
  out->version = PKT_VERSION;
  out->type = (uint8_t)type;
  out->seq = seq;
  out->payload_len = payload_len;
  if (payload_len > 0 && payload != NULL)
  {
    memcpy(out->payload, payload, payload_len);
  }
  out->crc16 = protocol_crc16((const uint8_t *)out, PKT_RAW_SIZE - 2U);
  return 0;
}

int protocol_validate(const pkt_raw_t *pkt)
{
  if (pkt == NULL || pkt->magic != PKT_MAGIC || pkt->version != PKT_VERSION)
  {
    return -1;
  }
  if (pkt->payload_len > PKT_PAYLOAD_MAX)
  {
    return -1;
  }
  const uint16_t expect = protocol_crc16((const uint8_t *)pkt, PKT_RAW_SIZE - 2U);
  return (expect == pkt->crc16) ? 0 : -1;
}

uint32_t protocol_cobs_encode(const uint8_t *raw, uint32_t raw_len, uint8_t *out,
                              uint32_t out_max)
{
  uint32_t read_index = 0;
  uint32_t write_index = 1;
  uint32_t code_index = 0;
  uint8_t code = 1;

  if (raw_len == 0 || out_max < 2)
  {
    return 0;
  }

  while (read_index < raw_len)
  {
    if (raw[read_index] == 0)
    {
      out[code_index] = code;
      code = 1;
      code_index = write_index++;
      read_index++;
    }
    else
    {
      out[write_index++] = raw[read_index++];
      code++;
      if (code == 0xFF)
      {
        out[code_index] = code;
        code = 1;
        code_index = write_index++;
      }
    }
    if (write_index >= out_max)
    {
      return 0;
    }
  }
  out[code_index] = code;
  return write_index;
}

static int cobs_decode_frame(const uint8_t *in, uint32_t in_len, uint8_t *raw, uint32_t raw_len)
{
  if (in_len < 2 || raw_len < PKT_RAW_SIZE)
  {
    return -1;
  }
  uint32_t read_index = 0;
  uint32_t write_index = 0;
  while (read_index < in_len)
  {
    uint8_t code = in[read_index++];
    if (code == 0)
    {
      return -1;
    }
    for (uint8_t i = 1; i < code; i++)
    {
      if (read_index >= in_len || write_index >= raw_len)
      {
        return -1;
      }
      if (in[read_index] == 0)
      {
        return -1;
      }
      raw[write_index++] = in[read_index++];
    }
    if (code < 0xFF && read_index < in_len)
    {
      if (write_index >= raw_len)
      {
        return -1;
      }
      raw[write_index++] = 0;
    }
  }
  return (write_index == PKT_RAW_SIZE) ? 0 : -1;
}

void protocol_rx_reset(void)
{
  rx_len = 0;
}

int protocol_rx_feed(uint8_t byte, pkt_raw_t *raw_out)
{
  if (byte == 0)
  {
    if (rx_len == 0)
    {
      return 0;
    }
    uint8_t decoded[PKT_RAW_SIZE];
    if (cobs_decode_frame(rx_cobs, rx_len, decoded, PKT_RAW_SIZE) != 0)
    {
      protocol_rx_reset();
      return -1;
    }
    protocol_rx_reset();
    memcpy(raw_out, decoded, PKT_RAW_SIZE);
    return (protocol_validate(raw_out) == 0) ? 1 : -1;
  }

  if (rx_len >= PKT_COBS_MAX)
  {
    protocol_rx_reset();
    return -1;
  }
  rx_cobs[rx_len++] = byte;
  return 0;
}
