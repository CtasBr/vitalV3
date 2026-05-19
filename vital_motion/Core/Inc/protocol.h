/**
 * @file protocol.h
 * @brief M4: COBS + CRC16 binary packets (60 B raw).
 */
#ifndef PROTOCOL_H
#define PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define PKT_MAGIC 0x56U
#define PKT_VERSION 1U
#define PKT_RAW_SIZE 60U
#define PKT_PAYLOAD_MAX 44U
#define PKT_COBS_MAX 70U

typedef enum {
  PKT_PING = 0x01,
  PKT_PONG = 0x02,
  PKT_MOVE_SEGMENT = 0x10,
  PKT_TELEMETRY = 0x20,
  PKT_SEGMENT_DONE = 0x21,
  PKT_ESTOP = 0x30,
  PKT_FAULT = 0x31,
  PKT_HEARTBEAT = 0x3F,
} pkt_type_t;

typedef struct __attribute__((packed)) {
  uint8_t magic;
  uint8_t version;
  uint8_t type;
  uint8_t flags;
  uint16_t seq;
  uint16_t payload_len;
  uint8_t payload[PKT_PAYLOAD_MAX];
  uint16_t crc16;
} pkt_raw_t;

uint16_t protocol_crc16(const uint8_t *data, uint32_t len);

/** Build raw packet in @p out (60 bytes). Returns 0 on success. */
int protocol_build(pkt_raw_t *out, pkt_type_t type, uint16_t seq, const uint8_t *payload,
                   uint16_t payload_len);

/** Validate CRC and magic. Returns 0 if OK. */
int protocol_validate(const pkt_raw_t *pkt);

/** COBS encode raw 60B → wire buffer. Returns encoded length (excl. trailing 0). */
uint32_t protocol_cobs_encode(const uint8_t *raw, uint32_t raw_len, uint8_t *out,
                              uint32_t out_max);

/**
 * Feed RX bytes; when one COBS frame complete, decode to @p raw_out.
 * Returns 1 if frame ready, 0 if need more bytes, -1 on error.
 */
int protocol_rx_feed(uint8_t byte, pkt_raw_t *raw_out);

/** Reset RX parser (e.g. after line noise). */
void protocol_rx_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_H */
