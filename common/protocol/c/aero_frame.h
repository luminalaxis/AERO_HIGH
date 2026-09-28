/*
 * AERO frame protocol v1 — shared by vehicle-data-logger and
 * wireless-powertrain-control. See docs/protocol.md for the wire format.
 *
 * The same encoded bytes are written to the SD card and sent over the radio,
 * so a frame is identified everywhere by (source_id, boot_id, seq).
 */
#ifndef AERO_FRAME_H
#define AERO_FRAME_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AERO_SYNC0 0xA5u
#define AERO_SYNC1 0x5Au
#define AERO_PROTO_VERSION 1u

#define AERO_HEADER_SIZE 22u
#define AERO_CRC_SIZE 4u
#define AERO_FRAME_OVERHEAD (AERO_HEADER_SIZE + AERO_CRC_SIZE)
#define AERO_MAX_PAYLOAD 1024u
#define AERO_MAX_FRAME (AERO_FRAME_OVERHEAD + AERO_MAX_PAYLOAD)

/* flags */
#define AERO_FLAG_RETRANSMIT 0x01u /* resent in response to a NACK / backfill */

/* source_id */
enum {
    AERO_SRC_VEHICLE_LOGGER = 0x01,
    AERO_SRC_POWERTRAIN = 0x02,
    AERO_SRC_GATEWAY = 0x0F,
    AERO_SRC_PIT = 0x10,
};

/* msg_type */
enum {
    AERO_MSG_HEARTBEAT = 0x01,
    AERO_MSG_SUSPENSION = 0x10,
    AERO_MSG_IMU = 0x11,
    AERO_MSG_GPS = 0x12,
    AERO_MSG_CAN_RAW = 0x20,
    AERO_MSG_NACK = 0x80,
    AERO_MSG_TIME_SYNC = 0x81,
};

typedef struct {
    uint8_t version;
    uint8_t source_id;
    uint8_t msg_type;
    uint8_t flags;
    uint16_t payload_len;
    uint16_t boot_id;
    uint32_t seq;
    uint64_t timestamp_us;
} aero_header_t;

/* zlib-compatible CRC-32 (IEEE 802.3). Start with crc = 0; chainable. */
uint32_t aero_crc32_update(uint32_t crc, const uint8_t *data, size_t len);

/*
 * Encode one frame into out. hdr->version is ignored (AERO_PROTO_VERSION is
 * written). Returns the frame length, or 0 if the payload is too large or
 * out is too small.
 */
size_t aero_frame_encode(uint8_t *out, size_t out_cap, const aero_header_t *hdr,
                         const uint8_t *payload);

/* ---- streaming decoder (UART / SD file / any byte stream) ---- */

typedef void (*aero_frame_cb)(void *ctx, const aero_header_t *hdr,
                              const uint8_t *payload);

typedef struct {
    uint8_t buf[AERO_MAX_FRAME];
    size_t pos;
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t header_errors;   /* bad version or payload_len */
    uint32_t bytes_discarded; /* bytes skipped while hunting for sync */
} aero_parser_t;

void aero_parser_init(aero_parser_t *p);

/* Feed any number of bytes; cb is called once per valid frame. */
void aero_parser_feed(aero_parser_t *p, const uint8_t *data, size_t len,
                      aero_frame_cb cb, void *ctx);

/* ---- sequence tracking (loss detection) ---- */

typedef enum {
    AERO_SEQ_NEW_SESSION, /* first frame, or boot_id changed */
    AERO_SEQ_IN_ORDER,
    AERO_SEQ_GAP,  /* frames [gap_first, gap_first + gap_count) are missing */
    AERO_SEQ_LATE, /* older than expected: duplicate, reordered or backfill */
} aero_seq_result_t;

typedef struct {
    bool started;
    uint16_t boot_id;
    uint32_t expected;
    uint32_t received;
    uint32_t lost; /* cumulative frames reported in gaps (this session) */
} aero_seq_tracker_t;

void aero_seq_init(aero_seq_tracker_t *t);
aero_seq_result_t aero_seq_update(aero_seq_tracker_t *t, uint16_t boot_id,
                                  uint32_t seq, uint32_t *gap_first,
                                  uint32_t *gap_count);

/* ---- NACK payload (pit -> car): array of 10-byte ranges ---- */

#define AERO_NACK_RANGE_SIZE 10u

typedef struct {
    uint8_t source_id;
    uint16_t boot_id;
    uint32_t first_seq;
    uint16_t count;
} aero_nack_range_t;

size_t aero_nack_encode(uint8_t *out, size_t out_cap,
                        const aero_nack_range_t *ranges, size_t n);
/* Returns number of ranges decoded (payload_len / 10, capped at max). */
size_t aero_nack_decode(const uint8_t *payload, size_t payload_len,
                        aero_nack_range_t *ranges, size_t max);

#ifdef __cplusplus
}
#endif

#endif /* AERO_FRAME_H */
