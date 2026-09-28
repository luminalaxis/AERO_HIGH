/* Host-side unit tests: `make test` */
#include "aero_frame.h"

#include <stdio.h>
#include <string.h>

static int failures;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
            failures++;                                                    \
        }                                                                  \
    } while (0)

/* Same vector as docs/protocol.md and test_aero_protocol.py */
static const uint8_t golden[] = {
    0xa5, 0x5a, 0x01, 0x01, 0x10, 0x00, 0x04, 0x00, 0x02, 0x01, 0x04,
    0x03, 0x02, 0x01, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,
    0xde, 0xad, 0xbe, 0xef, 0x32, 0xc9, 0x27, 0xd5,
};

static const aero_header_t golden_hdr = {
    .source_id = AERO_SRC_VEHICLE_LOGGER,
    .msg_type = AERO_MSG_SUSPENSION,
    .payload_len = 4,
    .boot_id = 0x0102,
    .seq = 0x01020304,
    .timestamp_us = 0x0102030405060708ull,
};
static const uint8_t golden_payload[] = {0xde, 0xad, 0xbe, 0xef};

typedef struct {
    int n;
    aero_header_t last;
    uint8_t payload[AERO_MAX_PAYLOAD];
} sink_t;

static void on_frame(void *ctx, const aero_header_t *h, const uint8_t *payload)
{
    sink_t *s = ctx;
    s->n++;
    s->last = *h;
    memcpy(s->payload, payload, h->payload_len);
}

static void test_crc(void)
{
    const uint8_t check[] = "123456789";
    CHECK(aero_crc32_update(0, check, 9) == 0xCBF43926u);
    uint32_t c = aero_crc32_update(0, check, 4);
    CHECK(aero_crc32_update(c, check + 4, 5) == 0xCBF43926u);
}

static void test_encode_golden(void)
{
    uint8_t buf[64];
    size_t n = aero_frame_encode(buf, sizeof buf, &golden_hdr, golden_payload);
    CHECK(n == sizeof golden);
    CHECK(memcmp(buf, golden, sizeof golden) == 0);
    CHECK(aero_frame_encode(buf, sizeof golden - 1, &golden_hdr, golden_payload) == 0);
}

static void test_parse_with_noise_and_corruption(void)
{
    aero_parser_t p;
    sink_t s = {0};
    aero_parser_init(&p);

    uint8_t stream[200];
    size_t n = 0;
    const uint8_t noise[] = {0x00, 0xa5, 0x11, 0xa5, 0xa5};
    memcpy(stream + n, noise, sizeof noise); n += sizeof noise;
    memcpy(stream + n, golden, sizeof golden); n += sizeof golden;
    /* corrupted copy: must be rejected by CRC */
    memcpy(stream + n, golden, sizeof golden);
    stream[n + 23] ^= 0xFF;
    n += sizeof golden;
    memcpy(stream + n, golden, sizeof golden); n += sizeof golden;

    /* feed byte by byte to exercise partial frames */
    for (size_t i = 0; i < n; i++)
        aero_parser_feed(&p, &stream[i], 1, on_frame, &s);

    CHECK(s.n == 2);
    CHECK(p.crc_errors == 1);
    CHECK(s.last.seq == golden_hdr.seq);
    CHECK(s.last.boot_id == golden_hdr.boot_id);
    CHECK(s.last.timestamp_us == golden_hdr.timestamp_us);
    CHECK(memcmp(s.payload, golden_payload, 4) == 0);
}

static void test_truncated_frame_then_valid(void)
{
    /* A frame cut off mid-way (e.g. power loss on SD write) followed by a
     * valid frame: the valid one must still be recovered. */
    aero_parser_t p;
    sink_t s = {0};
    aero_parser_init(&p);
    aero_parser_feed(&p, golden, 12, on_frame, &s);
    aero_parser_feed(&p, golden, sizeof golden, on_frame, &s);
    aero_parser_feed(&p, golden, sizeof golden, on_frame, &s);
    /* the truncated frame swallows bytes of the next one and fails CRC;
     * resync rescans the buffer so both following frames are recovered */
    CHECK(s.n == 2);
    CHECK(p.crc_errors == 1);
}

static void test_seq_tracker(void)
{
    aero_seq_tracker_t t;
    uint32_t first = 0, count = 0;
    aero_seq_init(&t);

    CHECK(aero_seq_update(&t, 7, 100, &first, &count) == AERO_SEQ_NEW_SESSION);
    CHECK(aero_seq_update(&t, 7, 101, &first, &count) == AERO_SEQ_IN_ORDER);
    CHECK(aero_seq_update(&t, 7, 105, &first, &count) == AERO_SEQ_GAP);
    CHECK(first == 102 && count == 3);
    CHECK(t.lost == 3);
    CHECK(aero_seq_update(&t, 7, 103, &first, &count) == AERO_SEQ_LATE);
    CHECK(aero_seq_update(&t, 8, 0, &first, &count) == AERO_SEQ_NEW_SESSION);

    /* wrap-around */
    aero_seq_init(&t);
    aero_seq_update(&t, 1, 0xFFFFFFFEu, NULL, NULL);
    CHECK(aero_seq_update(&t, 1, 0xFFFFFFFFu, NULL, NULL) == AERO_SEQ_IN_ORDER);
    CHECK(aero_seq_update(&t, 1, 0, NULL, NULL) == AERO_SEQ_IN_ORDER);
    CHECK(aero_seq_update(&t, 1, 3, &first, &count) == AERO_SEQ_GAP);
    CHECK(first == 1 && count == 2);
}

static void test_nack_roundtrip(void)
{
    aero_nack_range_t in[2] = {
        {AERO_SRC_VEHICLE_LOGGER, 3, 1000, 20},
        {AERO_SRC_POWERTRAIN, 4, 0xFFFFFFF0u, 65535},
    };
    aero_nack_range_t out[2];
    uint8_t buf[32];
    size_t n = aero_nack_encode(buf, sizeof buf, in, 2);
    CHECK(n == 20);
    CHECK(aero_nack_decode(buf, n, out, 2) == 2);
    CHECK(out[1].source_id == AERO_SRC_POWERTRAIN && out[1].boot_id == 4 &&
          out[1].first_seq == 0xFFFFFFF0u && out[1].count == 65535);
}

int main(void)
{
    test_crc();
    test_encode_golden();
    test_parse_with_noise_and_corruption();
    test_truncated_frame_then_valid();
    test_seq_tracker();
    test_nack_roundtrip();
    if (failures) {
        printf("%d failure(s)\n", failures);
        return 1;
    }
    printf("all C protocol tests passed\n");
    return 0;
}
