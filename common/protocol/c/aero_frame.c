#include "aero_frame.h"

#include <string.h>

/* ---- little-endian helpers ---- */

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
    put_u16(p, (uint16_t)v);
    put_u16(p + 2, (uint16_t)(v >> 16));
}

static void put_u64(uint8_t *p, uint64_t v)
{
    put_u32(p, (uint32_t)v);
    put_u32(p + 4, (uint32_t)(v >> 32));
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)get_u16(p) | ((uint32_t)get_u16(p + 2) << 16);
}

static uint64_t get_u64(const uint8_t *p)
{
    return (uint64_t)get_u32(p) | ((uint64_t)get_u32(p + 4) << 32);
}

/* ---- CRC-32, nibble table (64 bytes of flash) ---- */

static const uint32_t crc_nibble[16] = {
    0x00000000u, 0x1DB71064u, 0x3B6E20C8u, 0x26D930ACu,
    0x76DC4190u, 0x6B6B51F4u, 0x4DB26158u, 0x5005713Cu,
    0xEDB88320u, 0xF00F9344u, 0xD6D6A3E8u, 0xCB61B38Cu,
    0x9B64C2B0u, 0x86D3D2D4u, 0xA00AE278u, 0xBDBDF21Cu,
};

uint32_t aero_crc32_update(uint32_t crc, const uint8_t *data, size_t len)
{
    crc = ~crc;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        crc = (crc >> 4) ^ crc_nibble[crc & 0x0F];
        crc = (crc >> 4) ^ crc_nibble[crc & 0x0F];
    }
    return ~crc;
}

/* ---- encode ---- */

size_t aero_frame_encode(uint8_t *out, size_t out_cap, const aero_header_t *hdr,
                         const uint8_t *payload)
{
    size_t total = AERO_FRAME_OVERHEAD + hdr->payload_len;
    if (hdr->payload_len > AERO_MAX_PAYLOAD || out_cap < total)
        return 0;

    out[0] = AERO_SYNC0;
    out[1] = AERO_SYNC1;
    out[2] = AERO_PROTO_VERSION;
    out[3] = hdr->source_id;
    out[4] = hdr->msg_type;
    out[5] = hdr->flags;
    put_u16(out + 6, hdr->payload_len);
    put_u16(out + 8, hdr->boot_id);
    put_u32(out + 10, hdr->seq);
    put_u64(out + 14, hdr->timestamp_us);
    if (hdr->payload_len)
        memcpy(out + AERO_HEADER_SIZE, payload, hdr->payload_len);

    size_t body = AERO_HEADER_SIZE + hdr->payload_len;
    put_u32(out + body, aero_crc32_update(0, out + 2, body - 2));
    return total;
}

/* ---- streaming decoder ---- */

void aero_parser_init(aero_parser_t *p)
{
    memset(p, 0, sizeof(*p));
}

/* Drop buf[0] and everything up to the next possible sync byte. */
static void resync(aero_parser_t *p)
{
    size_t k = 1;
    while (k < p->pos && p->buf[k] != AERO_SYNC0)
        k++;
    p->bytes_discarded += (uint32_t)k;
    memmove(p->buf, p->buf + k, p->pos - k);
    p->pos -= k;
}

static void parse_header(const uint8_t *b, aero_header_t *h)
{
    h->version = b[2];
    h->source_id = b[3];
    h->msg_type = b[4];
    h->flags = b[5];
    h->payload_len = get_u16(b + 6);
    h->boot_id = get_u16(b + 8);
    h->seq = get_u32(b + 10);
    h->timestamp_us = get_u64(b + 14);
}

static void process(aero_parser_t *p, aero_frame_cb cb, void *ctx)
{
    for (;;) {
        if (p->pos >= 1 && p->buf[0] != AERO_SYNC0) {
            resync(p);
            continue;
        }
        if (p->pos >= 2 && p->buf[1] != AERO_SYNC1) {
            resync(p);
            continue;
        }
        if (p->pos < AERO_HEADER_SIZE)
            return;

        aero_header_t h;
        parse_header(p->buf, &h);
        if (h.version != AERO_PROTO_VERSION || h.payload_len > AERO_MAX_PAYLOAD) {
            p->header_errors++;
            resync(p);
            continue;
        }

        size_t body = AERO_HEADER_SIZE + h.payload_len;
        size_t total = body + AERO_CRC_SIZE;
        if (p->pos < total)
            return;

        if (aero_crc32_update(0, p->buf + 2, body - 2) != get_u32(p->buf + body)) {
            p->crc_errors++;
            resync(p);
            continue;
        }

        p->frames_ok++;
        if (cb)
            cb(ctx, &h, p->buf + AERO_HEADER_SIZE);
        memmove(p->buf, p->buf + total, p->pos - total);
        p->pos -= total;
    }
}

void aero_parser_feed(aero_parser_t *p, const uint8_t *data, size_t len,
                      aero_frame_cb cb, void *ctx)
{
    for (size_t i = 0; i < len; i++) {
        /* process() keeps pos < AERO_MAX_FRAME, so there is always room. */
        p->buf[p->pos++] = data[i];
        process(p, cb, ctx);
    }
}

/* ---- sequence tracking ---- */

void aero_seq_init(aero_seq_tracker_t *t)
{
    memset(t, 0, sizeof(*t));
}

aero_seq_result_t aero_seq_update(aero_seq_tracker_t *t, uint16_t boot_id,
                                  uint32_t seq, uint32_t *gap_first,
                                  uint32_t *gap_count)
{
    if (!t->started || t->boot_id != boot_id) {
        t->started = true;
        t->boot_id = boot_id;
        t->expected = seq + 1;
        t->received = 1;
        t->lost = 0;
        return AERO_SEQ_NEW_SESSION;
    }

    int32_t d = (int32_t)(seq - t->expected); /* wrap-safe */
    if (d < 0)
        return AERO_SEQ_LATE;

    t->received++;
    t->expected = seq + 1;
    if (d == 0)
        return AERO_SEQ_IN_ORDER;

    t->lost += (uint32_t)d;
    if (gap_first)
        *gap_first = seq - (uint32_t)d;
    if (gap_count)
        *gap_count = (uint32_t)d;
    return AERO_SEQ_GAP;
}

/* ---- NACK ---- */

size_t aero_nack_encode(uint8_t *out, size_t out_cap,
                        const aero_nack_range_t *ranges, size_t n)
{
    size_t need = n * AERO_NACK_RANGE_SIZE;
    if (out_cap < need)
        return 0;
    for (size_t i = 0; i < n; i++) {
        uint8_t *o = out + i * AERO_NACK_RANGE_SIZE;
        o[0] = ranges[i].source_id;
        o[1] = 0;
        put_u16(o + 2, ranges[i].boot_id);
        put_u32(o + 4, ranges[i].first_seq);
        put_u16(o + 8, ranges[i].count);
    }
    return need;
}

size_t aero_nack_decode(const uint8_t *payload, size_t payload_len,
                        aero_nack_range_t *ranges, size_t max)
{
    size_t n = payload_len / AERO_NACK_RANGE_SIZE;
    if (n > max)
        n = max;
    for (size_t i = 0; i < n; i++) {
        const uint8_t *s = payload + i * AERO_NACK_RANGE_SIZE;
        ranges[i].source_id = s[0];
        ranges[i].boot_id = get_u16(s + 2);
        ranges[i].first_seq = get_u32(s + 4);
        ranges[i].count = get_u16(s + 8);
    }
    return n;
}
