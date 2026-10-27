#include "protocol.h"
#include <string.h>

enum { S_SYNC0, S_SYNC1, S_TYPE, S_LEN, S_PAYLOAD, S_CRC0, S_CRC1 };

uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t crc)
{
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

size_t proto_encode(uint8_t type, const void *payload, uint8_t len,
                    uint8_t *out, size_t out_cap)
{
    if (len > PROTO_MAX_PAYLOAD || out_cap < (size_t)len + PROTO_OVERHEAD) return 0;
    out[0] = PROTO_SYNC0;
    out[1] = PROTO_SYNC1;
    out[2] = type;
    out[3] = len;
    if (len) memcpy(&out[4], payload, len);
    uint16_t crc = crc16_ccitt(&out[2], (size_t)len + 2u, 0xFFFFu);
    out[4 + len] = (uint8_t)(crc & 0xFFu);
    out[5 + len] = (uint8_t)(crc >> 8);
    return (size_t)len + PROTO_OVERHEAD;
}

void proto_parser_init(proto_parser_t *p)
{
    memset(p, 0, sizeof(*p));
    p->state = S_SYNC0;
}

int proto_feed(proto_parser_t *p, uint8_t byte)
{
    switch (p->state) {
    case S_SYNC0:
        if (byte == PROTO_SYNC0) p->state = S_SYNC1;
        break;
    case S_SYNC1:
        if (byte == PROTO_SYNC1)      p->state = S_TYPE;
        else if (byte != PROTO_SYNC0) p->state = S_SYNC0;
        break;
    case S_TYPE:
        p->type = byte;
        p->state = S_LEN;
        break;
    case S_LEN:
        if (byte > PROTO_MAX_PAYLOAD) { p->state = S_SYNC0; break; }
        p->len = byte;
        p->idx = 0;
        p->state = byte ? S_PAYLOAD : S_CRC0;
        break;
    case S_PAYLOAD:
        p->buf[p->idx++] = byte;
        if (p->idx >= p->len) p->state = S_CRC0;
        break;
    case S_CRC0:
        p->crc_rx = byte;
        p->state = S_CRC1;
        break;
    case S_CRC1: {
        p->crc_rx |= (uint16_t)byte << 8;
        p->state = S_SYNC0;
        uint8_t hdr[2] = { p->type, p->len };
        uint16_t crc = crc16_ccitt(hdr, 2, 0xFFFFu);
        crc = crc16_ccitt(p->buf, p->len, crc);
        if (crc == p->crc_rx) { p->frames++; return 1; }
        p->crc_errors++;
        break;
    }
    default:
        p->state = S_SYNC0;
        break;
    }
    return 0;
}
