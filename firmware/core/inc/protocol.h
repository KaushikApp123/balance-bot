/*
 * protocol.h - Binary framing between the robot and the dashboard.
 *
 * Frame: AA 55 | type | len | payload[len] | crc16 (LE)
 * CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type, len and payload.
 * All multi-byte fields are little-endian, floats are IEEE-754 single.
 *
 * The ESP32-C3 bridge forwards bytes without parsing, so this file and
 * host/balancebot/protocol.py are the only two places the format lives.
 */
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define PROTO_SYNC0        0xAAu
#define PROTO_SYNC1        0x55u
#define PROTO_MAX_PAYLOAD  64u
#define PROTO_OVERHEAD     6u
#define PROTO_MAX_FRAME    (PROTO_MAX_PAYLOAD + PROTO_OVERHEAD)

enum {
    /* robot -> host */
    MSG_TELEMETRY  = 0x01,
    MSG_GAINS      = 0x02,
    MSG_ACK        = 0x03,
    /* host -> robot */
    MSG_SET_GAINS  = 0x10,
    MSG_GET_GAINS  = 0x11,
    MSG_SAVE_GAINS = 0x12,
    MSG_ARM        = 0x13,
    MSG_DRIVE      = 0x14,
    MSG_SET_TRIM   = 0x15,
};

enum { ACK_OK = 0, ACK_BAD_ARG = 1, ACK_BUSY = 2, ACK_REFUSED = 3, ACK_FAILED = 4 };

typedef enum { STATE_BOOT = 0, STATE_DISARMED = 1, STATE_ARMED = 2, STATE_FAULT = 3 } robot_state_t;

enum {
    FAULT_TILT   = 1u << 0,
    FAULT_IMU    = 1u << 1,
    FAULT_BATT   = 1u << 2,
    FAULT_DRIVER = 1u << 3,
    FAULT_TASK   = 1u << 4,
};

#pragma pack(push, 1)
typedef struct {
    uint32_t t_ms;
    float angle;        /* rad   */
    float rate;         /* rad/s */
    float angle_sp;     /* rad   */
    float p, i, d;      /* angle loop terms, m/s^2 */
    float v_sp;         /* m/s */
    float v_left, v_right;
    float duty_left, duty_right;
    float vbatt;
    uint8_t state;
    uint8_t faults;
} msg_telemetry_t;

typedef struct {
    float kp[3], ki[3], kd[3];  /* indexed by loop_id_t */
    float trim;
} msg_gains_t;

typedef struct { uint8_t loop; float kp, ki, kd; } msg_set_gains_t;
typedef struct { uint8_t msg; uint8_t status; } msg_ack_t;
typedef struct { uint8_t arm; } msg_arm_t;
typedef struct { float v_ref; float yaw_rate; } msg_drive_t;
typedef struct { float trim; } msg_set_trim_t;
#pragma pack(pop)

_Static_assert(sizeof(msg_telemetry_t) == 54, "telemetry layout");
_Static_assert(sizeof(msg_gains_t) == 40, "gains layout");
_Static_assert(sizeof(msg_set_gains_t) == 13, "set_gains layout");
_Static_assert(sizeof(msg_telemetry_t) <= PROTO_MAX_PAYLOAD, "payload too big");

uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t crc);

/* Returns bytes written, or 0 if out is too small / len too big. */
size_t proto_encode(uint8_t type, const void *payload, uint8_t len,
                    uint8_t *out, size_t out_cap);

typedef struct {
    uint8_t state;
    uint8_t type;
    uint8_t len;
    uint8_t idx;
    uint16_t crc_rx;
    uint8_t buf[PROTO_MAX_PAYLOAD];
    uint32_t crc_errors;
    uint32_t frames;
} proto_parser_t;

void proto_parser_init(proto_parser_t *p);
/* Feed one byte. Returns 1 when a valid frame is ready in p->type/len/buf. */
int  proto_feed(proto_parser_t *p, uint8_t byte);

#endif /* PROTOCOL_H */
