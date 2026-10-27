/*
 * task_telemetry.c - 50 Hz link to the ESP32-C3 BLE bridge on USART1.
 *
 * TX: one DMA transfer per cycle carrying the telemetry frame plus any
 *     replies queued since the last cycle.
 * RX: circular DMA + UART idle-line interrupt. The ISR copies new bytes into
 *     a FreeRTOS stream buffer; this task parses them.
 *
 * This is the lowest-priority task that talks to hardware on purpose: if the
 * CPU is ever short on time, telemetry is what should slip, not control.
 */
#include <string.h>

#include "board.h"
#include "gains_store.h"
#include "protocol.h"
#include "shared.h"
#include "stream_buffer.h"
#include "tasks.h"

#define RX_DMA_SIZE   128u
#define TX_BUF_SIZE   256u

static uint8_t s_rx_dma[RX_DMA_SIZE];
static volatile uint16_t s_rx_pos;
static StreamBufferHandle_t s_rx_stream;

static uint8_t s_tx[TX_BUF_SIZE];       /* owned by DMA while a transfer runs */
static uint8_t s_reply[TX_BUF_SIZE];    /* replies waiting for the next cycle */
static size_t  s_reply_len;

static void queue_frame(uint8_t type, const void *payload, uint8_t len)
{
    size_t n = proto_encode(type, payload, len, &s_reply[s_reply_len],
                            sizeof s_reply - s_reply_len);
    s_reply_len += n;               /* n == 0 if it did not fit: dropped */
}

static void reply_ack(uint8_t msg, uint8_t status)
{
    msg_ack_t a = { .msg = msg, .status = status };
    queue_frame(MSG_ACK, &a, sizeof a);
}

static void reply_gains(void)
{
    ctrl_gains_t g;
    gains_get(&g);
    msg_gains_t m;
    for (int i = 0; i < LOOP_COUNT; i++) {
        m.kp[i] = g.g[i].kp;
        m.ki[i] = g.g[i].ki;
        m.kd[i] = g.g[i].kd;
    }
    m.trim = g.angle_trim;
    queue_frame(MSG_GAINS, &m, sizeof m);
}

static void handle(const proto_parser_t *p)
{
    g_stats.cmd_frames++;
    switch (p->type) {
    case MSG_SET_GAINS: {
        if (p->len != sizeof(msg_set_gains_t)) { reply_ack(p->type, ACK_BAD_ARG); break; }
        msg_set_gains_t m;
        memcpy(&m, p->buf, sizeof m);
        bool ok = gains_set_loop(m.loop, m.kp, m.ki, m.kd);
        reply_ack(p->type, ok ? ACK_OK : ACK_BAD_ARG);
        if (ok) reply_gains();
        break;
    }
    case MSG_GET_GAINS:
        reply_gains();
        break;
    case MSG_SAVE_GAINS: {
        if (state_get() == STATE_ARMED) { reply_ack(p->type, ACK_REFUSED); break; }
        ctrl_gains_t g;
        gains_get(&g);
        reply_ack(p->type, gains_store_save(&g) ? ACK_OK : ACK_FAILED);
        break;
    }
    case MSG_ARM: {
        if (p->len != sizeof(msg_arm_t)) { reply_ack(p->type, ACK_BAD_ARG); break; }
        arm_request(p->buf[0] ? ARM_REQ_ARM : ARM_REQ_DISARM);
        reply_ack(p->type, ACK_OK);   /* accepted; the state field shows the result */
        break;
    }
    case MSG_DRIVE: {
        if (p->len != sizeof(msg_drive_t)) break;
        msg_drive_t m;
        memcpy(&m, p->buf, sizeof m);
        drive_cmd_t d = { .v_ref = m.v_ref, .yaw_rate = m.yaw_rate, .tick = xTaskGetTickCount() };
        if (d.v_ref > 0.5f) d.v_ref = 0.5f;
        if (d.v_ref < -0.5f) d.v_ref = -0.5f;
        if (d.yaw_rate > 3.0f) d.yaw_rate = 3.0f;
        if (d.yaw_rate < -3.0f) d.yaw_rate = -3.0f;
        if (d.v_ref == d.v_ref && d.yaw_rate == d.yaw_rate) xQueueOverwrite(mb_drive, &d);
        break;
    }
    case MSG_SET_TRIM: {
        if (p->len != sizeof(msg_set_trim_t)) { reply_ack(p->type, ACK_BAD_ARG); break; }
        msg_set_trim_t m;
        memcpy(&m, p->buf, sizeof m);
        gains_set_trim(m.trim);
        reply_ack(p->type, ACK_OK);
        reply_gains();
        break;
    }
    default:
        reply_ack(p->type, ACK_BAD_ARG);
        break;
    }
}

static void build_telemetry(msg_telemetry_t *t)
{
    attitude_t a;
    wheel_state_t w;
    balance_out_t b;
    xQueuePeek(mb_attitude, &a, 0);
    xQueuePeek(mb_wheels, &w, 0);
    xQueuePeek(mb_balance, &b, 0);
    t->t_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    t->angle = a.angle;
    t->rate = a.rate;
    t->angle_sp = b.angle_sp;
    t->p = b.p;
    t->i = b.i;
    t->d = b.d;
    t->v_sp = b.v_sp;
    t->v_left = w.v_left;
    t->v_right = w.v_right;
    t->duty_left = w.duty_left;
    t->duty_right = w.duty_right;
    t->vbatt = g_vbatt;
    t->state = (uint8_t)state_get();
    t->faults = faults_get();
}

static void rx_start(void)
{
    s_rx_pos = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_rx_dma, sizeof s_rx_dma);
}

void task_telemetry(void *arg)
{
    (void)arg;
    s_rx_stream = xStreamBufferCreate(256, 1);
    configASSERT(s_rx_stream != NULL);
    rx_start();

    proto_parser_t parser;
    proto_parser_init(&parser);
    const TickType_t period = pdMS_TO_TICKS(1000u / TELEMETRY_RATE_HZ);
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last, period);

        /* Commands first so their replies ride on this cycle's transfer. */
        uint8_t chunk[32];
        size_t n;
        while ((n = xStreamBufferReceive(s_rx_stream, chunk, sizeof chunk, 0)) > 0) {
            for (size_t i = 0; i < n; i++) {
                if (proto_feed(&parser, chunk[i])) handle(&parser);
            }
        }

        if (huart1.gState != HAL_UART_STATE_READY) continue;   /* last DMA still going */

        msg_telemetry_t t;
        build_telemetry(&t);
        size_t len = proto_encode(MSG_TELEMETRY, &t, sizeof t, s_tx, sizeof s_tx);
        if (s_reply_len && len + s_reply_len <= sizeof s_tx) {
            memcpy(&s_tx[len], s_reply, s_reply_len);
            len += s_reply_len;
            s_reply_len = 0;
        }
        HAL_UART_Transmit_DMA(&huart1, s_tx, (uint16_t)len);
    }
}

/* ---------------- ISR side ---------------- */

static void push_isr(const uint8_t *p, size_t n, BaseType_t *woken)
{
    if (n == 0) return;
    xStreamBufferSendFromISR(s_rx_stream, p, n, woken);
    g_stats.uart_rx_bytes += n;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *h, uint16_t pos)
{
    if (h->Instance != USART1 || s_rx_stream == NULL) return;
    BaseType_t woken = pdFALSE;
    /* pos is how far the circular DMA has written. Copy what is new. */
    if (pos != s_rx_pos) {
        if (pos > s_rx_pos) {
            push_isr(&s_rx_dma[s_rx_pos], pos - s_rx_pos, &woken);
        } else {
            push_isr(&s_rx_dma[s_rx_pos], RX_DMA_SIZE - s_rx_pos, &woken);
            push_isr(&s_rx_dma[0], pos, &woken);
        }
        s_rx_pos = pos;
    }
    if (s_rx_pos >= RX_DMA_SIZE) s_rx_pos = 0;
    portYIELD_FROM_ISR(woken);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *h)
{
    /* Noise or overrun aborts the receive. Re-arm it; the CRC catches the rest. */
    if (h->Instance == USART1) rx_start();
}
