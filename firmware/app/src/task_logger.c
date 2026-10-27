/*
 * task_logger.c - Lowest priority housekeeping: status LED and a once-a-second
 * line on the debug UART (USART2, 115200) with health counters.
 */
#include <stdio.h>

#include "board.h"
#include "shared.h"
#include "tasks.h"

static void print(const char *s, int n)
{
    if (n > 0) HAL_UART_Transmit(&huart2, (const uint8_t *)s, (uint16_t)n, 50);
}

static unsigned hwm(TaskHandle_t t)
{
    return t ? (unsigned)uxTaskGetStackHighWaterMark(t) : 0u;
}

void task_logger(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(1000u / LOGGER_RATE_HZ);
    TickType_t last = xTaskGetTickCount();
    uint32_t n = 0;
    char line[200];

    int len = snprintf(line, sizeof line, "\r\nbalance-bot up, %lu Hz core\r\n",
                       (unsigned long)SystemCoreClock);
    print(line, len);

    for (;;) {
        vTaskDelayUntil(&last, period);
        n++;

        /* LED: slow blink disarmed, solid armed, fast blink fault */
        switch (state_get()) {
        case STATE_ARMED:    board_led(1); break;
        case STATE_FAULT:    board_led((int)(n & 1u)); break;
        default:             board_led((n % 10u) < 1u); break;
        }

        if (n % LOGGER_RATE_HZ != 0u) continue;

        attitude_t a;
        xQueuePeek(mb_attitude, &a, 0);
        len = snprintf(line, sizeof line,
            "st=%d f=0x%02x ang=%.2f bias=%.3f vb=%.2f imu=%lu ovr=%lu i2cerr=%lu rec=%lu "
            "gate=%lu jit=%luus mot=%luus rx=%lu stk[s%u m%u i%u b%u t%u l%u] heap=%u\r\n",
            (int)state_get(), (unsigned)faults_get(),
            (double)RAD2DEG(a.angle), (double)RAD2DEG(a.bias), (double)g_vbatt,
            (unsigned long)g_stats.imu_samples, (unsigned long)g_stats.imu_overruns,
            (unsigned long)g_stats.i2c_errors, (unsigned long)g_stats.i2c_recoveries,
            (unsigned long)g_stats.kf_gated, (unsigned long)g_stats.balance_jitter_max_us,
            (unsigned long)g_stats.motor_exec_max_us, (unsigned long)g_stats.uart_rx_bytes,
            hwm(h_safety), hwm(h_motor), hwm(h_sensor), hwm(h_balance), hwm(h_telemetry),
            hwm(h_logger), (unsigned)xPortGetMinimumEverFreeHeapSize());
        print(line, len);
    }
}
