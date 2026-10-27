/*
 * task_sensor.c - Wakes on every IMU sample (500 Hz) and runs the Kalman filter.
 *
 * The task blocks in ulTaskNotifyTake. The I2C DMA-complete ISR wakes it with
 * vTaskNotifyGiveFromISR: no semaphore or queue object, and the notification
 * count also tells us if we ever fell behind.
 */
#include "board.h"
#include "imu_mpu6050.h"
#include "kalman.h"
#include "shared.h"
#include "tasks.h"

float g_boot_gyro_bias;
float g_boot_angle;

void task_sensor(void *arg)
{
    (void)arg;
    kalman_t kf;
    kalman_init(&kf, KF_DEFAULT_Q_ANGLE, KF_DEFAULT_Q_BIAS, KF_DEFAULT_R);
    kalman_reset(&kf, g_boot_angle, g_boot_gyro_bias);

    const float dt = 1.0f / (float)IMU_RATE_HZ;
    imu_start();

    for (;;) {
        const uint32_t pending = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(IMU_TIMEOUT_MS));
        if (pending == 0u) {
            /* No sample in 20 ms. Safety will already be flagging FAULT_IMU;
             * try to get the bus back. */
            g_stats.imu_timeouts++;
            imu_bus_recover();
            hb_sensor++;      /* the task is alive; missing data is FAULT_IMU's job */
            continue;
        }
        if (pending > 1u) g_stats.imu_overruns += pending - 1u;

        imu_sample_t s;
        if (!imu_read_latest(&s)) continue;

        attitude_t a;
        a.angle = kalman_imu_step(&kf, s.a_fwd, s.a_side, s.a_up, s.gyro_pitch, dt);
        a.rate = kf.rate;
        a.bias = kf.bias;
        a.acc_angle = accel_pitch(s.a_fwd, s.a_up);
        a.tick = xTaskGetTickCount();
        xQueueOverwrite(mb_attitude, &a);

        g_stats.imu_samples++;
        g_stats.kf_gated = kf.gated;
        hb_sensor++;
    }
}
