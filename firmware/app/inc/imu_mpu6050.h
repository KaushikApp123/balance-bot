/*
 * imu_mpu6050.h - MPU-6050 over I2C1, interrupt + DMA driven.
 *
 * Flow per sample (500 Hz):
 *   MPU INT rising edge -> EXTI ISR starts a 14-byte DMA read of the
 *   accel/temp/gyro registers -> DMA complete ISR copies the bytes and wakes
 *   the sensor task with vTaskNotifyGiveFromISR. The CPU never polls the bus.
 */
#ifndef IMU_MPU6050_H
#define IMU_MPU6050_H

#include <stdbool.h>
#include <stdint.h>

#define MPU_ADDR            0x68u
#define MPU_ACCEL_LSB_PER_G 8192.0f   /* +-4 g   */
#define MPU_GYRO_LSB_PER_DPS 65.5f    /* +-500 dps */

typedef struct {
    float a_fwd, a_side, a_up;   /* g, robot frame */
    float gyro_pitch;            /* rad/s, + = tipping forward */
    float temp_c;
} imu_sample_t;

bool imu_init(void);                     /* blocking, scheduler may or may not run */
bool imu_calibrate(float *gyro_bias, float *angle0, uint32_t n);  /* robot still */
void imu_start(void);                    /* unmask data-ready interrupt */
void imu_stop(void);
bool imu_read_latest(imu_sample_t *out); /* convert the last DMA buffer */
bool imu_bus_recover(void);              /* clock out a stuck slave, re-init */

#endif /* IMU_MPU6050_H */
