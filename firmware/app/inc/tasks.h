/*
 * tasks.h - The six FreeRTOS tasks, highest priority first.
 *
 *  Task       Prio  Rate      Trigger                Job
 *  ---------  ----  --------  ---------------------  -------------------------------------
 *  safety      6    100 Hz    vTaskDelayUntil        tilt/IMU/battery/driver/heartbeat checks,
 *                                                    owns the state machine and the watchdog
 *  motor       5    1 kHz     vTaskDelayUntil        encoders -> speed, inner speed PI -> PWM
 *  sensor      4    500 Hz    notify from DMA ISR    Kalman filter -> attitude mailbox
 *  balance     3    200 Hz    vTaskDelayUntil        angle + velocity loops -> speed setpoint
 *  telemetry   2    50 Hz     vTaskDelayUntil        frames to BLE bridge, commands back
 *  logger      1    10 Hz     vTaskDelayUntil        debug console, battery-free stats, LED
 *
 * Motor runs above sensor because rate-monotonic ordering puts the fastest
 * periodic loop first, and its work is tiny (~10 us). Safety sits above all
 * of them because it must be able to cut the motors no matter what.
 */
#ifndef TASKS_H
#define TASKS_H

#include <stdint.h>

void task_safety(void *arg);
void task_motor(void *arg);
void task_sensor(void *arg);
void task_balance(void *arg);
void task_telemetry(void *arg);
void task_logger(void *arg);

/* Set before the scheduler starts, read by the sensor task. */
extern float g_boot_gyro_bias;
extern float g_boot_angle;

#endif /* TASKS_H */
