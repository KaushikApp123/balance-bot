/*
 * task_safety.c - Highest priority. Owns the robot state machine.
 *
 *   BOOT -> DISARMED --arm request, upright, no faults--> ARMED
 *   ARMED --any fault--> FAULT (latched, motors asleep)
 *   ARMED/FAULT --disarm request--> DISARMED
 *
 * Faults:
 *   TILT    |angle| > 40 deg (it has fallen; spinning the wheels only hurts)
 *   IMU     no fresh sample for 20 ms
 *   BATT    pack under 9.9 V for 1 s (3.3 V/cell, protects the LiPo)
 *   DRIVER  a DRV8874 pulled nFAULT low (overcurrent, overtemp, UVLO)
 *   TASK    the motor, sensor or balance task stopped running
 *
 * The watchdog is only kicked when every control task has checked in, so a
 * hung task resets the MCU even if this task keeps running.
 */
#include <math.h>

#include "board.h"
#include "motor.h"
#include "shared.h"
#include "tasks.h"

#define VBATT_LOW_COUNT   (SAFETY_RATE_HZ)          /* 1 s below cutoff */
#define VBATT_ALPHA       0.05f                     /* ~0.2 s IIR       */

static float read_vbatt(void)
{
    if (HAL_ADC_Start(&hadc1) != HAL_OK) return g_vbatt;
    float v = g_vbatt;
    if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) {
        v = (float)HAL_ADC_GetValue(&hadc1) * VBATT_SCALE;
    }
    HAL_ADC_Stop(&hadc1);
    return v;
}

static void motors_off(void)
{
    motor_stop();
    motor_enable(false);
}

void task_safety(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000u / SAFETY_RATE_HZ);
    uint32_t hb_s = hb_sensor, hb_b = hb_balance, hb_m = hb_motor;
    TickType_t hb_last_ok = last;
    uint32_t low_batt = 0;

    g_vbatt = read_vbatt();
    state_set(STATE_DISARMED, 0);

    for (;;) {
        vTaskDelayUntil(&last, period);
        const TickType_t now = xTaskGetTickCount();

        /* ---- gather inputs ---- */
        attitude_t att;
        xQueuePeek(mb_attitude, &att, 0);
        g_vbatt += VBATT_ALPHA * (read_vbatt() - g_vbatt);

        uint8_t active = 0;
        if (fabsf(att.angle) > DEG2RAD(TILT_CUTOFF_DEG)) active |= FAULT_TILT;
        if ((now - att.tick) > pdMS_TO_TICKS(IMU_TIMEOUT_MS)) active |= FAULT_IMU;
        low_batt = (g_vbatt < VBATT_CUTOFF_V) ? low_batt + 1u : 0u;
        if (low_batt >= VBATT_LOW_COUNT) active |= FAULT_BATT;
        if (motor_driver_fault() && state_get() == STATE_ARMED) active |= FAULT_DRIVER;

        /* All three control tasks must have advanced since the last check. */
        const bool alive = (hb_sensor != hb_s) && (hb_balance != hb_b) && (hb_motor != hb_m);
        hb_s = hb_sensor; hb_b = hb_balance; hb_m = hb_motor;
        if (alive) hb_last_ok = now;
        if ((now - hb_last_ok) > pdMS_TO_TICKS(TASK_HEARTBEAT_MS)) active |= FAULT_TASK;
        else HAL_IWDG_Refresh(&hiwdg);

        /* ---- state machine ---- */
        const robot_state_t st = state_get();
        const arm_req_t req = arm_request_take();

        switch (st) {
        case STATE_ARMED:
            if (active) {
                motors_off();
                state_set(STATE_FAULT, active);
            } else if (req == ARM_REQ_DISARM) {
                motors_off();
                state_set(STATE_DISARMED, 0);
            } else {
                state_set(STATE_ARMED, 0);
            }
            break;

        case STATE_FAULT:
            motors_off();
            if (req == ARM_REQ_DISARM) state_set(STATE_DISARMED, active);
            else state_set(STATE_FAULT, faults_get() | active);
            break;

        case STATE_DISARMED:
        case STATE_BOOT:
        default:
            motors_off();
            if (req == ARM_REQ_ARM && !active &&
                fabsf(att.angle) < DEG2RAD(TILT_ARM_DEG)) {
                /* Balance/motor tasks reset their loops on the edge into ARMED. */
                motor_enable(true);
                state_set(STATE_ARMED, 0);
            } else {
                state_set(STATE_DISARMED, active);   /* shown, not latched */
            }
            break;
        }
    }
}
