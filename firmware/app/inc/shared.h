/*
 * shared.h - Data passed between tasks.
 *
 * Rule of thumb used here:
 *   - Latest-value data (attitude, wheel speeds, setpoints) goes through
 *     length-1 queues used as mailboxes: xQueueOverwrite to publish,
 *     xQueuePeek to read. Readers never block and always get a whole struct.
 *   - The gains are written rarely and read every cycle by two tasks, so they
 *     sit behind a mutex. A FreeRTOS mutex has priority inheritance, so the
 *     low-priority telemetry task holding it cannot be preempted by mid
 *     priority work while the motor task waits (no unbounded inversion).
 *   - The robot state machine is owned by the safety task. Everyone else only
 *     reads it or posts a request.
 */
#ifndef SHARED_H
#define SHARED_H

#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "bb_config.h"
#include "controller.h"
#include "protocol.h"

/* Task priorities, highest first. */
enum {
    PRIO_LOGGER    = 1,
    PRIO_TELEMETRY = 2,
    PRIO_BALANCE   = 3,
    PRIO_SENSOR    = 4,
    PRIO_MOTOR     = 5,
    PRIO_SAFETY    = 6,
};

typedef struct {
    float angle;        /* rad */
    float rate;         /* rad/s, bias corrected */
    float bias;         /* rad/s */
    float acc_angle;    /* raw accel angle, for telemetry/debug */
    uint32_t tick;      /* xTaskGetTickCount() at the sample */
} attitude_t;

typedef struct {
    float v_left, v_right;       /* m/s at the wheel */
    float duty_left, duty_right;
    float dist_m;                /* average distance since boot */
} wheel_state_t;

typedef struct {
    float v_sp;         /* common wheel speed setpoint, m/s */
    float yaw_rate;     /* rad/s */
    float angle_sp;     /* rad */
    float p, i, d;      /* angle loop terms (m/s^2) */
} balance_out_t;

typedef struct {
    float v_ref;        /* m/s */
    float yaw_rate;     /* rad/s */
    uint32_t tick;      /* when it arrived; stale commands are dropped */
} drive_cmd_t;

typedef struct {
    volatile uint32_t imu_samples;
    volatile uint32_t imu_overruns;   /* data-ready while previous read in flight */
    volatile uint32_t i2c_errors;
    volatile uint32_t i2c_recoveries;
    volatile uint32_t imu_timeouts;
    volatile uint32_t uart_rx_bytes;
    volatile uint32_t cmd_frames;
    volatile uint32_t balance_jitter_max_us;
    volatile uint32_t motor_exec_max_us;
    volatile uint32_t kf_gated;
} stats_t;

/* Mailboxes */
extern QueueHandle_t mb_attitude;
extern QueueHandle_t mb_wheels;
extern QueueHandle_t mb_balance;
extern QueueHandle_t mb_drive;

/* Task handles (needed for direct-to-task notifications from ISRs) */
extern TaskHandle_t h_safety, h_sensor, h_balance, h_motor, h_telemetry, h_logger;

/* Heartbeats, bumped by each control task every cycle */
extern volatile uint32_t hb_sensor, hb_balance, hb_motor;

extern stats_t g_stats;
extern volatile float g_vbatt;

/* initial = saved gains, or NULL for the defaults */
void shared_init(const ctrl_gains_t *initial);

/* Gains (mutex protected) */
void gains_get(ctrl_gains_t *out);
void gains_set(const ctrl_gains_t *in);
bool gains_set_loop(int loop, float kp, float ki, float kd);
void gains_set_trim(float trim);

/* State machine (owned by the safety task) */
robot_state_t state_get(void);
uint8_t       faults_get(void);
void          state_set(robot_state_t s, uint8_t faults);   /* safety task only */

typedef enum { ARM_REQ_NONE = 0, ARM_REQ_ARM, ARM_REQ_DISARM } arm_req_t;
void      arm_request(arm_req_t r);
arm_req_t arm_request_take(void);                           /* safety task only */

static inline TickType_t ms_to_ticks(uint32_t ms) { return pdMS_TO_TICKS(ms); }

#endif /* SHARED_H */
