/*
 * sil.c - Flat C API over the control core for the Python software-in-the-loop
 * sim (host/balancebot/core.py). Only built into the host shared library.
 * The sim calls these in the same order and at the same rates as the
 * firmware tasks, so what balances in the sim is the exact code on the robot.
 */
#include <stdlib.h>
#include "bb_config.h"
#include "kalman.h"
#include "controller.h"
#include "speed_est.h"

#if defined(_WIN32)
#define SIL_API __declspec(dllexport)
#else
#define SIL_API __attribute__((visibility("default")))
#endif

/* Same Kalman tuning as the firmware (app/src/task_sensor.c). */
#define KF_Q_ANGLE  KF_DEFAULT_Q_ANGLE
#define KF_Q_BIAS   KF_DEFAULT_Q_BIAS
#define KF_R        KF_DEFAULT_R

typedef struct {
    kalman_t kf;
    ctrl_gains_t gains;
    ctrl_outer_t outer;
    ctrl_inner_t inner[2];
    speed_est_t speed[2];
} sil_t;

SIL_API void *sil_create(void)
{
    sil_t *s = (sil_t *)calloc(1, sizeof(sil_t));
    if (!s) return NULL;
    kalman_init(&s->kf, KF_Q_ANGLE, KF_Q_BIAS, KF_R);
    ctrl_default_gains(&s->gains);
    ctrl_outer_reset(&s->outer);
    ctrl_inner_reset(&s->inner[0]);
    ctrl_inner_reset(&s->inner[1]);
    return s;
}

SIL_API void sil_destroy(void *h) { free(h); }

SIL_API void sil_set_gains(void *h, int loop, float kp, float ki, float kd)
{
    sil_t *s = (sil_t *)h;
    if (loop < 0 || loop >= LOOP_COUNT) return;
    s->gains.g[loop].kp = kp;
    s->gains.g[loop].ki = ki;
    s->gains.g[loop].kd = kd;
}

SIL_API void sil_get_gains(void *h, int loop, float *out3)
{
    sil_t *s = (sil_t *)h;
    if (loop < 0 || loop >= LOOP_COUNT) return;
    out3[0] = s->gains.g[loop].kp;
    out3[1] = s->gains.g[loop].ki;
    out3[2] = s->gains.g[loop].kd;
}

SIL_API void sil_set_kalman(void *h, float q_angle, float q_bias, float r)
{
    sil_t *s = (sil_t *)h;
    s->kf.q_angle = q_angle;
    s->kf.q_bias = q_bias;
    s->kf.r_measure = r;
}

SIL_API void  sil_set_trim(void *h, float trim) { ((sil_t *)h)->gains.angle_trim = trim; }
SIL_API float sil_get_trim(void *h) { return ((sil_t *)h)->gains.angle_trim; }

SIL_API void sil_reset(void *h, float angle, float bias)
{
    sil_t *s = (sil_t *)h;
    kalman_reset(&s->kf, angle, bias);
    ctrl_outer_reset(&s->outer);
    ctrl_inner_reset(&s->inner[0]);
    ctrl_inner_reset(&s->inner[1]);
    speed_est_init(&s->speed[0], 0);
    speed_est_init(&s->speed[1], 0);
}

/* Reset only the controllers (what arming does on the robot). */
SIL_API void sil_reset_ctrl(void *h)
{
    sil_t *s = (sil_t *)h;
    ctrl_outer_reset(&s->outer);
    ctrl_inner_reset(&s->inner[0]);
    ctrl_inner_reset(&s->inner[1]);
}

/* Motor task speed estimate from a raw 16-bit encoder count. */
SIL_API float sil_speed(void *h, int wheel, unsigned raw, float dt)
{
    sil_t *s = (sil_t *)h;
    return speed_est_update(&s->speed[wheel ? 1 : 0], (uint16_t)raw, dt);
}

/* Sensor task. Accel in g (forward, side, up), gyro in rad/s. */
SIL_API float sil_imu(void *h, float a_fwd, float a_side, float a_up, float gyro, float dt)
{
    return kalman_imu_step(&((sil_t *)h)->kf, a_fwd, a_side, a_up, gyro, dt);
}

/* Balance task. Returns the common wheel speed setpoint. */
SIL_API float sil_outer(void *h, float v_meas, float v_ref, float dt)
{
    sil_t *s = (sil_t *)h;
    return ctrl_outer_step(&s->outer, &s->gains, s->kf.angle, s->kf.rate, v_meas, v_ref, dt);
}

/* Motor task, one wheel. Returns duty. */
SIL_API float sil_inner(void *h, int wheel, float v_sp, float v_meas, float vbatt, float dt)
{
    sil_t *s = (sil_t *)h;
    return ctrl_inner_step(&s->inner[wheel ? 1 : 0], &s->gains, v_sp, v_meas, vbatt, dt);
}

/* angle, rate, bias, lean_sp, v_sp, p, i, d */
SIL_API void sil_state(void *h, float *out8)
{
    sil_t *s = (sil_t *)h;
    out8[0] = s->kf.angle;
    out8[1] = s->kf.rate;
    out8[2] = s->kf.bias;
    out8[3] = s->outer.lean_sp + s->gains.angle_trim;
    out8[4] = s->outer.v_sp;
    out8[5] = -s->outer.angle.p_term;
    out8[6] = -s->outer.angle.i_term;
    out8[7] = -s->outer.angle.d_term;
}
