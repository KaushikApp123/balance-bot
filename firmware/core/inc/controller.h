/*
 * controller.h - The cascaded balance controller.
 *
 *   velocity loop (50 Hz)  : v_ref - v_meas      -> lean setpoint (rad)
 *   angle loop    (200 Hz) : angle - lean_sp     -> wheel acceleration (m/s^2),
 *                                                   integrated into a speed setpoint
 *   speed loop    (1 kHz)  : v_sp - v_wheel      -> motor voltage -> PWM duty
 *
 * The angle loop outputs acceleration, not speed, on purpose. With a fast
 * inner speed loop the wheels move at whatever speed we command, and for an
 * inverted pendulum it is the base *acceleration* that rights the body. If the
 * angle PD output went straight to speed, the stiffness term would vanish and
 * the robot would fall slowly. See docs/design.md section 3.
 */
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "pid.h"

typedef enum {
    LOOP_ANGLE = 0,
    LOOP_SPEED = 1,
    LOOP_VELOCITY = 2,
    LOOP_COUNT
} loop_id_t;

typedef struct {
    pid_gains_t g[LOOP_COUNT];
    float angle_trim;   /* rad, offset for a COM that is not over the axle */
} ctrl_gains_t;

void ctrl_default_gains(ctrl_gains_t *g);

typedef struct {
    pid_state_t angle;
    pid_state_t velocity;
    float lean_sp;      /* rad, from the velocity loop       */
    float v_sp;         /* m/s, integrated speed setpoint    */
    float accel_cmd;    /* m/s^2, angle loop output          */
    unsigned div;
} ctrl_outer_t;

typedef struct {
    pid_state_t pid;
    float volts;
    float duty;
} ctrl_inner_t;

void  ctrl_outer_reset(ctrl_outer_t *c);
void  ctrl_inner_reset(ctrl_inner_t *c);

/* Runs at BALANCE_RATE_HZ. Returns the common wheel speed setpoint (m/s). */
float ctrl_outer_step(ctrl_outer_t *c, const ctrl_gains_t *g,
                      float angle, float rate, float v_meas, float v_ref, float dt);

/* Runs at MOTOR_RATE_HZ for each wheel. Returns PWM duty in [-1, 1]. */
float ctrl_inner_step(ctrl_inner_t *c, const ctrl_gains_t *g,
                      float v_sp, float v_meas, float vbatt, float dt);

/* Split the common setpoint into left/right for a yaw-rate command (rad/s). */
void  ctrl_differential(float v_sp, float yaw_rate, float *left, float *right);

#endif /* CONTROLLER_H */
