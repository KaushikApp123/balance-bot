/*
 * pid.h - PID with the features a real loop needs:
 *   - derivative on measurement (no kick on setpoint steps)
 *   - optional external rate for D (e.g. the gyro), so we never differentiate
 *     a noisy angle
 *   - first-order low-pass on the D term
 *   - integrator stored in output units, so changing Ki live does not bump
 *     the output
 *   - anti-windup: integrator clamp + conditional integration
 */
#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;
} pid_gains_t;

typedef struct {
    float i_limit;    /* |integrator| clamp, output units */
    float out_limit;  /* |output| clamp                    */
    float d_alpha;    /* D low-pass, 0..1 (1 = no filter)  */
} pid_limits_t;

typedef struct {
    float integ;
    float prev_meas;
    float d_filt;
    int   primed;
    /* last terms, for telemetry */
    float p_term, i_term, d_term, out;
} pid_state_t;

void  pid_reset(pid_state_t *s);

/* meas_rate is used for D when use_rate != 0, else D comes from the
 * difference of measurements. */
float pid_step(pid_state_t *s, const pid_gains_t *g, const pid_limits_t *lim,
               float setpoint, float meas, float meas_rate, int use_rate, float dt);

#endif /* PID_H */
