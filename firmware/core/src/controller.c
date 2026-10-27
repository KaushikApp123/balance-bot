#include "controller.h"
#include "bb_config.h"

/* Limits per loop. Gains are tunable live, limits are not. */
static const pid_limits_t LIM_ANGLE    = { .i_limit = 2.0f,  .out_limit = 20.0f, .d_alpha = 0.5f };
static const pid_limits_t LIM_VELOCITY = { .i_limit = 0.08f, .out_limit = ANGLE_SP_LIMIT_RAD, .d_alpha = 0.3f };
static const pid_limits_t LIM_SPEED    = { .i_limit = 3.0f,  .out_limit = 12.6f, .d_alpha = 1.0f };

void ctrl_default_gains(ctrl_gains_t *g)
{
    /* Starting point from the SIL sim (host/balancebot/sim.py).
     * Retune on the real robot from the dashboard. */
    g->g[LOOP_ANGLE]    = (pid_gains_t){ .kp = 100.0f, .ki = 0.0f,   .kd = 2.0f };
    g->g[LOOP_SPEED]    = (pid_gains_t){ .kp = 8.0f,   .ki = 100.0f, .kd = 0.0f };
    g->g[LOOP_VELOCITY] = (pid_gains_t){ .kp = 0.08f,  .ki = 0.02f,  .kd = 0.0f };
    g->angle_trim = 0.0f;
}

void ctrl_outer_reset(ctrl_outer_t *c)
{
    pid_reset(&c->angle);
    pid_reset(&c->velocity);
    c->lean_sp = 0.0f;
    c->v_sp = 0.0f;
    c->accel_cmd = 0.0f;
    c->div = 0;
}

void ctrl_inner_reset(ctrl_inner_t *c)
{
    pid_reset(&c->pid);
    c->volts = 0.0f;
    c->duty = 0.0f;
}

float ctrl_outer_step(ctrl_outer_t *c, const ctrl_gains_t *g,
                      float angle, float rate, float v_meas, float v_ref, float dt)
{
    /* Velocity loop at 1/VELOCITY_DIVIDER of the balance rate. If we are
     * going faster than asked, lean back (negative lean setpoint). */
    if (c->div++ % VELOCITY_DIVIDER == 0u) {
        c->lean_sp = pid_step(&c->velocity, &g->g[LOOP_VELOCITY], &LIM_VELOCITY,
                              v_ref, v_meas, 0.0f, 0, dt * (float)VELOCITY_DIVIDER);
    }

    /* Angle loop. Error is (angle - setpoint): leaning forward must drive the
     * wheels forward, so the PID output is negated. D uses the gyro rate. */
    const float sp = c->lean_sp + g->angle_trim;
    c->accel_cmd = -pid_step(&c->angle, &g->g[LOOP_ANGLE], &LIM_ANGLE,
                             sp, angle, rate, 1, dt);

    /* Integrate acceleration into the speed setpoint and clamp it. */
    c->v_sp += c->accel_cmd * dt;
    if (c->v_sp >  V_SP_MAX_MPS) c->v_sp =  V_SP_MAX_MPS;
    if (c->v_sp < -V_SP_MAX_MPS) c->v_sp = -V_SP_MAX_MPS;
    return c->v_sp;
}

float ctrl_inner_step(ctrl_inner_t *c, const ctrl_gains_t *g,
                      float v_sp, float v_meas, float vbatt, float dt)
{
    if (vbatt < 6.0f) vbatt = 6.0f;   /* avoid divide blow-up on a bad reading */

    /* Feedforward: the voltage that would spin the wheel at v_sp with no load.
     * PI only has to fix the error, and dividing by vbatt keeps the loop gain
     * the same as the pack drains. */
    const float v_ff = v_sp / WHEEL_MPS_PER_VOLT;
    const float v_fb = pid_step(&c->pid, &g->g[LOOP_SPEED], &LIM_SPEED,
                                v_sp, v_meas, 0.0f, 0, dt);
    float volts = v_ff + v_fb;
    float duty = volts / vbatt;
    if (duty >  DUTY_LIMIT) duty =  DUTY_LIMIT;
    if (duty < -DUTY_LIMIT) duty = -DUTY_LIMIT;

    c->volts = volts;
    c->duty = duty;
    return duty;
}

void ctrl_differential(float v_sp, float yaw_rate, float *left, float *right)
{
    const float dv = yaw_rate * (WHEEL_TRACK_M * 0.5f);
    *left  = v_sp - dv;
    *right = v_sp + dv;
}
