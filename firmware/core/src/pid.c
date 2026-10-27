#include "pid.h"

static float clampf(float x, float lim)
{
    if (x >  lim) return  lim;
    if (x < -lim) return -lim;
    return x;
}

void pid_reset(pid_state_t *s)
{
    s->integ = 0.0f;
    s->prev_meas = 0.0f;
    s->d_filt = 0.0f;
    s->primed = 0;
    s->p_term = s->i_term = s->d_term = s->out = 0.0f;
}

float pid_step(pid_state_t *s, const pid_gains_t *g, const pid_limits_t *lim,
               float setpoint, float meas, float meas_rate, int use_rate, float dt)
{
    const float err = setpoint - meas;

    /* P */
    const float p = g->kp * err;

    /* D on measurement */
    float rate;
    if (use_rate) {
        rate = meas_rate;
    } else {
        rate = (s->primed && dt > 0.0f) ? (meas - s->prev_meas) / dt : 0.0f;
    }
    s->prev_meas = meas;
    s->primed = 1;
    s->d_filt += lim->d_alpha * (rate - s->d_filt);
    const float d = -g->kd * s->d_filt;

    /* I with conditional integration: skip the update if the output is
     * already saturated and this error would push it further. */
    float integ_next = clampf(s->integ + g->ki * err * dt, lim->i_limit);
    const float unsat = p + integ_next + d;
    const int saturating = (unsat > lim->out_limit && err > 0.0f) ||
                           (unsat < -lim->out_limit && err < 0.0f);
    if (!saturating) {
        s->integ = integ_next;
    }

    const float out = clampf(p + s->integ + d, lim->out_limit);

    s->p_term = p;
    s->i_term = s->integ;
    s->d_term = d;
    s->out = out;
    return out;
}
