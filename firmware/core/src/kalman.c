#include "kalman.h"
#include <math.h>

#define KF_GATE_SIGMA2  9.0f   /* 3 sigma */

void kalman_init(kalman_t *k, float q_angle, float q_bias, float r_measure)
{
    k->q_angle   = q_angle;
    k->q_bias    = q_bias;
    k->r_measure = r_measure;
    kalman_reset(k, 0.0f, 0.0f);
}

void kalman_reset(kalman_t *k, float angle, float bias)
{
    k->angle = angle;
    k->bias  = bias;
    k->rate  = 0.0f;
    /* Start fairly unsure about the angle, a bit unsure about the bias. */
    k->P[0][0] = 0.1f;  k->P[0][1] = 0.0f;
    k->P[1][0] = 0.0f;  k->P[1][1] = 0.01f;
    k->last_gain = 0.0f;
    k->gated = 0;
}

float kalman_update(kalman_t *k, float acc_angle, float gyro_rate, float dt, float r_scale)
{
    /* ---- Predict: integrate the bias-corrected gyro rate ---- */
    k->rate   = gyro_rate - k->bias;
    k->angle += dt * k->rate;

    /* P = F P F' + Q, with F = [1 -dt; 0 1] */
    k->P[0][0] += dt * (dt * k->P[1][1] - k->P[0][1] - k->P[1][0] + k->q_angle);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += k->q_bias * dt;

    /* ---- Correct with the accelerometer angle (H = [1 0]) ---- */
    if (r_scale < 1.0f) r_scale = 1.0f;
    const float y = acc_angle - k->angle;
    float S = k->P[0][0] + k->r_measure * r_scale;

    /* Innovation gate: an accel angle more than 3 sigma away from the
     * prediction is most likely linear acceleration, not tilt. Instead of
     * throwing it away, inflate R just enough to put it at the gate. This
     * bounds how far one bad sample can pull the estimate. */
    if (y * y > KF_GATE_SIGMA2 * S) {
        S = y * y / KF_GATE_SIGMA2;
        k->gated++;
    }
    const float K0 = k->P[0][0] / S;
    const float K1 = k->P[1][0] / S;

    k->angle += K0 * y;
    k->bias  += K1 * y;

    const float P00 = k->P[0][0];
    const float P01 = k->P[0][1];
    k->P[0][0] -= K0 * P00;
    k->P[0][1] -= K0 * P01;
    k->P[1][0] -= K1 * P00;
    k->P[1][1] -= K1 * P01;

    k->last_gain = K0;
    return k->angle;
}

float accel_pitch(float a_fwd, float a_up)
{
    /* Leaning forward tips the up axis toward the ground in front, so the
     * forward axis reads -g*sin(theta). See docs/design.md for the frame. */
    return atan2f(-a_fwd, a_up);
}

float accel_r_scale(float acc_norm_g)
{
    /* Within 5% of 1 g the accel is mostly gravity: trust it normally.
     * Beyond that it is seeing linear acceleration or bumps, so back off
     * quadratically, capped at 100x. */
    float e = fabsf(acc_norm_g - 1.0f);
    if (e < 0.05f) return 1.0f;
    float s = 1.0f + 4000.0f * (e - 0.05f) * (e - 0.05f); /* 0.1 g -> 11x, 0.2 g -> 91x */
    return s > 100.0f ? 100.0f : s;
}

float kalman_imu_step(kalman_t *k, float a_fwd, float a_side, float a_up,
                      float gyro_pitch, float dt)
{
    const float norm = sqrtf(a_fwd * a_fwd + a_side * a_side + a_up * a_up);
    return kalman_update(k, accel_pitch(a_fwd, a_up), gyro_pitch, dt,
                         accel_r_scale(norm));
}
