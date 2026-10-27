/*
 * test_core.c - Host unit tests for the portable control core.
 * Build: cmake -S firmware -B build-host && cmake --build build-host && ctest --test-dir build-host
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bb_config.h"
#include "controller.h"
#include "kalman.h"
#include "pid.h"
#include "protocol.h"
#include "speed_est.h"

static int g_fail = 0, g_pass = 0;
#define CHECK(cond) do { if (cond) g_pass++; else { g_fail++; \
    printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define NEAR(a, b, tol) CHECK(fabs((double)(a) - (double)(b)) <= (double)(tol))

/* Small deterministic RNG so test runs are repeatable. */
static unsigned rng = 12345u;
static float frand(void) { rng = rng * 1103515245u + 12345u; return (float)((rng >> 8) & 0xFFFF) / 65535.0f; }
static float gauss(void) { float s = 0; for (int i = 0; i < 12; i++) s += frand(); return s - 6.0f; }

static void test_crc(void)
{
    /* Standard check value for CRC-16/CCITT-FALSE */
    const char *s = "123456789";
    CHECK(crc16_ccitt((const uint8_t *)s, 9, 0xFFFF) == 0x29B1);
}

static void test_protocol_roundtrip(void)
{
    msg_set_gains_t m = { .loop = 2, .kp = 1.5f, .ki = -0.25f, .kd = 3.0f };
    uint8_t frame[PROTO_MAX_FRAME];
    size_t n = proto_encode(MSG_SET_GAINS, &m, sizeof m, frame, sizeof frame);
    CHECK(n == sizeof m + PROTO_OVERHEAD);

    proto_parser_t p;
    proto_parser_init(&p);
    /* garbage in front, including a lone sync byte */
    const uint8_t junk[] = { 0x00, 0xAA, 0x13, 0x55, 0xFF };
    int got = 0;
    for (size_t i = 0; i < sizeof junk; i++) got += proto_feed(&p, junk[i]);
    for (size_t i = 0; i < n; i++) got += proto_feed(&p, frame[i]);
    CHECK(got == 1);
    CHECK(p.type == MSG_SET_GAINS && p.len == sizeof m);
    msg_set_gains_t r;
    memcpy(&r, p.buf, sizeof r);
    CHECK(r.loop == 2 && r.kp == 1.5f && r.ki == -0.25f && r.kd == 3.0f);

    /* flip a payload bit: must be rejected */
    frame[6] ^= 0x01;
    got = 0;
    for (size_t i = 0; i < n; i++) got += proto_feed(&p, frame[i]);
    CHECK(got == 0 && p.crc_errors == 1);

    /* zero-length frame */
    n = proto_encode(MSG_GET_GAINS, NULL, 0, frame, sizeof frame);
    got = 0;
    for (size_t i = 0; i < n; i++) got += proto_feed(&p, frame[i]);
    CHECK(got == 1 && p.type == MSG_GET_GAINS && p.len == 0);

    /* oversized length must not overflow */
    CHECK(proto_encode(MSG_TELEMETRY, frame, PROTO_MAX_PAYLOAD + 1, frame, sizeof frame) == 0);
}

static void test_pid_windup(void)
{
    pid_gains_t g = { .kp = 1.0f, .ki = 10.0f, .kd = 0.0f };
    pid_limits_t lim = { .i_limit = 5.0f, .out_limit = 2.0f, .d_alpha = 1.0f };
    pid_state_t s;
    pid_reset(&s);
    /* Hold a big error for 10 s: output saturates and the integrator must
     * stop growing instead of winding up to the clamp. */
    for (int i = 0; i < 1000; i++) pid_step(&s, &g, &lim, 10.0f, 0.0f, 0, 0, 0.01f);
    CHECK(s.out == 2.0f);
    CHECK(s.integ < 2.0f);
    /* When the error reverses, the output should leave saturation right away. */
    float out = pid_step(&s, &g, &lim, -1.0f, 0.0f, 0, 0, 0.01f);
    CHECK(out < 2.0f);
}

static void test_pid_ki_change_no_bump(void)
{
    pid_gains_t g = { .kp = 0.0f, .ki = 2.0f, .kd = 0.0f };
    pid_limits_t lim = { .i_limit = 100.0f, .out_limit = 100.0f, .d_alpha = 1.0f };
    pid_state_t s;
    pid_reset(&s);
    for (int i = 0; i < 100; i++) pid_step(&s, &g, &lim, 1.0f, 0.0f, 0, 0, 0.01f);
    float before = s.out;
    g.ki = 20.0f;                                  /* live retune */
    float after = pid_step(&s, &g, &lim, 0.0f, 0.0f, 0, 0, 0.01f);  /* zero error */
    NEAR(after, before, 1e-5);
}

static void test_pid_derivative_on_measurement(void)
{
    pid_gains_t g = { .kp = 0.0f, .ki = 0.0f, .kd = 1.0f };
    pid_limits_t lim = { .i_limit = 1.0f, .out_limit = 100.0f, .d_alpha = 1.0f };
    pid_state_t s;
    pid_reset(&s);
    pid_step(&s, &g, &lim, 0.0f, 0.0f, 0, 0, 0.01f);
    float out = pid_step(&s, &g, &lim, 50.0f, 0.0f, 0, 0, 0.01f); /* setpoint jump */
    NEAR(out, 0.0f, 1e-6);                                        /* no derivative kick */
}

static void test_kalman_bias_and_angle(void)
{
    kalman_t k;
    kalman_init(&k, KF_DEFAULT_Q_ANGLE, KF_DEFAULT_Q_BIAS, KF_DEFAULT_R);
    const float dt = 1.0f / IMU_RATE_HZ;
    const float true_bias = DEG2RAD(2.0f);          /* 2 deg/s gyro bias */
    float max_err_late = 0.0f;
    for (int i = 0; i < 10 * (int)IMU_RATE_HZ; i++) {
        float t = i * dt;
        float theta = DEG2RAD(10.0f) * sinf(2.0f * 3.14159f * 0.5f * t);
        float omega = DEG2RAD(10.0f) * 2.0f * 3.14159f * 0.5f * cosf(2.0f * 3.14159f * 0.5f * t);
        float gyro  = omega + true_bias + DEG2RAD(0.3f) * gauss();
        float acc   = theta + DEG2RAD(1.5f) * gauss();   /* noisy accel angle */
        kalman_update(&k, acc, gyro, dt, 1.0f);
        if (t > 5.0f) {
            float e = fabsf(k.angle - theta);
            if (e > max_err_late) max_err_late = e;
        }
    }
    printf("  kalman: bias est %.3f deg/s (true 2.000), worst late error %.3f deg\n",
           RAD2DEG(k.bias), RAD2DEG(max_err_late));
    NEAR(k.bias, true_bias, DEG2RAD(0.5f));
    CHECK(max_err_late < DEG2RAD(1.0f));
}

static void test_kalman_rejects_linear_accel(void)
{
    /* Robot held at 0 deg while the base accelerates at 0.3 g: the raw accel
     * angle reads ~17 deg. Adaptive R should keep the estimate near zero. */
    kalman_t k;
    kalman_init(&k, KF_DEFAULT_Q_ANGLE, KF_DEFAULT_Q_BIAS, KF_DEFAULT_R);
    const float dt = 1.0f / IMU_RATE_HZ;
    for (int i = 0; i < (int)IMU_RATE_HZ; i++) kalman_imu_step(&k, 0.0f, 0.0f, 1.0f, 0.0f, dt);
    for (int i = 0; i < (int)(0.2f * IMU_RATE_HZ); i++) kalman_imu_step(&k, -0.3f, 0.0f, 1.0f, 0.0f, dt);
    printf("  kalman: after 200 ms of 0.3 g push, angle %.2f deg (raw accel says %.1f)\n",
           RAD2DEG(k.angle), RAD2DEG(accel_pitch(-0.3f, 1.0f)));
    CHECK(fabsf(RAD2DEG(k.angle)) < 3.0f);
}

static void test_speed_est_wrap(void)
{
    speed_est_t s;
    speed_est_init(&s, 65530);
    uint16_t raw = 65530;
    const float dt = 1.0f / MOTOR_RATE_HZ;
    for (int i = 0; i < 50; i++) { raw = (uint16_t)(raw + 5); speed_est_update(&s, raw, dt); }
    /* 5 counts/ms = 5000 counts/s -> 5000/1320 rev/s * 2*pi*r */
    float expect = 5000.0f / ENC_CPR * 2.0f * 3.14159265f * WHEEL_RADIUS_M;
    NEAR(s.speed_mps, expect, 1e-3);
    NEAR(speed_est_distance_m(&s), 250.0f / ENC_CPR * 2.0f * 3.14159265f * WHEEL_RADIUS_M, 1e-4);
}

static void test_controller_signs(void)
{
    ctrl_gains_t g;
    ctrl_default_gains(&g);
    ctrl_outer_t o;
    ctrl_outer_reset(&o);
    /* Leaning forward must drive the wheels forward. */
    float v = 0;
    for (int i = 0; i < 10; i++) v = ctrl_outer_step(&o, &g, DEG2RAD(3.0f), 0.0f, 0.0f, 0.0f, 0.005f);
    CHECK(v > 0.0f);
    /* Falling forward fast must add to that. */
    ctrl_outer_reset(&o);
    float v_still = ctrl_outer_step(&o, &g, 0.0f, 0.0f, 0.0f, 0.0f, 0.005f);
    ctrl_outer_reset(&o);
    float v_falling = ctrl_outer_step(&o, &g, 0.0f, 1.0f, 0.0f, 0.0f, 0.005f);
    CHECK(v_falling > v_still);

    /* Inner loop: wheel too slow -> more positive duty; feedforward scales with battery. */
    ctrl_inner_t in;
    ctrl_inner_reset(&in);
    float d_full = ctrl_inner_step(&in, &g, 0.3f, 0.3f, 12.6f, 0.001f);
    ctrl_inner_reset(&in);
    float d_low = ctrl_inner_step(&in, &g, 0.3f, 0.3f, 10.0f, 0.001f);
    CHECK(d_low > d_full && d_full > 0.0f);

    float l, r;
    ctrl_differential(0.2f, 1.0f, &l, &r);
    CHECK(r > l);
}

int main(void)
{
    printf("core tests\n");
    test_crc();
    test_protocol_roundtrip();
    test_pid_windup();
    test_pid_ki_change_no_bump();
    test_pid_derivative_on_measurement();
    test_kalman_bias_and_angle();
    test_kalman_rejects_linear_accel();
    test_speed_est_wrap();
    test_controller_signs();
    printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? EXIT_FAILURE : EXIT_SUCCESS;
}
