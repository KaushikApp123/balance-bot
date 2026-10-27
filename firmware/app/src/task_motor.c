/*
 * task_motor.c - 1 kHz inner loop: encoders -> wheel speed -> PI -> PWM.
 */
#include "board.h"
#include "motor.h"
#include "shared.h"
#include "speed_est.h"
#include "tasks.h"

void task_motor(void *arg)
{
    (void)arg;
    const float dt = 1.0f / (float)MOTOR_RATE_HZ;
    const TickType_t period = pdMS_TO_TICKS(1000u / MOTOR_RATE_HZ);
    TickType_t last = xTaskGetTickCount();

    speed_est_t sl, sr;
    speed_est_init(&sl, encoder_left_raw());
    speed_est_init(&sr, encoder_right_raw());
    ctrl_inner_t il, ir;
    ctrl_inner_reset(&il);
    ctrl_inner_reset(&ir);
    robot_state_t prev = STATE_BOOT;
    ctrl_gains_t gains;
    gains_get(&gains);
    uint32_t n = 0;

    for (;;) {
        vTaskDelayUntil(&last, period);
        const uint32_t t0 = board_cycles();

        wheel_state_t w;
        w.v_left  = speed_est_update(&sl, encoder_left_raw(), dt);
        w.v_right = speed_est_update(&sr, encoder_right_raw(), dt);
        w.dist_m  = 0.5f * (speed_est_distance_m(&sl) + speed_est_distance_m(&sr));

        /* Refresh the gains copy at 100 Hz; the mutex is cheap but not free. */
        if (n++ % 10u == 0u) gains_get(&gains);

        const robot_state_t st = state_get();
        if (st == STATE_ARMED) {
            if (prev != STATE_ARMED) {
                ctrl_inner_reset(&il);
                ctrl_inner_reset(&ir);
            }
            balance_out_t b;
            xQueuePeek(mb_balance, &b, 0);
            float sp_l, sp_r;
            ctrl_differential(b.v_sp, b.yaw_rate, &sp_l, &sp_r);
            w.duty_left  = ctrl_inner_step(&il, &gains, sp_l, w.v_left,  g_vbatt, dt);
            w.duty_right = ctrl_inner_step(&ir, &gains, sp_r, w.v_right, g_vbatt, dt);
            motor_set(w.duty_left, w.duty_right);
        } else {
            w.duty_left = w.duty_right = 0.0f;
            motor_stop();
        }
        prev = st;
        xQueueOverwrite(mb_wheels, &w);
        hb_motor++;

        const uint32_t us = (board_cycles() - t0) / (SystemCoreClock / 1000000u);
        if (us > g_stats.motor_exec_max_us) g_stats.motor_exec_max_us = us;
    }
}
