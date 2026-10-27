/*
 * task_balance.c - 200 Hz outer loops: velocity -> lean angle -> speed setpoint.
 *
 * vTaskDelayUntil (not vTaskDelay) keeps the period fixed: it wakes relative
 * to the last wake time, so the time spent computing does not stretch the
 * period. The PID dt is a constant, which is only true if the period is.
 */
#include "board.h"
#include "shared.h"
#include "tasks.h"

#define DRIVE_TIMEOUT_MS  500u   /* stop if the dashboard stops sending */

void task_balance(void *arg)
{
    (void)arg;
    const float dt = 1.0f / (float)BALANCE_RATE_HZ;
    const TickType_t period = pdMS_TO_TICKS(1000u / BALANCE_RATE_HZ);
    TickType_t last = xTaskGetTickCount();
    uint32_t prev_cycles = board_cycles();
    const uint32_t cyc_per_us = SystemCoreClock / 1000000u;
    const uint32_t nominal_us = 1000000u / BALANCE_RATE_HZ;

    ctrl_outer_t ctrl;
    ctrl_outer_reset(&ctrl);
    robot_state_t prev = STATE_BOOT;

    for (;;) {
        vTaskDelayUntil(&last, period);

        /* Jitter: how far this wake-up landed from the nominal period. */
        const uint32_t now_c = board_cycles();
        const uint32_t us = (now_c - prev_cycles) / cyc_per_us;
        prev_cycles = now_c;
        const uint32_t jit = us > nominal_us ? us - nominal_us : nominal_us - us;
        if (prev == STATE_ARMED && jit > g_stats.balance_jitter_max_us) g_stats.balance_jitter_max_us = jit;

        attitude_t att;
        wheel_state_t w;
        drive_cmd_t cmd;
        xQueuePeek(mb_attitude, &att, 0);
        xQueuePeek(mb_wheels, &w, 0);
        xQueuePeek(mb_drive, &cmd, 0);
        if ((xTaskGetTickCount() - cmd.tick) > pdMS_TO_TICKS(DRIVE_TIMEOUT_MS)) {
            cmd.v_ref = 0.0f;
            cmd.yaw_rate = 0.0f;
        }

        ctrl_gains_t gains;
        gains_get(&gains);

        balance_out_t out = {0};
        const robot_state_t st = state_get();
        if (st == STATE_ARMED) {
            if (prev != STATE_ARMED) ctrl_outer_reset(&ctrl);
            const float v_avg = 0.5f * (w.v_left + w.v_right);
            out.v_sp = ctrl_outer_step(&ctrl, &gains, att.angle, att.rate, v_avg, cmd.v_ref, dt);
            out.yaw_rate = cmd.yaw_rate;
        } else {
            ctrl_outer_reset(&ctrl);
        }
        out.angle_sp = ctrl.lean_sp + gains.angle_trim;
        out.p = -ctrl.angle.p_term;
        out.i = -ctrl.angle.i_term;
        out.d = -ctrl.angle.d_term;
        prev = st;

        xQueueOverwrite(mb_balance, &out);
        hb_balance++;
    }
}
