/*
 * bb_config.h - Build-time constants shared by the firmware and the host
 * simulation. Everything here is plain C so the control core can be unit
 * tested and run in the software-in-the-loop sim on a PC.
 */
#ifndef BB_CONFIG_H
#define BB_CONFIG_H

/* ---------------- Loop rates ---------------- */
#define IMU_RATE_HZ          500u   /* MPU-6050 data-ready rate              */
#define BALANCE_RATE_HZ      200u   /* outer angle loop                      */
#define MOTOR_RATE_HZ        1000u  /* inner wheel-speed loop (5x outer)     */
#define VELOCITY_DIVIDER     4u     /* velocity loop = 200 / 4 = 50 Hz       */
#define SAFETY_RATE_HZ       100u
#define TELEMETRY_RATE_HZ    50u
#define LOGGER_RATE_HZ       10u

/* ---------------- Mechanics ---------------- */
#define WHEEL_RADIUS_M       0.0325f   /* 65 mm wheels                       */
#define WHEEL_TRACK_M        0.170f    /* distance between wheel centers     */
#define ENC_CPR              1320.0f   /* 11 PPR x 4 (quadrature) x 30:1 gear */
#define SPEED_WINDOW         8u        /* samples in the speed estimator (8 ms) */

/* Motor model used for feedforward: JGB37-520, 12 V, 330 rpm no-load.
 * 330 rpm = 34.6 rad/s at 12 V -> 2.88 rad/s per volt -> 0.0937 m/s per volt. */
#define WHEEL_MPS_PER_VOLT   0.0937f
#define V_SP_MAX_MPS         0.9f      /* clamp on the speed setpoint        */

/* ---------------- Safety ---------------- */
#define TILT_CUTOFF_DEG      40.0f  /* motors cut past this                   */
#define TILT_ARM_DEG         5.0f   /* must be this close to upright to arm   */
#define IMU_TIMEOUT_MS       20u    /* 10 missed samples at 500 Hz            */
#define TASK_HEARTBEAT_MS    50u
#define VBATT_CUTOFF_V       9.9f   /* 3S LiPo, 3.3 V per cell                */
#define VBATT_NOMINAL_V      11.1f

/* ---------------- Limits ---------------- */
#define ANGLE_SP_LIMIT_RAD   0.17f  /* velocity loop may ask for at most ~10 deg lean */
#define DUTY_LIMIT           0.95f

#define DEG2RAD(x) ((x) * 0.017453292519943295f)
#define RAD2DEG(x) ((x) * 57.29577951308232f)

#endif /* BB_CONFIG_H */
