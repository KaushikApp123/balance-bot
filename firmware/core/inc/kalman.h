/*
 * kalman.h - Two-state Kalman filter for pitch estimation.
 *
 * State x = [angle, gyro_bias]. The gyro rate drives the prediction step and
 * the accelerometer angle is the measurement. Units are radians and rad/s.
 *
 * The robot and the sim share the defaults below. r = 1e-3 rad^2 is about
 * 1.8 deg of accel noise; with q_angle it puts the accel correction time
 * constant near half a second, so short pushes ride on the gyro.
 */
#ifndef KALMAN_H
#define KALMAN_H

#define KF_DEFAULT_Q_ANGLE  3e-6f   /* rad^2 per s       */
#define KF_DEFAULT_Q_BIAS   1e-6f   /* (rad/s)^2 per s   */
#define KF_DEFAULT_R        1e-3f   /* rad^2             */

typedef struct {
    float angle;      /* estimated pitch (rad)                    */
    float bias;       /* estimated gyro bias (rad/s)              */
    float rate;       /* bias-corrected rate (rad/s)              */
    float P[2][2];    /* error covariance                         */
    float q_angle;    /* process noise on angle  (rad^2 per s)    */
    float q_bias;     /* process noise on bias   ((rad/s)^2 per s) */
    float r_measure;  /* accel angle noise       (rad^2)          */
    float last_gain;  /* K[0] from the last update, for telemetry  */
    unsigned gated;   /* samples where the innovation gate kicked in */
} kalman_t;

void  kalman_init(kalman_t *k, float q_angle, float q_bias, float r_measure);
void  kalman_reset(kalman_t *k, float angle, float bias);

/* One predict + correct step.
 * r_scale multiplies R for this sample (>= 1). Use it to trust the
 * accelerometer less while the robot is accelerating hard. */
float kalman_update(kalman_t *k, float acc_angle, float gyro_rate, float dt, float r_scale);

/* Pitch from accelerometer, given the forward and up components (any unit). */
float accel_pitch(float a_fwd, float a_up);

/* How much to inflate R based on |a| in g. Returns 1 near 1 g, up to 100. */
float accel_r_scale(float acc_norm_g);

/* Full sensor-task step used by both the firmware and the SIL sim:
 * accel (g, body frame: forward, side, up) + pitch gyro (rad/s) -> angle. */
float kalman_imu_step(kalman_t *k, float a_fwd, float a_side, float a_up,
                      float gyro_pitch, float dt);

#endif /* KALMAN_H */
