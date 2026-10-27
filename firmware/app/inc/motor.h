/*
 * motor.h - Two DRV8874 H-bridges in PH/EN mode plus quadrature encoders.
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>

/* Set to 1 if a motor spins backward for positive duty (they face opposite
 * ways on the chassis, so one of them normally needs flipping). */
#define MOTOR_LEFT_INVERT    0
#define MOTOR_RIGHT_INVERT   1
#define ENC_LEFT_INVERT      0
#define ENC_RIGHT_INVERT     1

void     motor_enable(bool on);          /* nSLEEP; off = outputs high-Z (coast) */
void     motor_set(float left, float right);   /* duty in [-1, 1] */
void     motor_stop(void);
bool     motor_driver_fault(void);       /* nFAULT low on either driver */
uint16_t encoder_left_raw(void);
uint16_t encoder_right_raw(void);

#endif /* MOTOR_H */
