/*
 * speed_est.h - Wheel speed from encoder counts.
 *
 * At 1 kHz a wheel at full speed only moves about 7 counts per sample, so a
 * one-sample difference is very coarse. We difference across a sliding window
 * of SPEED_WINDOW samples instead: resolution becomes 1 count per 8 ms
 * (about 2 cm/s) at the cost of ~4 ms of lag.
 */
#ifndef SPEED_EST_H
#define SPEED_EST_H

#include <stdint.h>
#include "bb_config.h"

typedef struct {
    int32_t hist[SPEED_WINDOW];
    int32_t position;     /* accumulated counts */
    uint16_t last_raw;
    unsigned idx;
    unsigned filled;
    float speed_mps;
} speed_est_t;

void  speed_est_init(speed_est_t *s, uint16_t raw);
/* raw = free-running 16-bit timer count. Handles wrap. */
float speed_est_update(speed_est_t *s, uint16_t raw, float dt);
float speed_est_distance_m(const speed_est_t *s);

#endif /* SPEED_EST_H */
