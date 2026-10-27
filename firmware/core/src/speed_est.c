#include "speed_est.h"

#define M_PER_COUNT  ((2.0f * 3.14159265f * WHEEL_RADIUS_M) / ENC_CPR)

void speed_est_init(speed_est_t *s, uint16_t raw)
{
    s->position = 0;
    s->last_raw = raw;
    s->idx = 0;
    s->filled = 0;
    s->speed_mps = 0.0f;
    for (unsigned i = 0; i < SPEED_WINDOW; i++) s->hist[i] = 0;
}

float speed_est_update(speed_est_t *s, uint16_t raw, float dt)
{
    /* Signed 16-bit difference handles counter wrap in both directions. */
    int16_t delta = (int16_t)(raw - s->last_raw);
    s->last_raw = raw;
    s->position += delta;

    /* hist[idx] holds the position from SPEED_WINDOW samples ago. */
    int32_t oldest = s->hist[s->idx];
    s->hist[s->idx] = s->position;
    s->idx = (s->idx + 1u) % SPEED_WINDOW;

    if (s->filled < SPEED_WINDOW) {
        s->filled++;
        s->speed_mps = (float)delta * M_PER_COUNT / dt;
    } else {
        s->speed_mps = (float)(s->position - oldest) * M_PER_COUNT /
                       (dt * (float)SPEED_WINDOW);
    }
    return s->speed_mps;
}

float speed_est_distance_m(const speed_est_t *s)
{
    return (float)s->position * M_PER_COUNT;
}
