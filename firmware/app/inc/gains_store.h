/*
 * gains_store.h - Persist tuned gains in flash sector 1 (16 KB at 0x08004000).
 */
#ifndef GAINS_STORE_H
#define GAINS_STORE_H

#include <stdbool.h>
#include "controller.h"

bool gains_store_load(ctrl_gains_t *out);         /* false if empty/corrupt */
bool gains_store_save(const ctrl_gains_t *in);    /* only call while disarmed */

#endif /* GAINS_STORE_H */
