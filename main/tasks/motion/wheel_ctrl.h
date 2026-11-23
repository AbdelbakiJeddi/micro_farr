#pragma once

/* One wheel's speed loop: speed estimate from encoder ticks + feed-forward/PI to duty. */

#include <stdint.h>

typedef struct {
    int32_t  ref_ticks;  /* wheel ticks at the last speed sample */
    uint32_t ref_ms;
    float    v;          /* filtered wheel speed (ticks/s) */
    float    i_term;     /* duty % */
} wheel_ctrl_t;

/* dir = +1 forward / -1 backward: sign of the static-friction pre-load of the integrator. */
void  wheel_reset(wheel_ctrl_t *w, int32_t ticks, uint32_t now_ms, float dir);

/* Duty % in [-P_RUN_MAX_DUTY, P_RUN_MAX_DUTY]; negative brakes actively. */
float wheel_duty(wheel_ctrl_t *w, float v_ref, int32_t ticks, uint32_t now_ms, float dt);
