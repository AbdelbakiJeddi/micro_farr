#pragma once

/* Trapezoid speed profile, unit agnostic (ticks or degrees). */

typedef struct {
    float v_max;   /* cruise */
    float accel;
    float decel;
    float v_min;   /* creep floor so a move always finishes */
} profile_limits_t;

/* Next speed: accelerate toward v_max, brake so the speed reaches v_min at remaining = 0. */
float profile_next(float v_prev, float remaining, float dt, const profile_limits_t *lim);
