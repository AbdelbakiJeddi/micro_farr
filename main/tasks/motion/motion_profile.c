#include "motion_profile.h"

#include <math.h>

float profile_next(float v_prev, float remaining, float dt, const profile_limits_t *lim)
{
    float v_up    = v_prev + lim->accel * dt;
    float v_brake = sqrtf(2.0f * lim->decel * fmaxf(remaining, 0.0f));

    return fmaxf(fminf(fminf(v_up, lim->v_max), v_brake), lim->v_min);
}
