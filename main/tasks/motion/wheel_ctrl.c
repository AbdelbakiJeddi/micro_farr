#include "wheel_ctrl.h"

#include <math.h>
#include <stdbool.h>

#include "app_config.h"
#include "params.h"

static float clamp_abs(float value, float limit)
{
    return fminf(fmaxf(value, -limit), limit);
}

void wheel_reset(wheel_ctrl_t *w, int32_t ticks, uint32_t now_ms, float dir)
{
    w->ref_ticks = ticks;
    w->ref_ms    = now_ms;
    w->v         = 0.0f;
    w->i_term    = dir >= 0.0f ? RUN_START_DUTY : -RUN_START_DUTY;
}

/* Wheel speed in ticks/s, measured over windows of at least SPEED_WINDOW_MS and filtered. */
static float speed_update(wheel_ctrl_t *w, int32_t ticks, uint32_t now_ms)
{
    uint32_t dt_ms = now_ms - w->ref_ms;

    if (dt_ms >= SPEED_WINDOW_MS) {
        float inst = (float)(ticks - w->ref_ticks) * 1000.0f / (float)dt_ms;

        w->v        += SPEED_FILTER_ALPHA * (inst - w->v);
        w->ref_ticks = ticks;
        w->ref_ms    = now_ms;
    }
    return w->v;
}

/* Feed-forward + PI. The integrator is frozen while the output is saturated and the error
 * would push it further (anti-windup). */
float wheel_duty(wheel_ctrl_t *w, float v_ref, int32_t ticks, uint32_t now_ms, float dt)
{
    float err      = v_ref - speed_update(w, ticks, now_ms);
    float max_duty = params_get(P_RUN_MAX_DUTY);
    float ff_p     = params_get(P_RUN_KFF) * v_ref + params_get(P_RUN_KP) * err;
    float out      = ff_p + w->i_term;
    bool  windup   = (out >= max_duty && err > 0.0f) || (out <= -max_duty && err < 0.0f);

    if (!windup) {
        w->i_term = clamp_abs(w->i_term + params_get(P_RUN_KI) * err * dt, RUN_I_MAX_DUTY);
        out       = ff_p + w->i_term;
    }
    return clamp_abs(out, max_duty);
}
