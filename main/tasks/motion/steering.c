#include "steering.h"

#include <math.h>
#include <stdbool.h>

#include "app_config.h"
#include "params.h"

#define DEG_TO_RAD 0.017453293f

static float wrap180(float deg)
{
    while (deg > 180.0f) {
        deg -= 360.0f;
    }
    while (deg < -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

static bool side_is_wall(int32_t tof_mm)
{
    return tof_mm >= 0 && tof_mm < TOF_WALL_MAX_MM;
}

/* Lateral error in mm: positive = too close to the right wall (steer left). */
static float lateral_error(const sensor_snapshot_t *snap)
{
    bool left  = side_is_wall(snap->left_tof);
    bool right = side_is_wall(snap->right_tof);

    if (left && right) {
        return 0.5f * (float)(snap->left_tof - snap->right_tof);
    }
    if (left) {
        return (float)(snap->left_tof - TOF_SIDE_TARGET_MM);
    }
    if (right) {
        return (float)(TOF_SIDE_TARGET_MM - snap->right_tof);
    }
    return 0.0f; /* no side walls: drive straight */
}

float steer_heading_err(const steer_t *s, float yaw)
{
    return wrap180(s->heading_ref - yaw);
}

/* Heading hold (YAW_KP) and wall-centering (TOF_CENTER_KP) summed, clamped together. */
float steer_rate_ref(steer_t *s, const sensor_snapshot_t *snap)
{
    float limit = params_get(P_YAW_MAX_RATE_DPS);
    float rate;

    s->lat_err = lateral_error(snap);
    rate = params_get(P_YAW_KP) * steer_heading_err(s, snap->yaw) +
           params_get(P_TOF_CENTER_KP) * s->lat_err;
    return fminf(fmaxf(rate, -limit), limit);
}

/* Half the left/right wheel speed difference (ticks/s) that produces the yaw rate. */
float steer_wheel_half_diff(float rate_dps)
{
    return (float)TURN_YAW_SIGN * rate_dps * DEG_TO_RAD * (0.5f * WHEEL_BASE_MM) *
           params_ticks_per_mm();
}
