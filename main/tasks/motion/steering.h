#pragma once

/* Straight-line steering: IMU heading hold + ToF wall-centering -> body yaw-rate reference,
 * and the conversion of a yaw rate into the left/right wheel speed difference. */

#include "app_types.h"

typedef struct {
    float heading_ref;  /* absolute heading to hold (deg), carried across moves */
    float lat_err;      /* last lateral error (mm), + = too close to the right wall */
} steer_t;

float steer_heading_err(const steer_t *s, float yaw);                 /* wrapped to +-180 */
float steer_rate_ref(steer_t *s, const sensor_snapshot_t *snap);      /* deg/s, clamped */
float steer_wheel_half_diff(float rate_dps);                          /* ticks/s */
