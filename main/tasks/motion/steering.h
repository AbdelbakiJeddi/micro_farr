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

/* Both side walls seen: the heading reference drifts toward the measured yaw at
 * P_REALIGN_RATE (1/s), removing gyro drift and the steady lateral offset. */
void    steer_realign(steer_t *s, const sensor_snapshot_t *snap, float dt);

/* Remaining distance of a straight: the front ToF distance to the stop point
 * (front - P_FRONT_ALIGN_MM) when it agrees with the encoder within P_FRONT_ALIGN_WIN_MM,
 * else the encoder value. Readings outside the window cannot stop the robot early. */
int32_t steer_front_remaining(int32_t enc_remaining_ticks, int32_t front_tof_mm);
