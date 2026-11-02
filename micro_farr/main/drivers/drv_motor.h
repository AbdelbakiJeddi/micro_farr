#pragma once

#include "esp_err.h"

/* Owned by motion_control_task only. */

esp_err_t drv_motor_init(void);

/* Normalised command per wheel, -1.0 .. +1.0. */
void drv_motor_set(float left, float right);

void drv_motor_stop(void);
