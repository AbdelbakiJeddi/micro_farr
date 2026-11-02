#pragma once

#include "esp_err.h"

#include "app_types.h"

/* Owned by sensor_task only. */

esp_err_t drv_tof_init(void);
esp_err_t drv_tof_read(tof_frame_t *out);
