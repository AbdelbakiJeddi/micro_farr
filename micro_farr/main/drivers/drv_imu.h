#pragma once

#include "esp_err.h"

#include "app_types.h"

/* Owned by sensor_task only. */

esp_err_t drv_imu_init(void);
esp_err_t drv_imu_read(imu_sample_t *out);
