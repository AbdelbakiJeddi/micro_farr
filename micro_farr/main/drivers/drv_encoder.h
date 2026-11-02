#pragma once

#include <stdint.h>

#include "esp_err.h"

/* Owned by motion_control_task only. */

esp_err_t drv_encoder_init(void);

/* Absolute counts since init. */
void drv_encoder_read(int32_t *left, int32_t *right);
