#pragma once

#include "esp_err.h"

#include "app_types.h"

/* Owned by supervisor_task only. */

esp_err_t drv_oled_init(void);
void drv_oled_show(const system_state_t *state);
