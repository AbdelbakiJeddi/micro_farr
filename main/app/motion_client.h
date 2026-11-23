#pragma once

/* App-side wrappers around the motion command queue and status mailbox. */

#include <stdbool.h>

#include "freertos/FreeRTOS.h"

#include "esp_err.h"

#include "app_types.h"

esp_err_t motion_straight(int cells, speed_profile_t profile);
esp_err_t motion_extend(int cells);                             /* add cells to the run in progress */
esp_err_t motion_turn(int angle_deg, speed_profile_t profile);  /* +left, -right */
esp_err_t motion_stop(void);
bool      motion_wait(TickType_t timeout, move_report_t *out);  /* true = move ended normally */
bool      motion_status(motion_status_t *out);
