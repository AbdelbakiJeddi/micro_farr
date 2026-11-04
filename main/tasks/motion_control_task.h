#pragma once

#include "esp_err.h"

void      motion_control_task_start(void);
esp_err_t straight_cells(uint8_t cells);
esp_err_t straight_one_cell(void);
esp_err_t turn_left(void);   /* 90 deg counter-clockwise */
esp_err_t turn_right(void);  /* 90 deg clockwise */
