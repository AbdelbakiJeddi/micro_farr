#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t oled_init(void);
bool      oled_ready(void);
void      oled_clear(void);
void      oled_fill(bool on);
void      oled_text_line(uint8_t row, const char *text);
void      oled_flush(void);
