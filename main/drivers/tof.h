#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    TOF_LEFT,
    TOF_FRONT,
    TOF_RIGHT,
    TOF_COUNT,
} tof_id_t;

typedef struct {
    uint16_t distance_mm;
    uint8_t  range_status;
    float    signal_rate_mcps;
    float    ambient_rate_mcps;
    bool     valid;
    uint32_t timestamp_ms;
} tof_reading_t;

esp_err_t tof_init_all(void);
void      tof_service(void);
bool      tof_get_reading(tof_id_t id, tof_reading_t *out);
