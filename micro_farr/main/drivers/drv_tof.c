#include "drv_tof.h"

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "drv_tof";

esp_err_t drv_tof_init(void)
{
    /* TODO: hold all XSHUT low, release one at a time, assign unique addresses. */
    ESP_LOGI(TAG, "init (stub)");
    return ESP_OK;
}

esp_err_t drv_tof_read(tof_frame_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* TODO: poll/read the three VL53L0X sensors. Stub: fixed distances. */
    out->left_mm = 300;
    out->center_mm = 400;
    out->right_mm = 300;
    out->timestamp_us = (uint32_t)esp_timer_get_time();
    out->valid = true;
    return ESP_OK;
}
