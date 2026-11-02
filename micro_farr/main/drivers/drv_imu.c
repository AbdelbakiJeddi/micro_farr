#include "drv_imu.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "drv_imu";

esp_err_t drv_imu_init(void)
{
    /* TODO: configure MPU6050 over I2C (FSR, DLPF, sample rate). */
    ESP_LOGI(TAG, "init (stub)");
    return ESP_OK;
}

esp_err_t drv_imu_read(imu_sample_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* TODO: burst-read accel + gyro over I2C. Stub: zeros. */
    memset(out, 0, sizeof(*out));
    out->timestamp_us = (uint32_t)esp_timer_get_time();
    return ESP_OK;
}
