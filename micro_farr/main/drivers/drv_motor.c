#include "drv_motor.h"

#include "esp_log.h"

static const char *TAG = "drv_motor";

esp_err_t drv_motor_init(void)
{
    /* TODO: configure TB6612FNG direction pins and LEDC PWM channels. */
    ESP_LOGI(TAG, "init (stub)");
    return ESP_OK;
}

void drv_motor_set(float left, float right)
{
    /* TODO: write direction pins and LEDC duty. Stub: ignore outputs. */
    (void)left;
    (void)right;
}

void drv_motor_stop(void)
{
    /* TODO: set duty to zero and disable the driver if required. */
}
