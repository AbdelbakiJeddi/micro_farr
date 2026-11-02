#include "drv_encoder.h"

#include "esp_log.h"

static const char *TAG = "drv_encoder";

static int32_t s_left;
static int32_t s_right;

esp_err_t drv_encoder_init(void)
{
    /* TODO: configure PCNT units for both quadrature encoders. */
    ESP_LOGI(TAG, "init (stub)");
    return ESP_OK;
}

void drv_encoder_read(int32_t *left, int32_t *right)
{
    /* Stub: advance slowly so odometry has something to integrate. */
    s_left += 1;
    s_right += 1;

    if (left != NULL) {
        *left = s_left;
    }
    if (right != NULL) {
        *right = s_right;
    }
}
