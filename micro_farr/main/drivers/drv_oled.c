#include "drv_oled.h"

#include "esp_log.h"

static const char *TAG = "drv_oled";

static const char *state_name(sys_state_t state)
{
    switch (state) {
    case SYS_BOOT:      return "BOOT";
    case SYS_CALIBRATE: return "CALIBRATE";
    case SYS_IDLE:      return "IDLE";
    case SYS_EXPLORE:   return "EXPLORE";
    case SYS_GOAL:      return "GOAL";
    case SYS_STOP:      return "STOP";
    case SYS_FAULT:     return "FAULT";
    default:            return "?";
    }
}

esp_err_t drv_oled_init(void)
{
    /* TODO: configure SSD1306 over I2C. */
    ESP_LOGI(TAG, "init (stub)");
    return ESP_OK;
}

void drv_oled_show(const system_state_t *state)
{
    if (state == NULL) {
        return;
    }

    /* TODO: render on the SSD1306. Stub: log the state. */
    ESP_LOGI(TAG, "state=%s cell=(%u,%u) heading=%.1f x=%.0f y=%.0f fault=%d",
             state_name(state->state),
             (unsigned)state->pose.cell_x, (unsigned)state->pose.cell_y,
             state->pose.heading_deg,
             state->pose.x_mm, state->pose.y_mm,
             (int)state->fault);
}
