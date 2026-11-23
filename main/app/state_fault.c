/* FAULT: motors stopped, reason on the OLED; any button returns to the menu.
 * The RAM map and the saved map are left untouched. */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "app_states.h"
#include "motion_client.h"
#include "ui.h"

static const char *TAG = "app";

static bool any_pressed(const ui_events_t *ev)
{
    for (int i = 0; i < BTN_COUNT; i++) {
        if (ev->pressed[i]) {
            return true;
        }
    }
    return false;
}

app_state_id_t state_fault_run(app_ctx_t *ctx)
{
    const char *reason = (ctx->fault_reason != NULL) ? ctx->fault_reason : "?";

    motion_stop();
    ESP_LOGW(TAG, "fault: %s", reason);
    ui_message("== FAULT ==", reason, "any button: menu");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(UI_POLL_MS));
        ui_events_t ev = ui_poll();
        if (any_pressed(&ev)) {
            ctx->fault_reason = NULL;
            return ST_MENU;
        }
    }
}
