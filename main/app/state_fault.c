/* FAULT: motors stopped, reason on the OLED; any button returns to the menu.
 * The RAM map and the saved map are left untouched. */

#include "esp_log.h"
#include "app_states.h"
#include "motion_client.h"
#include "ui.h"

static const char *TAG = "app";

app_state_id_t state_fault_run(app_ctx_t *ctx)
{
    const char *reason = (ctx->fault_reason != NULL) ? ctx->fault_reason : "?";

    motion_stop();
    ESP_LOGW(TAG, "fault: %s", reason);
    ui_message("== FAULT ==", reason, "any button: menu");
    ui_wait_any();
    ctx->fault_reason = NULL;
    return ST_MENU;
}
