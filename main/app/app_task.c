#include "app_task.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "app_states.h"

static const char *TAG = "app";

/* Until the explore and speed-run states exist. */
static app_state_id_t state_not_ready(app_ctx_t *ctx)
{
    ctx->fault_reason = "not ready";
    return ST_FAULT;
}

static const state_run_fn s_states[ST_COUNT] = {
    [ST_MENU]      = state_menu_run,
    [ST_EXPLORE]   = state_not_ready,
    [ST_SPEED_RUN] = state_not_ready,
    [ST_TUNER]     = state_tuner_run,
    [ST_FAULT]     = state_fault_run,
};

static app_ctx_t s_ctx;   /* holds the maze: kept off the task stack */

static void app_task(void *arg)
{
    (void)arg;
    app_state_id_t state = ST_MENU;

    while (1) {
        ESP_LOGI(TAG, "state %d", (int)state);
        state = s_states[state](&s_ctx);
    }
}

void app_task_start(void)
{
    BaseType_t ok = xTaskCreatePinnedToCore(app_task, "app", APP_TASK_STACK, NULL,
                                            APP_TASK_PRIO, NULL, APP_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "app task create failed");
    }
}
