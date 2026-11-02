#include "supervisor_task.h"

#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "app_config.h"
#include "app_ipc.h"
#include "drivers/drv_oled.h"

static const char *TAG = "supervisor";

static void supervisor_task(void *arg)
{
    (void)arg;

    ESP_ERROR_CHECK(drv_oled_init());

    system_state_t state;

    uint32_t last_sensor = g_heartbeats.sensor;
    uint32_t last_motion = g_heartbeats.motion;
    uint32_t last_nav = g_heartbeats.navigation;

    vTaskDelay(pdMS_TO_TICKS(SUPERVISOR_OFFSET_MS));
    TickType_t last_wake = xTaskGetTickCount();

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t seen_sensor = now_ms;
    uint32_t seen_motion = now_ms;
    uint32_t seen_nav = now_ms;

    uint32_t oled_div = 0;
    bool fault_reported = false;

    for (;;) {
        now_ms = (uint32_t)(esp_timer_get_time() / 1000);

        if (g_heartbeats.sensor != last_sensor) {
            last_sensor = g_heartbeats.sensor;
            seen_sensor = now_ms;
        }
        if (g_heartbeats.motion != last_motion) {
            last_motion = g_heartbeats.motion;
            seen_motion = now_ms;
        }
        if (g_heartbeats.navigation != last_nav) {
            last_nav = g_heartbeats.navigation;
            seen_nav = now_ms;
        }

        bool stale = (now_ms - seen_sensor > HEARTBEAT_TIMEOUT_MS) ||
                     (now_ms - seen_motion > HEARTBEAT_TIMEOUT_MS) ||
                     (now_ms - seen_nav > HEARTBEAT_TIMEOUT_MS);

        if (stale) {
            xEventGroupSetBits(g_system_events, EVT_FAULT_LATCHED);
            if (!fault_reported) {
                ESP_LOGE(TAG, "heartbeat timeout; requesting ESTOP");
                xTaskNotify(g_motion_task_handle, NOTIFY_ESTOP, eSetBits);
                fault_reported = true;
            }
        } else {
            fault_reported = false;
        }

        if (++oled_div >= (OLED_REFRESH_MS / SUPERVISOR_PERIOD_MS)) {
            oled_div = 0;
            if (xQueuePeek(g_system_state_q, &state, 0) == pdTRUE) {
                drv_oled_show(&state);
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SUPERVISOR_PERIOD_MS));
    }
}

void supervisor_task_start(void)
{
    BaseType_t ok = xTaskCreatePinnedToCore(supervisor_task, "supervisor",
                                            SUPERVISOR_STACK_BYTES, NULL,
                                            SUPERVISOR_PRIO, NULL, IO_CORE);
    configASSERT(ok == pdPASS);
    ESP_LOGI(TAG, "started");
}
