#include "navigation_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "app_config.h"
#include "app_ipc.h"

static const char *TAG = "navigation";

static void navigation_task(void *arg)
{
    (void)arg;

    pose_t pose = {0};
    system_state_t state = { .state = SYS_EXPLORE };

    for (;;) {
        tof_frame_t tof;

        /* Event-driven: wake on a new ToF frame, or on the reaction timeout. */
        BaseType_t got = xQueueReceive(g_tof_nav_q, &tof,
                                       pdMS_TO_TICKS(NAV_EVENT_TIMEOUT_MS));
        if (got == pdTRUE) {
            (void)xQueuePeek(g_pose_q, &pose, 0);

            /* TODO: wall detection, wall matrix update, flood fill, and
             * publish a motion_cmd_t to g_motion_cmd_q. */
            (void)tof;
        }

        state.state = SYS_EXPLORE;
        state.pose = pose;
        state.fault = FAULT_NONE;
        state.uptime_ms = (uint32_t)(esp_timer_get_time() / 1000);
        xQueueOverwrite(g_system_state_q, &state);

        g_heartbeats.navigation++;
    }
}

void navigation_task_start(void)
{
    BaseType_t ok = xTaskCreatePinnedToCore(navigation_task, "navigation",
                                            NAV_STACK_BYTES, NULL,
                                            NAV_PRIO, NULL, IO_CORE);
    configASSERT(ok == pdPASS);
    ESP_LOGI(TAG, "started");
}
