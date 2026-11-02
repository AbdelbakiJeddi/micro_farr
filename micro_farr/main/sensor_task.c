#include "sensor_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "app_config.h"
#include "app_ipc.h"
#include "drivers/drv_imu.h"
#include "drivers/drv_tof.h"

static const char *TAG = "sensor_task";

static void sensor_task(void *arg)
{
    (void)arg;

    ESP_ERROR_CHECK(drv_imu_init());
    ESP_ERROR_CHECK(drv_tof_init());

    imu_sample_t imu;
    tof_frame_t tof;
    tof_pair_t pair;

    vTaskDelay(pdMS_TO_TICKS(SENSOR_OFFSET_MS));
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        if (drv_imu_read(&imu) == ESP_OK) {
            xQueueOverwrite(g_imu_q, &imu);
        }

        if (drv_tof_read(&tof) == ESP_OK) {
            xQueueOverwrite(g_tof_nav_q, &tof);

            pair.left_mm = tof.left_mm;
            pair.right_mm = tof.right_mm;
            pair.timestamp_us = tof.timestamp_us;
            pair.valid = tof.valid;
            xQueueOverwrite(g_tof_motion_q, &pair);
        }

        g_heartbeats.sensor++;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}

void sensor_task_start(void)
{
    BaseType_t ok = xTaskCreatePinnedToCore(sensor_task, "sensor",
                                            SENSOR_STACK_BYTES, NULL,
                                            SENSOR_PRIO, NULL, IO_CORE);
    configASSERT(ok == pdPASS);
    ESP_LOGI(TAG, "started");
}
