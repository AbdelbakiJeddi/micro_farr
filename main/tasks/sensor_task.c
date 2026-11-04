#include "sensor_task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "app_ipc.h"
#include "encoder.h"
#include "imu.h"
#include "tof.h"

static const char *TAG = "sensor";

static int64_t s_last_us;

static float loop_dt_s(int64_t now_us)
{
    int64_t delta_us = now_us - s_last_us;
    s_last_us = now_us;

    if (delta_us <= 0) {
        return 1.0f / (float)SENSOR_TASK_HZ;
    }

    float dt_s = (float)delta_us / 1000000.0f;

    if (dt_s < 0.001f) {
        return 0.001f;
    }
    if (dt_s > 0.100f) {
        return 0.100f;
    }
    return dt_s;
}

static int32_t tof_distance_mm(tof_id_t id)
{
    tof_reading_t reading;
    if (tof_get_reading(id, &reading) && reading.valid) {
        return (int32_t)reading.distance_mm;
    }
    return -1;
}

static void publish_snapshot(void)
{
    sensor_snapshot_t snapshot = {
        .left_tof    = tof_distance_mm(TOF_LEFT),
        .right_tof   = tof_distance_mm(TOF_RIGHT),
        .front_tof   = tof_distance_mm(TOF_FRONT),
        .yaw         = imu_get_yaw_deg(),
        .left_ticks  = encoder_get_total(&encoder_left) * ENCODER_LEFT_SIGN,
        .right_ticks = encoder_get_total(&encoder_right) * ENCODER_RIGHT_SIGN,
        .timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL),
    };

    if (!app_ipc_publish_sensor(&snapshot)) {
        ESP_LOGW(TAG, "snapshot publish failed");
    }
}

static void sensor_task(void *arg)
{
    (void)arg;

    const TickType_t period = pdMS_TO_TICKS(1000 / SENSOR_TASK_HZ);
    TickType_t last_wake = xTaskGetTickCount();

    s_last_us = esp_timer_get_time();

    while (1) {
        tof_service();

        int64_t now_us = esp_timer_get_time();
        imu_update_yaw(loop_dt_s(now_us));

        publish_snapshot();

        vTaskDelayUntil(&last_wake, period);
    }
}

void sensor_task_start(void)
{
    BaseType_t ok = xTaskCreatePinnedToCore(sensor_task, "sensor",
                                            SENSOR_TASK_STACK, NULL,
                                            SENSOR_TASK_PRIO, NULL,
                                            SENSOR_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "sensor task create failed");
    }
}
