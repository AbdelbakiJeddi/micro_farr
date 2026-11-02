#include "motion_control_task.h"

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "app_config.h"
#include "app_ipc.h"
#include "drivers/drv_encoder.h"
#include "drivers/drv_motor.h"

static const char *TAG = "motion";

#define DEG_TO_RAD 0.01745329252f

/* TODO: replace with measured geometry. */
#define COUNTS_PER_MM   1.0f
#define WHEEL_BASE_MM   100.0f

static void motion_control_task(void *arg)
{
    (void)arg;

    ESP_ERROR_CHECK(drv_encoder_init());
    ESP_ERROR_CHECK(drv_motor_init());

    motion_cmd_t cmd = { .mode = MOTION_MODE_STOP };
    pose_t pose = {0};
    int32_t prev_left = 0;
    int32_t prev_right = 0;
    uint32_t prev_us = (uint32_t)esp_timer_get_time();

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        uint32_t notify = 0;
        xTaskNotifyWait(0, 0xFFFFFFFFu, &notify, 0);

        if (notify & NOTIFY_ESTOP) {
            xEventGroupSetBits(g_system_events, EVT_ESTOP_LATCHED);
        }
        if (notify & NOTIFY_RESET) {
            xEventGroupClearBits(g_system_events,
                                 EVT_ESTOP_LATCHED | EVT_FAULT_LATCHED);
        }

        motion_cmd_t incoming;
        if (xQueueReceive(g_motion_cmd_q, &incoming, 0) == pdTRUE) {
            cmd = incoming;
        }

        imu_sample_t imu;
        if (xQueuePeek(g_imu_q, &imu, 0) != pdTRUE) {
            memset(&imu, 0, sizeof(imu));
        }

        tof_pair_t tof;
        if (xQueuePeek(g_tof_motion_q, &tof, 0) != pdTRUE) {
            memset(&tof, 0, sizeof(tof));
        }
        (void)tof; /* TODO: wall centering correction. */

        int32_t left_count = 0;
        int32_t right_count = 0;
        drv_encoder_read(&left_count, &right_count);

        uint32_t now_us = (uint32_t)esp_timer_get_time();
        float dt = (float)(now_us - prev_us) / 1000000.0f;
        if (dt <= 0.0f) {
            dt = (float)MOTION_PERIOD_MS / 1000.0f;
        }
        prev_us = now_us;

        float dleft_mm = (float)(left_count - prev_left) / COUNTS_PER_MM;
        float dright_mm = (float)(right_count - prev_right) / COUNTS_PER_MM;
        prev_left = left_count;
        prev_right = right_count;

        float left_mps = dleft_mm / 1000.0f / dt;
        float right_mps = dright_mm / 1000.0f / dt;

        float dtheta_deg = (dright_mm - dleft_mm) / WHEEL_BASE_MM * (180.0f / 3.14159265f);

        /* TODO: proper IMU/odometry fusion. */
        pose.heading_deg += dtheta_deg + imu.gyro_dps[2] * dt;

        float dist_mm = 0.5f * (dleft_mm + dright_mm);
        float heading_rad = pose.heading_deg * DEG_TO_RAD;
        pose.x_mm += dist_mm * sinf(heading_rad);
        pose.y_mm += dist_mm * cosf(heading_rad);

        pose.left_mps = left_mps;
        pose.right_mps = right_mps;
        pose.cell_x = (uint8_t)(pose.x_mm / CELL_SIZE_MM);
        pose.cell_y = (uint8_t)(pose.y_mm / CELL_SIZE_MM);
        pose.timestamp_us = now_us;

        bool estop = (xEventGroupGetBits(g_system_events) & EVT_ESTOP_LATCHED) != 0;

        if (estop || cmd.mode == MOTION_MODE_STOP) {
            drv_motor_stop();
        } else if (cmd.mode == MOTION_MODE_STRAIGHT) {
            drv_motor_set(cmd.target_speed_mps, cmd.target_speed_mps);
        } else if (cmd.mode == MOTION_MODE_TURN) {
            drv_motor_set(-cmd.target_speed_mps, cmd.target_speed_mps);
        }

        xQueueOverwrite(g_pose_q, &pose);

        g_heartbeats.motion++;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MOTION_PERIOD_MS));
    }
}

void motion_control_task_start(void)
{
    BaseType_t ok = xTaskCreatePinnedToCore(motion_control_task, "motion",
                                            MOTION_STACK_BYTES, NULL,
                                            MOTION_PRIO, &g_motion_task_handle,
                                            MOTION_CORE);
    configASSERT(ok == pdPASS);
    ESP_LOGI(TAG, "started on core %d", MOTION_CORE);
}
