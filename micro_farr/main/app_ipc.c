#include "app_ipc.h"

#include "app_config.h"

QueueHandle_t g_imu_q = NULL;
QueueHandle_t g_tof_motion_q = NULL;
QueueHandle_t g_tof_nav_q = NULL;
QueueHandle_t g_pose_q = NULL;
QueueHandle_t g_motion_cmd_q = NULL;
QueueHandle_t g_system_state_q = NULL;

SemaphoreHandle_t g_i2c_mutex = NULL;
EventGroupHandle_t g_system_events = NULL;

TaskHandle_t g_motion_task_handle = NULL;

heartbeat_counters_t g_heartbeats = {0};

esp_err_t app_ipc_init(void)
{
    g_imu_q = xQueueCreate(APP_LATEST_DEPTH, sizeof(imu_sample_t));
    g_tof_motion_q = xQueueCreate(APP_LATEST_DEPTH, sizeof(tof_pair_t));
    g_tof_nav_q = xQueueCreate(APP_LATEST_DEPTH, sizeof(tof_frame_t));
    g_pose_q = xQueueCreate(APP_LATEST_DEPTH, sizeof(pose_t));
    g_motion_cmd_q = xQueueCreate(MOTION_CMD_DEPTH, sizeof(motion_cmd_t));
    g_system_state_q = xQueueCreate(APP_LATEST_DEPTH, sizeof(system_state_t));

    g_i2c_mutex = xSemaphoreCreateMutex();
    g_system_events = xEventGroupCreate();

    if (g_imu_q == NULL || g_tof_motion_q == NULL || g_tof_nav_q == NULL ||
        g_pose_q == NULL || g_motion_cmd_q == NULL || g_system_state_q == NULL ||
        g_i2c_mutex == NULL || g_system_events == NULL) {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}
