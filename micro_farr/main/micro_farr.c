#include "esp_err.h"
#include "esp_log.h"

#include "app_ipc.h"

#include "motion_control_task.h"
#include "navigation_task.h"
#include "sensor_task.h"
#include "supervisor_task.h"

static const char *TAG = "app";

void app_main(void)
{
    ESP_LOGI(TAG, "micromouse starting");

    ESP_ERROR_CHECK(app_ipc_init());

    /* Start motion first so its handle exists for ESTOP notifications. */
    motion_control_task_start();
    sensor_task_start();
    navigation_task_start();
    supervisor_task_start();
}
