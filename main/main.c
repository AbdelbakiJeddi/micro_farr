#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "app_ipc.h"
#include "bench.h"
#include "encoder.h"
#include "imu.h"
#include "motion_task.h"
#include "motor.h"
#include "nvs_flash.h"
#include "params.h"
#include "sensor_task.h"
#include "telemetry_task.h"
#include "tof.h"

static void app_fatal(void) __attribute__((noreturn));

static void app_fatal(void)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* NVS holds the saved maze; a full or old-format partition is erased and re-initialised. */
static esp_err_t storage_init(void)
{
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        if (nvs_flash_erase() != ESP_OK) {
            return ESP_FAIL;
        }
        err = nvs_flash_init();
    }
    return err;
}

static void app_init_and_start(void)
{
    esp_log_level_set("*", ESP_LOG_NONE);

    if (storage_init() != ESP_OK || app_ipc_init() != ESP_OK) {
        app_fatal();
    }

    telemetry_task_start();

    if (imu_init() != ESP_OK || imu_calibrate(IMU_CALIB_SAMPLES) != ESP_OK) {
        app_fatal();
    }

    if (tof_init_all() != ESP_OK) {
        app_fatal();
    }

    encoder_init();
    motor_init();
    motor_enable(false);

    app_ipc_set_event(APP_EVENT_CALIBRATED);

    sensor_task_start();
    motion_task_start();
}

void app_main(void)
{
    params_reset();

    if (bench_run_if_enabled(app_init_and_start)) {
        return;
    }

    app_init_and_start();
}
