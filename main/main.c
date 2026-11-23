#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "app_config.h"
#include "app_ipc.h"
#include "app_task.h"
#include "button.h"
#include "encoder.h"
#include "imu.h"
#include "motion_task.h"
#include "motor.h"
#include "oled.h"
#include "params.h"
#include "sensor_task.h"
#include "tof.h"
#include "ui.h"

static void app_fatal(const char *reason) __attribute__((noreturn));

static void app_fatal(const char *reason)
{
    ESP_LOGE("app", "init failed: %s", reason);
    ui_message("== INIT FAIL ==", reason, NULL);
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

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set("app", ESP_LOG_INFO);

    params_reset();

    if (storage_init() != ESP_OK) {
        app_fatal("NVS");
    }
    if (app_ipc_init() != ESP_OK) {
        app_fatal("IPC");
    }
    oled_init();   /* optional: the robot runs without a display */
    button_init();

    ui_message("== MICRO FARR ==", "calibrating", "keep still");
    if (imu_init() != ESP_OK || imu_calibrate(IMU_CALIB_SAMPLES) != ESP_OK) {
        app_fatal("IMU");
    }
    if (tof_init_all() != ESP_OK) {
        app_fatal("TOF");
    }
    encoder_init();
    motor_init();
    motor_enable(false);

    app_ipc_set_event(APP_EVENT_CALIBRATED);

    sensor_task_start();
    motion_task_start();
    app_task_start();
}
