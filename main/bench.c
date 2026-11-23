#include "bench.h"
#include "app_config.h"

#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_ipc.h"
#include "button.h"
#include "encoder.h"
#include "imu.h"
#include "motion_task.h"
#include "motor.h"
#include "navigation_task.h"
#include "oled.h"
#include "tuning_task.h"
#include "pins.h"
#include "tof.h"

#if BENCH_MOTOR_TEST || BENCH_ENCODER_TEST || BENCH_IMU_TEST || BENCH_TOF_TEST
static const char *TAG = "bench";
#endif

#if BENCH_OLED_TEST
static void oled_draw_test_text(void)
{
    oled_clear();
    oled_text_line(0, "MICRO FARR");
    oled_text_line(1, "OLED TEST");
    oled_text_line(2, "L1 1234567890");
    oled_text_line(3, "L2 ABCDEFGHIJ");
    oled_text_line(4, "L3 abcdefghij");
    oled_text_line(5, "L4 !@#$%^&*()");
    oled_text_line(6, "L5 128x64 SSD");
    oled_text_line(7, "L6 BOTTOM LINE");
    oled_flush();
}

static void bench_oled_test(void)
{
    if (oled_init() != ESP_OK) {
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    while (1) {
        oled_fill(true);
        oled_flush();
        vTaskDelay(pdMS_TO_TICKS(3000));

        oled_clear();
        oled_flush();
        vTaskDelay(pdMS_TO_TICKS(3000));

        oled_draw_test_text();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
#endif

#if BENCH_MOTOR_TEST
static void bench_motor_test(void)
{
    ESP_ERROR_CHECK(motor_init());
    motor_enable(true);

    while (1) {
        ESP_LOGI(TAG, "forward");
        motor_set_left(BENCH_MOTOR_SPEED);
        motor_set_right(BENCH_MOTOR_SPEED);
        vTaskDelay(pdMS_TO_TICKS(BENCH_MOTOR_TIME_MS));

        ESP_LOGI(TAG, "reverse");
        motor_set_left(-BENCH_MOTOR_SPEED);
        motor_set_right(-BENCH_MOTOR_SPEED);
        vTaskDelay(pdMS_TO_TICKS(BENCH_MOTOR_TIME_MS));

        ESP_LOGI(TAG, "stop");
        motor_set_left(0);
        motor_set_right(0);
        vTaskDelay(pdMS_TO_TICKS(BENCH_MOTOR_TIME_MS));
    }
}
#endif

#if BENCH_ENCODER_TEST
static void bench_encoder_test(void)
{
    encoder_init();

    while (1) {
        int32_t delta_l = encoder_get_delta(&encoder_left);
        int32_t delta_r = encoder_get_delta(&encoder_right);

        ESP_LOGI(TAG, "L d=%ld t=%ld | R d=%ld t=%ld",
                 (long)delta_l, (long)encoder_get_total(&encoder_left),
                 (long)delta_r, (long)encoder_get_total(&encoder_right));

        vTaskDelay(pdMS_TO_TICKS(BENCH_ENCODER_PERIOD_MS));
    }
}
#endif

#if BENCH_IMU_TEST
static void bench_imu_test(void)
{
    ESP_ERROR_CHECK(imu_init());
    ESP_LOGI(TAG, "IMU detected: %s", imu_model_name());
    ESP_ERROR_CHECK(imu_calibrate(IMU_CALIB_SAMPLES));

    const float dt_s = (float)BENCH_IMU_PERIOD_MS / 1000.0f;

    while (1) {
        imu_update_yaw(dt_s);
        ESP_LOGI(TAG, "yaw=%.1f", imu_get_yaw_deg());
        vTaskDelay(pdMS_TO_TICKS(BENCH_IMU_PERIOD_MS));
    }
}
#endif

#if BENCH_TOF_TEST
static void bench_tof_test(void)
{
    ESP_ERROR_CHECK(tof_init_all());

    tof_reading_t left, front, right;

    while (1) {
        tof_service();

        tof_get_reading(TOF_LEFT, &left);
        tof_get_reading(TOF_FRONT, &front);
        tof_get_reading(TOF_RIGHT, &right);

        ESP_LOGI(TAG, "L=%u(s%u %.2f/%.2f) F=%u(s%u %.2f/%.2f) R=%u(s%u %.2f/%.2f)",
                 left.distance_mm, left.range_status,
                 left.signal_rate_mcps, left.ambient_rate_mcps,
                 front.distance_mm, front.range_status,
                 front.signal_rate_mcps, front.ambient_rate_mcps,
                 right.distance_mm, right.range_status,
                 right.signal_rate_mcps, right.ambient_rate_mcps);

        vTaskDelay(pdMS_TO_TICKS(BENCH_TOF_PERIOD_MS));
    }
}
#endif

bool bench_run_if_enabled(bench_init_fn_t init_fn)
{
#if BENCH_OLED_TEST
    (void)init_fn;
    esp_log_level_set("*", ESP_LOG_NONE);
    bench_oled_test();
    return true;
#elif BENCH_MOTOR_TEST
    (void)init_fn;
    bench_motor_test();
    return true;
#elif BENCH_ENCODER_TEST
    (void)init_fn;
    ESP_LOGI(TAG, "micro_farr boot - encoder bench");
    bench_encoder_test();
    return true;
#elif BENCH_IMU_TEST
    (void)init_fn;
    ESP_LOGI(TAG, "micro_farr boot - imu bench");
    bench_imu_test();
    return true;
#elif BENCH_TOF_TEST
    (void)init_fn;
    ESP_LOGI(TAG, "micro_farr boot - tof bench");
    bench_tof_test();
    return true;
#elif BENCH_TELEMETRY_TEST
    if (init_fn) init_fn();
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    return true;
#elif BENCH_MAZE_TEST
    if (init_fn) init_fn();
    navigation_task_start();
    return true;
#elif BENCH_TUNE_TEST
    if (init_fn) init_fn();
    tuning_task_start();
    return true;
#else
    (void)init_fn;
    return false;
#endif
}
