#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "encoder.h"
#include "imu.h"
#include "motor.h"

static const char *TAG = "main";

#if BENCH_MOTOR_TEST
static void bench_motor_test(void)
{
    motor_init();
    motor_enable(true);

    motor_set_speed(&motor_left, BENCH_MOTOR_SPEED);
    vTaskDelay(pdMS_TO_TICKS(BENCH_MOTOR_TIME_MS));
    motor_stop(&motor_left);

    motor_set_speed(&motor_right, BENCH_MOTOR_SPEED);
    vTaskDelay(pdMS_TO_TICKS(BENCH_MOTOR_TIME_MS));
    motor_stop(&motor_right);

    motor_enable(false);
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

void app_main(void)
{
    ESP_LOGI(TAG, "micro_farr boot");

#if BENCH_MOTOR_TEST
    bench_motor_test();
#endif
#if BENCH_ENCODER_TEST
    bench_encoder_test();
#endif
#if BENCH_IMU_TEST
    bench_imu_test();
#endif
}
