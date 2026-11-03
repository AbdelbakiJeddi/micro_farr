#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "motor.h"

static const char *TAG = "main";

static void run_bench_motor_test(void)
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

void app_main(void)
{
    ESP_LOGI(TAG, "micro_farr boot");

#if BENCH_MOTOR_TEST
    run_bench_motor_test();
#endif
}
