#include "bench.h"
#include "app_config.h"

#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_ipc.h"
#include "encoder.h"
#include "imu.h"
#include "motion_control_task.h"
#include "motor.h"
#include "oled.h"
#include "pins.h"
#include "tof.h"

#if BENCH_MOTOR_TEST || BENCH_ENCODER_TEST || BENCH_IMU_TEST || BENCH_TOF_TEST || BENCH_CELL_CAL_TEST || BENCH_STRAIGHT_TEST || BENCH_TURN_TEST
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

#if BENCH_CELL_CAL_TEST
static void bench_cell_cal_test(void)
{
    bool oled_ok = (oled_init() == ESP_OK);
    if (oled_ok) {
        esp_log_level_set("*", ESP_LOG_NONE);
    }

    encoder_init();

    int32_t start_l = encoder_get_total(&encoder_left);
    int32_t start_r = encoder_get_total(&encoder_right);

    while (1) {
        int32_t left  = encoder_get_total(&encoder_left) - start_l;
        int32_t right = encoder_get_total(&encoder_right) - start_r;
        int32_t avg   = (left + right) / 2;

        if (oled_ok) {
            char line[22];

            oled_clear();
            oled_text_line(0, "CELL CAL");

            snprintf(line, sizeof(line), "L %ld", (long)left);
            oled_text_line(1, line);

            snprintf(line, sizeof(line), "R %ld", (long)right);
            oled_text_line(2, line);

            snprintf(line, sizeof(line), "AVG %ld", (long)avg);
            oled_text_line(3, line);

            oled_flush();
        } else {
            ESP_LOGI(TAG, "L=%ld R=%ld avg=%ld",
                     (long)left, (long)right, (long)avg);
        }

        vTaskDelay(pdMS_TO_TICKS(BENCH_CELL_CAL_PERIOD_MS));
    }
}
#endif

#if BENCH_STRAIGHT_TEST || BENCH_TURN_TEST
static void button_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_BUTTON),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
}

static void button_wait_press(void)
{
    /* If button was held down during power-on/reset, wait for release */
    while (gpio_get_level(PIN_BUTTON) == BUTTON_ACTIVE_LEVEL) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    /* Wait for press */
    while (gpio_get_level(PIN_BUTTON) != BUTTON_ACTIVE_LEVEL) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    /* Debounce */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Wait for release */
    while (gpio_get_level(PIN_BUTTON) == BUTTON_ACTIVE_LEVEL) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    /* Short safety settle delay before launch so hand is clear */
    vTaskDelay(pdMS_TO_TICKS(BENCH_STRAIGHT_DELAY_MS));
}
#endif

#if BENCH_STRAIGHT_TEST
static void bench_straight_test(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    button_init();

    while (1) {
        ESP_LOGI(TAG, "spawned in IDLE. Press BOOT button to launch (%d cells)...",
                 BENCH_STRAIGHT_CELLS);

        button_wait_press();

        ESP_LOGI(TAG, "button pressed! Starting straight run (%d cells, %ld ticks)",
                 BENCH_STRAIGHT_CELLS,
                 (long)(BENCH_STRAIGHT_CELLS * CELL_TICKS));

        esp_err_t err = straight_cells(BENCH_STRAIGHT_CELLS);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "straight_cells failed: %s", esp_err_to_name(err));
            continue;
        }

        uint32_t wait_ms = (SEGMENT_TIMEOUT_MS * BENCH_STRAIGHT_CELLS) + 2000;
        EventBits_t bits = app_ipc_wait_event(
            APP_EVENT_SEGMENT_DONE | APP_EVENT_FAULT, true,
            pdMS_TO_TICKS(wait_ms));

        sensor_snapshot_t snapshot;
        bool have_snapshot = app_ipc_read_sensor(&snapshot);

        if (bits & APP_EVENT_SEGMENT_DONE) {
            ESP_LOGI(TAG, "run done: L=%ld R=%ld yaw=%.1f",
                     (long)(have_snapshot ? snapshot.left_ticks : 0),
                     (long)(have_snapshot ? snapshot.right_ticks : 0),
                     have_snapshot ? snapshot.yaw : 0.0f);
        } else if (bits & APP_EVENT_FAULT) {
            ESP_LOGE(TAG, "run fault");
        } else {
            ESP_LOGE(TAG, "run timeout waiting for segment");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#endif

#if BENCH_TURN_TEST
static void bench_turn_test(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    button_init();

    while (1) {
        ESP_LOGI(TAG, "spawned in IDLE. Press BOOT button to run %d turns...",
                 BENCH_TURN_REPEATS);

        button_wait_press();

        for (int i = 0; i < BENCH_TURN_REPEATS; i++) {
            bool left = (i % 2) == 0;

            sensor_snapshot_t before;
            bool have_before = app_ipc_read_sensor(&before);
            float start_yaw = have_before ? before.yaw : 0.0f;

            ESP_LOGI(TAG, "turn %d/%d: %s", i + 1, BENCH_TURN_REPEATS,
                     left ? "left" : "right");

            esp_err_t err = left ? turn_left() : turn_right();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "turn request failed: %s", esp_err_to_name(err));
                break;
            }

            EventBits_t bits = app_ipc_wait_event(
                APP_EVENT_SEGMENT_DONE | APP_EVENT_FAULT, true,
                pdMS_TO_TICKS(TURN_TIMEOUT_MS + 1000));

            sensor_snapshot_t after;
            bool have_after = app_ipc_read_sensor(&after);
            float end_yaw = have_after ? after.yaw : 0.0f;
            float expected = (left ? 1.0f : -1.0f) * TURN_TARGET_DEG *
                             (float)TURN_YAW_SIGN;

            if (bits & APP_EVENT_SEGMENT_DONE) {
                ESP_LOGI(TAG, "done: yaw %.1f -> %.1f (delta %.1f, expected %+.1f)",
                         start_yaw, end_yaw, end_yaw - start_yaw, expected);
            } else if (bits & APP_EVENT_FAULT) {
                ESP_LOGE(TAG, "turn fault");
            } else {
                ESP_LOGE(TAG, "turn timeout waiting for completion");
            }

            vTaskDelay(pdMS_TO_TICKS(BENCH_TURN_DELAY_MS));
        }
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
#elif BENCH_CELL_CAL_TEST
    (void)init_fn;
    bench_cell_cal_test();
    return true;
#elif BENCH_TELEMETRY_TEST
    if (init_fn) init_fn();
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    return true;
#elif BENCH_STRAIGHT_TEST
    if (init_fn) init_fn();
    bench_straight_test();
    return true;
#elif BENCH_TURN_TEST
    if (init_fn) init_fn();
    bench_turn_test();
    return true;
#else
    (void)init_fn;
    return false;
#endif
}
