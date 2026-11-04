#include "tof.h"
#include "i2c_bus.h"
#include "pins.h"
#include "app_config.h"

#include "esp_check.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vl53l0x.h"

static const char *TAG = "tof";

typedef struct {
    gpio_num_t       xshut;
    uint8_t          address;
    vl53l0x_handle_t handle;
    tof_reading_t    reading;
} tof_sensor_t;

static tof_sensor_t s_sensors[TOF_COUNT] = {
    [TOF_LEFT]  = { PIN_XSHUT_L, TOF_ADDR_LEFT,  NULL, { 0, 0, 0, 0, false, 0 } },
    [TOF_FRONT] = { PIN_XSHUT_F, TOF_ADDR_FRONT, NULL, { 0, 0, 0, 0, false, 0 } },
    [TOF_RIGHT] = { PIN_XSHUT_R, TOF_ADDR_RIGHT, NULL, { 0, 0, 0, 0, false, 0 } },
};

static tof_id_t s_active = TOF_COUNT;
static tof_id_t s_next = TOF_LEFT;
static bool     s_busy;
static int64_t  s_started_us;

static void xshut_hold_all_low(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_XSHUT_L) |
                        (1ULL << PIN_XSHUT_F) |
                        (1ULL << PIN_XSHUT_R),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);

    gpio_set_level(PIN_XSHUT_L, 0);
    gpio_set_level(PIN_XSHUT_F, 0);
    gpio_set_level(PIN_XSHUT_R, 0);

    vTaskDelay(pdMS_TO_TICKS(TOF_BOOT_MS));
}

static esp_err_t verify_address(uint8_t address)
{
    esp_err_t old_err = i2c_master_probe(i2c_bus_handle(), TOF_DEFAULT_ADDR, TOF_PROBE_MS);
    if (old_err == ESP_OK) {
        ESP_LOGE(TAG, "address 0x%02X still responds", TOF_DEFAULT_ADDR);
        return ESP_FAIL;
    }

    esp_err_t new_err = i2c_master_probe(i2c_bus_handle(), address, TOF_PROBE_MS);
    if (new_err != ESP_OK) {
        ESP_LOGE(TAG, "address 0x%02X not responding", address);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "address 0x%02X -> 0x%02X verified", TOF_DEFAULT_ADDR, address);
    return ESP_OK;
}

static esp_err_t sensor_start(tof_sensor_t *s)
{
    gpio_set_level(s->xshut, 1);
    vTaskDelay(pdMS_TO_TICKS(TOF_BOOT_MS));

    ESP_RETURN_ON_ERROR(vl53l0x_create(&s->handle, i2c_bus_handle()), TAG, "create failed");
    ESP_RETURN_ON_ERROR(vl53l0x_init(s->handle), TAG, "init failed");

    vl53l0x_ref_calibration_t cal = {0};
    ESP_RETURN_ON_ERROR(vl53l0x_perform_ref_calibration(s->handle, &cal),
                        TAG, "ref calibration failed");

    ESP_RETURN_ON_ERROR(vl53l0x_set_profile(s->handle, VL53L0X_PROFILE_DEFAULT),
                        TAG, "set profile failed");
    ESP_RETURN_ON_ERROR(vl53l0x_set_mode(s->handle, VL53L0X_MODE_SINGLE),
                        TAG, "set mode failed");

    ESP_RETURN_ON_ERROR(vl53l0x_set_address(s->handle, i2c_bus_handle(), s->address),
                        TAG, "set address failed");
    vTaskDelay(pdMS_TO_TICKS(TOF_ADDR_SETTLE_MS));
    ESP_RETURN_ON_ERROR(verify_address(s->address), TAG, "address verify failed");
    return ESP_OK;
}

esp_err_t tof_init_all(void)
{
    ESP_RETURN_ON_ERROR(i2c_bus_init(), TAG, "i2c bus init failed");

    xshut_hold_all_low();

    for (tof_id_t id = 0; id < TOF_COUNT; id++) {
        ESP_RETURN_ON_ERROR(sensor_start(&s_sensors[id]), TAG, "sensor %d start failed", id);
        ESP_LOGI(TAG, "sensor %d ready @0x%02X", id, s_sensors[id].address);
    }
    return ESP_OK;
}

static void tof_store_reading(tof_sensor_t *s, const vl53l0x_data_t *data)
{
    s->reading.distance_mm       = data->distance_mm;
    s->reading.range_status      = data->range_status;
    s->reading.signal_rate_mcps  = data->signal_rate_mcps;
    s->reading.ambient_rate_mcps = data->ambient_rate_mcps;
    s->reading.valid             = data->valid;
    s->reading.timestamp_ms      = data->timestamp_ms;
}

static void tof_start_next(void)
{
    tof_id_t id = s_next;
    s_next = (tof_id_t)(((uint8_t)s_next + 1U) % TOF_COUNT);

    tof_sensor_t *s = &s_sensors[id];
    (void)vl53l0x_clear_interrupt_mask(s->handle);

    esp_err_t err = vl53l0x_start_measurement(s->handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "sensor %d start err %s", id, esp_err_to_name(err));
        return;
    }

    s_active     = id;
    s_busy       = true;
    s_started_us = esp_timer_get_time();
}

static void tof_collect(void)
{
    tof_sensor_t *s = &s_sensors[s_active];
    bool ready = false;

    esp_err_t err = vl53l0x_get_ready(s->handle, &ready);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "sensor %d ready err %s", s_active, esp_err_to_name(err));
        (void)vl53l0x_clear_interrupt_mask(s->handle);
        s_busy = false;
        return;
    }

    if (!ready) {
        int64_t age_us = esp_timer_get_time() - s_started_us;
        if (age_us < ((int64_t)TOF_MEASURE_TIMEOUT_MS * 1000)) {
            return;
        }
        ESP_LOGW(TAG, "sensor %d measure timeout", s_active);
        (void)vl53l0x_clear_interrupt_mask(s->handle);
        s_busy = false;
        return;
    }

    vl53l0x_data_t data;
    err = vl53l0x_get_data(s->handle, &data);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "sensor %d get data err %s", s_active, esp_err_to_name(err));
    } else {
        tof_store_reading(s, &data);
    }

    (void)vl53l0x_clear_interrupt_mask(s->handle);
    s_busy = false;
}

void tof_service(void)
{
    if (s_busy) {
        tof_collect();
    } else {
        tof_start_next();
    }
}

bool tof_get_reading(tof_id_t id, tof_reading_t *out)
{
    if (id >= TOF_COUNT || out == NULL) {
        return false;
    }
    *out = s_sensors[id].reading;
    return out->valid;
}
