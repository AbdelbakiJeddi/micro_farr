#include "i2c_bus.h"
#include "pins.h"
#include "app_config.h"

#include "esp_log.h"

static const char *TAG = "i2c_bus";

static i2c_master_bus_handle_t s_bus = NULL;

esp_err_t i2c_bus_init(void)
{
    if (s_bus) {
        return ESP_OK;
    }

    i2c_master_bus_config_t cfg = {
        .i2c_port                 = I2C_PORT_NUM,
        .sda_io_num               = PIN_I2C_SDA,
        .scl_io_num               = PIN_I2C_SCL,
        .clk_source               = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt        = 7,
        .flags.enable_internal_pullup = 1,
    };

    return i2c_new_master_bus(&cfg, &s_bus);
}

i2c_master_bus_handle_t i2c_bus_handle(void)
{
    return s_bus;
}

void i2c_bus_scan(void)
{
    if (!s_bus) {
        return;
    }

    ESP_LOGI(TAG, "scan start");
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(s_bus, addr, I2C_TIMEOUT_MS) == ESP_OK) {
            ESP_LOGI(TAG, "found 0x%02X", addr);
        }
    }
    ESP_LOGI(TAG, "scan done");
}
