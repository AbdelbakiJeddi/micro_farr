#include "imu.h"
#include "i2c_bus.h"
#include "app_config.h"

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_GYRO_ZOUT_H     0x47
#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75

#define WHO_AM_I_MPU6050    0x68
#define WHO_AM_I_MPU6500    0x70
#define WHO_AM_I_MPU9250    0x71

#define GYRO_Z_BYTES        2

static i2c_master_dev_handle_t s_dev;
static imu_model_t s_model = IMU_MODEL_UNKNOWN;
static float s_gyro_bias_dps;
static float s_yaw_deg;

static esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_transmit(s_dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

static esp_err_t read_gyro_z(int16_t *gz)
{
    uint8_t reg = REG_GYRO_ZOUT_H;
    uint8_t buf[GYRO_Z_BYTES];

    esp_err_t err = i2c_master_transmit_receive(s_dev, &reg, 1, buf, sizeof(buf),
                                                I2C_TIMEOUT_MS);
    if (err != ESP_OK) return err;

    *gz = (int16_t)((buf[0] << 8) | buf[1]);
    return ESP_OK;
}

static imu_model_t identify_model(uint8_t who)
{
    switch (who) {
    case WHO_AM_I_MPU6050: return IMU_MODEL_MPU6050;
    case WHO_AM_I_MPU6500: return IMU_MODEL_MPU6500;
    case WHO_AM_I_MPU9250: return IMU_MODEL_MPU9250;
    default:               return IMU_MODEL_UNKNOWN;
    }
}

static esp_err_t add_device(uint16_t address)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = address,
        .scl_speed_hz    = I2C_SPEED_HZ,
    };
    return i2c_master_bus_add_device(i2c_bus_handle(), &cfg, &s_dev);
}

static esp_err_t probe_address(i2c_master_bus_handle_t bus, uint16_t *found)
{
    const uint16_t candidates[] = { IMU_ADDR_PRIMARY, IMU_ADDR_ALT };

    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
        if (i2c_master_probe(bus, candidates[i], I2C_TIMEOUT_MS) == ESP_OK) {
            *found = candidates[i];
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t configure_device(void)
{
    esp_err_t err;

    err = write_reg(REG_PWR_MGMT_1, 0x80);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(100));

    err = write_reg(REG_PWR_MGMT_1, 0x01);
    if (err != ESP_OK) return err;

    err = write_reg(REG_SMPLRT_DIV, 0x04);
    if (err != ESP_OK) return err;

    err = write_reg(REG_CONFIG, 0x03);
    if (err != ESP_OK) return err;

    err = write_reg(REG_GYRO_CONFIG, 0x08);
    if (err != ESP_OK) return err;

    err = write_reg(REG_ACCEL_CONFIG, 0x00);
    return err;
}

esp_err_t imu_init(void)
{
    esp_err_t err = i2c_bus_init();
    if (err != ESP_OK) return err;

    uint16_t address = 0;
    err = probe_address(i2c_bus_handle(), &address);
    if (err != ESP_OK) return err;

    err = add_device(address);
    if (err != ESP_OK) return err;

    uint8_t reg = REG_WHO_AM_I;
    uint8_t who = 0;
    err = i2c_master_transmit_receive(s_dev, &reg, 1, &who, 1, I2C_TIMEOUT_MS);
    if (err != ESP_OK) return err;

    s_model = identify_model(who);

    return configure_device();
}

imu_model_t imu_get_model(void)
{
    return s_model;
}

const char *imu_model_name(void)
{
    switch (s_model) {
    case IMU_MODEL_MPU6050: return "MPU6050";
    case IMU_MODEL_MPU6500: return "MPU6500";
    case IMU_MODEL_MPU9250: return "MPU9250";
    default:                return "unknown";
    }
}

esp_err_t imu_calibrate(uint16_t samples)
{
    if (!s_dev || samples == 0) return ESP_ERR_INVALID_STATE;

    /* Settling delay for sensor PLL & DLPF to stabilize after boot */
    vTaskDelay(pdMS_TO_TICKS(150));

    int16_t gz;
    const uint8_t max_attempts = 5;

    for (uint8_t attempt = 0; attempt < max_attempts; attempt++) {
        int64_t sum = 0;
        int16_t min_gz = 32767;
        int16_t max_gz = -32768;

        for (uint16_t i = 0; i < samples; i++) {
            esp_err_t err = read_gyro_z(&gz);
            if (err != ESP_OK) return err;

            sum += gz;
            if (gz < min_gz) min_gz = gz;
            if (gz > max_gz) max_gz = gz;

            vTaskDelay(pdMS_TO_TICKS(IMU_CALIB_DELAY_MS));
        }

        /* 120 LSBs pk-pk at 65.5 LSB/dps is ~1.8 dps threshold for stillness */
        if ((max_gz - min_gz) < 120 || attempt == max_attempts - 1) {
            s_gyro_bias_dps = ((float)sum / samples) / GYRO_LSB_PER_DPS;
            s_yaw_deg = 0.0f;
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }

    return ESP_OK;
}

void imu_update_yaw(float dt_s)
{
    int16_t gz;
    if (read_gyro_z(&gz) != ESP_OK) return;

    float rate_dps = (float)gz / GYRO_LSB_PER_DPS - s_gyro_bias_dps;
    s_yaw_deg += rate_dps * dt_s;
}

float imu_get_yaw_deg(void)
{
    return s_yaw_deg;
}
