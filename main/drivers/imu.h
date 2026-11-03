#pragma once

#include <stdint.h>

#include "esp_err.h"

typedef enum {
    IMU_MODEL_UNKNOWN,
    IMU_MODEL_MPU6050,
    IMU_MODEL_MPU6500,
    IMU_MODEL_MPU9250,
} imu_model_t;

esp_err_t   imu_init(void);
imu_model_t imu_get_model(void);
const char *imu_model_name(void);
esp_err_t   imu_calibrate(uint16_t samples);
void        imu_update_yaw(float dt_s);
float       imu_get_yaw_deg(void);
