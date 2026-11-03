#pragma once

#define BENCH_MOTOR_TEST        0
#define BENCH_MOTOR_SPEED       30
#define BENCH_MOTOR_TIME_MS     2000

#define BENCH_ENCODER_TEST      0
#define BENCH_ENCODER_PERIOD_MS 100

#define BENCH_IMU_TEST          1
#define BENCH_IMU_PERIOD_MS     100

#define ENCODER_HIGH_LIMIT      30000
#define ENCODER_LOW_LIMIT      -30000
#define ENCODER_GLITCH_NS       1000

#define I2C_PORT_NUM            0
#define I2C_SPEED_HZ            400000
#define I2C_TIMEOUT_MS          100

#define IMU_ADDR_PRIMARY        0x68
#define IMU_ADDR_ALT            0x69
#define IMU_CALIB_SAMPLES       500
#define IMU_CALIB_DELAY_MS      2
#define GYRO_LSB_PER_DPS        65.5f
