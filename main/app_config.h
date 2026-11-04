#pragma once

#define BENCH_OLED_TEST         0
#define BENCH_TELEMETRY_TEST    0

#define BENCH_MOTOR_TEST        0
#define BENCH_MOTOR_SPEED       30
#define BENCH_MOTOR_TIME_MS     2000

#define BENCH_ENCODER_TEST      0
#define BENCH_ENCODER_PERIOD_MS 100

#define BENCH_IMU_TEST          0
#define BENCH_IMU_PERIOD_MS     100

#define BENCH_TOF_TEST          0
#define BENCH_TOF_PERIOD_MS     50

#define BENCH_CELL_CAL_TEST      0
#define BENCH_CELL_CAL_PERIOD_MS 200

#define BENCH_STRAIGHT_TEST      0
#define BENCH_STRAIGHT_CELLS     5
#define BENCH_STRAIGHT_REPEATS   1
#define BENCH_STRAIGHT_DELAY_MS  500

#define BENCH_TURN_TEST          1
#define BENCH_TURN_REPEATS       4
#define BENCH_TURN_DELAY_MS      800

#define BUTTON_ACTIVE_LEVEL      0

#define ENCODER_HIGH_LIMIT      30000
#define ENCODER_LOW_LIMIT      -30000
#define ENCODER_GLITCH_NS       1000

#define I2C_PORT_NUM            0
#define I2C_SPEED_HZ            400000
#define I2C_TIMEOUT_MS          100

#define OLED_ADDR_PRIMARY       0x3C
#define OLED_ADDR_ALT           0x3D
#define OLED_WIDTH              128
#define OLED_HEIGHT             64
#define OLED_PAGES              (OLED_HEIGHT / 8)
#define OLED_COLS               (OLED_WIDTH / 6)
#define OLED_I2C_TIMEOUT_MS     1000
#define OLED_I2C_SPEED_HZ       400000
#define OLED_COL_OFFSET         2

#define IMU_ADDR_PRIMARY        0x68
#define IMU_ADDR_ALT            0x69
#define IMU_CALIB_SAMPLES       1000
#define IMU_CALIB_DELAY_MS      2
#define GYRO_LSB_PER_DPS        65.5f
#define GYRO_SCALE              1.029f  /* sensitivity correction; 1.0 = none */

#define TOF_DEFAULT_ADDR        0x29
#define TOF_ADDR_LEFT           0x30
#define TOF_ADDR_FRONT          0x31
#define TOF_ADDR_RIGHT          0x32
#define TOF_BOOT_MS             10
#define TOF_ADDR_SETTLE_MS      100
#define TOF_PROBE_MS            20
#define TOF_MEASURE_TIMEOUT_MS  100

#define SENSOR_TASK_HZ          100
#define MOTION_TASK_HZ          200
#define TELEMETRY_TASK_HZ       20
#define TELEMETRY_TASK_CORE     0
#define TELEMETRY_TASK_PRIO     2
#define TELEMETRY_TASK_STACK    4096
#define SENSOR_TASK_CORE        1
#define MOTION_TASK_CORE        1
#define SENSOR_TASK_PRIO        9
#define MOTION_TASK_PRIO        10
#define SENSOR_TASK_STACK       4096
#define MOTION_TASK_STACK       4096

#define CELL_TICKS              1495
#define ENCODER_LEFT_SIGN       1
#define ENCODER_RIGHT_SIGN      1

#define STRAIGHT_FF_DUTY        0.0f
#define DIST_KP                 0.06f
#define MOTION_MIN_DUTY         15.0f
#define MOTION_MAX_DUTY         30.0f
#define MOTION_ACCEL_SLEW       0.5f

#define YAW_KP                  0.5f
#define YAW_MAX_CORR            8.0f
#define YAW_STEER_SIGN          1

#define SEGMENT_TIMEOUT_MS      5000

#define TURN_TARGET_DEG         90.0f
#define TURN_TOL_DEG            1.0f
#define TURN_YAW_SIGN           1
#define TURN_KP                 1.70f
#define TURN_FF_DUTY            12.0f
#define TURN_MIN_DUTY           12.0f
#define TURN_MAX_DUTY           40.0f
#define TURN_ACCEL_SLEW         0.8f
#define TURN_COAST_MS           60
#define TURN_SETTLE_MS          250
#define TURN_TIMEOUT_MS         3000
