#pragma once

#include <stdbool.h>
#include <stdint.h>

/* -------------------------------------------------------------------------- */
/* Sensors                                                                    */
/* -------------------------------------------------------------------------- */

#define TOF_COUNT 3

typedef enum {
    TOF_LEFT = 0,
    TOF_CENTER,
    TOF_RIGHT,
} tof_id_t;

/* Full frame: all three sensors, for navigation. */
typedef struct {
    int16_t left_mm;
    int16_t center_mm;
    int16_t right_mm;
    uint32_t timestamp_us;
    bool valid;
} tof_frame_t;

/* Reduced frame: left/right only, for wall centering in motion control. */
typedef struct {
    int16_t left_mm;
    int16_t right_mm;
    uint32_t timestamp_us;
    bool valid;
} tof_pair_t;

typedef struct {
    float gyro_dps[3];
    float accel_mps2[3];
    uint32_t timestamp_us;
} imu_sample_t;

/* -------------------------------------------------------------------------- */
/* Motion                                                                     */
/* -------------------------------------------------------------------------- */

typedef struct {
    int32_t left_count;
    int32_t right_count;
    float left_mps;
    float right_mps;
    uint32_t timestamp_us;
} odometry_t;

typedef struct {
    float x_mm;
    float y_mm;
    float heading_deg; /* 0 = north, increasing clockwise */
    float left_mps;
    float right_mps;
    uint8_t cell_x;
    uint8_t cell_y;
    bool moving;
    uint32_t timestamp_us;
} pose_t;

typedef enum {
    MOTION_MODE_STOP = 0,
    MOTION_MODE_STRAIGHT,
    MOTION_MODE_TURN,
} motion_mode_t;

typedef struct {
    motion_mode_t mode;
    float target_heading_deg;
    float target_distance_mm;
    float target_speed_mps;
    uint32_t command_id;
} motion_cmd_t;

/* -------------------------------------------------------------------------- */
/* System                                                                     */
/* -------------------------------------------------------------------------- */

typedef enum {
    SYS_BOOT = 0,
    SYS_CALIBRATE,
    SYS_IDLE,
    SYS_EXPLORE,
    SYS_GOAL,
    SYS_STOP,
    SYS_FAULT,
} sys_state_t;

typedef enum {
    FAULT_NONE = 0,
    FAULT_ESTOP,
    FAULT_SENSOR_TIMEOUT,
    FAULT_MOTOR,
    FAULT_IMU,
    FAULT_TOF,
    FAULT_NAV,
    FAULT_WATCHDOG,
} fault_code_t;

typedef struct {
    sys_state_t state;
    pose_t pose;
    fault_code_t fault;
    uint32_t uptime_ms;
} system_state_t;
