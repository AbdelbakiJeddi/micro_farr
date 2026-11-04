#pragma once

#include <stdint.h>

typedef enum {
    ROBOT_BOOT,
    ROBOT_CALIBRATE,
    ROBOT_EXPLORE,
    ROBOT_SOLVE,
    ROBOT_STOP,
    ROBOT_FAULT,
} robot_state_t;

typedef enum {
    CMD_STRAIGHT_CELL,
    CMD_TURN_LEFT,
    CMD_TURN_RIGHT,
    CMD_STOP,
} motion_cmd_t;

typedef struct {
    int32_t  left_tof;
    int32_t  right_tof;
    int32_t  front_tof;
    float    yaw;
    int32_t  left_ticks;
    int32_t  right_ticks;
    uint32_t timestamp_ms;
} sensor_snapshot_t;

typedef enum {
    MOTION_IDLE,
    MOTION_RUNNING,
    MOTION_DONE,
    MOTION_FAULT,
} motion_state_t;

typedef struct {
    motion_state_t state;
    int32_t        progress_ticks;
    int32_t        target_ticks;
    char           note[20];
} motion_status_t;
