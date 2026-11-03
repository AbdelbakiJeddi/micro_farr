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
    uint32_t timestamp_ms;
} sensor_snapshot_t;
