#pragma once

/* Types shared between tasks through app_ipc. No ESP-IDF headers: host tests include it. */

#include <stdint.h>

typedef enum {
    MOVE_STRAIGHT,
    MOVE_EXTEND,   /* add cells to the straight run in progress */
    MOVE_TURN,
    MOVE_STOP,
    MOVE_TYPE_COUNT
} move_type_t;

typedef enum {
    PROFILE_EXPLORE,
    PROFILE_FAST,
    PROFILE_COUNT
} speed_profile_t;

typedef struct {
    move_type_t     type;
    int16_t         cells;      /* MOVE_STRAIGHT: cells to drive; MOVE_EXTEND: cells to add */
    int16_t         angle_deg;  /* MOVE_TURN: +90 left, -90 right, 180 */
    speed_profile_t profile;
} motion_cmd_t;

typedef enum {
    RESULT_NONE,
    RESULT_DONE,
    RESULT_TIMEOUT,
    RESULT_WALL_STOP,
    RESULT_SENSOR_LOST,
    RESULT_STOPPED,
    RESULT_COUNT
} move_result_t;

typedef struct {
    move_result_t result;
    int32_t       distance_ticks;   /* straight: ticks covered */
    float         heading_err_deg;  /* final heading vs reference */
    float         lat_err_end_mm;   /* straight: last lateral error (0 if no walls) */
    float         lat_err_max_mm;   /* straight: worst |lateral error| */
    float         peak_speed_mm_s;
    uint32_t      duration_ms;
} move_report_t;

typedef struct {
    int32_t  left_tof;    /* mm, -1 = invalid */
    int32_t  right_tof;
    int32_t  front_tof;
    float    yaw;
    float    yaw_rate_dps;
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
    move_report_t  last;   /* filled when a move ends */
} motion_status_t;
