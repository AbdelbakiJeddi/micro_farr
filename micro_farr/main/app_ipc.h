#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_err.h"

#include "app_types.h"

/* -------------------------------------------------------------------------- */
/* Heartbeats                                                                 */
/* -------------------------------------------------------------------------- */

/* Each task increments only its own counter; supervisor reads all of them. */
typedef struct {
    volatile uint32_t sensor;
    volatile uint32_t motion;
    volatile uint32_t navigation;
} heartbeat_counters_t;

extern heartbeat_counters_t g_heartbeats;

/* -------------------------------------------------------------------------- */
/* Queues (latest-value unless stated otherwise)                              */
/* -------------------------------------------------------------------------- */

extern QueueHandle_t g_imu_q;            /* sensor -> motion                       */
extern QueueHandle_t g_tof_motion_q;     /* sensor -> motion (left/right)          */
extern QueueHandle_t g_tof_nav_q;        /* sensor -> navigation (all three)       */
extern QueueHandle_t g_pose_q;           /* motion -> navigation                   */
extern QueueHandle_t g_motion_cmd_q;     /* navigation -> motion (depth > 1)       */
extern QueueHandle_t g_system_state_q;   /* navigation/motion -> supervisor (OLED) */

/* -------------------------------------------------------------------------- */
/* Shared resources and synchronisation                                       */
/* -------------------------------------------------------------------------- */

extern SemaphoreHandle_t g_i2c_mutex;    /* sensor (ToF/IMU) <-> supervisor (OLED) */
extern EventGroupHandle_t g_system_events;

#define EVT_ESTOP_LATCHED BIT0
#define EVT_FAULT_LATCHED BIT1

/* Task notification bits delivered to the motion task. */
#define NOTIFY_ESTOP      BIT0
#define NOTIFY_RESET      BIT1

extern TaskHandle_t g_motion_task_handle;

/* -------------------------------------------------------------------------- */

esp_err_t app_ipc_init(void);
