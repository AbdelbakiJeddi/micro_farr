#pragma once

/* -------------------------------------------------------------------------- */
/* Timing                                                                     */
/* -------------------------------------------------------------------------- */

#define APP_TICK_HZ              1000

#define SENSOR_PERIOD_MS         10
#define MOTION_PERIOD_MS         10
#define SUPERVISOR_PERIOD_MS     20
#define NAV_EVENT_TIMEOUT_MS     50
#define OLED_REFRESH_MS          200

#define HEARTBEAT_TIMEOUT_MS     100

/* Maze geometry (TODO: measure). */
#define CELL_SIZE_MM             180.0f

/* Initial phase offsets, in ms. */
#define SENSOR_OFFSET_MS         0
#define SUPERVISOR_OFFSET_MS     5

/* -------------------------------------------------------------------------- */
/* Priorities (ESP-IDF: 0..24)                                                */
/* -------------------------------------------------------------------------- */

#define MOTION_PRIO              20
#define SENSOR_PRIO              15
#define SUPERVISOR_PRIO          10
#define NAV_PRIO                 5

/* -------------------------------------------------------------------------- */
/* Core affinity                                                              */
/* -------------------------------------------------------------------------- */

#define MOTION_CORE              1
#define IO_CORE                  0

/* -------------------------------------------------------------------------- */
/* Stacks (bytes, ESP-IDF convention)                                         */
/* -------------------------------------------------------------------------- */

#define SENSOR_STACK_BYTES       4096
#define MOTION_STACK_BYTES       4096
#define NAV_STACK_BYTES          6144
#define SUPERVISOR_STACK_BYTES   4096

/* -------------------------------------------------------------------------- */
/* Queue depths                                                               */
/* -------------------------------------------------------------------------- */

#define APP_LATEST_DEPTH         1
#define MOTION_CMD_DEPTH         4
