#pragma once

#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_err.h"

#include "app_types.h"

#define APP_EVENT_CALIBRATED   (1U << 0)
#define APP_EVENT_SEGMENT_DONE (1U << 1)
#define APP_EVENT_MAZE_SOLVED  (1U << 2)
#define APP_EVENT_FAULT        (1U << 3)

esp_err_t app_ipc_init(void);

bool app_ipc_publish_sensor(const sensor_snapshot_t *snapshot);
bool app_ipc_read_sensor(sensor_snapshot_t *snapshot);

bool app_ipc_publish_status(const motion_status_t *status);
bool app_ipc_read_status(motion_status_t *status);

bool app_ipc_send_motion(motion_cmd_t cmd, TickType_t wait_ticks);
bool app_ipc_recv_motion(motion_cmd_t *cmd, TickType_t wait_ticks);

void        app_ipc_set_event(EventBits_t bits);
void        app_ipc_clear_event(EventBits_t bits);
EventBits_t app_ipc_wait_event(EventBits_t bits, bool clear_on_exit, TickType_t wait_ticks);
