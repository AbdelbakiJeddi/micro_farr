#include "telemetry_task.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "app_ipc.h"
#include "app_types.h"
#include "oled.h"

static const char *state_text(motion_state_t state)
{
    switch (state) {
    case MOTION_RUNNING:
        return "RUN";
    case MOTION_DONE:
        return "DONE";
    case MOTION_FAULT:
        return "FAULT";
    case MOTION_IDLE:
    default:
        return "IDLE";
    }
}

static void render_frame(bool have_status, const motion_status_t *status,
                         bool have_snapshot, const sensor_snapshot_t *snapshot)
{
    char line[22];

    oled_text_line(0, have_status ? state_text(status->state) : "WAIT");

    if (have_status) {
        snprintf(line, sizeof(line), "P %04ld/%04ld",
                 (long)status->progress_ticks, (long)status->target_ticks);
    } else {
        snprintf(line, sizeof(line), "P ----/----");
    }
    oled_text_line(1, line);

    if (have_snapshot) {
        snprintf(line, sizeof(line), "Y %5.1f", snapshot->yaw);
    } else {
        snprintf(line, sizeof(line), "Y  --.-");
    }
    oled_text_line(2, line);

    if (have_status && status->state == MOTION_FAULT && status->note[0] != '\0') {
        oled_text_line(3, status->note);
    } else if (have_snapshot) {
        snprintf(line, sizeof(line), "L%ld R%ld",
                 (long)snapshot->left_ticks, (long)snapshot->right_ticks);
        oled_text_line(3, line);
    } else {
        oled_text_line(3, "L--- R---");
    }

    oled_flush();
}

static void telemetry_task(void *arg)
{
    (void)arg;

    const TickType_t period = pdMS_TO_TICKS(1000 / TELEMETRY_TASK_HZ);
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        motion_status_t status;
        sensor_snapshot_t snapshot;

        bool have_status = app_ipc_read_status(&status);
        bool have_snapshot = app_ipc_read_sensor(&snapshot);

        render_frame(have_status, &status, have_snapshot, &snapshot);

        vTaskDelayUntil(&last_wake, period);
    }
}

void telemetry_task_start(void)
{
    if (oled_init() != ESP_OK) {
        return;
    }

    xTaskCreatePinnedToCore(telemetry_task, "telemetry",
                            TELEMETRY_TASK_STACK, NULL,
                            TELEMETRY_TASK_PRIO, NULL,
                            TELEMETRY_TASK_CORE);
}
