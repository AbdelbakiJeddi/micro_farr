#include "motion_client.h"

#include <string.h>

#include "app_ipc.h"

#define MOTION_SEND_WAIT_MS 100

static esp_err_t send(const motion_cmd_t *cmd)
{
    return app_ipc_send_motion(cmd, pdMS_TO_TICKS(MOTION_SEND_WAIT_MS)) ? ESP_OK : ESP_ERR_TIMEOUT;
}

/* A new move: clear the events of the previous one first, so a wait sees only this move. */
static esp_err_t send_new_move(const motion_cmd_t *cmd)
{
    app_ipc_clear_event(APP_EVENT_MOVE_DONE | APP_EVENT_MOVE_FAULT);
    return send(cmd);
}

esp_err_t motion_straight(int cells, speed_profile_t profile)
{
    if (cells <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    motion_cmd_t cmd = { .type = MOVE_STRAIGHT, .cells = (int16_t)cells, .profile = profile };
    return send_new_move(&cmd);
}

/* No event clearing: the run in progress keeps its own completion event. */
esp_err_t motion_extend(int cells)
{
    motion_cmd_t cmd = { .type = MOVE_EXTEND, .cells = (int16_t)cells };
    return send(&cmd);
}

esp_err_t motion_turn(int angle_deg, speed_profile_t profile)
{
    if (angle_deg != 90 && angle_deg != -90 && angle_deg != 180) {
        return ESP_ERR_INVALID_ARG;
    }
    motion_cmd_t cmd = { .type = MOVE_TURN, .angle_deg = (int16_t)angle_deg, .profile = profile };
    return send_new_move(&cmd);
}

esp_err_t motion_stop(void)
{
    motion_cmd_t cmd = { .type = MOVE_STOP };
    return send(&cmd);
}

bool motion_wait(TickType_t timeout, move_report_t *out)
{
    EventBits_t bits = app_ipc_wait_event(APP_EVENT_MOVE_DONE | APP_EVENT_MOVE_FAULT, true, timeout);
    motion_status_t st;

    if (out != NULL) {
        memset(out, 0, sizeof(*out));
        if (bits != 0 && app_ipc_read_status(&st)) {
            *out = st.last;
        }
    }
    return (bits & APP_EVENT_MOVE_DONE) != 0;
}

bool motion_status(motion_status_t *out)
{
    return app_ipc_read_status(out);
}
