/* SPEED RUN: load the saved maze, plan the shortest path through visited cells and drive it
 * with the FAST profile, one command per straight run or pivot. */

#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "app_states.h"
#include "maze_store.h"
#include "motion_client.h"
#include "path.h"
#include "ui.h"

static const char *TAG = "app";

typedef struct {
    int index;
    int count;
    TickType_t last_draw;
} run_progress_t;

static esp_err_t start_move(const path_move_t *m)
{
    return (m->type == MOVE_TURN) ? motion_turn(m->angle_deg, PROFILE_FAST)
                                  : motion_straight(m->cells, PROFILE_FAST);
}

static void on_move_poll(void *arg)
{
    run_progress_t *pr  = arg;
    TickType_t      now = xTaskGetTickCount();
    char            line[OLED_COLS + 1];

    if (now - pr->last_draw < pdMS_TO_TICKS(UI_REDRAW_MS)) {
        return;
    }
    pr->last_draw = now;
    snprintf(line, sizeof(line), "move %d/%d", pr->index + 1, pr->count);
    ui_message("== SPEED RUN ==", line, "DEC stop");
}

/* Drives every move; returns NULL when done, else the failure reason. */
static const char *drive(const path_move_t *moves, int count)
{
    run_progress_t pr = { .count = count };
    move_report_t  r;

    for (pr.index = 0; pr.index < count; pr.index++) {
        if (start_move(&moves[pr.index]) != ESP_OK) {
            return "MOVE FAULT";
        }
        if (!ui_wait_move(on_move_poll, &pr, &r)) {
            return ui_result_name(r.result);
        }
    }
    return NULL;
}

app_state_id_t state_speed_run_run(app_ctx_t *ctx)
{
    static path_move_t moves[PATH_MAX_MOVES];
    char               line[32];   /* the OLED clips at OLED_COLS */

    if (maze_store_load(&ctx->maze) != ESP_OK) {
        ui_message("== SPEED RUN ==", "No maze", "any button: menu");
        ui_wait_any();
        return ST_MENU;
    }
    ctx->maze_valid = true;

    int count = path_plan(&ctx->maze, MAZE_START_X, MAZE_START_Y, (maze_dir_t)MAZE_START_DIR,
                          moves, PATH_MAX_MOVES);
    if (count < 0) {
        ctx->fault_reason = "NO PATH";
        return ST_FAULT;
    }

    snprintf(line, sizeof(line), "RUN %d moves", count);
    if (!ui_arm_and_wait(line)) {
        return ST_MENU;
    }

    TickType_t  start = xTaskGetTickCount();
    const char *fail  = drive(moves, count);
    if (fail != NULL) {
        ctx->fault_reason = fail;
        return ST_FAULT;
    }

    float seconds = (float)((xTaskGetTickCount() - start) * portTICK_PERIOD_MS) / 1000.0f;
    ESP_LOGI(TAG, "speed run done: %d moves, %.2f s", count, seconds);
    snprintf(line, sizeof(line), "%.2f s", seconds);
    ui_message("== SPEED RUN ==", "DONE", line);
    ui_wait_any();
    return ST_MENU;
}
