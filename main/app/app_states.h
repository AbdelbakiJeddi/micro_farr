#pragma once

/* Robot FSM states (app-internal). Each state runs until it decides the next one. */

#include <stdbool.h>

#include "maze.h"

typedef enum {
    ST_MENU,
    ST_EXPLORE,
    ST_SPEED_RUN,
    ST_TUNER,
    ST_FAULT,
    ST_COUNT
} app_state_id_t;

typedef struct {
    maze_t      maze;           /* RAM map, kept across states */
    bool        maze_valid;
    const char *fault_reason;   /* set before -> ST_FAULT */
} app_ctx_t;

typedef app_state_id_t (*state_run_fn)(app_ctx_t *ctx);

app_state_id_t state_menu_run(app_ctx_t *ctx);
app_state_id_t state_explore_run(app_ctx_t *ctx);
app_state_id_t state_speed_run_run(app_ctx_t *ctx);
app_state_id_t state_tuner_run(app_ctx_t *ctx);
app_state_id_t state_fault_run(app_ctx_t *ctx);
