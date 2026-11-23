/* EXPLORE: from the start cell to the goal, mapping walls on the way. Standing at a cell
 * centre: sense, flood fill, face the best neighbour and run. While running, the walls of
 * each cell ahead are decided at NAV_DECIDE_FRAC of the way and the run is extended without
 * stopping while the path goes straight on. At the goal the map is saved to flash. */

#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "app_ipc.h"
#include "app_states.h"
#include "maze_store.h"
#include "motion_client.h"
#include "params.h"
#include "ui.h"

static const char *TAG = "app";

typedef struct {
    maze_t     *maze;
    int         x;
    int         y;
    maze_dir_t  dir;
    const char *fail;        /* reason, set when a step fails */
    TickType_t  last_draw;
} explore_t;

static bool wall_seen(int32_t tof_mm, int32_t limit_mm)
{
    return tof_mm >= 0 && tof_mm < limit_mm; /* invalid or far reading = open */
}

static void draw_status(explore_t *e)
{
    static const char DIR_NAME[4] = { 'N', 'E', 'S', 'W' };
    TickType_t        now         = xTaskGetTickCount();
    char              line[OLED_COLS + 1];

    if (now - e->last_draw < pdMS_TO_TICKS(UI_REDRAW_MS)) {
        return;
    }
    e->last_draw = now;
    snprintf(line, sizeof(line), "cell %d,%d %c", e->x, e->y, DIR_NAME[e->dir]);
    ui_message("== EXPLORE ==", line, "DEC stop");
}

/* Wait for the robot to settle, then fetch a fresh sensor snapshot. */
static bool read_sensors_settled(explore_t *e, sensor_snapshot_t *snapshot)
{
    vTaskDelay(pdMS_TO_TICKS(NAV_SENSE_SETTLE_MS));
    if (!app_ipc_read_sensor(snapshot)) {
        e->fail = "NO SENSOR";
        return false;
    }
    return true;
}

/* Record front/left/right walls of cell (x,y), seen while heading dir. */
static void record_walls(maze_t *m, int x, int y, maze_dir_t dir,
                         const sensor_snapshot_t *s, int32_t front_limit_mm)
{
    maze_set_wall(m, x, y, dir, wall_seen(s->front_tof, front_limit_mm));
    maze_set_wall(m, x, y, maze_turn_dir(dir, -1), wall_seen(s->left_tof, TOF_WALL_MAX_MM));
    maze_set_wall(m, x, y, maze_turn_dir(dir, 1), wall_seen(s->right_tof, TOF_WALL_MAX_MM));
}

/* One poll while a move runs. Returns 1 = move done, -1 = failed (reason set), 0 = running.
 * DEC stops the robot. */
static int poll_move(explore_t *e)
{
    move_report_t r;

    if (!ui_run_guard()) {
        e->fail = "STOPPED";
        return -1;
    }
    if (motion_wait(pdMS_TO_TICKS(NAV_POLL_MS), &r)) {
        return 1;
    }
    if (r.result != RESULT_NONE) {
        e->fail = (r.result == RESULT_STOPPED) ? "STOPPED" : "MOVE FAULT";
        return -1;
    }
    draw_status(e);
    return 0;
}

static bool wait_move(explore_t *e)
{
    int state;

    while ((state = poll_move(e)) == 0) {
    }
    return state > 0;
}

/* Rotate in place until the robot faces target: one pivot of -90, 180 or +90. */
static bool face(explore_t *e, maze_dir_t target)
{
    static const int16_t ANGLE[4] = { 0, -90, 180, 90 };
    int right_turns = ((int)target - (int)e->dir + 4) & 3;

    if (right_turns != 0) {
        if (motion_turn(ANGLE[right_turns], PROFILE_EXPLORE) != ESP_OK) {
            e->fail = "MOVE FAULT";
            return false;
        }
        if (!wait_move(e)) {
            return false;
        }
    }
    e->dir = target;
    return true;
}

/* Cell k steps ahead of the robot along its heading. */
static void cell_ahead(const explore_t *e, int k, int *x, int *y)
{
    *x = e->x;
    *y = e->y;
    for (int i = 0; i < k; i++) {
        maze_step(e->dir, x, y);
    }
}

/* Ticks of run progress at which the walls of the k-th cell ahead are read and decided. */
static int32_t decide_ticks(int k)
{
    return (int32_t)(((float)(k - 1) + NAV_DECIDE_FRAC) * params_get(P_TICKS_PER_CELL));
}

/* While moving: read the walls of the k-th cell ahead and decide whether to carry on
 * straight through it. Returns true if the run should be extended by one more cell. */
static bool decide_continue(explore_t *e, int k)
{
    sensor_snapshot_t s;
    int x, y;

    if (!app_ipc_read_sensor(&s)) {
        return false;
    }
    cell_ahead(e, k, &x, &y);
    record_walls(e->maze, x, y, e->dir, &s, NAV_RUN_FRONT_WALL_MM);
    maze_flood(e->maze);

    ESP_LOGI(TAG, "pass (%d,%d) L=%ld F=%ld R=%ld", x, y,
             (long)s.left_tof, (long)s.front_tof, (long)s.right_tof);

    if (maze_is_goal(e->maze, x, y)) {
        return false;
    }
    return maze_next_dir(e->maze, x, y, e->dir) == (int)e->dir;
}

/* Cells actually covered by a finished run, from the measured distance. */
static int cells_travelled(int planned)
{
    motion_status_t st;
    int cells = planned;

    if (motion_status(&st)) {
        int32_t cell = (int32_t)params_get(P_TICKS_PER_CELL);
        cells = (st.last.distance_ticks + cell / 2) / cell;
    }
    return cells < planned ? cells : planned;
}

/* Extend the run when the progress reaches the decide point of the next cell ahead. */
static void decide_ahead(explore_t *e, int *planned, int *decided)
{
    motion_status_t st;
    int next = *decided + 1;

    if (next <= *planned && motion_status(&st) && st.progress_ticks >= decide_ticks(next)) {
        if (decide_continue(e, next) && motion_extend(1) == ESP_OK) {
            (*planned)++;
        }
        *decided = next;
    }
}

/* Drive straight from the centre of this cell. Keeps extending the run, without
 * stopping, while the cells ahead are straight through; stops at the centre of the
 * first cell where the path turns (or at the goal). Updates and marks the robot's cells. */
static bool run(explore_t *e)
{
    int planned = 1;   /* cells in the run so far */
    int decided = 0;   /* cells ahead whose walls have been decided */
    int state;

    if (motion_straight(1, PROFILE_EXPLORE) != ESP_OK) {
        e->fail = "MOVE FAULT";
        return false;
    }
    while ((state = poll_move(e)) == 0) {
        decide_ahead(e, &planned, &decided);
    }
    if (state < 0) {
        return false;
    }

    int cells = cells_travelled(planned);
    for (int i = 0; i < cells; i++) {
        maze_step(e->dir, &e->x, &e->y);
        maze_mark_visited(e->maze, e->x, e->y);
    }
    return true;
}

static void log_cell(const explore_t *e, const sensor_snapshot_t *s)
{
    ESP_LOGI(TAG, "cell (%d,%d) dir %d dist %d | L=%ld F=%ld R=%ld",
             e->x, e->y, (int)e->dir, maze_dist(e->maze, e->x, e->y),
             (long)s->left_tof, (long)s->front_tof, (long)s->right_tof);
}

/* Standing at a cell centre: sense, plan, turn toward the best cell and drive.
 * If a wall shows up ahead after turning, record it and return without moving so the
 * next call re-plans. */
static bool explore_step(explore_t *e)
{
    sensor_snapshot_t s;

    if (!read_sensors_settled(e, &s)) {
        return false;
    }
    record_walls(e->maze, e->x, e->y, e->dir, &s, NAV_FRONT_WALL_MM);
    maze_flood(e->maze);
    log_cell(e, &s);

    if (maze_is_goal(e->maze, e->x, e->y)) {
        return true;
    }

    int best = maze_next_dir(e->maze, e->x, e->y, e->dir);
    if (best < 0) {
        e->fail = "NO PATH";
        return false;
    }
    if (!face(e, (maze_dir_t)best) || !read_sensors_settled(e, &s)) {
        return false;
    }
    if (wall_seen(s.front_tof, NAV_FRONT_WALL_MM)) {
        ESP_LOGW(TAG, "wall ahead after turn, re-planning");
        maze_set_wall(e->maze, e->x, e->y, e->dir, true);
        return true;
    }
    return run(e);
}

static bool solve_maze(explore_t *e)
{
    for (int step = 0; step < NAV_MAX_STEPS; step++) {
        if (!explore_step(e)) {
            return false;
        }
        if (maze_is_goal(e->maze, e->x, e->y)) {
            return true;
        }
    }
    e->fail = "STEP LIMIT";
    return false;
}

app_state_id_t state_explore_run(app_ctx_t *ctx)
{
    if (!ui_arm_and_wait("== EXPLORE ==")) {
        return ST_MENU;
    }

    explore_t e = {
        .maze = &ctx->maze,
        .x    = MAZE_START_X,
        .y    = MAZE_START_Y,
        .dir  = (maze_dir_t)MAZE_START_DIR,
    };
    maze_init_configured(&ctx->maze);
    maze_mark_visited(&ctx->maze, e.x, e.y);
    ctx->maze_valid = false;

    if (!solve_maze(&e)) {
        ESP_LOGE(TAG, "explore failed at (%d,%d): %s", e.x, e.y, e.fail);
        ctx->fault_reason = e.fail;
        return ST_FAULT;
    }

    ctx->maze_valid = true;
    esp_err_t err = maze_store_save(&ctx->maze);
    ESP_LOGI(TAG, "goal at (%d,%d), save: %s", e.x, e.y, esp_err_to_name(err));
    ui_message("== EXPLORE ==", "GOAL", err == ESP_OK ? "saved" : "SAVE FAILED");
    ui_wait_any();
    return ST_MENU;
}
