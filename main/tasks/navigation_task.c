#include "navigation_task.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "app_ipc.h"
#include "button.h"
#include "maze.h"
#include "motion_control_task.h"

static const char *TAG = "nav";

typedef struct {
    maze_t     maze;
    int        x;
    int        y;
    maze_dir_t dir;
} nav_state_t;

static bool wall_seen(int32_t tof_mm, int32_t limit_mm)
{
    return tof_mm >= 0 && tof_mm < limit_mm; /* invalid or far reading = open */
}

static bool at_goal(int x, int y)
{
    return x == MAZE_GOAL_X && y == MAZE_GOAL_Y;
}

/* Wait for the robot to settle, then fetch a fresh sensor snapshot. */
static bool read_sensors_settled(sensor_snapshot_t *snapshot)
{
    vTaskDelay(pdMS_TO_TICKS(NAV_SENSE_SETTLE_MS));
    return app_ipc_read_sensor(snapshot);
}

/* Record front/left/right walls of cell (x,y), seen while heading dir. */
static void record_walls(maze_t *m, int x, int y, maze_dir_t dir,
                         const sensor_snapshot_t *s, int32_t front_limit_mm)
{
    maze_set_wall(m, x, y, dir, wall_seen(s->front_tof, front_limit_mm));
    maze_set_wall(m, x, y, maze_turn_dir(dir, -1), wall_seen(s->left_tof, TOF_WALL_MAX_MM));
    maze_set_wall(m, x, y, maze_turn_dir(dir, 1), wall_seen(s->right_tof, TOF_WALL_MAX_MM));
}

static bool wait_motion_done(void)
{
    EventBits_t bits = app_ipc_wait_event(APP_EVENT_SEGMENT_DONE | APP_EVENT_FAULT, true,
                                          pdMS_TO_TICKS(NAV_MOVE_WAIT_MS));
    return (bits & APP_EVENT_SEGMENT_DONE) != 0;
}

static bool quarter_turn(bool left)
{
    esp_err_t err = left ? turn_left() : turn_right();
    return err == ESP_OK && wait_motion_done();
}

/* Rotate in place until the robot faces target. */
static bool face(nav_state_t *n, maze_dir_t target)
{
    int right_turns = ((int)target - (int)n->dir + 4) & 3;

    if (right_turns == 3) {
        if (!quarter_turn(true)) {
            return false;
        }
    } else {
        for (int i = 0; i < right_turns; i++) {
            if (!quarter_turn(false)) {
                return false;
            }
        }
    }
    n->dir = target;
    return true;
}

/* Cell k steps ahead of the robot along its heading. */
static void cell_ahead(const nav_state_t *n, int k, int *x, int *y)
{
    *x = n->x;
    *y = n->y;
    for (int i = 0; i < k; i++) {
        maze_step(n->dir, x, y);
    }
}

/* Ticks of run progress at which the walls of the k-th cell ahead are read and decided. */
static int32_t decide_ticks(int k)
{
    return (int32_t)(((float)(k - 1) + NAV_DECIDE_FRAC) * (float)CELL_TICKS);
}

/* While moving: read the walls of the k-th cell ahead and decide whether to carry on
 * straight through it. Returns true if the run should be extended by one more cell. */
static bool decide_continue(nav_state_t *n, int k)
{
    sensor_snapshot_t s;
    int x, y;

    if (!app_ipc_read_sensor(&s)) {
        return false;
    }
    cell_ahead(n, k, &x, &y);
    record_walls(&n->maze, x, y, n->dir, &s, NAV_RUN_FRONT_WALL_MM);
    maze_flood(&n->maze, MAZE_GOAL_X, MAZE_GOAL_Y);

    ESP_LOGI(TAG, "pass (%d,%d) L=%ld F=%ld R=%ld", x, y,
             (long)s.left_tof, (long)s.front_tof, (long)s.right_tof);

    if (at_goal(x, y)) {
        return false;
    }
    return maze_next_dir(&n->maze, x, y, n->dir) == (int)n->dir;
}

/* Cells actually covered by a finished run, from the measured distance. */
static int cells_travelled(int planned)
{
    motion_status_t st;
    int cells = planned;

    if (app_ipc_read_status(&st)) {
        cells = (st.progress_ticks + CELL_TICKS / 2) / CELL_TICKS;
    }
    return cells < planned ? cells : planned;
}

/* Drive straight from the centre of this cell. Keeps extending the run, without
 * stopping, while the cells ahead are straight through; stops at the centre of the
 * first cell where the path turns (or at the goal). Updates the robot's cell. */
static bool run(nav_state_t *n)
{
    int planned = 1;   /* cells in the run so far */
    int decided = 0;   /* cells ahead whose walls have been decided */

    if (straight_one_cell() != ESP_OK) {
        return false;
    }

    while (1) {
        EventBits_t bits = app_ipc_wait_event(APP_EVENT_SEGMENT_DONE | APP_EVENT_FAULT, true,
                                              pdMS_TO_TICKS(NAV_POLL_MS));
        if (bits & APP_EVENT_FAULT) {
            return false;
        }
        if (bits & APP_EVENT_SEGMENT_DONE) {
            break;
        }

        motion_status_t st;
        int next = decided + 1;
        if (next <= planned && app_ipc_read_status(&st) &&
            st.progress_ticks >= decide_ticks(next)) {
            if (decide_continue(n, next) && straight_extend_cell() == ESP_OK) {
                planned++;
            }
            decided = next;
        }
    }

    int cells = cells_travelled(planned);
    cell_ahead(n, cells, &n->x, &n->y);
    return true;
}

static void nav_reset(nav_state_t *n)
{
    maze_init(&n->maze, MAZE_W, MAZE_H);
    n->x   = MAZE_START_X;
    n->y   = MAZE_START_Y;
    n->dir = (maze_dir_t)MAZE_START_DIR;
}

static void log_cell(const nav_state_t *n, const sensor_snapshot_t *s)
{
    ESP_LOGI(TAG, "cell (%d,%d) dir %d dist %d | L=%ld F=%ld R=%ld",
             n->x, n->y, (int)n->dir, maze_dist(&n->maze, n->x, n->y),
             (long)s->left_tof, (long)s->front_tof, (long)s->right_tof);
}

/* Standing at a cell centre: sense, plan, turn toward the best cell and drive.
 * If a wall shows up ahead after turning, record it and return without moving so the
 * next call re-plans. */
static bool explore_step(nav_state_t *n)
{
    sensor_snapshot_t s;

    if (!read_sensors_settled(&s)) {
        return false;
    }
    record_walls(&n->maze, n->x, n->y, n->dir, &s, NAV_FRONT_WALL_MM);
    maze_flood(&n->maze, MAZE_GOAL_X, MAZE_GOAL_Y);
    log_cell(n, &s);

    if (at_goal(n->x, n->y)) {
        return true;
    }

    int best = maze_next_dir(&n->maze, n->x, n->y, n->dir);
    if (best < 0) {
        ESP_LOGE(TAG, "no path to goal");
        return false;
    }
    if (!face(n, (maze_dir_t)best) || !read_sensors_settled(&s)) {
        return false;
    }
    if (wall_seen(s.front_tof, NAV_FRONT_WALL_MM)) {
        ESP_LOGW(TAG, "wall ahead after turn, re-planning");
        maze_set_wall(&n->maze, n->x, n->y, n->dir, true);
        return true;
    }

    return run(n);
}

static bool solve_maze(nav_state_t *n)
{
    for (int step = 0; step < NAV_MAX_STEPS; step++) {
        if (!explore_step(n)) {
            return false;
        }
        if (at_goal(n->x, n->y)) {
            return true;
        }
    }
    ESP_LOGE(TAG, "step limit reached");
    return false;
}

static void navigation_task(void *arg)
{
    (void)arg;
    nav_state_t nav;

    esp_log_level_set(TAG, ESP_LOG_INFO);
    app_ipc_wait_event(APP_EVENT_CALIBRATED, false, portMAX_DELAY);
    button_init();

    while (1) {
        nav_reset(&nav);
        ESP_LOGI(TAG, "ready at (%d,%d): press a button to start", nav.x, nav.y);
        button_wait_press(NAV_START_DELAY_MS);

        ESP_LOGI(TAG, "start: goal (%d,%d) in %dx%d maze",
                 MAZE_GOAL_X, MAZE_GOAL_Y, MAZE_W, MAZE_H);

        if (solve_maze(&nav)) {
            ESP_LOGI(TAG, "GOAL reached at (%d,%d)", nav.x, nav.y);
        } else {
            ESP_LOGE(TAG, "run failed at (%d,%d)", nav.x, nav.y);
            app_ipc_set_event(APP_EVENT_FAULT);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void navigation_task_start(void)
{
    BaseType_t ok = xTaskCreatePinnedToCore(navigation_task, "nav",
                                            NAV_TASK_STACK, NULL,
                                            NAV_TASK_PRIO, NULL,
                                            NAV_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "nav task create failed");
    }
}
