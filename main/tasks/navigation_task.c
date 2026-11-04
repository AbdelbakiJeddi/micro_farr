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

/* Wait for the robot to settle, then fetch a fresh sensor snapshot. */
static bool read_sensors(sensor_snapshot_t *snapshot)
{
    vTaskDelay(pdMS_TO_TICKS(NAV_SENSE_SETTLE_MS));
    return app_ipc_read_sensor(snapshot);
}

static bool front_blocked(const sensor_snapshot_t *s)
{
    return wall_seen(s->front_tof, NAV_FRONT_WALL_MM);
}

/* Record front/left/right walls of the current cell relative to the heading. */
static void record_walls(nav_state_t *n, const sensor_snapshot_t *s)
{
    maze_set_wall(&n->maze, n->x, n->y, n->dir, front_blocked(s));
    maze_set_wall(&n->maze, n->x, n->y, maze_turn_dir(n->dir, -1),
                  wall_seen(s->left_tof, TOF_WALL_MAX_MM));
    maze_set_wall(&n->maze, n->x, n->y, maze_turn_dir(n->dir, 1),
                  wall_seen(s->right_tof, TOF_WALL_MAX_MM));
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

static bool advance(nav_state_t *n)
{
    if (straight_one_cell() != ESP_OK || !wait_motion_done()) {
        return false;
    }
    maze_step(n->dir, &n->x, &n->y);
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

static bool at_goal(const nav_state_t *n)
{
    return n->x == MAZE_GOAL_X && n->y == MAZE_GOAL_Y;
}

/* One exploration step: sense, plan, turn, drive. Returns false on failure.
 * If a wall shows up ahead after turning, it is recorded and the step ends without
 * moving, so the next call re-plans. */
static bool explore_step(nav_state_t *n)
{
    sensor_snapshot_t s;

    if (!read_sensors(&s)) {
        return false;
    }
    record_walls(n, &s);
    maze_flood(&n->maze, MAZE_GOAL_X, MAZE_GOAL_Y);
    log_cell(n, &s);

    if (at_goal(n)) {
        return true;
    }

    int best = maze_next_dir(&n->maze, n->x, n->y, n->dir);
    if (best < 0) {
        ESP_LOGE(TAG, "no path to goal");
        return false;
    }
    if (!face(n, (maze_dir_t)best)) {
        return false;
    }

    /* Re-check the wall now that we face it; if it is blocked, plan again. */
    if (!read_sensors(&s)) {
        return false;
    }
    if (front_blocked(&s)) {
        ESP_LOGW(TAG, "wall ahead after turn, re-planning");
        maze_set_wall(&n->maze, n->x, n->y, n->dir, true);
        return true;
    }

    return advance(n);
}

static bool solve_maze(nav_state_t *n)
{
    for (int step = 0; step < NAV_MAX_STEPS; step++) {
        if (!explore_step(n)) {
            return false;
        }
        if (at_goal(n)) {
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
