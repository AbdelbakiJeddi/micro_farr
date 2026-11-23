#include "motion_task.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "app_ipc.h"
#include "app_types.h"
#include "motion_profile.h"
#include "motor.h"
#include "params.h"
#include "steering.h"
#include "wheel_ctrl.h"

static const char *TAG = "motion";

/* One kind of move: started from a command on a fresh snapshot, then stepped once per
 * snapshot until it calls end_move(). */
typedef struct {
    void (*begin)(const motion_cmd_t *cmd, const sensor_snapshot_t *snap);
    void (*update)(const sensor_snapshot_t *snap, float dt);
} move_ops_t;

typedef void (*cmd_handler_t)(const motion_cmd_t *cmd);

typedef struct {
    param_id_t cruise, accel, decel;
} profile_params_t;

typedef struct {
    int32_t         start_left;
    int32_t         start_right;
    int32_t         target_ticks;
    float           v_ref;   /* body speed reference (ticks/s) */
    speed_profile_t profile;
} straight_state_t;

typedef enum {
    TURN_DRIVE,
    TURN_SETTLE,
} turn_phase_t;

typedef struct {
    turn_phase_t phase;
    float        angle_deg;  /* |commanded angle| */
    float        half_deg;   /* half the commanded heading change: centre of the error window */
    float        rate;       /* |yaw-rate reference| (deg/s) from the angle profile */
    uint32_t     settle_ms;
} turn_state_t;

static const profile_params_t s_profiles[PROFILE_COUNT] = {
    [PROFILE_EXPLORE] = { P_EXP_CRUISE,  P_EXP_ACCEL,  P_EXP_DECEL  },
    [PROFILE_FAST]    = { P_FAST_CRUISE, P_FAST_ACCEL, P_FAST_DECEL },
};

static const move_ops_t *s_active;        /* NULL = idle */
static motion_cmd_t      s_pending;
static bool              s_has_pending;

static straight_state_t s_straight;
static turn_state_t     s_turn;
static wheel_ctrl_t     s_wheel_left;
static wheel_ctrl_t     s_wheel_right;
static steer_t          s_steer;          /* heading reference carried across moves */
static move_report_t    s_report;
static uint32_t         s_move_start_ms;
static uint32_t         s_last_ms;        /* timestamp of the last snapshot used */

static bool     s_heading_valid;
static uint32_t s_last_end_ms;

static motion_status_t s_status = { .state = MOTION_IDLE };

static void publish_status(void)
{
    app_ipc_publish_status(&s_status);
}

static void motors_brake(void)
{
    motor_brake(&motor_left);
    motor_brake(&motor_right);
}

/* Re-capture the heading reference if it is stale (robot may have been moved by hand). */
static void sync_heading_ref(const sensor_snapshot_t *snap)
{
    if (!s_heading_valid || snap->timestamp_ms - s_last_end_ms > HEADING_REF_TIMEOUT_MS) {
        s_steer.heading_ref = snap->yaw;
        s_heading_valid     = true;
    }
}

/* Control step from snapshot timestamps, clamped so a late or repeated sample stays sane. */
static float step_dt(uint32_t now_ms)
{
    float dt = (float)(now_ms - s_last_ms) / 1000.0f;

    s_last_ms = now_ms;
    return fminf(fmaxf(dt, MOTION_DT_MIN_S), MOTION_DT_MAX_S);
}

/* ---- move lifecycle ---- */

/* Stop the motors, publish the report and raise the matching event. */
static void end_move(move_result_t result)
{
    bool normal = (result == RESULT_DONE || result == RESULT_WALL_STOP);

    motors_brake();
    s_active      = NULL;
    s_last_end_ms = s_last_ms;

    s_report.result      = result;
    s_report.duration_ms = s_last_ms - s_move_start_ms;

    if (normal) {
        s_status.state = MOTION_DONE;
    } else {
        s_status.state = (result == RESULT_STOPPED) ? MOTION_IDLE : MOTION_FAULT;
    }
    s_status.last = s_report;
    publish_status();

    app_ipc_set_event(normal ? APP_EVENT_MOVE_DONE : APP_EVENT_MOVE_FAULT);
}

/* ---- wheels ---- */

static int round_duty(float duty)
{
    return (int)(duty >= 0.0f ? (duty + 0.5f) : (duty - 0.5f));
}

/* Body speed v and half wheel-speed difference (ticks/s) through both wheel loops. */
static void drive_wheels(float v_body, float half_diff, const sensor_snapshot_t *snap, float dt)
{
    float left  = wheel_duty(&s_wheel_left, v_body - half_diff, snap->left_ticks,
                             snap->timestamp_ms, dt);
    float right = wheel_duty(&s_wheel_right, v_body + half_diff, snap->right_ticks,
                             snap->timestamp_ms, dt);

    motor_set_speed(&motor_left, round_duty(left));
    motor_set_speed(&motor_right, round_duty(right));
}

/* ---- straight ---- */

/* Distance trapezoid on the remaining ticks, from the selected speed profile.
 * Extending the target moves the braking point further away. */
static float next_speed_ref(int32_t remaining_ticks, float dt)
{
    const profile_params_t *p   = &s_profiles[s_straight.profile];
    float                   tpm = params_ticks_per_mm();
    profile_limits_t        lim = {
        .v_max = params_get(p->cruise) * tpm,
        .accel = params_get(p->accel) * tpm,
        .decel = params_get(p->decel) * tpm,
        .v_min = RUN_MIN_MM_S * tpm,
    };

    s_straight.v_ref = profile_next(s_straight.v_ref, (float)remaining_ticks, dt, &lim);
    return s_straight.v_ref;
}

static void begin_straight(const motion_cmd_t *cmd, const sensor_snapshot_t *snap)
{
    int cells = cmd->cells > 0 ? cmd->cells : 1;

    wheel_reset(&s_wheel_left, snap->left_ticks, snap->timestamp_ms, 1.0f);
    wheel_reset(&s_wheel_right, snap->right_ticks, snap->timestamp_ms, 1.0f);

    s_straight.start_left   = snap->left_ticks;
    s_straight.start_right  = snap->right_ticks;
    s_straight.target_ticks = (int32_t)((float)cells * params_get(P_TICKS_PER_CELL));
    s_straight.v_ref        = RUN_MIN_MM_S * params_ticks_per_mm();
    s_straight.profile      = cmd->profile;

    s_status.target_ticks = s_straight.target_ticks;
}

/* Fill the straight part of the report from the current sample. */
static void straight_report(const sensor_snapshot_t *snap, int32_t progress)
{
    float v_mm_s = 0.5f * (s_wheel_left.v + s_wheel_right.v) / params_ticks_per_mm();

    s_report.distance_ticks  = progress;
    s_report.heading_err_deg = steer_heading_err(&s_steer, snap->yaw);
    s_report.lat_err_end_mm  = s_steer.lat_err;
    s_report.lat_err_max_mm  = fmaxf(s_report.lat_err_max_mm, fabsf(s_steer.lat_err));
    s_report.peak_speed_mm_s = fmaxf(s_report.peak_speed_mm_s, v_mm_s);
}

static uint32_t straight_timeout_ms(void)
{
    int32_t cell  = (int32_t)params_get(P_TICKS_PER_CELL);
    int32_t cells = (s_straight.target_ticks + cell - 1) / cell;

    return SEGMENT_TIMEOUT_MS * (uint32_t)cells;
}

static void update_straight(const sensor_snapshot_t *snap, float dt)
{
    int32_t progress = ((snap->left_ticks - s_straight.start_left) +
                        (snap->right_ticks - s_straight.start_right)) / 2;
    float   rate_ref = steer_rate_ref(&s_steer, snap);

    straight_report(snap, progress);

    if (snap->timestamp_ms - s_move_start_ms > straight_timeout_ms()) {
        ESP_LOGE(TAG, "segment timeout");
        end_move(RESULT_TIMEOUT);
        return;
    }
    if (snap->front_tof >= 0 && snap->front_tof < FRONT_STOP_MM) {
        ESP_LOGW(TAG, "front wall: stop");
        end_move(RESULT_WALL_STOP);
        return;
    }
    if (progress >= s_straight.target_ticks) {
        ESP_LOGI(TAG, "segment done: progress=%ld", (long)progress);
        end_move(RESULT_DONE);
        return;
    }

    s_status.progress_ticks = progress;
    s_status.target_ticks   = s_straight.target_ticks;
    publish_status();

    drive_wheels(next_speed_ref(s_straight.target_ticks - progress, dt),
                 steer_wheel_half_diff(rate_ref), snap, dt);
}

/* ---- turn ---- */

/* Pivot: the right wheel goes forward for a left (+) turn, the left wheel backward. */
static void begin_turn(const motion_cmd_t *cmd, const sensor_snapshot_t *snap)
{
    float dir = cmd->angle_deg >= 0 ? 1.0f : -1.0f;

    /* Turn relative to the carried heading reference, not to the current yaw, so the
     * residual error of one turn is not added to the next. */
    s_steer.heading_ref += (float)cmd->angle_deg * (float)TURN_YAW_SIGN;

    wheel_reset(&s_wheel_left, snap->left_ticks, snap->timestamp_ms, -dir);
    wheel_reset(&s_wheel_right, snap->right_ticks, snap->timestamp_ms, dir);

    s_turn.phase     = TURN_DRIVE;
    s_turn.angle_deg = fabsf((float)cmd->angle_deg);
    s_turn.half_deg  = 0.5f * (float)cmd->angle_deg * (float)TURN_YAW_SIGN;
    s_turn.rate      = 0.0f;

    s_status.target_ticks = (int32_t)s_turn.angle_deg;
}

/* Heading error wrapped into a window centred on half the commanded change, so a 180 deg
 * turn has one well-defined direction from its first sample. */
static float turn_error(const sensor_snapshot_t *snap)
{
    return steer_heading_err(&s_steer, snap->yaw + s_turn.half_deg) + s_turn.half_deg;
}

static uint32_t turn_timeout_ms(void)
{
    return TURN_TIMEOUT_MS * (uint32_t)ceilf(s_turn.angle_deg / 90.0f);
}

/* Yaw-rate reference from the angle trapezoid on the remaining angle. */
static float turn_rate_ref(float err, float dt)
{
    profile_limits_t lim = {
        .v_max = params_get(P_TURN_MAX_RATE),
        .accel = params_get(P_TURN_ACCEL),
        .decel = params_get(P_TURN_DECEL),
        .v_min = TURN_MIN_RATE_DPS,
    };

    s_turn.rate = profile_next(s_turn.rate, fabsf(err), dt, &lim);
    return err >= 0.0f ? s_turn.rate : -s_turn.rate;
}

/* Pivot in place (v = 0) through the wheel speed loops. A residual error after settling
 * is not re-driven; the carried heading reference absorbs it on the next move. */
static void update_turn(const sensor_snapshot_t *snap, float dt)
{
    uint32_t now = snap->timestamp_ms;
    float    err = turn_error(snap);

    s_report.heading_err_deg = err;

    if (now - s_move_start_ms > turn_timeout_ms()) {
        ESP_LOGE(TAG, "turn timeout");
        end_move(RESULT_TIMEOUT);
        return;
    }

    if (s_turn.phase == TURN_SETTLE) {
        if (now - s_turn.settle_ms >= (uint32_t)params_get(P_TURN_SETTLE_MS)) {
            ESP_LOGI(TAG, "turn done: err=%.2f", err);
            s_status.progress_ticks = (int32_t)s_turn.angle_deg;
            end_move(RESULT_DONE);
        }
        return;
    }

    s_status.progress_ticks = (int32_t)(s_turn.angle_deg - fabsf(err));
    publish_status();

    if (fabsf(err) <= TURN_TOL_DEG) {
        motors_brake();
        s_turn.phase     = TURN_SETTLE;
        s_turn.settle_ms = now;
        return;
    }

    drive_wheels(0.0f, steer_wheel_half_diff(turn_rate_ref(err, dt)), snap, dt);
}

static const move_ops_t s_straight_ops = { begin_straight, update_straight };
static const move_ops_t s_turn_ops     = { begin_turn,     update_turn     };

static const move_ops_t *const s_move_ops[MOVE_TYPE_COUNT] = {
    [MOVE_STRAIGHT] = &s_straight_ops,
    [MOVE_TURN]     = &s_turn_ops,
};

static void start_move(const motion_cmd_t *cmd, const sensor_snapshot_t *snap)
{
    sync_heading_ref(snap);
    memset(&s_report, 0, sizeof(s_report));
    s_move_start_ms = snap->timestamp_ms;

    s_active = s_move_ops[cmd->type];
    s_active->begin(cmd, snap);

    s_status.state          = MOTION_RUNNING;
    s_status.progress_ticks = 0;
    publish_status();

    motor_enable(true);
}

/* ---- commands ---- */

static void cmd_queue_move(const motion_cmd_t *cmd)
{
    if (s_active != NULL || s_has_pending) {
        ESP_LOGW(TAG, "move ignored: motion busy");
        return;
    }
    s_pending = *cmd;
    if ((unsigned)s_pending.profile >= PROFILE_COUNT) {
        s_pending.profile = PROFILE_EXPLORE;
    }
    s_has_pending = true;
}

/* Add cells to the run in progress so the robot keeps going without braking. */
static void cmd_extend(const motion_cmd_t *cmd)
{
    if (s_active == &s_straight_ops) {
        s_straight.target_ticks += (int32_t)((float)cmd->cells * params_get(P_TICKS_PER_CELL));
        s_status.target_ticks    = s_straight.target_ticks;
    } else if (s_has_pending && s_pending.type == MOVE_STRAIGHT) {
        s_pending.cells += cmd->cells;
    } else {
        ESP_LOGW(TAG, "extend ignored: no straight run");
    }
}

static void cmd_stop(const motion_cmd_t *cmd)
{
    (void)cmd;
    s_has_pending = false;
    if (s_active != NULL) {
        end_move(RESULT_STOPPED);
    }
}

static const cmd_handler_t s_cmd_handlers[MOVE_TYPE_COUNT] = {
    [MOVE_STRAIGHT] = cmd_queue_move,
    [MOVE_EXTEND]   = cmd_extend,
    [MOVE_TURN]     = cmd_queue_move,
    [MOVE_STOP]     = cmd_stop,
};

static void dispatch(const motion_cmd_t *cmd)
{
    if ((unsigned)cmd->type < MOVE_TYPE_COUNT) {
        s_cmd_handlers[cmd->type](cmd);
    }
}

/* No new snapshot in time: a move cannot be controlled blind, so it is ended. */
static void on_sensor_timeout(void)
{
    if (s_active == NULL && !s_has_pending) {
        return;
    }
    ESP_LOGE(TAG, "sensor lost");
    if (s_active == NULL) {
        memset(&s_report, 0, sizeof(s_report));
        s_move_start_ms = s_last_ms;
    }
    s_has_pending = false;
    end_move(RESULT_SENSOR_LOST);
}

/* Runs once per sensor snapshot: commands first, then one control step. */
static void motion_task(void *arg)
{
    (void)arg;

    publish_status();

    while (1) {
        bool fresh = app_ipc_wait_sensor(pdMS_TO_TICKS(MOTION_SENSOR_TIMEOUT_MS));

        motion_cmd_t cmd;
        while (app_ipc_recv_motion(&cmd, 0)) {
            dispatch(&cmd);
        }

        sensor_snapshot_t snap;
        if (!fresh || !app_ipc_read_sensor(&snap)) {
            on_sensor_timeout();
            continue;
        }

        float dt = step_dt(snap.timestamp_ms);

        if (s_active == NULL && s_has_pending) {
            s_has_pending = false;
            start_move(&s_pending, &snap);
        }
        if (s_active != NULL) {
            s_active->update(&snap, dt);
        }
    }
}

void motion_task_start(void)
{
    BaseType_t ok = xTaskCreatePinnedToCore(motion_task, "motion",
                                            MOTION_TASK_STACK, NULL,
                                            MOTION_TASK_PRIO, NULL,
                                            MOTION_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "motion task create failed");
    }
}
