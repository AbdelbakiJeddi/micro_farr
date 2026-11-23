#include "motion_control_task.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "app_ipc.h"
#include "app_types.h"
#include "motor.h"
#include "params.h"

static const char *TAG = "motion";

#define DEG_TO_RAD 0.017453293f

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
    int32_t  ref_ticks;      /* wheel ticks at the last speed sample */
    uint32_t ref_ms;
    float    v;              /* filtered wheel speed (ticks/s) */
} speed_est_t;

/* One wheel's speed loop: its own speed estimate and its own integral. */
typedef struct {
    speed_est_t est;
    float       i_term;      /* duty % */
} wheel_ctrl_t;

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
    float        current_duty;
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
static move_report_t    s_report;
static uint32_t         s_move_start_ms;
static uint32_t         s_last_ms;        /* timestamp of the last snapshot used */

/* Absolute heading the robot should be facing, carried from move to move so that
 * small turn errors do not accumulate. */
static float    s_heading_ref;
static bool     s_heading_valid;
static uint32_t s_last_end_ms;

static motion_status_t s_status = { .state = MOTION_IDLE };

static void publish_status(void)
{
    app_ipc_publish_status(&s_status);
}

static float wrap180(float deg)
{
    while (deg > 180.0f) {
        deg -= 360.0f;
    }
    while (deg < -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

static float clamp_float(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static int clamp_duty(float duty, float min_duty, float max_duty)
{
    if (duty > max_duty) {
        duty = max_duty;
    }
    if (duty < -max_duty) {
        duty = -max_duty;
    }

    int out = (int)(duty >= 0.0f ? (duty + 0.5f) : (duty - 0.5f));

    if (out > 0 && out < (int)min_duty) {
        out = (int)min_duty;
    }
    if (out < 0 && out > -(int)min_duty) {
        out = -(int)min_duty;
    }
    return out;
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
        s_heading_ref   = snap->yaw;
        s_heading_valid = true;
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

/* ---- wheel speed loops ---- */

static void wheel_reset(wheel_ctrl_t *w, int32_t ticks, uint32_t now_ms)
{
    w->est.ref_ticks = ticks;
    w->est.ref_ms    = now_ms;
    w->est.v         = 0.0f;
    w->i_term        = RUN_START_DUTY;
}

/* Wheel speed in ticks/s, measured over windows of at least SPEED_WINDOW_MS and filtered. */
static float speed_update(speed_est_t *est, int32_t ticks, uint32_t now_ms)
{
    uint32_t dt_ms = now_ms - est->ref_ms;

    if (dt_ms >= SPEED_WINDOW_MS) {
        float inst = (float)(ticks - est->ref_ticks) * 1000.0f / (float)dt_ms;

        est->v += SPEED_FILTER_ALPHA * (inst - est->v);
        est->ref_ticks = ticks;
        est->ref_ms    = now_ms;
    }
    return est->v;
}

/* Feed-forward + PI on one wheel's speed; returns duty in percent. */
static float wheel_duty(wheel_ctrl_t *w, float v_ref, int32_t ticks, uint32_t now_ms, float dt)
{
    float err = v_ref - speed_update(&w->est, ticks, now_ms);

    w->i_term = clamp_float(w->i_term + params_get(P_RUN_KI) * err * dt, RUN_I_MAX_DUTY);
    return params_get(P_RUN_KFF) * v_ref + params_get(P_RUN_KP) * err + w->i_term;
}

/* Negative duty is allowed so a wheel loop can brake actively. */
static int clamp_run_duty(float duty)
{
    duty = clamp_float(duty, params_get(P_RUN_MAX_DUTY));
    return (int)(duty >= 0.0f ? (duty + 0.5f) : (duty - 0.5f));
}

/* ---- steering ---- */

static bool side_is_wall(int32_t tof_mm)
{
    return tof_mm >= 0 && tof_mm < TOF_WALL_MAX_MM;
}

/* Lateral error in mm: positive = too close to the right wall (steer left). */
static float tof_lateral_error(const sensor_snapshot_t *snap)
{
    bool left  = side_is_wall(snap->left_tof);
    bool right = side_is_wall(snap->right_tof);

    if (left && right) {
        return 0.5f * (float)(snap->left_tof - snap->right_tof);
    }
    if (left) {
        return (float)(snap->left_tof - TOF_SIDE_TARGET_MM);
    }
    if (right) {
        return (float)(TOF_SIDE_TARGET_MM - snap->right_tof);
    }
    return 0.0f; /* no side walls: drive straight */
}

/* Body yaw-rate reference (deg/s): IMU heading hold + ToF wall-centering summed directly. */
static float steer_rate_ref(float yaw_err, float lat_err)
{
    return clamp_float(params_get(P_YAW_KP) * yaw_err + params_get(P_TOF_CENTER_KP) * lat_err,
                       params_get(P_YAW_MAX_RATE_DPS));
}

/* Half the left/right wheel speed difference (ticks/s) that produces the yaw rate,
 * with gyro feedback correcting for wheel slip and wheelbase error. */
static float wheel_half_diff(float rate_ref_dps, float rate_meas_dps)
{
    float rate = rate_ref_dps + YAW_RATE_KP * (rate_ref_dps - rate_meas_dps);

    return (float)TURN_YAW_SIGN * rate * DEG_TO_RAD * (0.5f * WHEEL_BASE_MM) * TICKS_PER_MM;
}

/* ---- straight ---- */

/* Trapezoid profile: accelerate to cruise, brake to reach the target with the speed
 * falling to the creep floor. Extending the target moves the braking point further away. */
static float next_speed_ref(int32_t remaining_ticks, float dt)
{
    const profile_params_t *p = &s_profiles[s_straight.profile];
    float remaining = fmaxf((float)remaining_ticks, 0.0f);
    float v_brake   = fmaxf(sqrtf(2.0f * params_get(p->decel) * TICKS_PER_MM * remaining),
                            RUN_MIN_TICKS_S);
    float v_up      = s_straight.v_ref + params_get(p->accel) * TICKS_PER_MM * dt;
    float v         = fminf(fminf(v_up, params_get(p->cruise) * TICKS_PER_MM), v_brake);

    s_straight.v_ref = v;
    return v;
}

static void begin_straight(const motion_cmd_t *cmd, const sensor_snapshot_t *snap)
{
    int cells = cmd->cells > 0 ? cmd->cells : 1;

    wheel_reset(&s_wheel_left, snap->left_ticks, snap->timestamp_ms);
    wheel_reset(&s_wheel_right, snap->right_ticks, snap->timestamp_ms);

    s_straight.start_left   = snap->left_ticks;
    s_straight.start_right  = snap->right_ticks;
    s_straight.target_ticks = (int32_t)((float)cells * params_get(P_TICKS_PER_CELL));
    s_straight.v_ref        = RUN_MIN_TICKS_S;
    s_straight.profile      = cmd->profile;

    s_status.target_ticks = s_straight.target_ticks;
}

/* Fill the straight part of the report from the current sample. */
static void straight_report(const sensor_snapshot_t *snap, int32_t progress, float lat_err)
{
    float v_mm_s = 0.5f * (s_wheel_left.est.v + s_wheel_right.est.v) / TICKS_PER_MM;

    s_report.distance_ticks  = progress;
    s_report.heading_err_deg = wrap180(s_heading_ref - snap->yaw);
    s_report.lat_err_end_mm  = lat_err;
    s_report.lat_err_max_mm  = fmaxf(s_report.lat_err_max_mm, fabsf(lat_err));
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
    float   lat_err  = tof_lateral_error(snap);

    straight_report(snap, progress, lat_err);

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

    float v_body = next_speed_ref(s_straight.target_ticks - progress, dt);
    float half   = wheel_half_diff(steer_rate_ref(s_report.heading_err_deg, lat_err),
                                   snap->yaw_rate_dps);

    float left_duty  = wheel_duty(&s_wheel_left, v_body - half,
                                  snap->left_ticks, snap->timestamp_ms, dt);
    float right_duty = wheel_duty(&s_wheel_right, v_body + half,
                                  snap->right_ticks, snap->timestamp_ms, dt);

    motor_set_speed(&motor_left, clamp_run_duty(left_duty));
    motor_set_speed(&motor_right, clamp_run_duty(right_duty));
}

/* ---- turn ---- */

static void begin_turn(const motion_cmd_t *cmd, const sensor_snapshot_t *snap)
{
    (void)snap;

    /* Turn relative to the carried heading reference, not to the current yaw, so the
     * residual error of one turn is not added to the next. */
    s_heading_ref += (float)cmd->angle_deg * (float)TURN_YAW_SIGN;

    s_turn.phase        = TURN_DRIVE;
    s_turn.angle_deg    = fabsf((float)cmd->angle_deg);
    s_turn.current_duty = TURN_MIN_DUTY;

    s_status.target_ticks = (int32_t)s_turn.angle_deg;
}

/* Duty magnitude for the remaining error, slewed both up and down. */
static float turn_next_duty(float abs_err)
{
    float target = fminf(params_get(P_TURN_FF_DUTY) + params_get(P_TURN_KP) * abs_err,
                         params_get(P_TURN_MAX_DUTY));
    float step   = clamp_float(target - s_turn.current_duty, params_get(P_TURN_SLEW));

    s_turn.current_duty += step;
    return s_turn.current_duty;
}

static uint32_t turn_timeout_ms(void)
{
    return TURN_TIMEOUT_MS * (uint32_t)ceilf(s_turn.angle_deg / 90.0f);
}

/* Pivot in place toward the heading reference using the gyro yaw. A residual error after
 * settling is not re-driven; the carried heading reference absorbs it on the next move. */
static void update_turn(const sensor_snapshot_t *snap, float dt)
{
    (void)dt;
    uint32_t now     = snap->timestamp_ms;
    float    yaw_err = wrap180(s_heading_ref - snap->yaw);
    float    abs_err = fabsf(yaw_err);

    s_report.heading_err_deg = yaw_err;

    if (now - s_move_start_ms > turn_timeout_ms()) {
        ESP_LOGE(TAG, "turn timeout");
        end_move(RESULT_TIMEOUT);
        return;
    }

    if (s_turn.phase == TURN_SETTLE) {
        if (now - s_turn.settle_ms >= (uint32_t)params_get(P_TURN_SETTLE_MS)) {
            ESP_LOGI(TAG, "turn done: err=%.2f", yaw_err);
            s_status.progress_ticks = (int32_t)s_turn.angle_deg;
            end_move(RESULT_DONE);
        }
        return;
    }

    s_status.progress_ticks = (int32_t)(s_turn.angle_deg - abs_err);
    publish_status();

    if (abs_err <= TURN_TOL_DEG) {
        motors_brake();
        s_turn.phase     = TURN_SETTLE;
        s_turn.settle_ms = now;
        return;
    }

    float duty        = turn_next_duty(abs_err);
    float physical    = yaw_err * (float)TURN_YAW_SIGN;
    float signed_duty = physical >= 0.0f ? duty : -duty;
    float max_duty    = params_get(P_TURN_MAX_DUTY);

    motor_set_speed(&motor_left,  clamp_duty(-signed_duty, TURN_MIN_DUTY, max_duty));
    motor_set_speed(&motor_right, clamp_duty(signed_duty, TURN_MIN_DUTY, max_duty));
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
static void motion_control_task(void *arg)
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

void motion_control_task_start(void)
{
    BaseType_t ok = xTaskCreatePinnedToCore(motion_control_task, "motion",
                                            MOTION_TASK_STACK, NULL,
                                            MOTION_TASK_PRIO, NULL,
                                            MOTION_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "motion task create failed");
    }
}
