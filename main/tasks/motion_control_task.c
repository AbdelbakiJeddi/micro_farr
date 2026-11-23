#include "motion_control_task.h"

#include <math.h>
#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "app_ipc.h"
#include "app_types.h"
#include "motor.h"
#include "params.h"

static const char *TAG = "motion";

typedef struct {
    bool       active;
    int32_t    start_left;
    int32_t    start_right;
    int32_t    target_ticks;
    float      yaw_ref;
    float      v_ref;        /* body speed reference (ticks/s) */
    int32_t    progress;     /* ticks travelled in this run */
    TickType_t start_tick;
} straight_state_t;

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

typedef enum {
    TURN_DRIVE,
    TURN_SETTLE,
} turn_phase_t;

typedef struct {
    bool         active;
    turn_phase_t phase;
    float        target_yaw;
    float        current_duty;
    TickType_t   start_tick;
    TickType_t   settle_tick;
} turn_state_t;

#define MOTION_DT_S  (1.0f / (float)MOTION_TASK_HZ)
#define DEG_TO_RAD   0.017453293f

static straight_state_t s_straight;
static wheel_ctrl_t     s_wheel_left;
static wheel_ctrl_t     s_wheel_right;
static turn_state_t     s_turn;
static bool             s_pending_straight;
static bool             s_pending_turn;
static bool             s_pending_turn_left;
static int32_t          s_target_ticks = CELL_TICKS;

/* Absolute heading the robot should be facing, carried from move to move so that
 * small turn errors do not accumulate. */
static float      s_heading_ref;
static bool       s_heading_valid;
static TickType_t s_last_end_tick;

static motion_status_t s_status = {
    .state          = MOTION_IDLE,
    .progress_ticks = 0,
    .target_ticks   = CELL_TICKS,
    .note           = "",
};

static void publish_status(void)
{
    app_ipc_publish_status(&s_status);
}

static void set_status_note(const char *note)
{
    snprintf(s_status.note, sizeof(s_status.note), "%s", note);
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

static bool motion_busy(void)
{
    return s_straight.active || s_turn.active;
}

/* Re-capture the heading reference if it is stale (robot may have been moved by hand). */
static void sync_heading_ref(const sensor_snapshot_t *snapshot)
{
    TickType_t idle = xTaskGetTickCount() - s_last_end_tick;

    if (!s_heading_valid || idle > pdMS_TO_TICKS(HEADING_REF_TIMEOUT_MS)) {
        s_heading_ref   = snapshot->yaw;
        s_heading_valid = true;
    }
}

/* Stop the motors, publish the outcome and raise the matching event. */
static void publish_result(bool success, const char *fail_note)
{
    motors_brake();
    s_last_end_tick = xTaskGetTickCount();

    s_status.state = success ? MOTION_DONE : MOTION_FAULT;
    set_status_note(success ? "" : fail_note);
    publish_status();

    app_ipc_set_event(success ? APP_EVENT_SEGMENT_DONE : APP_EVENT_FAULT);
}

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
static float wheel_duty(wheel_ctrl_t *w, float v_ref, int32_t ticks, uint32_t now_ms)
{
    float err = v_ref - speed_update(&w->est, ticks, now_ms);

    w->i_term = clamp_float(w->i_term + params_get(P_RUN_KI) * err * MOTION_DT_S, RUN_I_MAX_DUTY);
    return params_get(P_RUN_KFF) * v_ref + params_get(P_RUN_KP) * err + w->i_term;
}

static void begin_straight(const sensor_snapshot_t *snapshot)
{
    sync_heading_ref(snapshot);
    wheel_reset(&s_wheel_left, snapshot->left_ticks, snapshot->timestamp_ms);
    wheel_reset(&s_wheel_right, snapshot->right_ticks, snapshot->timestamp_ms);

    s_straight.active       = true;
    s_straight.start_left   = snapshot->left_ticks;
    s_straight.start_right  = snapshot->right_ticks;
    s_straight.target_ticks = s_target_ticks > 0 ? s_target_ticks : CELL_TICKS;
    s_straight.yaw_ref      = s_heading_ref;
    s_straight.v_ref        = RUN_MIN_TICKS_S;
    s_straight.progress     = 0;
    s_straight.start_tick   = xTaskGetTickCount();

    s_status.state          = MOTION_RUNNING;
    s_status.progress_ticks = 0;
    s_status.target_ticks   = s_straight.target_ticks;
    set_status_note("");
    publish_status();

    motor_enable(true);
}

static void finish_straight(bool success)
{
    if (!s_straight.active) {
        return;
    }

    s_straight.active = false;
    s_status.progress_ticks = s_straight.progress; /* actual distance, not the target */
    publish_result(success, "TIMEOUT");
}

static void begin_turn(const sensor_snapshot_t *snapshot, bool left)
{
    float direction = left ? 1.0f : -1.0f;

    /* Turn relative to the carried heading reference, not to the current yaw, so the
     * residual error of one turn is not added to the next. */
    sync_heading_ref(snapshot);
    s_heading_ref += direction * TURN_TARGET_DEG * (float)TURN_YAW_SIGN;

    s_turn.active       = true;
    s_turn.phase        = TURN_DRIVE;
    s_turn.target_yaw   = s_heading_ref;
    s_turn.current_duty = TURN_MIN_DUTY;
    s_turn.start_tick   = xTaskGetTickCount();

    s_status.state          = MOTION_RUNNING;
    s_status.progress_ticks = 0;
    s_status.target_ticks   = (int32_t)TURN_TARGET_DEG;
    set_status_note(left ? "TURN L" : "TURN R");
    publish_status();

    motor_enable(true);
}

static void finish_turn(bool success)
{
    if (!s_turn.active) {
        return;
    }

    s_turn.active = false;
    publish_result(success, "TURN TMO");
}

static void stop_motion(void)
{
    s_pending_straight = false;
    s_pending_turn     = false;

    if (motion_busy()) {
        s_straight.active = false;
        s_turn.active     = false;
        motors_brake();
        s_last_end_tick = xTaskGetTickCount();
        s_status.state  = MOTION_IDLE;
        set_status_note("STOP");
        publish_status();
    }
}

static void queue_straight(void)
{
    if (motion_busy()) {
        ESP_LOGW(TAG, "straight ignored: motion active");
        return;
    }
    s_pending_straight = true;
}

/* Add one cell to the run in progress so the robot keeps going without braking. */
static void extend_straight(void)
{
    if (s_straight.active) {
        s_straight.target_ticks += CELL_TICKS;
        s_status.target_ticks    = s_straight.target_ticks;
    } else if (s_pending_straight) {
        s_target_ticks += CELL_TICKS;
    } else {
        ESP_LOGW(TAG, "extend ignored: no straight run");
    }
}

static void queue_turn(bool left)
{
    if (motion_busy()) {
        ESP_LOGW(TAG, "turn ignored: motion active");
        return;
    }
    s_pending_turn_left = left;
    s_pending_turn      = true;
}

static void handle_command(motion_cmd_t cmd)
{
    switch (cmd) {
    case CMD_STRAIGHT_CELL:
        queue_straight();
        break;

    case CMD_EXTEND_CELL:
        extend_straight();
        break;

    case CMD_TURN_LEFT:
        queue_turn(true);
        break;

    case CMD_TURN_RIGHT:
        queue_turn(false);
        break;

    case CMD_STOP:
        stop_motion();
        break;

    default:
        break;
    }
}

static bool side_is_wall(int32_t tof_mm)
{
    return tof_mm >= 0 && tof_mm < TOF_WALL_MAX_MM;
}

/* Lateral error in mm: positive = too close to the right wall (steer left). */
static float tof_lateral_error(const sensor_snapshot_t *snapshot)
{
    bool left  = side_is_wall(snapshot->left_tof);
    bool right = side_is_wall(snapshot->right_tof);

    if (left && right) {
        return 0.5f * (float)(snapshot->left_tof - snapshot->right_tof);
    }
    if (left) {
        return (float)(snapshot->left_tof - TOF_SIDE_TARGET_MM);
    }
    if (right) {
        return (float)(TOF_SIDE_TARGET_MM - snapshot->right_tof);
    }
    return 0.0f; /* no side walls: drive straight */
}

/* Body yaw-rate reference (deg/s): IMU heading hold + ToF wall-centering summed directly.
 * YAW_KP corrects heading error; TOF_CENTER_KP corrects lateral position error.
 * Both contributions are in deg/s and feed into the same per-wheel speed difference. */
static float steer_rate_ref(const sensor_snapshot_t *snapshot)
{
    float yaw_err = wrap180(s_straight.yaw_ref - snapshot->yaw);
    float lat_err = tof_lateral_error(snapshot);
    return clamp_float(params_get(P_YAW_KP) * yaw_err + params_get(P_TOF_CENTER_KP) * lat_err, params_get(P_YAW_MAX_RATE_DPS));
}

/* Half the left/right wheel speed difference (ticks/s) that produces the yaw rate,
 * with gyro feedback correcting for wheel slip and wheelbase error. */
static float wheel_half_diff(float rate_ref_dps, float rate_meas_dps)
{
    float rate = rate_ref_dps + YAW_RATE_KP * (rate_ref_dps - rate_meas_dps);

    return (float)TURN_YAW_SIGN * rate * DEG_TO_RAD * (0.5f * WHEEL_BASE_MM) * TICKS_PER_MM;
}

/* Trapezoid profile: accelerate to cruise, brake to reach the target with the speed
 * falling to the creep floor. Extending the target moves the braking point further away. */
static float next_speed_ref(int32_t remaining_ticks)
{
    float v_brake = fmaxf(sqrtf(2.0f * (params_get(P_EXP_DECEL) * TICKS_PER_MM) * (float)remaining_ticks),
                          RUN_MIN_TICKS_S);
    float v_up    = s_straight.v_ref + (params_get(P_EXP_ACCEL) * TICKS_PER_MM) * MOTION_DT_S;
    float v       = fminf(fminf(v_up, params_get(P_EXP_CRUISE) * TICKS_PER_MM), v_brake);

    s_straight.v_ref = v;
    return v;
}

/* Negative duty is allowed so a wheel loop can brake actively. */
static int clamp_run_duty(float duty)
{
    duty = clamp_float(duty, params_get(P_RUN_MAX_DUTY));
    return (int)(duty >= 0.0f ? (duty + 0.5f) : (duty - 0.5f));
}

static void update_straight(const sensor_snapshot_t *snapshot)
{
    uint32_t timeout_ms = SEGMENT_TIMEOUT_MS * (uint32_t)((s_straight.target_ticks + CELL_TICKS - 1) / CELL_TICKS);
    TickType_t elapsed = xTaskGetTickCount() - s_straight.start_tick;
    if (elapsed > pdMS_TO_TICKS(timeout_ms)) {
        ESP_LOGE(TAG, "segment timeout");
        finish_straight(false);
        return;
    }

    if (snapshot->front_tof >= 0 && snapshot->front_tof < FRONT_STOP_MM) {
        ESP_LOGW(TAG, "front wall: stop");
        finish_straight(true);
        return;
    }

    int32_t progress = ((snapshot->left_ticks - s_straight.start_left) +
                        (snapshot->right_ticks - s_straight.start_right)) / 2;
    s_straight.progress = progress;

    if (progress >= s_straight.target_ticks) {
        ESP_LOGI(TAG, "segment done: progress=%ld", (long)progress);
        finish_straight(true);
        return;
    }

    s_status.state          = MOTION_RUNNING;
    s_status.progress_ticks = progress;
    s_status.target_ticks   = s_straight.target_ticks;
    publish_status();

    float v_body = next_speed_ref(s_straight.target_ticks - progress);
    float half   = wheel_half_diff(steer_rate_ref(snapshot), snapshot->yaw_rate_dps);

    float left_duty  = wheel_duty(&s_wheel_left, v_body - half,
                                  snapshot->left_ticks, snapshot->timestamp_ms);
    float right_duty = wheel_duty(&s_wheel_right, v_body + half,
                                  snapshot->right_ticks, snapshot->timestamp_ms);

    motor_set_speed(&motor_left, clamp_run_duty(left_duty));
    motor_set_speed(&motor_right, clamp_run_duty(right_duty));
}

static void turn_enter_settle(float abs_err, TickType_t now)
{
    motors_brake();
    s_turn.phase       = TURN_SETTLE;
    s_turn.settle_tick = now;

    s_status.progress_ticks = (int32_t)(TURN_TARGET_DEG - abs_err);
    publish_status();
}

/* Settle phase: wait for the robot to stop, then finish. A residual error is not
 * re-driven (that ping-pongs around the tolerance); the carried heading reference
 * absorbs it on the next move. Returns true while settling. */
static bool turn_handle_settle(float abs_err, TickType_t now)
{
    if (now - s_turn.settle_tick < pdMS_TO_TICKS((uint32_t)params_get(P_TURN_SETTLE_MS))) {
        return true;
    }
    if (abs_err <= TURN_TOL_DEG) {
        ESP_LOGI(TAG, "turn done: err=%.2f", abs_err);
    } else {
        ESP_LOGW(TAG, "turn done outside tolerance: err=%.2f", abs_err);
    }
    s_status.progress_ticks = (int32_t)TURN_TARGET_DEG;
    finish_turn(true);
    return true;
}

/* Duty magnitude for the remaining error, slewed both up and down. */
static float turn_next_duty(float abs_err)
{
    float target = fminf(params_get(P_TURN_FF_DUTY) + params_get(P_TURN_KP) * abs_err, params_get(P_TURN_MAX_DUTY));
    float step   = clamp_float(target - s_turn.current_duty, params_get(P_TURN_SLEW));

    s_turn.current_duty += step;
    return s_turn.current_duty;
}

/* Pivot in place toward the heading reference using the gyro yaw. */
static void update_turn(const sensor_snapshot_t *snapshot)
{
    TickType_t now = xTaskGetTickCount();

    if (now - s_turn.start_tick > pdMS_TO_TICKS(TURN_TIMEOUT_MS)) {
        ESP_LOGE(TAG, "turn timeout");
        finish_turn(false);
        return;
    }

    float yaw_err = wrap180(s_turn.target_yaw - snapshot->yaw);
    float abs_err = fabsf(yaw_err);

    if (s_turn.phase == TURN_SETTLE && turn_handle_settle(abs_err, now)) {
        return;
    }

    if (abs_err <= TURN_TOL_DEG) {
        turn_enter_settle(abs_err, now);
        return;
    }

    s_status.progress_ticks = (int32_t)(TURN_TARGET_DEG - abs_err);
    publish_status();

    float duty        = turn_next_duty(abs_err);
    float physical    = yaw_err * (float)TURN_YAW_SIGN;
    float signed_duty = physical >= 0.0f ? duty : -duty;

    motor_set_speed(&motor_left,  clamp_duty(-signed_duty, TURN_MIN_DUTY, params_get(P_TURN_MAX_DUTY)));
    motor_set_speed(&motor_right, clamp_duty(signed_duty, TURN_MIN_DUTY, params_get(P_TURN_MAX_DUTY)));
}

static void start_pending_motion(const sensor_snapshot_t *snapshot)
{
    if (s_pending_straight) {
        s_pending_straight = false;
        begin_straight(snapshot);
    } else if (s_pending_turn) {
        s_pending_turn = false;
        begin_turn(snapshot, s_pending_turn_left);
    }
}

static void motion_control_task(void *arg)
{
    (void)arg;

    const TickType_t period = pdMS_TO_TICKS(1000 / MOTION_TASK_HZ);
    TickType_t last_wake = xTaskGetTickCount();

    publish_status();

    while (1) {
        motion_cmd_t cmd;
        while (app_ipc_recv_motion(&cmd, 0)) {
            handle_command(cmd);
        }

        sensor_snapshot_t snapshot;
        bool have_snapshot = app_ipc_read_sensor(&snapshot);

        if (have_snapshot) {
            start_pending_motion(&snapshot);

            if (s_straight.active) {
                update_straight(&snapshot);
            } else if (s_turn.active) {
                update_turn(&snapshot);
            }
        }

        vTaskDelayUntil(&last_wake, period);
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

static esp_err_t send_motion_cmd(motion_cmd_t cmd)
{
    app_ipc_clear_event(APP_EVENT_SEGMENT_DONE | APP_EVENT_FAULT);

    if (!app_ipc_send_motion(cmd, pdMS_TO_TICKS(100))) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t straight_cells(uint8_t cells)
{
    if (cells == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    s_target_ticks = (int32_t)cells * CELL_TICKS;
    return send_motion_cmd(CMD_STRAIGHT_CELL);
}

esp_err_t straight_extend_cell(void)
{
    /* No event clearing here: the run in progress must keep its own completion event. */
    return app_ipc_send_motion(CMD_EXTEND_CELL, pdMS_TO_TICKS(100)) ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t turn_left(void)
{
    return send_motion_cmd(CMD_TURN_LEFT);
}

esp_err_t turn_right(void)
{
    return send_motion_cmd(CMD_TURN_RIGHT);
}

esp_err_t straight_one_cell(void)
{
    return straight_cells(1);
}
