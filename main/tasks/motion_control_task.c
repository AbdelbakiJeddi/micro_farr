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

static const char *TAG = "motion";

typedef struct {
    bool       active;
    int32_t    start_left;
    int32_t    start_right;
    int32_t    target_ticks;
    float      yaw_ref;
    float      yaw_offset;   /* ToF-driven correction added to yaw_ref (deg) */
    float      v_ref;        /* speed reference (ticks/s) */
    float      i_term;       /* speed-loop integral term (duty %) */
    int32_t    progress;     /* ticks travelled in this run */
    TickType_t start_tick;
} straight_state_t;

typedef struct {
    int32_t  ref_ticks;      /* average wheel ticks at the last speed sample */
    uint32_t ref_ms;
    float    v;              /* filtered speed (ticks/s) */
} speed_est_t;

typedef enum {
    TURN_DRIVE,
    TURN_SETTLE,
} turn_phase_t;

typedef struct {
    bool         active;
    turn_phase_t phase;
    float        target_yaw;
    float        current_duty;
    float        last_yaw;
    float        rate_dps;
    uint32_t     last_ms;
    TickType_t   start_tick;
    TickType_t   settle_tick;
} turn_state_t;

#define MOTION_DT_S (1.0f / (float)MOTION_TASK_HZ)

static straight_state_t s_straight;
static speed_est_t      s_speed;
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

static int32_t avg_ticks(const sensor_snapshot_t *snapshot)
{
    return (snapshot->left_ticks + snapshot->right_ticks) / 2;
}

static void speed_reset(const sensor_snapshot_t *snapshot)
{
    s_speed.ref_ticks = avg_ticks(snapshot);
    s_speed.ref_ms    = snapshot->timestamp_ms;
    s_speed.v         = 0.0f;
}

/* Wheel speed in ticks/s, measured over windows of at least SPEED_WINDOW_MS and filtered. */
static float speed_update(const sensor_snapshot_t *snapshot)
{
    uint32_t dt_ms = snapshot->timestamp_ms - s_speed.ref_ms;

    if (dt_ms >= SPEED_WINDOW_MS) {
        int32_t ticks = avg_ticks(snapshot);
        float   inst  = (float)(ticks - s_speed.ref_ticks) * 1000.0f / (float)dt_ms;

        s_speed.v += SPEED_FILTER_ALPHA * (inst - s_speed.v);
        s_speed.ref_ticks = ticks;
        s_speed.ref_ms    = snapshot->timestamp_ms;
    }
    return s_speed.v;
}

static void begin_straight(const sensor_snapshot_t *snapshot)
{
    sync_heading_ref(snapshot);
    speed_reset(snapshot);

    s_straight.active       = true;
    s_straight.start_left   = snapshot->left_ticks;
    s_straight.start_right  = snapshot->right_ticks;
    s_straight.target_ticks = s_target_ticks > 0 ? s_target_ticks : CELL_TICKS;
    s_straight.yaw_ref      = s_heading_ref;
    s_straight.yaw_offset   = 0.0f;
    s_straight.v_ref        = RUN_MIN_TICKS_S;
    s_straight.i_term       = RUN_START_DUTY;
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
    s_turn.last_yaw     = snapshot->yaw;
    s_turn.rate_dps     = 0.0f;
    s_turn.last_ms      = snapshot->timestamp_ms;
    s_turn.start_tick   = xTaskGetTickCount();
    s_turn.settle_tick  = s_turn.start_tick;

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

/* ToF only: steer straight from the lateral error. */
static float tof_only_steer(const sensor_snapshot_t *snapshot)
{
    return clamp_float(TOF_CENTER_KP * tof_lateral_error(snapshot), TOF_CENTER_MAX_CORR);
}

/* IMU inner loop: hold (yaw_ref + offset_deg). */
static float yaw_hold_steer(const sensor_snapshot_t *snapshot, float offset_deg)
{
    float yaw_err = wrap180(s_straight.yaw_ref + offset_deg - snapshot->yaw);
    return clamp_float(YAW_KP * yaw_err, YAW_MAX_CORR);
}

/* ToF outer loop: turn the lateral error into a smoothed heading offset. */
static float tof_heading_offset(const sensor_snapshot_t *snapshot)
{
    float target = clamp_float(TOF_YAW_GAIN * tof_lateral_error(snapshot),
                               TOF_YAW_MAX_OFFSET_DEG);
    s_straight.yaw_offset += TOF_OFFSET_ALPHA * (target - s_straight.yaw_offset);
    return s_straight.yaw_offset;
}

static float compute_steer(const sensor_snapshot_t *snapshot)
{
    if (STEER_MODE == STEER_MODE_YAW) {
        return yaw_hold_steer(snapshot, 0.0f);
    }
    if (STEER_MODE == STEER_MODE_TOF) {
        return tof_only_steer(snapshot);
    }
    return yaw_hold_steer(snapshot, tof_heading_offset(snapshot));
}

/* Trapezoid profile: accelerate to cruise, brake so the speed reaches the creep speed
 * exactly at the target. Extending the target moves the braking point further away. */
static float next_speed_ref(int32_t remaining_ticks)
{
    float v_brake = sqrtf(2.0f * RUN_DECEL_TICKS_S2 * (float)remaining_ticks) + RUN_MIN_TICKS_S;
    float v_up    = s_straight.v_ref + RUN_ACCEL_TICKS_S2 * MOTION_DT_S;
    float v       = fminf(fminf(v_up, RUN_CRUISE_TICKS_S), v_brake);

    s_straight.v_ref = v;
    return v;
}

/* Feed-forward + PI on wheel speed; returns duty in percent. */
static float speed_duty(float v_ref, float v_meas)
{
    float err = v_ref - v_meas;

    s_straight.i_term += RUN_KI * err * MOTION_DT_S;
    if (s_straight.i_term > RUN_I_MAX_DUTY) {
        s_straight.i_term = RUN_I_MAX_DUTY;
    }
    if (s_straight.i_term < -RUN_I_MAX_DUTY) {
        s_straight.i_term = -RUN_I_MAX_DUTY;
    }
    return RUN_KFF * v_ref + RUN_KP * err + s_straight.i_term;
}

/* Straight running never reverses a wheel. */
static int clamp_run_duty(float duty)
{
    if (duty < 0.0f) {
        duty = 0.0f;
    }
    if (duty > RUN_MAX_DUTY) {
        duty = RUN_MAX_DUTY;
    }
    return (int)(duty + 0.5f);
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

    float v_meas = speed_update(snapshot);
    float v_ref  = next_speed_ref(s_straight.target_ticks - progress);
    float cmd    = speed_duty(v_ref, v_meas);
    float steer  = compute_steer(snapshot);

    int left_cmd  = clamp_run_duty(cmd - (float)YAW_STEER_SIGN * steer);
    int right_cmd = clamp_run_duty(cmd + (float)YAW_STEER_SIGN * steer);

    motor_set_speed(&motor_left, left_cmd);
    motor_set_speed(&motor_right, right_cmd);
}

/* Turn rate (deg/s) from successive yaw samples, used to predict the coast after braking. */
static void turn_update_rate(const sensor_snapshot_t *snapshot)
{
    if (snapshot->timestamp_ms == s_turn.last_ms) {
        return;
    }

    float dt_s = (float)(snapshot->timestamp_ms - s_turn.last_ms) / 1000.0f;
    if (dt_s > 0.0f) {
        s_turn.rate_dps = (snapshot->yaw - s_turn.last_yaw) / dt_s;
    }
    s_turn.last_yaw = snapshot->yaw;
    s_turn.last_ms  = snapshot->timestamp_ms;
}

static void turn_enter_settle(float abs_err, TickType_t now)
{
    motors_brake();
    s_turn.phase       = TURN_SETTLE;
    s_turn.settle_tick = now;
    s_turn.rate_dps    = 0.0f;

    s_status.progress_ticks = (int32_t)(TURN_TARGET_DEG - abs_err);
    publish_status();
}

/* Settle phase: wait for the robot to stop, then finish if on target or resume a slow
 * correction. Returns true while the turn is still settling or has just finished. */
static bool turn_handle_settle(float abs_err, TickType_t now)
{
    if (now - s_turn.settle_tick < pdMS_TO_TICKS(TURN_SETTLE_MS)) {
        return true;
    }
    if (abs_err <= TURN_TOL_DEG) {
        ESP_LOGI(TAG, "turn done: err=%.2f", abs_err);
        s_status.progress_ticks = (int32_t)TURN_TARGET_DEG;
        finish_turn(true);
        return true;
    }

    /* Coasted past the target: resume a low-speed correction. */
    s_turn.phase        = TURN_DRIVE;
    s_turn.current_duty = TURN_MIN_DUTY;
    s_turn.rate_dps     = 0.0f;
    return false;
}

/* Duty magnitude for the remaining error, with an acceleration slew. */
static float turn_next_duty(float abs_err)
{
    float target = TURN_FF_DUTY + TURN_KP * abs_err;
    if (target > TURN_MAX_DUTY) {
        target = TURN_MAX_DUTY;
    }

    if (s_turn.current_duty < target) {
        s_turn.current_duty += TURN_ACCEL_SLEW;
        if (s_turn.current_duty > target) {
            s_turn.current_duty = target;
        }
    } else {
        s_turn.current_duty = target;
    }
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

    turn_update_rate(snapshot);

    float yaw_err = wrap180(s_turn.target_yaw - snapshot->yaw);
    float abs_err = fabsf(yaw_err);

    if (s_turn.phase == TURN_SETTLE && turn_handle_settle(abs_err, now)) {
        return;
    }

    /* Predict how far the robot will coast after braking and stop early. */
    float coast_deg  = s_turn.rate_dps * ((float)TURN_COAST_MS / 1000.0f);
    float projected  = yaw_err - coast_deg;
    bool  will_reach = (yaw_err > 0.0f && projected <= TURN_TOL_DEG) ||
                       (yaw_err < 0.0f && projected >= -TURN_TOL_DEG);

    if (abs_err <= TURN_TOL_DEG || will_reach) {
        turn_enter_settle(abs_err, now);
        return;
    }

    s_status.progress_ticks = (int32_t)(TURN_TARGET_DEG - abs_err);
    publish_status();

    float duty        = turn_next_duty(abs_err);
    float physical    = yaw_err * (float)TURN_YAW_SIGN;
    float signed_duty = physical >= 0.0f ? duty : -duty;

    motor_set_speed(&motor_left,  clamp_duty(-signed_duty, TURN_MIN_DUTY, TURN_MAX_DUTY));
    motor_set_speed(&motor_right, clamp_duty(signed_duty, TURN_MIN_DUTY, TURN_MAX_DUTY));
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
