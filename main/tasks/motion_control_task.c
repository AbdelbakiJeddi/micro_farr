#include "motion_control_task.h"

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
    float      current_duty;
    TickType_t start_tick;
} straight_state_t;

typedef enum {
    TURN_DRIVE,
    TURN_SETTLE,
} turn_phase_t;

typedef struct {
    bool         active;
    turn_phase_t phase;
    float        start_yaw;
    float        target_yaw;
    float        current_duty;
    float        last_yaw;
    float        rate_dps;
    uint32_t     last_ms;
    TickType_t   start_tick;
    TickType_t   settle_tick;
} turn_state_t;

static straight_state_t s_straight;
static turn_state_t     s_turn;
static bool             s_pending_straight;
static bool             s_pending_turn;
static bool             s_pending_turn_left;
static int32_t          s_target_ticks = CELL_TICKS;

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

static void begin_straight(const sensor_snapshot_t *snapshot)
{
    s_straight.active       = true;
    s_straight.start_left   = snapshot->left_ticks;
    s_straight.start_right  = snapshot->right_ticks;
    s_straight.target_ticks = s_target_ticks > 0 ? s_target_ticks : CELL_TICKS;
    s_straight.yaw_ref      = snapshot->yaw;
    s_straight.current_duty = MOTION_MIN_DUTY;
    s_straight.start_tick   = xTaskGetTickCount();

    s_status.state          = MOTION_RUNNING;
    s_status.progress_ticks = 0;
    s_status.target_ticks   = s_straight.target_ticks;
    set_status_note("");
    publish_status();

    motor_enable(true);
}

static void finish_segment(bool success, const char *fault_note)
{
    s_straight.active = false;
    s_turn.active     = false;
    motors_brake();

    if (success) {
        s_status.state = MOTION_DONE;
        set_status_note("");
    } else {
        s_status.state = MOTION_FAULT;
        set_status_note(fault_note);
    }
    publish_status();

    app_ipc_set_event(success ? APP_EVENT_SEGMENT_DONE : APP_EVENT_FAULT);
}

static void begin_turn(const sensor_snapshot_t *snapshot, bool left)
{
    float direction = left ? 1.0f : -1.0f;

    s_turn.active       = true;
    s_turn.phase        = TURN_DRIVE;
    s_turn.start_yaw    = snapshot->yaw;
    s_turn.target_yaw   = wrap180(snapshot->yaw +
                                  direction * TURN_TARGET_DEG * (float)TURN_YAW_SIGN);
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

static void update_turn(const sensor_snapshot_t *snapshot)
{
    TickType_t now = xTaskGetTickCount();

    if (now - s_turn.start_tick > pdMS_TO_TICKS(TURN_TIMEOUT_MS)) {
        ESP_LOGE(TAG, "turn timeout");
        finish_segment(false, "TURN TIMEOUT");
        return;
    }

    if (snapshot->timestamp_ms != s_turn.last_ms) {
        float dt_s = (float)(snapshot->timestamp_ms - s_turn.last_ms) / 1000.0f;
        if (dt_s > 0.0f) {
            s_turn.rate_dps = (snapshot->yaw - s_turn.last_yaw) / dt_s;
        }
        s_turn.last_yaw = snapshot->yaw;
        s_turn.last_ms  = snapshot->timestamp_ms;
    }

    float yaw_err = wrap180(s_turn.target_yaw - snapshot->yaw);
    float abs_err = yaw_err < 0.0f ? -yaw_err : yaw_err;

    if (s_turn.phase == TURN_SETTLE) {
        if (now - s_turn.settle_tick < pdMS_TO_TICKS(TURN_SETTLE_MS)) {
            return;
        }
        if (abs_err <= TURN_TOL_DEG) {
            ESP_LOGI(TAG, "turn done: err=%.2f", abs_err);
            s_status.progress_ticks = (int32_t)TURN_TARGET_DEG;
            finish_segment(true, NULL);
            return;
        }

        /* Coasted past the target: resume a low-speed correction. */
        s_turn.phase        = TURN_DRIVE;
        s_turn.current_duty = TURN_MIN_DUTY;
        s_turn.rate_dps     = 0.0f;
    }

    /* Predict how far the robot will coast after braking and stop early. */
    float coast_deg = s_turn.rate_dps * ((float)TURN_COAST_MS / 1000.0f);
    float projected = yaw_err - coast_deg;
    bool  will_reach = (yaw_err > 0.0f && projected <= TURN_TOL_DEG) ||
                       (yaw_err < 0.0f && projected >= -TURN_TOL_DEG);

    if (abs_err <= TURN_TOL_DEG || will_reach) {
        motors_brake();
        s_turn.phase        = TURN_SETTLE;
        s_turn.settle_tick  = now;
        s_turn.rate_dps     = 0.0f;
        s_status.progress_ticks = (int32_t)(TURN_TARGET_DEG - abs_err);
        publish_status();
        return;
    }

    s_status.progress_ticks = (int32_t)(TURN_TARGET_DEG - abs_err);
    publish_status();

    float target_cmd = TURN_FF_DUTY + TURN_KP * abs_err;
    if (target_cmd > TURN_MAX_DUTY) {
        target_cmd = TURN_MAX_DUTY;
    }

    if (s_turn.current_duty < target_cmd) {
        s_turn.current_duty += TURN_ACCEL_SLEW;
        if (s_turn.current_duty > target_cmd) {
            s_turn.current_duty = target_cmd;
        }
    } else {
        s_turn.current_duty = target_cmd;
    }

    float physical = yaw_err * (float)TURN_YAW_SIGN;
    float signed_duty = physical >= 0.0f ? s_turn.current_duty : -s_turn.current_duty;

    motor_set_speed(&motor_left, clamp_duty(-signed_duty, TURN_MIN_DUTY, TURN_MAX_DUTY));
    motor_set_speed(&motor_right, clamp_duty(signed_duty, TURN_MIN_DUTY, TURN_MAX_DUTY));
}

static bool segment_active(void)
{
    return s_straight.active || s_turn.active;
}

static void handle_command(motion_cmd_t cmd)
{
    switch (cmd) {
    case CMD_STRAIGHT_CELL:
        if (segment_active()) {
            ESP_LOGW(TAG, "straight ignored: segment active");
        } else {
            s_pending_straight = true;
        }
        break;

    case CMD_TURN_LEFT:
    case CMD_TURN_RIGHT:
        if (segment_active()) {
            ESP_LOGW(TAG, "turn ignored: segment active");
        } else {
            s_pending_turn      = true;
            s_pending_turn_left = (cmd == CMD_TURN_LEFT);
        }
        break;

    case CMD_STOP:
        s_pending_straight = false;
        s_pending_turn     = false;
        if (segment_active()) {
            s_straight.active = false;
            s_turn.active     = false;
            motors_brake();
            s_status.state = MOTION_IDLE;
            set_status_note("STOP");
            publish_status();
        }
        break;

    default:
        break;
    }
}

static void update_straight(const sensor_snapshot_t *snapshot)
{
    uint32_t timeout_ms = SEGMENT_TIMEOUT_MS * (uint32_t)((s_straight.target_ticks + CELL_TICKS - 1) / CELL_TICKS);
    TickType_t elapsed = xTaskGetTickCount() - s_straight.start_tick;
    if (elapsed > pdMS_TO_TICKS(timeout_ms)) {
        ESP_LOGE(TAG, "segment timeout");
        finish_segment(false, "TIMEOUT");
        return;
    }

    int32_t progress = ((snapshot->left_ticks - s_straight.start_left) +
                        (snapshot->right_ticks - s_straight.start_right)) / 2;

    if (progress >= s_straight.target_ticks) {
        ESP_LOGI(TAG, "segment done: progress=%ld", (long)progress);
        s_status.progress_ticks = s_straight.target_ticks;
        finish_segment(true, NULL);
        return;
    }

    s_status.state          = MOTION_RUNNING;
    s_status.progress_ticks = progress;
    s_status.target_ticks   = s_straight.target_ticks;
    publish_status();

    float dist_err = (float)s_straight.target_ticks - (float)progress;
    float target_cmd = STRAIGHT_FF_DUTY + DIST_KP * dist_err;
    if (target_cmd > MOTION_MAX_DUTY) {
        target_cmd = MOTION_MAX_DUTY;
    }

    /* Smooth acceleration ramp-up to prevent sudden wheel-spin and current spikes */
    if (s_straight.current_duty < target_cmd) {
        s_straight.current_duty += MOTION_ACCEL_SLEW;
        if (s_straight.current_duty > target_cmd) {
            s_straight.current_duty = target_cmd;
        }
    } else {
        s_straight.current_duty = target_cmd;
    }

    float cmd = s_straight.current_duty;

    float yaw_err = wrap180(s_straight.yaw_ref - snapshot->yaw);
    float steer = clamp_float(YAW_KP * yaw_err, YAW_MAX_CORR);

    int left_cmd  = clamp_duty(cmd - (float)YAW_STEER_SIGN * steer,
                               MOTION_MIN_DUTY, MOTION_MAX_DUTY);
    int right_cmd = clamp_duty(cmd + (float)YAW_STEER_SIGN * steer,
                               MOTION_MIN_DUTY, MOTION_MAX_DUTY);

    motor_set_speed(&motor_left, left_cmd);
    motor_set_speed(&motor_right, right_cmd);
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

        if (s_pending_straight && have_snapshot) {
            begin_straight(&snapshot);
            s_pending_straight = false;
        } else if (s_pending_turn && have_snapshot) {
            begin_turn(&snapshot, s_pending_turn_left);
            s_pending_turn = false;
        }

        if (s_straight.active && have_snapshot) {
            update_straight(&snapshot);
        } else if (s_turn.active && have_snapshot) {
            update_turn(&snapshot);
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

esp_err_t straight_cells(uint8_t cells)
{
    if (cells == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    s_target_ticks = (int32_t)cells * CELL_TICKS;
    app_ipc_clear_event(APP_EVENT_SEGMENT_DONE | APP_EVENT_FAULT);

    if (!app_ipc_send_motion(CMD_STRAIGHT_CELL, pdMS_TO_TICKS(100))) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t straight_one_cell(void)
{
    return straight_cells(1);
}

static esp_err_t request_turn(motion_cmd_t cmd)
{
    app_ipc_clear_event(APP_EVENT_SEGMENT_DONE | APP_EVENT_FAULT);

    if (!app_ipc_send_motion(cmd, pdMS_TO_TICKS(100))) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t turn_left(void)
{
    return request_turn(CMD_TURN_LEFT);
}

esp_err_t turn_right(void)
{
    return request_turn(CMD_TURN_RIGHT);
}
