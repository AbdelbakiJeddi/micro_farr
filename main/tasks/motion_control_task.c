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

static straight_state_t s_straight;
static bool             s_pending_straight;
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

#if !STEER_USE_TOF
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
#endif

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

static int clamp_duty(float duty)
{
    if (duty > MOTION_MAX_DUTY) {
        duty = MOTION_MAX_DUTY;
    }
    if (duty < -MOTION_MAX_DUTY) {
        duty = -MOTION_MAX_DUTY;
    }

    int out = (int)(duty >= 0.0f ? (duty + 0.5f) : (duty - 0.5f));

    if (out > 0 && out < (int)MOTION_MIN_DUTY) {
        out = (int)MOTION_MIN_DUTY;
    }
    if (out < 0 && out > -(int)MOTION_MIN_DUTY) {
        out = -(int)MOTION_MIN_DUTY;
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

static void finish_straight(bool success)
{
    if (!s_straight.active) {
        return;
    }

    s_straight.active = false;
    motors_brake();

    if (success) {
        s_status.progress_ticks = s_straight.target_ticks;
        s_status.state          = MOTION_DONE;
        set_status_note("");
    } else {
        s_status.state = MOTION_FAULT;
        set_status_note("TIMEOUT");
    }
    publish_status();

    app_ipc_set_event(success ? APP_EVENT_SEGMENT_DONE : APP_EVENT_FAULT);
}

static void handle_command(motion_cmd_t cmd)
{
    switch (cmd) {
    case CMD_STRAIGHT_CELL:
        if (s_straight.active) {
            ESP_LOGW(TAG, "straight ignored: segment active");
        } else {
            s_pending_straight = true;
        }
        break;

    case CMD_STOP:
        s_pending_straight = false;
        if (s_straight.active) {
            s_straight.active = false;
            motors_brake();
            s_status.state = MOTION_IDLE;
            set_status_note("STOP");
            publish_status();
        }
        break;

    case CMD_TURN_LEFT:
    case CMD_TURN_RIGHT:
        ESP_LOGW(TAG, "turn command not implemented");
        break;

    default:
        break;
    }
}

#if STEER_USE_TOF
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

static float compute_steer(const sensor_snapshot_t *snapshot)
{
    return clamp_float(TOF_CENTER_KP * tof_lateral_error(snapshot), TOF_CENTER_MAX_CORR);
}
#else
static float compute_steer(const sensor_snapshot_t *snapshot)
{
    float yaw_err = wrap180(s_straight.yaw_ref - snapshot->yaw);
    return clamp_float(YAW_KP * yaw_err, YAW_MAX_CORR);
}
#endif

static void update_straight(const sensor_snapshot_t *snapshot)
{
    uint32_t timeout_ms = SEGMENT_TIMEOUT_MS * (uint32_t)((s_straight.target_ticks + CELL_TICKS - 1) / CELL_TICKS);
    TickType_t elapsed = xTaskGetTickCount() - s_straight.start_tick;
    if (elapsed > pdMS_TO_TICKS(timeout_ms)) {
        ESP_LOGE(TAG, "segment timeout");
        finish_straight(false);
        return;
    }

    int32_t progress = ((snapshot->left_ticks - s_straight.start_left) +
                        (snapshot->right_ticks - s_straight.start_right)) / 2;

    if (progress >= s_straight.target_ticks) {
        ESP_LOGI(TAG, "segment done: progress=%ld", (long)progress);
        finish_straight(true);
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

    float steer = compute_steer(snapshot);

    int left_cmd = clamp_duty(cmd - (float)YAW_STEER_SIGN * steer);
    int right_cmd = clamp_duty(cmd + (float)YAW_STEER_SIGN * steer);

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
        }

        if (s_straight.active && have_snapshot) {
            update_straight(&snapshot);
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
