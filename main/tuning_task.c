#include "tuning_task.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_types.h"
#include "button.h"
#include "motion_client.h"
#include "oled.h"
#include "params.h"
#include "telemetry_task.h"

static const char *TAG = "tuner";

#define TUNE_POLL_MS    50
#define TUNE_TASK_STACK 4096
#define TUNE_TASK_PRIO  3
#define TUNE_TASK_CORE  0
#define DISPLAY_ROWS    7   /* rows 1..7; row 0 is the stage header */

typedef enum {
    STAGE_STRAIGHT = 0,
    STAGE_TURN,
    STAGE_SPEED,
    STAGE_DUMP,
    STAGE_COUNT,
} tuner_stage_t;

typedef enum {
    ROW_PARAM,
    ROW_CELLS,
    ROW_ACTION,
} row_kind_t;

typedef struct {
    row_kind_t  kind;
    param_id_t  param;       /* ROW_PARAM */
    const char *label;       /* ROW_CELLS / ROW_ACTION */
    void      (*run)(void);  /* ROW_ACTION */
} param_entry_t;

#define PARAM_ROW(id)          { ROW_PARAM, (id), NULL, NULL }
#define CELLS_ROW              { ROW_CELLS, P_COUNT, "CELLS", NULL }
#define ACTION_ROW(name, fn)   { ROW_ACTION, P_COUNT, (name), (fn) }

#define CELLS_MAX 16

static int s_cells = 1;

static void dump_all(void);

static void run_straight(void)
{
    motion_straight(1, PROFILE_EXPLORE);
}

static void run_speed(void)
{
    motion_straight(s_cells, PROFILE_EXPLORE);
}

static void run_turn_right(void)
{
    motion_turn(-90, PROFILE_EXPLORE);
}

static void run_turn_left(void)
{
    motion_turn(90, PROFILE_EXPLORE);
}

static void run_turn_180(void)
{
    motion_turn(180, PROFILE_EXPLORE);
}

static const param_entry_t s_straight[] = {
    PARAM_ROW(P_YAW_KP),
    PARAM_ROW(P_YAW_MAX_RATE_DPS),
    PARAM_ROW(P_TOF_CENTER_KP),
    PARAM_ROW(P_RUN_KFF),
    PARAM_ROW(P_RUN_KP),
    PARAM_ROW(P_RUN_KI),
    ACTION_ROW("[RUN]", run_straight),
};

static const param_entry_t s_turn[] = {
    PARAM_ROW(P_TURN_MAX_RATE),
    PARAM_ROW(P_TURN_ACCEL),
    PARAM_ROW(P_TURN_DECEL),
    PARAM_ROW(P_TURN_SETTLE_MS),
    ACTION_ROW("[RUN R]", run_turn_right),
    ACTION_ROW("[RUN L]", run_turn_left),
    ACTION_ROW("[RUN 180]", run_turn_180),
};

static const param_entry_t s_speed[] = {
    CELLS_ROW,
    PARAM_ROW(P_EXP_CRUISE),
    PARAM_ROW(P_EXP_ACCEL),
    PARAM_ROW(P_EXP_DECEL),
    PARAM_ROW(P_RUN_MAX_DUTY),
    ACTION_ROW("[RUN]", run_speed),
};

static const param_entry_t s_dump[] = {
    ACTION_ROW("[DUMP]", dump_all),
};

typedef struct {
    const char          *name;
    const param_entry_t *entries;
    int                  count;
} stage_def_t;

#define STAGE(name, rows) { name, rows, (int)(sizeof(rows) / sizeof(rows[0])) }

static const stage_def_t s_stages[STAGE_COUNT] = {
    STAGE("STRAIGHT", s_straight),
    STAGE("TURN",     s_turn),
    STAGE("SPEED",    s_speed),
    STAGE("DUMP",     s_dump),
};

static tuner_stage_t s_stage  = STAGE_STRAIGHT;
static int           s_cursor = 0;
static int           s_scroll = 0;

static void clamp_scroll(void)
{
    int count = s_stages[s_stage].count;

    if (s_cursor < s_scroll) {
        s_scroll = s_cursor;
    }
    if (s_cursor >= s_scroll + DISPLAY_ROWS) {
        s_scroll = s_cursor - DISPLAY_ROWS + 1;
    }
    if (s_scroll < 0) {
        s_scroll = 0;
    }
    int max_scroll = count - DISPLAY_ROWS;
    if (max_scroll < 0) {
        max_scroll = 0;
    }
    if (s_scroll > max_scroll) {
        s_scroll = max_scroll;
    }
}

static const char *entry_label(const param_entry_t *e)
{
    return e->kind == ROW_PARAM ? params_info(e->param)->label : e->label;
}

static void format_value(char *buf, size_t len, const param_entry_t *e)
{
    if (e->kind == ROW_PARAM) {
        params_format_value(e->param, buf, len);
    } else {
        snprintf(buf, len, "%d", s_cells);
    }
}

static void display_refresh(void)
{
    if (!oled_ready()) {
        return;
    }

    const stage_def_t   *stage   = &s_stages[s_stage];
    const param_entry_t *entries = stage->entries;

    oled_clear();

    char hdr[22];
    snprintf(hdr, sizeof(hdr), "== %s ==", stage->name);
    oled_text_line(0, hdr);

    int visible = stage->count;
    if (visible > DISPLAY_ROWS) {
        visible = DISPLAY_ROWS;
    }

    for (int i = 0; i < visible; i++) {
        int idx = s_scroll + i;
        if (idx >= stage->count) {
            break;
        }

        const param_entry_t *e   = &entries[idx];
        char                 buf[22];

        if (e->kind == ROW_ACTION) {
            snprintf(buf, sizeof(buf), "%c %s",
                     (idx == s_cursor) ? '>' : ' ', e->label);
        } else {
            char val[12];
            format_value(val, sizeof(val), e);
            snprintf(buf, sizeof(buf), "%c%-8.8s%s",
                     (idx == s_cursor) ? '>' : ' ', entry_label(e), val);
        }

        oled_text_line((uint8_t)(i + 1), buf);
    }

    oled_flush();
}

static void dump_all(void)
{
    params_dump();

    oled_clear();
    oled_text_line(0, "== DUMP ==");
    oled_text_line(2, "See UART");
    oled_text_line(4, "for values");
    oled_flush();
}

static bool act_on_entry(const param_entry_t *e, bool increment)
{
    int dir = increment ? +1 : -1;

    if (e->kind == ROW_ACTION) {
        e->run();
        return false;
    }
    if (e->kind == ROW_CELLS) {
        s_cells += dir;
        if (s_cells < 1) s_cells = 1;
        if (s_cells > CELLS_MAX) s_cells = CELLS_MAX;
        return true;
    }
    params_step(e->param, dir);
    return true;
}

static bool s_last[4];

#define BTN_BOOT 0
#define BTN_SEL  1
#define BTN_INC  2
#define BTN_DEC  3

static bool btn_edge(int idx, bool current)
{
    bool edge   = current && !s_last[idx];
    s_last[idx] = current;
    return edge;
}

static void tuning_task(void *arg)
{
    (void)arg;
    display_refresh();

    while (1) {
        bool boot = btn_edge(BTN_BOOT, button_boot_pressed());
        bool sel  = btn_edge(BTN_SEL,  button_sel_pressed());
        bool inc  = btn_edge(BTN_INC,  button_inc_pressed());
        bool dec  = btn_edge(BTN_DEC,  button_dec_pressed());
        bool dirty = false;

        if (boot) {
            s_stage  = (tuner_stage_t)((s_stage + 1) % STAGE_COUNT);
            s_cursor = 0;
            s_scroll = 0;
            dirty    = true;
        }

        if (sel) {
            s_cursor = (s_cursor + 1) % s_stages[s_stage].count;
            clamp_scroll();
            dirty = true;
        }

        if (inc) {
            dirty = act_on_entry(&s_stages[s_stage].entries[s_cursor], true);
        }

        if (dec) {
            dirty = act_on_entry(&s_stages[s_stage].entries[s_cursor], false);
        }

        if (dirty) {
            display_refresh();
        }

        vTaskDelay(pdMS_TO_TICKS(TUNE_POLL_MS));
    }
}

void tuning_task_start(void)
{
    button_init();
    telemetry_set_oled_active(false);

    BaseType_t ok = xTaskCreatePinnedToCore(tuning_task, "tuner",
                                            TUNE_TASK_STACK, NULL,
                                            TUNE_TASK_PRIO, NULL,
                                            TUNE_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "tuner task create failed");
    }
}
