#include "tuning_task.h"
#include "tuning_params.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_ipc.h"
#include "app_types.h"
#include "button.h"
#include "motion_control_task.h"
#include "oled.h"
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

typedef struct {
    const char *name;
    float      *value;
    float       step;
} param_entry_t;

#define RUN_ENTRY { "[RUN]", NULL, 0.0f }

static const param_entry_t s_straight[] = {
    { "YAW_KP",  &g_yaw_kp,           1.0f   },
    { "YAW_MAX", &g_yaw_max_rate_dps,  5.0f   },
    { "TOF_KP",  &g_tof_center_kp,     0.1f   },
    { "RUN_KFF", &g_run_kff,           0.001f },
    { "RUN_KP",  &g_run_kp,            0.001f },
    { "RUN_KI",  &g_run_ki,            0.005f },
    RUN_ENTRY,
};

static const param_entry_t s_turn[] = {
    { "TURN_KP",  &g_turn_kp,         0.05f  },
    { "TURN_FF",  &g_turn_ff_duty,     0.5f   },
    { "TURN_MAX", &g_turn_max_duty,    1.0f   },
    { "SLEW",     &g_turn_accel_slew,  0.05f  },
    { "SETTL_MS", &g_turn_settle_ms,  10.0f   },
    RUN_ENTRY,
};

static const param_entry_t s_speed[] = {
    { "CELLS",   &g_run_cells,         1.0f   },
    { "CRUISE",  &g_run_cruise_mm_s,  10.0f  },
    { "ACCEL",   &g_run_accel_mm_s2, 100.0f  },
    { "DECEL",   &g_run_decel_mm_s2, 100.0f  },
    { "MAX_D",   &g_run_max_duty,      1.0f   },
    RUN_ENTRY,
};

static const param_entry_t s_dump[] = {
    { "[DUMP]", NULL, 0.0f },
};

typedef struct {
    const char         *name;
    const param_entry_t *entries;
    int                  count;
    motion_cmd_t         run_cmd;
} stage_def_t;

static const stage_def_t s_stages[STAGE_COUNT] = {
    { "STRAIGHT", s_straight, (int)(sizeof(s_straight) / sizeof(s_straight[0])), CMD_STRAIGHT_CELL },
    { "TURN",     s_turn,     (int)(sizeof(s_turn)     / sizeof(s_turn[0])),     CMD_TURN_RIGHT    },
    { "SPEED",    s_speed,    (int)(sizeof(s_speed)    / sizeof(s_speed[0])),    CMD_STRAIGHT_CELL },
    { "DUMP",     s_dump,     (int)(sizeof(s_dump)     / sizeof(s_dump[0])),     CMD_STOP          },
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

static void format_value(char *buf, size_t len, const param_entry_t *e)
{
    float v = *e->value;
    if (e->step >= 1.0f) {
        snprintf(buf, len, "%.0f", v);
    } else if (e->step >= 0.1f) {
        snprintf(buf, len, "%.1f", v);
    } else if (e->step >= 0.01f) {
        snprintf(buf, len, "%.2f", v);
    } else {
        snprintf(buf, len, "%.3f", v);
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

        if (e->value == NULL) {
            snprintf(buf, sizeof(buf), "%c %s",
                     (idx == s_cursor) ? '>' : ' ', e->name);
        } else {
            char val[12];
            format_value(val, sizeof(val), e);
            snprintf(buf, sizeof(buf), "%c%-8.8s%s",
                     (idx == s_cursor) ? '>' : ' ', e->name, val);
        }

        oled_text_line((uint8_t)(i + 1), buf);
    }

    oled_flush();
}

static void dump_all(void)
{
    printf("\n--- TUNED GAINS ---\n");
    printf("#define YAW_KP           %.3ff\n", g_yaw_kp);
    printf("#define YAW_MAX_RATE_DPS %.1ff\n", g_yaw_max_rate_dps);
    printf("#define TOF_CENTER_KP    %.3ff\n", g_tof_center_kp);
    printf("#define RUN_KFF          %.4ff\n", g_run_kff);
    printf("#define RUN_KP           %.4ff\n", g_run_kp);
    printf("#define RUN_KI           %.4ff\n", g_run_ki);
    printf("#define TURN_KP          %.3ff\n", g_turn_kp);
    printf("#define TURN_FF_DUTY     %.2ff\n", g_turn_ff_duty);
    printf("#define TURN_MAX_DUTY    %.1ff\n", g_turn_max_duty);
    printf("#define TURN_ACCEL_SLEW  %.3ff\n", g_turn_accel_slew);
    printf("#define TURN_SETTLE_MS   %.0ff\n", g_turn_settle_ms);
    printf("#define RUN_CRUISE_MM_S  %.1ff\n", g_run_cruise_mm_s);
    printf("#define RUN_ACCEL_MM_S2  %.1ff\n", g_run_accel_mm_s2);
    printf("#define RUN_DECEL_MM_S2  %.1ff\n", g_run_decel_mm_s2);
    printf("#define RUN_MAX_DUTY     %.1ff\n", g_run_max_duty);
    printf("-------------------\n");
    fflush(stdout);

    oled_clear();
    oled_text_line(0, "== DUMP ==");
    oled_text_line(2, "See UART");
    oled_text_line(4, "for values");
    oled_flush();
}

static bool act_on_entry(const param_entry_t *e, bool increment)
{
    if (e->value == NULL) {
        if (s_stage == STAGE_DUMP) {
            dump_all();
        } else if (s_stage == STAGE_SPEED) {
            uint8_t cells = (uint8_t)g_run_cells;
            if (cells < 1) cells = 1;
            straight_cells(cells);
        } else {
            app_ipc_send_motion(s_stages[s_stage].run_cmd, pdMS_TO_TICKS(100));
        }
        return false;
    }
    *e->value += increment ? e->step : -e->step;
    if (e->value == &g_run_cells) {
        if (*e->value < 1.0f) *e->value = 1.0f;
        if (*e->value > 16.0f) *e->value = 16.0f;
    }
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
