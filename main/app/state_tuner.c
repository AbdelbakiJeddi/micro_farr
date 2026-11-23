/* TUNER: one page per capability, in tuning order (DISTANCE -> CENTERING -> SPEED -> TURNS
 * -> DUMP). Parameter rows: INC/DEC step the value. Action rows: INC starts the test, DEC
 * stops it. SEL = next row, BOOT = next page, BOOT on DUMP = back to the menu. The last
 * test result is shown on the bottom line. */

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "app_states.h"
#include "motion_client.h"
#include "params.h"
#include "ui.h"

#define LINE_LEN (OLED_COLS + 1)
#define MAX_ROWS 12
#define COUNT(a) ((int)(sizeof(a) / sizeof((a)[0])))

/* Row ids: values below P_COUNT are parameters, the rest are tuner rows. */
enum {
    ROW_CELLS = P_COUNT,
    ROW_PROFILE,
    ROW_CRUISE,   /* cruise / accel / decel of the selected speed profile */
    ROW_ACCEL,
    ROW_DECEL,
};

typedef struct {
    const char *label;
    bool      (*run)(int cells);   /* true = a move was started */
} tuner_action_t;

typedef struct {
    const char           *title;
    const int            *rows;
    int                   n;
    const tuner_action_t *actions;
    int                   n_actions;
    void                (*format)(const move_report_t *r, char *buf, size_t len);
} tuner_page_t;

typedef struct {
    const char *label;
    void      (*value)(char *buf, size_t len);
    void      (*step)(int dir);
} special_row_t;

static int             s_cells   = 1;
static speed_profile_t s_profile = PROFILE_EXPLORE;
static int32_t         s_target_ticks;
static char            s_result[LINE_LEN];

/* ---- special rows ---- */

static const char *const s_profile_name[PROFILE_COUNT] = {
    [PROFILE_EXPLORE] = "EXP",
    [PROFILE_FAST]    = "FAST",
};

static void cells_value(char *buf, size_t len)
{
    snprintf(buf, len, "%d", s_cells);
}

static void cells_step(int dir)
{
    s_cells += dir;
    if (s_cells < 1) {
        s_cells = 1;
    }
    if (s_cells > TUNER_CELLS_MAX) {
        s_cells = TUNER_CELLS_MAX;
    }
}

static void profile_value(char *buf, size_t len)
{
    snprintf(buf, len, "%s", s_profile_name[s_profile]);
}

static void profile_step(int dir)
{
    (void)dir;
    s_profile = (speed_profile_t)((s_profile + 1) % PROFILE_COUNT);
}

static const special_row_t s_special[] = {
    [ROW_CELLS - P_COUNT]   = { "CELLS",   cells_value,   cells_step   },
    [ROW_PROFILE - P_COUNT] = { "PROFILE", profile_value, profile_step },
};

/* Selected-profile rows map to that profile's parameter. */
static int resolve(int row)
{
    static const param_id_t map[PROFILE_COUNT][3] = {
        [PROFILE_EXPLORE] = { P_EXP_CRUISE,  P_EXP_ACCEL,  P_EXP_DECEL  },
        [PROFILE_FAST]    = { P_FAST_CRUISE, P_FAST_ACCEL, P_FAST_DECEL },
    };
    return (row >= ROW_CRUISE) ? (int)map[s_profile][row - ROW_CRUISE] : row;
}

static void row_text(int row, char *buf, size_t len)
{
    int         id = resolve(row);
    const char *label;
    char        value[12];

    if (id < P_COUNT) {
        label = params_info((param_id_t)id)->label;
        params_format_value((param_id_t)id, value, sizeof(value));
    } else {
        label = s_special[id - P_COUNT].label;
        s_special[id - P_COUNT].value(value, sizeof(value));
    }
    snprintf(buf, len, "%-8.8s %s", label, value);
}

static void row_step(int row, int dir)
{
    int id = resolve(row);

    if (id < P_COUNT) {
        params_step((param_id_t)id, dir);
    } else {
        s_special[id - P_COUNT].step(dir);
    }
}

/* ---- test actions ---- */

static bool start_straight(int cells, speed_profile_t profile)
{
    s_target_ticks = (int32_t)((float)cells * params_get(P_TICKS_PER_CELL));
    return motion_straight(cells, profile) == ESP_OK;
}

static bool act_straight(int cells)
{
    return start_straight(cells, PROFILE_EXPLORE);
}

static bool act_speed(int cells)
{
    return start_straight(cells, s_profile);
}

static bool act_turn_right(int cells)
{
    (void)cells;
    return motion_turn(-90, PROFILE_EXPLORE) == ESP_OK;
}

static bool act_turn_left(int cells)
{
    (void)cells;
    return motion_turn(90, PROFILE_EXPLORE) == ESP_OK;
}

static bool act_turn_180(int cells)
{
    (void)cells;
    return motion_turn(180, PROFILE_EXPLORE) == ESP_OK;
}

static bool act_dump(int cells)
{
    (void)cells;
    params_dump();
    snprintf(s_result, sizeof(s_result), "See UART");
    return false;
}

/* ---- result lines ---- */

static void fmt_distance(const move_report_t *r, char *buf, size_t len)
{
    snprintf(buf, len, "%ld/%ld %s", (long)r->distance_ticks, (long)s_target_ticks,
             ui_result_name(r->result));
}

static void fmt_centering(const move_report_t *r, char *buf, size_t len)
{
    snprintf(buf, len, "L%+.0f/%.0f H%+.1f", r->lat_err_end_mm, r->lat_err_max_mm,
             r->heading_err_deg);
}

static void fmt_speed(const move_report_t *r, char *buf, size_t len)
{
    snprintf(buf, len, "D%.0f V%.0f %.2fs", (float)r->distance_ticks / params_ticks_per_mm(),
             r->peak_speed_mm_s, (float)r->duration_ms / 1000.0f);
}

static void fmt_turn(const move_report_t *r, char *buf, size_t len)
{
    snprintf(buf, len, "H%+.1f %.2fs", r->heading_err_deg, (float)r->duration_ms / 1000.0f);
}

/* ---- pages ---- */

static const int s_distance_rows[]  = { P_TICKS_PER_CELL, P_FRONT_ALIGN_MM, P_FRONT_ALIGN_WIN_MM,
                                        ROW_CELLS };
static const int s_centering_rows[] = { P_YAW_KP, P_TOF_CENTER_KP, P_YAW_MAX_RATE_DPS,
                                        P_REALIGN_RATE, ROW_CELLS };
static const int s_speed_rows[]     = { P_RUN_KFF, P_RUN_KP, P_RUN_KI, P_RUN_MAX_DUTY, ROW_PROFILE,
                                        ROW_CRUISE, ROW_ACCEL, ROW_DECEL, ROW_CELLS };
static const int s_turn_rows[]      = { P_TURN_MAX_RATE, P_TURN_ACCEL, P_TURN_DECEL,
                                        P_TURN_SETTLE_MS };

static const tuner_action_t s_straight_test[] = { { "[RUN]", act_straight } };
static const tuner_action_t s_speed_test[]    = { { "[RUN]", act_speed } };
static const tuner_action_t s_turn_tests[]    = { { "[RUN R]",   act_turn_right },
                                                  { "[RUN L]",   act_turn_left  },
                                                  { "[RUN 180]", act_turn_180   } };
static const tuner_action_t s_dump_action[]   = { { "[DUMP]", act_dump } };

#define PAGE(title, rows, actions, fmt) { title, rows, COUNT(rows), actions, COUNT(actions), fmt }

static const tuner_page_t s_pages[] = {
    PAGE("DISTANCE",  s_distance_rows,  s_straight_test, fmt_distance),
    PAGE("CENTERING", s_centering_rows, s_straight_test, fmt_centering),
    PAGE("SPEED",     s_speed_rows,     s_speed_test,    fmt_speed),
    PAGE("TURNS",     s_turn_rows,      s_turn_tests,    fmt_turn),
    { "DUMP", NULL, 0, s_dump_action, COUNT(s_dump_action), NULL },
};

#define PAGE_COUNT COUNT(s_pages)

static int row_count(const tuner_page_t *p)
{
    return p->n + p->n_actions;
}

static void draw(const tuner_page_t *p, int cursor)
{
    char        text[MAX_ROWS][LINE_LEN];
    const char *rows[MAX_ROWS];
    char        title[LINE_LEN];

    for (int i = 0; i < p->n; i++) {
        row_text(p->rows[i], text[i], LINE_LEN);
        rows[i] = text[i];
    }
    for (int j = 0; j < p->n_actions; j++) {
        rows[p->n + j] = p->actions[j].label;
    }
    snprintf(title, sizeof(title), "== %s ==", p->title);
    ui_draw_list(title, rows, row_count(p), cursor, s_result);
}

typedef struct {
    const tuner_page_t *page;
    int                 cursor;
    TickType_t          last_draw;
} progress_t;

/* While a test runs: progress on the result line, at most every UI_REDRAW_MS. */
static void on_test_poll(void *arg)
{
    progress_t     *pr  = arg;
    TickType_t      now = xTaskGetTickCount();
    motion_status_t st;

    if (now - pr->last_draw < pdMS_TO_TICKS(UI_REDRAW_MS) || !motion_status(&st)) {
        return;
    }
    pr->last_draw = now;
    snprintf(s_result, sizeof(s_result), "RUN %ld/%ld", (long)st.progress_ticks,
             (long)st.target_ticks);
    draw(pr->page, pr->cursor);
}

static void run_test(const tuner_page_t *p, const tuner_action_t *a, int cursor)
{
    move_report_t r;
    progress_t    pr = { .page = p, .cursor = cursor, .last_draw = xTaskGetTickCount() };

    if (!a->run(s_cells)) {
        return;
    }
    ui_wait_move(on_test_poll, &pr, &r);
    if (r.result == RESULT_DONE) {
        p->format(&r, s_result, sizeof(s_result));
    } else {
        ui_format_report(&r, s_result, sizeof(s_result));
    }
}

static void press(const tuner_page_t *p, int cursor, int dir)
{
    if (cursor < p->n) {
        row_step(p->rows[cursor], dir);
    } else if (dir > 0) {
        run_test(p, &p->actions[cursor - p->n], cursor);
    } else {
        motion_stop();
    }
}

app_state_id_t state_tuner_run(app_ctx_t *ctx)
{
    (void)ctx;
    int page   = 0;
    int cursor = 0;

    s_result[0] = '\0';
    draw(&s_pages[page], cursor);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(UI_POLL_MS));
        ui_events_t         ev = ui_poll();
        const tuner_page_t *p  = &s_pages[page];

        if (ev.pressed[BTN_BOOT]) {
            if (page == PAGE_COUNT - 1) {
                return ST_MENU;
            }
            page++;
            cursor      = 0;
            s_result[0] = '\0';
        } else if (ev.pressed[BTN_SEL]) {
            cursor = (cursor + 1) % row_count(p);
        } else if (ev.pressed[BTN_INC] || ev.pressed[BTN_DEC]) {
            press(p, cursor, ev.pressed[BTN_INC] ? +1 : -1);
        } else {
            continue;
        }
        draw(&s_pages[page], cursor);
    }
}
