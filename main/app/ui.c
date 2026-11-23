#include "ui.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "button.h"
#include "motion_client.h"
#include "oled.h"

#define UI_LINE_LEN (OLED_COLS + 1)

typedef bool (*button_read_fn)(void);

static const button_read_fn s_read[BTN_COUNT] = {
    [BTN_BOOT] = button_boot_pressed,
    [BTN_SEL]  = button_sel_pressed,
    [BTN_INC]  = button_inc_pressed,
    [BTN_DEC]  = button_dec_pressed,
};

static const char *const s_result_name[RESULT_COUNT] = {
    [RESULT_NONE]        = "-",
    [RESULT_DONE]        = "DONE",
    [RESULT_TIMEOUT]     = "TIMEOUT",
    [RESULT_WALL_STOP]   = "WALL",
    [RESULT_SENSOR_LOST] = "SENS LOST",
    [RESULT_STOPPED]     = "STOPPED",
};

static bool s_last[BTN_COUNT];
static bool s_primed;

ui_events_t ui_poll(void)
{
    ui_events_t ev = { { false } };

    for (int i = 0; i < BTN_COUNT; i++) {
        bool now = s_read[i]();

        ev.pressed[i] = s_primed && now && !s_last[i];
        s_last[i]     = now;
    }
    s_primed = true;
    return ev;
}

void ui_draw_list(const char *title, const char *const *rows, int count, int cursor,
                  const char *footer)
{
    if (!oled_ready()) {
        return;
    }

    int  visible = (footer != NULL) ? OLED_PAGES - 2 : OLED_PAGES - 1;
    int  first   = (cursor >= visible) ? cursor - visible + 1 : 0;
    char line[UI_LINE_LEN];

    oled_clear();
    oled_text_line(0, title);
    for (int i = 0; i < visible && first + i < count; i++) {
        snprintf(line, sizeof(line), "%c%s", (first + i == cursor) ? '>' : ' ', rows[first + i]);
        oled_text_line((uint8_t)(1 + i), line);
    }
    if (footer != NULL) {
        oled_text_line(OLED_PAGES - 1, footer);
    }
    oled_flush();
}

void ui_message(const char *title, const char *line1, const char *line2)
{
    if (!oled_ready()) {
        return;
    }
    oled_clear();
    oled_text_line(0, title);
    oled_text_line(3, line1 != NULL ? line1 : "");
    oled_text_line(5, line2 != NULL ? line2 : "");
    oled_flush();
}

bool ui_arm_and_wait(const char *title)
{
    ui_message(title, "INC start", "DEC back");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(UI_POLL_MS));
        ui_events_t ev = ui_poll();
        if (ev.pressed[BTN_DEC]) {
            return false;
        }
        if (ev.pressed[BTN_INC]) {
            break;
        }
    }
    while (s_read[BTN_INC]()) {
        vTaskDelay(pdMS_TO_TICKS(UI_POLL_MS));
    }
    ui_poll();
    vTaskDelay(pdMS_TO_TICKS(START_DELAY_MS));   /* let the hand clear */
    return true;
}

void ui_wait_any(void)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(UI_POLL_MS));
        ui_events_t ev = ui_poll();
        for (int i = 0; i < BTN_COUNT; i++) {
            if (ev.pressed[i]) {
                return;
            }
        }
    }
}

bool ui_run_guard(void)
{
    if (ui_poll().pressed[BTN_DEC]) {
        motion_stop();
        return false;
    }
    return true;
}

const char *ui_result_name(move_result_t result)
{
    return ((unsigned)result < RESULT_COUNT) ? s_result_name[result] : "?";
}

void ui_format_report(const move_report_t *r, char *buf, size_t len)
{
    snprintf(buf, len, "%s H%+.1f %.2fs", ui_result_name(r->result), r->heading_err_deg,
             (float)r->duration_ms / 1000.0f);
}
