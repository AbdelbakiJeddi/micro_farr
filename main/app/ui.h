#pragma once

/* OLED + button helpers shared by the app states. Only the app task calls these. */

#include <stdbool.h>
#include <stddef.h>

#include "app_types.h"

typedef enum {
    BTN_BOOT,
    BTN_SEL,
    BTN_INC,
    BTN_DEC,
    BTN_COUNT
} ui_btn_t;

typedef struct {
    bool pressed[BTN_COUNT];   /* press edges since the last poll */
} ui_events_t;

/* Edge-detected; the first poll only primes, so a button held at boot is not a press. */
ui_events_t ui_poll(void);

/* Title on row 0, a scrolling list with a '>' cursor, optional footer on the last row. */
void ui_draw_list(const char *title, const char *const *rows, int count, int cursor,
                  const char *footer);
void ui_message(const char *title, const char *line1, const char *line2);

/* "INC start / DEC back". INC: waits for release + START_DELAY_MS, returns true. */
bool ui_arm_and_wait(const char *title);

/* Block until any button is pressed. */
void ui_wait_any(void);

/* Call while a move runs: DEC sends motion_stop() and returns false. */
bool ui_run_guard(void);

const char *ui_result_name(move_result_t result);
void        ui_format_report(const move_report_t *r, char *buf, size_t len);
