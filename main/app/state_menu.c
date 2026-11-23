/* MENU: SEL moves the cursor, INC enters the selected program. */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "app_states.h"
#include "ui.h"

typedef struct {
    const char    *label;
    app_state_id_t target;
} menu_item_t;

static const menu_item_t s_items[] = {
    { "EXPLORE",   ST_EXPLORE   },
    { "SPEED RUN", ST_SPEED_RUN },
    { "TUNER",     ST_TUNER     },
};

#define ITEM_COUNT ((int)(sizeof(s_items) / sizeof(s_items[0])))

static void draw(int cursor)
{
    const char *rows[ITEM_COUNT];

    for (int i = 0; i < ITEM_COUNT; i++) {
        rows[i] = s_items[i].label;
    }
    ui_draw_list("== MICRO FARR ==", rows, ITEM_COUNT, cursor, "SEL next  INC enter");
}

app_state_id_t state_menu_run(app_ctx_t *ctx)
{
    (void)ctx;
    int cursor = 0;

    draw(cursor);
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(UI_POLL_MS));
        ui_events_t ev = ui_poll();

        if (ev.pressed[BTN_INC]) {
            return s_items[cursor].target;
        }
        if (ev.pressed[BTN_SEL]) {
            cursor = (cursor + 1) % ITEM_COUNT;
            draw(cursor);
        }
    }
}
