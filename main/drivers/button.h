#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Configure all buttons as pulled-up inputs. */
void button_init(void);

/* True while BOOT or SEL is held (maze start compatibility). */
bool button_pressed(void);

/* Per-button state queries. */
bool button_boot_pressed(void);
bool button_sel_pressed(void);
bool button_inc_pressed(void);
bool button_dec_pressed(void);

/* Block until a button is pressed and released, then wait settle_ms so a hand can clear. */
void button_wait_press(uint32_t settle_ms);
