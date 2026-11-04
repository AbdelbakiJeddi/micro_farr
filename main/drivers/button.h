#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Configure the BOOT button and the external button as pulled-up inputs. */
void button_init(void);

/* True while either button is held down. */
bool button_pressed(void);

/* Block until a button is pressed and released, then wait settle_ms so a hand can clear. */
void button_wait_press(uint32_t settle_ms);
