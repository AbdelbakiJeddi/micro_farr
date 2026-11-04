#include "button.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "pins.h"

#define BUTTON_POLL_MS     50
#define BUTTON_DEBOUNCE_MS 50

void button_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_BUTTON) | (1ULL << PIN_BUTTON_EXT),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
}

bool button_pressed(void)
{
    return gpio_get_level(PIN_BUTTON) == BUTTON_ACTIVE_LEVEL ||
           gpio_get_level(PIN_BUTTON_EXT) == BUTTON_ACTIVE_LEVEL;
}

static void wait_while(bool pressed)
{
    while (button_pressed() == pressed) {
        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
    }
}

void button_wait_press(uint32_t settle_ms)
{
    wait_while(true);   /* ignore a button already held at power-on */
    wait_while(false);  /* wait for press */
    vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
    wait_while(true);   /* wait for release */
    vTaskDelay(pdMS_TO_TICKS(settle_ms));
}
