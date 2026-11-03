#include "encoder.h"
#include "pins.h"
#include "app_config.h"

#include "esp_err.h"

encoder_t encoder_left  = { PIN_ENC_L_A, PIN_ENC_L_B, NULL, 0 };
encoder_t encoder_right = { PIN_ENC_R_A, PIN_ENC_R_B, NULL, 0 };

static void encoder_enable_pullups(const encoder_t *e)
{
    gpio_pullup_en(e->ch_a);
    gpio_pullup_en(e->ch_b);
}

static void encoder_set_quadrature(pcnt_channel_handle_t chan, bool primary)
{
    if (primary) {
        pcnt_channel_set_edge_action(chan,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    } else {
        pcnt_channel_set_edge_action(chan,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    }
    pcnt_channel_set_level_action(chan,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
}

static void encoder_create_channels(encoder_t *e)
{
    pcnt_channel_handle_t chan_a = NULL;
    pcnt_channel_handle_t chan_b = NULL;

    pcnt_chan_config_t cfg_a = {
        .edge_gpio_num  = e->ch_a,
        .level_gpio_num = e->ch_b,
    };
    pcnt_chan_config_t cfg_b = {
        .edge_gpio_num  = e->ch_b,
        .level_gpio_num = e->ch_a,
    };

    ESP_ERROR_CHECK(pcnt_new_channel(e->unit, &cfg_a, &chan_a));
    ESP_ERROR_CHECK(pcnt_new_channel(e->unit, &cfg_b, &chan_b));

    encoder_set_quadrature(chan_a, true);
    encoder_set_quadrature(chan_b, false);
}

static void encoder_create_unit(encoder_t *e)
{
    pcnt_unit_config_t cfg = {
        .high_limit        = ENCODER_HIGH_LIMIT,
        .low_limit         = ENCODER_LOW_LIMIT,
        .flags.accum_count = 1,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&cfg, &e->unit));

    pcnt_glitch_filter_config_t filter = {
        .max_glitch_ns = ENCODER_GLITCH_NS,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(e->unit, &filter));
}

static void encoder_start(encoder_t *e)
{
    ESP_ERROR_CHECK(pcnt_unit_enable(e->unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(e->unit));
    ESP_ERROR_CHECK(pcnt_unit_start(e->unit));
    e->last_count = 0;
}

static void encoder_setup(encoder_t *e)
{
    encoder_enable_pullups(e);
    encoder_create_unit(e);
    encoder_create_channels(e);
    encoder_start(e);
}

static int32_t encoder_read(const encoder_t *e)
{
    int count = 0;
    pcnt_unit_get_count(e->unit, &count);
    return count;
}

void encoder_init(void)
{
    encoder_setup(&encoder_left);
    encoder_setup(&encoder_right);
}

int32_t encoder_get_delta(encoder_t *e)
{
    int32_t now = encoder_read(e);
    int32_t delta = now - e->last_count;
    e->last_count = now;
    return delta;
}

int32_t encoder_get_total(const encoder_t *e)
{
    return encoder_read(e);
}
