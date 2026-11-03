#pragma once

#include <stdint.h>

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

typedef struct {
    gpio_num_t         ch_a;
    gpio_num_t         ch_b;
    pcnt_unit_handle_t unit;
    int32_t            last_count;
} encoder_t;

extern encoder_t encoder_left;
extern encoder_t encoder_right;

void    encoder_init(void);
int32_t encoder_get_delta(encoder_t *e);
int32_t encoder_get_total(const encoder_t *e);
