#pragma once

#include <stdbool.h>

#include "driver/gpio.h"
#include "driver/ledc.h"

typedef struct {
    gpio_num_t     in1;
    gpio_num_t     in2;
    gpio_num_t     pwm_gpio;
    ledc_channel_t pwm_ch;
    bool           inverted;
} motor_t;

extern motor_t motor_left;
extern motor_t motor_right;

void motor_init(void);
void motor_set_speed(motor_t *m, int percent);
void motor_brake(motor_t *m);
void motor_stop(motor_t *m);
void motor_enable(bool on);
