#include "motor.h"
#include "pins.h"

#define PWM_FREQ_HZ 20000
#define PWM_RES     LEDC_TIMER_10_BIT
#define PWM_MAX     1023
#define PWM_MODE    LEDC_LOW_SPEED_MODE
#define PWM_TIMER   LEDC_TIMER_0

motor_t motor_left  = { PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, PIN_MOTOR_L_PWM,
                        LEDC_CHANNEL_0, false };
motor_t motor_right = { PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, PIN_MOTOR_R_PWM,
                        LEDC_CHANNEL_1, false };

static int clamp_percent(int percent)
{
    if (percent > 100) return 100;
    if (percent < -100) return -100;
    return percent;
}

static void gpio_init_outputs(gpio_num_t pin_a, gpio_num_t pin_b)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << pin_a) | (1ULL << pin_b),
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
}

static void channel_init(const motor_t *m)
{
    ledc_channel_config_t ch = {
        .gpio_num   = m->pwm_gpio,
        .speed_mode = PWM_MODE,
        .channel    = m->pwm_ch,
        .timer_sel  = PWM_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ledc_channel_config(&ch);
}

static void timer_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = PWM_MODE,
        .timer_num       = PWM_TIMER,
        .duty_resolution = PWM_RES,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);
}

static void motor_set_direction(const motor_t *m, int dir)
{
    gpio_set_level(m->in1, dir > 0);
    gpio_set_level(m->in2, dir < 0);
}

static void motor_set_duty(const motor_t *m, int magnitude)
{
    ledc_set_duty(PWM_MODE, m->pwm_ch, magnitude * PWM_MAX / 100);
    ledc_update_duty(PWM_MODE, m->pwm_ch);
}

void motor_init(void)
{
    timer_init();

    gpio_init_outputs(PIN_MOTOR_STBY, PIN_MOTOR_STBY);
    channel_init(&motor_left);
    channel_init(&motor_right);
    gpio_init_outputs(motor_left.in1, motor_left.in2);
    gpio_init_outputs(motor_right.in1, motor_right.in2);

    motor_enable(false);
    motor_stop(&motor_left);
    motor_stop(&motor_right);
}

void motor_set_speed(motor_t *m, int percent)
{
    percent = clamp_percent(percent);
    if (m->inverted) percent = -percent;

    int magnitude = percent < 0 ? -percent : percent;
    motor_set_direction(m, percent);
    motor_set_duty(m, magnitude);
}

void motor_brake(motor_t *m)
{
    motor_set_duty(m, 0);
    gpio_set_level(m->in1, 1);
    gpio_set_level(m->in2, 1);
}

void motor_stop(motor_t *m)
{
    motor_set_speed(m, 0);
}

void motor_enable(bool on)
{
    gpio_set_level(PIN_MOTOR_STBY, on ? 1 : 0);
}
