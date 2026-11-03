#pragma once

#include "driver/gpio.h"

#define PIN_I2C_SDA        GPIO_NUM_8
#define PIN_I2C_SCL        GPIO_NUM_9

#define PIN_ENC_L_A        GPIO_NUM_17
#define PIN_ENC_L_B        GPIO_NUM_18
#define PIN_ENC_R_A        GPIO_NUM_15
#define PIN_ENC_R_B        GPIO_NUM_16

#define PIN_XSHUT_L        GPIO_NUM_6
#define PIN_XSHUT_F        GPIO_NUM_5
#define PIN_XSHUT_R        GPIO_NUM_4

#define PIN_MOTOR_L_PWM    GPIO_NUM_1
#define PIN_MOTOR_L_IN1    GPIO_NUM_2
#define PIN_MOTOR_L_IN2    GPIO_NUM_42

#define PIN_MOTOR_R_PWM    GPIO_NUM_38
#define PIN_MOTOR_R_IN1    GPIO_NUM_39
#define PIN_MOTOR_R_IN2    GPIO_NUM_40

#define PIN_MOTOR_STBY     GPIO_NUM_41
