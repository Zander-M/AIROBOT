/*
    Motor driver
*/
#include <stdio.h>

#include "motor_control.h"
#include <geometry_msgs/msg/twist.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <math.h>

void setupPins() {

    // Led. Set it to GPIO_MODE_INPUT_OUTPUT, because we want to read back the state we set it to.
    gpio_reset_pin(LED_BUILTIN);
    gpio_set_direction(LED_BUILTIN, GPIO_MODE_INPUT_OUTPUT);

    // Configure timer
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQUENCY,
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    // Configure 4 PWM channels and assign output pins
    ledc_channel_config_t ledc_channel[4] = {
        {
            .channel    = PWM_LEFT_FORWARD,
            .duty       = 0,
            .gpio_num   = PIN_LEFT_FORWARD,
            .speed_mode = PWM_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_1
        },
        {
            .channel    = PWM_LEFT_BACKWARD,
            .duty       = 0,
            .gpio_num   = PIN_LEFT_BACKWARD,
            .speed_mode = PWM_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_1
        },
        {
            .channel    = PWM_RIGHT_FORWARD,
            .duty       = 0,
            .gpio_num   = PIN_RIGHT_FORWARD,
            .speed_mode = PWM_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_1
        },
        {
            .channel    = PWM_RIGHT_BACKWARD,
            .duty       = 0,
            .gpio_num   = PIN_RIGHT_BACKWARD,
            .speed_mode = PWM_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_1
        },
    };

    for (int i = 0; i < 4; i++) {
        ledc_channel_config(&ledc_channel[i]);
    }
}

void setMotorFromTwist(geometry_msgs__msg__Twist* msg) {
    float linear = constrain(msg->linear.x, -1, 1);
    float angular = constrain(msg->angular.z, -1, 1);

    float left = (linear - angular) / 2.0f;
    float right = (linear + angular) / 2.0f;

    // Then map those values to PWM intensities. PWM_MOTOR_MAX = full speed, PWM_MOTOR_MIN = the minimal amount of power at which the motors begin moving.
    uint16_t pwmLeft = (uint16_t) fmap(fabs(left), 0, 1, PWM_MOTOR_MIN, PWM_MOTOR_MAX);
    uint16_t pwmRight = (uint16_t) fmap(fabs(right), 0, 1, PWM_MOTOR_MIN, PWM_MOTOR_MAX);

    // Each wheel has a channel for forwards and backwards movement
    ledc_set_duty(PWM_MODE, PWM_LEFT_FORWARD, pwmLeft * (left > 0));
    ledc_set_duty(PWM_MODE, PWM_LEFT_BACKWARD, pwmLeft * (left < 0));
    ledc_set_duty(PWM_MODE, PWM_RIGHT_FORWARD, pwmRight * (right > 0));
    ledc_set_duty(PWM_MODE, PWM_RIGHT_BACKWARD, pwmRight * (right < 0));

    ledc_update_duty(PWM_MODE, PWM_LEFT_FORWARD);
    ledc_update_duty(PWM_MODE, PWM_LEFT_BACKWARD);
    ledc_update_duty(PWM_MODE, PWM_RIGHT_FORWARD);
    ledc_update_duty(PWM_MODE, PWM_RIGHT_BACKWARD);

    printf("%d, %d %d, %d, %d %d, %f, %f\n", pwmLeft, left > 0, left < 0, pwmRight, right > 0, right < 0, left, right);
}

float fmap(float val, float in_min, float in_max, float out_min, float out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}