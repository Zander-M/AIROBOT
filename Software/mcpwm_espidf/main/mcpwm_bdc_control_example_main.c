/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"
#include "pinout.h"

static const char *TAG = "example";

// Enable this config,  we will print debug formated string, which in return can be captured and parsed by Serial-Studio
#define SERIAL_STUDIO_DEBUG           CONFIG_SERIAL_STUDIO_DEBUG

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define BDC_PID_LOOP_PERIOD_MS        10   // calculate the motor speed every 10ms

#define BDC_ENCODER_PCNT_HIGH_LIMIT   1000
#define BDC_ENCODER_PCNT_LOW_LIMIT    -1000


#define MAX_SPEED 42 // max number of ticks
#define SPEED_TO_TICK(A) ({A / 100 * MAX_SPEED;})

const unsigned int left_speed =  100;// expected motor speed, in the pulses counted by the rotary encoder
const unsigned int right_speed = 100;// expected motor speed, in the pulses counted by the rotary encoder

typedef struct {
    unsigned int speed;
    bool dir;
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl;
    int report_pulses;
} motor_control_context_t;

static void pid_loop_cb(void *args)
{
    static int last_pulse_count = 0;
    motor_control_context_t *ctx = (motor_control_context_t *)args;
    pcnt_unit_handle_t pcnt_unit = ctx->pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl = ctx->pid_ctrl;
    bdc_motor_handle_t motor = ctx->motor;

    // get the result from rotary encoder
    int cur_pulse_count = 0;
    pcnt_unit_get_count(pcnt_unit, &cur_pulse_count);
    int real_pulses = cur_pulse_count - last_pulse_count;
    last_pulse_count = cur_pulse_count;
    ctx->report_pulses = real_pulses;

    // calculate the speed error
    float error = ctx->speed - real_pulses;
    float new_speed = 0;

    // set the new speed
    pid_compute(pid_ctrl, error, &new_speed);
    bdc_motor_set_speed(motor, (uint32_t)new_speed);
}

void app_main(void)
{
    static motor_control_context_t left_motor_ctrl_ctx = {
        .speed = left_speed,
        .pcnt_encoder = NULL,
    };

    static motor_control_context_t right_motor_ctrl_ctx = {
        .speed = right_speed,
        .pcnt_encoder = NULL,
    };

    ESP_LOGI(TAG, "Create DC motor");
    bdc_motor_config_t left_motor_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = ME1A,
        .pwmb_gpio_num = ME1B,
    };

    bdc_motor_config_t right_motor_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = ME2A,
        .pwmb_gpio_num = ME2B,
    };

    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };

    bdc_motor_handle_t left_motor = NULL;
    bdc_motor_handle_t right_motor = NULL;

    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&left_motor_config, &mcpwm_config, &left_motor));
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&right_motor_config, &mcpwm_config, &right_motor));

    left_motor_ctrl_ctx.motor = left_motor;
    right_motor_ctrl_ctx.motor = right_motor;

    ESP_LOGI(TAG, "Init pcnt driver to decode rotary signal");

    pcnt_unit_config_t left_unit_config = {
        .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true, // enable counter accumulation
    };

    pcnt_unit_config_t right_unit_config = {
        .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true, // enable counter accumulation
    };
    pcnt_unit_handle_t left_pcnt_unit = NULL;
    pcnt_unit_handle_t right_pcnt_unit = NULL;

    ESP_ERROR_CHECK(pcnt_new_unit(&left_unit_config, &left_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_new_unit(&right_unit_config, &right_pcnt_unit));
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(left_pcnt_unit, &filter_config));
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(right_pcnt_unit, &filter_config));

    pcnt_chan_config_t left_chan_a_config = {
        .edge_gpio_num = E1A,
        .level_gpio_num = E1B,
    };

    pcnt_chan_config_t right_chan_a_config = {
        .edge_gpio_num = E2A,
        .level_gpio_num = E2B,
    };

    pcnt_channel_handle_t left_pcnt_chan_a = NULL;
    pcnt_channel_handle_t right_pcnt_chan_a = NULL;

    ESP_ERROR_CHECK(pcnt_new_channel(left_pcnt_unit, &left_chan_a_config, &left_pcnt_chan_a));
    ESP_ERROR_CHECK(pcnt_new_channel(right_pcnt_unit, &right_chan_a_config, &right_pcnt_chan_a));

    pcnt_chan_config_t left_chan_b_config = {
        .edge_gpio_num = E1B,
        .level_gpio_num = E1A,
    };

    pcnt_chan_config_t right_chan_b_config = {
        .edge_gpio_num = E2B,
        .level_gpio_num = E2A,
    };

    pcnt_channel_handle_t left_pcnt_chan_b = NULL;
    pcnt_channel_handle_t right_pcnt_chan_b = NULL;

    ESP_ERROR_CHECK(pcnt_new_channel(left_pcnt_unit, &left_chan_b_config, &left_pcnt_chan_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(left_pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(left_pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(left_pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(left_pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(left_pcnt_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(left_pcnt_unit, BDC_ENCODER_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(left_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(left_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(left_pcnt_unit));
    left_motor_ctrl_ctx.pcnt_encoder = left_pcnt_unit;

    ESP_ERROR_CHECK(pcnt_new_channel(right_pcnt_unit, &right_chan_b_config, &right_pcnt_chan_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(right_pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(right_pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(right_pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(right_pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(right_pcnt_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(right_pcnt_unit, BDC_ENCODER_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(right_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(right_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(right_pcnt_unit));
    right_motor_ctrl_ctx.pcnt_encoder = right_pcnt_unit;


    ESP_LOGI(TAG, "Create PID control block");
    pid_ctrl_parameter_t pid_runtime_param = {
        .kp = 0.6,
        .ki = 0.4,
        .kd = 0.2,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output   = BDC_MCPWM_DUTY_TICK_MAX - 1,
        .min_output   = 0,
        .max_integral = 1000,
        .min_integral = -1000,
    };
    pid_ctrl_block_handle_t pid_ctrl = NULL;
    pid_ctrl_config_t pid_config = {
        .init_param = pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &pid_ctrl));
    left_motor_ctrl_ctx.pid_ctrl = pid_ctrl;
    right_motor_ctrl_ctx.pid_ctrl = pid_ctrl;

    ESP_LOGI(TAG, "Create a timer to do PID calculation periodically");
    const esp_timer_create_args_t left_periodic_timer_args = {
        .callback = pid_loop_cb,
        .arg = &left_motor_ctrl_ctx,
        .name = "left_pid_loop"
    };

    const esp_timer_create_args_t right_periodic_timer_args = {
        .callback = pid_loop_cb,
        .arg = &right_motor_ctrl_ctx,
        .name = "right_pid_loop"
    };

    esp_timer_handle_t left_pid_loop_timer = NULL;
    esp_timer_handle_t right_pid_loop_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&left_periodic_timer_args, &left_pid_loop_timer));
    ESP_ERROR_CHECK(esp_timer_create(&right_periodic_timer_args, &right_pid_loop_timer));

    ESP_LOGI(TAG, "Enable motor");
    ESP_ERROR_CHECK(bdc_motor_enable(left_motor));
    ESP_ERROR_CHECK(bdc_motor_enable(right_motor));

    ESP_LOGI(TAG, "Forward motor");
    ESP_ERROR_CHECK(bdc_motor_forward(left_motor));
    ESP_ERROR_CHECK(bdc_motor_forward(right_motor));

    ESP_LOGI(TAG, "Start motor speed loop");
    ESP_ERROR_CHECK(esp_timer_start_periodic(left_pid_loop_timer, BDC_PID_LOOP_PERIOD_MS * 1000));
    ESP_ERROR_CHECK(esp_timer_start_periodic(right_pid_loop_timer, BDC_PID_LOOP_PERIOD_MS * 1000));

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
        // the following logging format is according to the requirement of serial-studio frame format
        // also see the dashboard config file `serial-studio-dashboard.json` for more information
#if SERIAL_STUDIO_DEBUG
        printf("/*%d*/\r\n", left_motor_ctrl_ctx.report_pulses);
        printf("/*%d*/\r\n", right_motor_ctrl_ctx.report_pulses);
#endif
    }
}
