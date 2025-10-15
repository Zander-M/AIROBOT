/*
    Motor Driver for ESP-IDF with Velocity PID
*/
#include <stdio.h>
#include <math.h>
#include <driver/gpio.h>
#include <driver/ledc.h>

#include <esp_log.h>
#include <esp_attr.h>
#include <esp_timer.h>

#include "pinout.h"
#include "wheels.h"

static const char* TAG="WHEELS";

// Wheel Direction
#define LEFT_DIR -1
#define RIGHT_DIR 1

// Encoder Positions 
volatile int l_pos = 0; 
volatile int r_pos = 0; 

// Targets in tics per second 
volatile float l_target_speed = 0;
volatile float r_target_speed = 0;

// Previous encoder counts for velocity calc
int l_prev_count = 0;
int r_prev_count = 0;

// Prev time
int64_t prevT = 0;

// PID state
float l_eprev = 0;
float l_eintegral = 0;
float r_eprev = 0;
float r_eintegral = 0;

// PID parameter
static PIDConfig pid = {1.0f, 0.0f, 0.0f};

int min_power = 50;
int max_power = 255;

// ISR Handlers for encoder reading
static void IRAM_ATTR readLeftEncoder(void* arg){
    int b = gpio_get_level(E1B);
    if (b) l_pos--; else l_pos++;
}

static void IRAM_ATTR readRightEncoder(void* arg){
    int b = gpio_get_level(E2B);
    if (b) r_pos--; else r_pos++;
}

/// @brief pin setups
void wheel_setPID(PIDConfig cfg) {pid = cfg;}

void wheel_init() {
    // GPIO setup
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << E1A) | (1ULL << E2A),
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    gpio_config(&io_conf);

    gpio_config_t io_confB = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << E1B) | (1ULL << E2B),
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    gpio_config(&io_confB);

    // Install ISR service
    gpio_install_isr_service(0);
    gpio_isr_handler_add(E1A, readLeftEncoder, NULL);
    gpio_isr_handler_add(E2A, readRightEncoder, NULL);

    // Configure motor pins
    gpio_set_direction(ME1A, GPIO_MODE_OUTPUT);
    gpio_set_direction(ME1B, GPIO_MODE_OUTPUT);
    gpio_set_direction(ME2A, GPIO_MODE_OUTPUT);
    gpio_set_direction(ME2B, GPIO_MODE_OUTPUT);

    // Configure LEDC for PWM
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    // Configure 4 channels
    ledc_channel_config_t ledc_channel[4] = {
        {
            .channel    = LEDC_CHANNEL_0,
            .duty       = 0,
            .gpio_num   = ME1A,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_0
        },
        {
            .channel    = LEDC_CHANNEL_1,
            .duty       = 0,
            .gpio_num   = ME1B,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_0
        },
        {
            .channel    = LEDC_CHANNEL_2,
            .duty       = 0,
            .gpio_num   = ME2A,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_0
        },
        {
            .channel    = LEDC_CHANNEL_3,
            .duty       = 0,
            .gpio_num   = ME2B,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_0
        }
    };

    for (int i = 0; i < 4; i++) {
        ledc_channel_config(&ledc_channel[i]);
    }

    // Set time
    prevT = esp_timer_get_time();
    ESP_LOGI(TAG, "Whees initialized");
}

void wheel_run() {
    // time difference
    int64_t currT = esp_timer_get_time();
    float dt = (currT - prevT) / 1e6f; // seconds
    prevT = currT;

    // read encoder deltas
    int l_count = l_pos;
    int r_count = r_pos;
    int l_delta = l_count - l_prev_count;
    int r_delta = r_count - r_prev_count;
    l_prev_count = l_count;
    r_prev_count = r_count;

    // measured speed in ticks/s
    float l_speed = l_delta / dt;
    float r_speed = r_delta / dt;

    // error
    float l_err = l_target_speed - l_speed;
    float r_err = r_target_speed - r_speed;

    // integral
    l_eintegral += l_err * dt;
    r_eintegral += r_err * dt;

    // derivative
    float l_deriv = (l_err - l_eprev) / dt;
    float r_deriv = (r_err - r_eprev) / dt;

    // PID control signal
    float l_u = pid.kp*l_err + pid.ki*l_eintegral + pid.kd*l_deriv;
    float r_u = pid.kp*r_err + pid.ki*r_eintegral + pid.kd*r_deriv;

    // store previous error
    l_eprev = l_err;
    r_eprev = r_err;

    // map to PWM duty
    int l_pwm = (int)fminf(fabsf(l_u), max_power);
    int r_pwm = (int)fminf(fabsf(r_u), max_power);

    // apply
    if (l_u >= 0) {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, l_pwm);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
    } else {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, l_pwm);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    }

    if (r_u >= 0) {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, r_pwm);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3, 0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3);
    } else {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3, r_pwm);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, 0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);
    }

    // ESP_LOGI(TAG, "L: tgt=%.2f, meas=%.2f | R: tgt=%.2f, meas=%.2f", 
            //  l_target_speed, l_speed, r_target_speed, r_speed);
}

// setters
void setLeftTarget(float target_ticks_per_s) {
    l_target_speed = LEFT_DIR * target_ticks_per_s;
}

void setRightTarget(float target_ticks_per_s) {
    r_target_speed = RIGHT_DIR * target_ticks_per_s;
}
