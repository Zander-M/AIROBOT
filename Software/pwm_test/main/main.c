#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

static const char *TAG = "motor_pwm_console";

/* =========================
   USER CONFIG (EDIT THESE)
   ========================= */

// Motor driver pins (typical: one PWM + one DIR)
#define PIN_MOTOR_PWM   18   // <-- PWM output pin (LEDC)
#define PIN_MOTOR_DIR   19   // <-- Direction pin (GPIO)

// Optional motor enable pin (set to -1 if not used)
#define PIN_MOTOR_EN    -1   // e.g. 5 for TB6612 STBY/EN, or -1 to disable

// UART console (usually UART0)
#define CONSOLE_UART_NUM UART_NUM_0
#define CONSOLE_BAUD     115200

// LEDC configuration
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_HIGH_SPEED_MODE
#define LEDC_CHANNEL            LEDC_CHANNEL_0

// Pick a PWM freq your driver likes.
// For DC motors: 15-25 kHz is common to push noise out of audible range.
#define PWM_FREQ_HZ             1000

// Duty resolution: 13-bit => duty range [0..8191]
#define PWM_DUTY_RES            LEDC_TIMER_13_BIT
#define PWM_DUTY_MAX            ((1 << 13) - 1)

/* =========================
   INTERNAL STATE
   ========================= */

static int g_dir = 1;           // 1 = forward, 0 = reverse
static int g_duty = 0;

static void motor_set_dir(int forward)
{
    g_dir = forward ? 1 : 0;
    gpio_set_level(PIN_MOTOR_DIR, g_dir);

    ESP_LOGI(TAG, "Direction: %s", g_dir ? "FORWARD" : "REVERSE");
}

static void motor_set_enable(int enable)
{
    if (PIN_MOTOR_EN >= 0) {
        gpio_set_level(PIN_MOTOR_EN, enable ? 1 : 0);
        ESP_LOGI(TAG, "Enable: %s", enable ? "ON" : "OFF");
    }
}

static void motor_set_duty(int duty)
{
    if (duty < 0) duty = 0;
    if (duty > PWM_DUTY_MAX) duty = PWM_DUTY_MAX;

    g_duty = duty;

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, (uint32_t)g_duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

    ESP_LOGI(TAG, "Duty set: %d / %d (%.1f%%)",
             g_duty, PWM_DUTY_MAX, 100.0f * (float)g_duty / (float)PWM_DUTY_MAX);
}

static void motor_hw_init(void)
{
    // DIR pin
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_MOTOR_DIR),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    // EN pin (optional)
    if (PIN_MOTOR_EN >= 0) {
        gpio_config_t en = {
            .pin_bit_mask = (1ULL << PIN_MOTOR_EN),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        ESP_ERROR_CHECK(gpio_config(&en));
    }

    // LEDC timer
    ledc_timer_config_t tcfg = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = PWM_DUTY_RES,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

    // LEDC channel
    ledc_channel_config_t ccfg = {
        .gpio_num = PIN_MOTOR_PWM,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ccfg));

    // Default states
    motor_set_enable(1);
    motor_set_dir(1);
    motor_set_duty(0);

    ESP_LOGI(TAG, "Motor HW init done. PWM=%d Hz, duty range [0..%d]",
             PWM_FREQ_HZ, PWM_DUTY_MAX);
}

static void console_init(void)
{
    uart_config_t cfg = {
        .baud_rate = CONSOLE_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    ESP_ERROR_CHECK(uart_driver_install(CONSOLE_UART_NUM, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CONSOLE_UART_NUM, &cfg));
}

static void print_help(void)
{
    printf("\nCommands:\n");
    printf("  <number>     Set PWM duty (0..%d). Example: 1200\n", PWM_DUTY_MAX);
    printf("  f            Direction forward\n");
    printf("  r            Direction reverse\n");
    printf("  stop         Duty = 0\n");
    printf("  status       Print current state\n");
    printf("  help         Print this help\n\n");
}

static void handle_line(const char *line)
{
    // Trim leading spaces
    while (*line && isspace((unsigned char)*line)) line++;

    if (*line == '\0') return;

    if (!strcmp(line, "help")) {
        print_help();
        return;
    }
    if (!strcmp(line, "stop")) {
        motor_set_duty(0);
        return;
    }
    if (!strcmp(line, "status")) {
        printf("STATUS: dir=%s, duty=%d/%d (%.1f%%)\n",
               g_dir ? "FORWARD" : "REVERSE",
               g_duty, PWM_DUTY_MAX, 100.0f * (float)g_duty / (float)PWM_DUTY_MAX);
        return;
    }
    if (!strcmp(line, "f")) {
        motor_set_dir(1);
        return;
    }
    if (!strcmp(line, "r")) {
        motor_set_dir(0);
        return;
    }

    // Otherwise, parse as integer duty
    char *end = NULL;
    long val = strtol(line, &end, 10);
    if (end == line) {
        printf("Unknown command: '%s' (type 'help')\n", line);
        return;
    }
    motor_set_duty((int)val);
}

static void console_task(void *arg)
{
    (void)arg;
    uint8_t c;
    char buf[128];
    int pos = 0;

    print_help();
    printf("Type a duty value and press Enter.\n");

    while (1) {
        int n = uart_read_bytes(CONSOLE_UART_NUM, &c, 1, portMAX_DELAY);
        if (n <= 0) continue;

        if (c == '\r' || c == '\n') {
            if (pos > 0) {
                buf[pos] = '\0';
                handle_line(buf);
                pos = 0;
            }
        } else if (c == 0x08 || c == 0x7F) { // backspace
            if (pos > 0) pos--;
        } else {
            if (pos < (int)sizeof(buf) - 1) {
                buf[pos++] = (char)c;
            }
        }
    }
}

void app_main(void)
{
    // 1) Init motor PWM + pins
    motor_hw_init();

    // 2) Init UART console input
    console_init();

    // 3) Start console task
    xTaskCreate(console_task, "console_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Ready. Use serial monitor to input duty.");
}

