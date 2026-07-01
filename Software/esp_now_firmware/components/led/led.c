#include "led.h"
#include "led_strip.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "led";
static int pin = LED_PIN;
static int count = LED_COUNT;

#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000) // 10 MHz resolution

// Internal singleton LED driver instance
static led_strip_handle_t s_led_strip = NULL;
static int s_led_count = 0;

esp_err_t led_init(void)
{
    if (s_led_strip != NULL) {
        ESP_LOGW(TAG, "LED strip already initialized");
        return ESP_OK;
    }

    led_strip_config_t strip_config = {
        .strip_gpio_num = pin,
        .max_leds = count,
        .flags = {.invert_out = false},
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LED_STRIP_RMT_RES_HZ,
        .mem_block_symbols = 0,
        .flags = {.with_dma = false},
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led_strip));
    s_led_count = count;

    ESP_LOGI(TAG, "LED strip initialized on GPIO %d (%d LEDs)", pin, count);
    return ESP_OK;
}

esp_err_t led_set_pixel(int index, uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_led_strip) return ESP_ERR_INVALID_STATE;
    if (index >= s_led_count) return ESP_ERR_INVALID_ARG;
    return led_strip_set_pixel(s_led_strip, index, r, g, b);
}

esp_err_t led_refresh(void)
{
    if (!s_led_strip) return ESP_ERR_INVALID_STATE;
    return led_strip_refresh(s_led_strip);
}

esp_err_t led_clear(void)
{
    if (!s_led_strip) return ESP_ERR_INVALID_STATE;
    return led_strip_clear(s_led_strip);
}

esp_err_t led_set_all(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_led_strip) return ESP_ERR_INVALID_STATE;
    for (int i = 0; i < s_led_count; i++) {
        led_strip_set_pixel(s_led_strip, i, r, g, b);
    }
    return led_strip_refresh(s_led_strip);
}

static void hsv_to_rgb(float h, uint8_t *r, uint8_t *g, uint8_t *b)
{
    float s = 1.0f, v = 1.0f;
    int i = (int)(h / 60.0f) % 6;
    float f = (h / 60.0f) - i;
    float p = v * (1.0f - s);
    float q = v * (1.0f - f * s);
    float t = v * (1.0f - (1.0f - f) * s);
    float R, G, B;
    switch (i) {
        case 0: R = v; G = t; B = p; break;
        case 1: R = q; G = v; B = p; break;
        case 2: R = p; G = v; B = t; break;
        case 3: R = p; G = q; B = v; break;
        case 4: R = t; G = p; B = v; break;
        default: R = v; G = p; B = q; break;
    }
    *r = (uint8_t)(R * 255);
    *g = (uint8_t)(G * 255);
    *b = (uint8_t)(B * 255);
}

void led_rainbow(int delay_ms)
{
    if (!s_led_strip) return;

    float hue = 0.0f;
    while (1) {
        for (int i = 0; i < s_led_count; i++) {
            uint8_t r, g, b;
            float local_hue = fmodf(hue + (360.0f / s_led_count) * i, 360.0f);
            hsv_to_rgb(local_hue, &r, &g, &b);
            led_strip_set_pixel(s_led_strip, i, r, g, b);
        }
        led_strip_refresh(s_led_strip);
        hue += 3.0f;
        if (hue >= 360.0f) hue -= 360.0f;
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

