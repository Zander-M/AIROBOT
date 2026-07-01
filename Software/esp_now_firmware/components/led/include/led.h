#pragma once
#include "robot_params.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the LED strip driver (singleton instance).
 * 
 * @param pin GPIO pin connected to the LED strip
 * @param count Number of LEDs
 * @return esp_err_t
 */
esp_err_t led_init(void);

/**
 * @brief Set a specific LED color (RGB 0–255).
 */
esp_err_t led_set_pixel(int index, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Refresh (push updates to LED strip).
 */
esp_err_t led_refresh(void);

/**
 * @brief Clear all LEDs.
 */
esp_err_t led_clear(void);

/**
 * @brief Simple rainbow animation (blocking).
 */
void led_rainbow(int delay_ms);

/**
 * @brief Convenience: set all LEDs to one color.
 */
esp_err_t led_set_all(uint8_t r, uint8_t g, uint8_t b);

#ifdef __cplusplus
}
#endif

