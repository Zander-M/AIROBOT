/*
    Read battery level, protect battery from discharging too much
    Both serve as an internal protection and publish battery level as percentage
    as a ROS topic
*/

// ADC1_7, GPIO 35
// Schematic:
// Battery+ --- 47K --- 10K --- GND
//                   |
//                 BATLVL

// ADC formula
// Vout = Dout * Vmax / Dmax

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "battery.h"

static const char *TAG = "battery";

int adc_raw;
static BatteryState battery_state;
volatile bool low_battery = false;

static adc_cali_handle_t adc_cali_handle = NULL;
static float ema_voltage = 0.0f;
static bool ema_initialized = false;

void battery_init(void){
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_CHAN0, ADC_ATTEN));

    // IDF v5 calibration API — falls back to raw formula if eFuse data is absent
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id   = ADC_UNIT_1,
        .atten     = ADC_ATTEN,
        .bitwidth  = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t ret = adc_cali_create_scheme_line_fitting(&cali_config, &adc_cali_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC calibration ready");
    } else {
        ESP_LOGW(TAG, "ADC calibration unavailable (ret=%d), using raw formula", ret);
        adc_cali_handle = NULL;
    }
}

void battery_read_task(void* args) {
    while (1) {
        adc_raw = adc1_get_raw(ADC_CHAN0);

        // Voltage at ADC pin, scaled to battery voltage via divider (57k/10k)
        float adc_pin_v;
        if (adc_cali_handle) {
            int mv = 0;
            adc_cali_raw_to_voltage(adc_cali_handle, adc_raw, &mv);
            adc_pin_v = (float)mv / 1000.0f;
        } else {
            adc_pin_v = (float)adc_raw * 1.1f / 4095.0f;
        }
        battery_state.voltage = adc_pin_v * (57.0f / 10.0f);

        // EMA filter to reject single-sample ADC noise
        if (!ema_initialized) {
            ema_voltage = battery_state.voltage;
            ema_initialized = true;
        } else {
            ema_voltage = VOLTAGE_EMA_ALPHA * battery_state.voltage
                        + (1.0f - VOLTAGE_EMA_ALPHA) * ema_voltage;
        }
        battery_state.filtered_voltage = filter_voltage(ema_voltage);
        battery_state.percentage = voltage_percentage(battery_state.filtered_voltage);

        // Hysteresis: latch low until voltage recovers above RECOVER threshold
        if (!battery_state.battery_low) {
            battery_state.battery_low = battery_state.filtered_voltage <= LOW_BATTERY_CUTOFF_V;
        } else {
            battery_state.battery_low = battery_state.filtered_voltage <= LOW_BATTERY_RECOVER_V;
        }

        low_battery = battery_state.battery_low;
        vTaskDelay(pdMS_TO_TICKS(1000)); //1Hz
    }
}

BatteryState battery_get_state(void){
    return battery_state;
}
