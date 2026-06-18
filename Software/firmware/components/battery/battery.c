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
#include "battery.h"

int adc_raw;
static BatteryState battery_state;
volatile bool low_battery = false; // for notifying low battery level

void battery_init(void){
    // ADC config
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_CHAN0, ADC_ATTEN));
}

void battery_read_task(void* args) {
    while (1) {
        adc_raw = adc1_get_raw(ADC_CHAN0);
        battery_state.voltage = ADC_TO_BATTERY_VOLTAGE(adc_raw);
        battery_state.filtered_voltage = filter_voltage(battery_state.voltage);
        battery_state.percentage = voltage_percentage(battery_state.filtered_voltage);
        battery_state.battery_low = battery_state.filtered_voltage <= LOW_BATTERY_CUTOFF_V;

        low_battery = battery_state.battery_low;
        vTaskDelay(pdMS_TO_TICKS(1000)); //1Hz
    }
}

BatteryState battery_get_state(void){
    return battery_state;
}
