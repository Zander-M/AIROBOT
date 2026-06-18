// Battery level reading
#pragma once

#include <stdbool.h>
#include <math.h>
#include "driver/adc.h"

// ADC1_7, GPIO 35
// Schematic:
// Battery+ --- 47K --- 10K --- GND
//                   |
//                 BATLVL
// Fully charged reading: 4.2V * 10 / 57 ~= 0.737
// Low battery reading: 3.2V * 10 / 57 ~= 0.561

// ADC formula
// Vout = Dout * Vmax / Dmax

// ADC Config
#define ADC_CHAN0    ADC1_CHANNEL_7
#define ADC_ATTEN    ADC_ATTEN_DB_0

// Voltage Reading 
#define V_MIN 3.4f
#define V_MAX 4.2f
#define LOW_BATTERY_CUTOFF_V 3.4f
#define V_ATTEN_MAX 1.100f
#define ADC_TO_BATTERY_VOLTAGE(adc) ((float)adc* V_ATTEN_MAX / 4095.0f / 10.0f * 57.0f)

typedef struct {
    float voltage;
    float filtered_voltage;
    float percentage;
    bool battery_low;
} BatteryState;

extern volatile bool low_battery;

void battery_init(void);
void battery_read_task(void* args); // Callback

BatteryState battery_get_state(void);

// Helper function

static inline float filter_voltage(float voltage) {
    return fmaxf(fminf(V_MAX, voltage), V_MIN);
}

static inline float voltage_percentage(float voltage) {
    return (voltage - V_MIN) / (V_MAX - V_MIN);
}
